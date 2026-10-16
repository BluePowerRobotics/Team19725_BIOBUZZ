package org.firstinspires.ftc.teamcode.Auto.AutomaticCalibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.filter.EMA;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 底盘 PIDSVA（前馈 S/V/A + 反馈 P/D + 运动约束）自动标定 OpMode。
 *
 * <h3>标定对象与基准</h3>
 * 标定对象是<b>底盘真实控制链路</b>（{@code MecanumDrive} 的
 * {@code HolonomicController} + {@code MotorFeedforward}），不是
 * {@code utility/PID/PIDSVAController}（后者为功率量纲，仅供炮台 / 飞轮 / 吸球使用）。
 * 底盘链路模型为：
 * <pre>
 *   u_volts = kS·sign(v) + kV·v + kA·a         power = u_volts / V_batt
 *   v_cmd   = posGain·(位姿误差) + velGain·(速度误差) + v_desired
 * </pre>
 * 与 Road Runner 官方模板完全同构，量纲自洽（kV × 满速 tick ≈ 8~9 V，落在 12 V 电瓶区间），
 * 经核对<b>专业、正确、合适</b>，故本程序以之为基准、保持量级一致，<b>不修改其数学模型</b>。
 *
 * <h3>不写回参数</h3>
 * <b>本程序绝不写入 {@code MecanumDrive.PARAMS} 的任何静态字段。</b>
 * 所有识别结果只做三件事：
 * <ol>
 *   <li>在 FTC Dashboard 上实时显示原始曲线、拟合残差与<b>建议值</b>；</li>
 *   <li>把原始采样与静态字段建议值写入 {@code /sdcard/AutomaticCalibration/<时间戳>/} 的 CSV；</li>
 *   <li>把拟合曲线 / 残差 / 阶跃响应导出为 SVG 图像。
 *       操作手确认后再手动把建议值填回参数文件。</li>
 * </ol>
 *
 * <h3>阶段状态机</h3>
 * <pre>
 *   IDLE(按 START) → SVA_RAMP → KA → FB_AXIAL → FB_LATERAL → FB_HEADING → ACCEL_SLIP → ANG → DONE
 * </pre>
 * 手柄：{@code A}=开始（Init 阶段），{@code B}=跳过当前阶段，{@code Y}=立即急停终止。
 * 前置条件：机器人四周留出足够空间（加速 / 阶跃阶段会移动约 10~40 英寸）。
 *
 * <p><b>注意</b>：本程序通过 {@link RobotPosition#RobotPositioninit} 初始化，依赖
 * {@code limelight} 与 Pinpoint 融合定位器提供地面真值速度，用于打滑判据。
 */
@Autonomous(name = "Cal: Chassis SVA/PID", group = "Calibration")
public class AutomaticChassisCalibration extends LinearOpMode {

    /** 标定参数（FTC Dashboard 可调、可保存）。 */
    @Config
    public static class Cal {
        /** 底盘参数存储文件中的单位换算：每 tick 英寸数（只读，用于约束折算） */
        // 直接读取 MecanumDrive.PARAMS.inPerTick，此处不重复定义

        /** SVA 三角波功率幅值（0~1） */
        public static double SVA_RAMP_MAX_POWER = 0.5;
        /** SVA 三角波单程时长（s），总时长翻倍 */
        public static double SVA_RAMP_TIME = 5.0;

        /** kA 正弦激励幅值（0~1），零均值以抑制漂移 */
        public static double KA_AMPLITUDE = 0.5;
        /** kA 正弦频率（Hz） */
        public static double KA_FREQ_HZ = 0.5;
        /** kA 激励时长（s） */
        public static double KA_TIME = 6.0;

        /** 反馈阶跃的平移目标增量（in） */
        public static double FB_STEP_DIST_IN = 10.0;
        /** 反馈阶跃的航向目标增量（deg） */
        public static double FB_STEP_HEADING_DEG = 30.0;
        /** 反馈阶跃记录时长（s） */
        public static double FB_TIME = 2.5;

        /** 约束扫描：首个阶跃功率（0~1），后续按步长递增并正负交替 */
        public static double ACCEL_START_POWER = 0.3;
        /** 约束扫描：每级功率增量 */
        public static double ACCEL_STEP_POWER = 0.2;
        /** 约束扫描：级数 */
        public static int ACCEL_STEPS = 4;
        /** 约束扫描：每级持续通电时长（s） */
        public static double ACCEL_STEP_TIME = 0.35;
        /** 约束扫描：每级之间的断电保持时长（s） */
        public static double ACCEL_SETTLE_TIME = 0.8;

        /** 自转扫描功率幅值（0~1） */
        public static double ANG_MAX_POWER = 0.6;
        /** 自转扫描单程时长（s），总时长翻倍 */
        public static double ANG_TIME = 4.0;

        /** 目标阻尼比（≤0 表示保持实测阻尼） */
        public static double ZETA_TARGET = 0.9;
        /** 目标带宽相对实测的倍率（1.0 表示保持实测带宽；>1 提速） */
        public static double OMEGA_TARGET_SCALE = 1.0;
        /** 单次增益修正比例上下限，避免噪声导致参数暴涨 */
        public static double SCALE_MIN = 0.5;
        public static double SCALE_MAX = 3.0;

        /** 运动约束安全系数（0, 1] */
        public static double SAFETY = 0.85;
        /** 打滑判据相对容差 */
        public static double SLIP_TOL = 0.25;

        /** SVA 拟合的轮速死区阈值（tick/s），剔除静止点 */
        public static double VEL_EPS_TICKS = 20.0;
        /** 轮速 / 加速度 EMA 平滑系数 */
        public static double EMA_ALPHA = 0.35;

        /** 标定开始前的前进预热时长（s），用于稳定定位器与轮速估计 */
        public static double WARMUP_TIME = 0.4;
        /** 预热功率 */
        public static double WARMUP_POWER = 0.25;
    }

    /** 阶段状态机 */
    private enum Phase {
        SVA_RAMP, KA, FB_AXIAL, FB_LATERAL, FB_HEADING, ACCEL_SLIP, ANG, DONE
    }

    // ---- 硬件与定位 ----
    private MecanumDrive drive;
    private DcMotorEx lf, lb, rb, rf;
    private Localizer localizer;
    private FtcDashboard dashboard;

    // ---- 计时与估计 ----
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final EMA velEma = new EMA(Cal.EMA_ALPHA);
    private final EMA accEma = new EMA(Cal.EMA_ALPHA);
    private final EMA omegaEma = new EMA(Cal.EMA_ALPHA);
    private final EMA alphaEma = new EMA(Cal.EMA_ALPHA);
    private boolean estInit;
    private double lastPosTicks, lastTimeMs, lastOmega;
    private double wheelVelTicks, wheelAccelTicks, omega, omegaAccel;

    // ---- 手柄边沿 ----
    private boolean prevY, prevB, skipRequested, abortRequested;

    // ---- 采样与结果 ----
    private final List<CalibrationSample> allSamples = new ArrayList<>();
    private final List<CalibrationSample> svaSamples = new ArrayList<>();
    private final List<CalibrationSample> kaSamples = new ArrayList<>();
    private final List<Double> fbT = new ArrayList<>();
    private final List<Double> fbY = new ArrayList<>();

    private SvaIdentifier.Result svaResult;
    private FeedbackIdentifier.ClosedLoopMetrics fbAxial, fbLateral, fbHeading;
    private FeedbackIdentifier.GainRecommendation recAxial, recLateral, recHeading;
    private double peakAccelNoSlipTicks, peakAngVel, peakAngAccel;
    private boolean slipDetectedAccel, slipDetectedAng;

    private File sessionDir;

    @Override
    public final void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // ================= Init =================
        initHardware();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("==== 底盘 SVA / PID 自动标定 ====");
            telemetry.addLine("标定对象：RoadRunner 底盘链路（前馈 kS/kV/kA + 反馈 pos/vel 增益）");
            telemetry.addLine("结果不写回 PARAMS，仅 Dashboard 显示 + CSV/SVG 导出");
            telemetry.addLine();
            telemetry.addLine("ASTART 后请勿触碰机器人；四周留出 ≥ 4 英尺空间");
            telemetry.addLine("B = 跳过当前阶段    Y = 急停终止");
            telemetry.addLine();
            telemetry.addLine("当前 PARAMS：kS=" + fmt(MecanumDrive.PARAMS.kS)
                    + " kV=" + fmt(MecanumDrive.PARAMS.kV)
                    + " kA=" + fmt(MecanumDrive.PARAMS.kA));
            telemetry.addLine("当前反馈：axial=" + fmt(MecanumDrive.PARAMS.axialGain)
                    + "/" + fmt(MecanumDrive.PARAMS.axialVelGain)
                    + "  lateral=" + fmt(MecanumDrive.PARAMS.lateralGain)
                    + "/" + fmt(MecanumDrive.PARAMS.lateralVelGain));
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        RobotPosition.getInstance().update();
        warmUp();
        sessionDir = CalibrationExporter.newSessionDir();

        // ================= 依次执行各阶段 =================
        runSvaRamp();
        if (!abortRequested) runKa();
        if (!abortRequested) runFeedbackStep(Phase.FB_AXIAL);
        if (!abortRequested) runFeedbackStep(Phase.FB_LATERAL);
        if (!abortRequested) runFeedbackStep(Phase.FB_HEADING);
        if (!abortRequested) runAccelSlip();
        if (!abortRequested) runAngular();

        stopMotors();
        phaseTimer.reset();

        // ================= 导出与显示建议值 =================
        exportAndShowSuggestions();

        // 保持显示，等待操作手抄录 / 导出
        while (opModeIsActive()) {
            pollGamepad();
            if (abortRequested) {
                break;
            }
            RobotPosition.getInstance().update();
            showSuggestionTelemetry(true);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("cal_done", 1);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            sleep(50);
        }
        stopMotors();
    }

    // ==================================================================================
    // 初始化与通用工具
    // ==================================================================================

    private void initHardware() {
        RobotPosition.RobotPositioninit(hardwareMap, new Pose2d(0, 0, 0), TeamColor.RED);
        drive = RobotPosition.getInstance().getDrive();
        localizer = drive.localizer;
        lf = drive.leftFront;
        lb = drive.leftBack;
        rb = drive.rightBack;
        rf = drive.rightFront;
    }

    /** 前进预热：小幅正向前进一小段，让定位器与轮速估计进入稳态。 */
    private void warmUp() {
        phaseTimer.reset();
        resetEstimators();
        while (opModeIsActive() && phaseTimer.seconds() < Cal.WARMUP_TIME) {
            RobotPosition.getInstance().update();
            setForwardPower(Cal.WARMUP_POWER);
            updateEstimators();
        }
        setForwardPower(0);
    }

    /**
     * 单帧推进：推进定位器、读手柄、处理跳过 / 急停。
     *
     * @return {@code false} 表示本阶段应立即结束（跳过或急停）
     */
    private boolean tick() {
        RobotPosition.getInstance().update();
        pollGamepad();
        if (abortRequested || skipRequested) {
            stopMotors();
            return false;
        }
        return true;
    }

    private void pollGamepad() {
        boolean y = gamepad1.y;
        boolean b = gamepad1.b;
        if (y && !prevY) {
            abortRequested = true;
        }
        if (b && !prevB) {
            skipRequested = true;
        }
        prevY = y;
        prevB = b;
    }

    private void beginPhase() {
        skipRequested = false;
        phaseTimer.reset();
        resetEstimators();
    }

    /** 每个阶段开始时重置估计器状态。 */
    private void resetEstimators() {
        estInit = false;
        velEma.reset();
        accEma.reset();
        omegaEma.reset();
        alphaEma.reset();
        wheelVelTicks = 0;
        wheelAccelTicks = 0;
        omega = 0;
        omegaAccel = 0;
        lastOmega = 0;
    }

    /** 由编码器差分 + EMA 更新轮速 / 轮加速度 / 角速度 / 角加速度估计。 */
    private void updateEstimators() {
        long nowMs = (long) phaseTimer.milliseconds();
        double pos = wheelForwardTicks();
        double om = drive.getLastVelocity().angVel;

        if (!estInit) {
            estInit = true;
            lastPosTicks = pos;
            lastTimeMs = nowMs;
            lastOmega = om;
            return;
        }
        double dt = (nowMs - lastTimeMs) / 1000.0;
        if (dt <= 0) {
            return;
        }
        double rawVel = (pos - lastPosTicks) / dt;
        double vel = velEma.update(rawVel);
        wheelAccelTicks = accEma.update((vel - wheelVelTicks) / dt);
        wheelVelTicks = vel;

        double rawOmega = (om - lastOmega) / dt;
        double omF = omegaEma.update(om);
        omegaAccel = alphaEma.update((omF - omega) / dt);
        omega = omF;

        lastPosTicks = pos;
        lastTimeMs = nowMs;
        lastOmega = om;
    }

    /**
     * 轮子「前进方向」位置（tick），对编码器正负号约定做运行时自适配。
     *
     * <p>底盘右侧电机方向为 REVERSE，因此「前进」时右轮编码器读数的正负取决于 SDK 是否
     * 对 REVERSE 电机取反，且各版本 / 接线可能不同。这里同时计算两种候选：
     * <pre>
     *   all = lf + lb + rb + rf        （各轮读数直接相加）
     *   alt = lf + lb − rb − rf        （右侧取反后相加）
     * </pre>
     * 直线前进时，正确约定对应的候选值约为 {@code 4n} 而错误约定约为 {@code 0}，
     * 故取<b>绝对值较大</b>者为前进方向速度，除以 4 得到单轮值。该做法对两种约定都成立。
     */
    private double wheelForwardTicks() {
        double a = lf.getCurrentPosition() + lb.getCurrentPosition()
                + rb.getCurrentPosition() + rf.getCurrentPosition();
        double b = lf.getCurrentPosition() + lb.getCurrentPosition()
                - rb.getCurrentPosition() - rf.getCurrentPosition();
        return (Math.abs(a) >= Math.abs(b)) ? a / 4.0 : b / 4.0;
    }

    private double battVolts() {
        return drive.voltageSensor.getVoltage();
    }

    /** 四轮同功率前进（右侧电机方向为 REVERSE，正功率即为前进）。 */
    private void setForwardPower(double p) {
        double c = clamp(p, -1, 1);
        lf.setPower(c);
        lb.setPower(c);
        rb.setPower(c);
        rf.setPower(c);
    }

    /** 原地自转：左侧正、右侧反。 */
    private void setSpinPower(double p) {
        double c = clamp(p, -1, 1);
        lf.setPower(c);
        lb.setPower(c);
        rb.setPower(-c);
        rf.setPower(-c);
    }

    private void stopMotors() {
        lf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        rf.setPower(0);
    }

    /** 记录一个采样点（同时写入总表与阶段表）。 */
    private void record(String phase, double t, double power, List<CalibrationSample> phaseSink) {
        RobotPosition rp = RobotPosition.getInstance();
        double batt = battVolts();
        CalibrationSample s = new CalibrationSample(phase, t, batt, power, power * batt,
                wheelVelTicks, wheelAccelTicks, rp.getVx(), rp.getVy(), rp.getOmega());
        allSamples.add(s);
        if (phaseSink != null) {
            phaseSink.add(s);
        }
    }

    /** 当前是否打滑：轮速折算的机器人速度与地面真值速度相对偏差超容差。 */
    private boolean slipping() {
        double wheelImplied = wheelVelTicks * MecanumDrive.PARAMS.inPerTick;
        double ground = Math.hypot(lastVx, lastVy);
        return ConstraintIdentifier.isSlipping(wheelImplied, ground, Cal.SLIP_TOL);
    }

    private double lastVx, lastVy;

    private void captureGroundVelocity() {
        RobotPosition rp = RobotPosition.getInstance();
        lastVx = rp.getVx();
        lastVy = rp.getVy();
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private static String fmt(double x) {
        return String.format(Locale.US, "%.6g", x);
    }

    // ==================================================================================
    // 阶段 1：SVA 三角波功率扫描 → kS / kV（并由同一段数据提供 kA 初值）
    // ==================================================================================

    private void runSvaRamp() {
        beginPhase();
        svaSamples.clear();
        double total = 2 * Cal.SVA_RAMP_TIME;

        while (opModeIsActive()) {
            if (!tick()) break;
            double t = phaseTimer.seconds();
            if (t >= total) break;

            captureGroundVelocity();
            double frac = t / Cal.SVA_RAMP_TIME;                 // 0..2
            double power = (frac <= 1 ? frac : (2 - frac)) * Cal.SVA_RAMP_MAX_POWER;
            setForwardPower(power);
            updateEstimators();
            record("SVA_RAMP", t, power, svaSamples);

            double modelVolts = MecanumDrive.PARAMS.kS * Math.signum(wheelVelTicks)
                    + MecanumDrive.PARAMS.kV * wheelVelTicks;
            sendPacket("SVA_RAMP", power * battVolts(), modelVolts,
                    power * battVolts() - modelVolts, 0);
        }
        stopMotors();
        if (skipRequested || abortRequested) {
            return;
        }

        // 两参数最小二乘拟合 kS / kV
        List<Double> s = new ArrayList<>(), v = new ArrayList<>(), volts = new ArrayList<>();
        for (CalibrationSample c : svaSamples) {
            if (Math.abs(c.wheelVelTicks) > Cal.VEL_EPS_TICKS) {
                s.add((double) Math.signum(c.wheelVelTicks));
                v.add(c.wheelVelTicks);
                volts.add(c.voltsEff);
            }
        }
        try {
            SvaIdentifier.Result r = SvaIdentifier.fitKsKv(toArray(s), toArray(v), toArray(volts));
            svaResult = new SvaIdentifier.Result(r.kS, r.kV, svaResult != null ? svaResult.kA : 0,
                    r.rmsResidual, r.nSamples);
            RobotLog.ii("Cal", "SVA_RAMP: " + svaResult);
        } catch (RuntimeException e) {
            RobotLog.ee("Cal", "SVA_RAMP 拟合失败: " + e.getMessage());
        }
    }

    // ==================================================================================
    // 阶段 2：kA 零均值正弦激励 → 固定 kS/kV 后回归 kA
    // ==================================================================================

    private void runKa() {
        beginPhase();
        kaSamples.clear();

        while (opModeIsActive()) {
            if (!tick()) break;
            double t = phaseTimer.seconds();
            if (t >= Cal.KA_TIME) break;

            captureGroundVelocity();
            double power = Cal.KA_AMPLITUDE * Math.sin(2 * Math.PI * Cal.KA_FREQ_HZ * t);
            setForwardPower(power);
            updateEstimators();
            record("KA", t, power, kaSamples);

            double kS = (svaResult != null) ? svaResult.kS : MecanumDrive.PARAMS.kS;
            double kV = (svaResult != null) ? svaResult.kV : MecanumDrive.PARAMS.kV;
            double modelVolts = kS * Math.signum(wheelVelTicks) + kV * wheelVelTicks;
            double measured = power * battVolts();
            sendPacket("KA", measured, modelVolts, measured - modelVolts, 0);
        }
        stopMotors();
        if (skipRequested || abortRequested || svaResult == null) {
            return;
        }

        List<Double> s = new ArrayList<>(), v = new ArrayList<>(),
                a = new ArrayList<>(), volts = new ArrayList<>();
        for (CalibrationSample c : kaSamples) {
            if (Math.abs(c.wheelVelTicks) > Cal.VEL_EPS_TICKS) {
                s.add((double) Math.signum(c.wheelVelTicks));
                v.add(c.wheelVelTicks);
                a.add(c.wheelAccelTicks);
                volts.add(c.voltsEff);
            }
        }
        try {
            SvaIdentifier.Result r = SvaIdentifier.fitKaDetailed(
                    toArray(s), toArray(v), toArray(a), toArray(volts), svaResult.kS, svaResult.kV);
            svaResult = r;
            RobotLog.ii("Cal", "KA: " + svaResult);
        } catch (RuntimeException e) {
            RobotLog.ee("Cal", "KA 拟合失败: " + e.getMessage());
        }
    }

    // ==================================================================================
    // 阶段 3：闭环阶跃响应 → 反馈 P/D 建议增益
    // ==================================================================================

    private void runFeedbackStep(Phase which) {
        beginPhase();
        fbT.clear();
        fbY.clear();

        Pose2d start = localizer.getPose();
        double startHeading = start.heading.toDouble();
        Pose2d target;
        switch (which) {
            case FB_LATERAL:
                target = new Pose2d(start.position.x, start.position.y + Cal.FB_STEP_DIST_IN, startHeading);
                break;
            case FB_HEADING:
                target = new Pose2d(start.position.x, start.position.y,
                        startHeading + Math.toRadians(Cal.FB_STEP_HEADING_DEG));
                break;
            case FB_AXIAL:
            default:
                target = new Pose2d(start.position.x + Cal.FB_STEP_DIST_IN, start.position.y, startHeading);
                break;
        }
        Pose2dDual<Time> txTarget = Pose2dDual.constant(target, 1);

        HolonomicController controller = new HolonomicController(
                MecanumDrive.PARAMS.axialGain, MecanumDrive.PARAMS.lateralGain,
                MecanumDrive.PARAMS.headingGain, MecanumDrive.PARAMS.axialVelGain,
                MecanumDrive.PARAMS.lateralVelGain, MecanumDrive.PARAMS.headingVelGain);
        MotorFeedforward ff = new MotorFeedforward(MecanumDrive.PARAMS.kS,
                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);

        double targetScalar = 0;
        while (opModeIsActive()) {
            if (!tick()) break;
            double t = phaseTimer.seconds();
            if (t >= Cal.FB_TIME) break;

            PoseVelocity2d vel = drive.getLastVelocity();
            Pose2d cur = localizer.getPose();
            PoseVelocity2dDual<Time> command = controller.compute(txTarget, cur, vel);
            MecanumKinematics.WheelVelocities<Time> wv = drive.kinematics.inverse(command);
            double volts = battVolts();
            lf.setPower(ff.compute(wv.leftFront) / volts);
            lb.setPower(ff.compute(wv.leftBack) / volts);
            rb.setPower(ff.compute(wv.rightBack) / volts);
            rf.setPower(ff.compute(wv.rightFront) / volts);

            double y;
            if (which == Phase.FB_HEADING) {
                y = Math.toDegrees(cur.heading.minus(start.heading));
                targetScalar = Cal.FB_STEP_HEADING_DEG;
            } else if (which == Phase.FB_LATERAL) {
                y = cur.position.y - start.position.y;
                targetScalar = Cal.FB_STEP_DIST_IN;
            } else {
                y = cur.position.x - start.position.x;
                targetScalar = Cal.FB_STEP_DIST_IN;
            }

            captureGroundVelocity();
            updateEstimators();
            double avgPower = (lf.getPower() + lb.getPower() + rb.getPower() + rf.getPower()) / 4.0;
            record(which.name(), t, avgPower, null);

            fbT.add(t);
            fbY.add(y);
            sendPacket(which.name(), y, targetScalar, targetScalar - y, 0);
        }
        stopMotors();
        if (skipRequested || abortRequested) {
            return;
        }

        double[] tt = toArray(fbT);
        double[] yy = toArray(fbY);
        FeedbackIdentifier.StepMetrics step = FeedbackIdentifier.measureStepResponse(tt, yy);
        FeedbackIdentifier.ClosedLoopMetrics metrics = FeedbackIdentifier.closedLoopFromStep(step);
        FeedbackIdentifier.GainRecommendation rec = FeedbackIdentifier.recommendGains(
                currentPosGain(which), currentVelGain(which), metrics,
                metrics.omegaN * Cal.OMEGA_TARGET_SCALE, Cal.ZETA_TARGET,
                Cal.SCALE_MIN, Cal.SCALE_MAX);

        // 每段阶跃响应独立导出 SVG（含响应曲线、目标线、峰值标记）
        CalibrationExporter.writeStepChart(sessionDir, "fit_feedback_" + which.name() + ".svg",
                tt, yy, targetScalar, step.peakTime,
                which.name() + "  step  target=" + fmt(targetScalar)
                        + "  M_p=" + fmt(step.overshoot * 100) + "%");

        switch (which) {
            case FB_LATERAL:
                fbLateral = metrics;
                recLateral = rec;
                break;
            case FB_HEADING:
                fbHeading = metrics;
                recHeading = rec;
                break;
            case FB_AXIAL:
            default:
                fbAxial = metrics;
                recAxial = rec;
                break;
        }
        RobotLog.ii("Cal", which + ": " + metrics + " -> " + rec);
    }

    private double currentPosGain(Phase which) {
        switch (which) {
            case FB_LATERAL:
                return MecanumDrive.PARAMS.lateralGain;
            case FB_HEADING:
                return MecanumDrive.PARAMS.headingGain;
            default:
                return MecanumDrive.PARAMS.axialGain;
        }
    }

    private double currentVelGain(Phase which) {
        switch (which) {
            case FB_LATERAL:
                return MecanumDrive.PARAMS.lateralVelGain;
            case FB_HEADING:
                return MecanumDrive.PARAMS.headingVelGain;
            default:
                return MecanumDrive.PARAMS.axialVelGain;
        }
    }

    // ==================================================================================
    // 阶段 4：递增正负交替功率阶跃 → 打滑前最大加速度
    // ==================================================================================

    private void runAccelSlip() {
        beginPhase();
        peakAccelNoSlipTicks = 0;
        slipDetectedAccel = false;
        double cycle = Cal.ACCEL_STEP_TIME + Cal.ACCEL_SETTLE_TIME;

        while (opModeIsActive()) {
            if (!tick()) break;
            double t = phaseTimer.seconds();
            int idx = (int) (t / cycle);
            if (idx >= Cal.ACCEL_STEPS) break;
            double tc = t - idx * cycle;

            double mag = Cal.ACCEL_START_POWER + idx * Cal.ACCEL_STEP_POWER;
            double sign = (idx % 2 == 0) ? 1.0 : -1.0;
            double power = (tc < Cal.ACCEL_STEP_TIME) ? sign * mag : 0.0;
            setForwardPower(power);
            updateEstimators();
            captureGroundVelocity();

            if (tc < Cal.ACCEL_STEP_TIME) {
                if (slipping()) {
                    slipDetectedAccel = true;
                } else {
                    peakAccelNoSlipTicks = Math.max(peakAccelNoSlipTicks, Math.abs(wheelAccelTicks));
                }
            }
            record("ACCEL_SLIP", t, power, null);
            sendPacket("ACCEL_SLIP", power * battVolts(),
                    wheelVelTicks * MecanumDrive.PARAMS.inPerTick,
                    slipRatioNow(), 0);
        }
        stopMotors();
    }

    private double slipRatioNow() {
        double wheelImplied = wheelVelTicks * MecanumDrive.PARAMS.inPerTick;
        double ground = Math.hypot(lastVx, lastVy);
        return ConstraintIdentifier.slipRatio(wheelImplied, ground);
    }

    // ==================================================================================
    // 阶段 5：自转三角波功率扫描 → 最大角速度 / 角加速度
    // ==================================================================================

    private void runAngular() {
        beginPhase();
        peakAngVel = 0;
        peakAngAccel = 0;
        double total = 2 * Cal.ANG_TIME;

        while (opModeIsActive()) {
            if (!tick()) break;
            double t = phaseTimer.seconds();
            if (t >= total) break;

            double frac = t / Cal.ANG_TIME;
            double power = (frac <= 1 ? frac : (2 - frac)) * Cal.ANG_MAX_POWER;
            setSpinPower(power);
            updateEstimators();
            captureGroundVelocity();

            peakAngVel = Math.max(peakAngVel, Math.abs(omega));
            peakAngAccel = Math.max(peakAngAccel, Math.abs(omegaAccel));
            record("ANG", t, power, null);
            sendPacket("ANG", Math.toDegrees(omega), Math.toDegrees(power * Cal.ANG_MAX_POWER),
                    omegaAccel, 0);
        }
        stopMotors();
    }

    // ==================================================================================
    // 汇总：约束推算 + CSV / SVG 导出 + 建议值展示
    // ==================================================================================

    private double suggestedKS() {
        return svaResult != null ? svaResult.kS : Double.NaN;
    }

    private double suggestedKV() {
        return svaResult != null ? svaResult.kV : Double.NaN;
    }

    private double suggestedKA() {
        return svaResult != null ? svaResult.kA : Double.NaN;
    }

    private ConstraintIdentifier.Constraints buildConstraints() {
        double inPerTick = MecanumDrive.PARAMS.inPerTick;
        // 若 SVA 阶段被跳过，则回退用当前 PARAMS 值推算，避免 NaN 传播
        double kS = svaResult != null ? svaResult.kS : MecanumDrive.PARAMS.kS;
        double kV = svaResult != null ? svaResult.kV : MecanumDrive.PARAMS.kV;
        // 由前馈模型推算「满电压自由轮速」，避免为测极限而真跑到最高速（更安全）
        double freeSpeedTicks = ConstraintIdentifier.modelMaxWheelVelTicks(battVolts(), kS, kV);
        double maxWheelVel = ConstraintIdentifier.maxWheelVelFromPeak(freeSpeedTicks, inPerTick, Cal.SAFETY);
        if (maxWheelVel <= 0) {
            maxWheelVel = ConstraintIdentifier.maxWheelVelFromPeak(
                    MecanumDrive.PARAMS.maxWheelVel / inPerTick, inPerTick, 1.0);
        }
        double maxAccel = ConstraintIdentifier.maxProfileAccelFromPeak(
                peakAccelNoSlipTicks, inPerTick, Cal.SAFETY);
        if (maxAccel <= 0) {
            maxAccel = MecanumDrive.PARAMS.maxProfileAccel;
        }
        return new ConstraintIdentifier.Constraints(maxWheelVel, maxAccel,
                ConstraintIdentifier.minProfileAccelFromMax(maxAccel),
                peakAngVel > 0 ? peakAngVel * Cal.SAFETY : MecanumDrive.PARAMS.maxAngVel,
                peakAngAccel > 0 ? peakAngAccel * Cal.SAFETY : MecanumDrive.PARAMS.maxAngAccel,
                slipDetectedAccel);
    }

    private void exportAndShowSuggestions() {
        ConstraintIdentifier.Constraints cons = buildConstraints();
        List<CalibrationExporter.ResultEntry> entries = new ArrayList<>();
        double rms = svaResult != null ? svaResult.rmsResidual : Double.NaN;

        entries.add(new CalibrationExporter.ResultEntry("PARAMS.kS", "V",
                MecanumDrive.PARAMS.kS, suggestedKS(), rms));
        entries.add(new CalibrationExporter.ResultEntry("PARAMS.kV", "V/(tick/s)",
                MecanumDrive.PARAMS.kV, suggestedKV(), rms));
        entries.add(new CalibrationExporter.ResultEntry("PARAMS.kA", "V/(tick/s^2)",
                MecanumDrive.PARAMS.kA, suggestedKA(), rms));

        addFeedbackEntry(entries, "axial", MecanumDrive.PARAMS.axialGain,
                MecanumDrive.PARAMS.axialVelGain, recAxial, fbAxial);
        addFeedbackEntry(entries, "lateral", MecanumDrive.PARAMS.lateralGain,
                MecanumDrive.PARAMS.lateralVelGain, recLateral, fbLateral);
        addFeedbackEntry(entries, "heading", MecanumDrive.PARAMS.headingGain,
                MecanumDrive.PARAMS.headingVelGain, recHeading, fbHeading);

        entries.add(new CalibrationExporter.ResultEntry("PARAMS.maxWheelVel", "in/s",
                MecanumDrive.PARAMS.maxWheelVel, cons.maxWheelVel, 0));
        entries.add(new CalibrationExporter.ResultEntry("PARAMS.maxProfileAccel", "in/s^2",
                MecanumDrive.PARAMS.maxProfileAccel, cons.maxProfileAccel, 0));
        entries.add(new CalibrationExporter.ResultEntry("PARAMS.minProfileAccel", "in/s^2",
                MecanumDrive.PARAMS.minProfileAccel, cons.minProfileAccel, 0));
        entries.add(new CalibrationExporter.ResultEntry("PARAMS.maxAngVel", "rad/s",
                MecanumDrive.PARAMS.maxAngVel, cons.maxAngVel, 0));
        entries.add(new CalibrationExporter.ResultEntry("PARAMS.maxAngAccel", "rad/s^2",
                MecanumDrive.PARAMS.maxAngAccel, cons.maxAngAccel, 0));

        CalibrationExporter.writeSamples(sessionDir, allSamples);
        CalibrationExporter.writeResults(sessionDir, entries);

        // SVG：SVA 拟合直线 + kA 残差 + 三段阶跃响应
        double[] sv = new double[svaSamples.size()];
        double[] sV = new double[svaSamples.size()];
        for (int i = 0; i < svaSamples.size(); i++) {
            sv[i] = svaSamples.get(i).wheelVelTicks;
            sV[i] = svaSamples.get(i).voltsEff;
        }
        if (svaResult != null) {
            CalibrationExporter.writeSvaChart(sessionDir, sv, sV, svaResult.kS, svaResult.kV,
                    "SVA fit  kS=" + fmt(svaResult.kS) + "  kV=" + fmt(svaResult.kV));
        }

        double[] ka = new double[kaSamples.size()];
        double[] kRes = new double[kaSamples.size()];
        for (int i = 0; i < kaSamples.size(); i++) {
            CalibrationSample c = kaSamples.get(i);
            ka[i] = c.wheelAccelTicks;
            kRes[i] = c.voltsEff - ((svaResult != null ? svaResult.kS : 0) * Math.signum(c.wheelVelTicks)
                    + (svaResult != null ? svaResult.kV : 0) * c.wheelVelTicks);
        }
        if (svaResult != null) {
            CalibrationExporter.writeResidualChart(sessionDir, ka, kRes, svaResult.kA);
        }
        // 阶跃响应 SVG 已在各反馈阶段结束时分别导出（fit_feedback_<phase>.svg）
    }

    private void addFeedbackEntry(List<CalibrationExporter.ResultEntry> entries, String tag,
                                  double posGainCur, double velGainCur,
                                  FeedbackIdentifier.GainRecommendation rec,
                                  FeedbackIdentifier.ClosedLoopMetrics metrics) {
        double zeta = metrics != null && metrics.reliable ? metrics.zeta : Double.NaN;
        entries.add(new CalibrationExporter.ResultEntry("PARAMS." + tag + "Gain", "1/s",
                posGainCur, rec != null ? rec.posGain : Double.NaN, zeta));
        entries.add(new CalibrationExporter.ResultEntry("PARAMS." + tag + "VelGain", "-",
                velGainCur, rec != null ? rec.velGain : Double.NaN, zeta));
    }

    private void showSuggestionTelemetry(boolean finalView) {
        telemetry.addLine(finalView ? "==== 标定完成，建议值（需手动填回） ====" : "标定中");
        telemetry.addLine(String.format(Locale.US, "kS: %.6f -> %.6f V", MecanumDrive.PARAMS.kS, suggestedKS()));
        telemetry.addLine(String.format(Locale.US, "kV: %.9f -> %.9f V/(tick/s)", MecanumDrive.PARAMS.kV, suggestedKV()));
        telemetry.addLine(String.format(Locale.US, "kA: %.9f -> %.9f V/(tick/s^2)", MecanumDrive.PARAMS.kA, suggestedKA()));
        if (recAxial != null) {
            telemetry.addLine(String.format(Locale.US, "axial: %.3f/%.3f -> %.3f/%.3f (z=%.2f, w=%.2f)",
                    MecanumDrive.PARAMS.axialGain, MecanumDrive.PARAMS.axialVelGain,
                    recAxial.posGain, recAxial.velGain,
                    fbAxial != null ? fbAxial.zeta : 0, fbAxial != null ? fbAxial.omegaN : 0));
        }
        if (recLateral != null) {
            telemetry.addLine(String.format(Locale.US, "lateral: %.3f/%.3f -> %.3f/%.3f (z=%.2f, w=%.2f)",
                    MecanumDrive.PARAMS.lateralGain, MecanumDrive.PARAMS.lateralVelGain,
                    recLateral.posGain, recLateral.velGain,
                    fbLateral != null ? fbLateral.zeta : 0, fbLateral != null ? fbLateral.omegaN : 0));
        }
        if (recHeading != null) {
            telemetry.addLine(String.format(Locale.US, "heading: %.3f/%.3f -> %.3f/%.3f (z=%.2f, w=%.2f)",
                    MecanumDrive.PARAMS.headingGain, MecanumDrive.PARAMS.headingVelGain,
                    recHeading.posGain, recHeading.velGain,
                    fbHeading != null ? fbHeading.zeta : 0, fbHeading != null ? fbHeading.omegaN : 0));
        }
        if (svaResult != null) {
            telemetry.addData("SVA rms residual (V)", svaResult.rmsResidual);
        }
        telemetry.addData("CSV/SVG", sessionDir != null ? sessionDir.getAbsolutePath() : "未创建");
        if (slipDetectedAccel) {
            telemetry.addLine("⚠ 加速度扫描检出打滑，约束已按打滑前峰值 × 安全系数取值");
        }
    }

    // ==================================================================================
    // 单包遥测 / Dashboard 曲线（每帧仅发 1 个 packet，避免覆盖场地图）
    // ==================================================================================

    private void sendPacket(String phase, double measured, double model, double residual, double extra) {
        // 每帧只发一个新的 packet，避免复用 packet 造成键值/绘图残留
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("measured", measured);
        packet.put("model", model);
        packet.put("residual", residual);
        packet.put("extra", extra);
        packet.put("wheelVelTicks", wheelVelTicks);
        packet.put("wheelAccelTicks", wheelAccelTicks);
        packet.put("battVolts", battVolts());
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Phase", phase);
        telemetry.addData("wheelVelTicks", wheelVelTicks);
        telemetry.addData("residual", residual);
        telemetry.update();
    }

    // ==================================================================================
    // 数组工具
    // ==================================================================================

    private static double[] toArray(List<Double> list) {
        double[] out = new double[list.size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = list.get(i);
        }
        return out;
    }
}
