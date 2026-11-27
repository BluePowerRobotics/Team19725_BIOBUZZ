package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.utility.DpadStepper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

import java.util.ArrayList;
import java.util.List;

/**
 * 整车手动联调测试：底盘 + 定位 + intake + 炮台（<b>仅手动瞄准</b>）。
 *
 * <p>键位与操作逻辑与 {@code TeleOpMainBase} 的手动瞄准分支完全一致，
 * 只是去掉了自动瞄准（无 B 模式切换、无预载开关、无自动发射与刹车联动）。
 *
 * <p><b>主控（gamepad1）</b>
 * <ul>
 *   <li>左摇杆 → 底盘平移；右摇杆 X → 底盘旋转</li>
 *   <li>LT / RT → 按住时车头指向该扳机对应的球门（红方 LT = AudienceDown、RT = AudienceUp）</li>
 *   <li>X → 切换无头模式；A → 重新做 2 秒视觉均值校准并重置定位（不回退 ResetPose）</li>
 *   <li>LB → 吃球；RB → 吐出；B → 清零球道计数</li>
 * </ul>
 *
 * <p><b>副控（gamepad2）</b>
 * <ul>
 *   <li>LT / RT（按住） → 手动选择球门：LT = AudienceDown、RT = AudienceUp</li>
 *   <li>X（按下） → 循环手动平转模式 RELATIVE → ABSOLUTE → GOAL</li>
 *   <li>左摇杆 Y → 小球仰角速率；右摇杆 Y → 大球仰角速率（回中即停在当前角度）</li>
 *   <li>左摇杆 X → 平转角速度（回中停在当前角度）</li>
 *   <li>十字键 上/下 → 大球转速步进；左/右 → 小球转速步进</li>
 *   <li>A（按住） → 同时释放大球与小球扳机（启动后需先松开一次 A 才允许发射）</li>
 * </ul>
 *
 * <p>送球联动：扳机释放期间吃球器强制 {@code setGiveBall()}，忽略一操的吃球/吐出；
 * 发射停止后恢复（RB 吐出 &gt; LB 吃球 &gt; 均未按则停转）。
 *
 * <p>定位：启动后按 A 做视觉均值校准（与 {@code ChassisTesterRed} 相同：采集
 * {@link #CALIB_DURATION_MS} 毫秒内所有 HIVE 解算成功的 MT1 位姿，位置算术平均 /
 * 朝向圆周平均）作为初始位姿，随后进入手动操作；运行期间任何时候松开 A 都会重新做一次
 * 同样的 2 秒视觉均值校准，<b>不会</b>退回 {@code HypParams.ResetPoseRed}。此后每帧
 * {@code RobotPosition.update()} 推进。遥测同时给出融合位姿与纯里程计位姿，便于区分
 * 「定位漂移」来自融合链路还是 Pinpoint 本身。
 */
@Config
@TeleOp(name = "FullTest", group = "Tests")
public class FullTest extends LinearOpMode {

    // ==================== 超参数（Dashboard 可调） ====================

    /** 手动瞄准：每次方向键步进的飞轮转速增量 (tick/s) */
    public static int speedStep = 100;
    /** 手动瞄准：长按方向键的连续步进周期（毫秒） */
    public static int repeatPeriodMs = 120;
    /** 手动瞄准：平转角速度上限（度/秒），摇杆满偏对应此速率 */
    public static double maxYawRateDegPerSec = 120.0;
    /** 手动瞄准：仰角舵机速率（行程/秒），摇杆满偏对应此速率（0→1 约 1.25 s） */
    public static double pitchRatePerSec = 0.8;
    /** 手动瞄准：仰角摇杆死区，|axis| 小于该值视为不动 */
    public static double pitchAxisDeadband = 0.08;
    /** 开始后视觉校准的采样时长 (毫秒) */
    public static int CALIB_DURATION_MS = 2000;

    // ==================== 部件 ====================

    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private ActionRunner actionRunner;

    // ==================== 手动瞄准运行时状态 ====================

    private int targetSpeedBig = 0;
    private int targetSpeedSmall = 0;
    private final DpadStepper vertical = new DpadStepper();
    private final DpadStepper horizontal = new DpadStepper();

    private double targetPitchBig = 0.5;
    private double targetPitchSmall = 0.5;
    /** 上一次手动仰角积分时刻 (s)；0 = 尚未建立，首帧用标称周期 */
    private double lastPitchTime = 0;

    /** 扳机"已武装"：启动后必须先松开 A 再按下，才允许发射 */
    private boolean fireArmed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TeamColor teamColor = TeamColor.RED;

        // Chassis 构造时即完成 RobotPosition 初始化 (Limelight + 自适应 EKF)，
        // 先以 ResetPoseRed 作为 EKF 初值，待视觉校准取得均值后再 ResetPoseTo 覆盖。
        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, HypParams.ResetPoseRed);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        turret.stop();

        telemetry.setMsTransmissionInterval(20);
        telemetry.addLine("=== FullTest (manual only) ===");
        telemetry.addData("Fallback ResetPose", formatPose(HypParams.ResetPoseRed));
        telemetry.addLine("A (after start) = run vision calibration");
        telemetry.addLine("--- P1: 底盘 / 吃球 ---");
        telemetry.addLine("Left Stick: drive | Right Stick X: rotate");
        telemetry.addLine("LT / RT (hold): chassis aim DOWN / UP");
        telemetry.addLine("X: toggle no-head | A: vision recalibrate");
        telemetry.addLine("LB: eat | RB: output | B: reset ball count");
        telemetry.addLine("--- P2: 炮台（手动） ---");
        telemetry.addLine("LT / RT (hold): goal DOWN / UP");
        telemetry.addLine("X: cycle yaw mode REL / ABS / GOAL");
        telemetry.addLine("LstickY: small pitch rate | RstickY: big pitch rate | LstickX: yaw rate");
        telemetry.addLine("Dpad U/D: big speed | L/R: small speed");
        telemetry.addLine("A (hold): release BOTH triggers (re-press after start)");
        telemetry.update();
        telemetry.setMsTransmissionInterval(250);

        waitForStart();

        // ==== 1. 视觉均值确定初始位姿（与 ChassisTesterRed 相同） ====
        Pose2d calibratedPose = calibrateViaVision();
        RobotPosition.getInstance().ResetPoseTo(calibratedPose);

        // 校准结束时若 A 仍被按住，需等其松开，避免主循环的 aWasReleased() 误触发复位
        while (opModeIsActive() && gamepad1.a) {
            idle();
        }

        // ==== 2. 手动操作 ====
        while (opModeIsActive()) {
            // ======== 球门选择（放在 update() 之前，与比赛程序一致） ========
            if (gamepad2.left_trigger > 0.5) {
                RobotPosition.getInstance().setManualGoalDown(true);
            } else if (gamepad2.right_trigger > 0.5) {
                RobotPosition.getInstance().setManualGoalDown(false);
            }

            RobotPosition.getInstance().update();

            // ======== P1：底盘 ========
            if (gamepad1.left_trigger > 0.5) {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, HypParams.RedAudienceDown);
            } else if (gamepad1.right_trigger > 0.5) {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, HypParams.RedAudienceUp);
            } else {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            if (gamepad1.xWasReleased()) {
                chassis.exchangeUseNoHeadMode();
            }

            // 任何时候松开 A：重新做一次 2 秒视觉均值校准并覆盖当前定位
            // （不再退回 HypParams.ResetPoseRed；无有效视觉帧时保持原位姿不变）
            if (gamepad1.aWasReleased()) {
                Pose2d recalibrated = sampleVisionMean();
                if (recalibrated != null) {
                    RobotPosition.getInstance().ResetPoseTo(recalibrated);
                }
            }

            // ======== P2：炮台（手动） ========
            // A 松开过一次才武装：启动时不会因为按住 A 就直接开扳机
            if (!gamepad2.a) {
                fireArmed = true;
            }
            boolean fireKey = gamepad2.a && fireArmed;

            if (gamepad2.xWasReleased()) {
                turret.cycleManualYawMode();
            }

            Turret.FireReady ready = updateManualTurret(fireKey);

            // ======== 送球联动：扳机释放 → 强制供球，其余时间恢复一操控制 ========
            boolean firing = fireKey;
            if (firing) {
                sweeper.setGiveBall();
            } else if (gamepad1.right_bumper) {
                sweeper.setOutput();
            } else if (gamepad1.left_bumper) {
                sweeper.setEat();
            } else {
                sweeper.setStop();
            }

            // ======== 更新 & 遥测 ========
            // ActionRunner.update() 自带一次 Dashboard 场地图发送，无 Action 时不必每帧调用
            if (actionRunner.isBusy()) {
                actionRunner.update();
            }
            sweeper.update();

            // 手动清空放在满球停吸判定之后，保持「计数 → 停吸 → 手动清空」的帧内顺序
            if (gamepad1.bWasPressed()) {
                RobotPosition.getInstance().resetBallCount();
            }

            telemetry.addData("Team", "RED");
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());
            telemetry.addData("Fire ready big", ready.big);
            telemetry.addData("Fire ready small", ready.small);
            telemetry.addData("Ball count", "%d / %d",
                    RobotPosition.getInstance().getBallCount(), Sweeper.BallMax);

            telemetry.addData("Pose", "%.2f, %.2f, %.1f deg",
                    RobotPosition.getInstance().getX(), RobotPosition.getInstance().getY(),
                    Math.toDegrees(RobotPosition.getInstance().getTheta()));
            telemetry.addData("Vx / Vy (in/s)", "%.2f / %.2f",
                    RobotPosition.getInstance().getVx(), RobotPosition.getInstance().getVy());
            telemetry.addData("Omega (deg/s)", "%.2f", Math.toDegrees(RobotPosition.getInstance().getOmega()));
            // 纯里程计位姿：用于区分「Pinpoint 自身漂移」与「融合链路漂移」
            telemetry.addData("Pinpoint pose",
                    formatPose(RobotPosition.getInstance().getFusionLocalizer().getOdom().getPose()));

            telemetry.addData("Aim goal side", RobotPosition.getInstance().isGoalDown()
                    ? "AUDIENCE_DOWN" : "AUDIENCE_UP");

            chassis.telemetry();
            sweeper.setTelemetry();
            turret.setTelemetry();

            telemetry.addLine("--- manual targets ---");
            telemetry.addData("Manual yaw mode", turret.getManualYawMode());
            telemetry.addData("Target speed big (tick/s)", targetSpeedBig);
            telemetry.addData("Target speed small (tick/s)", targetSpeedSmall);
            telemetry.addData("Target pitch big / small", "%.3f / %.3f", targetPitchBig, targetPitchSmall);
            telemetry.addData("Pitch rate (travel/s)", "%.2f", pitchRatePerSec);
            telemetry.addData("speedStep", speedStep);
            telemetry.addData("Fire armed (A re-press)", fireArmed);

            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // stop 只登记请求，零速与刹车由 update() 下发（帧外补一次零速）
        chassis.stop();
        chassis.update(0, 0, 0);
        sweeper.setStop();
        sweeper.update();
        turret.stop();
    }

    // ==================== 视觉校准 ====================

    /**
     * 启动阶段的视觉均值校准：等待按 A 后采集 {@link #CALIB_DURATION_MS} 毫秒内的 MT1
     * 均值位姿（位置算术平均，朝向圆周平均）。
     *
     * <p>采样窗口内没有任何有效帧时提示重按 A，不会退回 {@code HypParams.ResetPoseRed}。
     * 校准期间底盘保持静止，但仍每帧调用 {@link RobotPosition#update()}，
     * 由融合定位器内部推进 Pinpoint 并拉取最新 MT1 结果。
     */
    private Pose2d calibrateViaVision() {
        MT1Localizer mt1 = RobotPosition.getInstance().getFusionLocalizer().getMT1();
        List<Pose2d> samples = new ArrayList<>();
        boolean prevA = false;

        while (opModeIsActive()) {
            chassis.stop();
            chassis.update(0, 0, 0);
            RobotPosition.getInstance().update();

            boolean usable = mt1.isValid() && mt1.isHiveEstimated();
            addCalibrationTelemetry(mt1, usable ? mt1.getPose() : null, samples, -1);

            boolean a = gamepad1.a;
            if (a && !prevA) {
                Pose2d mean = sampleVisionMean();
                if (mean != null) {
                    telemetry.addData("Calibrated start pose", formatPose(mean));
                    telemetry.update();
                    return mean;
                }
                telemetry.addLine("No valid samples, press A to retry");
            }
            prevA = a;
            telemetry.update();
        }
        // OpMode 被停止时才会执行到这里
        return HypParams.ResetPoseRed;
    }

    /**
     * 立即采集 {@link #CALIB_DURATION_MS} 毫秒的视觉均值位姿（不等待按键）。
     *
     * @return 位置算术平均 / 朝向圆周平均的位姿；采样窗口内无任何 HIVE 有效帧时返回
     *         {@code null}（调用方应保持原位姿）
     */
    private Pose2d sampleVisionMean() {
        MT1Localizer mt1 = RobotPosition.getInstance().getFusionLocalizer().getMT1();
        List<Pose2d> samples = new ArrayList<>();
        long startMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            // 采样期间底盘必须静止：持续下发零速并推进里程计
            chassis.stop();
            chassis.update(0, 0, 0);
            RobotPosition.getInstance().update();

            long elapsed = System.currentTimeMillis() - startMs;
            boolean usable = mt1.isValid() && mt1.isHiveEstimated();
            if (usable && elapsed < CALIB_DURATION_MS) {
                samples.add(mt1.getPose());
            }
            addCalibrationTelemetry(mt1, usable ? mt1.getPose() : null, samples, elapsed);

            if (elapsed >= CALIB_DURATION_MS) {
                break;
            }
            telemetry.update();
        }
        return samples.isEmpty() ? null : computeMeanPose(samples);
    }

    /**
     * 校准界面遥测（启动等待期与采样窗口共用）。
     *
     * @param usablePose 本帧可用的 MT1 位姿，不可用传 {@code null}
     * @param elapsedMs  采样已进行的毫秒数；{@code < 0} 表示尚未开始采样
     */
    private void addCalibrationTelemetry(MT1Localizer mt1, Pose2d usablePose,
                                         List<Pose2d> samples, long elapsedMs) {
        telemetry.addLine("=== Vision Calibration (MT1) ===");
        if (usablePose != null) {
            telemetry.addData("Live pose", formatPose(usablePose));
        } else {
            telemetry.addLine("No valid pose (need valid + hiveEstimated)");
        }
        telemetry.addData("Tag Count", mt1.getTagCount());
        telemetry.addData("Tag Ids", mt1.getTagIds());
        telemetry.addData("Ambiguity (m)", "%.4f", mt1.getAmbiguity());
        telemetry.addData("Hive State", mt1.getHiveState());
        telemetry.addData("Hive Angle (deg)", "%.2f", Math.toDegrees(mt1.getHiveAngle()));
        telemetry.addData("Pitch Check Err (deg)", "%.2f", mt1.getHivePitchCheckErrDeg());
        if (elapsedMs < 0) {
            telemetry.addLine("Press A to start 2 s sampling");
        } else {
            telemetry.addData("Recording", "Active");
            telemetry.addData("Samples", samples.size());
            telemetry.addData("Time left (ms)", Math.max(0, CALIB_DURATION_MS - elapsedMs));
        }
    }

    /** 位置算术平均，朝向圆周平均。 */
    private static Pose2d computeMeanPose(List<Pose2d> samples) {
        double sumX = 0, sumY = 0;
        double sumSin = 0, sumCos = 0;
        for (Pose2d p : samples) {
            sumX += p.position.x;
            sumY += p.position.y;
            sumSin += Math.sin(p.heading.toDouble());
            sumCos += Math.cos(p.heading.toDouble());
        }
        return new Pose2d(sumX / samples.size(), sumY / samples.size(), Math.atan2(sumSin, sumCos));
    }

    /**
     * 手动瞄准：与 {@code TeleOpMainBase#updateManualTurret} 完全一致的实现，
     * 直接给定 yaw / 仰角 / 转速 / 扳机。
     *
     * @param fireKey 扳机键（同时释放大球与小球）
     * @return 本帧两个发射器是否具备发射条件
     */
    private Turret.FireReady updateManualTurret(boolean fireKey) {
        // ---- 仰角：摇杆速率积分、松手保持 ----
        double now = getRuntime();
        double dt = (lastPitchTime == 0) ? 0.02 : (now - lastPitchTime);
        lastPitchTime = now;
        if (dt <= 0 || dt > 0.1) {
            // 首帧或长时间停顿：用标称周期，避免一次积分跳变
            dt = 0.02;
        }
        targetPitchSmall = stepPitch(targetPitchSmall, -gamepad2.left_stick_y, dt,
                Shooter.SMALL_PARAMS.pitchMin, Shooter.SMALL_PARAMS.pitchMax);
        targetPitchBig = stepPitch(targetPitchBig, -gamepad2.right_stick_y, dt,
                Shooter.BIG_PARAMS.pitchMin, Shooter.BIG_PARAMS.pitchMax);
        // 推右（axis = +1）→ 顺时针，对应负角速度；摇杆回中即停在当前角度
        double yawRate = -gamepad2.left_stick_x * Math.toRadians(maxYawRateDegPerSec);

        // ---- 十字键步进飞轮目标转速 ----
        int v = vertical.step(gamepad2.dpad_down, gamepad2.dpad_up, repeatPeriodMs);
        int h = horizontal.step(gamepad2.dpad_left, gamepad2.dpad_right, repeatPeriodMs);
        targetSpeedBig = clamp(targetSpeedBig + v * speedStep, 0, Shooter.BIG_PARAMS.flywheel.speedMax);
        targetSpeedSmall = clamp(targetSpeedSmall + h * speedStep, 0, Shooter.SMALL_PARAMS.flywheel.speedMax);

        return turret.updateManual(yawRate, targetPitchBig, targetPitchSmall,
                targetSpeedBig, targetSpeedSmall, fireKey, fireKey);
    }

    /** 仰角的单步积分：摇杆偏量 × pitchRatePerSec × dt，并钳到机械行程。 */
    private static double stepPitch(double current, double axis, double dt, double lo, double hi) {
        double input = (Math.abs(axis) < pitchAxisDeadband) ? 0.0 : axis;
        double next = current + input * pitchRatePerSec * dt;
        return Math.max(lo, Math.min(hi, next));
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static String formatPose(Pose2d pose) {
        return String.format("(%.2f, %.2f, %.1f°)",
                pose.position.x, pose.position.y,
                Math.toDegrees(pose.heading.toDouble()));
    }
}
