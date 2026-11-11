package org.firstinspires.ftc.teamcode.OpModes.CompetitionOpmode;

import org.firstinspires.ftc.teamcode.utility.DpadStepper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.RobotStateStore;

/**
 * 比赛主程序（TeleOp）的公共骨架：底盘 + 吃球 + 炮台（自动瞄准 / 手动瞄准）。
 *
 * <p>颜色相关的差异（默认停车位姿、车头瞄准的球门）由子类通过
 * {@link #defaultStopPose()} 与 {@link #aimGoal(boolean)} 提供。
 *
 * <p><b>主控（gamepad1）</b>
 * <ul>
 *   <li>左摇杆 → 底盘平移；右摇杆 X → 底盘旋转</li>
 *   <li>LT / RT → 按住时车头指向由该扳机选择的球门（颜色映射与副控球门选择一致，
 *       见 {@link #chassisAimGoal(boolean)}），松开恢复常规驾驶；<b>不改写炮台球门</b></li>
 *   <li>X → 切换无头模式；A → 重置定位到本方 ResetPose</li>
 *   <li>LB → 吃球；RB → 吐出</li>
 * </ul>
 *
 * <p><b>副控（gamepad2）</b>
 * <ul>
 *   <li>B（按下） → 切换 <b>自动瞄准 / 手动瞄准</b>，默认自动瞄准</li>
 *   <li>Y（按下） → 切换预载开关（仅自动瞄准生效）</li>
 *   <li>LT / RT（按住） → 手动选择球门：红方 LT = AudienceDown、RT = AudienceUp，
 *       蓝方相反；MT1 非 MIDDLE 读数优先，无有效读数时保持本选择</li>
 *   <li>自动瞄准：A（按住） → 允许发射，同时持续刹车并停住底盘；RB → 强制释放两个扳机</li>
 *   <li>X（按下） → 循环手动平转模式 RELATIVE → ABSOLUTE → GOAL（仅手动瞄准生效）</li>
 *   <li>手动瞄准：左摇杆 Y → 小球仰角<b>速率</b>，右摇杆 Y → 大球仰角<b>速率</b>（回中即停在当前角度）；
 *       左摇杆 X → 平转角速度（回中停在当前角度）；十字键 上/下 → 大球转速步进，
 *       左/右 → 小球转速步进；A → 同时释放大球与小球扳机</li>
 * </ul>
 *
 * <p><b>扳机武装</b>：切模式（B）后必须先松开 A 再按下才允许发射 —— 手动路径的扳机不带就绪门控，
 * 若切换瞬间 A 仍按住，会在飞轮未转起来、俯仰未到位时立刻出扳机（空打）。启动时同理。
 *
 * <p><b>送球联动</b>：任一发射器释放扳机时，吃球器自动 {@code setGiveArtifact()} 送球；
 * 停止发射后停转（主控 LB/RB 手动吃球/吐出仍然优先除外）。
 */
@Config
public abstract class TeleOpMainBase extends LinearOpMode {

    // ==================== 超参数（Dashboard 可调） ====================

    /** 手动瞄准：每次方向键步进的飞轮转速增量 (tick/s) */
    public static int speedStep = 100;
    /** 手动瞄准：长按方向键的连续步进周期（毫秒） */
    public static int repeatPeriodMs = 120;
    /** 手动瞄准：平转角速度上限（度/秒），摇杆满偏对应此速率 */
    public static double maxYawRateDegPerSec = 120.0;
    /** 手动瞄准：仰角舵机速率（行程/秒），摇杆满偏对应此速率（0→1 约 1.25 s） */
    public static double pitchRatePerSec = 0.8;
    /** 手动瞄准：仰角摇杆死区，|axis| 小于该值视为不动，避免松手后残余偏量导致缓慢漂移 */
    public static double pitchAxisDeadband = 0.08;

    // ==================== 部件 ====================

    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private ActionRunner actionRunner;

    private final TeamColor teamColor;

    // ==================== 手动瞄准运行时状态 ====================

    private int targetSpeedBig = 0;
    private int targetSpeedSmall = 0;
    private final DpadStepper vertical = new DpadStepper();
    private final DpadStepper horizontal = new DpadStepper();

    /**
     * 手动瞄准：当前仰角目标（舵机位置，行程 0~1）。
     *
     * <p>由摇杆速率积分推进、<b>松手保持</b>；旧实现把摇杆位置直接当舵机位置，
     * 回中即 0.5，导致无法停在任意角度、松手会跳回中点。
     */
    private double targetPitchBig = 0.5;
    private double targetPitchSmall = 0.5;
    /** 上一次手动仰角积分时刻 (s)；0 = 尚未建立，首帧用标称周期 */
    private double lastPitchTime = 0;

    /** true = 自动瞄准（炮台按定位解算），false = 手动瞄准（副控直接给定目标） */
    private boolean autoAim = true;
    /** 预载开关，仅自动瞄准生效；Y 切换 */
    private boolean preloadEnabled = true;
    /**
     * 扳机"已武装"：切模式后必须先松开 A 再按下，才允许发射。
     *
     * <p>手动路径的扳机不带就绪门控（{@code Turret.updateManual} 直接 {@code setTrigger}），
     * 从自动切到手动时若 A 仍按住，会在飞轮未转起来、俯仰未到位时立刻释放扳机（空打/卡球）。
     * 初始为 false，因此按住 A 启动也不会立刻发射。
     */
    private boolean fireArmed = false;

    protected TeleOpMainBase(TeamColor teamColor) {
        this.teamColor = teamColor;
    }

    /** 无持久化位姿时使用的默认停车位姿 */
    protected abstract Pose2d defaultStopPose();

    /**
     * GP1 LT / RT 对应的车头瞄准球门。
     *
     * <p>颜色映射与副控（GP2）的球门选择一致：红方 LT = AudienceDown、RT = AudienceUp，
     * 蓝方相反。各子类按自身颜色返回对应球门。
     *
     * @param leftTrigger true = 左扳机 (LT)，false = 右扳机 (RT)
     */
    protected abstract Pose2d aimGoal(boolean leftTrigger);

    /**
     * 车头瞄准目标：由 GP1 LT / RT 直接选择，规则与 GP2 的球门选择一致。
     *
     * <p>本方法<b>不修改</b> {@link RobotPosition#getGoalPose()}，也不调用
     * {@link RobotPosition#setManualGoalDown(boolean)}；它只影响按住扳机期间的车头朝向，
     * 便于操作手临时转动底盘，而炮台/共享球门保持原选择。
     *
     * @param leftTrigger true = 左扳机 (LT)，false = 右扳机 (RT)
     * @return 车头瞄准目标
     */
    protected Pose2d chassisAimGoal(boolean leftTrigger) {
        return aimGoal(leftTrigger);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- 读回自动阶段结束时刻持久化的机器状态作为初始位姿 ----
        Pose2d storedPose = RobotStateStore.loadPose();
        Pose2d initPose = (storedPose != null) ? storedPose : defaultStopPose();

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, initPose);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        turret.stop();

        telemetry.setMsTransmissionInterval(20);
        telemetry.addData("Team Color", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
        telemetry.addData("Init Pose Source", storedPose != null ? "Stored robot state" : "Default StopPose");
        telemetry.addData("Init Pose", "%.2f, %.2f, %.1f deg",
                initPose.position.x, initPose.position.y, Math.toDegrees(initPose.heading.toDouble()));
        telemetry.addData("--- P1: 底盘 / 吃球 ---", "");
        telemetry.addData("Left Stick", "Chassis Drive");
        telemetry.addData("Right Stick X", "Chassis Rotation (disabled while aiming)");
        telemetry.addData("Left / Right Trigger", "Aim goal selected by LT / RT (hold)");
        telemetry.addData("X", "Toggle No-Head Mode");
        telemetry.addData("A", "Reset Pose to " + (teamColor == TeamColor.BLUE ? "Blue" : "Red") + " ResetPose");
        telemetry.addData("Left Bumper", "Sweeper Eat");
        telemetry.addData("Right Bumper", "Sweeper Output");
        telemetry.addData("P1: B", "Reset ball count");
        telemetry.addData("--- P2: 炮台 ---", "");
        telemetry.addData("B", "Toggle Auto / Manual Aim (auto by default)");
        telemetry.addData("Y", "Toggle Preload (auto aim only)");
        telemetry.addData("P2: LT / RT", "Manual goal: Down / Up (RED), Up / Down (BLUE)");
        telemetry.addData("Auto: A", "Fire + Brake (hold)");
        telemetry.addData("Auto: RB", "Force release BOTH triggers");
        telemetry.addData("Manual: Sticks", "LstickY small pitch rate | RstickY big pitch rate | LstickX yaw rate");
        telemetry.addData("Manual: X", "Cycle yaw mode REL / ABS / GOAL");
        telemetry.addData("Manual: Dpad", "U/D big speed | L/R small speed");
        telemetry.addData("Manual: A", "Release BOTH triggers (hold; re-press after mode switch)");
        telemetry.update();
        telemetry.setMsTransmissionInterval(250);

        waitForStart();

        while (opModeIsActive()) {
            // ======== 球门选择：操作手手动指定（MT1 非 MIDDLE 读数优先级更高） ========
            // 放在 update() 之前：update() 会用本帧 MT1 的非 MIDDLE 观测覆盖手动选择；
            // 没有有效观测时保持上一帧读数或本次操作手输入。
            if (gamepad2.left_trigger > 0.5) {
                // 红方 LT → AudienceDown；蓝方 LT → AudienceUp
                RobotPosition.getInstance().setManualGoalDown(teamColor == TeamColor.RED);
            } else if (gamepad2.right_trigger > 0.5) {
                // 红方 RT → AudienceUp；蓝方 RT → AudienceDown
                RobotPosition.getInstance().setManualGoalDown(teamColor == TeamColor.BLUE);
            }

            // 本帧只决定“是否退回”；方向由下面的 Sweeper.setXxx() 写入 RobotPosition。
            boolean reject = gamepad1.right_bumper;

            // OpMode 不再传方向；RobotPosition.update() 保持无参数。
            RobotPosition.getInstance().update();

            // ======== P1：底盘 ========
            // 自动瞄准按住 A 时持续刹车：stop 只登记请求，硬件统一下发在 chassis.update() 内；
            // update 看到停止请求会下发零速并放下刹车。手动瞄准不受影响。
            if (autoAim && gamepad2.a) {
                chassis.stop(true);
            }
            if (gamepad1.left_trigger > 0.5) {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, chassisAimGoal(true));
            } else if (gamepad1.right_trigger > 0.5) {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, chassisAimGoal(false));
            } else {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            if (gamepad1.xWasReleased()) {
                chassis.exchangeUseNoHeadMode();
            }

            if (gamepad1.aWasReleased()) {
                Pose2d resetPose = (teamColor == TeamColor.BLUE) ?
                        HypParams.ResetPoseBlue : HypParams.ResetPoseRed;
                RobotPosition.getInstance().ResetPoseTo(resetPose);
            }

            // ======== P2：炮台 ========
            if (gamepad2.bWasReleased()) {
                autoAim = !autoAim;
                // 切模式必须解除扳机武装：手动路径没有就绪门控，切换瞬间若 A 仍按住会直接开扳机
                fireArmed = false;
                if (!autoAim) {
                    // 进入手动时把仰角目标对齐当前已下发值，首帧不产生舵机跳变
                    targetPitchBig = turret.shooterBig.getTargetPitch();
                    targetPitchSmall = turret.shooterSmall.getTargetPitch();
                    lastPitchTime = 0;
                }
            }
            // A 松开过一次才重新武装：启动时与每次切模式后都要求重按
            if (!gamepad2.a) {
                fireArmed = true;
            }
            boolean fireKey = gamepad2.a && fireArmed;

            if (gamepad2.yWasReleased()) {
                preloadEnabled = !preloadEnabled;
            }
            if (gamepad2.xWasReleased()) {
                turret.cycleManualYawMode();
            }

            // 扳机键：自动瞄准用 RB 强制释放；手动瞄准用 A（已武装）同时释放两个扳机
            Turret.FireReady ready;
            if (autoAim) {
                // 自动瞄准：A 按住即允许发射（同时刹车，见 P1 底盘段）；
                // RB 强制释放两个发射器（无视就绪与平转，也不受武装限制）
                boolean forceRelease = gamepad2.right_bumper;
                ready = turret.update(fireKey, preloadEnabled, forceRelease, forceRelease);
            } else {
                ready = updateManualTurret(fireKey);
            }

            // ======== 扳机释放 → 吃球器送球；停止发射后停转 ========
            // 自动瞄准以 Turret 实际释放状态为准：它含 fireHoldTime 的最短保持窗口，
            // 否则就绪状态在档位边界逐帧抖动会让送球跟着断续，打断装填。
            // 手动瞄准的扳机就是副控按键本身。
            boolean feeding = autoAim
                    ? (turret.isBigFiring() || turret.isSmallFiring())
                    : fireKey;
            // 最终命令优先级：退回 > 发射送球 > 手动吸球 > 停止。
            // 电机方向和 BallCounter 的计数方向由同一个状态来源决定。
            if (reject) {
                sweeper.setOutput();
            } else if (feeding) {
                sweeper.setGiveArtifact();
            } else if (gamepad1.left_bumper) {
                sweeper.setEat();
            } else {
                sweeper.setStop();
            }

            // ======== 更新 & 遥测 ========
            actionRunner.update();
            // 球道计数：光电门采样已在 RobotPosition.update() 内完成，
            // 满球停吸由 Sweeper 依据 getBallCount() 判定
            sweeper.update();

            // 手动清空放在满球停吸判定之后，保持「计数 → 停吸 → 手动清空」的帧内顺序
            if (gamepad1.bWasPressed()) {
                RobotPosition.getInstance().resetBallCount();
            }

            telemetry.addData("Team", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());
            telemetry.addData("Aim mode", autoAim ? "AUTO" : "MANUAL");
            if (autoAim) {
                telemetry.addData("Preload", preloadEnabled ? "ON" : "OFF");
            }
            // 车头与炮台共用的球门（单一来源，便于操作手确认"打的是哪一个门"）
            Pose2d sharedGoal = RobotPosition.getInstance().getGoalPose();
            telemetry.addData("Aim goal side", RobotPosition.getInstance().isGoalDown()
                    ? "AUDIENCE_DOWN" : "AUDIENCE_UP");
            telemetry.addData("Aim goal (shared)", "(%+.1f, %+.1f)",
                    sharedGoal.position.x, sharedGoal.position.y);
            telemetry.addData("Fire ready big", ready.big);
            telemetry.addData("Fire ready small", ready.small);
            telemetry.addData("Ball count", "%d / %d",
                    RobotPosition.getInstance().getBallCount(), Sweeper.BallMax);
            telemetry.addData("Ball lane mode",
                    reject ? "REJECT" : (feeding ? "FEED" : "INTAKE/STOP"));

            telemetry.addData("Pose X", "%.2f in", RobotPosition.getInstance().getX());
            telemetry.addData("Pose Y", "%.2f in", RobotPosition.getInstance().getY());
            telemetry.addData("Pose Theta", "%.2f deg", Math.toDegrees(RobotPosition.getInstance().getTheta()));

            chassis.telemetry();
            sweeper.setTelemetry();
            turret.setTelemetry();

            if (!autoAim) {
                telemetry.addLine("--- manual targets ---");
                telemetry.addData("Manual yaw mode", turret.getManualYawMode());
                telemetry.addData("Target speed big (tick/s)", targetSpeedBig);
                telemetry.addData("Target speed small (tick/s)", targetSpeedSmall);
                telemetry.addData("Target pitch big / small", "%.3f / %.3f", targetPitchBig, targetPitchSmall);
                telemetry.addData("Pitch rate (travel/s)", "%.2f", pitchRatePerSec);
                telemetry.addData("speedStep", speedStep);
            }
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

    /**
     * 手动瞄准：与 {@code TurretManualTest} 相同的控制方式，直接给定 yaw / 仰角 / 转速 / 扳机。
     *
     * @param fireKey 扳机键（同时释放大球与小球）
     * @return 本帧两个发射器是否具备发射条件
     */
    private Turret.FireReady updateManualTurret(boolean fireKey) {
        // ---- 仰角：摇杆速率积分、松手保持（与平转/转速同一手感）----
        // 旧实现把摇杆位置直接映射成舵机位置：回中 = 0.5，既停不住也调不细，
        // 且松手瞬间会从中点弹回。改为"推着走"后，摇杆回中即停在当前角度。
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
        double pitchSmall = targetPitchSmall;
        double pitchBig = targetPitchBig;
        // 推右（axis = +1）→ 顺时针，对应负角速度；摇杆回中即停在当前角度
        double yawRate = -gamepad2.left_stick_x * Math.toRadians(maxYawRateDegPerSec);

        // ---- 十字键步进飞轮目标转速 ----
        int v = vertical.step(gamepad2.dpad_down, gamepad2.dpad_up, repeatPeriodMs);
        int h = horizontal.step(gamepad2.dpad_left, gamepad2.dpad_right, repeatPeriodMs);
        targetSpeedBig = clamp(targetSpeedBig + v * speedStep, 0, Shooter.BIG_PARAMS.flywheel.speedMax);
        targetSpeedSmall = clamp(targetSpeedSmall + h * speedStep, 0, Shooter.SMALL_PARAMS.flywheel.speedMax);

        // ---- 扳机：按住释放（A 同时释放大球与小球，需已武装）----
        return turret.updateManual(yawRate, pitchBig, pitchSmall,
                targetSpeedBig, targetSpeedSmall, fireKey, fireKey);
    }

    /**
     * 仰角的单步积分：摇杆偏量 × {@link #pitchRatePerSec} × dt，并钳到机械行程。
     *
     * @param current 当前舵机目标位置
     * @param axis    摇杆量（上推为正；小于 {@link #pitchAxisDeadband} 视为不动）
     * @param dt      帧间隔 (s)
     * @param lo      机械行程下限
     * @param hi      机械行程上限
     * @return 新目标位置
     */
    private static double stepPitch(double current, double axis, double dt, double lo, double hi) {
        double input = (Math.abs(axis) < pitchAxisDeadband) ? 0.0 : axis;
        double next = current + input * pitchRatePerSec * dt;
        return Math.max(lo, Math.min(hi, next));
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** 方向键步进器见 {@link DpadStepper} */
}
