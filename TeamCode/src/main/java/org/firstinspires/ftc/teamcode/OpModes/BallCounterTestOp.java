package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.Processors.Sensors.BallCounter;

/**
 * BallCounter 球道计数联调程序（整车测试：需要 Limelight / Pinpoint / 光电门 / 炮台全部已配置）。
 *
 * <p><b>对象层级</b>（与架构图一致，每帧自上而下更新，禁止越级）：
 * <pre>
 * OpMode
 *   ├─ RobotPosition（总状态估计器）
 *   │    └─ BallCounter
 *   │         └─ BreakBeam（光电门）
 *   ├─ Sweeper（Intake 电机控制器）
 *   └─ Turret（炮台）
 * </pre>
 *
 * <p><b>实现的功能</b>：
 * <ul>
 *   <li>吸球 / 出球时自动计数（由 {@link BallCounter} 的上升沿检测完成）；</li>
 *   <li>计数达到 {@link Sweeper#BallCapacity}（默认 4）时自动停止吸球，送球 / 出球不受影响；</li>
 *   <li>炮台发射完毕（二操松开 A 键，扳机由释放回到关闭）后自动清空计数；</li>
 *   <li>一操 Y 键允许操作手手动清空计数。</li>
 * </ul>
 *
 * <p><b>手柄映射</b>：
 * <ul>
 *   <li>一操 左 bumper（按住）：吸球；右 bumper（按住）：出球</li>
 *   <li>一操 Y：手动清空球道计数</li>
 *   <li>二操 按住 A：发射（固定 yaw / 俯仰，飞轮目标转速见 Dashboard）；松开即视为发射完毕</li>
 * </ul>
 *
 * <p><b>验证步骤</b>：
 * <ul>
 *   <li>逐个吸球：Ball Count 每个球只 +1，达到 4 后 Intake 自动停转（Sweeper Target = 0）；</li>
 *   <li>按右 bumper 出球：每挡一个球 Ball Count -1，减到 0 后不再变负；</li>
 *   <li>按住二操 A 再松开：Ball Count 归零（Turret 检测扳机下降沿后自动清空）；</li>
 *   <li>按一操 Y：Ball Count 手动归零。</li>
 * </ul>
 *
 * <p><b>注意</b>：
 * <ul>
 *   <li>本程序不得再直接调用 {@code ballCounter.update()} 或 {@code beam.update()}，
 *       否则会与 {@link RobotPosition#update()} 内部的逐级更新重复推进滤波器。</li>
 *   <li>光电门的原始电压 / 阈值 / alpha 标定请用 BreakBeamTestOp
 *       （BallCounter 现在归 RobotPosition 管理，本程序不再单独持有光电门）。</li>
 * </ul>
 *
 * <p>调参：FTC Dashboard → Config → BallCounterTestOp / Sweeper / Turret。
 */
@Config
@TeleOp(name = "BallCounterTestOp", group = "Tests")
public class BallCounterTestOp extends LinearOpMode {

    // ================= FTC Dashboard 可调参数 =================

    /** 联调时炮台目标平转角（弧度） */
    public static double testYaw = 0.0;

    /** 联调时大球飞轮目标转速 (tick/s) */
    public static int testSpeedBig = Shooter.BIG_PARAMS.flywheel.speedMax;

    /** 联调时小球飞轮目标转速 (tick/s) */
    public static int testSpeedSmall = Shooter.SMALL_PARAMS.flywheel.speedMax;

    private Sweeper sweeper;
    private Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // RobotPosition 初始化时自下而上创建 BallCounter 与其下属 BreakBeam（设备名 "breakBeam"）
        RobotPosition.RobotPositioninit(hardwareMap, HypParams.StopPoseRed);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        turret.stop();

        telemetry.addLine("GP1 LB: Eat | RB: Output | Y: Clear ball count");
        telemetry.addLine("GP2 hold A: Fire (release = fire complete -> auto clear count)");
        telemetry.addLine("Ball lane auto-stops intake at BallCapacity = " + Sweeper.BallCapacity);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            RobotPosition robotPosition = RobotPosition.getInstance();

            // ---- 控制：先下达吸球 / 出球指令，球道计数方向据此决定 ----
            if (gamepad1.left_bumper) {
                sweeper.setEat();
            }
            if (gamepad1.right_bumper) {
                sweeper.setOutput();
            }

            // ---- 信号处理：球道方向与 Sweeper 指令同源（送球 / 吐球 -1，吸球 / 停止 +1）----
            int ballDirection = sweeper.isOutputDirection()
                    ? BallCounter.MODE_OUTPUT : BallCounter.MODE_INTAKE;
            robotPosition.setBallCounterMode(ballDirection);

            // ---- 状态估计：RobotPosition 内部更新 BallCounter → BreakBeam（本帧唯一推进点）----
            robotPosition.update();

            // ---- 操作手手动清空 ----
            if (gamepad1.yWasReleased()) {
                robotPosition.resetBallCount();
            }

            // ---- 控制：下发 Sweeper（满额自动停吸由 Sweeper 内部判定）----
            sweeper.update();

            // ---- 控制：炮台（固定 yaw / 俯仰，二操按住 A 发射）----
            double pitchBig = (Shooter.BIG_PARAMS.pitchMin + Shooter.BIG_PARAMS.pitchMax) / 2.0;
            double pitchSmall = (Shooter.SMALL_PARAMS.pitchMin + Shooter.SMALL_PARAMS.pitchMax) / 2.0;
            boolean fire = gamepad2.a;
            Turret.FireReady ready = turret.update(testYaw, pitchBig, pitchSmall,
                    testSpeedBig, testSpeedSmall, fire, fire);

            // ---- 炮台发射完毕 → 清空球道计数 ----
            // 由 Turret.update 检测扳机下降沿后直接调用 RobotPosition.resetBallCount()，
            // OpMode 无需转发。

            // ---- 遥测 ----
            telemetry.addData("Ball Direction", ballDirection == BallCounter.MODE_INTAKE
                    ? "INTAKE (+1)" : "OUTPUT (-1)");
            telemetry.addData("Ball Count", robotPosition.getBallCount());
            telemetry.addData("Ball Capacity", Sweeper.BallCapacity);
            telemetry.addData("Ball Lane Full", sweeper.isBallLaneFull());
            telemetry.addData("Sweeper Target (tick/s)", sweeper.getTargetVelocity());
            telemetry.addData("Fire Ready big/small", ready.big + " / " + ready.small);
            BallCounter ballCounter = robotPosition.getBallCounter();
            telemetry.addData("Break Beam",
                    ballCounter != null && ballCounter.isBlocked() ? 1 : 0);
            turret.setTelemetry();
            telemetry.update();
        }

        turret.stop();
        sweeper.setStop();
        sweeper.update();
    }
}
