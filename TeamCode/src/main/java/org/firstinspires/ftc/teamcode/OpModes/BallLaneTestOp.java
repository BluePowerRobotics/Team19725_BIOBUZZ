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
 * 球道计数架构联调程序（整车测试，需要 Limelight / Pinpoint / 光电门 / 炮台全部已配置）。
 *
 * <p><b>对象层级</b>（与架构图一致，每帧自上而下更新）：
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
 *   <li>计数达到 {@link Sweeper#BallCapacity}（默认 4）时停止吸球，送球 / 出球不受影响；</li>
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
 * <p>调参：FTC Dashboard → Config → BallLaneTestOp / Sweeper / Turret。
 */
@Config
@TeleOp(name = "BallLaneTestOp", group = "Tests")
public class BallLaneTestOp extends LinearOpMode {

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

            // ---- 信号处理：球道方向由本帧吸球 / 出球指令决定（出球 -1，其余 +1）----
            int ballDirection = gamepad1.right_bumper
                    ? BallCounter.MODE_OUTPUT : BallCounter.MODE_INTAKE;

            // ---- 状态估计：RobotPosition 内部更新 BallCounter → BreakBeam ----
            robotPosition.update(ballDirection);

            // ---- 操作手手动清空 ----
            if (gamepad1.yWasReleased()) {
                robotPosition.resetBallCount();
            }

            // ---- 控制：吸球 / 出球（满额自动停吸由 Sweeper 内部判定）----
            if (gamepad1.left_bumper) {
                sweeper.setEat();
            }
            if (gamepad1.right_bumper) {
                sweeper.setOutput();
            }
            sweeper.update(robotPosition.getBallCount());

            // ---- 控制：炮台（固定 yaw / 俯仰，二操按住 A 发射）----
            double pitchBig = (Shooter.BIG_PARAMS.pitchMin + Shooter.BIG_PARAMS.pitchMax) / 2.0;
            double pitchSmall = (Shooter.SMALL_PARAMS.pitchMin + Shooter.SMALL_PARAMS.pitchMax) / 2.0;
            boolean fire = gamepad2.a;
            Turret.FireReady ready = turret.update(testYaw, pitchBig, pitchSmall,
                    testSpeedBig, testSpeedSmall, fire, fire);

            // ---- 炮台发射完毕 → 清空球道计数 ----
            if (turret.consumeResetBallCountRequest()) {
                robotPosition.resetBallCount();
            }

            // ---- 遥测 ----
            telemetry.addData("Ball Count", robotPosition.getBallCount());
            telemetry.addData("Ball Capacity", Sweeper.BallCapacity);
            telemetry.addData("Ball Lane Full", sweeper.isBallLaneFull());
            telemetry.addData("Sweeper Target (tick/s)", sweeper.getTargetVelocity());
            telemetry.addData("Fire Ready big/small", ready.big + " / " + ready.small);
            telemetry.addData("Reset pending", turret.hasResetBallCountRequest());
            turret.setTelemetry();
            telemetry.update();
        }

        turret.stop();
        sweeper.setStop();
        sweeper.update();
    }
}
