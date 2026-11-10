package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Controllers.Turret.YawController;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.Processors.Sensors.BallCounter;

/**
 * 球道计数架构的总代码（顶层 OpMode）。
 *
 * <p><b>分层结构</b>
 * <pre>
 * OpMode（本类，总代码）
 *  ├─ update → RobotPosition（球道部分）─→ BallCounter ─→ BreakBeamSensor ─→ 光电门
 *  ├─ update → Sweeper      （满仓停吸，见 Sweeper）
 *  └─ update → Turret       （扳机下降沿 → resetBallCount，见 Turret）
 * </pre>
 *
 * <p><b>数据同步方式</b>
 * <ul>
 *   <li>只有 {@link RobotPosition#update()} 推进球道状态（唯一硬件采样点，计数方向由
 *       {@link RobotPosition#setBallMode(int)} 指定），Sweeper / Turret / 本类只读 getter；</li>
 *   <li>炮台发射完毕（扳机 true → false 且当时可发射）→ {@link RobotPosition#resetBallCount()}；</li>
 *   <li>操作手手动清空：A 键 = resetBallCount（轻量清空）；深复位 {@code reset} 由
 *       {@link RobotPosition#ResetPoseTo} 在重定位时内部发起。</li>
 * </ul>
 *
 * <p><b>每帧执行顺序</b>：① 定吸球/倒球意图与计数方向 → ② setBallMode + update() → ③ 手动清空
 * → ④ sweeper.update() → ⑤ turret.update() → ⑥ 遥测。
 *
 * <p><b>手柄映射</b>
 * <ul>
 *   <li>P1 左 bumper：吸球（intake，计数 +1）；右 bumper：倒球（output，计数 -1）</li>
 *   <li>P1 A：手动轻量清空计数（resetBallCount）</li>
 *   <li>P2 左摇杆 Y：小球俯仰；右摇杆 Y：大球俯仰；左摇杆 X：平转 yaw</li>
 *   <li>P2 左 bumper：小球扳机；右 bumper：大球扳机</li>
 * </ul>
 */
@Config
@TeleOp(name = "BallControlTeleOp", group = "BallControl")
public class BallControlTeleOp extends LinearOpMode {

    /** 大球飞轮目标转速 (tick/s)，超过上限会被钳制 */
    public static int fireSpeedBig = 2000;
    /** 小球飞轮目标转速 (tick/s)，超过上限会被钳制 */
    public static int fireSpeedSmall = 2000;

    private RobotPosition robotPosition;
    private Sweeper sweeper;
    private Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- init 顺序：先创建下属对象，再进入主循环使用（避免未写先用）----
        // ① RobotPosition（球道部分）先建：其 BallCounter 首次使用时再建 BreakBeamSensor
        robotPosition = RobotPosition.initBallChannelOnly(hardwareMap);
        // 在 init 阶段就创建光电门对象：设备缺失会在 init 报错，而不是主循环第一帧
        robotPosition.ensureBallChannel();
        // ② Sweeper / ③ Turret 依赖 RobotPosition，在其之后创建
        sweeper = new Sweeper(hardwareMap, telemetry, robotPosition);
        turret = new Turret(hardwareMap, telemetry, robotPosition);

        telemetry.addLine("P1 LB: eat | RB: output");
        telemetry.addLine("P1 A: resetBallCount");
        telemetry.addLine("P2 sticks: turret aim | LB/RB: small/big trigger");
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Ball full count", Sweeper.FULL_COUNT);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ① 操作手意图：吸球 / 倒球，计数方向与动作方向保持一致
            boolean wantEat = gamepad1.left_bumper;
            boolean wantOutput = gamepad1.right_bumper;

            int mode;
            if (wantEat) {
                sweeper.setEat();
                mode = BallCounter.MODE_INTAKE;
            } else if (wantOutput) {
                sweeper.setOutput();
                mode = BallCounter.MODE_OUTPUT;
            } else {
                sweeper.setStop();
                mode = BallCounter.MODE_INTAKE;
            }

            // ② 推进 RobotPosition：本帧先定计数方向，再走唯一的「更新」边
            //    （定位 + 球道计数；全工程唯一硬件采样点）
            robotPosition.setBallMode(mode);
            robotPosition.update();

            // ③ 手动清空（边沿触发：松开时触发一次）
            if (gamepad1.aWasReleased()) {
                robotPosition.resetBallCount();
            }

            // ④ 更新 Sweeper（下发本帧速度）
            sweeper.update();

            // ⑤ 炮台：读取 P2 并下发目标（扳机下降沿 → resetBallCount 在 Turret 内部触发）
            double pitchSmall = axisToRange(-gamepad2.left_stick_y,
                    Shooter.SMALL_PARAMS.pitchMin, Shooter.SMALL_PARAMS.pitchMax);
            double pitchBig = axisToRange(-gamepad2.right_stick_y,
                    Shooter.BIG_PARAMS.pitchMin, Shooter.BIG_PARAMS.pitchMax);
            double yaw = -gamepad2.left_stick_x * YawController.yawRange;
            int speedBig = clamp(fireSpeedBig, 0, Shooter.BIG_PARAMS.flywheel.speedMax);
            int speedSmall = clamp(fireSpeedSmall, 0, Shooter.SMALL_PARAMS.flywheel.speedMax);
            boolean triggerBig = gamepad2.right_bumper;
            boolean triggerSmall = gamepad2.left_bumper;

            Turret.FireReady ready = turret.update(yaw, pitchBig, pitchSmall,
                    speedBig, speedSmall, triggerBig, triggerSmall);

            // ⑥ 遥测（只读本帧缓存，不读硬件）
            robotPosition.setBallTelemetry(telemetry);
            telemetry.addData("Sweeper target vel", sweeper.getTargetVelocity());
            telemetry.addData("Fire ready big/small", "%b / %b", ready.big, ready.small);
            telemetry.update();
        }

        // ---- 退出：先停吸球，再停炮台 ----
        sweeper.stop();
        turret.stop();
    }

    /** 摇杆位置 [-1, 1] 线性映射到 [lo, hi] */
    private static double axisToRange(double axis, double lo, double hi) {
        double t = Math.max(0, Math.min(1, (axis + 1) / 2));
        return lo + t * (hi - lo);
    }

    /** 把整数 v 钳制到 [lo, hi] */
    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
