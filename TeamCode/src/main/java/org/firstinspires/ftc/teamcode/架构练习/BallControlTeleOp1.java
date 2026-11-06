package org.firstinspires.ftc.teamcode.架构练习;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Controllers.Turret.YawController;

/**
 * 球道计数架构的总代码（顶层 OpMode）—— 架构练习版。
 *
 * <p><b>分层结构</b>
 * <pre>
 * OpMode（本类，总代码）
 *  ├─ update → RobotPosition1 ─→ BallCounter1 ─→ BreakBeamSensor1 ─→ 光电门
 *  ├─ update → Sweeper1      （满仓停吸，见 Sweeper1）
 *  └─ update → Turret1       （扳机下降沿 → resetBallCount，见 Turret1）
 * </pre>
 *
 * <p><b>数据同步方式（对应架构图）</b>
 * <ul>
 *   <li>只有 {@link RobotPosition1#update(int)} 推进球道状态（唯一硬件采样点），
 *       Sweeper1 / Turret1 / 本类只读 {@link RobotPosition1} 的 getter；</li>
 *   <li>炮台发射完毕（扳机 true → false）→ {@link RobotPosition1#resetBallCount()}；</li>
 *   <li>操作手手动清空：A 键 = resetBallCount（轻量清空），B 键 = reset（深复位）。</li>
 * </ul>
 *
 * <p><b>每帧执行顺序</b>（与架构图自上而下一致）：
 * ① 定意图 → ② robotPosition.update(mode) → ③ 设吸球/倒球意图 → ④ 手动清空 →
 * ⑤ sweeper.update() → ⑥ turret.update() → ⑦ 遥测。
 *
 * <p><b>手柄映射</b>
 * <ul>
 *   <li>P1 左 bumper：吸球（intake，计数 +1）；右 bumper：倒球（output，计数 -1）</li>
 *   <li>P1 A：手动轻量清空计数（resetBallCount）；B：深复位（reset）</li>
 *   <li>P2 左摇杆 Y：小球俯仰；右摇杆 Y：大球俯仰；左摇杆 X：平转 yaw</li>
 *   <li>P2 左 bumper：小球扳机；右 bumper：大球扳机</li>
 * </ul>
 */
@Config
@TeleOp(name = "BallControlTeleOp1", group = "架构练习")
public class BallControlTeleOp1 extends LinearOpMode {

    /** 大球飞轮目标转速 (tick/s)，超过上限会被钳制 */
    public static int fireSpeedBig = 2000;
    /** 小球飞轮目标转速 (tick/s)，超过上限会被钳制 */
    public static int fireSpeedSmall = 2000;

    private RobotPosition1 robotPosition;
    private Sweeper1 sweeper;
    private Turret1 turret;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- init 顺序：先创建下属对象，再进入主循环使用（避免未写先用）----
        // ① RobotPosition1 先建（内部先建 BallCounter1 → 再建 BreakBeamSensor1）
        robotPosition = new RobotPosition1(hardwareMap);
        // ② Sweeper1 / ③ Turret1 依赖 RobotPosition1，在其之后创建
        sweeper = new Sweeper1(hardwareMap, telemetry, robotPosition);
        turret = new Turret1(hardwareMap, telemetry, robotPosition);

        telemetry.addLine("P1 LB: eat | RB: output");
        telemetry.addLine("P1 A: resetBallCount | B: reset");
        telemetry.addLine("P2 sticks: turret aim | LB/RB: small/big trigger");
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Ball full count", RobotPosition1.FULL_COUNT);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ① 操作手意图：吸球 / 倒球，计数方向与动作方向保持一致
            boolean wantEat = gamepad1.left_bumper;
            boolean wantOutput = gamepad1.right_bumper;

            int mode;
            if (wantEat) {
                sweeper.setEat();
                mode = BallCounter1.MODE_INTAKE;
            } else if (wantOutput) {
                sweeper.setOutput();
                mode = BallCounter1.MODE_OUTPUT;
            } else {
                sweeper.setStop();
                mode = BallCounter1.MODE_INTAKE;
            }

            // ② 推进球道状态（全工程唯一硬件采样点）
            robotPosition.update(mode);

            // ④ 手动清空（边沿触发：松开时触发一次）
            if (gamepad1.aWasReleased()) {
                robotPosition.resetBallCount();
            }
            if (gamepad1.bWasReleased()) {
                robotPosition.reset();
            }

            // ⑤ 更新 Sweeper（下发本帧速度）
            sweeper.update();

            // ⑥ 炮台：读取 P2 并下发目标（扳机下降沿 → resetBallCount 在 Turret1 内部触发）
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

            // ⑦ 遥测（只读本帧缓存，不读硬件）
            robotPosition.setTelemetry(telemetry);
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
