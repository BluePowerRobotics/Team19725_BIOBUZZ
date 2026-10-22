package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;

/**
 * 飞轮运转程序：Start 后把飞轮拉到待发转速，并允许现场微调目标转速。
 *
 * <p>面向「实际运转」而非整定：整定参数固化在 {@link org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelParams}
 * 的 {@code BIG} 参数集里（Dashboard 分组 {@code FlywheelParams}），本程序只做目标转速管理与发起闭环绕组。
 *
 * <p><b>操作</b>
 * <ul>
 *   <li>Start → 目标转速置为 {@link #startVelocity}（默认 1700 tick/s）</li>
 *   <li>方向键上 / 下 → 按 {@link #velocityStep} 步长增减目标转速，按住则按 {@link #repeatPeriodMs} 连续调节</li>
 *   <li>X 键 → 立即停转（目标转速置 0，闭环会立刻把占空比收到下限，飞轮惯性滑行）</li>
 *   <li>A 键 → 重启，目标转速重新置为 {@link #startVelocity}</li>
 * </ul>
 *
 * <p>目标转速限幅在 [0, {@link #maxVelocity}]（与 PrototypeShooterTest 一致，按电机实际最大转速设置）。
 *
 * <p>遥测为飞轮统一列表（5 行）：目标转速 / 当前转速 / power(占空比) / 电流 / 电压。
 */
@Config
@TeleOp(name = "FlywheelRun", group = "Tests")
public class FlywheelRun extends LinearOpMode {

    /** Start 后的初始目标转速 (tick/s) */
    public static int startVelocity = 1700;

    /** 方向键单次调节步长 (tick/s) */
    public static int velocityStep = 100;

    /** 目标转速上限 (tick/s) */
    public static int maxVelocity = 3000;

    /** 按住方向键时的连续调节周期 (毫秒) */
    public static long repeatPeriodMs = 100;

    private PrototypeShooter shooter;

    /** 上一次的方向键状态：-1 下 / 0 未按 / 1 上 */
    private int lastDir = 0;

    /** 连续调节计时器 */
    private final ElapsedTime repeatTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new PrototypeShooter(hardwareMap, telemetry);
        shooter.stop();

        telemetry.addLine("Start -> target = " + startVelocity + " tick/s");
        telemetry.addLine("Dpad Up/Down: adjust target velocity (hold to repeat)");
        telemetry.addLine("X: stop    A: restart to " + startVelocity);
        telemetry.addLine("Params: utility.Shooter.FlywheelParams.BIG (Dashboard group: FlywheelParams)");
        shooter.setTelemetry();
        telemetry.update();

        waitForStart();

        // Start 后进入待发转速
        shooter.setTargetVelocity(clampVelocity(startVelocity));

        while (opModeIsActive()) {
            if (gamepad1.xWasPressed()) {
                shooter.stop(); // 立即停转
            }
            if (gamepad1.aWasPressed()) {
                shooter.setTargetVelocity(clampVelocity(startVelocity)); // 重启到待发转速
            }

            // 方向键上下调节目标转速：按下立即走一步，按住则按 repeatPeriodMs 连续调节
            int dir = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
            if (dir != lastDir) {
                if (dir != 0) {
                    shooter.setTargetVelocity(clampVelocity(shooter.getTargetVelocity() + dir * velocityStep));
                }
                repeatTimer.reset();
            } else if (dir != 0 && repeatTimer.milliseconds() >= repeatPeriodMs) {
                shooter.setTargetVelocity(clampVelocity(shooter.getTargetVelocity() + dir * velocityStep));
                repeatTimer.reset();
            }
            lastDir = dir;

            shooter.update();
            shooter.setTelemetry();
            telemetry.update();
        }

        // 退出前收油门
        shooter.stop();
        shooter.update();
    }

    private static int clampVelocity(int velocity) {
        return Math.max(0, Math.min(maxVelocity, velocity));
    }
}
