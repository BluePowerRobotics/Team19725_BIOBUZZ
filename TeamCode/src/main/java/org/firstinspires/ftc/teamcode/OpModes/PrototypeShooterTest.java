package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;

/**
 * 原型发射机构（飞轮）测试：用手柄调节目标转速，并在 Dashboard 上观察速度环表现。
 *
 * <p><b>操作</b>
 * <ul>
 *   <li>方向键上 / 下 → 按 {@link #velocityStep} 步长增减目标转速，按住则连续调节</li>
 *   <li>A 键 → 立即停转（目标转速置 0）</li>
 * </ul>
 *
 * <p><b>整定</b>：第 4 代（2026-10-22）起参数不在本类也不在 {@code PrototypeShooter} 上，而在
 * {@link org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelParams#BIG}（Dashboard 分组
 * {@code FlywheelParams}）：kS / kff / kp / ki / maxI / 两个滤波器时间常数 / 电压归一化开关。
 * 现场调参时改这四个增益，观察遥测里"实际转速"跟随"目标转速"的上升时间、超调量与稳态误差；
 * 各参数的整定方法与验收判据见 {@code OpModes/Tuning/Guide.md}。
 *
 * <p>勾开 {@code PrototypeShooter.telemetryDetailed} 可展开误差/滤波/电压归一化等诊断行。
 *
 * <p>目标转速被限幅在 ±{@link #MaxTargetVelocity} 内，该值需按电机实际最大转速设置（tick/s）。
 */
@Config
@TeleOp(name = "PrototypeShooterTest", group = "Tests")
public class PrototypeShooterTest extends LinearOpMode {

    /** 目标转速绝对值上限 (tick/s) */
    public static int MaxTargetVelocity = 3000;

    /** 方向键单次调节步长 (tick/s) */
    public static int velocityStep = 100;

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

        telemetry.addLine("Dpad Up/Down: adjust target velocity (hold to repeat)");
        telemetry.addLine("A: stop");
        telemetry.addLine("Tune kS/kff/kp/ki in Dashboard (group: FlywheelParams -> BIG)");
        shooter.setTelemetry();
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                shooter.stop();
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

        shooter.stop();
        shooter.update();
    }

    private static int clampVelocity(int velocity) {
        return Math.max(-MaxTargetVelocity, Math.min(MaxTargetVelocity, velocity));
    }
}
