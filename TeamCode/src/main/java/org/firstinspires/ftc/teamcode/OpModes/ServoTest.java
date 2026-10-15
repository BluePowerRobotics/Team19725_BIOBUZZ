package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * 单舵机测试台：用一个舵机验证行程与目标位置控制。
 *
 * <p><b>手柄映射（一操）</b>
 * <ul>
 *   <li>方向键 上 / 下 → 增大 / 减小目标位置（上升沿触发，步长见 Dashboard）</li>
 * </ul>
 * 方向键采用上升沿检测（{@code dpad*WasPressed()}），每按一次走一步，避免长按粘滞。
 *
 * <p><b>控制方式</b>
 * 目标位置为归一化值 [0,1]，被钳制在 {@link #minPosition} ~ {@link #maxPosition}
 * 之间（防止机械过冲撞限位），每帧下发 {@code Servo.setPosition}。
 *
 * <p><b>整定</b>：在 FTC Dashboard 的 {@code ServoTest} 分组中实时修改
 * positionStep / minPosition / maxPosition；观察舵机实际行程与遥测的对应关系。
 */
@Config
@TeleOp(name = "ServoTest", group = "Tests")
public class ServoTest extends LinearOpMode {

    // ==================== 可调参数（FTC Dashboard 实时生效） ====================

    /** 舵机在 Robot Configuration 中的名称（改动后需重新 Init） */
    public static String servoName = "servo";

    /** 每次按键的位置增量 [0,1] */
    public static double positionStep = 0.05;
    /** 允许的最小位置（防机械过冲），范围 [0,1] */
    public static double minPosition = 0.0;
    /** 允许的最大位置（防机械过冲），范围 [0,1] */
    public static double maxPosition = 1.0;

    // ==================== 运行时状态 ====================

    private Servo servo;
    /** 当前目标位置（归一化 [0,1]） */
    private double targetPosition = 0.5;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = hardwareMap.get(Servo.class, servoName);
        // 以舵机当前的指令位置作为初始目标，避免上电即跳到 0.5
        targetPosition = servo.getPosition();

        telemetry.addData("Servo", servoName);
        telemetry.addLine("Dpad U/D: change target position");
        telemetry.addLine("Params: Dashboard group ServoTest");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 方向键上升沿步进目标位置（每按一次走一步，避免长按粘滞）
            int step = 0;
            if (gamepad1.dpadUpWasPressed()) step += 1;
            if (gamepad1.dpadDownWasPressed()) step -= 1;
            if (step != 0) {
                targetPosition = clamp(targetPosition + step * positionStep,
                        clamp01(minPosition), clamp01(maxPosition));
            }

            // 每帧下发，保证 Dashboard 改参后即时生效
            servo.setPosition(targetPosition);

            setTelemetry();
            telemetry.update();
        }
    }

    // ==================== 遥测 ====================

    private void setTelemetry() {
        telemetry.addData("Target position", "%.3f", targetPosition);
        telemetry.addData("Servo position", "%.3f", servo.getPosition());
        telemetry.addData("Range", "%.2f ~ %.2f", minPosition, maxPosition);
        telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** 把数值限制到 [0, 1] */
    private static double clamp01(double v) {
        return clamp(v, 0.0, 1.0);
    }

    /** 取各 Hub 电压最小值（近似电池电压） */
    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            min = Math.min(min, sensor.getVoltage());
        }
        return min;
    }
}
