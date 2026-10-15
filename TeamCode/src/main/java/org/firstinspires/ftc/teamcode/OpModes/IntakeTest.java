package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Processors.Sensors.PressureSwitcher;

/**
 * Intake 测试台：在同一个 OpMode 中同时测试 intake 电机与压敏开关。
 *
 * <p><b>电机部分</b>
 * <ul>
 *   <li>X 键 → 循环切换被控量：POWER → VELOCITY</li>
 *   <li>方向键 上 / 下 → 增大 / 减小当前被控量的目标值（上升沿触发，步长见 Dashboard）</li>
 *   <li>Y 键 → 紧急停车（目标归零、功率清零）</li>
 * </ul>
 * POWER：{@code RUN_WITHOUT_ENCODER} 下直接 {@code setPower}，不做闭环。
 * VELOCITY：与 {@code Controllers/Sweeper} 相同的<b>固件速度闭环</b>——
 * {@code RUN_USING_ENCODER} + {@code setVelocityPIDFCoefficients} + {@code setVelocity}，
 * PIDF 系数可在 Dashboard 实时调节。
 *
 * <p><b>压敏部分（与 {@code PressureTestOp} 相同）</b>
 * 每帧调用 {@link PressureSwitcher#update()}，实时显示原始电压 / 滤波电压 /
 * 阈值 / 按压判定 / alpha；threshold、alpha、resetFilter 在 Dashboard 可调。
 */
@Config
@TeleOp(name = "IntakeTest", group = "Tests")
public class IntakeTest extends LinearOpMode {

    // ==================== 硬件设备名 ====================

    /** 被控电机在 Robot Configuration 中的名称（改动后需重新 Init） */
    public static String motorName = "sweeperMotor";
    /** 压敏电阻模拟输入在 Robot Configuration 中的名称 */
    private static final String PRESSURE_DEVICE = "pressure_sensor";

    // ==================== 电机可调参数（FTC Dashboard 实时生效） ====================

    // ---- 方向键步长 ----
    /** 功率模式：每次按键的功率增量 */
    public static double powerStep = 0.05;
    /** 速度模式：每次按键的转速增量 (tick/s) */
    public static double velocityStep = 100;
    /** 速度模式目标转速限幅 (tick/s) */
    public static double maxVelocity = 3000;

    // ---- 速度环固件 PIDF（REV Hub 内置速度 PID，Dashboard 实时生效） ----
    public static double VelocityP = 1.17;
    public static double VelocityI = 0.117;
    public static double VelocityD = 0.0;
    public static double VelocityF = 0.0;

    // ==================== 压敏可调参数（FTC Dashboard 实时生效） ====================

    /** 归一化压力阈值 [0, 1] */
    public static double threshold = 0.5;
    /** EMA 平滑系数 (0, 1] */
    public static double alpha = 0.8;
    /** 置 true 时重置一次滤波器状态（仅上升沿生效） */
    public static boolean resetFilter = false;

    // ==================== 内部控制模式 ====================

    private static final int MODE_POWER = 0;
    private static final int MODE_VELOCITY = 1;
    private static final int MODE_COUNT = 2;
    private static final String[] MODE_NAMES = {"POWER", "VELOCITY"};

    // ==================== 运行时状态 ====================

    private DcMotorEx motor;

    private int mode = MODE_POWER;
    private double targetPower = 0;
    private double targetVelocity = 0;

    /** 最近一次下发的固件 PIDF 系数，仅在其变化时重新写入 Hub */
    private double appliedP = Double.NaN;
    private double appliedI = Double.NaN;
    private double appliedD = Double.NaN;
    private double appliedF = Double.NaN;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private double dt = 1e-3;

    private PressureSwitcher pressure;
    /** 上一次的 resetFilter 值，用于检测上升沿 */
    private boolean lastResetFilter = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- 电机初始化 ----
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        // 初始为 POWER 模式：直接下发功率
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        // ---- 压敏初始化 ----
        pressure = new PressureSwitcher(hardwareMap, PRESSURE_DEVICE);

        telemetry.addData("Motor", motorName);
        telemetry.addData("Pressure", PRESSURE_DEVICE);
        telemetry.addLine("X: switch mode (Power/Velocity)");
        telemetry.addLine("Dpad U/D: change setpoint | Y: E-stop");
        telemetry.addLine("Params: Dashboard group IntakeTest");
        telemetry.update();

        waitForStart();
        timer.reset();
        lastTime = 0;

        while (opModeIsActive()) {
            updateDt();
            handleModeSwitch();

            // 方向键上升沿步进当前被控量（每按一次走一步，避免长按粘滞）
            int step = 0;
            if (gamepad1.dpadUpWasPressed()) step += 1;
            if (gamepad1.dpadDownWasPressed()) step -= 1;
            applyStep(step);

            // Y 键急停
            if (gamepad1.yWasPressed()) {
                emergencyStop();
            }

            updatePressure();

            runControl();
            setTelemetry();
            telemetry.update();
        }

        motor.setPower(0);
    }

    // ==================== 电机辅助 ====================

    /** 计算本帧 dt（秒），并做过小保护，避免微分项发散 */
    private void updateDt() {
        double now = timer.seconds();
        dt = now - lastTime;
        lastTime = now;
        if (dt <= 1e-4) dt = 1e-4;
    }

    /**
     * X 键切换被控量；切换时停车并切换到对应 RunMode。
     * POWER 用 {@code RUN_WITHOUT_ENCODER}，VELOCITY 用固件闭环所需的
     * {@code RUN_USING_ENCODER}。
     */
    private void handleModeSwitch() {
        if (gamepad1.xWasPressed()) {
            mode = (mode + 1) % MODE_COUNT;
            targetPower = 0;
            targetVelocity = 0;
            motor.setPower(0);
            if (mode == MODE_VELOCITY) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    /** 速度模式下，把 Dashboard 上的固件 PIDF 系数写回 Hub（仅变化时写入） */
    private void updateFirmwarePidf() {
        if (VelocityP != appliedP || VelocityI != appliedI
                || VelocityD != appliedD || VelocityF != appliedF) {
            motor.setVelocityPIDFCoefficients(VelocityP, VelocityI, VelocityD, VelocityF);
            appliedP = VelocityP;
            appliedI = VelocityI;
            appliedD = VelocityD;
            appliedF = VelocityF;
        }
    }

    /** 按当前模式应用方向键步进量 */
    private void applyStep(int step) {
        if (step == 0) return;
        switch (mode) {
            case MODE_POWER:
                targetPower = clamp(targetPower + step * powerStep, -1.0, 1.0);
                break;
            case MODE_VELOCITY:
                targetVelocity = clamp(targetVelocity + step * velocityStep, -maxVelocity, maxVelocity);
                break;
        }
    }

    /** Y 键急停：目标归零、功率清零 */
    private void emergencyStop() {
        targetPower = 0;
        targetVelocity = 0;
        motor.setPower(0);
    }

    /** 按当前模式驱动电机：POWER 直通 setPower，VELOCITY 走固件速度闭环 */
    private void runControl() {
        switch (mode) {
            case MODE_POWER:
                motor.setPower(targetPower);
                break;
            case MODE_VELOCITY:
                updateFirmwarePidf();
                motor.setVelocity(targetVelocity);
                break;
        }
    }

    // ==================== 压敏辅助 ====================

    /** 采样 + EMA 滤波 + 阈值判定，并应用 Dashboard 参数与上升沿复位 */
    private void updatePressure() {
        // 必须每帧调用
        pressure.update();

        // 上升沿重置滤波器
        if (resetFilter && !lastResetFilter) {
            pressure.reset();
        }
        lastResetFilter = resetFilter;

        // 每帧应用 Dashboard 参数（带范围保护，避免非法值抛异常）
        pressure.setThreshold(clamp(threshold, 0.0, 1.0));      // threshold ∈ [0, 1]
        pressure.setAlpha(clamp(alpha, 0.01, 1.0));             // alpha ∈ (0, 1]
    }

    // ==================== 遥测 ====================

    private void setTelemetry() {
        telemetry.addData("Mode", "%s  (X to switch)", MODE_NAMES[mode]);
        telemetry.addData("dt (ms)", "%.1f", dt * 1000);
        // 两种模式统一显示当前功率与当前转速
        telemetry.addData("Power", "%.4f", motor.getPower());
        telemetry.addData("Velocity (tick/s)", "%.0f", motor.getVelocity());
        telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());
        switch (mode) {
            case MODE_POWER:
                telemetry.addData("Target power", "%.3f", targetPower);
                break;
            case MODE_VELOCITY:
                telemetry.addData("Target (tick/s)", "%.0f", targetVelocity);
                telemetry.addData("Velocity error (tick/s)", "%.0f", targetVelocity - motor.getVelocity());
                break;
        }

        // ---- 压敏遥测 ----
        telemetry.addData("Pressure raw", "%.3f", pressure.getRawVoltage());
        telemetry.addData("Pressure filtered", "%.3f", pressure.getVoltage());
        telemetry.addData("Pressure threshold", "%.3f", pressure.getThreshold());
        telemetry.addData("Pressure pressed", pressure.isPressed() ? 1 : 0);
        telemetry.addData("Pressure alpha", "%.3f", pressure.getAlpha());
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
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
