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

import org.firstinspires.ftc.teamcode.utility.PID.PIDSVAController;
import org.firstinspires.ftc.teamcode.utility.PID.SlotConfig;

/**
 * 单电机测试台：验证一个电机的三种被控量——功率 / 速度 / 距离（位置）。
 *
 * <p><b>手柄映射（一操）</b>
 * <ul>
 *   <li>X 键 → 循环切换被控量：POWER → VELOCITY → DISTANCE</li>
 *   <li>方向键 上 / 下 → 增大 / 减小当前被控量的目标值（上升沿触发，步长见 Dashboard）</li>
 *   <li>Y 键 → 紧急停车（目标归零、功率清零、控制器复位）</li>
 * </ul>
 * 方向键采用上升沿检测（{@code dpad*WasPressed()}），每按一次走一步，避免长按粘滞。
 *
 * <p><b>控制方式</b>
 * 电机运行在 {@code RUN_WITHOUT_ENCODER}，由程序直接下发功率。
 * <ul>
 *   <li>POWER：直接 {@code setPower}，不做闭环。</li>
 *   <li>VELOCITY / DISTANCE：使用 {@link PIDSVAController} 软件闭环
 *       （速度环 velocityLoop=true，位置环 velocityLoop=false）。</li>
 * </ul>
 *
 * <p><b>整定</b>：在 FTC Dashboard 的 {@code MotorTest} 分组中实时修改
 * 速度环/位置环的 kP/kI/kD/kS/kV/kA 与方向键步长；把 target / current / error
 * 加入 plot 观察闭环响应。这里的 PID 参数为占位值，需实机整定。
 */
@Config
@TeleOp(name = "MotorTest", group = "Tests")
public class MotorTest extends LinearOpMode {

    // ==================== 可调参数（FTC Dashboard 实时生效） ====================

    /** 被控电机在 Robot Configuration 中的名称（改动后需重新 Init） */
    public static String motorName = "motor";

    // ---- 方向键步长 ----
    /** 功率模式：每次按键的功率增量 */
    public static double powerStep = 0.05;
    /** 速度模式：每次按键的转速增量 (tick/s) */
    public static double velocityStep = 100;
    /** 距离模式：每次按键的位置增量 (tick) */
    public static double positionStep = 50;
    /** 速度模式目标转速限幅 (tick/s) */
    public static double maxVelocity = 3000;

    // ---- 速度环 PID/SVA（占位值，需实机整定） ----
    public static double velKP = 0.0005;
    public static double velKI = 0.0;
    public static double velKD = 0.0;
    public static double velKS = 0.0;
    public static double velKV = 0.0;
    public static double velKA = 0.0;
    public static double velMaxI = 0.5;
    public static double velIZone = 500;

    // ---- 位置环 PID/SVA（占位值，需实机整定） ----
    public static double posKP = 0.008;
    public static double posKI = 0.0;
    public static double posKD = 0.0;
    public static double posKS = 0.0;
    public static double posKV = 0.0;
    public static double posKA = 0.0;
    public static double posMaxI = 0.5;
    public static double posIZone = 200;

    // ==================== 内部控制模式 ====================

    private static final int MODE_POWER = 0;
    private static final int MODE_VELOCITY = 1;
    private static final int MODE_DISTANCE = 2;
    private static final int MODE_COUNT = 3;
    private static final String[] MODE_NAMES = {"POWER", "VELOCITY", "DISTANCE"};

    // ==================== 运行时状态 ====================

    private DcMotorEx motor;
    private PIDSVAController velocityCtrl;
    private PIDSVAController positionCtrl;
    private final SlotConfig velConfig = new SlotConfig().withOutputLimits(-1.0, 1.0);
    private final SlotConfig posConfig = new SlotConfig().withOutputLimits(-1.0, 1.0);

    private int mode = MODE_POWER;
    private double targetPower = 0;
    private double targetVelocity = 0;
    private double targetPosition = 0;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private double dt = 1e-3;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, motorName);
        // 软件闭环需要直接控制功率，故关闭固件闭环；编码器仍可读速度/位置
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        velocityCtrl = new PIDSVAController().withSlot0(velConfig);
        positionCtrl = new PIDSVAController().withSlot0(posConfig);
        targetPosition = motor.getCurrentPosition();

        telemetry.addLine("X: switch mode (Power/Velocity/Distance)");
        telemetry.addLine("Dpad U/D: change setpoint | Y: E-stop");
        telemetry.addLine("Params: Dashboard group MotorTest");
        telemetry.update();

        waitForStart();
        timer.reset();
        lastTime = 0;

        while (opModeIsActive()) {
            updateDt();
            handleModeSwitch();
            updateSlotConfigs();

            // 方向键上升沿步进当前被控量（每按一次走一步，避免长按粘滞）
            int step = 0;
            if (gamepad1.dpadUpWasPressed()) step += 1;
            if (gamepad1.dpadDownWasPressed()) step -= 1;
            applyStep(step);

            // Y 键急停
            if (gamepad1.yWasPressed()) {
                emergencyStop();
            }

            runControl();
            setTelemetry();
            telemetry.update();
        }

        motor.setPower(0);
    }

    // ==================== 主循环辅助 ====================

    /** 计算本帧 dt（秒），并做过小保护，避免微分项发散 */
    private void updateDt() {
        double now = timer.seconds();
        dt = now - lastTime;
        lastTime = now;
        if (dt <= 1e-4) dt = 1e-4;
    }

    /** X 键切换被控量；切换时停车并复位控制器，避免目标跳变 */
    private void handleModeSwitch() {
        if (gamepad1.xWasPressed()) {
            mode = (mode + 1) % MODE_COUNT;
            targetPower = 0;
            targetVelocity = 0;
            targetPosition = motor.getCurrentPosition();
            motor.setPower(0);
            velocityCtrl.reset();
            positionCtrl.reset();
        }
    }

    /** 把 Dashboard 上的静态参数写回两个 slot（复用同一 SlotConfig 实例） */
    private void updateSlotConfigs() {
        velConfig.withKP(velKP).withKI(velKI).withKD(velKD)
                .withKS(velKS).withKV(velKV).withKA(velKA)
                .withMaxI(velMaxI).withIZone(velIZone)
                .withOutputLimits(-1.0, 1.0);
        velocityCtrl.resetSlot(0, velConfig);

        posConfig.withKP(posKP).withKI(posKI).withKD(posKD)
                .withKS(posKS).withKV(posKV).withKA(posKA)
                .withMaxI(posMaxI).withIZone(posIZone)
                .withOutputLimits(-1.0, 1.0);
        positionCtrl.resetSlot(0, posConfig);
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
            case MODE_DISTANCE:
                targetPosition += step * positionStep;
                break;
        }
    }

    /** Y 键急停：目标归零（位置环锁定当前位）、功率清零、控制器复位 */
    private void emergencyStop() {
        targetPower = 0;
        targetVelocity = 0;
        targetPosition = motor.getCurrentPosition();
        motor.setPower(0);
        velocityCtrl.reset();
        positionCtrl.reset();
    }

    /** 按当前模式下发功率：功率直通，速度/距离走 PIDSVA 闭环 */
    private void runControl() {
        switch (mode) {
            case MODE_POWER:
                motor.setPower(targetPower);
                break;
            case MODE_VELOCITY:
                motor.setPower(velocityCtrl.calculate(targetVelocity, motor.getVelocity(), dt, true));
                break;
            case MODE_DISTANCE:
                motor.setPower(positionCtrl.calculate(targetPosition, motor.getCurrentPosition(), dt, false));
                break;
        }
    }

    private void setTelemetry() {
        telemetry.addData("Mode", "%s  (X to switch)", MODE_NAMES[mode]);
        telemetry.addData("dt (ms)", "%.1f", dt * 1000);
        // 三种模式统一显示当前功率与当前转速
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
                addPidTelemetry(velocityCtrl);
                break;
            case MODE_DISTANCE:
                telemetry.addData("Target (tick)", "%.0f", targetPosition);
                telemetry.addData("Position (tick)", motor.getCurrentPosition());
                telemetry.addData("Position error (tick)", "%.0f", targetPosition - motor.getCurrentPosition());
                addPidTelemetry(positionCtrl);
                break;
        }
    }

    private void addPidTelemetry(PIDSVAController c) {
        telemetry.addData("Terms P/I/D", "%.4f / %.4f / %.4f",
                c.getLastPTerm(), c.getLastITerm(), c.getLastDTerm());
        telemetry.addData("Terms S/V/A", "%.4f / %.4f / %.4f",
                c.getLastSTerm(), c.getLastVTerm(), c.getLastATerm());
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
