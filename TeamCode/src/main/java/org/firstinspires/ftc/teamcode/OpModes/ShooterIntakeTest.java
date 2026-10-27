package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter + Intake 联合测试台：用同一个手柄同时控制三路执行器。
 *
 * <p><b>执行器</b>
 * <ul>
 *   <li>intake 电机（对应 Controllers/Sweeper 的 sweeperMotor）</li>
 *   <li>shooter 飞轮电机</li>
 *   <li>shooter 舵机（两个已知位置间切换）</li>
 * </ul>
 *
 * <p><b>手柄映射（一操 gamepad1）</b>
 * <table>
 *   <tr><td>X</td><td>切换 intake 被控量：POWER → VELOCITY</td></tr>
 *   <tr><td>方向键 上 / 下</td><td>增大 / 减小 intake 目标值</td></tr>
 *   <tr><td>A</td><td>切换 shooter 被控量：POWER → VELOCITY</td></tr>
 *   <tr><td>方向键 左 / 右</td><td>增大 / 减小 shooter 目标值</td></tr>
 *   <tr><td>B</td><td>shooter 舵机在两个已知位置之间切换</td></tr>
 *   <tr><td>Y</td><td>急停（两个电机目标归零、功率清零）</td></tr>
 * </table>
 * 所有按键均为上升沿检测（{@code *WasPressed()}），避免长按粘滞；两个电机可独立设置目标，
 * 因此可以同时运行（例如一边吃球一边跑飞轮）。
 *
 * <p><b>控制方式</b>（与 {@code IntakeTest} 的电机部分一致）
 * 每个电机都有两种被控量：
 * <ul>
 *   <li>POWER：{@code RUN_WITHOUT_ENCODER} 下直接 {@code setPower}，不做闭环。</li>
 *   <li>VELOCITY：与 {@code Controllers/Sweeper} 相同的<b>固件速度闭环</b>——
 *       {@code RUN_USING_ENCODER} + {@code setVelocityPIDFCoefficients} + {@code setVelocity}。</li>
 * </ul>
 * 两路电机的 PIDF 系数各自独立，均可在 Dashboard 实时调节。
 *
 * <p><b>整定</b>：FTC Dashboard → {@code ShooterIntakeTest} 分组，可实时修改
 * 步长 / 两路 PIDF / 舵机两个位置；把 target 与 current 加入 plot 观察闭环响应。
 */
@Config
@TeleOp(name = "ShooterIntakeTest", group = "Tests")
public class ShooterIntakeTest extends LinearOpMode {

    // ==================== 硬件设备名 ====================

    /** intake 电机在 Robot Configuration 中的名称（改动后需重新 Init） */
    public static String intakeName = "sweeperMotor";
    /** shooter 飞轮电机在 Robot Configuration 中的名称 */
    public static String shooterName = "shooterMotor";
    /** shooter 舵机在 Robot Configuration 中的名称 */
    public static String shooterServoName = "shooterServo";

    // ==================== 可调参数（FTC Dashboard 实时生效） ====================

    // ---- 方向键步长 ----
    /** 功率模式：每次按键的功率增量 */
    public static double powerStep = 0.1;
    /** 速度模式：每次按键的转速增量 (tick/s) */
    public static double velocityStep = 100;
    /** 速度模式目标转速限幅 (tick/s) */
    public static double maxVelocity = 3000;

    // ---- intake 固件速度 PIDF ----
    public static double intakeP = 1.17;
    public static double intakeI = 0.117;
    public static double intakeD = 0.0;
    public static double intakeF = 0.0;

    // ---- shooter 固件速度 PIDF ----
    public static double shooterP = 1.17;
    public static double shooterI = 0.117;
    public static double shooterD = 0.0;
    public static double shooterF = 0.0;

    // ---- shooter 舵机两个已知位置 ----
    /** 舵机位置 A（如收回复位） */
    public static double servoPosA = 0.0;
    /** 舵机位置 B（如推出/释放） */
    public static double servoPosB = 1.0;

    // ==================== 内部控制模式 ====================

    private static final int MODE_POWER = 0;
    private static final int MODE_VELOCITY = 1;
    private static final int MODE_COUNT = 2;
    private static final String[] MODE_NAMES = {"POWER", "VELOCITY"};

    // ==================== 运行时状态 ====================

    private DcMotorEx intake;
    private DcMotorEx shooter;
    private Servo shooterServo;

    private int intakeMode = MODE_POWER;
    private int shooterMode = MODE_POWER;

    private double targetIntakePower = 0;
    private double targetIntakeVelocity = 0;
    private double targetShooterPower = 0;
    private double targetShooterVelocity = 0;

    /** shooter 舵机当前目标位置 */
    private double targetServoPos = 0;
    /** 舵机是否处于位置 B（用于 B 键在两个位置间切换） */
    private boolean servoAtB = false;

    /** 最近一次下发的固件 PIDF 系数，仅在其变化时重新写入 Hub */
    private double appliedIntakeP = Double.NaN;
    private double appliedIntakeI = Double.NaN;
    private double appliedIntakeD = Double.NaN;
    private double appliedIntakeF = Double.NaN;
    private double appliedShooterP = Double.NaN;
    private double appliedShooterI = Double.NaN;
    private double appliedShooterD = Double.NaN;
    private double appliedShooterF = Double.NaN;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private double dt = 1e-3;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- 两个电机初始化：初始为 POWER 模式，直接下发功率 ----
        intake = hardwareMap.get(DcMotorEx.class, intakeName);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        shooter = hardwareMap.get(DcMotorEx.class, shooterName);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setPower(0);

        // ---- 舵机初始化：以当前位置为初始目标，避免上电即跳变 ----
        shooterServo = hardwareMap.get(Servo.class, shooterServoName);
        targetServoPos = clamp(shooterServo.getPosition(), 0.0, 1.0);
        servoAtB = Math.abs(targetServoPos - servoPosB) < Math.abs(targetServoPos - servoPosA);

        telemetry.addData("Intake motor", intakeName);
        telemetry.addData("Shooter motor", shooterName);
        telemetry.addData("Shooter servo", shooterServoName);
        telemetry.addLine("X: intake mode | Dpad U/D: intake setpoint");
        telemetry.addLine("A: shooter mode | Dpad L/R: shooter setpoint");
        telemetry.addLine("B: servo A/B | Y: E-stop | Params: Dashboard");
        telemetry.update();

        waitForStart();
        timer.reset();
        lastTime = 0;

        while (opModeIsActive()) {
            updateDt();
            handleIntakeModeSwitch();
            handleShooterModeSwitch();

            // 方向键上升沿步进：上/下 = intake，左/右 = shooter
            int intakeStep = 0;
            if (gamepad1.dpadUpWasPressed()) intakeStep += 1;
            if (gamepad1.dpadDownWasPressed()) intakeStep -= 1;
            applyIntakeStep(intakeStep);

            int shooterStep = 0;
            if (gamepad1.dpadRightWasPressed()) shooterStep += 1;
            if (gamepad1.dpadLeftWasPressed()) shooterStep -= 1;
            applyShooterStep(shooterStep);

            // B 键在舵机两个已知位置间切换
            if (gamepad1.bWasPressed()) {
                servoAtB = !servoAtB;
                targetServoPos = servoAtB ? clamp01(servoPosB) : clamp01(servoPosA);
            }

            // Y 键急停
            if (gamepad1.yWasPressed()) {
                emergencyStop();
            }

            runIntake();
            runShooter();
            shooterServo.setPosition(targetServoPos);

            setTelemetry();
            telemetry.update();
        }

        intake.setPower(0);
        shooter.setPower(0);
    }

    // ==================== 主循环辅助 ====================

    /** 计算本帧 dt（秒），并做过小保护，避免微分项发散 */
    private void updateDt() {
        double now = timer.seconds();
        dt = now - lastTime;
        lastTime = now;
        if (dt <= 1e-4) dt = 1e-4;
    }

    /**
     * X 键切换 intake 被控量；切换时停车并切换到对应 RunMode。
     * POWER 用 {@code RUN_WITHOUT_ENCODER}，VELOCITY 用固件闭环所需的 {@code RUN_USING_ENCODER}。
     */
    private void handleIntakeModeSwitch() {
        if (gamepad1.xWasPressed()) {
            intakeMode = (intakeMode + 1) % MODE_COUNT;
            targetIntakePower = 0;
            targetIntakeVelocity = 0;
            intake.setPower(0);
            if (intakeMode == MODE_VELOCITY) {
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    /**
     * A 键切换 shooter 被控量；切换时停车并切换到对应 RunMode。
     */
    private void handleShooterModeSwitch() {
        if (gamepad1.aWasPressed()) {
            shooterMode = (shooterMode + 1) % MODE_COUNT;
            targetShooterPower = 0;
            targetShooterVelocity = 0;
            shooter.setPower(0);
            if (shooterMode == MODE_VELOCITY) {
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    /** 按 intake 当前模式应用方向键步进量 */
    private void applyIntakeStep(int step) {
        if (step == 0) return;
        if (intakeMode == MODE_POWER) {
            targetIntakePower = clamp(targetIntakePower + step * powerStep, -1.0, 1.0);
        } else {
            targetIntakeVelocity = clamp(targetIntakeVelocity + step * velocityStep,
                    -maxVelocity, maxVelocity);
        }
    }

    /** 按 shooter 当前模式应用方向键步进量 */
    private void applyShooterStep(int step) {
        if (step == 0) return;
        if (shooterMode == MODE_POWER) {
            targetShooterPower = clamp(targetShooterPower + step * powerStep, -1.0, 1.0);
        } else {
            targetShooterVelocity = clamp(targetShooterVelocity + step * velocityStep,
                    -maxVelocity, maxVelocity);
        }
    }

    /** Y 键急停：两个电机目标归零、功率清零（舵机保持当前位置） */
    private void emergencyStop() {
        targetIntakePower = 0;
        targetIntakeVelocity = 0;
        targetShooterPower = 0;
        targetShooterVelocity = 0;
        intake.setPower(0);
        shooter.setPower(0);
    }

    /** 驱动 intake：POWER 直通 setPower，VELOCITY 走固件速度闭环 */
    private void runIntake() {
        if (intakeMode == MODE_POWER) {
            intake.setPower(targetIntakePower);
        } else {
            updateIntakePidf();
            intake.setVelocity(targetIntakeVelocity);
        }
    }

    /** 驱动 shooter：POWER 直通 setPower，VELOCITY 走固件速度闭环 */
    private void runShooter() {
        if (shooterMode == MODE_POWER) {
            shooter.setPower(targetShooterPower);
        } else {
            updateShooterPidf();
            shooter.setVelocity(targetShooterVelocity);
        }
    }

    /** 速度模式下，把 Dashboard 上的 intake PIDF 系数写回 Hub（仅变化时写入） */
    private void updateIntakePidf() {
        if (intakeP != appliedIntakeP || intakeI != appliedIntakeI
                || intakeD != appliedIntakeD || intakeF != appliedIntakeF) {
            intake.setVelocityPIDFCoefficients(intakeP, intakeI, intakeD, intakeF);
            appliedIntakeP = intakeP;
            appliedIntakeI = intakeI;
            appliedIntakeD = intakeD;
            appliedIntakeF = intakeF;
        }
    }

    /** 速度模式下，把 Dashboard 上的 shooter PIDF 系数写回 Hub（仅变化时写入） */
    private void updateShooterPidf() {
        if (shooterP != appliedShooterP || shooterI != appliedShooterI
                || shooterD != appliedShooterD || shooterF != appliedShooterF) {
            shooter.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            appliedShooterP = shooterP;
            appliedShooterI = shooterI;
            appliedShooterD = shooterD;
            appliedShooterF = shooterF;
        }
    }

    // ==================== 遥测 ====================

    private void setTelemetry() {
        telemetry.addData("dt (ms)", "%.1f", dt * 1000);
        telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());

        // 两个电机恒定显示功率与转速
        telemetry.addData("Intake mode", "%s  (X)", MODE_NAMES[intakeMode]);
        telemetry.addData("Intake power", "%.4f", intake.getPower());
        telemetry.addData("Intake velocity (tick/s)", "%.0f", intake.getVelocity());
        if (intakeMode == MODE_POWER) {
            telemetry.addData("Intake target power", "%.3f", targetIntakePower);
        } else {
            telemetry.addData("Intake target (tick/s)", "%.0f", targetIntakeVelocity);
            telemetry.addData("Intake error (tick/s)", "%.0f",
                    targetIntakeVelocity - intake.getVelocity());
        }

        telemetry.addData("Shooter mode", "%s  (A)", MODE_NAMES[shooterMode]);
        telemetry.addData("Shooter power", "%.4f", shooter.getPower());
        telemetry.addData("Shooter velocity (tick/s)", "%.0f", shooter.getVelocity());
        if (shooterMode == MODE_POWER) {
            telemetry.addData("Shooter target power", "%.3f", targetShooterPower);
        } else {
            telemetry.addData("Shooter target (tick/s)", "%.0f", targetShooterVelocity);
            telemetry.addData("Shooter error (tick/s)", "%.0f",
                    targetShooterVelocity - shooter.getVelocity());
        }

        telemetry.addData("Servo pos", "%.3f  (B: A/B)", targetServoPos);
        telemetry.addData("Servo A / B", "%.2f / %.2f", servoPosA, servoPosB);
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
