package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * 炮台<b>开环</b>标定台：直接以开环方式驱动 yaw 电机与 4 个舵机，用于标定超参。
 *
 * <p>本 OpMode 不经过 {@code YawController} / {@code Shooter} 的闭环，因此可以用来
 * 标定它们依赖的机械量：
 * <ul>
 *   <li>{@code YawController.tickPerRad}、{@code YawController.yawZeroTicks}</li>
 *   <li>{@code Shooter.Params} 的 {@code pitchMin / pitchMax}、
 *       {@code triggerOnPos / triggerOffPos}</li>
 * </ul>
 *
 * <p><b>手柄映射</b>
 * <ul>
 *   <li>一操 左摇杆 左右 → yaw 电机功率（开环；符号与大小直接映射，经 maxYawPower 限幅）</li>
 *   <li>一操 左摇杆 上下 → 大球俯仰舵机位置（上 → 1，下 → 0）</li>
 *   <li>一操 右摇杆 上下 → 小球俯仰舵机位置（上 → 1，下 → 0）</li>
 *   <li>二操 左摇杆 上下 → 大球扳机舵机位置（上 → 1，下 → 0）</li>
 *   <li>二操 右摇杆 上下 → 小球扳机舵机位置（上 → 1，下 → 0）</li>
 *   <li>一操 A 键 → yaw 电机编码器清零（作为 tick 标定基准）</li>
 * </ul>
 *
 * <p><b>标定步骤</b>
 * <ol>
 *   <li>tickPerRad：把炮台摆到机械 yaw = 0 处 → 按 A 清零 → 用手柄转到已知角度 Δ（度）
 *       → 读"累计 tick" → {@code tickPerRad = ticks / toRadians(Δ)}</li>
 *   <li>yawZeroTicks：把炮台摆到机械 yaw = 0 处，读"累计 tick"并填入该超参</li>
 *   <li>舵机范围：推杆扫过全行程，把机械限位处的读数填入
 *       {@code pitchMin / pitchMax / triggerOnPos / triggerOffPos}</li>
 * </ol>
 * 注意：舵机无位置反馈，遥测中的舵机位置是<b>下发的目标位置</b>而非实测角度。
 */
@Config
@TeleOp(name = "TurretTickTest", group = "Tests")
public class TurretTickTest extends LinearOpMode {

    // ---- 设备名（与 Turret.java 保持一致） ----
    private static final String YAW_MOTOR = "yawMotor";
    private static final String BIG_PITCH_SERVO = "pitchBig";
    private static final String SMALL_PITCH_SERVO = "pitchSmall";
    private static final String BIG_TRIGGER_SERVO = "triggerBig";
    private static final String SMALL_TRIGGER_SERVO = "triggerSmall";

    /** yaw 开环功率上限（安全限幅；标定时建议保持较小值） */
    public static double maxYawPower = 0.5;
    /** yaw 摇杆死区，避免回中漂移导致电机持续微动 */
    public static double stickDeadZone = 0.05;

    private DcMotorEx yawMotor;
    private Servo pitchBig;
    private Servo pitchSmall;
    private Servo triggerBig;
    private Servo triggerSmall;

    private boolean prevA = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        yawMotor = hardwareMap.get(DcMotorEx.class, YAW_MOTOR);
        // 开环：setPower 直接控制功率；编码器仍可读累计 tick
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitchBig = hardwareMap.get(Servo.class, BIG_PITCH_SERVO);
        pitchSmall = hardwareMap.get(Servo.class, SMALL_PITCH_SERVO);
        triggerBig = hardwareMap.get(Servo.class, BIG_TRIGGER_SERVO);
        triggerSmall = hardwareMap.get(Servo.class, SMALL_TRIGGER_SERVO);

        telemetry.addLine("GP1 LstickX: yaw power | LstickY: big pitch | RstickY: small pitch");
        telemetry.addLine("GP2 LstickY: big trigger | RstickY: small trigger");
        telemetry.addLine("GP1 A: zero yaw encoder");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---- A 键：编码器清零（标定基准） ----
            if (gamepad1.a && !prevA) {
                yawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            prevA = gamepad1.a;

            // ---- yaw：开环功率 ----
            double yawPower = clamp(deadZone(gamepad1.left_stick_x, stickDeadZone) * maxYawPower, -1, 1);
            yawMotor.setPower(yawPower);

            // ---- 4 个舵机：摇杆位置直接映射到 [0, 1] ----
            // 摇杆上推时 axis = -1（FTC 约定），取负号使"上 → 1"
            double bigPitchPos = positionFromAxis(-gamepad1.left_stick_y);
            double smallPitchPos = positionFromAxis(-gamepad1.right_stick_y);
            double bigTriggerPos = positionFromAxis(-gamepad2.left_stick_y);
            double smallTriggerPos = positionFromAxis(-gamepad2.right_stick_y);

            pitchBig.setPosition(bigPitchPos);
            pitchSmall.setPosition(smallPitchPos);
            triggerBig.setPosition(bigTriggerPos);
            triggerSmall.setPosition(smallTriggerPos);

            // ---- 遥测 ----
            telemetry.addData("Yaw ticks (accumulated)", yawMotor.getCurrentPosition());
            telemetry.addData("Yaw power", "%.3f", yawPower);
            telemetry.addLine("--- servo commanded positions (no feedback) ---");
            telemetry.addData("pitchBig", "%.3f", pitchBig.getPosition());
            telemetry.addData("pitchSmall", "%.3f", pitchSmall.getPosition());
            telemetry.addData("triggerBig", "%.3f", triggerBig.getPosition());
            telemetry.addData("triggerSmall", "%.3f", triggerSmall.getPosition());
            telemetry.update();
        }

        yawMotor.setPower(0);
    }

    /** 摇杆位置 [-1, 1] 线性映射到舵机位置 [0, 1] */
    private static double positionFromAxis(double axis) {
        return clamp((axis + 1) / 2, 0, 1);
    }

    private static double deadZone(double v, double zone) {
        return Math.abs(v) < zone ? 0 : v;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}