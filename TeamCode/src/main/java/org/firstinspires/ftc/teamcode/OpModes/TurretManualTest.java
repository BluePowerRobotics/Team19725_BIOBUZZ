package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.utility.DpadStepper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Controllers.Turret.YawController;
import org.firstinspires.ftc.teamcode.Processors.Sensors.BreakBeamSensor;

/**
 * 炮台手动控制台：用二操完整驱动 {@link Turret}（平转 + 两个发射器），
 * 用于整体联调与手感确认。
 *
 * <p><b>手柄映射（二操）</b>
 * <ul>
 *   <li>左摇杆 左右 → yaw 平转（映射到 {@code [-yawRange, +yawRange]}；
 *       约定"推右 → 顺时针（负 yaw）"）</li>
 *   <li>小球 / 大球仰角舵机位置 → 由 Dashboard 的 {@link #targetPitchSmall} /
 *       {@link #targetPitchBig} 直接设置，手柄不控制仰角</li>
 *   <li>方向键 上 / 下 → 大球飞轮目标转速 ±{@link #speedStep}，范围 0 至
 *       {@code Shooter.BIG_PARAMS.speedMax}</li>
 *   <li>方向键 右 / 左 → 小球飞轮目标转速 ±{@link #speedStep}，范围 0 至
 *       {@code Shooter.SMALL_PARAMS.speedMax}</li>
 *   <li>左 bumper → 小球扳机（按住释放）</li>
 *   <li>右 bumper → 大球扳机（按住释放）</li>
 *   <li>右扳机 RT → intake 电机功率（0~1，直通 {@code setPower}，不做闭环）</li>
 * </ul>
 * 方向键为"按下即走一步，长按每 {@link #repeatPeriodMs} 毫秒再走一步"。
 *
 * <p><b>光电部分（与 {@code IntakeTest} 相同）</b>
 * 每帧调用 {@link BreakBeamSensor#update()}，实时显示原始电压 / 滤波电压 /
 * 阈值 / 遮挡判定 / alpha；threshold、alpha、resetFilter 在 Dashboard 可调。
 * 注意光电门"未遮挡 = 高电压、遮挡 = 低电压"，滤波电压<b>低于</b>阈值判定为遮挡。
 *
 * <p>遥测中的 {@code Fire ready big/small} 为 {@link Turret#update} 的返回值，
 * 即对应发射器当前是否具备发射条件（飞轮转速 + 俯仰 + 平转均到位）。
 */
@Config
@TeleOp(name = "TurretManualTest", group = "Tests")
public class TurretManualTest extends LinearOpMode {

    /** 每次按键的转速增量 (tick/s) */
    public static int speedStep = 100;
    /** 长按方向键的连续步进周期（毫秒） */
    public static int repeatPeriodMs = 120;

    /** 大球仰角舵机目标位置 [0,1]，由 Dashboard 直接设置 */
    public static double targetPitchBig = 0.5;
    /** 小球仰角舵机目标位置 [0,1]，由 Dashboard 直接设置 */
    public static double targetPitchSmall = 0.5;

    /** intake 电机在 Robot Configuration 中的名称（改动后需重新 Init） */
    public static String intakeMotorName = "sweeperMotor";

    /** 光电门模拟输入在 Robot Configuration 中的名称 */
    private static final String BEAM_DEVICE = "breakBeam";

    // ==================== 光电可调参数（FTC Dashboard 实时生效） ====================

    /** 归一化遮挡阈值 [0, 1]，滤波电压低于它即判定为遮挡 */
    public static double threshold = 0.5;
    /** EMA 平滑系数 (0, 1] */
    public static double alpha = 0.8;
    /** 置 true 时重置一次滤波器状态（仅上升沿生效） */
    public static boolean resetFilter = false;

    private Turret turret;

    /** intake 电机：由二操 RT 直通功率控制 */
    private DcMotorEx intakeMotor;

    private double targetYaw = 0;
    private int targetSpeedBig = 0;
    private int targetSpeedSmall = 0;

    private final DpadStepper vertical = new DpadStepper();
    private final DpadStepper horizontal = new DpadStepper();

    private BreakBeamSensor beam;
    /** 上一次的 resetFilter 值，用于检测上升沿 */
    private boolean lastResetFilter = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(hardwareMap, telemetry);
        turret.stop();

        // ---- intake 电机：功率直通（不做闭环，参照 IntakeTest 的 POWER 模式） ----
        intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        // ---- 光电初始化 ----
        beam = new BreakBeamSensor(hardwareMap, BEAM_DEVICE);

        telemetry.addLine("GP2 LstickX: yaw | pitch from Dashboard (targetPitchBig/Small)");
        telemetry.addLine("GP2 Dpad U/D: big speed | Dpad L/R: small speed");
        telemetry.addLine("GP2 LB: small trigger | RB: big trigger");
        telemetry.addLine("GP2 RT: intake power");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---- 摇杆：仰角改由 Dashboard 设置，手柄只控制 yaw ----
            double pitchSmall = targetPitchSmall;
            double pitchBig = targetPitchBig;
            // 推右（axis = +1）→ 顺时针，对应负 yaw
            targetYaw = -gamepad2.left_stick_x * YawController.yawRange;

            // ---- 方向键步进飞轮目标转速 ----
            int v = vertical.step(gamepad2.dpad_down, gamepad2.dpad_up, repeatPeriodMs);
            int h = horizontal.step(gamepad2.dpad_left, gamepad2.dpad_right, repeatPeriodMs);
            targetSpeedBig = clamp(targetSpeedBig + v * speedStep, 0, Shooter.BIG_PARAMS.flywheel.speedMax);
            targetSpeedSmall = clamp(targetSpeedSmall + h * speedStep, 0, Shooter.SMALL_PARAMS.flywheel.speedMax);

            // ---- 扳机：按住释放 ----
            boolean triggerBig = gamepad2.right_bumper;
            boolean triggerSmall = gamepad2.left_bumper;

            // ---- intake：RT 直通功率（0~1） ----
            intakeMotor.setPower(-gamepad2.right_trigger);

            // ---- 下发炮台目标 ----
            Turret.FireReady ready = turret.update(targetYaw, pitchBig, pitchSmall,
                    targetSpeedBig, targetSpeedSmall, triggerBig, triggerSmall);

            // ---- 光电门采样 + 滤波 + 遮挡判定 ----
            updateBeam();

            // ---- 遥测 ----
            turret.setTelemetry();
            telemetry.addLine("--- manual targets ---");
            telemetry.addData("Target yaw (deg)", "%.1f", Math.toDegrees(targetYaw));
            telemetry.addData("Target speed big (tick/s)", targetSpeedBig);
            telemetry.addData("Target speed small (tick/s)", targetSpeedSmall);
            telemetry.addData("Target pitch big", "%.3f", targetPitchBig);
            telemetry.addData("Target pitch small", "%.3f", targetPitchSmall);
            telemetry.addData("Fire ready big", ready.big);
            telemetry.addData("Fire ready small", ready.small);
            telemetry.addData("speedStep", speedStep);
            telemetry.addData("Intake power (RT)", "%.3f", intakeMotor.getPower());
            telemetry.addData("Intake velocity (tick/s)", "%.0f", intakeMotor.getVelocity());
            telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());

            // ---- 光电遥测 ----
            telemetry.addData("Beam raw", "%.3f", beam.getRawVoltage());
            telemetry.addData("Beam filtered", "%.3f", beam.getVoltage());
            telemetry.addData("Beam threshold", "%.3f", beam.getThreshold());
            telemetry.addData("Beam blocked", beam.isBlocked() ? 1 : 0);
            telemetry.addData("Beam alpha", "%.3f", beam.getAlpha());
            telemetry.update();
        }

        intakeMotor.setPower(0);
        turret.stop();
    }

    // ==================== 光电辅助 ====================

    /** 采样 + EMA 滤波 + 阈值判定，并应用 Dashboard 参数与上升沿复位 */
    private void updateBeam() {
        // 必须每帧调用
        beam.update();

        // 上升沿重置滤波器
        if (resetFilter && !lastResetFilter) {
            beam.reset();
        }
        lastResetFilter = resetFilter;

        // 每帧应用 Dashboard 参数（带范围保护，避免非法值抛异常）
        beam.setThreshold(clamp(threshold, 0.0, 1.0));      // threshold ∈ [0, 1]
        beam.setAlpha(clamp(alpha, 0.01, 1.0));             // alpha ∈ (0, 1]
    }

    /** 取各 Hub 电压最小值（近似电池电压） */
    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            min = Math.min(min, sensor.getVoltage());
        }
        return min;
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** 方向键步进器见 {@link DpadStepper} */
}
