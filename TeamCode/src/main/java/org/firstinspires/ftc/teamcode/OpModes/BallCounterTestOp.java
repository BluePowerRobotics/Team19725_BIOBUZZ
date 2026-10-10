package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Processors.Sensors.BallCounter;
import org.firstinspires.ftc.teamcode.Processors.Sensors.PressureSwitcher;

/**
 * BallCounter 小球计数器测试程序。
 *
 * 每帧调用 counter.update() 采样并做上升沿检测，实时显示：
 *   Ball Count       累计计数的小球个数
 *   New Ball         本帧是否检测到新小球（上升沿，1 = 是）
 *   Pressed          按压判定结果（1 = 压着，0 = 未压着）
 *   Raw Voltage      归一化原始电压（未滤波）
 *   Filtered Voltage 归一化滤波电压（EMA 输出）
 *   Threshold        当前归一化阈值
 *   Alpha            当前 EMA 平滑系数
 *
 * 调参方式：FTC Dashboard → Config → BallCounterTestOp
 *   threshold     归一化阈值 [0, 1]，超过该值的滤波电压判定为按压
 *   alpha         EMA 平滑系数 (0, 1]，越接近 1 响应越快、抗噪越差
 *   resetCounter  置 true 会清零一次计数并重置滤波器（仅上升沿生效）
 *
 * 验证步骤：
 *   1. 让小球逐个滚过传感器，每个球应只使 Ball Count +1；
 *   2. 让小球停在传感器上不动，Ball Count 应只 +1 且之后不再增加，
 *      Pressed 保持为 1，New Ball 仅在压上的那一帧为 1；
 *   3. 球滚走后（Pressed 回到 0）再压下一个球，应再次 +1。
 *
 * 注意：BallCounter.update() 内部已调用 PressureSwitcher.update()，
 *      本程序中不要再单独调用 pressure.update()，否则滤波器每帧会被推进两次。
 *
 * 校准建议见 Processors/Sensors/PressurSwitcher.md：
 *   先读取典型压力下的 Filtered Voltage，再把 threshold 调到略低于该值。
 */
@Config
@TeleOp(name = "BallCounterTestOp", group = "Tests")
public class BallCounterTestOp extends LinearOpMode {

    /** 模拟输入在机器人配置中的设备名（硬件参数，改动需重新 init） */
    private static final String DEVICE_NAME = "pressure_sensor";


    // ================= FTC Dashboard 可调参数 =================

    /** 归一化压力阈值 [0, 1] */
    public static double threshold = 0.5;

    /** EMA 平滑系数 (0, 1] */
    public static double alpha = 0.8;

    /** 置 true 时清零一次计数并重置滤波器 */
    public static boolean resetCounter = false;


    /** 上一次的 resetCounter 值，用于检测上升沿 */
    private boolean lastResetCounter = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PressureSwitcher pressure = new PressureSwitcher(hardwareMap, DEVICE_NAME);
        BallCounter counter = new BallCounter(pressure);

        telemetry.addData("Device", DEVICE_NAME);
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Tune threshold/alpha/resetCounter in FTC Dashboard > Config");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 采样 + EMA 滤波 + 上升沿检测计数（必须每帧调用）
            boolean newBall = counter.update();

            // 上升沿清零计数（同时重置底层滤波器）
            if (resetCounter && !lastResetCounter) {
                counter.reset();
            }
            lastResetCounter = resetCounter;

            // 每帧应用 Dashboard 参数（带范围保护，避免非法值抛异常）
            pressure.setThreshold(clamp01(threshold));               // threshold ∈ [0, 1]
            pressure.setAlpha(Math.max(0.01, Math.min(1.0, alpha))); // alpha ∈ (0, 1]

            // ---- 遥测 ----
            telemetry.addData("Ball Count", counter.getCount());
            telemetry.addData("New Ball", newBall ? 1 : 0);
            telemetry.addData("Pressed", pressure.isPressed() ? 1 : 0);
            telemetry.addData("Raw Voltage", "%.3f", pressure.getRawVoltage());
            telemetry.addData("Filtered Voltage", "%.3f", pressure.getVoltage());
            telemetry.addData("Threshold", "%.3f", pressure.getThreshold());
            telemetry.addData("Alpha", "%.3f", pressure.getAlpha());
            telemetry.update();
        }
    }

    /** 把数值限制到 [0, 1] */
    private static double clamp01(double value) {
        if (value < 0) {
            return 0;
        }
        if (value > 1) {
            return 1;
        }
        return value;
    }
}
