package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Processors.Sensors.BreakBeamSensor;

/**
 * BreakBeamSensor 对射光电传感器（模拟信号接口）测试程序。
 *
 * 每帧调用 update() 采样，实时显示：
 *   Raw Voltage      归一化原始电压（未滤波，= 伏特 / 3.3）
 *   Filtered Voltage 归一化滤波电压（EMA 输出）
 *   Threshold        当前归一化阈值
 *   Blocked          遮挡判定结果（滤波电压低于阈值即为遮挡）
 *   Alpha            当前 EMA 平滑系数
 *
 * 调参方式：FTC Dashboard → Config → BreakBeamTestOp
 *   threshold       归一化阈值 [0, 1]，滤波电压低于该值即判定为遮挡
 *   alpha           EMA 平滑系数 (0, 1]，越接近 1 响应越快、抗噪越差
 *   resetFilter     置 true 会重置一次滤波器状态（仅上升沿生效）
 *
 * 校准建议：先在"有遮挡 / 无遮挡"两种情况下观察 Raw Voltage，
 *   确认未遮挡为高电压、遮挡为低电压；再按抖动情况调整 alpha
 *   （0.5 附近抗抖更强，0.9 附近响应更快）。
 */
@Config
@TeleOp(name = "BreakBeamTestOp", group = "Tests")
public class BreakBeamTestOp extends LinearOpMode {

    /** 模拟口在机器人配置中的设备名（硬件参数，改动需重新 init） */
    private static final String DEVICE_NAME = "breakBeam";


    // ================= FTC Dashboard 可调参数 =================

    /** 归一化遮挡阈值 [0, 1]，滤波电压低于它即判定为遮挡 */
    public static double threshold = 0.5;

    /** EMA 平滑系数 (0, 1] */
    public static double alpha = 0.8;

    /** 置 true 时重置一次滤波器状态 */
    public static boolean resetFilter = false;


    /** 上一次的 resetFilter 值，用于检测上升沿 */
    private boolean lastResetFilter = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BreakBeamSensor beam = new BreakBeamSensor(hardwareMap, DEVICE_NAME);

        telemetry.addData("Device", DEVICE_NAME);
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Tune threshold/alpha/resetFilter in FTC Dashboard > Config");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 采样 + EMA 滤波 + 阈值判定（必须每帧调用）
            beam.update();

            // 上升沿重置滤波器
            if (resetFilter && !lastResetFilter) {
                beam.reset();
            }
            lastResetFilter = resetFilter;

            // 每帧应用 Dashboard 参数（带范围保护，避免非法值抛异常）
            beam.setThreshold(clamp01(threshold));                  // threshold ∈ [0, 1]
            beam.setAlpha(Math.max(0.01, Math.min(1.0, alpha)));     // alpha ∈ (0, 1]

            // ---- 遥测 ----
            telemetry.addData("Raw Voltage", "%.3f", beam.getRawVoltage());
            telemetry.addData("Filtered Voltage", "%.3f", beam.getVoltage());
            telemetry.addData("Threshold", "%.3f", beam.getThreshold());
            telemetry.addData("Blocked", beam.isBlocked() ? 1 : 0);
            telemetry.addData("Alpha", "%.3f", beam.getAlpha());
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
