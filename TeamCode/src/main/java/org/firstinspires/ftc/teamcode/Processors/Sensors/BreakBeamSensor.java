package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.filter.EMA;

/**
 * 对射光电门（自包含）—— 对应架构图中最底层的「光电门」节点。
 *
 * <p>本类自包含实现「模拟口采样 + EMA 滤波 + 阈值判定」，不依赖工程内其它传感器类，
 * 唯一外部依赖是 {@link EMA} 滤波器（{@code utility/filter}，属通用工具）。
 *
 * <p><b>约定</b>：未遮挡 = 高电压，遮挡 = 低电压，滤波电压低于阈值时 {@code blocked = true}。
 *
 * <p><b>架构图中的函数对应</b>：{@code IsBlocked} → {@link #isBlocked()}。
 */
@Config
public class BreakBeamSensor {

    /** 模拟输入满量程电压（V），所有电压/阈值的归一化基准 */
    public static final double VOLTAGE_REFERENCE = 3.3;

    /** 光电门模拟输入在 Robot Configuration 中的默认设备名 */
    public static final String DEVICE_NAME = "breakBeam";

    /** 归一化遮挡阈值 [0,1]，滤波电压低于它即判定遮挡（Dashboard 实时生效） */
    public static double beamThreshold = 0.5;

    /** 归一化遮挡阈值的出厂默认值（构造时使用） */
    private static final double DEFAULT_THRESHOLD = 0.5;

    /** EMA 平滑系数 (0,1]，越大响应越快（Dashboard 实时生效） */
    public static double beamAlpha = 0.8;

    private final AnalogInput analogInput;
    private final EMA ema;

    /** 归一化遮挡阈值 [0,1] */
    private double threshold;
    /** 最近一次 update() 采到的归一化原始电压（未滤波） */
    private double rawVoltage;
    /** 最近一次 update() 的归一化滤波电压（EMA 输出） */
    private double filteredVoltage;
    /** 最近一次 update() 的遮挡判定结果 */
    private boolean blocked;

    /**
     * 使用默认设备名创建光电门。
     */
    public BreakBeamSensor(HardwareMap hardwareMap) {
        this(hardwareMap, DEVICE_NAME);
    }

    /**
     * 指定设备名创建光电门。
     */
    public BreakBeamSensor(HardwareMap hardwareMap, String deviceName) {
        this.analogInput = hardwareMap.get(AnalogInput.class, deviceName);
        // 用钳制后的 alpha 构造，避免 Dashboard 把静态值改出 (0,1] 时初始化即抛异常
        this.ema = new EMA(clampAlpha(beamAlpha));
        this.threshold = DEFAULT_THRESHOLD;
        this.rawVoltage = 0.0;
        this.filteredVoltage = 0.0;
        this.blocked = false;
    }

    /**
     * 采样一次：读原始电压 → EMA 滤波 → 更新遮挡判定。应在主循环每帧调用。
     *
     * @return 本次采样后是否遮挡（滤波电压低于阈值）
     */
    public boolean update() {
        // 每帧同步 Dashboard 参数，改动下一帧生效
        setThreshold(clamp01(beamThreshold));
        setAlpha(clampAlpha(beamAlpha));

        double voltage = analogInput.getVoltage();
        // 无效读数（NaN / 负值）不按「低电压 = 遮挡」处理：保持上次判定，避免断线误判产生虚假上升沿
        if (Double.isNaN(voltage) || voltage < 0) {
            return blocked;
        }
        rawVoltage = normalize(voltage);
        filteredVoltage = ema.update(rawVoltage);
        blocked = filteredVoltage < threshold;
        return blocked;
    }

    /** @return IsBlocked：当前是否遮挡（由最近一次 update() 计算得到） */
    public boolean isBlocked() {
        return blocked;
    }

    /** @return 归一化原始电压（未滤波），范围 [0,1] */
    public double getRawVoltage() {
        return rawVoltage;
    }

    /** @return 归一化滤波电压（EMA 输出），范围 [0,1] */
    public double getVoltage() {
        return filteredVoltage;
    }

    /** 设置归一化遮挡阈值 [0,1] */
    public void setThreshold(double threshold) {
        if (threshold < 0 || threshold > 1) {
            throw new IllegalArgumentException("threshold must be in range [0, 1]");
        }
        this.threshold = threshold;
    }

    /** @return 当前归一化遮挡阈值 */
    public double getThreshold() {
        return threshold;
    }

    /** 设置 EMA 平滑系数 (0,1]，实时生效 */
    public void setAlpha(double alpha) {
        ema.setAlpha(alpha);
    }

    /** @return 当前 EMA 平滑系数 */
    public double getAlpha() {
        return ema.getAlpha();
    }

    /** 重置滤波器状态（阈值 / alpha 与最近一次判定结果保持不变） */
    public void reset() {
        ema.reset();
    }

    private static double normalize(double voltageVolts) {
        return Math.min(voltageVolts / VOLTAGE_REFERENCE, 1.0);
    }

    private static double clamp01(double v) {
        if (v < 0) return 0;
        if (v > 1) return 1;
        return v;
    }

    private static double clampAlpha(double v) {
        return Math.max(0.01, Math.min(1.0, v));
    }
}
