package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.filter.EMA;

/**
 * 对射光电传感器（断裂光束 / Break Beam）：把"模拟输出"封装成一个实时布尔开关量。
 *
 * 电路说明：
 *   对射光电（发射端 + 接收端）输出的模拟信号接到 Hub 的 Analog 口。
 *   未遮挡时输出高电压，遮挡时输出低电压（与压敏开关"压力越大电压越高"相反）。
 *   本类用 EMA 对归一化电压滤波，当滤波电压低于阈值时输出 blocked = true。
 *
 * 归一化约定：
 *   内部所有电压与阈值一律除以满量程电压 VOLTAGE_REFERENCE (3.3V)，
 *   归一化到 [0, 1] 区间。因此 getRawVoltage()/getVoltage()/getThreshold()/
 *   setThreshold() 等接口返回/接收的都是归一化数值。
 *
 * 层级约定（严格单级调用）：
 *   BreakBeamSensor 是叶子，不创建 / 不 update 任何下属。
 *   上级（BallCounter 等）只通过 update() 的返回值拿到本帧采样结果，
 *   本类不暴露模拟输入设备引用，也不主动向上调用任何方法。
 */
public class BreakBeamSensor {

    /**
     * 本帧采样的完整结果包，由 {@link #update()} 返回给上级。
     * 上级拿到这一个对象就能读到所有需要的数据，不需要再调本类的其他 getter。
     */
    public static class BeamSample {
        /** 本帧是否遮挡（滤波电压 < 阈值） */
        public final boolean blocked;
        /** 归一化原始电压（未滤波），范围 [0, 1] */
        public final double rawVoltage;
        /** 归一化滤波电压（EMA 输出），范围 [0, 1] */
        public final double filteredVoltage;

        public BeamSample(boolean blocked, double rawVoltage, double filteredVoltage) {
            this.blocked = blocked;
            this.rawVoltage = rawVoltage;
            this.filteredVoltage = filteredVoltage;
        }
    }

    /** 模拟输入满量程电压（V），所有电压/阈值的归一化基准 */
    public static final double VOLTAGE_REFERENCE = 3.3;

    /** 默认 EMA 平滑系数（越小越平滑、响应越慢） */
    private static final double DEFAULT_ALPHA = 0.8;
    /** 默认归一化遮挡阈值（滤波电压低于它即判定为遮挡） */
    private static final double DEFAULT_THRESHOLD = 0.5;

    private final AnalogInput analogInput;
    private final EMA ema;

    /** 归一化遮挡阈值 [0,1]，滤波电压低于它即判定为遮挡 */
    private double threshold;

    /** 最近一次 update() 采到的归一化原始电压（未滤波） */
    private double rawVoltage;
    /** 最近一次 update() 的归一化滤波电压（EMA 输出） */
    private double filteredVoltage;
    /** 最近一次 update() 的遮挡判定结果 */
    private boolean blocked;

    /**
     * 使用默认阈值与平滑系数创建对射光电传感器
     *
     * @param hardwareMap 硬件映射表
     * @param deviceName  模拟输入在配置中的设备名
     */
    public BreakBeamSensor(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap.get(AnalogInput.class, deviceName),
                DEFAULT_ALPHA, DEFAULT_THRESHOLD);
    }

    /**
     * 指定阈值与平滑系数创建对射光电传感器
     *
     * @param hardwareMap 硬件映射表
     * @param deviceName  模拟输入在配置中的设备名
     * @param alpha       EMA 平滑系数，范围 (0, 1]
     * @param threshold   归一化遮挡阈值，范围 [0, 1]
     */
    public BreakBeamSensor(HardwareMap hardwareMap, String deviceName,
                           double alpha, double threshold) {
        this(hardwareMap.get(AnalogInput.class, deviceName), alpha, threshold);
    }

    /**
     * @param analogInput 已获取的模拟输入设备
     * @param alpha       EMA 平滑系数，范围 (0, 1]
     * @param threshold   归一化遮挡阈值，范围 [0, 1]
     */
    public BreakBeamSensor(AnalogInput analogInput, double alpha, double threshold) {
        if (threshold < 0 || threshold > 1) {
            throw new IllegalArgumentException("threshold must be in range [0, 1]");
        }
        this.analogInput = analogInput;
        this.ema = new EMA(alpha);
        this.threshold = threshold;
        this.rawVoltage = 0.0;
        this.filteredVoltage = 0.0;
        this.blocked = false;
    }

    /**
     * 采样一次模拟输入：读取归一化原始电压 → EMA 滤波 → 更新遮挡判定。
     * 每帧只应被上级（BallCounter）调用一次，推进一次 EMA。
     *
     * <p>本方法是上级从本类取数的<strong>唯一通道</strong>：返回的 {@link BeamSample}
     * 包含本帧完整采样结果（遮挡判定 + 原始电压 + 滤波电压），上级拿到这一个对象
     * 后，本帧内所有需要的数据都应该从 sample 里读，不应再调本类的其他 getter
     * （那些 getter 返回的是同一帧的缓存值，只是为了兼容旧调用保留）。
     *
     * <p>注意：光电门"未遮挡 = 高电压、遮挡 = 低电压"，与压敏开关相反，
     *      因此用滤波电压"低于"阈值判定遮挡（压敏开关用的是高于阈值）。
     *
     * @return 本帧采样结果包（包含 blocked / rawVoltage / filteredVoltage），
     *         上级 BallCounter 用它做上升沿检测和标定透传
     */
    public BeamSample update() {
        double voltage = analogInput.getVoltage();
        // 无效读数（NaN / 负值）不按"低电压 = 遮挡"处理：保持上次判定并停止推进滤波，
        // 避免断线或坏读数被误判为遮挡而产生虚假上升沿（虚假计数）
        if (Double.isNaN(voltage) || voltage < 0) {
            return new BeamSample(blocked, rawVoltage, filteredVoltage);
        }
        rawVoltage = normalize(voltage);
        filteredVoltage = ema.update(rawVoltage);
        blocked = filteredVoltage < threshold;
        return new BeamSample(blocked, rawVoltage, filteredVoltage);
    }

    /**
     * 当前是否遮挡（滤波电压低于阈值），由最近一次 update() 计算得到
     */
    public boolean isBlocked() {
        return blocked;
    }

    /**
     * 归一化原始电压（未滤波），来自最近一次 update()，范围 [0,1]
     */
    public double getRawVoltage() {
        return rawVoltage;
    }

    /**
     * 归一化滤波电压（EMA 输出），来自最近一次 update()，范围 [0,1]
     */
    public double getVoltage() {
        return filteredVoltage;
    }

    /**
     * 调节归一化遮挡阈值
     *
     * @param threshold 归一化阈值，范围 [0, 1]
     */
    public void setThreshold(double threshold) {
        if (threshold < 0 || threshold > 1) {
            throw new IllegalArgumentException("threshold must be in range [0, 1]");
        }
        this.threshold = threshold;
    }

    /**
     * 获取当前归一化遮挡阈值
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * 调节 EMA 平滑系数，实时生效
     *
     * @param alpha 平滑系数，范围 (0, 1]
     */
    public void setAlpha(double alpha) {
        ema.setAlpha(alpha);
    }

    /**
     * 获取当前 EMA 平滑系数
     */
    public double getAlpha() {
        return ema.getAlpha();
    }

    /**
     * 重置滤波器状态（阈值 / alpha 与最近一次判定结果保持不变）。
     * <b>不读取硬件</b>：下一帧 {@link #update()} 会用真实采样重新建立滤波状态，
     * 因此调用方不会因重置而多一次硬件访问（见 FTC_Format.md §6）。
     */
    public void reset() {
        ema.reset();
    }

    /**
     * 把模拟输入电压(V)归一化到 [0,1] 并做上限钳位。
     * 调用方（update）已保证传入的是有效非负读数。
     */
    private static double normalize(double voltageVolts) {
        return Math.min(voltageVolts / VOLTAGE_REFERENCE, 1.0);
    }
}
