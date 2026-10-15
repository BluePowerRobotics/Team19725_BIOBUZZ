package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.filter.EMA;

/**
 * 对射光电传感器（断裂光束 / Break Beam）：把数字输入封装成一个实时布尔开关量。
 *
 * 电路说明：
 *   对射光电（发射端 + 接收端）输出数字信号接到 Hub 的 Digital 口（配置为 INPUT）。
 *   光束被遮挡时输出电平翻转，但不同型号/接法可能"遮挡=高"或"遮挡=低"，
 *   故用 blockedOnState 指定"遮挡对应的原始数字电平"，默认 true（读到 1 视为遮挡）。
 *
 * 滤波说明：
 *   把原始数字量映射成 0/1 样本（1 = 遮挡、0 = 未遮挡），再用 EMA 滤波，
 *   滤波值可理解为"近段时间被遮挡的程度"，超过阈值的滤波值判定为遮挡。
 *   该过程同时起到去抖动作用：阈值 0.5、alpha 越小(响应越慢)抗抖时间越长。
 *
 * 归一化约定：
 *   getFilteredValue()/getThreshold()/setThreshold() 均为 [0, 1] 的归一化数值。
 *
 * 用法（OpMode 主循环中，每帧调用 update()）：
 *   BreakBeamSensor beam = new BreakBeamSensor(hardwareMap, "breakBeam");
 *   while (opModeIsActive()) {
 *       beam.update();                    // 采样 + 滤波 + 判定
 *       if (beam.isBlocked()) { ... }     // 实时布尔开关量
 *   }
 */
@Config
public class BreakBeamSensor {

    /** 默认 EMA 平滑系数（越小越平滑、响应越慢） */
    public static double DEFAULT_ALPHA = 0.8;
    /** 默认归一化遮挡阈值 */
    public static double DEFAULT_THRESHOLD = 0.5;
    /** 默认遮挡对应的原始数字电平 */
    public static boolean DEFAULT_BLOCKED_ON_STATE = true;

    private final DigitalChannel digitalChannel;
    private final EMA ema;

    /** 遮挡对应的原始数字电平（用于适配不同接法的传感器） */
    private boolean blockedOnState;

    /** 归一化遮挡阈值 [0,1]，滤波值超过它即判定为遮挡 */
    private double threshold;

    /** 最近一次 update() 采到的原始数字电平（未滤波） */
    private boolean rawState;
    /** 最近一次 update() 的遮挡判定结果 */
    private boolean blocked;

    /**
     * 使用默认阈值、平滑系数与极性创建对射光电传感器
     *
     * @param hardwareMap 硬件映射表
     * @param deviceName  数字口在配置中的设备名
     */
    public BreakBeamSensor(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap.get(DigitalChannel.class, deviceName),
                DEFAULT_ALPHA, DEFAULT_THRESHOLD, DEFAULT_BLOCKED_ON_STATE);
    }

    /**
     * 指定阈值、平滑系数与极性创建对射光电传感器
     *
     * @param hardwareMap   硬件映射表
     * @param deviceName    数字口在配置中的设备名
     * @param alpha         EMA 平滑系数，范围 (0, 1]
     * @param threshold     归一化遮挡阈值，范围 [0, 1]
     * @param blockedOnState 遮挡对应的原始数字电平
     */
    public BreakBeamSensor(HardwareMap hardwareMap, String deviceName,
                           double alpha, double threshold, boolean blockedOnState) {
        this(hardwareMap.get(DigitalChannel.class, deviceName),
                alpha, threshold, blockedOnState);
    }

    /**
     * @param digitalChannel 已获取的数字口设备
     * @param alpha          EMA 平滑系数，范围 (0, 1]
     * @param threshold      归一化遮挡阈值，范围 [0, 1]
     * @param blockedOnState 遮挡对应的原始数字电平
     */
    public BreakBeamSensor(DigitalChannel digitalChannel,
                           double alpha, double threshold, boolean blockedOnState) {
        checkThreshold(threshold);
        this.digitalChannel = digitalChannel;
        this.ema = new EMA(alpha);
        this.threshold = threshold;
        this.blockedOnState = blockedOnState;
        this.rawState = false;
        this.blocked = false;
        // 数字口配置为输入
        this.digitalChannel.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * 采样一次数字输入：读取原始电平 → 映射为 0/1 → EMA 滤波 → 更新遮挡判定。
     * 应在 OpMode 主循环中每帧调用。
     *
     * @return 本次采样后是否遮挡（滤波值超过阈值）
     */
    public boolean update() {
        rawState = digitalChannel.getState();
        double sample = (rawState == blockedOnState) ? 1.0 : 0.0;
        blocked = ema.update(sample) > threshold;
        return blocked;
    }

    /**
     * 当前是否遮挡，由最近一次 update() 计算得到
     */
    public boolean isBlocked() {
        return blocked;
    }

    /**
     * 归一化原始遮挡程度（未滤波），来自最近一次 update()，非 0 即 1
     */
    public double getRawValue() {
        return (rawState == blockedOnState) ? 1.0 : 0.0;
    }

    /**
     * 原始数字电平（未滤波、未做极性换算），来自最近一次 update()
     */
    public boolean getRawState() {
        return rawState;
    }

    /**
     * 归一化滤波遮挡程度（EMA 输出），来自最近一次 update()，范围 [0,1]
     */
    public double getFilteredValue() {
        return ema.getFilteredValue();
    }

    /**
     * 调节归一化遮挡阈值
     *
     * @param threshold 归一化阈值，范围 [0, 1]
     */
    public void setThreshold(double threshold) {
        checkThreshold(threshold);
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
     * 获取当前"遮挡对应的原始数字电平"
     */
    public boolean isBlockedOnState() {
        return blockedOnState;
    }

    /**
     * 设置"遮挡对应的原始数字电平"，用于适配不同接法的传感器
     *
     * @param blockedOnState 遮挡时读到的原始数字电平
     */
    public void setBlockedOnState(boolean blockedOnState) {
        this.blockedOnState = blockedOnState;
    }

    /**
     * 重置滤波器状态与判定结果（阈值/alpha/极性保持不变）
     */
    public void reset() {
        ema.reset();
        rawState = false;
        blocked = false;
    }

    private static void checkThreshold(double threshold) {
        if (threshold < 0 || threshold > 1) {
            throw new IllegalArgumentException("threshold must be in range [0, 1]");
        }
    }
}
