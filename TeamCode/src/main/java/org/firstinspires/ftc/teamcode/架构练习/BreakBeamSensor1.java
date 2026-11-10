package org.firstinspires.ftc.teamcode.架构练习;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Processors.Sensors.BreakBeamSensor;

/**
 * 对射光电门（架构练习版）—— 对应架构图中最底层的「光电门」节点。
 *
 * <p>本类直接继承 {@link BreakBeamSensor}，复用其「模拟口采样 + EMA 滤波 + 阈值判定」的完整
 * 实现，并额外提供：
 * <ul>
 *   <li>设备名常量 {@link #DEVICE_NAME}；</li>
 *   <li>Dashboard 实时调参字段 {@link #beamThreshold} / {@link #beamAlpha}，每帧 {@link #update()}
 *       前同步到父类，实现现场在线调阈值 / 平滑系数。</li>
 * </ul>
 *
 * <p><b>架构图中的函数对应</b>：{@code IsBlocked} → {@link #isBlocked()}（继承自父类）。
 *
 * <p><b>依赖方向</b>：本层是叶子节点，不依赖其它「1」文件，只依赖硬件 AnalogInput。
 */
@Config
public class BreakBeamSensor1 extends BreakBeamSensor {

    /** 光电门模拟输入在 Robot Configuration 中的默认设备名 */
    public static final String DEVICE_NAME = "breakBeam";

    /** 归一化遮挡阈值 [0,1]，滤波电压低于它即判定遮挡（Dashboard 实时生效） */
    public static double beamThreshold = 0.5;

    /** EMA 平滑系数 (0,1]，越大响应越快（Dashboard 实时生效） */
    public static double beamAlpha = 0.8;

    /**
     * 使用默认设备名创建光电门。
     *
     * @param hardwareMap 硬件映射
     */
    public BreakBeamSensor1(HardwareMap hardwareMap) {
        super(hardwareMap, DEVICE_NAME);
    }

    /**
     * 指定设备名创建光电门。
     *
     * @param hardwareMap 硬件映射
     * @param deviceName  模拟输入在配置中的设备名
     */
    public BreakBeamSensor1(HardwareMap hardwareMap, String deviceName) {
        super(hardwareMap, deviceName);
    }

    /**
     * 每帧采样前，先把 Dashboard 的阈值 / 平滑系数同步到父类，再推进采样。
     *
     * @return 本次采样后是否遮挡
     */
    @Override
    public boolean update() {
        setThreshold(clamp01(beamThreshold));
        setAlpha(clampAlpha(beamAlpha));
        return super.update();
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
