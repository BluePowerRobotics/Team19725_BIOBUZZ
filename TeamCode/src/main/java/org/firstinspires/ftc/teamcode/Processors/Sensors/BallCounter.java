package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 小球计数器：基于对射光电门（BreakBeamSensor）的上升沿检测（边缘检测）
 * 统计经过的小球个数，计数方向由外部每帧传入的 0/1 信息决定。
 *
 * 两路 0/1 信号的区别（不要混淆）：
 *   1. 外部模式信息（update 的 externalState 参数）：
 *      0 = 进入模式（intake），1 = 出球模式（output）。它只决定"上升沿来了
 *      计数 +1 还是 -1"，不表示球有没有挡住光电门。
 *   2. 光电门信号（BreakBeamSensor 的 isBlocked()）：
 *      本项目光电门约定：0 = 遮挡，1 = 未遮挡（对应 BreakBeamSensor 的
 *      blockedOnState = false）。由数字电平经 EMA 滤波 + 阈值判定得到，
 *      用于检测上升沿（"球挡光"这一事件）。
 *
 * 计数规则：
 *   进入模式（外部信息 = 0，intake）：光电门每检测到一个上升沿，count +1
 *   出球模式（外部信息 = 1，output）：光电门每检测到一个上升沿，count -1
 *   count 不会减到负数（球已打空时再挡光，计数停在 0）
 *
 *   注意：发射（shooting）时需要清空球道内球的计数，该操作由上层程序负责，
 *   不在本类实现（上层可调用 reset() 清零）。
 *
 * 为什么不会重复计数：
 *   若小球正好停在光电门中不动，遮挡信号持续为 1（1, 1, 1, ...k,
 *   上升沿只在 0 → 1 的那一帧出现一次，之后不再触发，
 *   因此停留多久都只计 1 次。小球离开后信号回到 0，
 *   下一个小球挡光时才会产生新的上升沿。
 *
 * 去抖说明：
 *   光电门数字量的去抖已由 BreakBeamSensor 内部的 EMA 滤波 + 阈值判定完成
 *   （threshold / alpha / 极性可在 FTC Dashboard 调参），因此本类不再重复
 *   做去抖，只负责"上升沿检测 + 方向计数"。
 *
 * 用法（由上层 RobotPosition 创建，主循环中由 RobotPosition 每帧调用 update(externalState)）：
 *   层级：RobotPosition → BallCounter → BreakBeamSensor，
 *   光电门在 BallCounter 内部创建，外部不再单独 new / update 光电门。
 *   BallCounter counter = new BallCounter(hardwareMap, "breakBeam");
 *   while (opModeIsActive()) {
 *       int externalState = ...;                   // 0 = 进入(intake), 1 = 出球(output)
 *       counter.update(externalState);             // 采样 + 边缘检测计数
 *       telemetry.addData("balls", counter.getCount());  // 实时计数
 *   }
 */
public class BallCounter {

    /** 外部信息：进入模式（intake，球挡光计数 +1） */
    public static final int MODE_INTAKE = 0;
    /** 外部信息：出球模式（output，球挡光计数 -1） */
    public static final int MODE_OUTPUT = 1;

    private final BreakBeamSensor breakBeam;

    /** 上一帧的遮挡状态，用于与当前帧比较以检测跳变 */
    private boolean lastBlocked;

    /** 累计计数的小球个数（始终 ≥ 0） */
    private int count;

    /**
     * 创建球道计数器，并在内部创建其下属对射光电门（严格层级：
     * RobotPosition → BallCounter → BreakBeamSensor，光电门不对外暴露）。
     *
     * @param hardwareMap 硬件映射表
     * @param deviceName  对射光电门（AnalogInput）在机器人配置中的设备名
     */
    public BallCounter(HardwareMap hardwareMap, String deviceName) {
        this(new BreakBeamSensor(hardwareMap, deviceName));
    }

    /**
     * 创建球道计数器，并指定下属光电门的 EMA 平滑系数与遮挡阈值。
     * 与 {@code BallCounterTestOp} 中标定出的参数保持一致时使用。
     *
     * @param hardwareMap 硬件映射表
     * @param deviceName  对射光电门（AnalogInput）在机器人配置中的设备名
     * @param alpha       EMA 平滑系数，范围 (0, 1]
     * @param threshold   归一化遮挡阈值，范围 [0, 1]
     */
    public BallCounter(HardwareMap hardwareMap, String deviceName,
                       double alpha, double threshold) {
        this(new BreakBeamSensor(hardwareMap, deviceName, alpha, threshold));
    }

    /**
     * 内部构造：光电门实例由本类独占持有，仅允许本类自身创建，
     * 避免上层绕过 BallCounter 直接操作 BreakBeam。
     */
    private BallCounter(BreakBeamSensor breakBeam) {
        this.breakBeam = breakBeam;
        this.lastBlocked = false;
        this.count = 0;
    }

    /**
     * 采样一次并做上升沿检测计数。
     * 应在 OpMode 主循环中每帧调用，且每帧传入当前外部模式信息。
     *
     * @param externalState 外部信息：0 = 进入模式（intake），1 = 出球模式（output）
     * @return 本帧是否有球挡光（上升沿；无论计数 +1 还是 -1）
     */
    public boolean update(int externalState) {
        if (externalState != MODE_INTAKE && externalState != MODE_OUTPUT) {
            throw new IllegalArgumentException(
                    "externalState must be 0 (intake) or 1 (output), got: " + externalState);
        }

        boolean output = (externalState == MODE_OUTPUT);
        boolean blocked = breakBeam.update();

        // 上升沿：0 → 1 跳变。持续为 1（球停住不动）不会重复触发
        boolean risingEdge = blocked && !lastBlocked;
        if (risingEdge) {
            if (output) {
                if (count > 0) {   // 不允许减到负数
                    count--;
                }
            } else {
                count++;
            }
        }

        lastBlocked = blocked;
        return risingEdge;
    }

    /**
     * 实时获取当前累计计数的小球个数（始终 ≥ 0）
     */
    public int getCount() {
        return count;
    }

    /**
     * 清零计数并复位边缘检测状态（不影响光电门的 threshold/alpha 配置）。
     * <b>不读取硬件</b>：以最近一帧 {@link BreakBeamSensor#isBlocked()} 的结果作为
     * 边缘检测基准，下一帧 update() 若物理状态未变就不会产生虚假上升沿。
     */
    public void reset() {
        count = 0;
        breakBeam.reset();
        lastBlocked = breakBeam.isBlocked();
    }

    // ==================== 光电门状态透传（只读 / 调参，不推进 update）====================
    // 光电门由本类独占持有，上层需要查看原始电压、滤波电压或调阈值时通过本类访问，
    // 既保证层级（RobotPosition → BallCounter → BreakBeam），又便于标定调试。

    /** @return 本帧光电门是否遮挡（BreakBeamSensor 最近一次 update 的结果，不重复采样） */
    public boolean isBlocked() {
        return breakBeam.isBlocked();
    }

    /** @return 归一化原始电压（未滤波） */
    public double getRawVoltage() {
        return breakBeam.getRawVoltage();
    }

    /** @return 归一化滤波电压（EMA 输出） */
    public double getFilteredVoltage() {
        return breakBeam.getVoltage();
    }

    /** @return 当前归一化遮挡阈值 */
    public double getThreshold() {
        return breakBeam.getThreshold();
    }

    /** 设置归一化遮挡阈值，范围 [0, 1] */
    public void setThreshold(double threshold) {
        breakBeam.setThreshold(threshold);
    }

    /** @return 当前 EMA 平滑系数 */
    public double getAlpha() {
        return breakBeam.getAlpha();
    }

    /** 设置 EMA 平滑系数，范围 (0, 1] */
    public void setAlpha(double alpha) {
        breakBeam.setAlpha(alpha);
    }

    /** @return 下属光电门（只读访问，禁止在上层调用其 update()） */
    public BreakBeamSensor getBreakBeamSensor() {
        return breakBeam;
    }
}
