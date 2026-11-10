package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 小球计数器 —— 对应架构图中的「计数」节点。
 *
 * <p>基于对射光电门 {@link BreakBeamSensor} 的上升沿检测统计经过的小球个数，计数方向由外部
 * 每帧传入的 0/1 模式决定。本类提供架构图要求的两个能力：
 * <ul>
 *   <li>{@link #isBlocked()}：直读光电门遮挡状态（对应架构图 {@code IsBlocked}）；</li>
 *   <li>{@link #clearCount()}：只清计数、不动边缘基准（供 {@code resetBallCount} 使用），
 *       与 {@link #reset()} 的「深复位」区分开来。</li>
 * </ul>
 *
 * <p><b>计数规则</b>
 * <ul>
 *   <li>{@link #MODE_INTAKE} (0)：吸球，光电门每检测到一个上升沿，count +1</li>
 *   <li>{@link #MODE_OUTPUT} (1)：出球，光电门每检测到一个上升沿，count -1（不会减到负数）</li>
 * </ul>
 *
 * <p><b>为什么不会重复计数</b>：小球停在光电门中时遮挡持续为真，上升沿只在 0→1 的那一帧出现一次，
 * 之后不再触发；小球离开后回到 0，下一个小球挡光才会产生新上升沿。
 *
 * <p><b>Init / 每帧顺序</b>：本层在构造时先创建下属 {@link BreakBeamSensor}（不依赖他人），
 * 再初始化自身字段；每帧 {@link #update(int)} 内部调用 {@link BreakBeamSensor#update()} 推进硬件采样。
 */
public class BallCounter {

    /** 外部信息：进入模式（intake，球挡光计数 +1） */
    public static final int MODE_INTAKE = 0;
    /** 外部信息：出球模式（output，球挡光计数 -1） */
    public static final int MODE_OUTPUT = 1;

    /** 下属对象：对射光电门（先声明，构造时先创建） */
    private final BreakBeamSensor breakBeam;

    /** 上一帧的遮挡状态，用于与当前帧比较以检测上升沿 */
    private boolean lastBlocked;

    /** 累计计数的小球个数（始终 ≥ 0） */
    private int count;

    /**
     * @param hardwareMap 硬件映射
     */
    public BallCounter(HardwareMap hardwareMap) {
        // Init 顺序：先下属、后自身
        this(new BreakBeamSensor(hardwareMap));
    }

    /**
     * 复用外部已创建的光电门（便于测试程序对光电门单独调参）。
     *
     * @param breakBeam 已创建的对射光电门
     */
    public BallCounter(BreakBeamSensor breakBeam) {
        this.breakBeam = breakBeam;
        this.lastBlocked = false;
        this.count = 0;
    }

    /**
     * 采样一次并做上升沿检测计数。应在主循环每帧调用。
     *
     * @param externalState 外部信息：0 = 进入（intake），1 = 出球（output）
     * @return 本帧是否有球挡光（上升沿；无论计数 +1 还是 -1）
     */
    public boolean update(int externalState) {
        if (externalState != MODE_INTAKE && externalState != MODE_OUTPUT) {
            throw new IllegalArgumentException(
                    "externalState must be 0 (intake) or 1 (output), got: " + externalState);
        }

        boolean output = (externalState == MODE_OUTPUT);
        boolean blocked = breakBeam.update();

        boolean risingEdge = blocked && !lastBlocked;
        if (risingEdge) {
            if (output) {
                if (count > 0) {
                    count--;
                }
            } else {
                count++;
            }
        }

        lastBlocked = blocked;
        return risingEdge;
    }

    /** @return BallCount：当前累计计数的小球个数（始终 ≥ 0） */
    public int getCount() {
        return count;
    }

    /** @return IsBlocked：光电门当前是否遮挡（最近一次 {@link #update(int)} 的结果） */
    public boolean isBlocked() {
        return breakBeam.isBlocked();
    }

    /**
     * resetBallCount 用的轻量清零：只把计数归零，<b>不</b>重置光电门滤波与边缘基准。
     * 用于「炮台发射完毕」这类只需把数字归零的场合；需要连滤波状态一起复位时用 {@link #reset()}。
     */
    public void clearCount() {
        count = 0;
    }

    /**
     * 深复位：计数归零，同时重置光电门 EMA 滤波与边缘检测基准（不影响 threshold/alpha 配置）。
     * <b>不读取硬件</b>：以最近一帧 {@link BreakBeamSensor#isBlocked()} 的结果作为边缘检测基准，
     * 下一帧 update() 若物理状态未变就不会产生虚假上升沿。
     */
    public void reset() {
        count = 0;
        breakBeam.reset();
        lastBlocked = breakBeam.isBlocked();
    }
}
