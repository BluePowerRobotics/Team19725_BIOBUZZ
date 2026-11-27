package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 球道计数器：读取对射光电门，按边缘记录球道内的小球个数。
 *
 * <p><b>职责边界（只做计数）</b>
 * <ul>
 *   <li>读取光电门：光电门挂在本类之下，由本类创建并每帧调用其 update()，
 *       电压采集、滤波与遮挡判定全部由 {@link BreakBeamSensor} 完成；</li>
 *   <li>计数：进球按上升沿 +1；出球按下降沿 -1（不小于 0），并带出球意图锁存；</li>
 *   <li>清零：{@link #reset()} 把计数重置为当前光路中的球数（0 或 1）并复位边缘检测基准。</li>
 * </ul>
 * 本类<b>不做</b>满球判定、不控制电机、不读手柄；计数方向由上层每帧传入，
 * 本类只对“出球意图”做短时锁存，避免松开吐球键后把在途的球误判为进球：
 * 容量判断（BallMax）在 {@code Sweeper} 中，发射完毕与操作手手动清空
 * 由 {@code Turret} / {@code OpMode} 调用 {@code RobotPosition.resetBallCount()} 完成。
 *
 * <p><b>为什么不会重复计数</b>：小球停在光电门中时遮挡持续为 true，
 * 边缘只在 false ↔ true 切换的那一帧出现一次，之后不再触发；
 * 进球只认上升沿、出球只认下降沿，每颗球在各自方向下只被计数一次。
 *
 * <p><b>非阻塞</b>：{@link #update()} 每帧 O(1)，无循环与等待。
 */
public class BallCounter {

    /** 进球模式：每个上升沿计数 +1 */
    public static final int MODE_INTAKE = 0;
    /** 出球模式：按下降沿计数 -1（不小于 0） */
    public static final int MODE_OUTPUT = 1;
    /** 空闲：本帧没有吸球/吐球请求，让出球意图锁存逐帧衰减（不改变当前方向） */
    public static final int MODE_IDLE = 2;

    /** 出球意图锁存帧数（约 0.8 s @50Hz）：覆盖球从滚轮到穿过光电门的时间 */
    private static final int OUTPUT_LATCH_FRAMES = 40;

    private final BreakBeamSensor breakBeam;

    /**
     * 是否已建立边缘检测基准。
     * 构造阶段<b>不读硬件</b>（初始化不属于更新链），基准由第一次 {@link #update()} 建立：
     * 该帧只记录当前遮挡状态、不计数，从而避免上电时已停在光路中的球被误计为新球。
     */
    private boolean hasBaseline;

    /** 上一帧的遮挡状态，用于与当前帧比较以检测上升沿 */
    private boolean lastBlocked;

    /** 球道内累计的小球个数（始终 ≥ 0，只能被 {@link #reset()} 清零） */
    private int count;

    /** 出球意图锁存剩余帧数：吐球键每帧续期，松开后仍保持，直到有球出口或超时 */
    private int outputLatchFrames;

    /**
     * 按设备名创建光电门（读取全部在本类内部完成，调用方无需接触传感器）。
     *
     * @param hardwareMap 硬件映射
     * @param deviceName  光电门模拟口在机器人配置中的名称
     */
    public BallCounter(HardwareMap hardwareMap, String deviceName) {
        this(new BreakBeamSensor(hardwareMap, deviceName));
    }

    /**
     * 使用已创建好的光电门，供需要读取原始电压 / 调整阈值的测试程序使用。
     *
     * @param breakBeam 已创建的对射光电门（本类内部负责调用其 update()）
     */
    public BallCounter(BreakBeamSensor breakBeam) {
        this.breakBeam = breakBeam;
        this.count = 0;
        // 初始化阶段只保存下级引用，不采样光电门；
        // 光电门 → BallCounter 的更新只发生在 update() 链内
        this.lastBlocked = false;
        this.hasBaseline = false;
    }

    /**
     * 每帧调用一次：采样光电门，出现边缘时按方向计数。
     *
     * <p>第一帧只建立边缘检测基准（不计数）。
     * <br>进球（{@link #MODE_INTAKE}）：按 false → true 的上升沿 +1。
     * <br>出球（{@link #MODE_OUTPUT}）：按 true → false 的下降沿 -1（不小于 0）。
     * 球离开球道时先遮挡、再离开光路，用下降沿可保证“进入时已 +1、仍停在光路中”的球
     * 离开时也能被减去，且每颗球只减一次（出球状态下的上升沿不计）。
     *
     * <p>出球意图锁存：吐球键只在按住的帧传 {@link #MODE_OUTPUT}；
     * 松开后传 {@link #MODE_IDLE} 让锁存逐帧衰减（约 0.8 s），
     * 因此球在穿过光电门前松开按键也不会被误判为进球；一旦有球出口立即结束锁存。
     * 显式吸球 {@link #MODE_INTAKE} 会立刻取消锁存。
     *
     * @param externalState 本帧的吸/吐状态（由 OpMode 每帧经 RobotPosition 传入）：
     *                      {@link #MODE_INTAKE} = 进球 +1，{@link #MODE_OUTPUT} = 出球 -1，
     *                      {@link #MODE_IDLE} = 本帧无请求（锁存逐帧衰减）
     * @return 本帧是否计到一颗球（进球或出球）
     */
    public boolean update(int externalState) {
        boolean blocked = breakBeam.update();
        if (!hasBaseline) {
            hasBaseline = true;
            lastBlocked = blocked;
            return false;
        }
        boolean risingEdge = blocked && !lastBlocked;
        boolean fallingEdge = !blocked && lastBlocked;

        // 出球意图锁存：按住吐球每帧续期；显式吸球立即取消；无请求则逐帧衰减
        if (externalState == MODE_OUTPUT) {
            outputLatchFrames = OUTPUT_LATCH_FRAMES;
        } else if (externalState == MODE_INTAKE) {
            outputLatchFrames = 0;
        } else if (outputLatchFrames > 0) {
            outputLatchFrames--;
        }

        boolean counted = false;
        if (outputLatchFrames > 0) {
            // 出球：下降沿 = 一颗球离开球道（含“进入时已计数、仍停在光路中”的球）
            if (fallingEdge && count > 0) {
                count--;
                counted = true;
                outputLatchFrames = 0;   // 出口事件已完成，结束本次锁存
            }
        } else if (risingEdge) {
            count++;
            counted = true;
        }
        lastBlocked = blocked;
        return counted;
    }

    /** @return 当前累计的小球个数（始终 ≥ 0） */
    public int getCount() {
        return count;
    }

    /**
     * 计数清零并复位边缘检测基准（阈值 / alpha 配置不受影响）。
     * <b>不读取硬件</b>：以最近一帧 {@link BreakBeamSensor#isBlocked()} 的结果作为
     * 边缘检测基准，下一帧 update() 若物理状态未变就不会产生虚假上升沿。
     *
     * <p><b>光路中仍有球时不清成 0</b>：该球已越过计数点、物理上仍在球道内，必须计入。
     * 若置 0，它会以 blocked = true 一路离开，全程不产生上升沿，从此永久漏计；
     * 表现为"实际已满、计数未满"，吃球器继续吸球直至溢出/堵球。
     */
    public void reset() {
        breakBeam.reset();
        lastBlocked = breakBeam.isBlocked();
        hasBaseline = true;
        count = lastBlocked ? 1 : 0;
    }
}
