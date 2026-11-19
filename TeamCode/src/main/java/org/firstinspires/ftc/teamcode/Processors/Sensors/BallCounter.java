package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 球道计数器：读取对射光电门，按球流方向做双向边沿计数（吸球 +1，吐球 −1）。
 *
 * <p><b>职责边界（只做计数）</b>
 * <ul>
 *   <li>读取光电门：光电门挂在本类之下，由本类创建并每帧调用其 update()，
 *       电压采集、滤波与遮挡判定全部由 {@link BreakBeamSensor} 完成；</li>
 *   <li>计数：吸球方向每出现一个上升沿计数 +1；吐球方向每出现一个下降沿计数 −1（不低于 0）；</li>
 *   <li>清零：{@link #reset()} 把计数重置为当前光路中的球数（0 或 1）并复位边缘检测基准。</li>
 * </ul>
 * 本类<b>不做</b>满球判定、不控制电机、不读手柄：
 * 容量判断（BallMax）在 {@code Sweeper} 中，发射完毕与操作手手动清空
 * 由 {@code Turret} / {@code OpMode} 调用 {@code RobotPosition.resetBallCount()} 完成。
 *
 * <p><b>球流方向由上层给定，不由本类测量</b>：单颗对射光电门只能看到"被挡 / 未被挡"，
 * 无法分辨小球是进还是出。上层（{@code OpMode}）知道吃球器的指令方向，因此每帧把
 * {@code outflow} 作为更新输入传给 {@link #update(boolean)}；无参 {@link #update()}
 * 等价于吸球方向，保证不关心方向的调用点行为不变。
 *
 * <p><b>为什么不会重复计数</b>：小球停在光电门中时遮挡持续为 true，
 * 吸球方向的上升沿只在 false → true 的那一帧出现一次，之后不再触发；
 * 小球离开后信号回到 false，下一个小球挡光时才会产生新的上升沿。
 * 吐球方向同理只认 true → false 的下降沿：球从球道内向外排出时先挡光（该球已被计过），
 * 离开光路的那一帧才说明它真正排出球道。
 *
 * <p><b>非阻塞</b>：{@link #update()} 每帧 O(1)，无循环与等待。
 */
public class BallCounter {

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
     * 每帧调用一次（默认吸球方向）：等价于 {@link #update(boolean) update(false)}。
     *
     * @return 本帧计数是否 +1（有新球挡光进入）
     */
    public boolean update() {
        return update(false);
    }

    /**
     * 每帧调用一次：采样光电门，按本帧球流方向做双向计数。
     *
     * <p>吸球方向（{@code outflow = false}）：false → true 上升沿 = 一个新球进入球道，计数 +1；
     * 吐球方向（{@code outflow = true}）：true → false 下降沿 = 一个球离开球道，计数 −1（钳位 ≥ 0）。
     *
     * <p>第一帧只建立边缘检测基准（不计数），之后每帧按对应方向的边沿计数。
     *
     * @param outflow 本帧球流方向：true = 吐球（球经该光电门排出球道），false = 吸球 / 静止
     * @return 本帧计数是否发生变化（吸球上升沿，或吐球下降沿且计数大于 0）
     */
    public boolean update(boolean outflow) {
        boolean blocked = breakBeam.update();
        if (!hasBaseline) {
            hasBaseline = true;
            lastBlocked = blocked;
            return false;
        }
        boolean changed = false;
        if (outflow) {
            // 吐球：球先进入光路（该球已计入），离开光路的下降沿才是真正排出
            // 上升沿不计数，否则会把"出球进入光路"误当成新球
            if (!blocked && lastBlocked && count > 0) {
                count--;
                changed = true;
            }
        } else if (blocked && !lastBlocked) {
            count++;
            changed = true;
        }
        lastBlocked = blocked;
        return changed;
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
