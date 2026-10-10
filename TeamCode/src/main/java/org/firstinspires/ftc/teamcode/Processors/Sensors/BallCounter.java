package org.firstinspires.ftc.teamcode.Processors.Sensors;

/**
 * 小球计数器：基于压敏开关的上升沿检测（边缘检测）统计经过的小球个数。
 *
 * 原理：
 *   把压敏开关的实时布尔量看作数字信号：压着 = 1，没压着 = 0。
 *   小球经过/落到电阻上时，信号发生 0 → 1 跳变，即"上升沿"。
 *   每检测到一个上升沿，计数 +1。
 *
 * 为什么不会重复计数：
 *   若小球正好停在电阻上不动，信号持续为 1（1, 1, 1, ...），
 *   上升沿只在 0 → 1 的那一帧出现一次，之后不再触发，
 *   因此停留多久都只计 1 个。小球滚走后信号回到 0，
 *   下一个小球压上来时才会产生新的上升沿。
 *
 * 信号序列示意：
 *   pressed:  0 0 1 1 1 0 0 1 1 1 1 0
 *   上升沿:   _ _ ↑ _ _ _ _ ↑ _ _ _ _   →  计数 = 2（第2个球停在电阻上也只计1次）
 *
 * 用法（OpMode 主循环中，每帧调用 update()）：
 *   BallCounter counter = new BallCounter(new PressureSwitcher(hardwareMap, "pressure"));
 *   while (opModeIsActive()) {
 *       counter.update();                      // 采样 + 边缘检测
 *       telemetry.addData("balls", counter.getCount());  // 实时计数
 *   }
 */
public class BallCounter {

    private final PressureSwitcher pressureSwitcher;

    /** 上一帧的按压状态，用于与当前帧比较以检测跳变 */
    private boolean previousPressed;

    /** 累计检测到的小球个数 */
    private int count;

    /**
     * @param pressureSwitcher 已创建好的压敏开关（本类内部负责调用其 update()）
     */
    public BallCounter(PressureSwitcher pressureSwitcher) {
        this.pressureSwitcher = pressureSwitcher;
        this.previousPressed = false;
        this.count = 0;
    }

    /**
     * 采样一次并做上升沿检测：当前帧为 1 且上一帧为 0 时计数 +1。
     * 应在 OpMode 主循环中每帧调用。
     *
     * @return 本帧是否检测到新小球（是否出现上升沿）
     */
    public boolean update() {
        boolean currentPressed = pressureSwitcher.update();

        // 上升沿：0 → 1 跳变。持续为 1（球停住不动）不会重复触发
        boolean risingEdge = currentPressed && !previousPressed;
        if (risingEdge) {
            count++;
        }

        previousPressed = currentPressed;
        return risingEdge;
    }

    /**
     * 实时获取当前累计计数的小球个数
     */
    public int getCount() {
        return count;
    }

    /**
     * 清零计数并复位边缘检测状态（不影响压敏开关的阈值/alpha 配置）
     */
    public void reset() {
        count = 0;
        previousPressed = false;
        pressureSwitcher.reset();
    }
}
