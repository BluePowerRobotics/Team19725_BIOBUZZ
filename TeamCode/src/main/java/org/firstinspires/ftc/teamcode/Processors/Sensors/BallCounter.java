package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 小球计数器：对"是否遮挡"开关量做上升沿检测（边缘检测），统计经过的小球个数，
 * 计数方向由外部每帧传入的 0/1 模式信息决定。
 *
 * <p><b>更新链路（单向串行，无循环依赖）</b>：
 * <pre>
 * OpMode
 *   └─ RobotPosition.update()             // 1. 先完成自身状态更新（位姿 / EKF）
 *        └─ BallCounter.update(mode)      // 2. 主动触发计数器更新
 *             └─ BreakBeamSensor.update()  // 3. 计数器完成计数后，再触发光电门采样
 * </pre>
 * 即：计数器先用光电门<b>最近一次</b>采样结果完成本帧计数（核心业务逻辑不变），
 * 自身状态更新完成后再推进下属光电门，为下一帧准备数据。因此遮挡信号相对计数
 * 固定延迟一帧（约一个主循环周期），但模块间严格单向调用、不会出现同一对象
 * 被重复更新或半更新状态被上层读取的问题。
 *
 * 两路 0/1 信号的区别（不要混淆）：
 *   1. 外部模式信息（update 的 externalState 参数）：
 *      0 = 进入模式（intake），1 = 出球模式（output）。它只决定"上升沿来了
 *      计数 +1 还是 -1"，不表示球有没有挡住光电门。
 *   2. 遮挡信号（光电门 isBlocked()）：
 *      由光电门完成归一化、EMA 滤波与阈值判定后给出，用于检测上升沿
 *      （"球挡光"这一事件）。
 *
 * 计数规则：
 *   进入模式（外部信息 = 0，intake）：每检测到一个上升沿，count +1
 *   出球模式（外部信息 = 1，output）：每检测到一个上升沿，count -1
 *   count 不会减到负数（球已打空时再挡光，计数停在 0）
 *
 *   注意：发射（shooting）时需要清空球道内球的计数，该操作由上层程序负责，
 *   不在本类实现（上层可调用 reset() 清零）。
 *
 * 为什么不会重复计数：
 *   若小球正好停在光电门中不动，遮挡信号持续为 true，
 *   上升沿只在 false → true 的那一帧出现一次，之后不再触发，
 *   因此停留多久都只计 1 次。小球离开后信号回到 false，
 *   下一个小球挡光时才会产生新的上升沿。
 *
 * 去抖说明：
 *   遮挡信号的滤波 / 去抖由下属光电门完成，本类不再重复做去抖，
 *   只负责"上升沿检测 + 方向计数"。
 */
public class BallCounter {

    /** 外部信息：进入模式（intake，球挡光计数 +1） */
    public static final int MODE_INTAKE = 0;
    /** 外部信息：出球模式（output，球挡光计数 -1） */
    public static final int MODE_OUTPUT = 1;

    /** 下属光电门：由本类在 update 末尾触发采样（信号处理层） */
    private final BreakBeamSensor breakBeam;

    /** 上一帧的遮挡状态，用于与当前帧比较以检测跳变 */
    private boolean lastBlocked;

    /** 累计计数的小球个数（始终 ≥ 0） */
    private int count;

    /**
     * 按 Robot Configuration 中的设备名创建下属光电门（推荐入口）。
     *
     * @param hardwareMap 硬件映射表
     * @param deviceName  模拟输入在配置中的设备名（如 {@code "breakBeam"}）
     */
    public BallCounter(HardwareMap hardwareMap, String deviceName) {
        this(new BreakBeamSensor(hardwareMap, deviceName));
    }

    /**
     * 注入已创建好的下属光电门（便于台架测试与标定程序）。
     *
     * @param breakBeam 对射光电门（本类内部负责触发其 update()）
     */
    public BallCounter(BreakBeamSensor breakBeam) {
        this.breakBeam = breakBeam;
        this.lastBlocked = false;
        this.count = 0;
    }

    /**
     * 每帧调用一次：先完成自身的上升沿检测计数，再触发下属光电门采样。
     *
     * @param externalState 外部信息：0 = 进入模式（intake），1 = 出球模式（output）
     * @return 本帧是否有球挡光（上升沿；无论计数 +1 还是 -1）
     */
    public boolean update(int externalState) {
        if (externalState != MODE_INTAKE && externalState != MODE_OUTPUT) {
            throw new IllegalArgumentException(
                    "externalState must be 0 (intake) or 1 (output), got: " + externalState);
        }

        // ---- 1. 自身数据更新：用光电门最近一次采样结果做边缘检测与计数 ----
        boolean output = (externalState == MODE_OUTPUT);
        boolean blocked = breakBeam.isBlocked();

        // 上升沿：false → true 跳变。持续为 true（球停住不动）不会重复触发
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

        // ---- 2. 自身状态更新完成后，再触发下属光电门，为下一帧准备数据 ----
        breakBeam.update();

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
}
