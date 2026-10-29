package org.firstinspires.ftc.teamcode.Auto.AutomaticCalibration;

/**
 * 底盘运动约束推算器：由「打滑前的极限实测值」折算出 Road Runner 的运动约束建议值。
 *
 * <h3>量纲（与 {@code MecanumDrive.Params} 完全一致）</h3>
 * <ul>
 *   <li>{@code maxWheelVel}：英寸/秒（in/s）；</li>
 *   <li>{@code maxProfileAccel}：英寸/秒²（in/s²）；</li>
 *   <li>{@code minProfileAccel}：负数，取 {@code −maxProfileAccel} 以保证加减速对称；</li>
 *   <li>{@code maxAngVel}：弧度/秒（rad/s）；</li>
 *   <li>{@code maxAngAccel}：弧度/秒²（rad/s²）。</li>
 * </ul>
 *
 * <h3>打滑判据</h3>
 * 轮速链路推出的机器人速度 {@code wheelImplied}（= 编码器 tick 速度 × inPerTick）与
 * 融合定位器给出的地面真值速度 {@code groundTruth} 相互独立。二者相对偏差
 * <pre>
 *   slipRatio = (wheelImplied − groundTruth) / max(|wheelImplied|, eps)
 * </pre>
 * 超过容差即判为打滑：此时轮子转得比车体快，说明轮胎已失去抓地，继续提速只会
 * 让轨迹跟踪发散，故应把约束值锁定在打滑发生前的实测峰值 × 安全系数。
 *
 * <p>本类纯静态、无硬件依赖，可直接在 JVM 单元测试中用构造数据验证。
 */
public final class ConstraintIdentifier {

    /** 相对偏差分母保护值（in/s） */
    public static final double SPEED_EPS = 1e-6;
    /** 默认安全系数：取打滑前峰值的 85%，留出控制裕量 */
    public static final double DEFAULT_SAFETY = 0.85;
    /** 默认打滑判据容差：相对偏差超过 25% 判为打滑 */
    public static final double DEFAULT_SLIP_TOL = 0.25;

    private ConstraintIdentifier() {
    }

    /** 运动约束建议值（量纲见类注释）。 */
    public static final class Constraints {
        /** 最大轮速，in/s */
        public final double maxWheelVel;
        /** 最大加速度，in/s² */
        public final double maxProfileAccel;
        /** 最小加速度，in/s²（= −maxProfileAccel） */
        public final double minProfileAccel;
        /** 最大角速度，rad/s */
        public final double maxAngVel;
        /** 最大角加速度，rad/s² */
        public final double maxAngAccel;
        /** 是否在加速度/速度扫描中检出打滑 */
        public final boolean slipDetected;

        public Constraints(double maxWheelVel, double maxProfileAccel, double minProfileAccel,
                           double maxAngVel, double maxAngAccel, boolean slipDetected) {
            this.maxWheelVel = maxWheelVel;
            this.maxProfileAccel = maxProfileAccel;
            this.minProfileAccel = minProfileAccel;
            this.maxAngVel = maxAngVel;
            this.maxAngAccel = maxAngAccel;
            this.slipDetected = slipDetected;
        }

        @Override
        public String toString() {
            return String.format(
                    "maxWheelVel=%.2f in/s, maxProfileAccel=%.2f in/s^2, minProfileAccel=%.2f, "
                            + "maxAngVel=%.3f rad/s, maxAngAccel=%.3f rad/s^2, slip=%s",
                    maxWheelVel, maxProfileAccel, minProfileAccel, maxAngVel, maxAngAccel, slipDetected);
        }
    }

    /**
     * 由打滑前轮速峰值折算 {@code maxWheelVel}。
     *
     * @param peakTicks 打滑前 |轮速| 峰值，单位 tick/s
     * @param inPerTick 每 tick 英寸数
     * @param safety    安全系数（0, 1]
     * @return 建议 maxWheelVel，单位 in/s；输入非正时返回 0
     */
    public static double maxWheelVelFromPeak(double peakTicks, double inPerTick, double safety) {
        if (inPerTick <= 0 || safety <= 0) {
            return 0;
        }
        return Math.abs(peakTicks) * inPerTick * safety;
    }

    /**
     * 由打滑前轮加速度峰值折算 {@code maxProfileAccel}。
     *
     * @param peakAccelTicks 打滑前 |轮加速度| 峰值，单位 tick/s²
     * @param inPerTick      每 tick 英寸数
     * @param safety         安全系数（0, 1]
     * @return 建议 maxProfileAccel，单位 in/s²
     */
    public static double maxProfileAccelFromPeak(double peakAccelTicks, double inPerTick, double safety) {
        if (inPerTick <= 0 || safety <= 0) {
            return 0;
        }
        return Math.abs(peakAccelTicks) * inPerTick * safety;
    }

    /** 由 {@code maxProfileAccel} 取对称的减速上限 {@code minProfileAccel = −maxProfileAccel}。 */
    public static double minProfileAccelFromMax(double maxProfileAccel) {
        return -Math.abs(maxProfileAccel);
    }

    /**
     * 由前馈模型推算「满电压自由轮速」{@code v_free = (V_batt − kS) / kV}。
     *
     * <p>相比真跑到最高速，模型推算更安全：只需已辨识的 kS、kV 与当前电池电压，
     * 即可估计轮子在满电压下能达到的稳态轮速（单位 tick/s）。这是电机空载特性的
     * 合理近似，再乘安全系数即得建议 {@code maxWheelVel}。
     *
     * @param battVolts 当前电池电压，V
     * @param kS        静态补偿，V
     * @param kV        速度增益，V/(tick/s)
     * @return 推算的自由轮速，tick/s；kV 非正或电压低于 kS 时返回 0
     */
    public static double modelMaxWheelVelTicks(double battVolts, double kS, double kV) {
        if (kV <= 0 || battVolts <= kS) {
            return 0;
        }
        return (battVolts - kS) / kV;
    }

    /**
     * 相对打滑比。
     *
     * @param wheelImpliedSpeed 轮速链路推出的机器人速度，in/s
     * @param groundTruthSpeed  融合定位器地面真值速度，in/s
     * @return 相对偏差；分母用 {@code max(|wheelImplied|, eps)} 保护
     */
    public static double slipRatio(double wheelImpliedSpeed, double groundTruthSpeed) {
        double denom = Math.max(Math.abs(wheelImpliedSpeed), SPEED_EPS);
        return (wheelImpliedSpeed - groundTruthSpeed) / denom;
    }

    /**
     * 打滑判据。
     *
     * @param wheelImpliedSpeed 轮速链路推出的机器人速度，in/s
     * @param groundTruthSpeed  融合定位器地面真值速度，in/s
     * @param tolerance         相对偏差容差
     * @return {@code true} 表示相对偏差超容差，判定为打滑
     */
    public static boolean isSlipping(double wheelImpliedSpeed, double groundTruthSpeed,
                                     double tolerance) {
        return Math.abs(slipRatio(wheelImpliedSpeed, groundTruthSpeed)) > tolerance;
    }

    /**
     * 汇总各阶段实测峰值，生成完整约束建议。
     *
     * @param peakWheelVelTicks  平移阶段打滑前 |轮速| 峰值，tick/s
     * @param peakAccelTicks     加速阶段打滑前 |轮加速度| 峰值，tick/s²
     * @param peakAngVel         转向阶段打滑前 |角速度| 峰值，rad/s
     * @param peakAngAccel       转向阶段打滑前 |角加速度| 峰值，rad/s²
     * @param inPerTick          每 tick 英寸数
     * @param safety             安全系数
     * @param slipDetected       扫描过程中是否检出打滑
     */
    public static Constraints recommend(double peakWheelVelTicks, double peakAccelTicks,
                                        double peakAngVel, double peakAngAccel,
                                        double inPerTick, double safety, boolean slipDetected) {
        double maxWheelVel = maxWheelVelFromPeak(peakWheelVelTicks, inPerTick, safety);
        double maxAccel = maxProfileAccelFromPeak(peakAccelTicks, inPerTick, safety);
        return new Constraints(maxWheelVel, maxAccel, minProfileAccelFromMax(maxAccel),
                Math.abs(peakAngVel) * safety, Math.abs(peakAngAccel) * safety, slipDetected);
    }
}
