package org.firstinspires.ftc.teamcode.Auto.AutomaticCalibration;

/**
 * 底盘反馈（P / D）辨识器：由闭环阶跃响应反解二阶特征量并给出增益修正建议。
 *
 * <h3>为什么是 P / D 而不是 P / I / D</h3>
 * 底盘反馈由 {@code HolonomicController} 提供，其输出是<b>速度指令</b>：
 * <pre>
 *   v_cmd = posGain · (位姿误差) + velGain · (速度误差) + v_desired
 * </pre>
 * 因此只有「位置误差的 P 增益」与「速度误差的 D 增益」两项，<b>没有积分项</b>
 * （积分会使速度指令在稳态误差下持续增长，对底盘无益）。本辨识器对应输出：
 * <ul>
 *   <li>位置增益 {@code posGain}（对应 {@code axialGain / lateralGain / headingGain}），
 *       量纲 <b>1/s</b>；</li>
 *   <li>速度增益 {@code velGain}（对应 {@code axialVelGain / lateralVelGain / headingVelGain}），
 *       <b>无量纲</b>。</li>
 * </ul>
 *
 * <h3>为什么不用 {@code Kp = ωn²}、{@code Kd = 2ζωn} 的极点配置公式</h3>
 * 那组公式假设控制量是<b>加速度</b>（双积分对象 {@code ẍ = u}）。而本链路控制量是<b>速度</b>，
 * 直接把 {@code ωn²} 填进 {@code posGain} 会同时犯两个错误：
 * <ol>
 *   <li><b>量纲错误</b>：{@code ωn²} 是 1/s²，而 {@code posGain} 只能是 1/s；</li>
 *   <li><b>量级错误</b>：实测 {@code ωn≈3 rad/s} 时 {@code ωn²≈10}，会把
 *       {@code axialGain} 从 2 顶到 10，与现有参数差一个数量级，违反「量级一致」要求。</li>
 * </ol>
 * 因此本辨识器改用<b>相对修正</b>：先由实测阶跃响应标定「当前增益下的闭环特征量
 * ζ_meas、ωn_meas」，再按目标阻尼/带宽对现有增益做比例修正。这样建议值天然与现有参数
 * 同量级，操作手可逐次迭代逼近。
 *
 * <h3>阶跃特征量</h3>
 * 对归一化阶跃响应：
 * <pre>
 *   超调量 M_p  →  ζ_meas = −ln(M_p) / √(π² + ln²(M_p))
 *   峰值时间 t_p →  ωn_meas = π / (t_p · √(1 − ζ_meas²))      （ζ_meas < 1 时）
 * </pre>
 * 若无超调（M_p ≤ 0，过阻尼/一阶），超调公式失效，退化为用 10%–90% 上升时间估计
 * {@code ωn ≈ 2.2 / t_r} 并把 {@code ζ_meas} 记为 1.0，同时标记 {@code reliable=false}。
 *
 * <p>本类纯静态、无硬件依赖，可直接在 JVM 单元测试中用构造数据验证。
 */
public final class FeedbackIdentifier {

    /** 超调量小于该值即视为「无超调」，走上升时间回退路径。 */
    public static final double NO_OVERSHOOT_EPS = 0.01;
    /** ζ_meas 上限：避免 ζ→1 时 √(1−ζ²)→0 导致 ωn 发散。 */
    public static final double ZETA_MAX = 0.95;
    /** ζ_meas 下限：避免 ln(M_p) 过大导致 ωn 异常。 */
    public static final double ZETA_MIN = 0.05;

    private FeedbackIdentifier() {
    }

    /** 阶跃响应特征量（归一化后测量，与增益无关）。 */
    public static final class StepMetrics {
        /** 超调量（比例，如 0.25 表示超调 25%），无超调时为 ≤0 */
        public final double overshoot;
        /** 峰值时刻，单位 s */
        public final double peakTime;
        /** 10%–90% 上升时间，单位 s */
        public final double riseTime;
        /** 稳态值（归一化前，即原始物理单位） */
        public final double steadyState;
        /** 样本数 */
        public final int nSamples;
        /** 是否有效（样本足够、稳态值正向且显著） */
        public final boolean valid;

        StepMetrics(double overshoot, double peakTime, double riseTime,
                    double steadyState, int nSamples, boolean valid) {
            this.overshoot = overshoot;
            this.peakTime = peakTime;
            this.riseTime = riseTime;
            this.steadyState = steadyState;
            this.nSamples = nSamples;
            this.valid = valid;
        }

        @Override
        public String toString() {
            return String.format("overshoot=%.1f%%, peak=%.3fs, rise=%.3fs, ss=%.3f (n=%d, valid=%s)",
                    overshoot * 100, peakTime, riseTime, steadyState, nSamples, valid);
        }
    }

    /** 实测闭环二阶特征量。 */
    public static final class ClosedLoopMetrics {
        /** 实测阻尼比 */
        public final double zeta;
        /** 实测自然频率，单位 rad/s */
        public final double omegaN;
        /** 是否由超调量可靠反解（false 表示走上升时间回退） */
        public final boolean reliable;
        /** 原始阶跃特征量 */
        public final StepMetrics step;

        ClosedLoopMetrics(double zeta, double omegaN, boolean reliable, StepMetrics step) {
            this.zeta = zeta;
            this.omegaN = omegaN;
            this.reliable = reliable;
            this.step = step;
        }

        @Override
        public String toString() {
            return String.format("zeta=%.3f, omegaN=%.3f rad/s, reliable=%s | %s",
                    zeta, omegaN, reliable, step);
        }
    }

    /** 增益修正建议。 */
    public static final class GainRecommendation {
        /** 建议位置增益（1/s），对应 axialGain / lateralGain / headingGain */
        public final double posGain;
        /** 建议速度增益（无量纲），对应 axialVelGain / lateralVelGain / headingVelGain */
        public final double velGain;
        /** 修正比例：posGain / posGainCurrent */
        public final double posScale;
        /** 修正比例：velGain / velGainCurrent */
        public final double velScale;

        GainRecommendation(double posGain, double velGain, double posScale, double velScale) {
            this.posGain = posGain;
            this.velGain = velGain;
            this.posScale = posScale;
            this.velScale = velScale;
        }

        @Override
        public String toString() {
            return String.format("posGain=%.4f (×%.2f), velGain=%.4f (×%.2f)",
                    posGain, posScale, velGain, velScale);
        }
    }

    /**
     * 由时间序列测量阶跃响应特征量。
     *
     * <p>稳态值取末尾 {@code max(1, n/10)} 个样本的均值；超调量为
     * {@code max(y)/steady − 1}；上升时间为首次到达 {@code 0.1·steady} 与
     * {@code 0.9·steady} 的插值时刻之差。
     *
     * @param t 时间序列，单位 s，单调递增
     * @param y 响应序列（与目标同量纲；上升阶跃目标为正，稳态值应 {@code > 0}）
     */
    public static StepMetrics measureStepResponse(double[] t, double[] y) {
        if (t.length != y.length) {
            throw new IllegalArgumentException("时间与响应数组长度不一致");
        }
        int n = t.length;
        if (n < 3) {
            return new StepMetrics(0, 0, 0, 0, n, false);
        }

        // 稳态值：末尾 10% 均值
        int tail = Math.max(1, n / 10);
        double ss = 0;
        for (int i = n - tail; i < n; i++) {
            ss += y[i];
        }
        ss /= tail;

        if (ss <= 1e-6) {
            return new StepMetrics(0, 0, 0, ss, n, false);
        }

        // 峰值与峰值时刻
        double peak = Double.NEGATIVE_INFINITY;
        double peakTime = 0;
        for (int i = 0; i < n; i++) {
            if (y[i] > peak) {
                peak = y[i];
                peakTime = t[i];
            }
        }
        double overshoot = peak / ss - 1.0;

        // 10%–90% 上升时间（线性插值）
        double t10 = interpolateCrossing(t, y, 0.1 * ss);
        double t90 = interpolateCrossing(t, y, 0.9 * ss);
        double riseTime = t90 - t10;

        boolean valid = true;
        if (overshoot < -0.5) {
            // 稳态值正向但峰值远小于稳态（数据异常，如中途跌落）
            valid = false;
        }
        return new StepMetrics(overshoot, peakTime, riseTime, ss, n, valid);
    }

    /**
     * 由阶跃特征量反解闭环 ζ、ωn。
     *
     * @param m 阶跃特征量；{@code !m.valid} 时返回 {@code reliable=false} 且 ζ=1、ωn=0
     */
    public static ClosedLoopMetrics closedLoopFromStep(StepMetrics m) {
        if (!m.valid || m.overshoot <= NO_OVERSHOOT_EPS) {
            // 无超调：二阶超调公式失效，用上升时间回退估计带宽
            double omega = (m.riseTime > 1e-6) ? 2.2 / m.riseTime : 0;
            return new ClosedLoopMetrics(1.0, omega, false, m);
        }
        double lnMp = Math.log(m.overshoot);
        double zeta = -lnMp / Math.sqrt(Math.PI * Math.PI + lnMp * lnMp);
        zeta = clamp(zeta, ZETA_MIN, ZETA_MAX);
        double omega = (m.peakTime > 1e-6)
                ? Math.PI / (m.peakTime * Math.sqrt(1 - zeta * zeta))
                : 0;
        return new ClosedLoopMetrics(zeta, omega, m.valid, m);
    }

    /** 便捷入口：直接由时间序列得到闭环特征量。 */
    public static ClosedLoopMetrics identify(double[] t, double[] y) {
        return closedLoopFromStep(measureStepResponse(t, y));
    }

    /**
     * 内环（速度环）时间常数估计：{@code τ = kA / kV}。
     *
     * <p>由前馈模型 {@code u ≈ kS + kV·v + kA·a} 对 {@code v} 求传递函数，得到
     * 一阶滞后 {@code v(s)/u(s) ≈ (1/kV) / (1 + τ s)}，故 τ 即速度环带宽倒数。
     * τ 仅作为交叉校验信息输出，不直接决定反馈增益。
     *
     * @param kV V/(tick/s)
     * @param kA V/(tick/s²)
     * @return τ，单位 s；kV 过小时返回 0
     */
    public static double velocityLoopTau(double kV, double kA) {
        if (Math.abs(kV) < 1e-12) {
            return 0;
        }
        return kA / kV;
    }

    /**
     * 按目标阻尼/带宽对现有增益做比例修正。
     *
     * <p>修正规则（start-from-current，天然保持量级）：
     * <pre>
     *   posScale = ωn_target / ωn_meas       （带宽不足则放大位置增益）
     *   velScale = ζ_target  / ζ_meas        （阻尼不足则放大速度增益）
     *   posGain  = posGain_current × posScale
     *   velGain  = velGain_current × velScale
     * </pre>
     * 比例被钳制在 {@code [scaleMin, scaleMax]}，避免单次噪声导致参数暴涨；
     * 当 {@code ωn_meas} 或 {@code ζ_meas} 不可用时对应比例取 1（不修改）。
     *
     * @param posGainCurrent 当前位置增益（1/s）
     * @param velGainCurrent 当前速度增益（无量纲）
     * @param m              实测闭环特征量
     * @param omegaTarget    目标自然频率，单位 rad/s（≤0 表示保持实测带宽）
     * @param zetaTarget     目标阻尼比（≤0 表示保持实测阻尼）
     * @param scaleMin       单次修正比例下限
     * @param scaleMax       单次修正比例上限
     */
    public static GainRecommendation recommendGains(double posGainCurrent, double velGainCurrent,
                                                    ClosedLoopMetrics m,
                                                    double omegaTarget, double zetaTarget,
                                                    double scaleMin, double scaleMax) {
        double posScale = 1.0;
        if (omegaTarget > 0 && m.omegaN > 1e-6) {
            posScale = clamp(omegaTarget / m.omegaN, scaleMin, scaleMax);
        }
        double velScale = 1.0;
        if (zetaTarget > 0 && m.zeta > 1e-6) {
            velScale = clamp(zetaTarget / m.zeta, scaleMin, scaleMax);
        }
        return new GainRecommendation(posGainCurrent * posScale, velGainCurrent * velScale,
                posScale, velScale);
    }

    /** 过阻尼交叉校验：由模型 {@code ζ = (1+velGain)/(2·√(posGain·τ))} 预测阻尼比（信息用）。 */
    public static double modelZeta(double posGain, double velGain, double tau) {
        if (posGain <= 0 || tau <= 0) {
            return 0;
        }
        return (1 + velGain) / (2.0 * Math.sqrt(posGain * tau));
    }

    /** 由模型 {@code ωn = √(posGain/τ)} 预测自然频率（信息用）。 */
    public static double modelOmegaN(double posGain, double tau) {
        if (posGain <= 0 || tau <= 0) {
            return 0;
        }
        return Math.sqrt(posGain / tau);
    }

    /** 线性插值求 y 首次上升穿过阈值 level 的时刻。 */
    private static double interpolateCrossing(double[] t, double[] y, double level) {
        for (int i = 1; i < y.length; i++) {
            if (y[i - 1] < level && y[i] >= level) {
                double dy = y[i] - y[i - 1];
                if (Math.abs(dy) < 1e-12) {
                    return t[i];
                }
                double frac = (level - y[i - 1]) / dy;
                return t[i - 1] + frac * (t[i] - t[i - 1]);
            }
        }
        return Double.NaN;
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
