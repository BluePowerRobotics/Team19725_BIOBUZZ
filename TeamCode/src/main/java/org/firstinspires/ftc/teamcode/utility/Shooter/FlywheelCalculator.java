package org.firstinspires.ftc.teamcode.utility.Shooter;

/**
 * 飞轮（发射机构）速度环计算器 —— <b>纯计算类</b>，不持有任何硬件引用。
 *
 * <p>本类只做一件事：{@code (目标转速 ω*, 当前转速 ω, 母线电压 V_bus, dt) → 占空比 duty}。
 * 电机、遥测、硬件初始化全部留在调用方（{@code Controllers.Shooter.PrototypeShooter}、
 * {@code Controllers.Turret.Shooter}），因此本类可离线复算（给一串采样即可回放整条控制轨迹），
 * 也便于单独做数值验证。
 *
 * <p><b>简化控制律（SimpleShooter）</b>：
 * <pre>
 *   e      = ω* - ω
 *   P/I    = 直接使用 e（不做速度滤波）
 *   s      = clamp(V_nom / V_bus_raw, sMin, sMax)       // 原始母线电压，只乘前馈
 *   ff     = s·(kS·sgn(ω*) + kff·ω*)
 *   duty   = sat( ff + kp·e + ki·∫e , dutyMin, dutyMax )
 * </pre>
 * 简化说明：
 * <ul>
 *   <li><b>无速度滤波、无母线电压滤波/冻结</b>：P/I 直接使用原始误差，前馈直接使用原始母线电压。</li>

 *   <li><b>积分冻结</b>：只有 {@code |e| < iZone} 才累加；否则保留原积分（冻结），目标为 0 时清零。</li>

 *   <li><b>积分限幅</b>：I 项仍限制在 ±maxI，触限时回算 integral。</li>

 *   <li><b>母线电压归一化</b>：只对前馈做 {@code s} 缩放（不乘反馈），直接使用本帧原始电压。
 *       让「同一目标转速在不同电量下要的占空比」保持一致，积分不必长期负重。

 * </ul>
 *
 * <p><b>量纲</b>：所有增益单位均为「占空比」，所以本类不直接读写硬件，调用方负责把
 * {@code DcMotorEx#getVelocity()} 的 tick/s 与 {@code VoltageSensor#getVoltage()} 的 V 传进来。
 *
 * <p><b>参数从哪来</b>：一套机器一套参数，见 {@link FlywheelParams}（实机有两套：大球 / 小球）。
 * 调用方每帧把 {@code FlywheelParams#applyTo(FlywheelCalculator)} 调一遍即可；积分门限
 * 每套参数的 {@code iZone} 也由该调用同步，Dashboard 改动实时生效。
 */
public class FlywheelCalculator {
    // ==================== 可调参数（由 FlywheelParams#applyTo 每帧同步） ====================
    //
    // 这里<b>不放工程默认值</b>：一套机器一套参数，唯一出处是 utility/Shooter/FlywheelParams.java
    // 里的 BIG / SMALL 两个块。调用方必须先 applyTo 再 calculate（Controllers 里都是这么做的），
    // 否则所有增益为 0、输出恒为 dutyMin。

    /** 静摩擦前馈 (占空比)，按 {@code kS·sgn(ω*)} 施加 */
    public double kS;
    /** 稳态前馈增益 (占空比)/(tick/s) */
    public double kff;
    /** 比例增益 (占空比)/(tick/s) */
    public double kp;
    /** 积分增益 (占空比)/(tick/s·s) */
    public double ki;
    /** I 项限幅 (占空比) */
    public double maxI;

    /** 输出下限 (占空比) */
    public double dutyMin;
    /** 输出上限 (占空比) */
    public double dutyMax;

    /** 速度量化台阶 (tick/s) */
    public double quantStep;
    /** 稳态误差带比例：eBand = max(settleFrac·|ω*|, 2·quantStep) */
    public double settleFrac;
    /** 连续落在误差带内多少帧算到位 */
    public int settleFrames;

    /** 积分门限 (tick/s)：|e| < iZone 时累加，否则冻结；由每套 FlywheelParams 同步 */
    public double iZone;

    /** 是否启用母线电压归一化（只乘前馈） */
    public boolean useVoltageNorm;
    /** 额定母线电压 (V) */
    public double vBusNominal;
    /** 归一化系数下限 */
    public double vScaleMin;
    /** 归一化系数上限 */
    public double vScaleMax;

    // ==================== 运行时状态 ====================

    /** 最近一次输出的占空比 */
    private double duty = 0;

    /** 最近一次控制量各分量（供遥测：静摩擦 / 前馈 / 比例 / 积分） */
    private double lastKSTerm = 0, lastFFTerm = 0, lastPTerm = 0, lastITerm = 0;

    /** 积分累计量 ∫e·dt (tick/s·s) */
    private double integral = 0;

    /** 最近一次原始误差 (tick/s) */
    private double errorRaw = 0;

    /** 当前稳态误差带 (tick/s) */
    private double eBand = 0;

    /** 当前电压归一化系数 */
    private double vScale = 1.0;

    /** 积分是否处于活跃（累加）状态 */
    private boolean integralActive = false;

    /** 稳态判据：连续在带内的帧数 */
    private int settleCnt = 0;

    /** 上一次的目标转速，用于检测目标改变 */
    private double lastTargetVel = Double.NaN;

    /** dt 异常（首帧、长时间停顿、时钟回退）时使用的标称循环周期 (s) */
    public static final double DEFAULT_DT = 0.02;

    public FlywheelCalculator() {
        lastTime = now();
    }

    /** 上一次 calculate 的时间戳 (s)，仅 2 参数/3 参数入口使用 */
    private double lastTime;

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    // ==================== 计算 ====================

    /**
     * 单步计算（内部自行推进 dt，用单调时钟）。
     *
     * <p>每帧只可调用一次（内部计时器与积分状态会推进）。
     *
     * @param targetVel  目标转速 (tick/s)，0 表示停转
     * @param currentVel 当前转速 (tick/s)
     * @param vBus       母线电压 (V)；≤1 或 NaN 视为无有效读数，此时 vScale = 1
     * @return 占空比，已限幅到 [dutyMin, dutyMax]
     */
    public double calculate(double targetVel, double currentVel, double vBus) {
        double currentTime = now();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        return calculate(targetVel, currentVel, vBus, dt);
    }

    /**
     * 单步计算（显式 dt，含母线电压）—— 实机主入口 / 离线复算入口。
     *
     * @param targetVel  目标转速 (tick/s)
     * @param currentVel 当前转速 (tick/s)
     * @param vBus       母线电压 (V)；≤1 或 NaN 视为无有效读数
     * @param dt         距上一步的时间 (s)；≤0 或 &gt;1 时用 {@link #DEFAULT_DT} 兜底
     * @return 占空比，已限幅到 [dutyMin, dutyMax]
     */
    public double calculate(double targetVel, double currentVel, double vBus, double dt) {
        if (dt <= 0 || dt > 1.0) {
            dt = DEFAULT_DT;
        }
        // ---- 首帧：只建立目标跟踪 ----
        if (Double.isNaN(lastTargetVel)) {
            lastTargetVel = targetVel;
        }
        // ---- 目标改变：只重置到位计数，积分按简化规则保留 ----
        if (targetVel != lastTargetVel) {
            lastTargetVel = targetVel;
            settleCnt = 0;
        }
        // ---- 误差与稳态误差带 ----
        errorRaw = targetVel - currentVel;
        eBand = bandFor(targetVel);
        // ---- 积分门控：只用原始误差和 iZone ----
        if (targetVel == 0) {
            integralActive = false;
        } else if (Math.abs(errorRaw) < iZone) {
            integralActive = true;
        } else {
            integralActive = false;
        }
        // ---- P/I 直接使用 errorRaw ----
        // ---- 母线电压归一化：直接使用本帧原始电压 ----
        vScale = 1.0;
        if (useVoltageNorm && validVoltage(vBus)) {
            vScale = vBusNominal / vBus;
            if (vScale < vScaleMin) {
                vScale = vScaleMin;
            } else if (vScale > vScaleMax) {
                vScale = vScaleMax;
            }
        }
        // ---- 前馈（静摩擦 + 稳态）：只有这两项乘电压归一化系数 ----
        // sgn(ω*)：ω*=0 时为 0，避免停转指令下静摩擦前馈把飞轮推转
        double ffSign = targetVel > 0 ? 1.0 : (targetVel < 0 ? -1.0 : 0.0);
        lastKSTerm = vScale * kS * ffSign;
        lastFFTerm = vScale * kff * targetVel;
        // ---- 比例 ----
        lastPTerm = kp * errorRaw;
        // ---- 积分：|e| < iZone 才累加；停机清零；其余情况冻结原积分 ----
        // integralActive 已在上面按 targetVel 和 iZone 计算。
        if (integralActive) {
            integral += errorRaw * dt;
        } else if (targetVel == 0) {
            integral = 0.0;
        } else {
            // 冻结：保持原积分不变。
        }
        double iTerm = ki * integral;
        // 抗饱和：对积分项回算限幅
        if (iTerm > maxI) {
            iTerm = maxI;
            if (ki != 0) {
                integral = maxI / ki;
            }
        } else if (iTerm < -maxI) {
            iTerm = -maxI;
            if (ki != 0) {
                integral = -maxI / ki;
            }
        }
        lastITerm = iTerm;
        // ---- 合成输出 ----
        double out = lastKSTerm + lastFFTerm + lastPTerm + lastITerm;
        if (out > dutyMax) {
            out = dutyMax;
        }
        if (out < dutyMin) {
            out = dutyMin;
        }
        duty = out;
        // ---- 到位判据：连续 settleFrames 帧误差在带内 ----
        boolean nearBand = targetVel != 0 && Math.abs(errorRaw) <= eBand;
        if (nearBand) {
            if (settleCnt < settleFrames) {
                settleCnt++;
            }
        } else {
            settleCnt = 0;
        }
        return out;
    }

    /**
     * 单步计算（内部推进 dt，<b>不含</b>母线电压）—— 兼容旧调用方式，实机请用带 vBus 的重载。
     *
     * <p>注意：Java 无法用「第三参数的含义」区分重载，因此<b>没有</b>
     * {@code calculate(t, ω, dt)} 这个三参数无电压版本。离线回放无电压列的旧日志请用
     * {@code calculate(t, ω, Double.NaN, dt)}（等价于电压归一化关闭）。
     *
     * @param targetVel  目标转速 (tick/s)
     * @param currentVel 当前转速 (tick/s)
     * @return 占空比
     */
    public double calculate(double targetVel, double currentVel) {
        return calculate(targetVel, currentVel, Double.NaN);
    }

    // ==================== 内部工具 ====================

    /**
     * 稳态误差带：{@code max(settleFrac·|ω*|, 2·quantStep)}。
     *
     * <p>下限 2 个量化台阶是物理下限——再小就没有意义，编码器给不出那样的分辨率；
     * 上限保证高转速下相对误差不会被压到 0（否则永远「不到位」）。
     *
     * @param targetVel 目标转速 (tick/s)
     * @return 误差带 (tick/s)
     */
    private double bandFor(double targetVel) {
        double floor = 2.0 * quantStep;
        double relative = settleFrac * Math.abs(targetVel);
        return relative > floor ? relative : floor;
    }

    /** @return 电压读数是否有效 */
    private static boolean validVoltage(double v) {
        return !Double.isNaN(v) && v > 1.0;
    }

    // ==================== 状态重置 ====================

    /** 重置积分与全部瞬态状态（从开环切回闭环前调用）。 */
    public void reset() {
        integral = 0.0;
        errorRaw = 0.0;
        eBand = 0.0;
        duty = 0.0;
        lastKSTerm = 0.0;
        lastFFTerm = 0.0;
        lastPTerm = 0.0;
        lastITerm = 0.0;
        integralActive = false;
        settleCnt = 0;
        lastTargetVel = Double.NaN;
        lastTime = now();
    }

    // ==================== 读取 ====================

    /** @return 最近一次输出的占空比 */
    public double getDuty() {
        return duty;
    }

    /** @return 内部积分累计量 ∫e·dt (tick/s·s) */
    public double getIntegral() {
        return integral;
    }

    /** @return 最近一次的静摩擦前馈分量（占空比，已乘电压归一化系数） */
    public double getKSTerm() {
        return lastKSTerm;
    }

    /** @return 最近一次的前馈分量 kff·ω*（占空比，已乘电压归一化系数） */
    public double getFfTerm() {
        return lastFFTerm;
    }

    /** @return 最近一次的比例分量 kp·e（占空比） */
    public double getPTerm() {
        return lastPTerm;
    }

    /** @return 最近一次的积分分量（占空比，已限幅） */
    public double getITerm() {
        return lastITerm;
    }

    /** @return 最近一次的原始误差 ω*−ω (tick/s) */
    public double getErrorRaw() {
        return errorRaw;
    }

    /** @return 当前稳态误差带 (tick/s) */
    public double getErrorBand() {
        return eBand;
    }

    /** @return 当前积分门限 iZone (tick/s) */
    public double getIZone() {
        return iZone;
    }

    /** @return 当前电压归一化系数 s（未启用恒为 1） */
    public double getVScale() {
        return vScale;
    }

    /** @return 积分当前是否在累加（false = 冻结或停机） */
    public boolean isIntegralActive() {
        return integralActive;
    }

    /** @return 是否已到位（连续 {@link #settleFrames} 帧误差在带内） */
    public boolean isSettled() {
        return settleCnt >= settleFrames;
    }

    /**
     * @return 量化极限环幅值下限 kp × 一个量化台阶（占空比）。
     *         工程上应 ≤0.05；超过 0.1 会跑出明显的量化极限环，须降 kp。
     */
    public double getKpQuantization() {
        return kp * quantStep;
    }
}
