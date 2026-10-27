package org.firstinspires.ftc.teamcode.utility.Shooter;

import com.acmerobotics.dashboard.config.Config;

/**
 * 飞轮速度环的<b>参数集</b>（纯数据类，无硬件依赖）。
 *
 * <p><b>所有飞轮控制参数只在本文件里出现，且大球 / 小球分开写</b>：
 * <ul>
 *   <li>{@link #BIG} —— 大球飞轮，取值见文件末尾「大球飞轮参数」块</li>
 *   <li>{@link #SMALL} —— 小球飞轮，取值见文件末尾「小球飞轮参数」块</li>
 * </ul>
 * 中间是字段声明（只有含义，没有数值），<b>要改参数就只改末尾那两个块</b>。
 *
 * <p>两个实例由 {@code Controllers.Turret.Shooter} 在构造两个发射器时分别传入
 * （{@code Shooter.BIG_PARAMS} / {@code Shooter.SMALL_PARAMS}），并由
 * {@link #applyTo(FlywheelCalculator)} 每帧灌进计算器，
 * 因此在 FTC Dashboard 上改值也是下一帧即生效（两个飞轮互不影响）。
 *
 * <p><b>为什么参数不放 {@link FlywheelCalculator} 里</b>：计算器是「一套算法」，参数是「一台机器」。
 * 分开后计算器可以换一套参数就复算另一台飞轮的历史日志，两个飞轮也不必各写一份控制代码。
 *
 * <p><b>当前控制律</b>（第 4 代，2026-10-22 起，无 boost）：
 * <pre>
 *   e_raw  = ω* - ω
 *   s      = clamp(V_nom / V_busFilt, sMin, sMax)        // 母线电压归一化，只作用于前馈
 *   ff     = s·(kS·sgn(ω*) + kff·ω*)
 *   ωf     = EMA(ω, τ(|e_raw|))                          // 误差越大 τ 越小
 *   duty   = sat( ff + kp·(ω*-ωf) + ki·∫(ω*-ωf) , 0, 1 )
 * </pre>
 */
@Config
public class FlywheelParams {

    // ==================================================================================
    //  字段声明（只有含义，数值一律写在文件末尾的两套参数块里）
    // ==================================================================================

    // ---------------- 前馈：稳态模型 d_ss(ω) = kS + kff·ω ----------------

    /** 静摩擦前馈 (占空比)：稳态模型的截距，按 {@code kS·sgn(ω*)} 施加（ω*=0 时为 0） */
    public double kS;

    /** 稳态前馈增益 (占空比)/(tick/s)：稳态模型的斜率 */
    public double kff;

    // ---------------- 反馈 PI ----------------

    /**
     * 比例增益 (占空比)/(tick/s)。
     *
     * <p>量化台阶自检：{@code kp × quantStep} 是「一格编码器测量噪声被放大成多少占空比」，
     * 即量化极限环的幅值下限。工程判据：≤0.05 合格，≤0.02 好，&gt;0.1 会跑出教科书极限环。
     * 遥测里有一行 {@code kp*step} 自动告警。
     */
    public double kp;

    /** 积分增益 (占空比)/(tick/s·s) */
    public double ki;

    /** I 项上限 (占空比)：对 {@code ki·∫e} 限幅，同时回算积分量（抗饱和） */
    public double maxI;

    // ---------------- 输出限幅 ----------------

    /** 输出下限 (占空比) */
    public double dutyMin;

    /** 输出上限 (占空比) */
    public double dutyMax;

    // ---------------- 量化与稳态判据 ----------------

    /** 速度量化台阶 (tick/s)：编码器测速最小分辨率（用于误差带下限、kp 告警、滤波拐点） */
    public double quantStep;

    /** 稳态误差带比例：{@code eBand = max(settleFrac·|ω*|, 2·quantStep)} */
    public double settleFrac;

    /** 连续落在误差带内多少帧算「到位」（10 帧 ≈ 0.1 s） */
    public int settleFrames;

    // ---------------- 速度滤波（误差调度 EMA） ----------------

    /** 稳态时间常数 (s)：越小噪声越小、相位滞后越大 */
    public double tauVMax;

    /** 大误差时间常数 (s)：约 1 个控制周期，几乎不滤波，保证加速/退饱和不滞后 */
    public double tauVMin;

    /** 调度下拐点（倍 eBand）：{@code |e| ≤ eLoFrac·eBand} 用 tauVMax */
    public double eLoFrac;

    /** 调度上拐点（倍 eBand）：{@code |e| ≥ eHiFrac·eBand} 用 tauVMin */
    public double eHiFrac;

    // ---------------- 掉速判定与积分冻结 ----------------

    /** 进掉速态的阈值（倍 eBand）：{@code |e| > dropEnterFrac·eBand}；退出用 eBand（滞回） */
    public double dropEnterFrac;

    /** 掉速态下积分最多冻结多久 (s)，超时强制清零（防误差长期带外导致积分永久失效） */
    public double iFreezeTimeoutSec;

    // ---------------- 母线电压归一化 ----------------

    /** 是否启用母线电压归一化（只乘前馈）。默认关：先按 T1~T4 协议验证再打开 */
    public boolean useVoltageNorm;

    /** 额定母线电压 (V)：前馈按该电压标定 */
    public double vBusNominal;

    /** 归一化系数下限（欠压时前馈最多放大到 1/sMin 倍） */
    public double vScaleMin;

    /** 归一化系数上限（过压时前馈最多缩小到 1/sMax 倍） */
    public double vScaleMax;

    /**
     * 母线电压低通时间常数 (s)。必须远大于单发发射造成的电压跌落时长（≈0.4 s），
     * 否则会把「单发掉压」当成「电池没电」，在恢复期末把前馈放大到 &gt;1，人造一次饱和。
     */
    public double tauBus;

    /** 掉速结束后的电压采样 hold-off (s)：跳过电池 RC 回弹段，避免采样到回弹中段 */
    public double busHoldOffSec;

    /** 电压采样最长冻结时长 (s)：超时强制重新采样（防长期冻结导致 s 永久失真） */
    public double busFreezeTimeoutSec;

    // ---------------- 装配与范围 ----------------

    /** 电机方向：由飞轮接线极性决定（构造时下发一次），与整定参数无关 */
    public boolean motorReversed;

    /** 该飞轮最大转速 (tick/s)，目标转速会被钳制到 [0, speedMax] */
    public int speedMax;

    // ==================================================================================
    //  大球飞轮参数（BIG）—— 2026-09-04 实机整定，8 工作点回归 R²=0.9949
    // ==================================================================================

    /**
     * 大球飞轮参数。改这里即改大球；Dashboard 上显示为「FlywheelParams.BIG」分组。
     */
    public static final FlywheelParams BIG = new FlywheelParams();

    static {
        // ---- 前馈（Tune6.5 Feedforward：多点稳态回归 d_ss = kS + kff·ω）----
        BIG.kS = 0.02945;
        BIG.kff = 3.8307e-4;

        // ---- PI（Tune6.1 StepResponse；kp 另需满足 kp×20 ≤ 0.05）----
        BIG.kp = 1.81917e-3;
        BIG.ki = 2.59224e-3;
        BIG.maxI = 0.3;

        // ---- 输出限幅 ----
        BIG.dutyMin = 0.0;
        BIG.dutyMax = 1.0;

        // ---- 量化与稳态判据 ----
        BIG.quantStep = 20.0;      // 实测编码器测速台阶
        BIG.settleFrac = 0.02;     // ±2%
        BIG.settleFrames = 10;     // 10 帧 ≈ 0.1 s

        // ---- 速度滤波（Tune6.6 VelocityFilter 反解 tauVMax）----
        BIG.tauVMax = 0.030;
        BIG.tauVMin = 0.008;       // ≈ 1 个控制周期
        BIG.eLoFrac = 0.30;
        BIG.eHiFrac = 0.90;

        // ---- 掉速判定与积分冻结 ----
        BIG.dropEnterFrac = 3.0;
        BIG.iFreezeTimeoutSec = 1.5;

        // ---- 母线电压归一化（Tune6.7 BusVoltageFilter 定 tauBus）----
        BIG.useVoltageNorm = false;    // 默认关，验证后再开
        BIG.vBusNominal = 12.0;
        BIG.vScaleMin = 0.85;
        BIG.vScaleMax = 1.25;
        BIG.tauBus = 1.0;
        BIG.busHoldOffSec = 0.6;
        BIG.busFreezeTimeoutSec = 2.0;

        // ---- 装配与范围 ----
        BIG.motorReversed = true;
        BIG.speedMax = 3000;
    }

    // ==================================================================================
    //  小球飞轮参数（SMALL）—— ★ 尚未标定，下面是大球值的占位拷贝
    // ==================================================================================

    /**
     * 小球飞轮参数。改这里即改小球；Dashboard 上显示为「FlywheelParams.SMALL」分组。
     *
     * <p><b>★ 目前全部是大球值占位</b>：小球飞轮只做过一次 2026-09-05 18:55 的实测
     * （当时 kS=0、kff 沿用大球），稳态误差 +10.9 / +27.3 tick/s，属系统性欠给，
     * 说明它的 kS 与 kff 都与大球不同。上机按下表逐项替换：
     * <ol>
     *   <li>kS / kff —— Tune6.5 Feedforward（多点稳态回归，每档停 ≥2 s）</li>
     *   <li>kp / ki —— Tune6.1 StepResponse（kp 仍需满足 kp×quantStep ≤ 0.05）</li>
     *   <li>tauVMax —— Tune6.6 VelocityFilter</li>
     *   <li>tauBus —— Tune6.7 BusVoltageFilter（要先 enable useVoltageNorm）</li>
     * </ol>
     */
    public static final FlywheelParams SMALL = new FlywheelParams();

    static {
        // ---- 前馈：★ 待实测（当前为大球占位值）----
        SMALL.kS = 0.02945;
        SMALL.kff = 3.8307e-4;

        // ---- PI：★ 待实测 ----
        SMALL.kp = 1.81917e-3;
        SMALL.ki = 2.59224e-3;
        SMALL.maxI = 0.3;

        // ---- 输出限幅（一般与大球相同）----
        SMALL.dutyMin = 0.0;
        SMALL.dutyMax = 1.0;

        // ---- 量化与稳态判据：台阶由编码器/齿比决定，换了机构要重测 ----
        SMALL.quantStep = 20.0;
        SMALL.settleFrac = 0.02;
        SMALL.settleFrames = 10;

        // ---- 速度滤波：★ 待实测（惯量不同，时间常数通常不同）----
        SMALL.tauVMax = 0.030;
        SMALL.tauVMin = 0.008;
        SMALL.eLoFrac = 0.30;
        SMALL.eHiFrac = 0.90;

        // ---- 掉速判定与积分冻结 ----
        SMALL.dropEnterFrac = 3.0;
        SMALL.iFreezeTimeoutSec = 1.5;

        // ---- 母线电压归一化 ----
        SMALL.useVoltageNorm = false;
        SMALL.vBusNominal = 12.0;
        SMALL.vScaleMin = 0.85;
        SMALL.vScaleMax = 1.25;
        SMALL.tauBus = 1.0;
        SMALL.busHoldOffSec = 0.6;
        SMALL.busFreezeTimeoutSec = 2.0;

        // ---- 装配与范围：方向与最大转速按小球机构填 ----
        SMALL.motorReversed = true;
        SMALL.speedMax = 3000;
    }

    // ==================================================================================
    //  装配 / 复制
    // ==================================================================================

    /** 供 Dashboard 反射与离线脚本使用的空构造（字段无工程默认值，请用 {@link #copyOf} 取默认） */
    public FlywheelParams() {
    }

    /**
     * @param src 源参数集（为 null 时返回空实例）
     * @return 一份独立副本（字段与 src 相同，但不共享引用）
     */
    public static FlywheelParams copyOf(FlywheelParams src) {
        FlywheelParams copy = new FlywheelParams();
        copy.copyFrom(src);
        return copy;
    }

    /**
     * 把本参数集灌进计算器（每帧调用，Dashboard 改动即时生效）。
     *
     * @param calc 目标计算器；为 null 时直接返回
     */
    public void applyTo(FlywheelCalculator calc) {
        if (calc == null) {
            return;
        }
        calc.kS = kS;
        calc.kff = kff;
        calc.kp = kp;
        calc.ki = ki;
        calc.maxI = maxI;
        calc.dutyMin = dutyMin;
        calc.dutyMax = dutyMax;
        calc.quantStep = quantStep;
        calc.settleFrac = settleFrac;
        calc.settleFrames = settleFrames;
        calc.tauVMax = tauVMax;
        calc.tauVMin = tauVMin;
        calc.eLoFrac = eLoFrac;
        calc.eHiFrac = eHiFrac;
        calc.dropEnterFrac = dropEnterFrac;
        calc.iFreezeTimeoutSec = iFreezeTimeoutSec;
        calc.useVoltageNorm = useVoltageNorm;
        calc.vBusNominal = vBusNominal;
        calc.vScaleMin = vScaleMin;
        calc.vScaleMax = vScaleMax;
        calc.tauBus = tauBus;
        calc.busHoldOffSec = busHoldOffSec;
        calc.busFreezeTimeoutSec = busFreezeTimeoutSec;
    }

    /**
     * 用另一套参数覆盖本参数集（整定辅助：把标定结果在两个飞轮之间互相复制）。
     *
     * @param other 源参数集；为 null 或 this 时直接返回
     */
    public void copyFrom(FlywheelParams other) {
        if (other == null || other == this) {
            return;
        }
        kS = other.kS;
        kff = other.kff;
        kp = other.kp;
        ki = other.ki;
        maxI = other.maxI;
        dutyMin = other.dutyMin;
        dutyMax = other.dutyMax;
        quantStep = other.quantStep;
        settleFrac = other.settleFrac;
        settleFrames = other.settleFrames;
        tauVMax = other.tauVMax;
        tauVMin = other.tauVMin;
        eLoFrac = other.eLoFrac;
        eHiFrac = other.eHiFrac;
        dropEnterFrac = other.dropEnterFrac;
        iFreezeTimeoutSec = other.iFreezeTimeoutSec;
        useVoltageNorm = other.useVoltageNorm;
        vBusNominal = other.vBusNominal;
        vScaleMin = other.vScaleMin;
        vScaleMax = other.vScaleMax;
        tauBus = other.tauBus;
        busHoldOffSec = other.busHoldOffSec;
        busFreezeTimeoutSec = other.busFreezeTimeoutSec;
        motorReversed = other.motorReversed;
        speedMax = other.speedMax;
    }
}
