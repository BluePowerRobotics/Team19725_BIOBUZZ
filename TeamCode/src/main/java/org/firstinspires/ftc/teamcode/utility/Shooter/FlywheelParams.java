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
 * 两套实例参数在 Dashboard 上分别可见、分别整定。
 *
 * <p><b>为什么参数不放 {@link FlywheelCalculator} 里</b>：计算器是「一套算法」，参数是「一台机器」。
 * 分开后计算器可以换一套参数就复算另一台飞轮的历史日志，两个飞轮也不必各写一份控制代码。
 *
 * <p><b>当前控制律</b>（SimpleShooter，无滤波、无 boost）：
 * <pre>
 *   e      = ω* - ω
 *   s      = clamp(V_nom / V_bus_raw, sMin, sMax)       // 原始电压，只作用于前馈
 *   ff     = s·(kS·sgn(ω*) + kff·ω*)
 *   P/I    = 直接使用 e（不做速度滤波）
 *   duty   = sat( ff + kp·e + ki·∫e , 0, 1 )
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

    /**
     * 积分门限 (tick/s)：只有 {@code |error| < iZone} 才累加积分。
     * Dashboard 可调；在 BIG / SMALL 两个参数块中分别赋值。
     */
    public double iZone;

    /** I 项上限 (占空比)：对 {@code ki·∫e} 限幅，同时回算积分量（抗饱和） */
    public double maxI;

    // ---------------- 输出限幅 ----------------

    /** 输出下限 (占空比) */
    public double dutyMin;

    /** 输出上限 (占空比) */
    public double dutyMax;

    // ---------------- 量化与稳态判据 ----------------

    /** 速度量化台阶 (tick/s)：编码器测速最小分辨率（用于误差带下限与 kp 告警） */
    public double quantStep;

    /** 稳态误差带比例：{@code eBand = max(settleFrac·|ω*|, 2·quantStep)} */
    public double settleFrac;

    /** 连续落在误差带内多少帧算「到位」（10 帧 ≈ 0.1 s） */
    public int settleFrames;

    // ---------------- 母线电压归一化 ----------------

    /** 是否启用母线电压归一化（只乘前馈，直接使用原始电压） */
    public boolean useVoltageNorm;

    /** 额定母线电压 (V)：前馈按该电压标定 */
    public double vBusNominal;

    /** 归一化系数下限（欠压时前馈最多放大到 1/sMin 倍） */
    public double vScaleMin;

    /** 归一化系数上限（过压时前馈最多缩小到 1/sMax 倍） */
    public double vScaleMax;

    // ---------------- 装配与范围 ----------------

    /** 电机方向：由飞轮接线极性决定（构造时下发一次），与整定参数无关 */
    public boolean motorReversed;

    /** 该飞轮最大转速 (tick/s)，目标转速会被钳制到 [0, speedMax] */
    public int speedMax;

    // ==================================================================================
    //  大球飞轮参数（BIG）—— 2026-09-04 实机整定，8 工作点回归 R²=0.9949
    //  SimpleShooter：无速度/电压滤波；BIG / SMALL 各自维护 iZone
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
        // 2026-10 按 09-02 实测日志重定：单步增益 kp·Kd（Kd≈2740 tick/s 每单位 duty）
        // 从 6.0 降到 2.7 → 稳态极限环由 ±2 个量化台阶收到 ±1 个，见 doc/现在的控制系统.md §2.15。
        // 2026-10-27 实机复测（kp=1.9095e-3，7 发）：回带中位 0.31 s、duty_sd 0.013~0.017，
        // 比 kp=1.0e-3 的回带 0.91 s 明显改善，且仍在判据①②内 → 采用 1.9095e-3。
        BIG.kp = 1.9095e-3;
        BIG.ki = 2.59224e-3;
        BIG.maxI = 0.3;
        // ---- 输出限幅 ----
        BIG.dutyMin = 0.0;
        BIG.dutyMax = 1.0;
        // ---- 量化与稳态判据 ----
        BIG.quantStep = 20.0;      // 实测编码器测速台阶
        BIG.settleFrac = 0.02;     // ±2%
        BIG.settleFrames = 10;     // 10 帧 ≈ 0.1 s
        BIG.iZone = 120.0;         // 积分门限 (tick/s)
        // ---- 母线电压归一化（直接使用原始采样）----
        // 2026-10-27 起默认开启：09-02 两份实测（大轮 20:12 vscale≡1、小轮 20:16 vscale≈0.96）
        // SimpleShooter 直接使用原始母线电压做归一化，不再做低通或冻结。
        BIG.useVoltageNorm = true;
        BIG.vBusNominal = 12.0;
        BIG.vScaleMin = 0.85;
        BIG.vScaleMax = 1.25;
        // ---- 装配与范围 ----
        BIG.motorReversed = true;
        BIG.speedMax = 3000;
    }

    // ==================================================================================
    //  小球飞轮参数（SMALL）—— 2026-10-27 实机整定完成
    // ==================================================================================

    /**
     * 小球飞轮参数。改这里即改小球；Dashboard 上显示为「FlywheelParams.SMALL」分组。
     *
     * <p>2026-10-27 按 09-02 20:16 的实测日志定稿（0→1900→1700，8 发）：
     * 稳态 1700 误差 +0.10 tick/s、duty 0.6534±0.0206、积分中位 −1.6（判据③通过），
     * 回带中位 0.19 s、最大 0.32 s（判据④ 0.43 s），`vscale≈0.96`（归一化已开）。
     * 其静态模型 `duty/s = 0.0295 + 3.83e-4·ω` 与大球几乎一致，故 kS/kff 沿用大球量级。
     */
    public static final FlywheelParams SMALL = new FlywheelParams();

    static {
        // ---- 前馈（Tune6.5：多点稳态回归 d_ss = kS + kff·ω，实测 kff=3.8346e-4）----
        SMALL.kS = 0.02945;
        SMALL.kff = 3.8346e-4;
        // ---- PI（Tune6.1；kp×20 = 0.0348 ≤ 0.05）----
        SMALL.kp = 1.739e-3;
        SMALL.ki = 2.50771e-3;
        SMALL.maxI = 0.3;
        // ---- 输出限幅（一般与大球相同）----
        SMALL.dutyMin = 0.0;
        SMALL.dutyMax = 1.0;
        // ---- 量化与稳态判据：台阶由编码器/齿比决定，换了机构要重测 ----
        SMALL.quantStep = 20.0;
        SMALL.settleFrac = 0.02;
        SMALL.settleFrames = 10;
        SMALL.iZone = 120.0;       // 积分门限 (tick/s)
        // ---- 母线电压归一化（2026-10-27 实测已开：vscale≈0.96，前馈补齐母线跌落）----
        SMALL.useVoltageNorm = true;
        SMALL.vBusNominal = 12.0;
        SMALL.vScaleMin = 0.85;
        SMALL.vScaleMax = 1.25;
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
        calc.iZone = iZone;
        calc.useVoltageNorm = useVoltageNorm;
        calc.vBusNominal = vBusNominal;
        calc.vScaleMin = vScaleMin;
        calc.vScaleMax = vScaleMax;
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
        iZone = other.iZone;
        useVoltageNorm = other.useVoltageNorm;
        vBusNominal = other.vBusNominal;
        vScaleMin = other.vScaleMin;
        vScaleMax = other.vScaleMax;
        motorReversed = other.motorReversed;
        speedMax = other.speedMax;
    }
}
