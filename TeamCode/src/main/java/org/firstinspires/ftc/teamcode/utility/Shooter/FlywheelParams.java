package org.firstinspires.ftc.teamcode.utility.Shooter;

import com.acmerobotics.dashboard.config.Config;

/**
 * 飞轮速度环的<b>完整参数集</b>（纯数据类，无硬件依赖）。
 *
 * <p>把「一套飞轮控制参数」封装成一个对象，是为了满足实机上<b>两个飞轮</b>（大球 / 小球）的需求：
 * 二者的惯量、齿比、静摩擦都不同，必须各持一份参数，且能在 FTC Dashboard 上分组、分别整定。
 * 类内的两个静态实例 {@link #BIG} / {@link #SMALL} 就是这两套参数，
 * 由 {@code Controllers.Turret.Shooter} 在构造两个发射器时分别传入，并由
 * {@link #applyTo(FlywheelCalculator)} 每帧灌进计算器（Dashboard 改动因此实时生效）。
 *
 * <p><b>为什么参数不放 {@code FlywheelCalculator} 里</b>：计算器是「一套算法」，参数是「一台机器」；
 * 分开后计算器可以不持状态地做离线复算（同一个实例换一套参数即可回放另一台飞轮的数据），
 * 两个飞轮也不再需要两份重复的控制代码。
 *
 * <p><b>当前这套控制律</b>（第 4 代，2026-10-22 起）：
 * <pre>
 *   e_raw  = ω* - ω
 *   s      = clamp(V_nom / V_busFilt, sMin, sMax)        // 母线电压归一化，只作用于前馈
 *   ff     = s·(kS·sgn(ω*) + kff·ω*)
 *   ωf     = EMA(ω, τ(|e_raw|))                          // 速度滤波，误差越大 τ 越小
 *   duty   = sat( ff + kp·(ω*-ωf) + ki·∫(ω*-ωf) , 0, 1 )
 * </pre>
 * 与第 3 代相比：<b>删掉了 boost 掉速补偿</b>（实测其饱和段贡献恒为 0，唯一作用是抹掉退饱和段、
 * 结构性造成 +50~90 tick/s 过冲），改由「积分冻结 + 误差带」来处理掉速与过冲。
 *
 * <p>默认值取自 2026-09-04 大球飞轮实机整定（kS=0.02945、kff=0.00038307、kp=0.00181917、
 * ki=0.00259224，8 工作点回归 R²=0.9949）。
 */
@Config
public class FlywheelParams {

    // ==================== 前馈 ====================

    /**
     * 静摩擦前馈 (占空比)：稳态模型 {@code d_ss(ω) = kS + kff·ω} 的截距，
     * 即「只对抗摩擦、不产生转速」的那份占空比，按 {@code kS·sgn(ω*)} 施加（ω*=0 时自动为 0）。
     */
    public double kS = 0.02945;

    /** 稳态前馈增益 (占空比)/(tick/s)，即稳态模型的斜率 */
    public double kff = 3.8307e-4;

    // ==================== 反馈 ====================

    /**
     * 比例增益 (占空比)/(tick/s)。
     *
     * <p>量化台阶自检：{@code kp × quantStep} 是「一格编码器测量噪声被放大成多少占空比」，
     * 也就是量化极限环的幅值下限。工程判据 ≤0.05 合格（当前 0.036），≤0.02 好，&gt;0.1 会跑出教科书极限环。
     */
    public double kp = 1.81917e-3;

    /** 积分增益 (占空比)/(tick/s·s) */
    public double ki = 2.59224e-3;

    /** I 项上限 (占空比)：对积分项 ki·∫e 的限幅（同时回算积分量，抗饱和） */
    public double maxI = 0.3;

    // ==================== 输出限幅 ====================

    /** 输出下限 (占空比) */
    public double dutyMin = 0.0;

    /** 输出上限 (占空比) */
    public double dutyMax = 1.0;

    // ==================== 量化与稳态判据 ====================

    /**
     * 速度量化台阶 (tick/s)：编码器测速的最小分辨率，实测为 20 tick/s。
     * 用于三处：稳态误差带下限、{@code kp×台阶} 告警、速度滤波拐点计算。
     */
    public double quantStep = 20.0;

    /** 稳态判据中「相对误差」的比例：误差带取 {@code max(settleFrac·|ω*|, 2·quantStep)} */
    public double settleFrac = 0.02;

    /** 连续落在此误差带内多少帧算「到位」（10 帧 ≈ 0.1 s） */
    public int settleFrames = 10;

    // ==================== 速度滤波（误差调度 EMA） ====================

    /** 稳态时的速度滤波时间常数 (s)：越小噪声越小，但相位滞后越大 */
    public double tauVMax = 0.030;

    /** 大误差时的速度滤波时间常数 (s)：约 1 个控制周期，几乎不滤波，保证加速/退饱和不滞后 */
    public double tauVMin = 0.008;

    /** 误差调度下拐点：{@code |e_raw| ≤ eLoFrac·eBand} 时用 tauVMax */
    public double eLoFrac = 0.30;

    /** 误差调度上拐点：{@code |e_raw| ≥ eHiFrac·eBand} 时用 tauVMin */
    public double eHiFrac = 0.90;

    // ==================== 掉速判定（误差带滞回） ====================

    /** 进入掉速态的阈值（倍 eBand）：{@code |e_raw| > dropEnterFrac·eBand} 判定为掉速 */
    public double dropEnterFrac = 3.0;

    /**
     * 掉速态下积分最多冻结多久 (s)：超时后强制清零积分。
     * 防「误差长期卡在带外 → 积分永久失效 → 稳态误差无法消除」的死区。
     */
    public double iFreezeTimeoutSec = 1.5;

    // ==================== 母线电压归一化 ====================

    /** 是否启用母线电压归一化（只作用于前馈项）。默认关闭：先在台架上用 T1~T4 协议验证再打开 */
    public boolean useVoltageNorm = false;

    /** 额定母线电压 (V)：前馈按该电压标定，电池电压偏离时按比例放大/缩小前馈 */
    public double vBusNominal = 12.0;

    /** 电压归一化系数的下限（欠压时最多把前馈放大到 1/sMin 倍） */
    public double vScaleMin = 0.85;

    /** 电压归一化系数的上限（过压时最多把前馈缩小到 1/sMax 倍） */
    public double vScaleMax = 1.25;

    /**
     * 母线电压低通时间常数 (s)。必须远大于单发发射造成的电压跌落时长（≈0.4 s），
     * 否则滤波器会把「单发掉压」当成「电池没电」，在恢复期末把前馈放大到 >1，
     * 人为造出一次饱和（第 3 代 boost 的翻版）。取 1.0 s 时单发跌落只漏进 7%。
     */
    public double tauBus = 1.0;

    /**
     * 掉速结束后的电压采样 hold-off (s)：掉速态刚结束时电池电压仍在恢复（RC 回弹），
     * 继续冻结一小段时间再恢复采样，避免把回弹中段采成新的稳态值。
     */
    public double busHoldOffSec = 0.6;

    /** 电压采样最长冻结时长 (s)：超时强制重新采样（防长期冻结导致 s 永久失真） */
    public double busFreezeTimeoutSec = 2.0;

    // ==================== 装配与范围 ====================

    /** 电机方向：由飞轮装配方向（接线极性）决定，与整定参数无关；只在构造时下发一次 */
    public boolean motorReversed = true;

    /** 飞轮最大转速 (tick/s)，目标转速会被钳制到 [0, speedMax] */
    public int speedMax = 3000;

    // ==================== 两套实机参数 ====================

    /**
     * 大球飞轮参数（实机 2026-09-04 整定值）。
     *
     * <p>这是 Dashboard 上「FlywheelParams」分组里的第一组；改这里即改大球飞轮。
     */
    public static FlywheelParams BIG = new FlywheelParams();

    /**
     * 小球飞轮参数。
     *
     * <p><b>尚未标定</b>：2026-09-05 18:55 唯一一次小飞轮实测（kS=0、kff 沿用大球）稳态误差
     * +10.9 / +27.3 tick/s（系统性欠给，说明 kS/kff 都偏小）。下面暂时复制大球初值，
     * 上机后按 {@code doc/飞轮调参指南.md} 的「多点稳态回归」重新标 kS/kff，
     * 再用 `VelocityFilterTuner` / `BusVoltageFilterTuner` 定两个滤波参数。
     */
    public static FlywheelParams SMALL = new FlywheelParams();

    /** 供离线脚本 / Tester 使用的空构造（字段取上面的默认值） */
    public FlywheelParams() {
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
     * @param other 源参数集；为 null 时直接返回
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
