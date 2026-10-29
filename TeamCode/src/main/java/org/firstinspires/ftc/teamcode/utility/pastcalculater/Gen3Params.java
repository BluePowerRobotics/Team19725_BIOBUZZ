package org.firstinspires.ftc.teamcode.utility.pastcalculater;

import com.acmerobotics.dashboard.config.Config;

/**
 * 第 3 代（boost 版）的参数容器。
 *
 * <p>数值来自被第 4 代删除的那份实现
 * （{@code git show 0969cbd^:TeamCode/.../utility/Shooter/FlywheelCalculator.java}）
 * 里的硬编码默认值，**原样搬过来**，以便对比实验是“当年的真实参数”。
 *
 * <p>字段全部 {@code static} → Dashboard 可直接改。
 */
@Config
public class Gen3Params {

    /** 静摩擦前馈 (占空比)，作用在 sgn(ω*) 上 */
    public static double kS = 0.02945;

    /** 速度前馈 (占空比/(tick/s)) */
    public static double kff = 3.8307e-4;

    /** 比例增益 */
    public static double kp = 1.81917e-3;

    /** 积分增益 */
    public static double ki = 2.59224e-3;

    /** 积分项限幅 (占空比)，超限回算 integral = maxI/ki */
    public static double maxI = 0.3;

    /** 只有 |e| < iZone 才积；否则清零（第 3 代的关键缺陷：掉速期间积分直接丢） */
    public static double iZone = 500.0;

    // ---------------------------------------------------------------- 掉速 boost

    /** 「进入稳态」的误差阈值：|e| 回到该值内才允许触发 boost（武装） */
    public static double dropThreshold = 100.0;

    /** 掉速判据：转速下降速率超过该值 (tick/s²) 也算掉速 */
    public static double dropRateThreshold = 12000.0;

    /** boost 增益：boost = clamp(kBoost·e, 0, boostMax) */
    public static double kBoost = 0.00362834;

    /** boost 上限 (占空比) */
    public static double boostMax = 0.4;

    /** boost 持续时间 (s)；到点或 e ≤ 0 退出 */
    public static double boostDurationSec = 0.3;

    // ---------------------------------------------------------------- 输出与判据

    public static double dutyMin = 0.0;
    public static double dutyMax = 1.0;

    public static double quantStep = 20.0;
    public static double settleFrac = 0.02;
    public static int settleFrames = 10;

    /** 恢复到出厂值。 */
    public static void resetToDefaults() {
        kS = 0.02945;
        kff = 3.8307e-4;
        kp = 1.81917e-3;
        ki = 2.59224e-3;
        maxI = 0.3;
        iZone = 500.0;
        dropThreshold = 100.0;
        dropRateThreshold = 12000.0;
        kBoost = 0.00362834;
        boostMax = 0.4;
        boostDurationSec = 0.3;
        dutyMin = 0.0;
        dutyMax = 1.0;
        quantStep = 20.0;
        settleFrac = 0.02;
        settleFrames = 10;
    }

    /** 一行摘要，写进 CSV 注释。 */
    public static String summary() {
        return String.format(java.util.Locale.US,
                "kS=%.6g kff=%.6g kp=%.6g ki=%.6g iZone=%.1f dropTh=%.1f "
                        + "dropRate=%.1f kBoost=%.8g boostMax=%.3f boostDur=%.3f",
                kS, kff, kp, ki, iZone, dropThreshold, dropRateThreshold,
                kBoost, boostMax, boostDurationSec);
    }
}
