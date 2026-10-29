package org.firstinspires.ftc.teamcode.utility.pastcalculater;

import com.acmerobotics.dashboard.config.Config;

/**
 * 第 1/2 代（{@code PIDSVAController}）的参数容器。
 *
 * <p>字段全部是 {@code static}，所以**可以直接在 FTC Dashboard 里改**（改完下一次
 * {@code calculate} 立即生效，不用重新部署）。
 *
 * <p>默认值来自历史提交 {@code 9f05ad2}「第二代 + pidsva 统一测试」与
 * {@code 1268150}「抽出速度环计算器、加 kS」里飞轮用过的量级；
 * 第 1/2 代从未系统标定过，这里只是**可运行的起点**，不是“正确答案”。
 */
@Config
public class Gen1Params {

    // ---------------------------------------------------------------- 反馈

    /** 比例增益 (占空比/(tick/s)) */
    public static double kP = 6.0e-4;

    /** 积分增益 (占空比/(tick/s·s)) */
    public static double kI = 1.2e-3;

    /** 微分增益（历史默认 0：飞轮量化噪声大，kD 基本没用） */
    public static double kD = 0.0;

    // ---------------------------------------------------------------- 前馈

    /** 静摩擦前馈 (占空比)，作用在 sgn(ω*) 上 */
    public static double kS = 0.02945;

    /** 速度前馈 (占空比/(tick/s)) */
    public static double kV = 3.8307e-4;

    /** 加速度前馈 (占空比/(tick/s²))——PIDSVA 的 A 项，用目标转速的导数 */
    public static double kA = 0.0;

    // ---------------------------------------------------------------- 积分门限

    /** 只有 |e| < iZone 才累加积分，否则清零（第 1/2 代的做法） */
    public static double iZone = 500.0;

    /** 积分项限幅 (占空比) */
    public static double maxI = 0.3;

    // ---------------------------------------------------------------- 输出与判据

    public static double dutyMin = 0.0;
    public static double dutyMax = 1.0;

    /** 编码器量化台阶 (tick/s)，用于「稳态」判据与出图对照 */
    public static double quantStep = 20.0;

    /** 稳态判据：|e| ≤ max(settleFrac·|ω*|, 2·quantStep) */
    public static double settleFrac = 0.02;

    /** 稳态判据需要连续满足的帧数 */
    public static int settleFrames = 10;

    /** 恢复到出厂值（Dashboard 改乱了用）。 */
    public static void resetToDefaults() {
        kP = 6.0e-4;
        kI = 1.2e-3;
        kD = 0.0;
        kS = 0.02945;
        kV = 3.8307e-4;
        kA = 0.0;
        iZone = 500.0;
        maxI = 0.3;
        dutyMin = 0.0;
        dutyMax = 1.0;
        quantStep = 20.0;
        settleFrac = 0.02;
        settleFrames = 10;
    }

    /** 一行摘要，写进 CSV 注释。 */
    public static String summary() {
        return String.format(java.util.Locale.US,
                "kP=%.6g kI=%.6g kD=%.6g kS=%.6g kV=%.6g kA=%.6g iZone=%.1f maxI=%.3f",
                kP, kI, kD, kS, kV, kA, iZone, maxI);
    }
}
