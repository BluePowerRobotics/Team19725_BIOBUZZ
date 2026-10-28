package org.firstinspires.ftc.teamcode.utility.pastcalculater;

/**
 * 可对比的飞轮控制代际：DECODE 赛季以来本队用过的单飞轮速度环方案。
 *
 * <p>第 1、2 代都是 {@code PIDSVAController}（同一套 kP/kI/kD + kS/kV/kA 结构，只是槽位配法不同），
 * 按“合起来”的决定合并为 {@link #GEN1_PIDSVA} 一代。
 *
 * <p>本包里的实现都是**精简版**：只保留影响 {@code duty} 的部分，
 * 供「同一段 ω 序列喂给各代、只比 duty」的对比实验使用，**不接入正式机器人控制**。
 */
public enum PastGen {

    GEN1_PIDSVA(1, "Gen1/2 PIDSVA",
            "kP·e + kI·∫e + kD·de + kS·sgn(ω*) + kV·ω* + kA·dω*/dt，|e|<iZone 才积，无掉速补偿、无电压归一化"),

    GEN3_BOOST(3, "Gen3 Boost",
            "kS·sgn(ω*) + kff·ω* + kp·e + ki·∫e(|e|<iZone=500) + 掉速 boost(kBoost·e, ≤boostMax)，无电压归一化"),

    GEN4_CURRENT(4, "Gen4 现行",
            "速度滤波 τV 调度 + 掉速冻结积分 + 母线电压归一化，无 boost、无 kD");

    /** 在 Dashboard 里用来选择的编号（1 / 3 / 4） */
    public final int id;
    /** 显示名 */
    public final String label;
    /** 一句话控制律说明 */
    public final String note;

    PastGen(int id, String label, String note) {
        this.id = id;
        this.label = label;
        this.note = note;
    }

    /** Dashboard 的数字 → 代际；未知编号返回 {@link #GEN4_CURRENT}。 */
    public static PastGen fromId(int id) {
        for (PastGen g : values()) {
            if (g.id == id) {
                return g;
            }
        }
        return GEN4_CURRENT;
    }

    @Override
    public String toString() {
        return label;
    }
}
