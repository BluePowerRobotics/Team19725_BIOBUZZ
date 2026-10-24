package org.firstinspires.ftc.teamcode.Auto.AutomaticCalibration;

import java.util.List;

/**
 * 底盘前馈 S/V/A（kS / kV / kA）辨识器。
 *
 * <h3>数学模型（与 {@code MecanumDrive} 链路完全同构）</h3>
 * Road Runner 模板对每个轮子施加的电压为：
 * <pre>
 *   u_volts = kS·sign(v) + kV·v + kA·a          （v、a 为「编码器 tick 单位」）
 *   power   = u_volts / V_batt
 * </pre>
 * 因此反解时把实测功率还原成有效电压 {@code voltsEff = power × V_batt}，用它对轮速
 * 做线性回归即可得到与 {@code MecanumDrive.Params} 同量纲的：
 * <ul>
 *   <li>{@code kS}：V（伏特），静态摩擦补偿；</li>
 *   <li>{@code kV}：V / (tick/s)，速度增益；</li>
 *   <li>{@code kA}：V / (tick/s²)，加速度增益。</li>
 * </ul>
 * 由于使用了 {@code voltsEff}（已除过电池电压），辨识结果与电池电压无关，
 * 可直接与 {@code PARAMS} 中的数值比较量级。
 *
 * <h3>辨识算法</h3>
 * <ol>
 *   <li><b>kS / kV 两参数最小二乘</b>：以 {@code sign(v)} 与 {@code v} 为基，
 *       对 {@code V} 做普通最小二乘（正规方程直接解 2×2 线性方程组，无迭代、无需库）；</li>
 *   <li><b>kA 一参数最小二乘</b>：固定上一步的 kS、kV，令残差
 *       {@code y = V − kS·sign(v) − kV·v}，则 {@code kA = Σ(a·y) / Σ(a²)}。</li>
 * </ol>
 * 两种拟合都返回拟合的 RMS 残差（伏特），用于评估模型质量；残差过大说明存在打滑、
 * 编码器噪声或电压跌落，应由标定程序在 Dashboard 上提示。
 *
 * <p>本类纯静态、无硬件依赖，可直接在 JVM 单元测试中用构造数据验证。
 */
public final class SvaIdentifier {

    /** det 小于该阈值即认为正规方程退化（激励不足），拒绝给出 kS/kV。 */
    public static final double DEGENERATE_DET = 1e-9;

    private SvaIdentifier() {
    }

    /** 辨识结果：参数 + 拟合残差 + 参与拟合的样本数（单位见类注释）。 */
    public static final class Result {
        /** 静态摩擦补偿，单位 V */
        public final double kS;
        /** 速度增益，单位 V/(tick/s) */
        public final double kV;
        /** 加速度增益，单位 V/(tick/s²) */
        public final double kA;
        /** 对 {@code V = kS·sign(v)+kV·v+kA·a} 的 RMS 残差，单位 V */
        public final double rmsResidual;
        /** 参与拟合的样本数 */
        public final int nSamples;

        public Result(double kS, double kV, double kA, double rmsResidual, int nSamples) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.rmsResidual = rmsResidual;
            this.nSamples = nSamples;
        }

        @Override
        public String toString() {
            return String.format("kS=%.6f V, kV=%.9f V/(tick/s), kA=%.9f V/(tick/s^2), rms=%.4f V (n=%d)",
                    kS, kV, kA, rmsResidual, nSamples);
        }
    }

    /**
     * kS / kV 两参数最小二乘。
     *
     * @param s     静态项自变量，取 {@code sign(v)}（±1）
     * @param v     轮速，单位 tick/s
     * @param volts 有效电压，单位 V
     * @return kS、kV（kA 置 0）与 RMS 残差
     * @throws IllegalArgumentException 数组长度不一致、样本不足或激励退化
     */
    public static Result fitKsKv(double[] s, double[] v, double[] volts) {
        requireConsistent(s, v, volts);
        int n = s.length;
        if (n < 2) {
            throw new IllegalArgumentException("fitKsKv 至少需要 2 个样本，实际 " + n);
        }

        double sss = 0, ssv = 0, svv = 0;
        double ssy = 0, svy = 0;
        for (int i = 0; i < n; i++) {
            sss += s[i] * s[i];
            ssv += s[i] * v[i];
            svv += v[i] * v[i];
            ssy += s[i] * volts[i];
            svy += v[i] * volts[i];
        }

        double det = sss * svv - ssv * ssv;
        if (Math.abs(det) < DEGENERATE_DET) {
            throw new IllegalArgumentException("fitKsKv 激励退化（det=" + det
                    + "）：轮速范围过小，无法区分 kS 与 kV");
        }

        double kS = (ssy * svv - ssv * svy) / det;
        double kV = (sss * svy - ssv * ssy) / det;

        double sumSq = 0;
        for (int i = 0; i < n; i++) {
            double r = volts[i] - (kS * s[i] + kV * v[i]);
            sumSq += r * r;
        }
        return new Result(kS, kV, 0, Math.sqrt(sumSq / n), n);
    }

    /**
     * 固定 kS、kV 后的一参数 kA 最小二乘。
     *
     * @param s     静态项自变量（±1）
     * @param v     轮速，tick/s
     * @param a     轮加速度，tick/s²
     * @param volts 有效电压，V
     * @param kS    已知静态补偿
     * @param kV    已知速度增益
     * @return kA 与对完整模型 {@code V = kS·s+kV·v+kA·a} 的 RMS 残差
     * @throws IllegalArgumentException 数组长度不一致、样本不足或加速度激励退化
     */
    public static double fitKa(double[] s, double[] v, double[] a, double[] volts, double kS, double kV) {
        return fitKaDetailed(s, v, a, volts, kS, kV).kA;
    }

    /** 同 {@link #fitKa}，额外返回 RMS 残差与样本数。 */
    public static Result fitKaDetailed(double[] s, double[] v, double[] a, double[] volts,
                                       double kS, double kV) {
        requireConsistent(s, v, a, volts);
        int n = s.length;
        if (n < 2) {
            throw new IllegalArgumentException("fitKa 至少需要 2 个样本，实际 " + n);
        }

        double saa = 0, say = 0;
        for (int i = 0; i < n; i++) {
            double y = volts[i] - (kS * s[i] + kV * v[i]);
            saa += a[i] * a[i];
            say += a[i] * y;
        }
        if (saa < DEGENERATE_DET) {
            throw new IllegalArgumentException("fitKa 加速度激励退化（Σa²=" + saa
                    + "）：轨迹加速度过小，无法辨识 kA");
        }
        double kA = say / saa;

        double sumSq = 0;
        for (int i = 0; i < n; i++) {
            double r = volts[i] - (kS * s[i] + kV * v[i] + kA * a[i]);
            sumSq += r * r;
        }
        return new Result(kS, kV, kA, Math.sqrt(sumSq / n), n);
    }

    /**
     * 完整辨识：先由 {@code (s, v, volts)} 拟合 kS/kV，再用 {@code (s, v, a, volts)}
     * 拟合 kA，返回统一量纲的完整结果。
     *
     * @param s     静态项自变量（±1）
     * @param v     轮速，tick/s
     * @param a     轮加速度，tick/s²
     * @param volts 有效电压，V
     */
    public static Result identify(double[] s, double[] v, double[] a, double[] volts) {
        Result base = fitKsKv(s, v, volts);
        return fitKaDetailed(s, v, a, volts, base.kS, base.kV);
    }

    /**
     * 由采样点列表提取激励与观测量后执行完整辨识。
     *
     * @param samples 采样点列表；其 {@code s} 由 {@code sign(wheelVelTicks)} 自动生成，
     *                且仅保留 {@code |wheelVelTicks| > velEps} 的样本
     * @param velEps  轮速死区阈值（tick/s），用于剔除静止点
     */
    public static Result fromSamples(List<CalibrationSample> samples, double velEps) {
        int n = 0;
        for (CalibrationSample c : samples) {
            if (Math.abs(c.wheelVelTicks) > velEps) {
                n++;
            }
        }
        if (n < 2) {
            throw new IllegalArgumentException("fromSamples 有效样本不足：|v|>" + velEps
                    + " 的样本仅 " + n + " 个");
        }
        double[] s = new double[n];
        double[] v = new double[n];
        double[] a = new double[n];
        double[] volts = new double[n];
        int k = 0;
        for (CalibrationSample c : samples) {
            if (Math.abs(c.wheelVelTicks) > velEps) {
                s[k] = Math.signum(c.wheelVelTicks);
                v[k] = c.wheelVelTicks;
                a[k] = c.wheelAccelTicks;
                volts[k] = c.voltsEff;
                k++;
            }
        }
        return identify(s, v, a, volts);
    }

    private static void requireConsistent(double[]... arrays) {
        int n = arrays[0].length;
        for (double[] arr : arrays) {
            if (arr.length != n) {
                throw new IllegalArgumentException("输入数组长度不一致");
            }
        }
    }
}
