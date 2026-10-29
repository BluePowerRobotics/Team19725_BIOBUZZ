package org.firstinspires.ftc.teamcode.utility.pastcalculater;

/**
 * 分代 Tuner 共用的数学工具（最小二乘、一阶阶跃辨识）。
 *
 * <p>算法与 {@code OpModes/Tuning/StepResponseTuner}（Tune6.1）保持一致，
 * 这样各代给出的「建议 kp/ki」能直接和第 4 代的整定结果放在一起比。
 */
public final class PastTuneSupport {

    private PastTuneSupport() {
    }

    /**
     * 最小二乘直线拟合 {@code y = slope·x + intercept}。
     *
     * @return {@code {slope, intercept}}；样本 &lt; 2 或 x 无变化时返回 {@code null}
     */
    public static double[] leastSquares(double[] x, double[] y) {
        int n = Math.min(x.length, y.length);
        if (n < 2) {
            return null;
        }
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < n; i++) {
            sx += x[i];
            sy += y[i];
            sxx += x[i] * x[i];
            sxy += x[i] * y[i];
        }
        double den = n * sxx - sx * sx;
        if (Math.abs(den) < 1e-12) {
            return null;
        }
        double slope = (n * sxy - sx * sy) / den;
        double intercept = (sy - slope * sx) / n;
        return new double[]{slope, intercept};
    }

    /**
     * 一阶阶跃响应辨识（与 Tune6.1 同口径）。
     *
     * @param time           采样时刻 (s)，从阶跃开始算
     * @param velocity       转速 (tick/s)
     * @param count          有效样本数
     * @param d0             阶跃前占空比
     * @param dStep          阶跃后占空比
     * @param steadyWindowSec 取稳态均值的末段长度 (s)
     * @param tRecover       期望恢复时间 (s)
     * @param ratio          tRecover / τc，取 3~5
     * @return {@code {omega0, omegaSS, Kd, tau, kff, kp, ki}}；无效返回 {@code null}
     */
    public static double[] fitStep(double[] time, double[] velocity, int count,
                                   double d0, double dStep,
                                   double steadyWindowSec, double tRecover, double ratio) {
        if (count < 10) {
            return null;
        }
        double tEnd = time[count - 1];

        // 稳态均值：末段 steadyWindowSec
        double sum = 0;
        int n = 0;
        for (int i = count - 1; i >= 0; i--) {
            if (time[i] < tEnd - steadyWindowSec) {
                break;
            }
            sum += velocity[i];
            n++;
        }
        double omegaSS = n > 0 ? sum / n : 0;
        double omega0 = velocity[0];

        double deltaD = dStep - d0;
        double deltaW = omegaSS - omega0;
        if (Math.abs(deltaD) < 1e-6 || Math.abs(deltaW) < 1e-6) {
            return null;
        }
        double kd = deltaW / deltaD; // (tick/s)/占空比
        if (kd <= 0) {
            return null;
        }

        // τ：第一次达到 63.2% 增量的时刻
        double targetW = omega0 + 0.632 * deltaW;
        double tau = Double.NaN;
        for (int i = 0; i < count; i++) {
            if (velocity[i] >= targetW) {
                tau = time[i];
                break;
            }
        }
        if (Double.isNaN(tau)) {
            return null;
        }

        double kff = 1.0 / kd;
        double tc = tRecover / ratio;
        double kp = tau / (kd * tc);
        double ki = 1.0 / (kd * tc);
        return new double[]{omega0, omegaSS, kd, tau, kff, kp, ki};
    }

    /** {@code {slope, intercept}} → 一行文本（方便遥测）。 */
    public static String describe(double[] fit) {
        if (fit == null) {
            return "拟合失败（样本不足或 x 无变化）";
        }
        return String.format(java.util.Locale.US, "slope=%.8g intercept=%.6g", fit[0], fit[1]);
    }
}
