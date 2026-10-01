package org.firstinspires.ftc.teamcode.utility.Algebra;

/**
 * 线性二元数据统计类
 * <p>
 * 通过 {@link #add(double, double)} 逐个添加数据点，累积求和量，
 * 实时计算均值、总体方差、协方差、相关系数及一元线性回归系数。
 * 所有方差均采用总体方差（分母为 n）计算。
 */
public class LinearStatistics {
    private static final double EPSILON = 1e-12;

    private int n = 0;
    private double sumX = 0;
    private double sumY = 0;
    private double sumXX = 0;
    private double sumYY = 0;
    private double sumXY = 0;

    /**
     * 添加一个数据点 (x, y)
     */
    public void add(double x, double y) {
        n++;
        sumX += x;
        sumY += y;
        sumXX += x * x;
        sumYY += y * y;
        sumXY += x * y;
    }

    /**
     * 返回数据点数量
     */
    public int sampleCount() {
        return n;
    }

    /**
     * x 的均值
     */
    public double avgX() {
        return n == 0 ? 0 : sumX / n;
    }

    /**
     * y 的均值
     */
    public double avgY() {
        return n == 0 ? 0 : sumY / n;
    }

    /**
     * x 的总体方差
     */
    public double varX() {
        return n == 0 ? 0 : sumXX / n - avgX() * avgX();
    }

    /**
     * y 的总体方差
     */
    public double varY() {
        return n == 0 ? 0 : sumYY / n - avgY() * avgY();
    }

    /**
     * x 与 y 的总体协方差
     */
    public double covXY() {
        return n == 0 ? 0 : sumXY / n - avgX() * avgY();
    }

    /**
     * x 与 y 的相关系数
     */
    public double corrXY() {
        double varX = varX();
        double varY = varY();
        if (varX < EPSILON || varY < EPSILON) {
            return 0;
        }
        return covXY() / Math.sqrt(varX * varY);
    }

    /**
     * 线性回归系数（斜率）
     */
    public double weight() {
        double varX = varX();
        if (varX < EPSILON) {
            return 0;
        }
        return covXY() / varX;
    }

    /**
     * 线性回归截距
     */
    public double bias() {
        return avgY() - weight() * avgX();
    }

    /**
     * 决定系数 R²
     */
    public double calculateR2() {
        double corr = corrXY();
        return corr * corr;
    }
}