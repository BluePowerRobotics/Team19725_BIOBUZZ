package org.firstinspires.ftc.teamcode.Auto.AutomaticCalibration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * {@link SvaIdentifier} 单元测试：验证最小二乘在无噪声数据上精确还原 kS/kV/kA、
 * 残差计算正确，以及激励退化时抛出异常。
 */
public class SvaIdentifierTest {

    /** 与 PARAMS 同量级的参考真值 */
    private static final double KS_TRUE = 1.2019792244145228;
    private static final double KV_TRUE = 3.5261972751626936e-4;
    private static final double KA_TRUE = 7.0e-5;

    // ==================== kS / kV 两参数拟合 ====================

    @Test
    public void fitKsKv_recoversExactParameters_positiveVelocities() {
        double[] v = {512, 1024, 4096, 8192, 16384, 24576, 25012, 30000};
        double[] s = new double[v.length];
        double[] volts = new double[v.length];
        for (int i = 0; i < v.length; i++) {
            s[i] = Math.signum(v[i]);
            volts[i] = KS_TRUE * s[i] + KV_TRUE * v[i];
        }

        SvaIdentifier.Result r = SvaIdentifier.fitKsKv(s, v, volts);

        assertEquals(KS_TRUE, r.kS, 1e-6);
        assertEquals(KV_TRUE, r.kV, 1e-12);
        assertTrue("完美数据残差应接近 0", r.rmsResidual < 1e-6);
        assertEquals(v.length, r.nSamples);
    }

    @Test
    public void fitKsKv_recoversExactParameters_bothSigns() {
        double[] v = {-30000, -12000, -3000, -500, 500, 3000, 12000, 30000};
        double[] s = new double[v.length];
        double[] volts = new double[v.length];
        for (int i = 0; i < v.length; i++) {
            s[i] = Math.signum(v[i]);
            volts[i] = KS_TRUE * s[i] + KV_TRUE * v[i];
        }

        SvaIdentifier.Result r = SvaIdentifier.fitKsKv(s, v, volts);

        assertEquals(KS_TRUE, r.kS, 1e-6);
        assertEquals(KV_TRUE, r.kV, 1e-12);
    }

    @Test
    public void fitKsKv_rmsResidualReflectsNoise() {
        double[] v = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
        double[] s = {1, 1, 1, 1, 1, 1, 1, 1};
        double[] volts = new double[v.length];
        // 噪声模式 {+1,-1,-1,+1,0,...} 对 [1, v] 两个基向量都正交（Σn=0 且 Σn·v=0），
        // 因此最小二乘拟合不会把它吸收掉，RMS 残差应精确等于 0.1·√(4/8)
        double[] noise = {0.1, -0.1, -0.1, 0.1, 0, 0, 0, 0};
        for (int i = 0; i < v.length; i++) {
            volts[i] = KS_TRUE * s[i] + KV_TRUE * v[i] + noise[i];
        }

        SvaIdentifier.Result r = SvaIdentifier.fitKsKv(s, v, volts);

        assertEquals(0.070711, r.rmsResidual, 1e-6);
        assertEquals(KS_TRUE, r.kS, 1e-6);
        assertEquals(KV_TRUE, r.kV, 1e-12);
    }

    @Test(expected = IllegalArgumentException.class)
    public void fitKsKv_degenerateExcitation_throws() {
        // 所有轮速相同 → 无法区分 kS 与 kV
        double[] v = {5000, 5000, 5000, 5000};
        double[] s = {1, 1, 1, 1};
        double[] volts = {3, 3, 3, 3};
        SvaIdentifier.fitKsKv(s, v, volts);
    }

    // ==================== kA 一参数拟合 ====================

    @Test
    public void fitKa_recoversExactParameter_givenKsKv() {
        double[] v = {1000, 4000, 9000, 16000, 25000};
        double[] a = {40, -35, 60, -55, 25};
        double[] s = new double[v.length];
        double[] volts = new double[v.length];
        for (int i = 0; i < v.length; i++) {
            s[i] = Math.signum(v[i]);
            volts[i] = KS_TRUE * s[i] + KV_TRUE * v[i] + KA_TRUE * a[i];
        }

        SvaIdentifier.Result r = SvaIdentifier.fitKaDetailed(s, v, a, volts, KS_TRUE, KV_TRUE);

        assertEquals(KA_TRUE, r.kA, 1e-12);
        assertTrue(r.rmsResidual < 1e-9);
    }

    @Test
    public void fitKa_convenienceOverload_matchesDetailed() {
        double[] v = {1000, 4000, 9000, 16000, 25000};
        double[] a = {40, -35, 60, -55, 25};
        double[] s = {1, 1, 1, 1, 1};
        double[] volts = new double[v.length];
        for (int i = 0; i < v.length; i++) {
            volts[i] = KS_TRUE * s[i] + KV_TRUE * v[i] + KA_TRUE * a[i];
        }

        double kA = SvaIdentifier.fitKa(s, v, a, volts, KS_TRUE, KV_TRUE);

        assertEquals(KA_TRUE, kA, 1e-12);
    }

    @Test(expected = IllegalArgumentException.class)
    public void fitKa_noAccelerationExcitation_throws() {
        double[] v = {1000, 4000, 9000};
        double[] a = {0, 0, 0};
        double[] s = {1, 1, 1};
        double[] volts = {1, 2, 3};
        SvaIdentifier.fitKa(s, v, a, volts, KS_TRUE, KV_TRUE);
    }

    // ==================== 完整辨识与采样点入口 ====================

    @Test
    public void identify_fullModel_recoversAllThree() {
        // 正交设计：Σv=0、Σa=0、Σ(a·v)=0，使三参数互不混淆，可精确还原
        double[] v = {-24000, -12000, 12000, 24000};
        double[] a = {50, -50, -50, 50};
        double[] s = {1, 1, 1, 1};
        double[] volts = new double[v.length];
        for (int i = 0; i < v.length; i++) {
            volts[i] = KS_TRUE * s[i] + KV_TRUE * v[i] + KA_TRUE * a[i];
        }

        SvaIdentifier.Result r = SvaIdentifier.identify(s, v, a, volts);

        assertEquals(KS_TRUE, r.kS, 1e-6);
        assertEquals(KV_TRUE, r.kV, 1e-12);
        assertEquals(KA_TRUE, r.kA, 1e-12);
        assertTrue("正交设计下残差应接近 0", r.rmsResidual < 1e-6);
    }

    @Test
    public void fromSamples_skipsDeadZoneSamples() {
        // 构造：静止（|v| 小于阈值）的点应被剔除
        List<CalibrationSample> samples = new ArrayList<>();
        samples.add(sample(0, 0));          // 静止，剔除
        samples.add(sample(5, 0));          // 静止，剔除
        double[] v = {-24000, -12000, 12000, 24000};
        double[] a = {50, -50, -50, 50};
        for (int i = 0; i < v.length; i++) {
            samples.add(sample(v[i], a[i]));
        }

        SvaIdentifier.Result r = SvaIdentifier.fromSamples(samples, 20.0);

        assertEquals(v.length, r.nSamples);
        assertEquals(KS_TRUE, r.kS, 1e-6);
        assertEquals(KV_TRUE, r.kV, 1e-12);
        assertEquals(KA_TRUE, r.kA, 1e-12);
    }

    private static CalibrationSample sample(double vTicks, double aTicks) {
        double volts = KS_TRUE * Math.signum(vTicks) + KV_TRUE * vTicks + KA_TRUE * aTicks;
        return new CalibrationSample("T", 0, 12.0, volts / 12.0, volts,
                vTicks, aTicks, 0, 0, 0);
    }
}
