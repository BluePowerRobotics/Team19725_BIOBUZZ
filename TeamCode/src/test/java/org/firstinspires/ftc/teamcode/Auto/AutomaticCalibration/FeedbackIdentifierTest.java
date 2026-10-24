package org.firstinspires.ftc.teamcode.Auto.AutomaticCalibration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * {@link FeedbackIdentifier} 单元测试：用解析二阶阶跃响应验证超调 / 峰值时间测量、
 * ζ / ωn 反解精度，以及增益修正建议的方向与钳制。
 */
public class FeedbackIdentifierTest {

    private static final double EPS = 1e-9;

    /** 由解析二阶系统生成欠阻尼阶跃响应（终值 1）。 */
    private static double[] secondOrderStep(double[] t, double zeta, double omegaN) {
        double wd = omegaN * Math.sqrt(1 - zeta * zeta);
        double[] y = new double[t.length];
        for (int i = 0; i < t.length; i++) {
            double e = Math.exp(-zeta * omegaN * t[i]);
            y[i] = 1 - e * (Math.cos(wd * t[i])
                    + (zeta / Math.sqrt(1 - zeta * zeta)) * Math.sin(wd * t[i]));
        }
        return y;
    }

    private static double[] timeSeries(double dt, int n) {
        double[] t = new double[n];
        for (int i = 0; i < n; i++) {
            t[i] = i * dt;
        }
        return t;
    }

    // ==================== 阶跃特征量测量 ====================

    @Test
    public void measureStepResponse_extractsOvershootAndPeakTime() {
        double zeta = 0.5, omegaN = 4.0;
        // 6 s 足够长，使稳态值收敛到 1（±1e-3 内）
        double[] t = timeSeries(0.005, 1200);
        double[] y = secondOrderStep(t, zeta, omegaN);

        FeedbackIdentifier.StepMetrics m = FeedbackIdentifier.measureStepResponse(t, y);

        assertTrue(m.valid);
        assertEquals(1.0, m.steadyState, 1e-3);
        // 解析超调 M_p = exp(-πζ/√(1-ζ²)) ≈ 0.1630
        assertEquals(0.1630, m.overshoot, 0.005);
        // 解析峰值时间 t_p = π/ω_d ≈ 0.907 s
        assertEquals(0.907, m.peakTime, 0.01);
        assertTrue("上升时间应为正", m.riseTime > 0);
    }

    @Test
    public void measureStepResponse_invalidWhenTooFewSamples() {
        FeedbackIdentifier.StepMetrics m = FeedbackIdentifier.measureStepResponse(
                new double[]{0, 0.1}, new double[]{0, 1});
        assertFalse(m.valid);
    }

    @Test
    public void measureStepResponse_invalidWhenNoMotion() {
        // 响应恒为 0 → 稳态值非正，判为无效
        double[] t = timeSeries(0.01, 100);
        double[] y = new double[100];
        FeedbackIdentifier.StepMetrics m = FeedbackIdentifier.measureStepResponse(t, y);
        assertFalse(m.valid);
    }

    @Test
    public void measureStepResponse_simpleOvershootIsExact() {
        // 手工构造：峰值 1.2、稳态 1.0 → 超调 20%
        double[] t = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        double[] y = {0, 0.8, 1.2, 1.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        FeedbackIdentifier.StepMetrics m = FeedbackIdentifier.measureStepResponse(t, y);
        assertEquals(0.2, m.overshoot, 1e-9);
        assertEquals(2.0, m.peakTime, 1e-9);
    }

    // ==================== ζ / ωn 反解 ====================

    @Test
    public void closedLoopFromStep_recoversZetaAndOmega() {
        double zeta = 0.5, omegaN = 4.0;
        double[] t = timeSeries(0.005, 1200);
        double[] y = secondOrderStep(t, zeta, omegaN);

        FeedbackIdentifier.ClosedLoopMetrics c = FeedbackIdentifier.identify(t, y);

        assertTrue(c.reliable);
        assertEquals(0.5, c.zeta, 0.02);
        assertEquals(4.0, c.omegaN, 0.15);
    }

    @Test
    public void closedLoopFromStep_overDampedUsesRiseTimeFallback() {
        // ζ=0.9 时超调仅约 0.15%，低于阈值 → 应走上升时间回退，reliable=false
        double[] t = timeSeries(0.005, 1200);
        double[] y = secondOrderStep(t, 0.9, 4.0);

        FeedbackIdentifier.ClosedLoopMetrics c = FeedbackIdentifier.identify(t, y);

        assertFalse(c.reliable);
        assertEquals(1.0, c.zeta, EPS);
        assertTrue("回退估计的带宽应为正", c.omegaN > 0);
    }

    // ==================== 增益修正建议 ====================

    @Test
    public void recommendGains_scalesTowardsTargets() {
        FeedbackIdentifier.ClosedLoopMetrics m = metricsWith(0.5, 3.0);

        FeedbackIdentifier.GainRecommendation rec = FeedbackIdentifier.recommendGains(
                2.0, 0.5, m, 3.0, 0.9, 0.5, 3.0);

        // 带宽目标 = 实测 → 位置增益不变
        assertEquals(1.0, rec.posScale, 1e-9);
        assertEquals(2.0, rec.posGain, 1e-9);
        // 阻尼不足（0.5 < 0.9）→ 速度增益按 1.8 倍放大
        assertEquals(1.8, rec.velScale, 1e-9);
        assertEquals(0.9, rec.velGain, 1e-9);
    }

    @Test
    public void recommendGains_clampsScaleRange() {
        // 实测阻尼 0.1，目标 0.9 → 比例 9，应被钳制到 SCALE_MAX=3
        FeedbackIdentifier.ClosedLoopMetrics m = metricsWith(0.1, 3.0);

        FeedbackIdentifier.GainRecommendation rec = FeedbackIdentifier.recommendGains(
                2.0, 0.5, m, 3.0, 0.9, 0.5, 3.0);

        assertEquals(3.0, rec.velScale, 1e-9);
        assertEquals(1.5, rec.velGain, 1e-9);
    }

    @Test
    public void recommendGains_noTargets_keepsCurrent() {
        FeedbackIdentifier.ClosedLoopMetrics m = metricsWith(0.5, 3.0);
        // omegaTarget<=0 且 zetaTarget<=0 → 保持原值
        FeedbackIdentifier.GainRecommendation rec = FeedbackIdentifier.recommendGains(
                2.0, 0.5, m, 0, 0, 0.5, 3.0);
        assertEquals(2.0, rec.posGain, EPS);
        assertEquals(0.5, rec.velGain, EPS);
    }

    // ==================== 内环时间常数与模型交叉校验 ====================

    @Test
    public void velocityLoopTau_matchesKaOverKv() {
        // PARAMS 量级：kA=7e-5, kV=3.526e-4 → τ ≈ 0.1986 s
        double tau = FeedbackIdentifier.velocityLoopTau(3.5261972751626936e-4, 7.0e-5);
        assertEquals(0.1986, tau, 1e-3);
    }

    @Test
    public void velocityLoopTau_zeroKv_returnsZero() {
        assertEquals(0, FeedbackIdentifier.velocityLoopTau(0, 7e-5), EPS);
    }

    @Test
    public void modelPrediction_matchesMagnitudeOfCurrentGains() {
        // 当前 axialGain=2、axialVelGain=0.5、τ≈0.1986 → 模型预测 ωn≈3.17 rad/s, ζ≈1.19
        double tau = 0.1986;
        assertEquals(3.173, FeedbackIdentifier.modelOmegaN(2.0, tau), 0.01);
        assertEquals(1.190, FeedbackIdentifier.modelZeta(2.0, 0.5, tau), 0.01);
    }

    /** 直接构造一个闭环特征量对象（同包可访问包级构造器），用于隔离地验证增益建议逻辑。 */
    private static FeedbackIdentifier.ClosedLoopMetrics metricsWith(double zeta, double omegaN) {
        return new FeedbackIdentifier.ClosedLoopMetrics(zeta, omegaN, true, null);
    }
}
