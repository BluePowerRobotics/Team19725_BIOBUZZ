package org.firstinspires.ftc.teamcode.Auto.AutomaticCalibration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * {@link ConstraintIdentifier} 单元测试：验证约束折算量纲（in/s、in/s²、rad/s、rad/s²）、
 * 打滑判据、模型自由轮速推算与汇总建议。
 */
public class ConstraintIdentifierTest {

    private static final double IN_PER_TICK = 0.001999;
    private static final double EPS = 1e-9;

    // ==================== 峰值折算 ====================

    @Test
    public void maxWheelVelFromPeak_convertsTicksToInchesPerSecond() {
        // 25012 tick/s ≈ 50 in/s，乘安全系数 0.85 ≈ 42.5
        double v = ConstraintIdentifier.maxWheelVelFromPeak(25012, IN_PER_TICK, 0.85);
        assertEquals(42.5, v, 0.01);
    }

    @Test
    public void maxWheelVelFromPeak_badInputsReturnZero() {
        assertEquals(0, ConstraintIdentifier.maxWheelVelFromPeak(25012, 0, 0.85), EPS);
        assertEquals(0, ConstraintIdentifier.maxWheelVelFromPeak(25012, IN_PER_TICK, 0), EPS);
    }

    @Test
    public void maxProfileAccelFromPeak_convertsTicksToInchesPerSecondSquared() {
        double a = ConstraintIdentifier.maxProfileAccelFromPeak(100, IN_PER_TICK, 0.85);
        // 100 tick/s² × 0.001999 in/tick × 0.85 = 0.169915 in/s²
        assertEquals(0.169915, a, 1e-9);
    }

    @Test
    public void minProfileAccelFromMax_isSymmetricNegative() {
        assertEquals(-50.0, ConstraintIdentifier.minProfileAccelFromMax(50.0), EPS);
        assertEquals(-12.0, ConstraintIdentifier.minProfileAccelFromMax(-12.0), EPS);
    }

    // ==================== 打滑判据 ====================

    @Test
    public void slipRatio_relativeDeviation() {
        // 轮子比地面快 10 in/s（轮速 50 in/s）→ 相对偏差 0.2
        assertEquals(0.2, ConstraintIdentifier.slipRatio(50, 40), EPS);
        assertEquals(-0.2, ConstraintIdentifier.slipRatio(50, 60), EPS);
    }

    @Test
    public void slipRatio_zeroWheelSpeedIsGuarded() {
        // 分母保护：不产生除零 / 无穷
        assertTrue(Double.isFinite(ConstraintIdentifier.slipRatio(0, 0)));
        assertTrue(Double.isFinite(ConstraintIdentifier.slipRatio(0, 5)));
    }

    @Test
    public void isSlipping_respectsTolerance() {
        // 相对偏差 0.2 < 0.25 → 不打滑
        assertFalse(ConstraintIdentifier.isSlipping(50, 40, 0.25));
        // 相对偏差 0.3 > 0.25 → 打滑
        assertTrue(ConstraintIdentifier.isSlipping(50, 35, 0.25));
        // 反向同理
        assertTrue(ConstraintIdentifier.isSlipping(-50, -35, 0.25));
    }

    // ==================== 模型自由轮速推算 ====================

    @Test
    public void modelMaxWheelVelTicks_freeSpeedFromFeedforwardModel() {
        // (12 − 1.2) / 3.5262e-4 ≈ 30628 tick/s ≈ 61.2 in/s
        double ticks = ConstraintIdentifier.modelMaxWheelVelTicks(12.0, 1.2,
                3.5261972751626936e-4);
        assertEquals(30628, ticks, 2.0);
        assertEquals(61.2, ticks * IN_PER_TICK, 0.05);
    }

    @Test
    public void modelMaxWheelVelTicks_degenerateInputsReturnZero() {
        assertEquals(0, ConstraintIdentifier.modelMaxWheelVelTicks(12.0, 1.2, 0), EPS);
        assertEquals(0, ConstraintIdentifier.modelMaxWheelVelTicks(12.0, 1.2, -1e-4), EPS);
        // 电池电压低于静态补偿 → 无法转动
        assertEquals(0, ConstraintIdentifier.modelMaxWheelVelTicks(1.0, 1.2, 3.5e-4), EPS);
    }

    // ==================== 汇总建议 ====================

    @Test
    public void recommend_aggregatesAllConstraints() {
        ConstraintIdentifier.Constraints c = ConstraintIdentifier.recommend(
                25012, 100, 3.0, 10.0, IN_PER_TICK, 0.85, true);

        assertEquals(42.5, c.maxWheelVel, 0.01);
        assertEquals(0.169915, c.maxProfileAccel, 1e-9);
        assertEquals(-0.169915, c.minProfileAccel, 1e-9);
        assertEquals(2.55, c.maxAngVel, 1e-9);
        assertEquals(8.5, c.maxAngAccel, 1e-9);
        assertTrue(c.slipDetected);
    }

    @Test
    public void recommend_handlesNegativeAngularPeaksByAbsoluteValue() {
        ConstraintIdentifier.Constraints c = ConstraintIdentifier.recommend(
                0, 0, -2.0, -8.0, IN_PER_TICK, 0.5, false);
        assertEquals(1.0, c.maxAngVel, EPS);
        assertEquals(4.0, c.maxAngAccel, EPS);
        assertFalse(c.slipDetected);
    }
}
