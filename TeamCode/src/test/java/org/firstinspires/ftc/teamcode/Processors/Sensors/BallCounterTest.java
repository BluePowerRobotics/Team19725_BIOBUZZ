package org.firstinspires.ftc.teamcode.Processors.Sensors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.junit.Test;

/**
 * {@link BallCounter} 单元测试：验证更新链路改为
 * {@code RobotPosition → BallCounter → BreakBeam} 后，计数结果仍然正确。
 *
 * <p><b>时序约定</b>：每次 {@link BallCounter#update(int)} 先用光电门<b>上一帧</b>的
 * 遮挡结果计数，随后才触发一次光电门采样。因此电压序列中的第 k 个样本，
 * 会在第 k+1 次 update 时参与计数（首帧使用初始状态 false）。
 *
 * <p>光电门约定：未遮挡输出高电压（归一化 1.0），遮挡输出低电压（≈0.03），
 * 滤波电压低于阈值 0.5 判定为遮挡。
 */
public class BallCounterTest {

    /** 未遮挡时的模拟电压（V），归一化后为 1.0 */
    private static final double OPEN_VOLTAGE = 3.3;
    /** 遮挡时的模拟电压（V），归一化后约 0.03 */
    private static final double BLOCKED_VOLTAGE = 0.1;

    /**
     * 用给定电压序列构造 BallCounter；序列耗尽后重复最后一个值。
     * 每次 counter.update() 末尾消费一个电压样本。
     */
    private static BallCounter counterWithVoltages(double... voltages) {
        AnalogInput input = mock(AnalogInput.class);
        final int[] index = {0};
        when(input.getVoltage()).thenAnswer(invocation -> {
            int i = Math.min(index[0], voltages.length - 1);
            index[0]++;
            return voltages[i];
        });
        return new BallCounter(new BreakBeamSensor(input, 0.8, 0.5));
    }

    /** 进球模式：每个球只计 1 次，球停在光电门中不重复计数。 */
    @Test
    public void countsOnePerBall() {
        BallCounter counter = counterWithVoltages(
                BLOCKED_VOLTAGE, OPEN_VOLTAGE, OPEN_VOLTAGE,
                BLOCKED_VOLTAGE, OPEN_VOLTAGE);

        assertFalse(counter.update(BallCounter.MODE_INTAKE));   // 无球（首帧无历史样本）
        assertTrue(counter.update(BallCounter.MODE_INTAKE));    // 第 1 球（上一帧已挡光）
        assertEquals(1, counter.getCount());
        assertFalse(counter.update(BallCounter.MODE_INTAKE));
        assertEquals(1, counter.getCount());
        assertFalse(counter.update(BallCounter.MODE_INTAKE));   // 球离开后不再计数
        assertTrue(counter.update(BallCounter.MODE_INTAKE));    // 第 2 球
        assertEquals(2, counter.getCount());
        assertFalse(counter.update(BallCounter.MODE_INTAKE));
    }

    /** 出球模式：每个上升沿 -1，且不会减到负数。 */
    @Test
    public void outputModeDecrementsAndClampsAtZero() {
        BallCounter counter = counterWithVoltages(
                BLOCKED_VOLTAGE, OPEN_VOLTAGE, OPEN_VOLTAGE,
                BLOCKED_VOLTAGE, OPEN_VOLTAGE,
                BLOCKED_VOLTAGE, OPEN_VOLTAGE, OPEN_VOLTAGE,
                BLOCKED_VOLTAGE, OPEN_VOLTAGE,
                BLOCKED_VOLTAGE, OPEN_VOLTAGE);

        // 先进入两球（第 2、5 帧各检测到一次上升沿）
        for (int i = 0; i < 5; i++) {
            counter.update(BallCounter.MODE_INTAKE);
        }
        assertEquals(2, counter.getCount());

        // 出球两次 → 0（第 7、10 帧各检测到一次上升沿）
        for (int i = 0; i < 5; i++) {
            counter.update(BallCounter.MODE_OUTPUT);
        }
        assertEquals(0, counter.getCount());

        // 球已打空，再挡光仍停在 0
        counter.update(BallCounter.MODE_OUTPUT);
        counter.update(BallCounter.MODE_OUTPUT);
        assertEquals(0, counter.getCount());
    }

    /** 清零后计数归零，且以最近一帧遮挡状态为基准，不产生虚假上升沿。 */
    @Test
    public void resetClearsWithoutFalseEdge() {
        BallCounter counter = counterWithVoltages(
                BLOCKED_VOLTAGE, OPEN_VOLTAGE, BLOCKED_VOLTAGE, OPEN_VOLTAGE);

        counter.update(BallCounter.MODE_INTAKE);   // 首帧：无历史样本
        counter.update(BallCounter.MODE_INTAKE);   // 检测到上升沿
        assertEquals(1, counter.getCount());

        // 清零（此时光电门最近一次采样为未遮挡）
        counter.reset();
        assertEquals(0, counter.getCount());

        // 下一球挡光才 +1
        assertFalse(counter.update(BallCounter.MODE_INTAKE));   // 基准为未遮挡
        assertTrue(counter.update(BallCounter.MODE_INTAKE));    // 上升沿
        assertEquals(1, counter.getCount());
    }

    /** 非法外部信息应保持原有校验行为。 */
    @Test(expected = IllegalArgumentException.class)
    public void rejectsInvalidMode() {
        counterWithVoltages(OPEN_VOLTAGE).update(2);
    }
}
