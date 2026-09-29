package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.junit.Test;

/** {@link EatAction} 单元测试。 */
public class EatActionTest {

    @Test
    public void run_withinDuration_eatsAndReturnsTrue() {
        Sweeper sweeper = mock(Sweeper.class);
        EatAction action = new EatAction(sweeper, 10_000); // 10s，远大于单帧

        assertTrue(action.run(new TelemetryPacket()));
        verify(sweeper).setEat();
        verify(sweeper).update();
        verify(sweeper, never()).setStop();
    }

    @Test
    public void run_afterDuration_stopsAndReturnsFalse() {
        Sweeper sweeper = mock(Sweeper.class);
        EatAction action = new EatAction(sweeper, 0); // 0ms，立即超时

        assertFalse(action.run(new TelemetryPacket()));
        verify(sweeper).setStop();
        verify(sweeper).update();
    }

    @Test
    public void run_thenTimeout_callsSetEatThenSetStop() {
        Sweeper sweeper = mock(Sweeper.class);
        EatAction action = new EatAction(sweeper, 50);

        // 先吃一帧
        assertTrue(action.run(new TelemetryPacket()));
        // 等待超过时长
        try {
            Thread.sleep(60);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        // 再调用一帧应结束
        assertFalse(action.run(new TelemetryPacket()));

        verify(sweeper, atLeastOnce()).setEat();
        verify(sweeper, times(1)).setStop();
    }
}
