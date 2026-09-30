package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.junit.Test;

/** {@link GoToEatPointAction} 单元测试。 */
public class GoToEatPointActionTest {

    @Test
    public void construct_firstAndSecond_succeed() {
        try (MockRobot m = new MockRobot(new Pose2d(81, -12, 0))) {
            new GoToEatPointAction(GoToEatPointAction.FIRST);
            new GoToEatPointAction(GoToEatPointAction.SECOND);
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void construct_invalidPointIndex_throws() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            new GoToEatPointAction(3);
        }
    }

    @Test
    public void run_delegatesToTrajectory() {
        try (MockRobot m = new MockRobot(new Pose2d(81, -12, 0))) {
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToEatPointAction action = new GoToEatPointAction(GoToEatPointAction.FIRST);
            assertTrue(action.run(new TelemetryPacket()));

            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            assertFalse(action.run(new TelemetryPacket()));
        }
    }
}
