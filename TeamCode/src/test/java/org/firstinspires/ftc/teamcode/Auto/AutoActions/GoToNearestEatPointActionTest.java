package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.junit.Test;

/** {@link GoToNearestEatPointAction} 单元测试。 */
public class GoToNearestEatPointActionTest {

    @Test
    public void construct_succeeds() {
        try (MockRobot m = new MockRobot(new Pose2d(81, -12, 0))) {
            new GoToNearestEatPointAction();
        }
    }

    @Test
    public void run_delegatesToTrajectory() {
        try (MockRobot m = new MockRobot(new Pose2d(81, -12, 0))) {
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToNearestEatPointAction action = new GoToNearestEatPointAction();
            assertTrue(action.run(new TelemetryPacket()));

            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            assertFalse(action.run(new TelemetryPacket()));
        }
    }
}
