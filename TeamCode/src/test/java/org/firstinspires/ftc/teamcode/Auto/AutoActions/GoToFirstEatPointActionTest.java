package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.junit.Test;

/** {@link GoToFirstEatPointAction} 单元测试。 */
public class GoToFirstEatPointActionTest {

    @Test
    public void construct_redAndBlue_succeeds() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            new GoToFirstEatPointAction(TeamColor.RED);
            new GoToFirstEatPointAction(TeamColor.BLUE);
        }
    }

    @Test
    public void run_delegatesToTrajectory() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToFirstEatPointAction action = new GoToFirstEatPointAction(TeamColor.RED);
            assertTrue(action.run(new TelemetryPacket()));

            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            assertFalse(action.run(new TelemetryPacket()));
        }
    }
}
