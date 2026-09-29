package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.junit.Test;

/** {@link GoToShootAreaAction} 单元测试。 */
public class GoToShootAreaActionTest {

    @Test
    public void construct_allColorAndAreaCombinations_succeed() {
        Pose2d start = new Pose2d(60, -24, 0);
        try (MockRobot m = new MockRobot(start)) {
            new GoToShootAreaAction(TeamColor.RED, GoToShootAreaAction.FIRST,
                    HypParams.RedAudienceUp);
            new GoToShootAreaAction(TeamColor.RED, GoToShootAreaAction.SECOND,
                    HypParams.RedAudienceDown);
            new GoToShootAreaAction(TeamColor.BLUE, GoToShootAreaAction.FIRST,
                    HypParams.BlueAudienceUp);
            new GoToShootAreaAction(TeamColor.BLUE, GoToShootAreaAction.SECOND,
                    HypParams.BlueAudienceDown);
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void construct_invalidAreaIndex_throws() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            new GoToShootAreaAction(TeamColor.RED, 3, HypParams.RedAudienceUp);
        }
    }

    @Test
    public void run_delegatesToTrajectory() {
        Pose2d start = new Pose2d(60, -24, 0);
        try (MockRobot m = new MockRobot(start)) {
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToShootAreaAction action = new GoToShootAreaAction(
                    TeamColor.RED, GoToShootAreaAction.FIRST, HypParams.RedAudienceUp);
            assertTrue(action.run(new TelemetryPacket()));

            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            assertFalse(action.run(new TelemetryPacket()));
        }
    }
}
