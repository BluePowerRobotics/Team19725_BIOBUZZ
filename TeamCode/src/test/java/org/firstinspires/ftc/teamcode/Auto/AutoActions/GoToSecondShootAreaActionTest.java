package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.junit.Test;

/** {@link GoToSecondShootAreaAction} 单元测试。 */
public class GoToSecondShootAreaActionTest {

    @Test
    public void construct_redAndBlue_succeeds() {
        Pose2d start = new Pose2d(-60, -24, 0);
        try (MockRobot m = new MockRobot(start)) {
            new GoToSecondShootAreaAction(TeamColor.RED);
            new GoToSecondShootAreaAction(TeamColor.BLUE);
        }
    }

    @Test
    public void run_delegatesToTrajectory() {
        Pose2d start = new Pose2d(-60, -24, 0);
        try (MockRobot m = new MockRobot(start)) {
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToSecondShootAreaAction action = new GoToSecondShootAreaAction(TeamColor.RED);
            assertTrue(action.run(new TelemetryPacket()));

            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            assertFalse(action.run(new TelemetryPacket()));
        }
    }
}
