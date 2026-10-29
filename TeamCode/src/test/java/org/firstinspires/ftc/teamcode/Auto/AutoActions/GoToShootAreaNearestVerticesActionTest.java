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

/** {@link GoToShootAreaNearestVerticesAction} 单元测试。 */
public class GoToShootAreaNearestVerticesActionTest {

    @Test
    public void construct_redAndBlue_succeed() {
        Pose2d start = new Pose2d(0, 0, 0);
        try (MockRobot m = new MockRobot(start)) {
            new GoToShootAreaNearestVerticesAction(TeamColor.RED, HypParams.RedAudienceUp);
            new GoToShootAreaNearestVerticesAction(TeamColor.BLUE, HypParams.BlueAudienceDown);
        }
    }

    @Test
    public void run_delegatesToSingleTrajectory() {
        Pose2d start = new Pose2d(81, -12, 0);
        try (MockRobot m = new MockRobot(start)) {
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToShootAreaNearestVerticesAction action =
                    new GoToShootAreaNearestVerticesAction(TeamColor.RED, HypParams.RedAudienceUp);

            // 单段连续轨迹：运行中返回 true
            assertTrue(action.run(new TelemetryPacket()));

            // 轨迹结束后返回 false（无第二段中途停顿）
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            assertFalse(action.run(new TelemetryPacket()));
        }
    }
}
