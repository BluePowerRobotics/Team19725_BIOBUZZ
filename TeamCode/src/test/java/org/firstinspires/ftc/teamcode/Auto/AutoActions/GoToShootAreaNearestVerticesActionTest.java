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
    public void run_firstLegRunningThenBothDone_returnsTrueThenFalse() {
        Pose2d start = new Pose2d(81, -12, 0);
        try (MockRobot m = new MockRobot(start)) {
            // 第1帧第一段运行中(true)；第2帧第一段结束(false)且第二段也结束(false)
            when(m.trajectoryAction.run(any(TelemetryPacket.class)))
                    .thenReturn(true, false, false);
            GoToShootAreaNearestVerticesAction action =
                    new GoToShootAreaNearestVerticesAction(TeamColor.RED, HypParams.RedAudienceUp);

            assertTrue(action.run(new TelemetryPacket()));   // 第一段运行中
            assertFalse(action.run(new TelemetryPacket()));  // 第一段结束 + 第二段结束
        }
    }

    @Test
    public void run_firstLegDoneSecondLegRunning_thenDone() {
        Pose2d start = new Pose2d(81, -12, 0);
        try (MockRobot m = new MockRobot(start)) {
            // 第1帧第一段结束(false)、第二段运行中(true)；第2帧第二段运行中(true)；第3帧第二段结束(false)
            when(m.trajectoryAction.run(any(TelemetryPacket.class)))
                    .thenReturn(false, true, true, false);
            GoToShootAreaNearestVerticesAction action =
                    new GoToShootAreaNearestVerticesAction(TeamColor.BLUE, HypParams.BlueAudienceUp);

            assertTrue(action.run(new TelemetryPacket()));   // 第一段结束，第二段运行中
            assertTrue(action.run(new TelemetryPacket()));   // 第二段运行中
            assertFalse(action.run(new TelemetryPacket()));  // 第二段结束
        }
    }
}
