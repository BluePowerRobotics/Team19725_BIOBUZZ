package org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions;

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
        // 起点须在两个发射区域之外，否则目标点与起点重合、路径长度为 0（无行驶段可执行）
        Pose2d start = new Pose2d(0, 0, 0);
        try (MockRobot m = new MockRobot(start)) {
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToShootAreaAction action = new GoToShootAreaAction(
                    TeamColor.RED, GoToShootAreaAction.FIRST, HypParams.RedAudienceUp);
            assertTrue("行驶中应返回 true", action.run(new TelemetryPacket()));

            // 行驶段由 ShootAreaApproachAction 按三等分拆成多段：
            // 每段结束时返回 true，最后一段结束时返回 false
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            boolean running = true;
            for (int i = 0; i < 10 && running; i++) {
                running = action.run(new TelemetryPacket());
            }
            assertFalse("全部行驶段结束后应返回 false", running);
        }
    }
}
