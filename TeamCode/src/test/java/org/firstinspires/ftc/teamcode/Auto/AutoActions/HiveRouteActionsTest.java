package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.utility.EatPointTracker;
import org.junit.Before;
import org.junit.Test;

/** 4 个 Hive 路线 Action 的构造与执行单元测试。 */
public class HiveRouteActionsTest {

    @Before
    public void setUp() {
        EatPointTracker.clear();
    }

    @Test
    public void viaHiveEatPoint_red_marksVisitedOnComplete() {
        try (MockRobot m = new MockRobot(new Pose2d(24, -60, 0))) {
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToUnvisitedEatPointViaHiveAction action =
                    new GoToUnvisitedEatPointViaHiveAction(TeamColor.RED);
            // 起点靠近 EatPoint1，应选 FIRST
            assertEquals(EatPointTracker.FIRST, action.pointIndex());
            assertTrue(action.run(new TelemetryPacket()));

            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            assertFalse(action.run(new TelemetryPacket()));
            assertTrue(EatPointTracker.isVisited(EatPointTracker.FIRST));
        }
    }

    @Test
    public void avoidHiveEatPoint_red_marksVisitedOnComplete() {
        try (MockRobot m = new MockRobot(new Pose2d(24, -60, 0))) {
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToUnvisitedEatPointAvoidHiveAction action =
                    new GoToUnvisitedEatPointAvoidHiveAction(TeamColor.RED);
            assertFalse(action.throughHive());
            assertTrue(action.run(new TelemetryPacket()));

            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            assertFalse(action.run(new TelemetryPacket()));
            assertTrue(EatPointTracker.isVisited(action.pointIndex()));
        }
    }

    @Test
    public void viaHiveEatPoint_blue_targetsSymmetricPointOnBlueSide() {
        try (MockRobot m = new MockRobot(new Pose2d(48, 15, 0))) {
            GoToUnvisitedEatPointViaHiveAction action =
                    new GoToUnvisitedEatPointViaHiveAction(TeamColor.BLUE);
            // 蓝队取球点为中心对称点，均位于 +y 侧，故终点必在 +y
            assertTrue(action.finalPose().position.y > 0);
        }
    }

    @Test
    public void otherShootAreaViaHive_targetsTheOtherArea() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            GoToOtherShootAreaViaHiveAction action =
                    new GoToOtherShootAreaViaHiveAction(TeamColor.RED, HypParams.RedAudienceUp);
            // 位于原点时到两个发射区域距离相等，取 FIRST，故目标是 SECOND
            assertEquals(HiveRouteAction.SECOND, action.targetAreaIndex());
            assertTrue(action.throughHive());
        }
    }

    @Test
    public void otherShootAreaAvoidHive_targetsTheOtherArea() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            GoToOtherShootAreaAvoidHiveAction action =
                    new GoToOtherShootAreaAvoidHiveAction(TeamColor.BLUE, HypParams.BlueAudienceUp);
            assertEquals(HiveRouteAction.SECOND, action.targetAreaIndex());
            assertFalse(action.throughHive());
        }
    }

    @Test
    public void run_delegatesToTrajectory() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToOtherShootAreaViaHiveAction action =
                    new GoToOtherShootAreaViaHiveAction(TeamColor.RED, HypParams.RedAudienceUp);
            assertTrue(action.run(new TelemetryPacket()));
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            assertFalse(action.run(new TelemetryPacket()));
        }
    }
}
