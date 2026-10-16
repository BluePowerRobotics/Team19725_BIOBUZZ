package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.utility.HiveRoutePlanner;
import org.junit.Test;

import java.util.List;

/** {@link HiveRoutePlanner} 的几何约束单元测试。 */
public class HiveRoutePlannerTest {

    /** 取途经点列表除末项（终点）外的部分，即真正的中转点。 */
    private static List<Pose2d> transit(List<Pose2d> wps) {
        return wps.subList(0, wps.size() - 1);
    }

    @Test
    public void traverseY_isHalfwayBetweenLineAndInnerEdge() {
        assertEquals(-12.5, HiveRoutePlanner.traverseY(TeamColor.RED), 1e-6);
        assertEquals(12.5, HiveRoutePlanner.traverseY(TeamColor.BLUE), 1e-6);
    }

    @Test
    public void lanes_lieOutsideHiveFootprint() {
        // 纵向车道 x 的 |x| 必须超过矩形半宽（24），且两侧对称
        double redRight = HiveRoutePlanner.approachX(TeamColor.RED, 1);
        double redLeft = HiveRoutePlanner.approachX(TeamColor.RED, -1);
        assertTrue(Math.abs(redRight) > 24);
        assertEquals(-redRight, redLeft, 1e-9);
        assertEquals(redRight, HiveRoutePlanner.approachX(TeamColor.BLUE, 1), 1e-9);

        // 外侧车道 y 的 |y| 必须超过矩形外边界（25）
        assertTrue(Math.abs(HiveRoutePlanner.outerLaneY(TeamColor.RED)) > 25);
        assertEquals(-HiveRoutePlanner.outerLaneY(TeamColor.RED),
                HiveRoutePlanner.outerLaneY(TeamColor.BLUE), 1e-9);
    }

    @Test
    public void viaHive_red_hasWaypointFullyInsideRect_andNoLineContact() {
        Pose2d start = new Pose2d(48, -15, 0);
        Pose2d finals = new Pose2d(15, -72, 0);
        List<Pose2d> wps = HiveRoutePlanner.waypoints(TeamColor.RED, start, finals, true);

        boolean anyInside = false;
        for (Pose2d wp : transit(wps)) {
            assertFalse("中转点不得触及禁线: " + wp,
                    HiveRoutePlanner.isFootprintTouchingForbiddenLine(TeamColor.RED, wp));
            anyInside |= HiveRoutePlanner.isFootprintInsideHiveRect(TeamColor.RED, wp);
        }
        assertTrue("经 Hive 路线必须存在整机完全落入矩形的途经点", anyInside);
    }

    @Test
    public void viaHive_blue_isCentralSymmetric_ofRed() {
        Pose2d start = new Pose2d(-48, 15, 0);
        Pose2d finals = new Pose2d(-15, 72, 0);
        List<Pose2d> wps = HiveRoutePlanner.waypoints(TeamColor.BLUE, start, finals, true);

        boolean anyInside = false;
        for (Pose2d wp : transit(wps)) {
            assertFalse(HiveRoutePlanner.isFootprintTouchingForbiddenLine(TeamColor.BLUE, wp));
            anyInside |= HiveRoutePlanner.isFootprintInsideHiveRect(TeamColor.BLUE, wp);
        }
        assertTrue(anyInside);
    }

    @Test
    public void avoidHive_red_staysOutsideRect_andNoLineContact() {
        Pose2d start = new Pose2d(48, -15, 0);
        Pose2d finals = new Pose2d(-63, -33, 0);
        List<Pose2d> wps = HiveRoutePlanner.waypoints(TeamColor.RED, start, finals, false);

        for (Pose2d wp : transit(wps)) {
            assertFalse("绕行中转点必须位于矩形之外: " + wp,
                    HiveRoutePlanner.isFootprintInsideHiveRect(TeamColor.RED, wp));
            assertFalse(HiveRoutePlanner.isFootprintTouchingForbiddenLine(TeamColor.RED, wp));
        }
    }

    @Test
    public void avoidHive_blue_staysOutsideRect_andNoLineContact() {
        Pose2d start = new Pose2d(-48, 15, 0);
        Pose2d finals = new Pose2d(63, 33, 0);
        List<Pose2d> wps = HiveRoutePlanner.waypoints(TeamColor.BLUE, start, finals, false);

        for (Pose2d wp : transit(wps)) {
            assertFalse(HiveRoutePlanner.isFootprintInsideHiveRect(TeamColor.BLUE, wp));
            assertFalse(HiveRoutePlanner.isFootprintTouchingForbiddenLine(TeamColor.BLUE, wp));
        }
    }

    @Test
    public void waypoints_lastIsFinalPose() {
        Pose2d start = new Pose2d(48, -15, 0);
        Pose2d finals = new Pose2d(15, -72, 0.5);
        List<Pose2d> wps = HiveRoutePlanner.waypoints(TeamColor.RED, start, finals, true);
        Pose2d last = wps.get(wps.size() - 1);
        assertEquals(finals.position.x, last.position.x, 1e-9);
        assertEquals(finals.position.y, last.position.y, 1e-9);
        assertEquals(finals.heading.toDouble(), last.heading.toDouble(), 1e-9);
    }
}
