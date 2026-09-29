package org.firstinspires.ftc.teamcode.utility;

import static org.junit.Assert.assertEquals;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.junit.Before;
import org.junit.Test;

/**
 * {@link ChassisGeometry} 单元测试。
 *
 * <p>约定：底盘半长 L = 9，中心 O、朝向 θ。
 * 后端中点 R = O - L·(cosθ,sinθ)，前端 F = O + L·(cosθ,sinθ)。
 */
public class ChassisGeometryTest {

    private static final double L = 9.0;
    private static final double EPS = 1e-6;

    @Before
    public void setUp() {
        HypParams.chassisHalfLengthIn = L;
    }

    @Test
    public void centerFromRearMidpoint_headingZero() {
        // 后端在 (72,-12)，朝向 0 → 中心 = (72+L, -12)
        Vector2D c = ChassisGeometry.centerFromRearMidpoint(72, -12, 0);
        assertEquals(72 + L, c.getX(), EPS);
        assertEquals(-12, c.getY(), EPS);
    }

    @Test
    public void centerFromRearMidpoint_headingPiHalf() {
        // 朝向 π/2，前进方向为 (0,1) → 中心 = (rearX, rearY+L)
        Vector2D c = ChassisGeometry.centerFromRearMidpoint(0, 0, Math.PI / 2);
        assertEquals(0, c.getX(), EPS);
        assertEquals(L, c.getY(), EPS);
    }

    @Test
    public void rearMidpointFromCenter_inverseOfCenterFromRear() {
        double rx = 72, ry = -12, h = 0.7;
        Vector2D c = ChassisGeometry.centerFromRearMidpoint(rx, ry, h);
        Vector2D r = ChassisGeometry.rearMidpointFromCenter(c.getX(), c.getY(), h);
        assertEquals(rx, r.getX(), EPS);
        assertEquals(ry, r.getY(), EPS);
    }

    @Test
    public void frontMidpointFromCenter_headingZero() {
        // 中心 (0,0)，朝向 0 → 前端 = (L, 0)
        Vector2D f = ChassisGeometry.frontMidpointFromCenter(0, 0, 0);
        assertEquals(L, f.getX(), EPS);
        assertEquals(0, f.getY(), EPS);
    }

    @Test
    public void frontAndRearSymmetricAboutCenter() {
        double cx = 10, cy = 20, h = 1.3;
        Vector2D f = ChassisGeometry.frontMidpointFromCenter(cx, cy, h);
        Vector2D r = ChassisGeometry.rearMidpointFromCenter(cx, cy, h);
        // 中心应是前后端中点
        assertEquals(cx, (f.getX() + r.getX()) / 2, EPS);
        assertEquals(cy, (f.getY() + r.getY()) / 2, EPS);
    }

    @Test
    public void centerPoseFromIntakeTarget_intakeReachesTarget() {
        // 目标 intake 点 (24,-72)，当前中心 (81,-12)
        Pose2d p = ChassisGeometry.centerPoseFromIntakeTarget(24, -72, 81, -12);
        // 前端 = 中心 + L·(cosθ,sinθ) 应等于目标点
        double fx = p.position.x + L * Math.cos(p.heading.toDouble());
        double fy = p.position.y + L * Math.sin(p.heading.toDouble());
        assertEquals(24, fx, EPS);
        assertEquals(-72, fy, EPS);
    }

    @Test
    public void centerPoseFromIntakeTarget_headingFacesTarget() {
        Pose2d p = ChassisGeometry.centerPoseFromIntakeTarget(24, -72, 81, -12);
        double expectedH = Math.atan2(-72 - (-12), 24 - 81);
        assertEquals(expectedH, p.heading.toDouble(), EPS);
    }

    @Test
    public void centerPoseFromIntakeTarget_selfConsistentAtArrival() {
        // 到位后，从目标中心指向 intake 点的方向应等于 heading
        double ix = -72, iy = -24;
        Pose2d p = ChassisGeometry.centerPoseFromIntakeTarget(ix, iy, 0, 0);
        double dirToIntake = Math.atan2(iy - p.position.y, ix - p.position.x);
        assertEquals(p.heading.toDouble(), dirToIntake, EPS);
    }
}
