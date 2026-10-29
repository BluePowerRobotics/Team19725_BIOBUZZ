package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;

import org.junit.Test;

import java.util.Arrays;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * 自动轨迹「时间最优」性质的回归测试。
 *
 * <p>Road Runner 1.0.1 在给定速度 / 加速度约束下自动生成<b>时间最优梯形速度曲线</b>
 * （以最大加速度加速到最大速度 → 保持最大速度巡航 → 以最大减速度停止），
 * 且同一 {@code actionBuilder} 内链式 {@code strafeToLinearHeading} 的途经点之间
 * <b>速度连续、不中途停车</b>。本测试守护这两条性质：任何 Action 只要把一段移动
 * 拆成多个 {@code actionBuilder}（多段独立轨迹），或调低约束，都会被这里或
 * {@code GoToShootAreaNearestVerticesActionTest} 暴露。
 */
public class TrajectoryProfileTest {

    /** 复现 {@code MecanumDrive.actionBuilder} 的默认约束，构建并返回捕获到的轨迹。 */
    private static TimeTrajectory build(Pose2d start, Pose2d[] waypoints) {
        MecanumKinematics kin = new MecanumKinematics(1.0, 1.0);
        MinVelConstraint vel = new MinVelConstraint(Arrays.asList(
                kin.new WheelVelConstraint(50.0),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accel = new ProfileAccelConstraint(-50.0, 50.0);
        TurnConstraints turn = new TurnConstraints(Math.PI, -Math.PI, Math.PI);

        final TimeTrajectory[] captured = new TimeTrajectory[1];
        TrajectoryActionBuilder builder = new TrajectoryActionBuilder(
                (TimeTurn t) -> null,
                (TimeTrajectory t) -> {
                    captured[0] = t;
                    return null;
                },
                new TrajectoryBuilderParams(1e-6, new ProfileParams(0.25, 0.1, 1e-2)),
                start, 0.0, turn, vel, accel);

        for (Pose2d wp : waypoints) {
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }
        builder.build();
        return captured[0];
    }

    /** 结论：链式共线途经点不会造成中途刹车（段间速度连续）。 */
    @Test
    public void chainedCollinearSegmentsDoNotStopAtWaypoints() {
        Pose2d start = new Pose2d(0, 0, 0);
        Pose2d mid = new Pose2d(30, 0, 0);
        Pose2d end = new Pose2d(60, 0, 0);

        TimeTrajectory chained = build(start, new Pose2d[]{mid, end});
        TimeTrajectory single = build(start, new Pose2d[]{end});

        System.out.println("single=" + single.duration + " chained=" + chained.duration);
        assertEquals("single=" + single.duration + " chained=" + chained.duration,
                single.duration, chained.duration, 1e-9);
    }

    /** 结论：长距离移动为梯形曲线——起步/末端静止、达到最大速度、总时长为解析时间最优值、中间存在高速巡航段。 */
    @Test
    public void longMoveHasTrapezoidalProfile() {
        Pose2d start = new Pose2d(-72, 0, 0);
        Pose2d end = new Pose2d(72, 0, 0);
        TimeTrajectory traj = build(start, new Pose2d[]{end});

        final double vmax = 50.0;   // in/s，与 build() 中 WheelVelConstraint 一致
        final double amax = 50.0;   // in/s^2，与 build() 中 ProfileAccelConstraint 一致
        final double d = Math.hypot(end.position.x - start.position.x,
                end.position.y - start.position.y);

        // 解析时间最优解（bang-bang 梯形）：加速到 vmax 所需行程各为 vmax²/(2·amax)
        double dAcc = vmax * vmax / (2 * amax);
        double tOptimal = (d >= 2 * dAcc)
                ? vmax / amax + (d - 2 * dAcc) / vmax + vmax / amax
                : 2 * Math.sqrt(d / amax);
        System.out.println("duration=" + traj.duration + " optimal=" + tOptimal);

        int n = 400;
        double dt = traj.duration / n;
        double[] speed = new double[n + 1];
        Pose2d prev = traj.get(0).value();
        double maxSpeed = 0;
        for (int i = 1; i <= n; i++) {
            Pose2d cur = traj.get(i * dt).value();
            speed[i] = Math.hypot(cur.position.x - prev.position.x,
                    cur.position.y - prev.position.y) / dt;
            maxSpeed = Math.max(maxSpeed, speed[i]);
            prev = cur;
        }

        // 1) 起步与末端速度归零（加速开始 / 减速结束）
        assertTrue("起步速度应≈0: " + speed[0], speed[0] < 0.05 * maxSpeed);
        assertTrue("末端速度应≈0: " + speed[n], speed[n] < 0.05 * maxSpeed);

        // 2) 达到最大速度
        assertEquals("应达到最大速度: " + maxSpeed, vmax, maxSpeed, 0.05 * vmax);

        // 3) 总时长等于解析时间最优值——「以最短时间完成整个路径」
        assertEquals("总时长应为解析时间最优值: " + traj.duration,
                tOptimal, traj.duration, 0.05 * tOptimal);

        // 4) 存在高速巡航段（梯形而非纯三角形）：接近最大速度的采样点应占相当比例
        int plateau = 0;
        for (double v : speed) {
            if (v >= 0.98 * maxSpeed) {
                plateau++;
            }
        }
        System.out.println("plateau=" + plateau + "/" + (n + 1));
        assertTrue("应存在高速巡航段（梯形曲线）: plateau=" + plateau + "/" + (n + 1),
                plateau > 0.3 * (n + 1));
    }
}
