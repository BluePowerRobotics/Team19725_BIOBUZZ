package org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.EatPointTracker;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HiveRoutePlanner;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.List;

/**
 * 【合成版】把「前往取球点 + 停车吃球 + 前往指定发射区域 +（可选）瞄准发射」
 * 合成 <b>一个 Action</b>，由 {@link SequentialAction} 串联：
 * <ol>
 *   <li>轨迹 1：当前位姿 → 取球点（{@link HiveRoutePlanner} 绕开 Hive 的外侧车道）；</li>
 *   <li>吃球：{@link EatAction}（<b>底盘原地不动</b>，默认 2.5 s），并标记该取球点已访问；</li>
 *   <li>轨迹 2：取球点 → 指定发射区域（{@code secondLegThroughHive} 控制是否经 Hive 正下方）；</li>
 *   <li>{@code includeAim} 为 true 时追加 {@link AutoAimAction}。</li>
 * </ol>
 *
 * <p><b>与保守版 {@link AutoCycleAction} 的差别</b>：仅组织方式不同（本类把整个周期合成
 * 一个 Action，便于一次入队），吃球同样是「到点停车吃」；两段轨迹之间因此必然停车。
 *
 * <p>本类在<b>首次 run()</b> 时以当时位姿规划，可直接放入队列。
 */
@Config
public class ContinuousRouteAction implements Action {

    /** 第二段是否经 Hive 正下方（true=通过，false=绕开） */
    public static boolean secondLegThroughHive = true;

    /** 是否在路线末尾追加自动瞄准 */
    public static boolean includeAim = true;

    /** 队伍颜色 */
    private final TeamColor teamColor;

    /** 集球器 */
    private final Sweeper sweeper;

    /** 炮台控制器（用于自动瞄准与发射） */
    private final Turret turret;

    /** 预瞄准的球门目标点 */
    private final Pose2d goal;

    /**
     * 回程发射区域目标：0=原逻辑（otherShootArea，按当前位姿自动选另一区域）；
     * 1=FIRST 第一发射区域；2=SECOND 第二发射区域。
     */
    private final int targetShootAreaIndex;

    /** 首次 run() 时构建的组合 Action */
    private Action impl;

    /** 取球点对应的底盘中心位姿（用于提前放铲），{@link #build()} 中写入 */
    private Pose2d eatTargetPose;

    /** 保证「放下铲子」只下发一次 */
    private boolean shovelDropped = false;

    /**
     * @param teamColor 队伍颜色
     * @param sweeper   集球器控制器
     * @param turret    炮台控制器
     * @param goal      预瞄准的单个球门目标点
     */
    public ContinuousRouteAction(TeamColor teamColor, Sweeper sweeper, Turret turret, Pose2d goal) {
        this(teamColor, sweeper, turret, goal, 0);
    }

    /**
     * 兼容旧 boolean 参数构造。
     *
     * @param goFirstShootArea true=本周期前往发射区域时改为第一发射区域（对应 targetShootAreaIndex=1）；
     *                         false=走原 otherShootArea 逻辑（对应 targetShootAreaIndex=0）
     */
    public ContinuousRouteAction(TeamColor teamColor, Sweeper sweeper, Turret turret, Pose2d goal,
                                 boolean goFirstShootArea) {
        this(teamColor, sweeper, turret, goal, goFirstShootArea ? 1 : 0);
    }

    /**
     * @param targetShootAreaIndex 0=otherShootArea（按当前位姿自动选另一区域）；
     *                              1={@link GoToShootAreaAction#FIRST}；2={@link GoToShootAreaAction#SECOND}
     */
    public ContinuousRouteAction(TeamColor teamColor, Sweeper sweeper, Turret turret, Pose2d goal,
                                 int targetShootAreaIndex) {
        this.teamColor = teamColor;
        this.sweeper = sweeper;
        this.turret = turret;
        this.goal = goal;
        this.targetShootAreaIndex = targetShootAreaIndex;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (impl == null) {
            impl = build();
        }

        // 到达取球点前 shovelLeadMs 放下铲子
        if (!shovelDropped && eatTargetPose != null && HiveRouteAction.shovelDue(eatTargetPose)) {
            sweeper.setShovelDown();
            sweeper.update();
            shovelDropped = true;
            packet.put("ContinuousRoute shovel", "Down");
        }

        packet.put("ContinuousRoute", "Running");
        return impl.run(packet);
    }

    /** 以首次执行时的位姿为起点，串联「轨迹 1 → 停车吃球 → 轨迹 2 →（可选）瞄准」。 */
    private Action build() {
        MecanumDrive drive = RobotPosition.getInstance().getDrive();
        Pose2d start = RobotPosition.getInstance().getPose2d();

        Pose2d eatTarget = HiveRouteAction.unvisitedEatPointTargetPose(teamColor);
        int eatIndex = HiveRouteAction.unvisitedEatPointIndex(teamColor);

        Pose2d shootTarget;
        switch (targetShootAreaIndex) {
            case GoToShootAreaAction.FIRST:
                shootTarget = shootAreaTargetPose(teamColor, GoToShootAreaAction.FIRST, goal);
                break;
            case GoToShootAreaAction.SECOND:
                shootTarget = shootAreaTargetPose(teamColor, GoToShootAreaAction.SECOND, goal);
                break;
            default: // 0 = 原逻辑 otherShootArea
                shootTarget = HiveRouteAction.otherShootAreaTargetPose(teamColor, goal);
        }

        eatTargetPose = eatTarget;

        Action leg1 = buildLeg(drive, start,
                HiveRoutePlanner.waypoints(teamColor, start, eatTarget, false));

        // 停车吃球：底盘原地不动，吃球结束后标记该取球点已访问
        Action eat = new SequentialAction(
                new EatAction(sweeper),
                new InstantAction(() -> {
                    // 吃球结束、前往发射区域前抬起铲子（已抬起则不改变当前铲子位置）
                    sweeper.raiseShovelIfNotUp();
                    EatPointTracker.markVisited(eatIndex);
                }));

        Action leg2 = buildLeg(drive, eatTarget,
                HiveRoutePlanner.waypoints(teamColor, eatTarget, shootTarget, secondLegThroughHive));

        Action route = new SequentialAction(leg1, eat, leg2);
        if (includeAim) {
            route = new SequentialAction(route, new AutoAimAction(drive, turret));
        }
        return route;
    }

    /** 把一组途经点串成一条以最大轮速为首尾速度的单段轨迹。 */
    private static Action buildLeg(MecanumDrive drive, Pose2d start, List<Pose2d> waypoints) {
        TrajectoryActionBuilder builder = drive.actionBuilder(start, MecanumDrive.PARAMS.maxWheelVel);
        for (Pose2d wp : waypoints) {
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }
        return builder.build();
    }

    /**
     * 指定发射区域内距机器人最近、朝向实时球门（{@code goal}）的底盘位姿。
     *
     * @param teamColor  队伍颜色
     * @param areaIndex  发射区域序号（{@link GoToShootAreaAction#FIRST} 或 {@link GoToShootAreaAction#SECOND}）
     * @param goal       预瞄准的单个球门目标点
     */
    private static Pose2d shootAreaTargetPose(TeamColor teamColor, int areaIndex, Pose2d goal) {
        ConvexPolygon area = HiveRouteAction.shootArea(teamColor, areaIndex);
        Pose2d current = RobotPosition.getInstance().getPose2d();
        Vector2D nearest = area.NearestVectorFrom(current.position.x, current.position.y);
        double x = current.position.x + nearest.getX();
        double y = current.position.y + nearest.getY();
        double heading = Math.atan2(goal.position.y - y, goal.position.x - x);
        return new Pose2d(x, y, heading);
    }
}
