package org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions;

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
 * <p><b>targetShootAreaIndex</b> 取值：
 * <ul>
 *   <li>{@code 0}（默认）：{@link HiveRouteAction#otherShootAreaTargetPose}，「另一发射区域」；</li>
 *   <li>{@link GoToShootAreaAction#FIRST}（1）：固定去<b>第一发射区域</b>；</li>
 *   <li>{@link GoToShootAreaAction#SECOND}（2）：固定去<b>第二发射区域</b>。</li>
 * </ul>
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
     * 目标发射区域序号：
     * 0 = otherShootArea（默认，与当前区域相反）；
     * 1 = FIRST（第一发射区域）；
     * 2 = SECOND（第二发射区域）。
     */
    private final int targetShootAreaIndex;

    /** 首次 run() 时构建的组合 Action */
    private Action impl;

    /** 取球点对应的底盘中心位姿（用于提前放铲），{@link #build()} 中写入 */
    private Pose2d eatTargetPose;

    /** 保证「放下铲子」只下发一次 */
    private boolean shovelDropped = false;

    /**
     * 默认构造：去「另一发射区域」。
     */
    public ContinuousRouteAction(TeamColor teamColor, Sweeper sweeper, Turret turret, Pose2d goal) {
        this(teamColor, sweeper, turret, goal, 0);
    }

    /**
     * 旧 boolean 兼容构造。
     *
     * @param goFirstShootArea true=第一发射区域（FIRST=1），false=另一发射区域（0，默认）
     */
    public ContinuousRouteAction(TeamColor teamColor, Sweeper sweeper, Turret turret, Pose2d goal,
                                 boolean goFirstShootArea) {
        this(teamColor, sweeper, turret, goal, goFirstShootArea ? GoToShootAreaAction.FIRST : 0);
    }

    /**
     * @param teamColor            队伍颜色
     * @param sweeper              集球器控制器
     * @param turret               炮台控制器
     * @param goal                 预瞄准的单个球门目标点
     * @param targetShootAreaIndex 目标发射区域序号：0=otherShootArea，1=FIRST，2=SECOND
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

        // 计算回程目标发射区域
        Pose2d shootTarget;
        switch (targetShootAreaIndex) {
            case GoToShootAreaAction.FIRST:
                shootTarget = shootAreaTargetPose(teamColor, GoToShootAreaAction.FIRST, goal);
                break;
            case GoToShootAreaAction.SECOND:
                shootTarget = shootAreaTargetPose(teamColor, GoToShootAreaAction.SECOND, goal);
                break;
            default:
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
            // 到达发射区后先检查抬升侧：不在抬升侧则自动改去另一发射区域再瞄准发射
            route = new SequentialAction(route,
                    new LiftedSideAimShootAction(teamColor, drive, turret, false, secondLegThroughHive));
        }
        return route;
    }

    /** 指定发射区域内距当前位置最近的点，航向指向调用方指定的单个球门。 */
    private static Pose2d shootAreaTargetPose(TeamColor teamColor, int areaIndex, Pose2d goal) {
        ConvexPolygon area = HiveRouteAction.shootArea(teamColor, areaIndex);
        Pose2d current = RobotPosition.getInstance().getPose2d();
        Vector2D nearest = area.NearestVectorFrom(current.position.x, current.position.y);
        double x = current.position.x + nearest.getX();
        double y = current.position.y + nearest.getY();
        double heading = Math.atan2(goal.position.y - y, goal.position.x - x);
        return new Pose2d(x, y, heading);
    }

    /** 把一组途经点串成一条以最大轮速为首尾速度的单段轨迹。 */
    private static Action buildLeg(MecanumDrive drive, Pose2d start, List<Pose2d> waypoints) {
        TrajectoryActionBuilder builder = drive.actionBuilder(start, MecanumDrive.PARAMS.maxWheelVel);
        for (Pose2d wp : waypoints) {
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }
        return builder.build();
    }
}
