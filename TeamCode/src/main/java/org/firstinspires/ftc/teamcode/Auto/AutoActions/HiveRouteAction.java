package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ChassisGeometry;
import org.firstinspires.ftc.teamcode.utility.EatPointTracker;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HiveRoutePlanner;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.List;

/**
 * 「经 Hive 正下方 / 绕开 Hive」两类移动 Action 的抽象基类。
 *
 * <p>构造时以当前融合位姿为起点，用 {@link HiveRoutePlanner#waypoints} 生成中转途经点，
 * 再用一段 RoadRunner 轨迹（逐点 {@code strafeToLinearHeading}）依次经过各途经点与终点。
 *
 * <p><b>定位来源</b>：起点取自 {@link RobotPosition#getPose2d()}（{@code AdaptiveEKFLocalizer}
 * 融合位姿），轨迹跟随过程中由同一融合定位器推进。
 *
 * <p><b>轨迹在构造时刻固化</b>：应在入队前、机器人位置基本确定后再创建子类实例。
 *
 * <p>子类只需提供三个要素：队伍颜色、是否从 Hive 正下方通过、终点位姿；
 * 可选覆写 {@link #onComplete()} 在轨迹结束时更新共享状态（如
 * {@link EatPointTracker#markVisited(int)}）。
 */
public abstract class HiveRouteAction implements Action {

    /** 取球点 / 发射区域序号：第一个 */
    public static final int FIRST = 1;
    /** 取球点 / 发射区域序号：第二个 */
    public static final int SECOND = 2;

    private final TeamColor teamColor;
    private final boolean throughHive;
    private final Pose2d finalPose;
    private final Action trajectoryAction;
    private boolean completed = false;

    /**
     * @param teamColor  队伍颜色
     * @param throughHive true=从 Hive 正下方通过；false=绕开 Hive
     * @param finalPose  终点位姿（含目标航向）
     */
    protected HiveRouteAction(TeamColor teamColor, boolean throughHive, Pose2d finalPose) {
        this.teamColor = teamColor;
        this.throughHive = throughHive;
        this.finalPose = finalPose;

        MecanumDrive drive = RobotPosition.getInstance().getDrive();
        Pose2d start = RobotPosition.getInstance().getPose2d();
        List<Pose2d> waypoints = HiveRoutePlanner.waypoints(teamColor, start, finalPose, throughHive);

        TrajectoryActionBuilder builder = drive.actionBuilder(start);
        for (Pose2d wp : waypoints) {
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }
        this.trajectoryAction = builder.build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("HiveRoute", throughHive ? "via Hive" : "avoid Hive");
        packet.put("HiveRoute final x", finalPose.position.x);
        packet.put("HiveRoute final y", finalPose.position.y);
        packet.put("HiveRoute final heading(deg)", Math.toDegrees(finalPose.heading.toDouble()));
        boolean running = trajectoryAction.run(packet);
        if (!running && !completed) {
            completed = true;
            onComplete();
        }
        return running;
    }

    /** 轨迹正常结束（非中断）时的回调；子类可在此更新共享状态。 */
    protected void onComplete() {
    }

    /** @return 本 Action 的终点位姿 */
    public Pose2d finalPose() {
        return finalPose;
    }

    /** @return 本 Action 是否从 Hive 正下方通过 */
    public boolean throughHive() {
        return throughHive;
    }

    /** @return 本 Action 的队伍颜色 */
    public TeamColor teamColor() {
        return teamColor;
    }

    // ==================== 子类共用的目标位姿计算 ====================

    /**
     * 「此前未到达过的取球点」的底盘中心目标位姿。
     *
     * <p>用 {@link EatPointTracker#nearestUnvisitedIndex} 选出未访问取球点，
     * 再由 {@link ChassisGeometry#centerPoseFromIntakeTarget} 反推使前端 intake 恰好
     * 抵达该取球点、车头面朝取球点的底盘中心位姿。
     */
    protected static Pose2d unvisitedEatPointTargetPose(TeamColor teamColor) {
        Pose2d current = RobotPosition.getInstance().getPose2d();
        int index = EatPointTracker.nearestUnvisitedIndex(teamColor, current.position.x, current.position.y);
        Vector2D point = EatPointTracker.eatPoint(teamColor, index);
        return ChassisGeometry.centerPoseFromIntakeTarget(
                point.getX(), point.getY(), current.position.x, current.position.y);
    }

    /** 与 {@link #unvisitedEatPointTargetPose} 选中的取球点序号。 */
    protected static int unvisitedEatPointIndex(TeamColor teamColor) {
        Pose2d current = RobotPosition.getInstance().getPose2d();
        return EatPointTracker.nearestUnvisitedIndex(teamColor, current.position.x, current.position.y);
    }

    /** 按队伍颜色与序号取发射区域。 */
    protected static ConvexPolygon shootArea(TeamColor teamColor, int areaIndex) {
        boolean red = (teamColor == TeamColor.RED);
        if (areaIndex == FIRST) {
            return red ? HypParams.ShootFirstAreaRed : HypParams.ShootFirstAreaBlue;
        }
        return red ? HypParams.ShootSecondAreaRed : HypParams.ShootSecondAreaBlue;
    }

    /** 按当前位姿自动判断机器人所在的发射区域序号（返回 {@link #FIRST} 或 {@link #SECOND}）。 */
    protected static int currentShootAreaIndex(TeamColor teamColor) {
        ConvexPolygon first = shootArea(teamColor, FIRST);
        ConvexPolygon second = shootArea(teamColor, SECOND);
        Pose2d current = RobotPosition.getInstance().getPose2d();
        boolean inFirst = first.Contains(current.position.x, current.position.y);
        boolean inSecond = second.Contains(current.position.x, current.position.y);
        if (inFirst && !inSecond) {
            return FIRST;
        }
        if (inSecond && !inFirst) {
            return SECOND;
        }
        double d1 = first.NearestVectorFrom(current.position.x, current.position.y).getDistance();
        double d2 = second.NearestVectorFrom(current.position.x, current.position.y).getDistance();
        return (d1 <= d2) ? FIRST : SECOND;
    }

    /** 「另一个发射区域」的序号。 */
    protected static int otherShootAreaIndex(TeamColor teamColor) {
        return (currentShootAreaIndex(teamColor) == FIRST) ? SECOND : FIRST;
    }

    /**
     * 「另一个发射区域」内距离机器人最近的点，航向指向调用方指定的单个球门
     * （与 {@code Chassis#aimOmega} 同一公式，精确瞄准由后续 {@link AutoAimAction} 完成）。
     *
     * @param teamColor 队伍颜色
     * @param goal      预瞄准的单个球门目标点（与手动瞄准传入 Chassis 的目标点一致）
     */
    protected static Pose2d otherShootAreaTargetPose(TeamColor teamColor, Pose2d goal) {
        int otherIndex = otherShootAreaIndex(teamColor);
        ConvexPolygon area = shootArea(teamColor, otherIndex);
        Pose2d current = RobotPosition.getInstance().getPose2d();
        Vector2D nearest = area.NearestVectorFrom(current.position.x, current.position.y);
        double x = current.position.x + nearest.getX();
        double y = current.position.y + nearest.getY();
        double heading = Math.atan2(goal.position.y - y, goal.position.x - x);
        return new Pose2d(x, y, heading);
    }
}
