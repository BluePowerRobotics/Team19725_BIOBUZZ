package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 依次抵达两个发射区域中各自距离场地原点最近的顶点。
 *
 * <p>构造时以当前融合位姿为基准完成两步决策：
 * <ol>
 *   <li>用 {@link ConvexPolygon#NearestVectorFrom} 分别计算机器人到本颜色两个发射区域
 *       （{@link HypParams#ShootFirstAreaRed} / {@link HypParams#ShootFirstAreaBlue} 与
 *       {@link HypParams#ShootSecondAreaRed} / {@link HypParams#ShootSecondAreaBlue}）
 *       边界的最近距离，确定<b>距离机器人最近的发射区域</b>与<b>另一个发射区域</b>；</li>
 *   <li>分别取两个多边形顶点中<b>距离场地原点最近</b>的顶点作为两段目标位置，
 *       航向朝向调用方指定的<b>单个具体球门</b>作预瞄准（精确瞄准由
 *       {@link AutoAimAction} 完成）。</li>
 * </ol>
 *
 * <p><b>定位来源</b>：起点位姿取自 {@link RobotPosition#getPose2d()}
 * （即 {@code AdaptiveEKFLocalizer} 融合位姿），轨迹跟随过程中也由该融合定位器推进。
 *
 * <p><b>瞄准目标与手动阶段一致</b>：{@code goal} 与
 * {@code Chassis#update(double, double, Pose2d)} 自动瞄准传入的目标点为同一个球门，
 * 航向公式 {@code atan2(goalY-y, goalX-x)} 与 {@code Chassis#aimOmega} 完全相同。
 *
 * <p>随后用 RoadRunner {@code strafeToLinearHeading} 构建两段轨迹并依次执行：
 * 先驶向最近区域的最近原点顶点并停稳，再驶向另一区域的最近原点顶点。
 * 两段轨迹均在构造时刻固化（第二段以第一段名义终点为起点），因此应在入队前、
 * 机器人位置基本确定后再创建本 Action。
 */
public class GoToShootAreaNearestVerticesAction implements Action {

    /** 第一段轨迹：驶向距离机器人最近的发射区域中距原点最近的顶点 */
    private final Action firstLeg;
    /** 第二段轨迹：驶向另一个发射区域中距原点最近的顶点 */
    private final Action secondLeg;
    /** 第一段目标位姿 */
    private final Pose2d firstVertexPose;
    /** 第二段目标位姿 */
    private final Pose2d secondVertexPose;

    /** 第一段轨迹是否已完成 */
    private boolean firstLegDone = false;

    /**
     * @param teamColor 队伍颜色
     * @param goal      预瞄准的单个球门目标点（与手动瞄准传入 Chassis 的目标点一致）
     */
    public GoToShootAreaNearestVerticesAction(TeamColor teamColor, Pose2d goal) {
        RobotPosition robotPosition = RobotPosition.getInstance();
        MecanumDrive drive = robotPosition.getDrive();

        ConvexPolygon firstArea = (teamColor == TeamColor.RED)
                ? HypParams.ShootFirstAreaRed : HypParams.ShootFirstAreaBlue;
        ConvexPolygon secondArea = (teamColor == TeamColor.RED)
                ? HypParams.ShootSecondAreaRed : HypParams.ShootSecondAreaBlue;

        Pose2d currentPose = robotPosition.getPose2d();

        // ---- 按边界最近距离确定距离机器人最近的发射区域与另一个发射区域 ----
        boolean firstNearest =
                distanceTo(firstArea, currentPose) <= distanceTo(secondArea, currentPose);
        ConvexPolygon nearestArea = firstNearest ? firstArea : secondArea;
        ConvexPolygon otherArea = firstNearest ? secondArea : firstArea;

        // ---- 两段目标：各自区域中距原点最近的顶点，朝向具体球门 ----
        this.firstVertexPose = nearestVertexToOriginPose(nearestArea, goal);
        this.secondVertexPose = nearestVertexToOriginPose(otherArea, goal);

        this.firstLeg = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(firstVertexPose.position, firstVertexPose.heading.toDouble())
                .build();
        this.secondLeg = drive.actionBuilder(firstVertexPose)
                .strafeToLinearHeading(secondVertexPose.position, secondVertexPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!firstLegDone) {
            packet.put("ShootVertices", "Leg1: nearest area origin-nearest vertex");
            packet.put("Leg1 target x", firstVertexPose.position.x);
            packet.put("Leg1 target y", firstVertexPose.position.y);
            packet.put("Leg1 heading(deg)", Math.toDegrees(firstVertexPose.heading.toDouble()));
            if (firstLeg.run(packet)) {
                return true;
            }
            firstLegDone = true;
        }

        packet.put("ShootVertices", "Leg2: other area origin-nearest vertex");
        packet.put("Leg2 target x", secondVertexPose.position.x);
        packet.put("Leg2 target y", secondVertexPose.position.y);
        packet.put("Leg2 heading(deg)", Math.toDegrees(secondVertexPose.heading.toDouble()));
        return secondLeg.run(packet);
    }

    /** 机器人位姿到多边形边界的最近距离（位于多边形内时为 0） */
    private static double distanceTo(ConvexPolygon polygon, Pose2d pose) {
        return polygon.NearestVectorFrom(pose.position.x, pose.position.y).getDistance();
    }

    /**
     * 取多边形中距离场地原点最近的顶点，并以该顶点指向具体球门的方向
     * （与 {@code Chassis#aimOmega} 同一公式）作为预瞄准航向，生成目标位姿。
     */
    private static Pose2d nearestVertexToOriginPose(ConvexPolygon polygon, Pose2d goal) {
        Vector2D nearestVertex = polygon.getVertex(0);
        double minDistSq = distSqToOrigin(nearestVertex);
        for (int i = 1; i < polygon.getVertexCount(); i++) {
            Vector2D v = polygon.getVertex(i);
            double distSq = distSqToOrigin(v);
            if (distSq < minDistSq) {
                minDistSq = distSq;
                nearestVertex = v;
            }
        }
        double heading = Math.atan2(
                goal.position.y - nearestVertex.getY(),
                goal.position.x - nearestVertex.getX());
        return new Pose2d(nearestVertex.getX(), nearestVertex.getY(), heading);
    }

    /** 顶点到场地原点的距离平方（避免开方） */
    private static double distSqToOrigin(Vector2D v) {
        return v.getX() * v.getX() + v.getY() * v.getY();
    }
}
