package org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HiveRoutePlanner;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.List;

/**
 * 抵达指定发射区域内距离机器人最近的点（合并原第一 / 第二发射区域 Action）。
 *
 * <p><b>保守备份特性</b>：所有移动均为<b>单段直达轨迹</b>，不经过三等分点 Hive 侧检查，
 * 不改道。如需经 Hive 正下方 / 绕外侧，用 4 参构造 {@code throughHive} 指定（通过
 * {@link HiveRoutePlanner} 规划途经点，但仍不做三等分点改道）。
 *
 * <p>目标区域按队伍颜色与区域序号取自 {@link HypParams#ShootFirstAreaRed} /
 * {@link HypParams#ShootFirstAreaBlue} / {@link HypParams#ShootSecondAreaRed} /
 * {@link HypParams#ShootSecondAreaBlue}（{@link ConvexPolygon} 凸多边形）。
 *
 * <p><b>瞄准目标与手动阶段一致</b>：{@code goal} 由调用方传入<b>单个具体球门</b>
 * （如 {@link HypParams#RedAudienceUp}），与
 * {@code Chassis#update(double, double, Pose2d)} 自动瞄准所用目标点完全相同；
 * 预瞄准航向 {@code atan2(goalY-y, goalX-x)} 也与 {@code Chassis#aimOmega} 同一公式。
 * 精确瞄准由后续 {@link AutoAimAction} 完成。
 *
 * <p>轨迹在构造时刻固化，因此应在入队前、机器人位于目标发射位置附近时再创建本 Action。
 */
public class GoToShootAreaAction implements Action {

    /** 区域序号：第一发射区域 */
    public static final int FIRST = 1;
    /** 区域序号：第二发射区域 */
    public static final int SECOND = 2;

    private final Action trajectoryAction;
    private final Pose2d targetPose;
    private final int areaIndex;

    /**
     * @param teamColor 队伍颜色
     * @param areaIndex 发射区域序号：{@link #FIRST} 或 {@link #SECOND}
     * @param goal      预瞄准的单个球门目标点（与手动瞄准传入 Chassis 的目标点一致）
     */
    public GoToShootAreaAction(TeamColor teamColor, int areaIndex, Pose2d goal) {
        this(teamColor, areaIndex, goal, false);
    }

    /**
     * @param teamColor  队伍颜色
     * @param areaIndex  发射区域序号：{@link #FIRST} 或 {@link #SECOND}
     * @param goal       预瞄准的单个球门目标点（与手动瞄准传入 Chassis 的目标点一致）
     * @param throughHive 是否经 Hive 正下方规划（{@code true} 走内车道，{@code false} 单段直达）；
     *                    保守版无三等分点检查，仅影响路径几何。
     */
    public GoToShootAreaAction(TeamColor teamColor, int areaIndex, Pose2d goal, boolean throughHive) {
        if (areaIndex != FIRST && areaIndex != SECOND) {
            throw new IllegalArgumentException("areaIndex must be FIRST(1) or SECOND(2)");
        }
        this.areaIndex = areaIndex;

        MecanumDrive drive = RobotPosition.getInstance().getDrive();
        boolean isRed = (teamColor == TeamColor.RED);
        ConvexPolygon shootArea;
        if (areaIndex == FIRST) {
            shootArea = isRed ? HypParams.ShootFirstAreaRed : HypParams.ShootFirstAreaBlue;
        } else {
            shootArea = isRed ? HypParams.ShootSecondAreaRed : HypParams.ShootSecondAreaBlue;
        }

        Pose2d currentPose = RobotPosition.getInstance().getPose2d();

        // ---- 计算多边形上离机器人当前位置最近的点作为目标位置 ----
        Vector2D currentPos = new Vector2D(currentPose.position.x, currentPose.position.y);
        Vector2D nearest = shootArea.NearestVectorFrom(currentPos);
        double targetX = currentPos.getX() + nearest.getX();
        double targetY = currentPos.getY() + nearest.getY();

        // ---- 预瞄准航向：指向具体球门（与 Chassis#aimOmega 同一公式）----
        double heading = Math.atan2(
                goal.position.y - targetY,
                goal.position.x - targetX);

        this.targetPose = new Pose2d(targetX, targetY, heading);

        if (throughHive) {
            // 经 Hive 正下方规划途经点（保守版：无三等分点检查，仅几何约束）
            List<Pose2d> waypoints = HiveRoutePlanner.waypoints(
                    teamColor, currentPose, targetPose, true);
            TrajectoryActionBuilder builder = drive.actionBuilder(currentPose);
            for (int i = 1; i < waypoints.size(); i++) {
                Pose2d wp = waypoints.get(i);
                builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
            }
            this.trajectoryAction = builder.build();
        } else {
            // 单段直达
            this.trajectoryAction = drive.actionBuilder(currentPose)
                    .strafeToLinearHeading(targetPose.position, targetPose.heading.toDouble())
                    .build();
        }
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("GoToShootArea", "Driving to shooting area #" + areaIndex);
        packet.put("ShootArea index", areaIndex);
        packet.put("ShootArea x", targetPose.position.x);
        packet.put("ShootArea y", targetPose.position.y);
        packet.put("ShootArea heading(deg)", Math.toDegrees(targetPose.heading.toDouble()));
        return trajectoryAction.run(packet);
    }
}
