package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
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
 * 抵达第一个发射区域内距离机器人最近的点。
 *
 * <p>目标区域按队伍颜色取自 {@link HypParams#ShootFirstAreaRed} /
 * {@link HypParams#ShootFirstAreaBlue}（{@link ConvexPolygon} 凸多边形）。
 * 构造时以当前融合位姿为起点，调用 {@link ConvexPolygon#NearestVectorFrom}
 * 求出多边形上离机器人最近的点作为目标位置，并以该点指向本联盟两球门中点的方向
 * 作为预瞄准航向，最后用 RoadRunner {@code strafeToLinearHeading} 一次性平移并旋转到位。
 * 精确瞄准由后续 {@link AutoAimAction} 完成。
 *
 * <p>轨迹在构造时刻固化（与 {@link GoToStopPose} 一致），因此应在入队前、机器人位于
 * 第一发射位置附近时再创建本 Action。
 */
@Config
public class GoToFirstShootAreaAction implements Action {

    private final Action trajectoryAction;
    private final Pose2d targetPose;

    public GoToFirstShootAreaAction(TeamColor teamColor) {
        MecanumDrive drive = RobotPosition.getInstance().getDrive();
        ConvexPolygon shootArea = (teamColor == TeamColor.RED)
                ? HypParams.ShootFirstAreaRed
                : HypParams.ShootFirstAreaBlue;

        Pose2d currentPose = RobotPosition.getInstance().getPose2d();

        // ---- 计算多边形上离机器人当前位置最近的点作为目标位置 ----
        Vector2D currentPos = new Vector2D(currentPose.position.x, currentPose.position.y);
        Vector2D nearest = shootArea.NearestVectorFrom(currentPos);
        double targetX = currentPos.getX() + nearest.getX();
        double targetY = currentPos.getY() + nearest.getY();

        // ---- 以本联盟两个球门的中点作为预瞄准参考点，计算目标航向 ----
        double goalY = (teamColor == TeamColor.RED)
                ? (HypParams.RedAudienceUp.position.y + HypParams.RedAudienceDown.position.y) / 2.0
                : (HypParams.BlueAudienceUp.position.y + HypParams.BlueAudienceDown.position.y) / 2.0;
        double heading = Math.atan2(goalY - targetY, 0.0 - targetX);

        this.targetPose = new Pose2d(targetX, targetY, heading);
        this.trajectoryAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(targetPose.position, targetPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("GoToFirstShootArea", "Driving to first shooting area");
        packet.put("FirstShootArea x", targetPose.position.x);
        packet.put("FirstShootArea y", targetPose.position.y);
        packet.put("FirstShootArea heading(deg)", Math.toDegrees(targetPose.heading.toDouble()));
        return trajectoryAction.run(packet);
    }
}
