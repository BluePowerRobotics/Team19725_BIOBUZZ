package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ChassisGeometry;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 前往距离机器人当前位置最近的取球点。
 *
 * <p>取球点（底盘前端 intake 的目标点）取自 {@link HypParams#EatPoint1} /
 * {@link HypParams#EatPoint2}。构造时以当前融合位姿为基准：
 * <ol>
 *   <li>计算底盘中心到两个取球点的直线距离，选择更近者；</li>
 *   <li>调用 {@link ChassisGeometry#centerPoseFromIntakeTarget} 求出目标底盘中心位姿
 *       —— 使前端 intake 恰好抵达该取球点，且机器人朝向面朝取球点；</li>
 *   <li>用 RoadRunner {@code strafeToLinearHeading} 一次性平移并旋转到位。</li>
 * </ol>
 *
 * <p>轨迹在构造时刻固化，因此应在入队前、机器人位置基本确定后再创建本 Action。
 * 到位后通常由 {@link EatAction} 执行吃球。
 */
@Config
public class GoToNearestEatPointAction implements Action {

    private final Action trajectoryAction;
    private final Pose2d targetPose;
    private final Vector2D targetEatPoint;

    public GoToNearestEatPointAction() {
        MecanumDrive drive = RobotPosition.getInstance().getDrive();
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        double cx = currentPose.position.x;
        double cy = currentPose.position.y;

        Vector2D p1 = HypParams.EatPoint1;
        Vector2D p2 = HypParams.EatPoint2;

        // 选择距离底盘中心更近的取球点
        double d1 = Math.hypot(p1.getX() - cx, p1.getY() - cy);
        double d2 = Math.hypot(p2.getX() - cx, p2.getY() - cy);
        this.targetEatPoint = (d1 <= d2) ? p1 : p2;

        // 由 intake 目标点反推底盘中心目标位姿（含朝向）
        this.targetPose = ChassisGeometry.centerPoseFromIntakeTarget(
                targetEatPoint.getX(), targetEatPoint.getY(), cx, cy);

        this.trajectoryAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(targetPose.position, targetPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("GoToNearestEatPoint", "Driving to nearest eat point");
        packet.put("EatPoint x", targetEatPoint.getX());
        packet.put("EatPoint y", targetEatPoint.getY());
        packet.put("Target center x", targetPose.position.x);
        packet.put("Target center y", targetPose.position.y);
        packet.put("Target heading(deg)", Math.toDegrees(targetPose.heading.toDouble()));
        return trajectoryAction.run(packet);
    }
}
