package org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ChassisGeometry;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 抵达指定取球点（合并原第一 / 第二吃球点 Action；自动阶段无第三取球点）。
 *
 * <p>取球点为底盘前端 intake 的目标点，按序号取自 {@link HypParams#EatPoint1} /
 * {@link HypParams#EatPoint2}（场地固定坐标，不区分红蓝）。
 *
 * <p>构造时以当前融合位姿为基准，调用
 * {@link ChassisGeometry#centerPoseFromIntakeTarget} 反推目标底盘中心位姿
 * —— 使前端 intake 恰好抵达该取球点，且机器人朝向面朝取球点，
 * 再用 RoadRunner {@code strafeToLinearHeading} 一次性平移并旋转到位。
 *
 * <p>轨迹在构造时刻固化，因此应在入队前、机器人位置基本确定后再创建本 Action。
 * 到位后通常由 {@link EatAction} 执行吃球。
 */
public class GoToEatPointAction implements Action {

    /** 取球点序号：第一个取球点 */
    public static final int FIRST = 1;
    /** 取球点序号：第二个取球点 */
    public static final int SECOND = 2;

    private final Action trajectoryAction;
    private final Pose2d targetPose;
    private final Vector2D targetEatPoint;
    private final int pointIndex;

    /** 集球器；为 {@code null} 时不提前放铲 */
    private final Sweeper sweeper;

    /** 保证「放下铲子」只下发一次 */
    private boolean shovelDropped = false;

    /**
     * @param pointIndex 取球点序号：{@link #FIRST} 或 {@link #SECOND}
     */
    public GoToEatPointAction(int pointIndex) {
        this(pointIndex, null);
    }

    /**
     * @param pointIndex 取球点序号：{@link #FIRST} 或 {@link #SECOND}
     * @param sweeper    集球器：到达取球点前 {@link HiveRouteAction#shovelLeadMs} 自动放下铲子
     */
    public GoToEatPointAction(int pointIndex, Sweeper sweeper) {
        if (pointIndex != FIRST && pointIndex != SECOND) {
            throw new IllegalArgumentException("pointIndex must be FIRST(1) or SECOND(2)");
        }
        this.pointIndex = pointIndex;
        this.sweeper = sweeper;

        MecanumDrive drive = RobotPosition.getInstance().getDrive();
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        double cx = currentPose.position.x;
        double cy = currentPose.position.y;

        this.targetEatPoint = (pointIndex == FIRST) ? HypParams.EatPoint1 : HypParams.EatPoint2;

        // 由 intake 目标点反推底盘中心目标位姿（含朝向）
        this.targetPose = ChassisGeometry.centerPoseFromIntakeTarget(
                targetEatPoint.getX(), targetEatPoint.getY(), cx, cy);

        this.trajectoryAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(targetPose.position, targetPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("GoToEatPoint", "Driving to eat point #" + pointIndex);
        packet.put("EatPoint index", pointIndex);
        packet.put("EatPoint x", targetEatPoint.getX());
        packet.put("EatPoint y", targetEatPoint.getY());
        packet.put("Target center x", targetPose.position.x);
        packet.put("Target center y", targetPose.position.y);
        packet.put("Target heading(deg)", Math.toDegrees(targetPose.heading.toDouble()));

        // 到达取球点前 shovelLeadMs 放下铲子
        if (sweeper != null && !shovelDropped && HiveRouteAction.shovelDue(targetPose)) {
            sweeper.setShovelDown();
            sweeper.update();
            shovelDropped = true;
            packet.put("GoToEatPoint shovel", "Down");
        }

        return trajectoryAction.run(packet);
    }
}
