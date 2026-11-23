package org.firstinspires.ftc.teamcode.Auto.AutoRoutine;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.HiveRouteAction;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.HiveRoutePlanner;

import java.util.List;

/**
 * 规定路线中的一段固定行驶：当前位姿 → 目标位姿。
 *
 * <p>途经点直接取 {@link HiveRoutePlanner#waypoints(TeamColor, Pose2d, Pose2d, boolean)}
 * （{@code throughHive = false}，即沿外侧车道绕开 Hive），在构造时刻一次性固化，
 * <b>不包含</b>任何运行时改道、就近选点等自动规划逻辑。
 *
 * <p>传入 {@link Sweeper} 时，按「剩余距离 ÷ 线速度」估算 ETA，
 * 在 {@link HiveRouteAction#shovelLeadMs} 内提前放下铲子，保证到达取球点即可吸球。
 */
public class RoutineMoveAction implements Action {

    /** 实际执行的单条轨迹（途经点已在构造时固化） */
    private final Action trajectoryAction;
    /** 终点位姿（用于遥测与放铲 ETA 估算） */
    private final Pose2d targetPose;
    /** 集球器，可为 null（不提前放铲） */
    private final Sweeper sweeper;
    /** 是否已提前放下铲子 */
    private boolean shovelDropped;

    /**
     * @param teamColor  队伍颜色（决定绕开 Hive 的车道方向）
     * @param targetPose 终点位姿（含目标航向）
     * @param sweeper    集球器，可为 null
     */
    public RoutineMoveAction(TeamColor teamColor, Pose2d targetPose, Sweeper sweeper) {
        this.targetPose = targetPose;
        this.sweeper = sweeper;

        RobotPosition robotPosition = RobotPosition.getInstance();
        MecanumDrive drive = robotPosition.getDrive();
        Pose2d start = robotPosition.getPose2d();

        List<Pose2d> waypoints = HiveRoutePlanner.waypoints(teamColor, start, targetPose, false);
        TrajectoryActionBuilder builder = drive.actionBuilder(start, MecanumDrive.PARAMS.maxWheelVel);
        for (Pose2d wp : waypoints) {
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }
        this.trajectoryAction = builder.build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("Routine target x", targetPose.position.x);
        packet.put("Routine target y", targetPose.position.y);
        packet.put("Routine target heading(deg)", Math.toDegrees(targetPose.heading.toDouble()));

        if (sweeper != null && !shovelDropped && HiveRouteAction.shovelDue(targetPose)) {
            sweeper.setShovelDown();
            sweeper.update();
            shovelDropped = true;
            packet.put("Routine shovel", "Down");
        }
        return trajectoryAction.run(packet);
    }
}
