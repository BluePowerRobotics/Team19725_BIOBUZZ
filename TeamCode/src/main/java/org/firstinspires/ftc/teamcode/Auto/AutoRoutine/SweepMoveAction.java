package org.firstinspires.ftc.teamcode.Auto.AutoRoutine;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * 规定路线中的「边前往吸球终点、边吸球」：当前位姿 → 目标位姿，全程开启集球器。
 *
 * <p>与 {@link RoutineMoveAction} 的差别：
 * <ul>
 *   <li>轨迹为<b>一条直线段</b>（不经过 {@code HiveRoutePlanner} 绕行），
 *       由调用方给定的吸球起点 / 终点决定；</li>
 *   <li>从第一帧起放下铲子并持续 {@link Sweeper#setEat()}，到位后 {@link Sweeper#setStop()}。</li>
 * </ul>
 *
 * <p>轨迹在<b>首次 run()</b> 时以当时位姿构建，可直接放入动作队列。
 */
public class SweepMoveAction implements Action {

    /** 吸球终点位姿（含目标航向） */
    private final Pose2d targetPose;
    /** 集球器 */
    private final Sweeper sweeper;

    /** 首次 run() 时构建的轨迹 */
    private Action trajectoryAction;
    /** 是否已放下铲子并开启吸球 */
    private boolean started;

    /**
     * @param targetPose 吸球终点位姿
     * @param sweeper    集球器
     */
    public SweepMoveAction(Pose2d targetPose, Sweeper sweeper) {
        this.targetPose = targetPose;
        this.sweeper = sweeper;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (trajectoryAction == null) {
            RobotPosition robotPosition = RobotPosition.getInstance();
            Pose2d start = robotPosition.getPose2d();
            MecanumDrive drive = robotPosition.getDrive();
            TrajectoryActionBuilder builder =
                    drive.actionBuilder(start, MecanumDrive.PARAMS.maxWheelVel);
            trajectoryAction = builder
                    .strafeToLinearHeading(targetPose.position, targetPose.heading.toDouble())
                    .build();
        }

        if (!started) {
            sweeper.setShovelDown();
            started = true;
        }
        sweeper.setEat();
        sweeper.update();

        packet.put("Sweep target x", targetPose.position.x);
        packet.put("Sweep target y", targetPose.position.y);

        if (trajectoryAction.run(packet)) {
            return true;
        }
        // 到位：停止吸球
        sweeper.setStop();
        sweeper.update();
        return false;
    }
}
