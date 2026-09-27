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

/**
 * 抵达第三个吃球点位。
 *
 * <p>目标位姿按队伍颜色取自 {@link HypParams#EatThirdAreaRed} /
 * {@link HypParams#EatThirdAreaBlue}，构造时以当前融合位姿为起点，
 * 用 RoadRunner {@code strafeToLinearHeading} 平移到点并对准吃球朝向，
 * 到位后通常由 {@link EatAction} 执行吃球。
 */
@Config
public class GoToThirdEatPointAction implements Action {

    private final Action trajectoryAction;
    private final Pose2d targetPose;

    public GoToThirdEatPointAction(TeamColor teamColor) {
        MecanumDrive drive = RobotPosition.getInstance().getDrive();
        this.targetPose = (teamColor == TeamColor.RED)
                ? HypParams.EatThirdAreaRed
                : HypParams.EatThirdAreaBlue;
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        this.trajectoryAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(targetPose.position, targetPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("GoToThirdEatPoint", "Driving to third eat point");
        packet.put("ThirdEatPoint x", targetPose.position.x);
        packet.put("ThirdEatPoint y", targetPose.position.y);
        return trajectoryAction.run(packet);
    }
}
