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
 * 抵达第二个发射位置周围最近的发射区域。
 *
 * <p>目标位姿按队伍颜色取自 {@link HypParams#ShootSecondAreaRed} /
 * {@link HypParams#ShootSecondAreaBlue}，构造时以 AdaptiveEKFLocalizer 当前融合位姿为起点，
 * 用 RoadRunner {@code strafeToLinearHeading} 一次性平移并旋转到发射朝向。
 * 轨迹在构造时刻固化（与 {@link GoToStopPose} 一致），因此应在入队前、机器人位于
 * 第二发射位置附近时再创建本 Action。
 */
@Config
public class GoToSecondShootAreaAction implements Action {

    private final Action trajectoryAction;
    private final Pose2d targetPose;

    public GoToSecondShootAreaAction(TeamColor teamColor) {
        MecanumDrive drive = RobotPosition.getInstance().getDrive();
        this.targetPose = (teamColor == TeamColor.RED)
                ? HypParams.ShootSecondAreaRed
                : HypParams.ShootSecondAreaBlue;
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        this.trajectoryAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(targetPose.position, targetPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("GoToSecondShootArea", "Driving to second shooting area");
        packet.put("SecondShootArea x", targetPose.position.x);
        packet.put("SecondShootArea y", targetPose.position.y);
        return trajectoryAction.run(packet);
    }
}
