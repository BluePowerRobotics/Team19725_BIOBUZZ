package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.AdaptiveParkAction;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.AutoTurretAimShootWithChassisAction;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.EatAction;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToOtherShootAreaViaHiveAction;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToStopPose;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToUnvisitedEatPointAvoidHiveAction;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.AutoParams;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.EatPointTracker;
import org.firstinspires.ftc.teamcode.utility.RobotStateStore;

/**
 * 红队自动阶段主程序（<b>底盘旋转辅助瞄准</b>版）。
 *
 * <p>与 {@link AutoRed} 的唯一区别：瞄准发射阶段使用
 * {@link AutoTurretAimShootWithChassisAction}——底盘主动转向球门，
 * 使炮台需要旋转的角度最小，缩短炮台到达目标位置的总时间。
 *
 * <p>其余逻辑（取球、吃球、移动、停靠、盟友禁区守卫）与 {@link AutoRed} 完全一致。
 */
@Autonomous(name = "AutoRedChassisAim", group = "Auto")
public class AutoRedChassisAim extends LinearOpMode {

    private enum AutoState {
        TO_EAT_POINT, EATING, TO_SHOOT_AREA, AIM_SHOOT, PARKING, STOP
    }

    private ActionRunner actionRunner;
    private MecanumDrive drive;
    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;

    private final TeamColor teamColor = TeamColor.RED;
    private final Pose2d startPose = HypParams.startPoseRed;
    private final Pose2d goal = HypParams.RedAudienceUp;

    private AutoState currentState = AutoState.TO_EAT_POINT;
    private boolean parkScheduled = false;
    private boolean zoneEscapeActive = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, startPose);
        drive = RobotPosition.getInstance().getDrive();
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team", teamColor);
        telemetry.update();

        waitForStart();

        RobotStateStore.clear();
        EatPointTracker.clear();
        ElapsedTime matchTime = new ElapsedTime();

        actionRunner.add(new GoToUnvisitedEatPointAvoidHiveAction(teamColor));

        while (opModeIsActive() && currentState != AutoState.STOP) {
            RobotPosition.getInstance().update();

            long matchMs = (long) matchTime.milliseconds();
            long remainingMs = HypParams.AUTONOMOUS_DURATION_MS - matchMs;

            if (!parkScheduled && remainingMs <= HypParams.PARK_TIME_THRESHOLD_MS) {
                actionRunner.clear();
                actionRunner.add(new AdaptiveParkAction(
                        drive, matchTime,
                        HypParams.AUTONOMOUS_DURATION_MS,
                        HypParams.PARK_TIME_THRESHOLD_MS));
                parkScheduled = true;
                currentState = AutoState.PARKING;
            }

            Pose2d pose = RobotPosition.getInstance().getPose2d();
            AutoParams.AllyZone conflict = parkScheduled || !AutoParams.useAllyZones
                    ? null
                    : AutoParams.firstConflictingZone(teamColor, pose, matchMs);
            if (conflict != null && !zoneEscapeActive) {
                zoneEscapeActive = true;
                actionRunner.clear();
                actionRunner.add(new GoToStopPose(
                        drive, AutoParams.nearestSafePose(teamColor, pose, matchMs)));
            } else if (conflict == null) {
                zoneEscapeActive = false;
            }

            if (!actionRunner.isBusy() && !zoneEscapeActive) {
                switch (currentState) {
                    case TO_EAT_POINT:
                        actionRunner.add(new EatAction(sweeper));
                        currentState = AutoState.EATING;
                        break;
                    case EATING:
                        actionRunner.add(new GoToOtherShootAreaViaHiveAction(teamColor, goal));
                        currentState = AutoState.TO_SHOOT_AREA;
                        break;
                    case TO_SHOOT_AREA:
                        actionRunner.add(new AutoTurretAimShootWithChassisAction(drive, turret));
                        currentState = AutoState.AIM_SHOOT;
                        break;
                    case AIM_SHOOT:
                        actionRunner.add(new GoToUnvisitedEatPointAvoidHiveAction(teamColor));
                        currentState = AutoState.TO_EAT_POINT;
                        break;
                    case PARKING:
                        currentState = AutoState.STOP;
                        break;
                    default:
                        break;
                }
            }

            actionRunner.update();

            telemetry.addData("State", currentState);
            telemetry.addData("Remaining(ms)", remainingMs);
            telemetry.addData("Active Zones", AutoParams.activeZoneNames(teamColor, matchMs));
            telemetry.addData("In Forbidden", conflict != null);
            telemetry.addData("Escaping", zoneEscapeActive);
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading(deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }

        RobotStateStore.savePose(RobotPosition.getInstance().getPose2d());
        sweeper.setStop();
        sweeper.update();
        turret.stop();
        chassis.stop();
    }
}
