package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.AdaptiveParkAction;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.ContinuousRouteAction;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToStopPose;
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
 * 红队自动阶段主程序（<b>激进版：单条轨迹不停车</b>）。
 *
 * <p>与 {@link AutoRed} 的区别：用 {@link ContinuousRouteAction} 把
 * 「前往取球点 + 路过式吃球 + 前往另一发射区域 + 自动瞄准发射」合成<b>一个 Action</b>，
 * 全程只有一条轨迹，途经点之间不减速到零，最大化每周期速度。
 *
 * <p>其余逻辑（自适应停靠、盟友禁区守卫、状态收尾）与 {@link AutoRed} 完全一致。
 */
@Autonomous(name = "AutoRedContinuous", group = "Auto")
public class AutoRedContinuous extends LinearOpMode {

    private enum AutoState {
        /** 执行一个完整周期（取球 + 吃球 + 移动 + 瞄准发射） */
        CYCLE,
        /** 自适应停靠 */
        PARKING,
        /** 结束 */
        STOP
    }

    private ActionRunner actionRunner;
    private MecanumDrive drive;
    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;

    private final TeamColor teamColor = TeamColor.RED;
    private final Pose2d startPose = HypParams.startPoseRed;
    private final Pose2d goal = HypParams.RedAudienceUp;

    private AutoState currentState = AutoState.CYCLE;
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
        telemetry.addData("Mode", "Continuous (single trajectory)");
        telemetry.update();

        waitForStart();

        RobotStateStore.clear();
        EatPointTracker.clear();
        ElapsedTime matchTime = new ElapsedTime();

        // 首个周期：单条轨迹完成取球+吃球+移动+瞄准发射
        actionRunner.add(new ContinuousRouteAction(teamColor, sweeper, turret, goal));

        while (opModeIsActive() && currentState != AutoState.STOP) {
            RobotPosition.getInstance().update();

            long matchMs = (long) matchTime.milliseconds();
            long remainingMs = HypParams.AUTONOMOUS_DURATION_MS - matchMs;

            // T-3s 转入自适应停靠
            if (!parkScheduled && remainingMs <= HypParams.PARK_TIME_THRESHOLD_MS) {
                actionRunner.clear();
                actionRunner.add(new AdaptiveParkAction(
                        drive, matchTime,
                        HypParams.AUTONOMOUS_DURATION_MS,
                        HypParams.PARK_TIME_THRESHOLD_MS));
                parkScheduled = true;
                currentState = AutoState.PARKING;
            }

            // 盟友禁区守卫
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

            // 队列空闲且未脱困时，投递下一周期
            if (!actionRunner.isBusy() && !zoneEscapeActive) {
                switch (currentState) {
                    case CYCLE:
                        actionRunner.add(new ContinuousRouteAction(teamColor, sweeper, turret, goal));
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
