package org.firstinspires.ftc.teamcode.Auto.AutoRoutine;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.AutoAimAction;
import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.EatAction;
import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.GoToStopPose;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.AutoStartOptions;
import org.firstinspires.ftc.teamcode.utility.ChassisGeometry;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.RobotStateStore;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 红队自动阶段 —— <b>规定路线（AutoRoutine）</b>。
 *
 * <p><b>固定路线</b>（路线取自 {@code Auto/AutoFullFeatured} 中最基础的一组，但不再动态选点）：
 * <pre>
 *   开局原地瞄准发射 × {@link #START_SHOTS}（waitForStart 期间可用 gamepad1.X 关闭）
 *     → 绕开 Hive 前往取球点 1 → 吃球
 *     → 绕开 Hive 前往第一发射区域固定点 → 原地瞄准发射
 *     → 绕开 Hive 前往取球点 2 → 吃球
 *     → 绕开 Hive 前往第二发射区域固定点 → 原地瞄准发射
 *     → 前往固定停靠位姿
 * </pre>
 *
 * <p><b>不包含</b>：盟友活动范围禁区（{@code AutoParams}）、取球点访问记账
 * （{@code EatPointTracker}）、就近发射区选择、行驶途中三等分点改道等自动规划功能；
 * 取球点 / 发射区域 / 停靠位姿全部为固定值。
 */
@Config
@Autonomous(name = "AutoRedRoutine", group = "AutoRoutine")
public class AutoRedRoutine extends LinearOpMode {

    /** 规定路线状态机 */
    private enum RoutineState {
        /** 起点原地瞄准发射 */
        START_SHOOT,
        /** 前往取球点 1 */
        TO_EAT_1,
        /** 吃球 1 */
        EATING_1,
        /** 前往第一发射区域固定点 */
        TO_SHOOT_1,
        /** 瞄准发射 1 */
        AIM_1,
        /** 前往取球点 2 */
        TO_EAT_2,
        /** 吃球 2 */
        EATING_2,
        /** 前往第二发射区域固定点 */
        TO_SHOOT_2,
        /** 瞄准发射 2 */
        AIM_2,
        /** 前往固定停靠位姿 */
        PARKING,
        /** 结束 */
        STOP
    }

    /** 开局发射的发数（等于预载球数），Dashboard 可实时调整 */
    public static int START_SHOTS = 4;

    private ActionRunner actionRunner;
    private MecanumDrive drive;
    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;

    /** 队伍颜色（红方） */
    private final TeamColor teamColor = TeamColor.RED;
    /** 红队起始位姿 */
    private final Pose2d startPose = HypParams.startPoseRed;
    /** 红队固定停靠位姿 */
    private final Pose2d stopPose = HypParams.StopPoseRed;
    /** 固定预瞄准球门（规定路线不做动态切换） */
    private final Pose2d fixedGoal = HypParams.RedAudienceUp;

    private RoutineState currentState = RoutineState.START_SHOOT;
    private int shotsFired = 0;
    private boolean parkScheduled = false;

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
        telemetry.addData("Route", "fixed: eat1 -> shoot1 -> eat2 -> shoot2 -> park");
        telemetry.update();

        // ===== waitForStart 之前：gamepad1 的 X 键切换「开局是否先发射」（默认 true）=====
        AutoStartOptions.reset();
        while (opModeInInit()) {
            AutoStartOptions.update(gamepad1);
            telemetry.addData("Start Shoot (gamepad1.X)", AutoStartOptions.startShoot());
            telemetry.update();
        }
        waitForStart();
        final boolean startShoot = AutoStartOptions.startShoot();

        RobotStateStore.clear();
        ElapsedTime matchTime = new ElapsedTime();

        while (opModeIsActive() && currentState != RoutineState.STOP) {
            RobotPosition.getInstance().update();

            long matchMs = (long) matchTime.milliseconds();
            long remainingMs = HypParams.AUTONOMOUS_DURATION_MS - matchMs;

            // 剩余时间不足时中断规定路线，直接前往固定停靠位姿
            if (!parkScheduled && remainingMs <= HypParams.PARK_TIME_THRESHOLD_MS) {
                startParking();
            }

            if (!actionRunner.isBusy()) {
                switch (currentState) {
                    case START_SHOOT:
                        if (startShoot && shotsFired < START_SHOTS) {
                            actionRunner.add(new AutoAimAction(drive, turret));
                            shotsFired++;
                        } else {
                            currentState = RoutineState.TO_EAT_1;
                        }
                        break;

                    case TO_EAT_1:
                        actionRunner.add(new RoutineMoveAction(
                                teamColor, fixedEatPose(HypParams.EatPoint1), sweeper));
                        currentState = RoutineState.EATING_1;
                        break;

                    case EATING_1:
                        actionRunner.add(new EatAction(sweeper));
                        currentState = RoutineState.TO_SHOOT_1;
                        break;

                    case TO_SHOOT_1:
                        actionRunner.add(new RoutineMoveAction(
                                teamColor, fixedShootPose(HypParams.ShootFirstAreaRed), null));
                        currentState = RoutineState.AIM_1;
                        break;

                    case AIM_1:
                        actionRunner.add(new AutoAimAction(drive, turret));
                        currentState = RoutineState.TO_EAT_2;
                        break;

                    case TO_EAT_2:
                        actionRunner.add(new RoutineMoveAction(
                                teamColor, fixedEatPose(HypParams.EatPoint2), sweeper));
                        currentState = RoutineState.EATING_2;
                        break;

                    case EATING_2:
                        actionRunner.add(new EatAction(sweeper));
                        currentState = RoutineState.TO_SHOOT_2;
                        break;

                    case TO_SHOOT_2:
                        actionRunner.add(new RoutineMoveAction(
                                teamColor, fixedShootPose(HypParams.ShootSecondAreaRed), null));
                        currentState = RoutineState.AIM_2;
                        break;

                    case AIM_2:
                        actionRunner.add(new AutoAimAction(drive, turret));
                        currentState = RoutineState.PARKING;
                        break;

                    case PARKING:
                        if (!parkScheduled) {
                            startParking();
                        } else {
                            currentState = RoutineState.STOP;
                        }
                        break;

                    case STOP:
                    default:
                        break;
                }
            }

            actionRunner.update();

            Pose2d pose = RobotPosition.getInstance().getPose2d();
            telemetry.addData("State", currentState);
            telemetry.addData("Start Shoot", startShoot);
            telemetry.addData("Shots fired", shotsFired + "/" + START_SHOTS);
            telemetry.addData("Remaining(ms)", remainingMs);
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
        chassis.update(0, 0, 0);
    }

    /** 固定取球位姿：前端 intake 落在指定取球点，车头朝该点。 */
    private Pose2d fixedEatPose(Vector2D eatPoint) {
        Pose2d current = RobotPosition.getInstance().getPose2d();
        return ChassisGeometry.centerPoseFromIntakeTarget(
                eatPoint.getX(), eatPoint.getY(),
                current.position.x, current.position.y);
    }

    /** 固定发射位姿：取区域中距场地原点最近的顶点，车头预瞄固定球门。 */
    private Pose2d fixedShootPose(ConvexPolygon area) {
        Vector2D nearest = area.getVertex(0);
        double minDistSq = nearest.getX() * nearest.getX() + nearest.getY() * nearest.getY();
        for (int i = 1; i < area.getVertexCount(); i++) {
            Vector2D v = area.getVertex(i);
            double distSq = v.getX() * v.getX() + v.getY() * v.getY();
            if (distSq < minDistSq) {
                minDistSq = distSq;
                nearest = v;
            }
        }
        double heading = Math.atan2(
                fixedGoal.position.y - nearest.getY(),
                fixedGoal.position.x - nearest.getX());
        return new Pose2d(nearest.getX(), nearest.getY(), heading);
    }

    /** 清空动作队列并前往固定停靠位姿（只执行一次）。 */
    private void startParking() {
        actionRunner.clear();
        actionRunner.add(new GoToStopPose(drive, stopPose));
        parkScheduled = true;
        currentState = RoutineState.PARKING;
    }
}
