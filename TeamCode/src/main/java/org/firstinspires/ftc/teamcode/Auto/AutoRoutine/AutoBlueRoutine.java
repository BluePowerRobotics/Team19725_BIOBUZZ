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
 * 蓝队自动阶段 —— <b>规定路线（AutoRoutine）</b>。
 *
 * <p><b>固定路线</b>（路线取自 {@code Auto/AutoFullFeatured} 中最基础的一组，但不再动态选点）：
 * <pre>
 *   开局原地瞄准发射 × {@link #START_SHOTS}（waitForStart 期间可用 gamepad1.X 关闭）
 *     → 绕开 Hive 前往取球点 1 → 吃球
 *     → 绕开 Hive 前往第二发射区域固定点 → 原地瞄准发射
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
@Autonomous(name = "AutoBlueRoutine", group = "AutoRoutine")
public class AutoBlueRoutine extends LinearOpMode {

    /** 规定路线状态机 */
    private enum RoutineState {
        /** 起点原地瞄准发射 */
        START_SHOOT,
        /** 前往取球点（复用：按 {@link #round} 选择第 1 / 2 个固定取球点） */
        TO_EAT,
        /** 吃球（复用） */
        EATING,
        /** 瞄准发射（复用） */
        AIM,
        /** 前往第二发射区域固定点 */
        TO_SHOOT_2,
        /** 前往固定停靠位姿 */
        PARKING,
        /** 结束 */
        STOP
    }

    /** 开局发射的发数（等于预载球数），Dashboard 可实时调整 */
    public static int START_SHOTS = 4;
    /** 第二次循环前往发射点时改为去第一发射区域（FtcDashboard 可实时改；init 时也可按 gamepad1.Y 切换） */
    public static boolean SECOND_CYCLE_FIRST_AREA = false;
    /** 循环数驱动发射区域切换模式（0=OFF, 1=EVEN, 2=ODD, 3=EVERY），init 时也可按 gamepad1.A 切换 */
    public static int AREA_CYCLE_MODE = 0;

    private ActionRunner actionRunner;
    private MecanumDrive drive;
    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;

    /** 队伍颜色（蓝方） */
    private final TeamColor teamColor = TeamColor.BLUE;
    /** 蓝队起始位姿 */
    private final Pose2d startPose = HypParams.startPoseBlue;
    /** 蓝队固定停靠位姿 */
    private final Pose2d stopPose = HypParams.StopPoseBlue;
    /** 固定预瞄准球门（规定路线不做动态切换） */
    private final Pose2d fixedGoal = HypParams.BlueAudienceUp;

    private RoutineState currentState = RoutineState.START_SHOOT;
    private int shotsFired = 0;
    private boolean parkScheduled = false;
    /** 当前轮次：1 → 取球点 1 / 第一发射区域，2 → 取球点 2 / 第二发射区域 */
    private int round = 1;

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
        telemetry.addData("Route", "fixed: eat1 -> shoot2 -> eat2 -> shoot2 -> park");
        telemetry.update();

        // ===== waitForStart 之前：gamepad1 的 X 键切换「开局是否先发射」（默认 true）=====
        AutoStartOptions.reset();
        while (opModeInInit()) {
            AutoStartOptions.update(gamepad1);
            telemetry.addData("Start Shoot (gamepad1.X)", AutoStartOptions.startShoot());
            telemetry.addData("2nd Cycle First Area (gamepad1.Y)",
                    SECOND_CYCLE_FIRST_AREA || AutoStartOptions.secondCycleFirstArea());
            telemetry.addData("Area Cycle Mode (gamepad1.A)", AutoStartOptions.areaCycleModeName(areaCycleMode()));
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
                            currentState = RoutineState.TO_EAT;
                        }
                        break;

                    case TO_EAT:
                        actionRunner.add(new RoutineMoveAction(teamColor,
                                fixedEatPose(round == 1 ? HypParams.EatPoint1 : HypParams.EatPoint2),
                                sweeper));
                        currentState = RoutineState.EATING;
                        break;

                    case EATING:
                        actionRunner.add(new EatAction(sweeper));
                        currentState = RoutineState.TO_SHOOT_2;
                        break;

                    case AIM:
                        actionRunner.add(new AutoAimAction(drive, turret));
                        if (round == 1) {
                            round = 2;
                            currentState = RoutineState.TO_EAT;
                        } else {
                            currentState = RoutineState.PARKING;
                        }
                        break;

                    case TO_SHOOT_2:
                        boolean toFirst = false;
                        int mode = areaCycleMode();
                        if (mode != AutoStartOptions.AREA_CYCLE_MODE_OFF) {
                            boolean isEven = round % 2 == 0;
                            switch (mode) {
                                case AutoStartOptions.AREA_CYCLE_MODE_EVEN:  toFirst = isEven; break;
                                case AutoStartOptions.AREA_CYCLE_MODE_ODD:   toFirst = !isEven; break;
                                case AutoStartOptions.AREA_CYCLE_MODE_EVERY: toFirst = isEven; break;
                            }
                        } else if (round >= 2 && secondCycleFirstArea()) {
                            toFirst = true;
                        }
                        Pose2d shootPose = toFirst
                                ? fixedShootPose(HypParams.ShootFirstAreaBlue)
                                : fixedShootPose(HypParams.ShootSecondAreaBlue);
                        actionRunner.add(new RoutineMoveAction(teamColor, shootPose, null));
                        currentState = RoutineState.AIM;
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

    /** 第二次及以后循环前往发射点时是否改去第一发射区域。 */
    private boolean secondCycleFirstArea() {
        return SECOND_CYCLE_FIRST_AREA || AutoStartOptions.secondCycleFirstArea();
    }

    /** 循环数驱动发射区域切换模式：优先用 AutoStartOptions（gamepad1.A），否则用本地 Dashboard 字段。 */
    private int areaCycleMode() {
        int opt = AutoStartOptions.getAreaCycleMode();
        return opt != AutoStartOptions.AREA_CYCLE_MODE_OFF ? opt : AREA_CYCLE_MODE;
    }

    /** 清空动作队列并前往固定停靠位姿（只执行一次）。 */
    private void startParking() {
        actionRunner.clear();
        actionRunner.add(new GoToStopPose(drive, stopPose));
        parkScheduled = true;
        currentState = RoutineState.PARKING;
    }
}
