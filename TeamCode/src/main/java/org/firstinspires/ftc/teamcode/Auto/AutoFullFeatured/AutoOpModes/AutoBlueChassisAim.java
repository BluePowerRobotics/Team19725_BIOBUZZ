package org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.AdaptiveParkAction;
import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.AutoTurretAimShootWithChassisAction;
import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.EatAction;
import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.GoToNearestShootAreaAvoidHiveAction;
import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.GoToStopPose;
import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.GoToUnvisitedEatPointAvoidHiveAction;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.AutoParams;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.AutoStartOptions;
import org.firstinspires.ftc.teamcode.utility.EatPointTracker;
import org.firstinspires.ftc.teamcode.utility.RobotStateStore;

/**
 * 蓝队自动阶段主程序 —— <b>第 3 组</b>（绕开 Hive · 底盘辅助转向瞄准）。
 *
 * <p>与第 1 组（{@link AutoBlue}）唯一区别：瞄准发射阶段使用
 * {@link AutoTurretAimShootWithChassisAction}——底盘主动转向球门，
 * 使炮台需要旋转的角度最小，缩短炮台到达目标位置的总时间。
 *
 * <p><b>动作序列</b>：
 * <pre>
 *   起点自动瞄准发射 × {@link #START_SHOTS}（底盘辅助转向）
 *     → 绕开 Hive 前往最近未访问取球点 → 吃球
 *     → 前往最近发射区域（途中三等分点面向 Hive 侧检查，偏离抬升侧则改道）→ 自动瞄准发射（底盘辅助转向）
 *     → 再循环一次 → 自适应停靠
 * </pre>
 *
 * <p>状态机：{@code START_SHOOT → TO_EAT_POINT → EATING → TO_SHOOT_AREA → AIM_SHOOT → CYCLE_END → … → PARKING → STOP}
 */
@Config
@Autonomous(name = "AutoBlueChassisAim", group = "Auto")
public class AutoBlueChassisAim extends LinearOpMode {

    /** 自动阶段状态机 */
    private enum AutoState {
        /** 起点连续瞄准发射（底盘辅助转向） */
        START_SHOOT,
        /** 前往未访问取球点（绕开 Hive） */
        TO_EAT_POINT,
        /** 吃球 */
        EATING,
        /** 前往最近发射区域（途中三等分点检查抬升侧，必要时改道另一个发射区域） */
        TO_SHOOT_AREA,
        /** 自动瞄准并发射（底盘辅助转向） */
        AIM_SHOOT,
        /** 一个循环结束，决定继续还是停靠 */
        CYCLE_END,
        /** 自适应停靠 */
        PARKING,
        /** 结束 */
        STOP
    }

    /** 起点连续发射的发数（等于预载球数），Dashboard 可实时调整 */
    public static int START_SHOTS = 4;
    /** 「取球 → 吃球 → 最近发射区 → 瞄准发射」的循环次数，Dashboard 可实时调整 */
    public static int CYCLES = 2;

    private ActionRunner actionRunner;
    private MecanumDrive drive;
    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;

    /** 队伍颜色（蓝方，Limelight pipeline 1） */
    private final TeamColor teamColor = TeamColor.BLUE;

    /** 蓝队起始位姿 */
    private final Pose2d startPose = HypParams.startPoseBlue;

    /**
     * 当前预瞄准球门：实时读取 {@link RobotPosition#getGoalPose()}，
     * 与炮台自动瞄准（{@code Turret.update}）使用同一来源，
     * 由 HIVE 抬升侧自动切换 AudienceUp / AudienceDown。
     */
    private Pose2d currentGoal() {
        return RobotPosition.getInstance().getGoalPose();
    }

    /** 当前状态，初始为「起点连续发射」 */
    private AutoState currentState = AutoState.START_SHOOT;

    /** 起点已完成的发射数 */
    private int shotsFired = 0;

    /** 已完成的「取球-发射」循环数 */
    private int cyclesDone = 0;

    /** 是否已投递自适应停靠动作（保证只投递一次） */
    private boolean parkScheduled = false;

    /** 是否正在执行「脱离盟友禁区」的动作（期间暂停常规动作投递） */
    private boolean zoneEscapeActive = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        actionRunner = new ActionRunner();

        // 初始化底盘与定位：Chassis 内部完成 RobotPosition 初始化（含 Limelight + 自适应 EKF）
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, startPose);
        drive = RobotPosition.getInstance().getDrive();
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team", teamColor);
        telemetry.addData("Route", "avoid Hive");
        telemetry.addData("Aim", "chassis assisted");
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

        // ===== 自动阶段开始：清空上一局遗留状态 =====
        RobotStateStore.clear();
        EatPointTracker.clear();

        // 比赛计时从 START 按下后开始（自适应停靠共用同一个计时器）
        ElapsedTime matchTime = new ElapsedTime();

        while (opModeIsActive() && currentState != AutoState.STOP) {
            // 更新位姿（每帧唯一一次推进定位器）
            RobotPosition.getInstance().update();

            long matchMs = (long) matchTime.milliseconds();
            long remainingMs = HypParams.AUTONOMOUS_DURATION_MS - matchMs;

            // ===== 规定时间前 3 秒：中断当前动作，转入自适应停靠 =====
            if (!parkScheduled && remainingMs <= HypParams.PARK_TIME_THRESHOLD_MS) {
                startParking(matchTime);
            }

            // ===== 盟友活动区（我方禁止活动区）守卫 =====
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

            // ===== 动作队列空闲且未在脱困时，投递下一段动作 =====
            if (!actionRunner.isBusy() && !zoneEscapeActive) {
                switch (currentState) {
                    case START_SHOOT:
                        if (startShoot && shotsFired < START_SHOTS) {
                            actionRunner.add(new AutoTurretAimShootWithChassisAction(drive, turret));
                            shotsFired++;
                        } else {
                            currentState = AutoState.TO_EAT_POINT;
                        }
                        break;

                    case TO_EAT_POINT:
                        actionRunner.add(new GoToUnvisitedEatPointAvoidHiveAction(teamColor, sweeper));
                        currentState = AutoState.EATING;
                        break;

                    case EATING:
                        actionRunner.add(new EatAction(sweeper));
                        currentState = AutoState.TO_SHOOT_AREA;
                        break;

                    case TO_SHOOT_AREA:
                        actionRunner.add(new GoToNearestShootAreaAvoidHiveAction(teamColor, currentGoal()));
                        currentState = AutoState.AIM_SHOOT;
                        break;

                    case AIM_SHOOT:
                        actionRunner.add(new AutoTurretAimShootWithChassisAction(drive, turret));
                        currentState = AutoState.CYCLE_END;
                        break;

                    case CYCLE_END:
                        cyclesDone++;
                        if (cyclesDone >= CYCLES) {
                            startParking(matchTime);
                        } else {
                            currentState = AutoState.TO_EAT_POINT;
                        }
                        break;

                    case PARKING:
                        currentState = AutoState.STOP;
                        break;

                    case STOP:
                    default:
                        break;
                }
            }

            actionRunner.update();

            telemetry.addData("State", currentState);
            telemetry.addData("Shots fired", shotsFired + "/" + START_SHOTS);
            telemetry.addData("Cycles done", cyclesDone + "/" + CYCLES);
            telemetry.addData("Remaining(ms)", remainingMs);
            telemetry.addData("Active Zones", AutoParams.activeZoneNames(teamColor, matchMs));
            telemetry.addData("In Forbidden", conflict != null);
            telemetry.addData("In Forbidden Half", pose.position.y < 0);
            telemetry.addData("Escaping", zoneEscapeActive);
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading(deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }

        // ===== 自动阶段结束：记录结束时刻的位姿，供手动阶段初始化时读回 =====
        RobotStateStore.savePose(RobotPosition.getInstance().getPose2d());

        // 确保底盘、集球器与炮台停稳
        sweeper.setStop();
        sweeper.update();
        turret.stop();
        chassis.stop();
        chassis.update(0, 0, 0);
    }

    /** 清空动作队列并投递自适应停靠（只执行一次）。 */
    private void startParking(ElapsedTime matchTime) {
        actionRunner.clear();
        actionRunner.add(new AdaptiveParkAction(
                drive, matchTime,
                HypParams.AUTONOMOUS_DURATION_MS,
                HypParams.PARK_TIME_THRESHOLD_MS));
        parkScheduled = true;
        currentState = AutoState.PARKING;
    }
}
