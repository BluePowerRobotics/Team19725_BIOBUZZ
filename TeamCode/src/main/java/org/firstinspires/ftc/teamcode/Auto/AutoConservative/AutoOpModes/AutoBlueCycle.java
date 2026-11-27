package org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions.AutoAimAction;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions.AdaptiveParkAction;
import org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions.AutoCycleAction;
import org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions.GoToShootAreaAction;
import org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions.GoToStopPose;
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
 * 蓝队自动阶段主程序（<b>保守版：取球点停车吃球</b>）。
 *
 * <p>与 {@link AutoBlue} 的区别：用 {@link AutoCycleAction} 把
 * 「前往取球点 → 吃球 → 前往另一发射区域 → 自动瞄准发射」合成为<b>一个 Action</b>，
 * 消除 ActionRunner 段间 1 帧调度空档，但各段仍是独立轨迹（取球点会停车吃球）。
 *
 * <p>其余逻辑（自适应停靠、盟友禁区守卫、状态收尾）与 {@link AutoBlue} 完全一致。
 */
@Config
@Autonomous(name = "ConservativeAutoBlueCycle", group = "Conservative")
public class AutoBlueCycle extends LinearOpMode {
    /** 开局发射的发数（等于预载球数），Dashboard 可实时调整 */
    public static int START_SHOTS = 4;
    /** 第二次及以后循环前往发射点时改为去第一发射区域（FtcDashboard 可实时改；init 时也可按 gamepad1.Y 切换） */
    public static boolean SECOND_CYCLE_FIRST_AREA = false;
    /** 循环数驱动发射区域切换模式（0=OFF, 1=EVEN, 2=ODD, 3=EVERY），init 时也可按 gamepad1.A 切换 */
    public static int AREA_CYCLE_MODE = 0;

    private enum AutoState {
        CYCLE, PARKING, STOP
    }

    private ActionRunner actionRunner;
    private MecanumDrive drive;
    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;

    private final TeamColor teamColor = TeamColor.BLUE;
    private final Pose2d startPose = HypParams.startPoseBlue;

    /**
     * 当前预瞄准球门：实时读取 {@link RobotPosition#getGoalPose()}，
     * 与炮台自动瞄准（{@code Turret.update}）使用同一来源，
     * 由 HIVE 抬升侧自动切换 AudienceUp / AudienceDown。
     */
    private Pose2d currentGoal() {
        return RobotPosition.getInstance().getGoalPose();
    }

    private AutoState currentState = AutoState.CYCLE;
    /** 已完成周期数（首个周期在循环外投递） */
    private int cycleCount = 1;
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
        telemetry.addData("Mode", "Cycle (stop at eat point)");
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
        EatPointTracker.clear();
        ElapsedTime matchTime = new ElapsedTime();

        // ===== 开局先发射 START_SHOTS 发（waitForStart 期间可用 gamepad1.X 关闭）=====
        if (startShoot) {
            List<Action> startShotActions = new ArrayList<>();
            for (int i = 0; i < START_SHOTS; i++) {
                startShotActions.add(new AutoAimAction(drive, turret));
            }
            actionRunner.add(new SequentialAction(startShotActions));
        }

        actionRunner.add(new AutoCycleAction(teamColor, sweeper, turret, currentGoal(),
                computeTargetAreaForCycle(cycleCount)));

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
                    case CYCLE:
                        actionRunner.add(new AutoCycleAction(
                                teamColor, sweeper, turret, currentGoal(),
                                computeTargetAreaForCycle(cycleCount)));
                        cycleCount++;
                        break;
                    case PARKING:
                        sweeper.raiseShovelIfNotUp();  // 两次循环结束后升起铲子（已升起则不改变）
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

    /** 第二次及以后循环前往发射点时是否改去第一发射区域。 */
    private boolean secondCycleFirstArea() {
        return SECOND_CYCLE_FIRST_AREA || AutoStartOptions.secondCycleFirstArea();
    }

    /** 循环数驱动发射区域切换模式：优先用 AutoStartOptions（gamepad1.A），否则用本地 Dashboard 字段。 */
    private int areaCycleMode() {
        int opt = AutoStartOptions.getAreaCycleMode();
        return opt != AutoStartOptions.AREA_CYCLE_MODE_OFF ? opt : AREA_CYCLE_MODE;
    }

    /** 根据循环序号（从 1 起）计算本次周期的目标发射区域序号（FIRST 或 SECOND）。 */
    private int computeTargetAreaForCycle(int cycleCount) {
        boolean toFirst = false;
        int mode = areaCycleMode();
        if (mode != AutoStartOptions.AREA_CYCLE_MODE_OFF) {
            boolean isEven = cycleCount % 2 == 0;
            switch (mode) {
                case AutoStartOptions.AREA_CYCLE_MODE_EVEN:  toFirst = isEven; break;
                case AutoStartOptions.AREA_CYCLE_MODE_ODD:   toFirst = !isEven; break;
                case AutoStartOptions.AREA_CYCLE_MODE_EVERY: toFirst = isEven; break;
            }
        } else if (cycleCount >= 2 && secondCycleFirstArea()) {
            toFirst = true;
        }
        return toFirst ? GoToShootAreaAction.FIRST : GoToShootAreaAction.SECOND;
    }
}
