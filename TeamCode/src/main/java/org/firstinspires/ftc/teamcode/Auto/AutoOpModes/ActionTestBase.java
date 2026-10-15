package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

/**
 * Action 实机测试 OpMode 基类。
 *
 * <p>在 Driver Station 的 <b>ActionTest</b> 分组下为每个 Action 提供一个独立测试程序，
 * Init 阶段用手柄交互选择参数，START 后构造并执行被测 Action，完成后自动结束：
 *
 * <ul>
 *   <li>{@code A} / {@code B}：红队 / 蓝队（决定起始位姿、视觉 pipeline 与球门颜色）；</li>
 *   <li>十字键上 / 下：切换 {@code FIRST(1)} / {@code SECOND(2)}
 *       （仅 {@link #hasIndexOption()} 返回 true 的测试启用）；</li>
 *   <li>{@code X}：切换瞄准球门 AudienceUp / AudienceDown
 *       （仅 {@link #hasGoalOption()} 返回 true 的测试启用）。</li>
 * </ul>
 *
 * <p>子类只需实现 {@link #testName()} 与 {@link #createTestAction()}；
 * 后者在 START 后、底盘与定位初始化完成时调用，此时构造轨迹类 Action 可拿到正确起点。
 */
public abstract class ActionTestBase extends LinearOpMode {

    /** Init 阶段选择的队伍颜色 */
    protected TeamColor teamColor = TeamColor.RED;
    /** Init 阶段选择的序号：1=FIRST，2=SECOND */
    protected int optionIndex = 1;
    /** Init 阶段选择的球门：false=AudienceUp，true=AudienceDown */
    protected boolean goalDown = false;

    private Chassis chassis;
    private ActionRunner actionRunner;

    /** Driver Station 中显示的测试名称（遥测用） */
    protected abstract String testName();

    /**
     * 构造被测 Action。在 START 按下、底盘与融合定位初始化完成后调用。
     * 此时可安全使用 {@link RobotPosition#getInstance()} 与 {@link #drive()}。
     */
    protected abstract Action createTestAction();

    /** 是否显示并启用 FIRST / SECOND 序号切换（十字键上下） */
    protected boolean hasIndexOption() {
        return false;
    }

    /** 是否显示并启用 AudienceUp / AudienceDown 球门切换（X 键） */
    protected boolean hasGoalOption() {
        return false;
    }

    /** 当前选择队伍的起始位姿 */
    protected Pose2d selectedStartPose() {
        return (teamColor == TeamColor.RED) ? HypParams.startPoseRed : HypParams.startPoseBlue;
    }

    /** 当前选择队伍的停车位姿 */
    protected Pose2d selectedStopPose() {
        return (teamColor == TeamColor.RED) ? HypParams.StopPoseRed : HypParams.StopPoseBlue;
    }

    /** 当前选择队伍 + 上下选择对应的具体球门（与手动阶段瞄准目标同一来源） */
    protected Pose2d selectedGoal() {
        if (teamColor == TeamColor.RED) {
            return goalDown ? HypParams.RedAudienceDown : HypParams.RedAudienceUp;
        }
        return goalDown ? HypParams.BlueAudienceDown : HypParams.BlueAudienceUp;
    }

    /** 底盘（底盘与定位初始化之后可用） */
    protected MecanumDrive drive() {
        return RobotPosition.getInstance().getDrive();
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ================= Init：手柄选择参数 =================
        boolean prevA = false, prevB = false;
        boolean prevDpadUp = false, prevDpadDown = false, prevX = false;

        while (!isStarted() && !isStopRequested()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean x = gamepad1.x;

            if (a && !prevA) teamColor = TeamColor.RED;
            if (b && !prevB) teamColor = TeamColor.BLUE;
            if (hasIndexOption()) {
                if (dpadUp && !prevDpadUp) optionIndex = 1;
                if (dpadDown && !prevDpadDown) optionIndex = 2;
            }
            if (hasGoalOption() && x && !prevX) {
                goalDown = !goalDown;
            }

            prevA = a;
            prevB = b;
            prevDpadUp = dpadUp;
            prevDpadDown = dpadDown;
            prevX = x;

            telemetry.addLine("==== Action 实机测试：" + testName() + " ====");
            telemetry.addLine("A=红队  B=蓝队    当前：" + teamColor);
            if (hasIndexOption()) {
                telemetry.addLine("十字键上=FIRST  下=SECOND    当前："
                        + (optionIndex == 1 ? "FIRST" : "SECOND"));
            }
            if (hasGoalOption()) {
                telemetry.addLine("X 切换球门    当前：" + (goalDown ? "AudienceDown" : "AudienceUp"));
            }
            telemetry.addLine();
            telemetry.addLine("起始位姿：" + (teamColor == TeamColor.RED ? "红队" : "蓝队")
                    + " (" + fmt(selectedStartPose()) + ")");
            telemetry.addLine("将机器人摆放到起始位姿后按 START");
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        // ================= START：初始化底盘与融合定位 =================
        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, selectedStartPose());

        // 先更新一帧融合位姿，保证轨迹 Action 构造时起点最新
        RobotPosition.getInstance().update();

        Action testAction = createTestAction();
        actionRunner.add(testAction);

        // ================= 运行被测 Action =================
        while (opModeIsActive()) {
            RobotPosition.getInstance().update();
            actionRunner.update();

            Pose2d pose = RobotPosition.getInstance().getPose2d();
            telemetry.addData("Test", testName());
            telemetry.addData("Busy", actionRunner.isBusy());
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading(deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            if (!actionRunner.isBusy()) {
                // Action 完成：短暂保持以便观察终态，随后自动结束 OpMode
                sleep(300);
                break;
            }
        }

        chassis.stop();
    }

    /** 位姿格式化为简短字符串，用于 Init 遥测 */
    private static String fmt(Pose2d p) {
        return String.format("%.1f, %.1f, %.0f°",
                p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
    }
}
