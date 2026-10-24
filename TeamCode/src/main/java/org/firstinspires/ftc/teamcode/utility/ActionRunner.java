package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.util.LinkedList;
import java.util.Queue;

public class ActionRunner {
    // 待执行Action队列
    private final Queue<Action> pendingActions = new LinkedList<>();
    // 当前正在执行的Action
    private Action currentAction = null;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * 添加一个要执行的 Action（串行队列）。
     */
    public void add(Action action) {
        if (action != null && !pendingActions.contains(action)) {
            pendingActions.add(action);
        }
    }

    /**
     * 每帧更新当前Action，完成后自动取下一个。
     * 应当在 OpMode中循环调用。
     */
    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        // 如果没有当前Action，从队列取下一个
        if (currentAction == null && !pendingActions.isEmpty()) {
            currentAction = pendingActions.poll();
        }

        // 执行当前Action
        if (currentAction != null) {
            currentAction.preview(packet.fieldOverlay());
            boolean running = currentAction.run(packet);
            if (!running) {
                currentAction = null; // 执行完毕，准备下一个
            }
        }

        // 每帧绘制机器人的实际运动路径与当前位置（与当前 Action 无关）。
        // 必须在同一个 packet 上绘制：FTC Dashboard 场地图每帧只渲染最新收到的
        // packet，另发一个 packet 会把轨迹覆盖掉。
        drawMotionPath(packet.fieldOverlay());

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * 把底盘的实际运动路径（位姿历史，青色）与当前位姿（蓝色）画到场地图上。
     * 定位系统未初始化时静默跳过。
     */
    private void drawMotionPath(Canvas c) {
        RobotPosition robotPosition = RobotPosition.getInstanceOrNull();
        if (robotPosition == null) {
            return;
        }
        MecanumDrive drive = robotPosition.getDrive();
        if (drive == null) {
            return;
        }

        drive.drawPoseHistory(c);

        Pose2d pose = robotPosition.getPose2d();
        if (pose != null) {
            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);
        }
    }

    /**
     * 是否仍有动作在执行或队列中。
     */
    public boolean isBusy() {
        return currentAction != null || !pendingActions.isEmpty();
    }

    public Action getCurrentAction() {
        return currentAction;
    }

    /**
     * 停止所有动作（清空队列和当前Action）。
     */
    public void clear() {
        pendingActions.clear();
        currentAction = null;
    }
}
