package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ChassisGeometry;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 自适应停靠 Action：在自动阶段结束前 {@link HypParams#PARK_TIME_THRESHOLD_MS}（默认 3 s）
 * 检测<b>整机外接矩形</b>是否与场地边框接触，并据此决定停靠点。
 *
 * <p><b>判定与动作</b>
 * <ul>
 *   <li><b>接触</b>：把底盘中心以<b>最小平移量</b>推回到「整机外接矩形完全位于场地内、
 *       且距边框不小于 {@link HypParams#parkBorderMarginIn}」的位置，沿 RoadRunner 轨迹开过去后停车。
 *       该位置由<b>当时的位姿</b>决定，因此本局的停靠点是<b>不定点</b>；</li>
 *   <li><b>不接触</b>：不移动，直接停车并结束。</li>
 * </ul>
 *
 * <p><b>场地边界</b>：12 ft × 12 ft 场地以原点为中心，边界为
 * {@code |x| = |y| = }{@link HypParams#fieldHalfSizeIn}（72 in）。
 *
 * <p><b>等待语义</b>：未到「结束前 3 秒」时本 Action 保持电机停止并返回 {@code true}，
 * 因此既可以作为动作队列的最后一个元素提前入队，也可以在 T-3s 时由 OpMode 直接投递
 * （此时会立即进入检测）。
 *
 * <p><b>位姿来源</b>：当前位姿取自 {@link RobotPosition#getPose2d()}（融合定位），
 * 本 Action 不推进定位器，由 OpMode 每帧唯一一次 {@code update()} 负责。
 */
public class AdaptiveParkAction implements Action {

    private static final String TAG = "AdaptivePark";
    /** 浮点比较容差（英寸） */
    private static final double BORDER_EPS = 1e-6;

    private final MecanumDrive drive;
    private final ElapsedTime matchTime;
    private final long durationMs;
    private final long leadTimeMs;

    /** 是否已完成 T-3s 检测 */
    private boolean evaluated = false;
    /** 是否已结束（不接触 → 立即结束；接触 → 到位后结束） */
    private boolean done = false;
    /** 接触时使用的停靠轨迹；不接触时为 null */
    private Action trajectoryAction;
    /** 本局停靠点（不接触时为检测时刻的当前位姿） */
    private Pose2d parkPose;
    /** 检测时刻整机是否与场地边框接触 */
    private boolean borderContact = false;

    /**
     * @param drive       底盘
     * @param matchTime   与 OpMode 共用的比赛计时器（START 时创建）
     * @param durationMs  自动阶段总时长（ms），通常为 {@link HypParams#AUTONOMOUS_DURATION_MS}
     * @param leadTimeMs  提前检测量（ms），通常为 {@link HypParams#PARK_TIME_THRESHOLD_MS}
     */
    public AdaptiveParkAction(MecanumDrive drive, ElapsedTime matchTime,
                             long durationMs, long leadTimeMs) {
        this.drive = drive;
        this.matchTime = matchTime;
        this.durationMs = durationMs;
        this.leadTimeMs = leadTimeMs;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (done) {
            return false;
        }

        long remainingMs = durationMs - (long) matchTime.milliseconds();

        // ---- 未到 T-3s：保持不动，等待 ----
        if (!evaluated && remainingMs > leadTimeMs) {
            stopDrive();
            packet.put("AdaptivePark", "waiting until T-" + (leadTimeMs / 1000) + "s");
            packet.put("AdaptivePark remaining(ms)", remainingMs);
            return true;
        }

        // ---- 到达 T-3s：检测一次，并决定停靠点 ----
        if (!evaluated) {
            evaluated = true;
            Pose2d current = RobotPosition.getInstance().getPose2d();
            borderContact = touchesBorder(current);
            parkPose = borderContact ? safeParkPose(current) : current;
            RobotLog.ii(TAG, "T-3s: pose=" + current
                    + " contact=" + borderContact + " parkPose=" + parkPose);

            if (!borderContact) {
                // 未接触场地边框：停止，停靠点即当前位置
                stopDrive();
                packet.put("AdaptivePark", "no border contact -> stop");
                packet.put("AdaptivePark park x", parkPose.position.x);
                packet.put("AdaptivePark park y", parkPose.position.y);
                done = true;
                return false;
            }

            // 接触：沿最小平移方向开出边界，目标位姿即本局停靠点
            trajectoryAction = drive.actionBuilder(current)
                    .strafeToLinearHeading(parkPose.position, parkPose.heading.toDouble())
                    .build();
        }

        packet.put("AdaptivePark", "border contact -> moving inside");
        packet.put("AdaptivePark park x", parkPose.position.x);
        packet.put("AdaptivePark park y", parkPose.position.y);
        packet.put("AdaptivePark park heading(deg)", Math.toDegrees(parkPose.heading.toDouble()));

        boolean running = trajectoryAction.run(packet);
        if (!running) {
            stopDrive();
            done = true;
        }
        return running;
    }

    /**
     * 整机外接矩形是否接触场地边框（四个角点任一到达或越过 ±{@link HypParams#fieldHalfSizeIn}）。
     *
     * @param pose 底盘中心位姿
     * @return 接触返回 {@code true}
     */
    public static boolean touchesBorder(Pose2d pose) {
        double half = HypParams.fieldHalfSizeIn;
        for (Vector2D corner : ChassisGeometry.footprintCorners(pose)) {
            if (Math.abs(corner.getX()) >= half - BORDER_EPS
                    || Math.abs(corner.getY()) >= half - BORDER_EPS) {
                return true;
            }
        }
        return false;
    }

    /**
     * 由当前位姿求「不接触场地边框」的最小平移停靠位姿（航向保持不变）。
     *
     * <p>做法是求整机外接矩形在场地坐标轴上的半投影：
     * <pre>
     *   ex = |L·cosθ| + |W·sinθ|
     *   ey = |L·sinθ| + |W·cosθ|
     * </pre>
     * 再把底盘中心夹到 {@code [-limit+ex, limit-ex]}（y 同理），
     * 其中 {@code limit = fieldHalfSizeIn - parkBorderMarginIn}。这样得到一个
     * <b>轴对齐包围盒完全落在安全区内的最小位移解</b>，无需旋转，也不会舍近求远。
     *
     * @param pose 当前位姿
     * @return 停靠位姿（位置已内缩，航向不变）
     */
    public static Pose2d safeParkPose(Pose2d pose) {
        double heading = pose.heading.toDouble();
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        double l = HypParams.chassisHalfLengthIn;
        double w = HypParams.chassisHalfWidthIn;
        double ex = Math.abs(l * c) + Math.abs(w * s);
        double ey = Math.abs(l * s) + Math.abs(w * c);
        double limit = HypParams.fieldHalfSizeIn - HypParams.parkBorderMarginIn;

        double loX = -limit + ex;
        double hiX = limit - ex;
        double loY = -limit + ey;
        double hiY = limit - ey;

        // 极端情况下（机器人外接盒比安全区还大）无法满足约束，退回场地中心
        double cx = (loX > hiX) ? 0.0 : clamp(pose.position.x, loX, hiX);
        double cy = (loY > hiY) ? 0.0 : clamp(pose.position.y, loY, hiY);
        return new Pose2d(cx, cy, heading);
    }

    /** 检测时刻整机是否接触场地边框。 */
    public boolean borderContact() {
        return borderContact;
    }

    /** 本局停靠点（检测前为 {@code null}）。 */
    public Pose2d parkPose() {
        return parkPose;
    }

    private void stopDrive() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }
}
