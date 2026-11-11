package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * 自动瞄准与发射 Action（<b>底盘自主旋转辅助</b>版）。
 *
 * <p>与 {@link AutoAimAction} 的区别：本 Action 让<b>底盘主动转向球门</b>，
 * 使炮台需要旋转的角度最小，从而缩短炮台到达目标位置的总时间。
 *
 * <p><b>控制分工</b>：
 * <ul>
 *   <li>底盘：PD 控制角速度，使车头朝向目标球门
 *       （{@code desiredHeading = atan2(goalY-y, goalX-x)}）；</li>
 *   <li>炮台：{@link Turret#update(boolean, boolean, boolean, boolean)} 自动瞄准并发射。</li>
 * </ul>
 * 底盘转向把目标拉到炮台行程中心附近，炮台只需补偿剩余小角度，响应最快。
 *
 * <p><b>结束条件</b>（任一满足）：
 * <ul>
 *   <li>大球或小球扳机释放完成（下降沿）；</li>
 *   <li>超过 {@link #timeoutMs} 毫秒。</li>
 * </ul>
 *
 * <p>本 Action 不判断底盘是否已对准——发射权完全交给 Turret 的就绪判据；
 * 只要炮台到位且机构就绪就会自动扣扳机，与底盘当前角度无关。
 */
@Config
public class AutoTurretAimShootWithChassisAction implements Action {

    /** 是否允许 Turret 扣扳机 */
    public static boolean allowFire = true;
    /** 是否启用预载 */
    public static boolean preload = true;
    /** 底盘转向环 P 增益 (rad/s per rad) */
    public static double yawKp = 3.0;
    /** 底盘转向环 D 增益 (rad/s per rad/s) */
    public static double yawKd = 0.15;
    /** 底盘角速度指令上限 (rad/s) */
    public static double maxOmega = Math.PI;
    /** 底盘转向死区 (rad)：航向误差小于该值时不再下发角速度，避免抖振 */
    public static double headingTol = Math.toRadians(1.5);
    /** 超时时间 (ms) */
    public static long timeoutMs = 6000;

    private final MecanumDrive drive;
    private final Turret turret;

    private ElapsedTime timer;
    private boolean initialized = false;
    private double lastError = 0;
    private double lastTime = 0;
    private boolean lastBigFiring = false;
    private boolean lastSmallFiring = false;
    private boolean finished = false;

    /**
     * @param drive  底盘
     * @param turret 炮台控制器
     */
    public AutoTurretAimShootWithChassisAction(MecanumDrive drive, Turret turret) {
        this.drive = drive;
        this.turret = turret;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (finished) {
            return false;
        }
        if (!initialized) {
            timer = new ElapsedTime();
            lastTime = timer.seconds();
            initialized = true;
        }

        RobotPosition rp = RobotPosition.getInstance();
        Pose2d pose = rp.getPose2d();
        Pose2d goal = rp.getGoalPose();

        // ---- 底盘：PD 跟踪「车头朝向球门」 ----
        double desiredHeading = Math.atan2(
                goal.position.y - pose.position.y,
                goal.position.x - pose.position.x);
        double error = Math.atan2(
                Math.sin(desiredHeading - pose.heading.toDouble()),
                Math.cos(desiredHeading - pose.heading.toDouble()));

        double now = timer.seconds();
        double dt = now - lastTime;
        lastTime = now;
        if (dt <= 0 || dt > 1.0) {
            dt = 0.02;
        }
        double dError = (error - lastError) / dt;
        lastError = error;

        double omega;
        if (Math.abs(error) <= headingTol) {
            omega = 0.0;
        } else {
            omega = yawKp * error + yawKd * dError;
            omega = Math.max(-maxOmega, Math.min(maxOmega, omega));
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), omega));

        // ---- 炮台：自动瞄准 + 自动发射 ----
        Turret.FireReady ready = turret.update(allowFire, preload, false, false);

        // 发射完成检测
        boolean bigFiring = turret.isBigFiring();
        boolean smallFiring = turret.isSmallFiring();
        boolean bigReleased = lastBigFiring && !bigFiring;
        boolean smallReleased = lastSmallFiring && !smallFiring;
        lastBigFiring = bigFiring;
        lastSmallFiring = smallFiring;

        long elapsedMs = (long) timer.milliseconds();
        if (bigReleased || smallReleased) {
            finish(packet, ready, elapsedMs, false);
            return false;
        }
        if (elapsedMs >= timeoutMs) {
            finish(packet, ready, elapsedMs, true);
            return false;
        }

        packet.put("AutoAimChassis", "Aiming/Firing");
        packet.put("AutoAimChassis heading error(deg)", Math.toDegrees(error));
        packet.put("AutoAimChassis omega(rad/s)", omega);
        packet.put("AutoAimChassis big ready", ready.big);
        packet.put("AutoAimChassis small ready", ready.small);
        packet.put("AutoAimChassis big firing", bigFiring);
        packet.put("AutoAimChassis small firing", smallFiring);
        packet.put("AutoAimChassis elapsed(ms)", elapsedMs);
        return true;
    }

    /** 结束：底盘停止，炮台停止 */
    private void finish(TelemetryPacket packet, Turret.FireReady ready,
                        long elapsedMs, boolean timeout) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        turret.stop();
        finished = true;

        packet.put("AutoAimChassis", timeout ? "Timeout" : "Fired");
        packet.put("AutoAimChassis elapsed(ms)", elapsedMs);
        packet.put("AutoAimChassis big ready", ready.big);
        packet.put("AutoAimChassis small ready", ready.small);
    }
}
