package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * 自动瞄准与发射 Action：<b>底盘完全不动</b>，由 {@link Turret} 独立完成瞄准与发射。
 *
 * <p>与旧版 {@code AutoAimAction}（仅用 PD 控制底盘转向）的区别：
 * <ul>
 *   <li>瞄准主体是<b>炮台</b>（平转 + 俯仰 + 飞轮），底盘全程
 *       {@code setDrivePowers(0,0,0)} 保持静止；</li>
 *   <li>发射由 {@link Turret#update(boolean, boolean, boolean, boolean)} 内部的
 *       {@code TriggerGate} 自动管理：平转到位 + 飞轮到位 + 俯仰到位即扣扳机，
 *       并保持 {@code fireHoldTime}；本 Action 只需判「发射完成」即可。</li>
 * </ul>
 *
 * <p><b>球门来源</b>：{@link Turret} 内部从 {@link RobotPosition#getGoalPose()} 读取，
 * 该值已由 HIVE 抬升侧自动选择（AudienceUp / AudienceDown），本 Action 不干预。
 *
 * <p><b>结束条件</b>（任一满足）：
 * <ul>
 *   <li>大球或小球扳机从「释放」回到「关闭」（下降沿）—— 视为一次发射完成；</li>
 *   <li>超过 {@link #timeoutMs} 毫秒 —— 强制结束并由上层状态机处理。</li>
 * </ul>
 *
 * <p><b>必须每帧调用 {@code turret.update()}</b>（由本 Action 的 run 负责），
 * 飞轮速度环与平转位置环都依赖周期刷新。
 */
@Config
public class AutoAimAction implements Action {

    /** 是否允许 Turret 扣扳机（false 时只预载、不发射，用于单独调试瞄准） */
    public static boolean allowFire = true;
    /** 是否启用预载（飞轮按预载转速待命，缩短发射响应） */
    public static boolean preload = true;
    /** 超时时间 (ms)，超时强制结束（交由状态机处理） */
    public static long timeoutMs = 5000;

    private final MecanumDrive drive;
    private final Turret turret;

    private ElapsedTime timer;
    private boolean initialized = false;
    private boolean lastBigFiring = false;
    private boolean lastSmallFiring = false;
    private boolean finished = false;

    /**
     * @param drive  底盘（用于每帧下发 0 速度保持静止）
     * @param turret 炮台控制器
     */
    public AutoAimAction(MecanumDrive drive, Turret turret) {
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
            initialized = true;
        }

        // 底盘全程保持静止：炮台独立瞄准与发射
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        // 炮台自动瞄准 + 自动发射：到位后内部自动扣扳机
        Turret.FireReady ready = turret.update(allowFire, preload, false, false);

        // 发射完成检测：任一扳机 true → false 的下降沿
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

        packet.put("AutoAim", "Aiming/Firing");
        packet.put("AutoAim allowFire", allowFire);
        packet.put("AutoAim big ready", ready.big);
        packet.put("AutoAim small ready", ready.small);
        packet.put("AutoAim big firing", bigFiring);
        packet.put("AutoAim small firing", smallFiring);
        packet.put("AutoAim elapsed(ms)", elapsedMs);
        return true;
    }

    /** 结束：底盘停止，炮台停止（停飞轮、关扳机） */
    private void finish(TelemetryPacket packet, Turret.FireReady ready,
                        long elapsedMs, boolean timeout) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        turret.stop();
        finished = true;

        packet.put("AutoAim", timeout ? "Timeout" : "Fired");
        packet.put("AutoAim elapsed(ms)", elapsedMs);
        packet.put("AutoAim big ready", ready.big);
        packet.put("AutoAim small ready", ready.small);
    }
}
