package org.firstinspires.ftc.teamcode.架构练习;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;

/**
 * 炮台（架构练习版）—— 对应架构图中的「Turret」节点。
 *
 * <p>组合原 {@link Turret}（复用其平转 + 双发射器的完整控制），并在其上增加架构图的跨层同步逻辑：
 * <b>发射完毕清空计数</b>——检测扳机从「释放」回落（true → false 下降沿），即一次发射结束，
 * 此时球道已空，调用 {@link RobotPosition1#resetBallCount()} 清空计数。
 *
 * <p><b>数据同步方向</b>：Turret1 →（触发）→ RobotPosition1.resetBallCount()。
 *
 * <p><b>注意</b>：下降沿检测需对比上一帧与当前帧的扳机状态，故本类在 {@link #update} 里维护
 * {@code lastTriggerBig / lastTriggerSmall} 两个私有字段。
 */
public class Turret1 {

    /** 被组合的原炮台控制器 */
    private final Turret turret;

    /** 只读依赖：状态估计器（触发发射完毕清空计数） */
    private final RobotPosition1 robotPosition;

    /** 上一帧扳机状态，用于检测「发射完毕」（true → false 下降沿） */
    private boolean lastTriggerBig = false;
    private boolean lastTriggerSmall = false;

    /**
     * @param hardwareMap   硬件映射
     * @param telemetry     遥测（传给原 Turret）
     * @param robotPosition 只读依赖的状态估计器
     */
    public Turret1(HardwareMap hardwareMap, Telemetry telemetry, RobotPosition1 robotPosition) {
        // Init 顺序：先下属（原 Turret），再记录依赖
        this.turret = new Turret(hardwareMap, telemetry);
        this.robotPosition = robotPosition;
    }

    /**
     * 每帧调用：下发炮台目标，并在扳机下降沿（发射完毕）时清空计数。
     *
     * @param yaw          目标平转角度（弧度）
     * @param pitchBig     大球俯仰舵机目标位置
     * @param pitchSmall   小球俯仰舵机目标位置
     * @param speedBig     大球飞轮目标转速 (tick/s)
     * @param speedSmall   小球飞轮目标转速 (tick/s)
     * @param triggerBig   大球扳机是否释放（true = 发射）
     * @param triggerSmall 小球扳机是否释放（true = 发射）
     * @return 两个发射器是否具备发射条件
     */
    public Turret.FireReady update(double yaw, double pitchBig, double pitchSmall,
                                   int speedBig, int speedSmall,
                                   boolean triggerBig, boolean triggerSmall) {
        Turret.FireReady ready = turret.update(yaw, pitchBig, pitchSmall,
                speedBig, speedSmall, triggerBig, triggerSmall);

        // 发射完毕：扳机从「释放」回落（true → false）即一次发射结束，球道已空
        if ((lastTriggerBig && !triggerBig) || (lastTriggerSmall && !triggerSmall)) {
            robotPosition.resetBallCount();
        }
        lastTriggerBig = triggerBig;
        lastTriggerSmall = triggerSmall;

        return ready;
    }

    /** 停止炮台（退出时调用） */
    public void stop() {
        turret.stop();
    }

    /** 输出炮台遥测（调用方需自行 telemetry.update()） */
    public void setTelemetry() {
        turret.setTelemetry();
    }
}
