package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;

/**
 * 吃球动作 Action：驱动 {@link Sweeper} 以吃球速度运转指定时长后停转。
 *
 * <p>非阻塞，每帧调用 {@link Sweeper#setEat()} + {@link Sweeper#update()} 下发目标速度
 * （Sweeper 每帧会自动归零目标，故必须每帧重新置位）；持续 {@link #eatDurationMs}
 * 毫秒后调用 {@link Sweeper#setStop()} 并结束。时长可在 FTC Dashboard 实时调整。
 *
 * <p><b>满额提前结束</b>：Sweeper 在球道计数达到 {@link Sweeper#BallCapacity} 时会强制
 * 停吸，此时继续跑满时长的剩余时间只是空转。因此本 Action 每帧检查
 * {@link Sweeper#isBallLaneFull()}，一旦球道已满立即停转并结束，比固定时长更省时。
 * 前提：主循环须先调用 {@code RobotPosition.update()} 再执行 Action，否则计数会滞后一帧。
 *
 * <p>典型用法：在 {@code GoToXxxEatPointAction} 到位后入队本 Action。
 */
@Config
public class EatAction implements Action {

    /** 默认吃球持续时长 (ms)，todo: 实机标定 */
    public static long eatDurationMs = 2000;

    private final Sweeper sweeper;
    private final long durationMs;

    private ElapsedTime timer;
    private boolean initialized = false;

    public EatAction(Sweeper sweeper) {
        this(sweeper, eatDurationMs);
    }

    public EatAction(Sweeper sweeper, long durationMs) {
        this.sweeper = sweeper;
        this.durationMs = durationMs;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            timer = new ElapsedTime();
            initialized = true;
        }

        long elapsedMs = (long) timer.milliseconds();
        if (elapsedMs >= durationMs) {
            // 到时停转，避免 Sweeper 在 Action 结束后继续维持吃球速度
            sweeper.setStop();
            sweeper.update();
            packet.put("Eat", "Done");
            return false;
        }

        sweeper.setEat();
        sweeper.update();

        // 球道已满：提前结束，避免满额保护把吸球拦下后仍空转到超时
        if (sweeper.isBallLaneFull()) {
            sweeper.setStop();
            sweeper.update();
            packet.put("Eat", "Full");
            packet.put("Eat ball count", sweeper.getBallCount());
            return false;
        }

        packet.put("Eat", "Eating");
        packet.put("Eat elapsed(ms)", elapsedMs);
        packet.put("Eat ball count", sweeper.getBallCount());
        packet.put("Eat duration(ms)", durationMs);
        packet.put("Eat sweeper vel(tick/s)", sweeper.getVel());
        packet.put("Eat sweeper current(A)", sweeper.getCurrent());
        return true;
    }
}
