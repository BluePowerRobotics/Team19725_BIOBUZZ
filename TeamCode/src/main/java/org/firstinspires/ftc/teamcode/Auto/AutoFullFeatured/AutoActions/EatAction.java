package org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * 吃球动作 Action：<b>底盘原地保持不动</b>，{@link Sweeper} 以吃球速度运转
 * {@link #eatDurationMs}（默认 2500 ms = 2.5 s）后停转。
 *
 * <p>非阻塞，每帧：
 * <ol>
 *   <li>{@code drive.setDrivePowers(0, 0, 0)} —— 底盘驱动归零，保证原地不动；</li>
 *   <li>{@link Sweeper#setEat()} + {@link Sweeper#update()} —— Sweeper 每帧会自动归零目标，
 *       故必须每帧重新置位；</li>
 * </ol>
 * 持续到时长后调用 {@link Sweeper#setStop()} 并结束。时长可在 FTC Dashboard 实时调整。
 *
 * <p>典型用法：在 {@code GoToXxxEatPointAction} 到位后入队本 Action。
 */
@Config
public class EatAction implements Action {

    /** 吃球持续时长 (ms)：默认 2500（2.5 s），Dashboard 可调 */
    public static long eatDurationMs = 2500;

    /** 底盘（锁死用） */
    private final MecanumDrive drive;
    private final Sweeper sweeper;
    private final long durationMs;

    private ElapsedTime timer;
    private boolean initialized = false;

    public EatAction(Sweeper sweeper) {
        this(RobotPosition.getInstance().getDrive(), sweeper, eatDurationMs);
    }

    public EatAction(Sweeper sweeper, long durationMs) {
        this(RobotPosition.getInstance().getDrive(), sweeper, durationMs);
    }

    private EatAction(MecanumDrive drive, Sweeper sweeper, long durationMs) {
        this.drive = drive;
        this.sweeper = sweeper;
        this.durationMs = durationMs;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        // 原地不动：每帧把底盘驱动归零
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        if (!initialized) {
            timer = new ElapsedTime();
            // 兜底：无论移动 Action 是否已提前放铲，吃球前一定让铲子处于放下位置
            sweeper.setShovelDown();
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

        packet.put("Eat", "Eating");
        packet.put("Eat elapsed(ms)", elapsedMs);
        packet.put("Eat duration(ms)", durationMs);
        packet.put("Eat sweeper vel(tick/s)", sweeper.getVel());
        packet.put("Eat sweeper current(A)", sweeper.getCurrent());
        return true;
    }
}
