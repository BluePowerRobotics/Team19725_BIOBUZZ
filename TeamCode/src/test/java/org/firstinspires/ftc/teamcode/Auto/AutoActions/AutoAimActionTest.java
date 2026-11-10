package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyBoolean;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.junit.Before;
import org.junit.Test;

/** {@link AutoAimAction} 单元测试（底盘不动、炮台自动瞄准与发射）。 */
public class AutoAimActionTest {

    @Before
    public void setUpTuning() {
        AutoAimAction.allowFire = true;
        AutoAimAction.preload = true;
        AutoAimAction.timeoutMs = 30_000;
    }

    /** 首帧未发射（big/small 均 false），无下降沿 → 继续运行返回 true。 */
    @Test
    public void run_firstFrame_notFiring_returnsTrue() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            Turret turret = mock(Turret.class);
            when(turret.update(anyBoolean(), anyBoolean(), anyBoolean(), anyBoolean()))
                    .thenReturn(new Turret.FireReady(false, false));
            when(turret.isBigFiring()).thenReturn(false);
            when(turret.isSmallFiring()).thenReturn(false);

            AutoAimAction action = new AutoAimAction(m.drive, turret);
            assertTrue(action.run(new TelemetryPacket()));

            // 底盘每帧下发 0 速度保持静止
            verify(m.drive).setDrivePowers(any());
            // 炮台每帧被推进
            verify(turret).update(anyBoolean(), anyBoolean(), anyBoolean(), anyBoolean());
        }
    }

    /** 大球扳机 true → false 的下降沿 → 发射完成，返回 false 并停止炮台。 */
    @Test
    public void run_bigFiringReleased_returnsFalseAndStops() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            Turret turret = mock(Turret.class);
            when(turret.update(anyBoolean(), anyBoolean(), anyBoolean(), anyBoolean()))
                    .thenReturn(new Turret.FireReady(true, false));
            // 第一帧 bigFiring=true，第二帧 bigFiring=false
            when(turret.isBigFiring()).thenReturn(true, false);
            when(turret.isSmallFiring()).thenReturn(false);

            AutoAimAction action = new AutoAimAction(m.drive, turret);
            TelemetryPacket p1 = new TelemetryPacket();
            assertTrue(action.run(p1));   // 第一帧：bigFiring=true，无下降沿
            assertFalse(action.run(new TelemetryPacket())); // 第二帧：下降沿 → 结束

            verify(turret).stop();
        }
    }

    /** 小球扳机下降沿同样触发结束。 */
    @Test
    public void run_smallFiringReleased_returnsFalse() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            Turret turret = mock(Turret.class);
            when(turret.update(anyBoolean(), anyBoolean(), anyBoolean(), anyBoolean()))
                    .thenReturn(new Turret.FireReady(false, true));
            when(turret.isBigFiring()).thenReturn(false);
            when(turret.isSmallFiring()).thenReturn(true, false);

            AutoAimAction action = new AutoAimAction(m.drive, turret);
            assertTrue(action.run(new TelemetryPacket()));
            assertFalse(action.run(new TelemetryPacket()));
            verify(turret).stop();
        }
    }

    /** 超时保护：timeoutMs=0 时首帧即判定超时并结束。 */
    @Test
    public void run_timeout_returnsFalseAndStops() {
        AutoAimAction.timeoutMs = 0;
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            Turret turret = mock(Turret.class);
            when(turret.update(anyBoolean(), anyBoolean(), anyBoolean(), anyBoolean()))
                    .thenReturn(new Turret.FireReady(false, false));
            when(turret.isBigFiring()).thenReturn(false);
            when(turret.isSmallFiring()).thenReturn(false);

            AutoAimAction action = new AutoAimAction(m.drive, turret);
            assertFalse(action.run(new TelemetryPacket()));
            verify(turret).stop();
        }
    }

    /** 已完成的 Action 再次调用 run 直接返回 false，不重复下发指令。 */
    @Test
    public void run_afterFinish_returnsFalse() {
        AutoAimAction.timeoutMs = 0;
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            Turret turret = mock(Turret.class);
            when(turret.update(anyBoolean(), anyBoolean(), anyBoolean(), anyBoolean()))
                    .thenReturn(new Turret.FireReady(false, false));
            when(turret.isBigFiring()).thenReturn(false);
            when(turret.isSmallFiring()).thenReturn(false);

            AutoAimAction action = new AutoAimAction(m.drive, turret);
            assertFalse(action.run(new TelemetryPacket()));
            assertFalse(action.run(new TelemetryPacket()));
        }
    }
}
