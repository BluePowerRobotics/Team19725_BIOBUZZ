package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.AdaptiveEKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.junit.Before;
import org.junit.Test;

/** {@link AutoAimAction} 单元测试。 */
public class AutoAimActionTest {

    @Before
    public void setUpTuning() {
        // 收敛判定容差与定时参数（避免依赖默认值的不确定性）
        AutoAimAction.headingTol = Math.toRadians(2.0);
        AutoAimAction.omegaTol = 0.15;
        AutoAimAction.settleMs = 0;
        AutoAimAction.timeoutMs = 30_000;
    }

    /** 机器人已对准球门（误差≈0、角速度≈0），应在首帧即判定瞄准完成。 */
    @Test
    public void run_aligned_returnsFalseImmediately() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            AdaptiveEKFLocalizer localizer = mock(AdaptiveEKFLocalizer.class);
            when(m.robotPosition.getFusionLocalizer()).thenReturn(localizer);
            // 球门 RedAudienceUp = (17.7, -12.75)；机器人放在其正左方、朝向 0 → 期望航向 0
            when(localizer.getPose()).thenReturn(new Pose2d(0, -12.75, 0));
            when(m.robotPosition.getHiveState()).thenReturn(MT1Localizer.HiveState.AUDIENCE_UP);
            when(m.robotPosition.getOmega()).thenReturn(0.0);

            AutoAimAction action = new AutoAimAction(TeamColor.RED);
            assertFalse(action.run(new TelemetryPacket()));
            // 对准路径下 PD 段与 finish() 都会下发动力指令
            org.mockito.Mockito.verify(m.drive, org.mockito.Mockito.atLeastOnce())
                    .setDrivePowers(any());
        }
    }

    /** 机器人航向偏离球门较大，且角速度不为零 → 未收敛，继续瞄准返回 true。 */
    @Test
    public void run_misaligned_returnsTrue() {
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            AdaptiveEKFLocalizer localizer = mock(AdaptiveEKFLocalizer.class);
            when(m.robotPosition.getFusionLocalizer()).thenReturn(localizer);
            when(localizer.getPose()).thenReturn(new Pose2d(0, -12.75, Math.PI / 2)); // 朝向偏离 90°
            when(m.robotPosition.getHiveState()).thenReturn(MT1Localizer.HiveState.AUDIENCE_UP);
            when(m.robotPosition.getOmega()).thenReturn(0.0);

            AutoAimAction action = new AutoAimAction(TeamColor.RED);
            assertTrue(action.run(new TelemetryPacket()));
        }
    }

    /** 超时保护：timeoutMs 设为 0，首帧即判定超时并结束。 */
    @Test
    public void run_timeout_returnsFalse() {
        AutoAimAction.timeoutMs = 0;
        try (MockRobot m = new MockRobot(new Pose2d(0, 0, 0))) {
            AdaptiveEKFLocalizer localizer = mock(AdaptiveEKFLocalizer.class);
            when(m.robotPosition.getFusionLocalizer()).thenReturn(localizer);
            when(localizer.getPose()).thenReturn(new Pose2d(0, -12.75, Math.PI / 2));
            when(m.robotPosition.getHiveState()).thenReturn(MT1Localizer.HiveState.AUDIENCE_UP);
            when(m.robotPosition.getOmega()).thenReturn(0.0);

            AutoAimAction action = new AutoAimAction(TeamColor.RED);
            assertFalse(action.run(new TelemetryPacket()));
        }
    }
}
