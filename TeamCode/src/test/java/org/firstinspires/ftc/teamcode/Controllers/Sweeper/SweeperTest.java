package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.mockStatic;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.junit.Test;
import org.mockito.MockedStatic;

/**
 * {@link Sweeper} 单元测试：
 *
 * <ul>
 *   <li>满球规则：吸球满球停转、送球满球放行、出球不受限；</li>
 *   <li>吐球：优先级、定时自动结束、提前停止、重新计时、零时长边界。</li>
 * </ul>
 */
public class SweeperTest {

    private static final int BALL_MAX = 4;

    private enum Command { EAT, GIVE_ARTIFACT, OUTPUT, EJECT_STOP }

    /** 用 Mock 硬件构造 Sweeper（构造过程不访问 RobotPosition）。 */
    private static Sweeper newSweeper() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        when(hardwareMap.get(DcMotorEx.class, "sweeperMotor")).thenReturn(mock(DcMotorEx.class));
        when(hardwareMap.get(Servo.class, "shovelServo")).thenReturn(mock(Servo.class));
        return new Sweeper(hardwareMap, null);
    }

    /**
     * 在 {@link RobotPosition} 静态单例 mock 作用域内构造 Sweeper，下发一次命令并更新，
     * 返回本帧实际下发的目标速度（不访问真实硬件）。
     */
    private static int targetVelocityAfter(int ballCount, Command command) {
        return targetVelocityAfter(ballCount, command, false);
    }

    /**
     * @param eject 是否在本帧同时触发吐球（用于验证吐球的优先级）
     */
    private static int targetVelocityAfter(int ballCount, Command command, boolean eject) {
        DcMotorEx motor = mock(DcMotorEx.class);
        HardwareMap hardwareMap = mock(HardwareMap.class);
        Servo shovelServo = mock(Servo.class);
        when(hardwareMap.get(DcMotorEx.class, "sweeperMotor")).thenReturn(motor);
        when(hardwareMap.get(Servo.class, "shovelServo")).thenReturn(shovelServo);

        RobotPosition robotPosition = mock(RobotPosition.class);
        when(robotPosition.getBallCount()).thenReturn(ballCount);

        try (MockedStatic<RobotPosition> ignored = mockStatic(RobotPosition.class)) {
            ignored.when(RobotPosition::getInstanceOrNull).thenReturn(robotPosition);

            Sweeper sweeper = new Sweeper(hardwareMap, null);
            switch (command) {
                case EAT:
                    sweeper.setEat();
                    break;
                case GIVE_ARTIFACT:
                    sweeper.setGiveArtifact();
                    break;
                case OUTPUT:
                    sweeper.setOutput();
                    break;
                case EJECT_STOP:
                    // 先触发吐球，再用停止指令取消
                    sweeper.setEject();
                    sweeper.setStop();
                    break;
                default:
                    break;
            }
            if (eject) {
                sweeper.setEject();
            }
            sweeper.update();
            return sweeper.getTargetVelocity();
        }
    }

    /** 满球 + 吸球：停止吸球。 */
    @Test
    public void fullLaneStopsEat() {
        assertEquals(0, targetVelocityAfter(BALL_MAX, Command.EAT));
    }

    /** 满球 + 送球：正常放行，允许炮台发射。 */
    @Test
    public void fullLaneAllowsGiveArtifact() {
        assertEquals(Sweeper.GiveTheArtifactVel,
                targetVelocityAfter(BALL_MAX, Command.GIVE_ARTIFACT));
    }

    /** 未满球 + 吸球：正常吸球。 */
    @Test
    public void notFullAllowsEat() {
        assertEquals(Sweeper.EatVel,
                targetVelocityAfter(BALL_MAX - 1, Command.EAT));
    }

    /** 满球 + 出球：不受上限限制。 */
    @Test
    public void fullLaneAllowsOutput() {
        assertEquals(Sweeper.OutputVel,
                targetVelocityAfter(BALL_MAX, Command.OUTPUT));
    }

    /**
     * 吐球优先：吐球期间即使满球吸球 / 送球命令，也强制按吐球速度反向排出。
     */
    @Test
    public void ejectOverridesOtherCommands() {
        double oldDuration = Sweeper.EjectDurationSec;
        Sweeper.EjectDurationSec = 10.0;   // 保证测试期间不会到期
        try {
            assertEquals(Sweeper.EjectVel,
                    targetVelocityAfter(BALL_MAX, Command.EAT, true));
            assertEquals(Sweeper.EjectVel,
                    targetVelocityAfter(BALL_MAX, Command.GIVE_ARTIFACT, true));
        } finally {
            Sweeper.EjectDurationSec = oldDuration;
        }
    }

    /** 停止指令会取消正在进行的吐球。 */
    @Test
    public void stopCancelsEject() {
        assertEquals(0, targetVelocityAfter(BALL_MAX, Command.EJECT_STOP));
    }

    /** 默认空闲：不吐球、剩余时间为 0。 */
    @Test
    public void ejectIdleByDefault() {
        Sweeper sweeper = newSweeper();
        assertFalse(sweeper.isEjecting());
        assertEquals(0.0, sweeper.getEjectRemainingSec(), 1e-9);
    }

    /** 启动后未到时长前持续吐球，剩余时间随计时递减。 */
    @Test
    public void ejectKeepsRunningUntilDuration() {
        Sweeper sweeper = newSweeper();
        double duration = Sweeper.EjectDurationSec;

        sweeper.setEject(100.0);
        assertTrue(sweeper.isEjecting());

        assertTrue(sweeper.updateEject(100.1));
        assertEquals(duration - 0.1, sweeper.getEjectRemainingSec(), 1e-6);

        // 距结束还有 0.01 s，仍处于吐球状态
        assertTrue(sweeper.updateEject(100.0 + duration - 0.01));
    }

    /** 到达时长自动结束，剩余时间归零。 */
    @Test
    public void ejectAutomaticallyStopsAtDuration() {
        Sweeper sweeper = newSweeper();
        sweeper.setEject(10.0);

        assertFalse(sweeper.updateEject(10.0 + Sweeper.EjectDurationSec));
        assertFalse(sweeper.isEjecting());
        assertEquals(0.0, sweeper.getEjectRemainingSec(), 1e-9);
    }

    /** 吐球中再次触发重新计时，不会因旧计时到期而提前结束。 */
    @Test
    public void ejectRestartResetsTimer() {
        Sweeper sweeper = newSweeper();
        double duration = Sweeper.EjectDurationSec;

        sweeper.setEject(1.0);
        assertTrue(sweeper.updateEject(1.0 + duration * 0.75));

        sweeper.setEject(1.0 + duration * 0.75);              // 重新触发
        assertTrue(sweeper.updateEject(1.0 + duration * 1.5)); // 旧计时本应到期
        assertTrue(sweeper.isEjecting());
    }

    /** 时长为 0 时，下一次推进立即结束（不会无限反转）。 */
    @Test
    public void ejectZeroDurationStopsImmediately() {
        double old = Sweeper.EjectDurationSec;
        Sweeper.EjectDurationSec = 0;
        try {
            Sweeper sweeper = newSweeper();
            sweeper.setEject(0.0);
            assertFalse(sweeper.updateEject(0.0));
            assertFalse(sweeper.isEjecting());
        } finally {
            Sweeper.EjectDurationSec = old;
        }
    }
}
