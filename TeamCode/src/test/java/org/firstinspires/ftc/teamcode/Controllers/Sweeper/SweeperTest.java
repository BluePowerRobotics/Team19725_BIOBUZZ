package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import static org.junit.Assert.assertEquals;
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
 * {@link Sweeper} 满球保护单元测试：验证按命令来源区分两种满球场景。
 *
 * <ul>
 *   <li>{@link Sweeper#setEat()}：满球时必须停止吸球（目标速度归零）；</li>
 *   <li>{@link Sweeper#setGiveArtifact()}：满球时必须放行，用于把球送给炮台发射；</li>
 *   <li>未满球时吸球正常；出球方向不受上限限制。</li>
 * </ul>
 */
public class SweeperTest {

    private static final int BALL_MAX = 4;

    private enum Command { EAT, GIVE_ARTIFACT, OUTPUT }

    /**
     * 在 {@link RobotPosition} 静态单例 mock 作用域内构造 Sweeper，下发一次命令并更新，
     * 返回本帧实际下发的目标速度（不访问真实硬件）。
     */
    private static int targetVelocityAfter(int ballCount, Command command) {
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
                default:
                    break;
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
}
