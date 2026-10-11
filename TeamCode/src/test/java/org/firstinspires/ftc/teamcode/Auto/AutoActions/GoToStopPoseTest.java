package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.junit.Test;

/** {@link GoToStopPose} 单元测试。 */
public class GoToStopPoseTest {

    @Test
    public void run_returnsTrajectoryResult() {
        Pose2d start = new Pose2d(0, 0, 0);
        Pose2d stop = new Pose2d(0, 24, Math.PI);
        try (MockRobot m = new MockRobot(start)) {
            // 模拟轨迹仍在运行
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(true);
            GoToStopPose action = new GoToStopPose(m.drive, stop);
            assertTrue(action.run(new TelemetryPacket()));

            // 模拟轨迹结束
            when(m.trajectoryAction.run(any(TelemetryPacket.class))).thenReturn(false);
            assertFalse(action.run(new TelemetryPacket()));
        }
    }
}
