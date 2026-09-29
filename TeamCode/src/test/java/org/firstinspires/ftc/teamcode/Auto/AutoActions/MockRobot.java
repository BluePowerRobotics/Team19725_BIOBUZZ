package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.mockito.MockedStatic;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

/**
 * Action 单元测试的共享 Mock 辅助：
 * 在 {@link RobotPosition#getInstance()} 单例与 {@link MecanumDrive#actionBuilder}
 * 轨迹构建链上打桩，使 Action 可在无硬件环境下构造。
 *
 * <p>使用方式（try-with-resources 保证静态 mock 释放）：
 * <pre>
 * try (MockRobot m = new MockRobot(startPose)) {
 *     MyAction a = new MyAction(...);
 *     ...
 *     when(m.trajectoryAction.run(any())).thenReturn(false);
 *     assertFalse(a.run(packet));
 * }
 * </pre>
 */
public class MockRobot implements AutoCloseable {

    public final MockedStatic<RobotPosition> robotStatic;
    public final RobotPosition robotPosition;
    public final MecanumDrive drive;
    public final TrajectoryActionBuilder builder;
    /** 由 build() 返回的轨迹 Action（mock），可在测试中对其 run() 打桩 */
    public final Action trajectoryAction;

    public MockRobot(Pose2d startPose) {
        robotStatic = org.mockito.Mockito.mockStatic(RobotPosition.class);
        try {
            robotPosition = mock(RobotPosition.class);
            drive = mock(MecanumDrive.class);
            builder = mock(TrajectoryActionBuilder.class);
            trajectoryAction = mock(Action.class);

            robotStatic.when(RobotPosition::getInstance).thenReturn(robotPosition);
            when(robotPosition.getDrive()).thenReturn(drive);
            when(robotPosition.getPose2d()).thenReturn(startPose);
            when(drive.actionBuilder(any(Pose2d.class))).thenReturn(builder);
            when(builder.strafeToLinearHeading(any(), anyDouble())).thenReturn(builder);
            when(builder.build()).thenReturn(trajectoryAction);
        } catch (RuntimeException e) {
            // 构造中途失败时必须释放静态 mock，否则会污染同一线程的后续测试
            robotStatic.close();
            throw e;
        }
    }

    @Override
    public void close() {
        robotStatic.close();
    }
}
