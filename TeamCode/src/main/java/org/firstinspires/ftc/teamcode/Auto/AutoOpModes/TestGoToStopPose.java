package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToStopPose;

/**
 * 实机测试：{@link GoToStopPose} —— 前往所选队伍的停车位姿并停车。
 * Init：A/B 选红蓝，START 后执行。
 */
@Autonomous(name = "Test: GoToStopPose", group = "ActionTest")
public class TestGoToStopPose extends ActionTestBase {

    @Override
    protected String testName() {
        return "GoToStopPose";
    }

    @Override
    protected Action createTestAction() {
        return new GoToStopPose(drive(), selectedStopPose());
    }
}
