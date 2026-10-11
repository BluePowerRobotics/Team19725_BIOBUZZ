package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToNearestEatPointAction;

/**
 * 实机测试：{@link GoToNearestEatPointAction} —— 自动选择更近的取球点并前往。
 * Init：A/B 选起始位姿所属红蓝，START 后执行。
 */
@Autonomous(name = "Test: GoToNearestEatPoint", group = "ActionTest")
public class TestGoToNearestEatPoint extends ActionTestBase {

    @Override
    protected String testName() {
        return "GoToNearestEatPoint";
    }

    @Override
    protected Action createTestAction() {
        return new GoToNearestEatPointAction();
    }
}
