package org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions.GoToNearestEatPointAction;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;

/**
 * 实机测试：{@link GoToNearestEatPointAction} —— 自动选择更近的取球点并前往。
 * Init：A/B 选起始位姿所属红蓝，START 后执行。
 */
@Autonomous(name = "ConservativeTest: GoToNearestEatPoint", group = "ActionTest")
public class TestGoToNearestEatPoint extends ActionTestBase {

    @Override
    protected String testName() {
        return "GoToNearestEatPoint";
    }

    @Override
    protected Action createTestAction() {
        Sweeper sweeper = new Sweeper(hardwareMap, telemetry);
        return new GoToNearestEatPointAction(sweeper);
    }
}
