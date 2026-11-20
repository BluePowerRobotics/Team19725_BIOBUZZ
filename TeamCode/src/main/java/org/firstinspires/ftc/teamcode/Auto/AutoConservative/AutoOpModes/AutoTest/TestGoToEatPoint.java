package org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions.GoToEatPointAction;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;

/**
 * 实机测试：{@link GoToEatPointAction} —— 到指定取球点（intake 到位、车头朝取球点）。
 * Init：十字键上下选 FIRST/SECOND 取球点（A/B 仅决定起始位姿），START 后执行。
 */
@Autonomous(name = "ConservativeTest: GoToEatPoint", group = "ActionTest")
public class TestGoToEatPoint extends ActionTestBase {

    @Override
    protected String testName() {
        return "GoToEatPoint (#" + optionIndex + ")";
    }

    @Override
    protected boolean hasIndexOption() {
        return true;
    }

    @Override
    protected Action createTestAction() {
        Sweeper sweeper = new Sweeper(hardwareMap, telemetry);
        return new GoToEatPointAction(optionIndex, sweeper);
    }
}
