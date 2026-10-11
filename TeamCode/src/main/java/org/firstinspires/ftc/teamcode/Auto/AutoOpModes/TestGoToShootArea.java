package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToShootAreaAction;

/**
 * 实机测试：{@link GoToShootAreaAction} —— 到所选发射区域内最近点并预瞄球门。
 * Init：A/B 选红蓝，十字键上下选 FIRST/SECOND，X 选 AudienceUp/Down，START 后执行。
 */
@Autonomous(name = "Test: GoToShootArea", group = "ActionTest")
public class TestGoToShootArea extends ActionTestBase {

    @Override
    protected String testName() {
        return "GoToShootArea (#" + optionIndex + ", goal "
                + (goalDown ? "Down" : "Up") + ")";
    }

    @Override
    protected boolean hasIndexOption() {
        return true;
    }

    @Override
    protected boolean hasGoalOption() {
        return true;
    }

    @Override
    protected Action createTestAction() {
        return new GoToShootAreaAction(teamColor, optionIndex, selectedGoal());
    }
}
