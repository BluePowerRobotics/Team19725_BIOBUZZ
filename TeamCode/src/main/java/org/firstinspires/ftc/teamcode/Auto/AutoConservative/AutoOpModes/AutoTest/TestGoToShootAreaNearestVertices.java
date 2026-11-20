package org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions.GoToShootAreaNearestVerticesAction;

/**
 * 实机测试：{@link GoToShootAreaNearestVerticesAction} ——
 * 先到最近发射区域距原点最近的顶点，再到另一区域的对应顶点。
 * Init：A/B 选红蓝，X 选 AudienceUp/Down，START 后执行。
 */
@Autonomous(name = "ConservativeTest: GoToShootAreaVertices", group = "ActionTest")
public class TestGoToShootAreaNearestVertices extends ActionTestBase {

    @Override
    protected String testName() {
        return "GoToShootAreaNearestVertices (goal " + (goalDown ? "Down" : "Up") + ")";
    }

    @Override
    protected boolean hasGoalOption() {
        return true;
    }

    @Override
    protected Action createTestAction() {
        return new GoToShootAreaNearestVerticesAction(teamColor, selectedGoal());
    }
}
