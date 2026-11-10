package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToOtherShootAreaAvoidHiveAction;

/**
 * 实机测试：{@link GoToOtherShootAreaAvoidHiveAction} —— 绕开 Hive 前往另一个发射区域并预瞄球门。
 * Init：A/B 选红蓝，X 选 AudienceUp/Down，START 后执行。
 */
@Autonomous(name = "Test: GoToOtherShootAreaAvoidHive", group = "ActionTest")
public class TestGoToOtherShootAreaAvoidHive extends ActionTestBase {

    @Override
    protected String testName() {
        return "GoToOtherShootAreaAvoidHive (goal " + (goalDown ? "Down" : "Up") + ")";
    }

    @Override
    protected boolean hasGoalOption() {
        return true;
    }

    @Override
    protected Action createTestAction() {
        return new GoToOtherShootAreaAvoidHiveAction(teamColor, selectedGoal());
    }
}
