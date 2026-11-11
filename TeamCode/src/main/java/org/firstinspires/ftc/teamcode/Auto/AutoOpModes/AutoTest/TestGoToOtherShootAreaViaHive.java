package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToOtherShootAreaViaHiveAction;

/**
 * 实机测试：{@link GoToOtherShootAreaViaHiveAction} —— 经 Hive 正下方前往另一个发射区域并预瞄球门。
 * Init：A/B 选红蓝，X 选 AudienceUp/Down，START 后执行。
 */
@Autonomous(name = "Test: GoToOtherShootAreaViaHive", group = "ActionTest")
public class TestGoToOtherShootAreaViaHive extends ActionTestBase {

    @Override
    protected String testName() {
        return "GoToOtherShootAreaViaHive (goal " + (goalDown ? "Down" : "Up") + ")";
    }

    @Override
    protected boolean hasGoalOption() {
        return true;
    }

    @Override
    protected Action createTestAction() {
        return new GoToOtherShootAreaViaHiveAction(teamColor, selectedGoal());
    }
}
