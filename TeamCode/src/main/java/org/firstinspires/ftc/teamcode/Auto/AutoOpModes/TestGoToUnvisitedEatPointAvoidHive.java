package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToUnvisitedEatPointAvoidHiveAction;

/**
 * 实机测试：{@link GoToUnvisitedEatPointAvoidHiveAction} —— 绕开 Hive 前往此前未到达过的吸球点。
 * Init：A/B 选红蓝，START 后执行。
 */
@Autonomous(name = "Test: GoToUnvisitedEatPointAvoidHive", group = "ActionTest")
public class TestGoToUnvisitedEatPointAvoidHive extends ActionTestBase {

    @Override
    protected String testName() {
        return "GoToUnvisitedEatPointAvoidHive";
    }

    @Override
    protected Action createTestAction() {
        return new GoToUnvisitedEatPointAvoidHiveAction(teamColor);
    }
}
