package org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions.GoToUnvisitedEatPointViaHiveAction;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;

/**
 * 实机测试：{@link GoToUnvisitedEatPointViaHiveAction} —— 经 Hive 正下方前往此前未到达过的吸球点。
 * Init：A/B 选红蓝，START 后执行。
 */
@Autonomous(name = "Test: GoToUnvisitedEatPointViaHive", group = "ActionTest")
public class TestGoToUnvisitedEatPointViaHive extends ActionTestBase {

    @Override
    protected String testName() {
        return "GoToUnvisitedEatPointViaHive";
    }

    @Override
    protected Action createTestAction() {
        Sweeper sweeper = new Sweeper(hardwareMap, telemetry);
        return new GoToUnvisitedEatPointViaHiveAction(teamColor, sweeper);
    }
}
