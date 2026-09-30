package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.EatAction;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;

/**
 * 实机测试：{@link EatAction} —— 集球器以吃球速度运转 {@code EatAction.eatDurationMs}
 * 毫秒（Dashboard 可调）后自动停转。
 * Init：A/B 仅决定底盘初始化参数，START 后执行（无底盘位移）。
 */
@Autonomous(name = "Test: Eat", group = "ActionTest")
public class TestEat extends ActionTestBase {

    @Override
    protected String testName() {
        return "Eat";
    }

    @Override
    protected Action createTestAction() {
        Sweeper sweeper = new Sweeper(hardwareMap, telemetry);
        return new EatAction(sweeper);
    }
}
