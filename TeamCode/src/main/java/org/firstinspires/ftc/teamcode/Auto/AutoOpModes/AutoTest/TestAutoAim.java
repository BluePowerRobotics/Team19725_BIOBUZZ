package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.AutoAimAction;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;

/**
 * 实机测试：{@link AutoAimAction} —— 底盘不动，炮台自动瞄准并发射。
 * Init：A/B 选红蓝（决定球门颜色与视觉 pipeline），START 后执行。
 * 测试前机器人应放在发射位置附近，使 Limelight 能看到目标侧。
 */
@Autonomous(name = "Test: AutoAim", group = "ActionTest")
public class TestAutoAim extends ActionTestBase {

    @Override
    protected String testName() {
        return "AutoAim";
    }

    @Override
    protected Action createTestAction() {
        Turret turret = new Turret(hardwareMap, telemetry);
        return new AutoAimAction(RobotPosition.getInstance().getDrive(), turret);
    }
}
