package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotStateStore;

/**
 * 机器状态持久化 —— <b>读取端</b>测试。
 *
 * <p>读取由 {@link StateStoreTest}（或自动阶段结束时）写入的状态文件并显示，
 * 用于验证状态确实落盘、且跨 OpMode / 跨断电仍然可读。
 *
 * <p><b>不依赖任何硬件</b>：不初始化底盘、定位与 Limelight，因此即使机器人
 * 未接硬件也可运行，便于单独验证文件读写路径。
 *
 * <p><b>操作</b>：A 键 = 重新读取一次（可在不重启 OpMode 的情况下，配合
 * 重新运行 StateStoreTest 后再次查看）。
 */
@Config
@TeleOp(name = "StateReadTest", group = "Tests")
public class StateReadTest extends LinearOpMode {

    /** 要读取的自定义字段键名（与 StateStoreTest.testKey 对应） */
    public static String testKey = "turret.heading.rad";

    // ---- 最近的读取结果（循环内重复显示） ----
    private Pose2d storedPose;
    private String storedKeyValue;
    private String status = "";

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        read();

        telemetry.addData("Status", status);
        telemetry.addLine("START to keep displaying. A: re-read.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                read();
            }

            telemetry.addData("Status", status);
            telemetry.addData("Stored pose X (in)", storedPose == null ? "-" : String.format("%.4f", storedPose.position.x));
            telemetry.addData("Stored pose Y (in)", storedPose == null ? "-" : String.format("%.4f", storedPose.position.y));
            telemetry.addData("Stored pose heading", storedPose == null ? "-"
                    : String.format("%.2f deg", Math.toDegrees(storedPose.heading.toDouble())));
            telemetry.addData("Stored " + testKey, String.valueOf(storedKeyValue));
            telemetry.update();
            idle();
        }
    }

    private void read() {
        storedPose = RobotStateStore.loadPose();
        storedKeyValue = RobotStateStore.get(testKey);
        if (storedPose == null) {
            status = "PASS (empty) - no stored pose; state file absent or cleared";
        } else {
            status = String.format("PASS - loaded pose: %s",
                    String.format("%.3f, %.3f, %.2f deg",
                            storedPose.position.x, storedPose.position.y,
                            Math.toDegrees(storedPose.heading.toDouble())));
        }
    }
}
