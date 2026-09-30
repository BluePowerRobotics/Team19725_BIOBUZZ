package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotStateStore;

/**
 * 机器状态持久化 —— <b>写入端</b>测试。
 *
 * <p>在 Dashboard 上指定一个位姿，运行本 OpMode 后会被写入状态文件；
 * 随后运行 {@link StateReadTest} 即可读回，用于验证"断电后仍然有效"。
 *
 * <p><b>流程</b>：init 阶段先显示文件中原有内容 → 按 START 后执行
 * {@code clear()} 清空 → {@code savePose()} 写入指定位姿 → {@code put()} 写入一个
 * 自定义字段 → 立刻 {@code loadPose()/get()} 回读并与期望值逐项比对，给出 PASS/FAIL。
 *
 * <p><b>操作</b>：A 键 = 用当前 Dashboard 参数重新执行一遍写入与校验
 * （改完 {@code testX/testY/testHeadingDeg} 无需重启 OpMode）。
 */
@Config
@TeleOp(name = "StateStoreTest", group = "Tests")
public class StateStoreTest extends LinearOpMode {

    // ---- 待写入的指定位姿（Dashboard 可改） ----
    public static double testX = 12.5;
    public static double testY = -34.0;
    public static double testHeadingDeg = 45.0;

    // ---- 自定义字段（验证 put/get 扩展机制） ----
    public static String testKey = "turret.heading.rad";
    public static String testValue = "1.57";

    /** 位姿比对容差（状态文件以十进制文本保存，回读会有极小浮点误差） */
    private static final double EPS = 1e-9;

    // ---- 最近一次写入/校验的结果（循环内重复显示，避免 Driver Station 刷屏） ----
    private Pose2d storedPose;
    private String storedKeyValue;
    private String result = "";

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- init：显示文件中已有的内容（可能是上一局 / 上一次测试留下的） ----
        Pose2d existing = RobotStateStore.loadPose();
        telemetry.addData("Existing stored pose", formatPose(existing));
        telemetry.addData("Existing " + testKey, String.valueOf(RobotStateStore.get(testKey)));
        telemetry.addLine("---");
        telemetry.addData("Target pose (to write)", formatPose(targetPose()));
        telemetry.addData("Target " + testKey, testValue);
        telemetry.addLine("START: clear + write + verify.  A: write again.");
        telemetry.update();

        waitForStart();

        // 开始即执行一次完整的 清空 → 写入 → 回读校验
        writeAndVerify();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                writeAndVerify();
            }

            telemetry.addData("Target pose", formatPose(targetPose()));
            telemetry.addData("Read-back pose", formatPose(storedPose));
            telemetry.addData("Read-back " + testKey, String.valueOf(storedKeyValue));
            telemetry.addData("RESULT", result);
            telemetry.addLine("Next: run StateReadTest to read this back");
            telemetry.update();
            idle();
        }
    }

    /** 清空文件后用 Dashboard 参数写入位姿与自定义字段，再回读比对 */
    private void writeAndVerify() {
        // 1. 清空上一局遗留状态
        RobotStateStore.clear();
        // 2. 写入指定位姿与一个自定义字段
        RobotStateStore.savePose(targetPose());
        RobotStateStore.put(testKey, testValue);
        // 3. 回读
        storedPose = RobotStateStore.loadPose();
        storedKeyValue = RobotStateStore.get(testKey);
        // 4. 比对
        result = verify() ? "PASS - written & verified" : "FAIL - see values above";
    }

    private boolean verify() {
        Pose2d target = targetPose();
        return storedPose != null
                && Math.abs(storedPose.position.x - target.position.x) < EPS
                && Math.abs(storedPose.position.y - target.position.y) < EPS
                && Math.abs(storedPose.heading.toDouble() - target.heading.toDouble()) < EPS
                && testValue.equals(storedKeyValue);
    }

    private Pose2d targetPose() {
        return new Pose2d(testX, testY, Math.toRadians(testHeadingDeg));
    }

    private static String formatPose(Pose2d pose) {
        if (pose == null) {
            return "none";
        }
        return String.format("%.3f, %.3f, %.2f deg",
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
    }
}
