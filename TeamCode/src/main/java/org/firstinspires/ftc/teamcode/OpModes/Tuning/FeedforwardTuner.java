package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 标定方法五（研究报告 6.5）：<b>无模型前馈增益标定</b>。
 *
 * <p>让飞轮在目标转速 {@code ω*} 下闭环稳定运行（本 OpMode 强制 kff=0，由 PI 提供全部占空比），
 * 记录稳态占空比 {@code d_ss}，则
 * <pre>
 *   Kff = d_ss / ω*      （占空比/(tick/s)）
 * </pre>
 * 可在多个目标转速下各测一点，用最小二乘拟合出 Kff（过原点斜率），提高泛化性。
 *
 * <p><b>操作</b>
 * <ul>
 *   <li>DPad 上 / 下 → 按 {@link #velocityStep} 增减目标转速</li>
 *   <li>A → 在最近设定速度下测量一点（自动等待 {@link #settleSec} 稳定后取 {@link #windowSec} 均值）</li>
 *   <li>B → 清空已记录的测量点</li>
 * </ul>
 *
 * <p><b>前置条件</b>：请先完成 PI 整定（kp/ki 非 0），否则系统无法稳定到目标转速。
 *
 * <p><b>调参</b>（Dashboard → FeedforwardTuner）：velocityStep 目标转速步长；
 * settleSec 测量前稳定时长；windowSec 取均值窗口。
 */
@Config
@TeleOp(name = "Tune6.5 Feedforward", group = "Tuning")
public class FeedforwardTuner extends LinearOpMode {

    // ==================== 可调参数 ====================

    /** 目标转速 (tick/s) */
    public static int targetVelocity = 3000;
    /** 目标转速单次调节步长 (tick/s) */
    public static int velocityStep = 200;
    /** 按住方向键时的连续调节周期 (ms) */
    public static long repeatPeriodMs = 150;
    /** 测量前稳定时长 (s) */
    public static double settleSec = 2.0;
    /** 取均值窗口 (s) */
    public static double windowSec = 1.5;

    private PrototypeShooter shooter;
    private final TuningLogger logger = new TuningLogger();
    private final ElapsedTime timer = new ElapsedTime();
    /** 测量专用计时器：不干扰主循环的日志时间轴 */
    private final ElapsedTime measureTimer = new ElapsedTime();

    /** 上一次方向键状态：-1 下 / 0 未按 / 1 上 */
    private int lastDir = 0;
    private final ElapsedTime repeatTimer = new ElapsedTime();

    /** 已记录的测量点：(目标转速, 稳态占空比) */
    private final List<double[]> points = new ArrayList<>();

    private double lastOmega = 0, lastDss = 0, lastKff = 0;
    private boolean hasLast = false;
    /** 最近一点是否已饱和（d_ss ≥ 0.95），饱和点不可用于定 Kff */
    private boolean lastSaturated = false;

    /** 进入时保存被本 OpMode 强制改写的 kff，退出时恢复 */
    private double origKff;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new PrototypeShooter(hardwareMap, telemetry);
        origKff = PrototypeShooter.kff;

        telemetry.addLine("Method 6.5  Feedforward gain calibration");
        telemetry.addLine("Dpad U/D: target    A: measure point    B: clear");
        telemetry.addLine("Requires kp/ki already tuned. kff is forced to 0 here.");
        telemetry.update();

        waitForStart();
        logger.start(hardwareMap.appContext, "ff05");
        logger.log("# feedforward calibration (kff forced 0), settleSec=" + settleSec
                + ", windowSec=" + windowSec + ", velocityStep=" + velocityStep);

        while (opModeIsActive()) {
            // 目标转速调节（按住则按 repeatPeriodMs 连续调节）
            int dir = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
            if (dir != lastDir) {
                if (dir != 0) targetVelocity += dir * velocityStep;
                repeatTimer.reset();
            } else if (dir != 0 && repeatTimer.milliseconds() >= repeatPeriodMs) {
                targetVelocity += dir * velocityStep;
                repeatTimer.reset();
            }
            lastDir = dir;
            if (targetVelocity < 0) targetVelocity = 0;

            // 强制 kff=0：让 PI 提供全部稳态占空比
            PrototypeShooter.kff = 0.0;

            shooter.setTargetVelocity(targetVelocity);
            shooter.update();
            logger.logRow(timer.seconds(), shooter.getDuty(), shooter.getVelocity());

            if (gamepad1.aWasPressed()) measurePoint();
            if (gamepad1.bWasPressed()) {
                points.clear();
                hasLast = false;
            }

            showTelemetry();
            telemetry.update();
            idle();
        }

        shooter.motor.setPower(0);
        PrototypeShooter.kff = origKff;
        logger.close();
    }

    /** 在当前目标转速下等待稳定，取稳态占空比作为一点。 */
    private void measurePoint() {
        // 稳定阶段（用独立的计时器，避免重置主循环的日志时间轴 → 否则 CSV 时间列会非单调）
        measureTimer.reset();
        while (opModeIsActive() && measureTimer.seconds() < settleSec) {
            PrototypeShooter.kff = 0.0;
            shooter.setTargetVelocity(targetVelocity);
            shooter.update();
            telemetry.addData("Status", "settling... %.1f", measureTimer.seconds());
            telemetry.update();
            idle();
        }

        // 取均值
        double sumDuty = 0, sumVel = 0;
        int n = 0;
        measureTimer.reset();
        while (opModeIsActive() && measureTimer.seconds() < windowSec) {
            PrototypeShooter.kff = 0.0;
            shooter.setTargetVelocity(targetVelocity);
            shooter.update();
            sumDuty += shooter.getDuty();
            sumVel += shooter.getVelocity();
            n++;
            idle();
        }

        if (n == 0) return;
        lastOmega = sumVel / n;
        lastDss = sumDuty / n;
        lastKff = lastOmega > 1e-6 ? lastDss / lastOmega : 0;
        // 占空比接近 1 说明已饱和，该点的 d_ss 不可信（Kff 会偏小）
        lastSaturated = lastDss >= 0.95;
        hasLast = true;

        points.add(new double[]{lastOmega, lastDss});
        logger.log(String.format(Locale.US, "# point omega=%.1f dss=%.4f kff=%.8f%s",
                lastOmega, lastDss, lastKff, lastSaturated ? " SATURATED(invalid)" : ""));
    }

    // 遥测在循环里直接输出，这里提供统一展示（每帧调用）；只保留判读必需的行
    private void showTelemetry() {
        telemetry.addData("target", targetVelocity);
        telemetry.addData("velocity / duty", "%.0f / %.4f", shooter.getVelocity(), shooter.getDuty());
        if (hasLast) {
            telemetry.addData("last point", "n=%d w=%.0f d_ss=%.4f Kff=%.8f%s",
                    points.size(), lastOmega, lastDss, lastKff, lastSaturated ? " [SATURATED]" : "");
        } else {
            telemetry.addData("points", points.size());
        }

        if (points.size() >= 1) {
            double sOO = 0, sOD = 0, sO = 0, sD = 0;
            int n = points.size();
            for (double[] p : points) {
                sOO += p[0] * p[0];
                sOD += p[0] * p[1];
                sO += p[0];
                sD += p[1];
            }
            telemetry.addData("Kff (through origin)", "%.8f", sOO > 1e-9 ? sOD / sOO : 0);
            if (n >= 2) {
                double denom = n * sOO - sO * sO;
                if (Math.abs(denom) > 1e-9) {
                    double slope = (n * sOD - sO * sD) / denom;
                    double intercept = (sD - slope * sO) / n;
                    telemetry.addData("Kff (fit) / intercept", "%.8f / %.4f", slope, intercept);
                }
            }
        }
    }
}
