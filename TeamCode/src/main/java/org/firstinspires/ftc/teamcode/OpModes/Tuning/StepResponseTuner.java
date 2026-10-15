package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;

/**
 * 标定方法一（研究报告 6.1）：<b>阶跃响应辨识一阶模型</b>。
 *
 * <p>把飞轮系统当黑箱，先以占空比 {@link #d0} 稳定运行，再阶跃到 {@link #dStep}，记录速度响应。
 * 一阶模型 {@code ω(t) = ω0 + Kd·Δd·(1 - e^(-t/τ))}，据此读：
 * <ul>
 *   <li>{@code Kd = (ω(∞) - ω0) / Δd}（(tick/s)/占空比）</li>
 *   <li>{@code τ = t63.2%}（速度到 63.2% 总变化量所需时间）</li>
 * </ul>
 * 再按零极点对消整定 PI，并给出前馈建议：
 * <pre>
 *   τc = tRecover / ratio
 *   kff = 1 / Kd              (占空比/(tick/s))
 *   kp  = τ / (Kd · τc)       (占空比/(tick/s))
 *   ki  = 1 / (Kd · τc)       (占空比/(tick/s·s))
 * </pre>
 *
 * <p><b>操作</b>：Init 后 Start 会自动跑一次；之后按 A 重跑，X 立即停转。
 * 结果同时显示在遥测，并把 (t, duty, velocity) 原始数据写入 CSV（见 {@link TuningLogger}）。
 *
 * <p><b>调参</b>（FTC Dashboard → Config → StepResponseTuner）：
 * d0 / dStep 阶跃前后占空比；settleSec 阶跃前稳定时长；recordSec 阶跃后记录时长；
 * steadyWindowSec 取稳态均值窗口；tRecover 期望恢复时间；ratio 取 3~5。
 */
@Config
@TeleOp(name = "Tune6.1 StepResponse", group = "Tuning")
public class StepResponseTuner extends LinearOpMode {

    // ==================== 可调参数 (FTC Dashboard 实时生效) ====================

    /** 阶跃前占空比（应能让飞轮稳定转起来） */
    public static double d0 = 0.15;
    /** 阶跃后占空比 */
    public static double dStep = 0.25;
    /** 阶跃前稳定时长 (s) */
    public static double settleSec = 2.0;
    /** 阶跃后记录时长 (s) */
    public static double recordSec = 3.0;
    /** 取稳态均值的窗口长度 (s) */
    public static double steadyWindowSec = 0.5;
    /** 期望恢复时间 (s) */
    public static double tRecover = 0.5;
    /** tRecover 与 τc 的比值，取 3~5 */
    public static double ratio = 4.0;

    /** 一次记录的最大采样数 */
    private static final int MAX_SAMPLES = 4000;

    private final double[] timeArr = new double[MAX_SAMPLES];
    private final double[] velArr = new double[MAX_SAMPLES];
    private int sampleCount = 0;

    private PrototypeShooter shooter;
    private final TuningLogger logger = new TuningLogger();
    private final ElapsedTime timer = new ElapsedTime();

    // 辨识结果
    private double omega0 = 0, omegaSS = 0, kd = 0, tau = 0, kff = 0, sugKp = 0, sugKi = 0;
    private boolean hasResult = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new PrototypeShooter(hardwareMap, telemetry);

        telemetry.addLine("Method 6.1  Step-response identification");
        telemetry.addLine("A: run test    X: stop motor");
        telemetry.addLine("Tune d0/dStep/settleSec/recordSec/tRecover in Dashboard");
        telemetry.update();

        waitForStart();

        runTest();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) runTest();
            if (gamepad1.xWasPressed()) shooter.motor.setPower(0);
            showResult();
            telemetry.update();
            idle();
        }

        shooter.motor.setPower(0);
        logger.close();
    }

    /** 执行一次「d0 稳定 → 阶跃到 dStep 记录」的完整辨识流程。 */
    private void runTest() {
        sampleCount = 0;
        hasResult = false;
        logger.start(hardwareMap.appContext, "step01");
        logger.log("# step response, d0=" + d0 + ", dStep=" + dStep
                + ", settleSec=" + settleSec + ", recordSec=" + recordSec
                + ", steadyWindowSec=" + steadyWindowSec
                + ", tRecover=" + tRecover + ", ratio=" + ratio);

        // ---- 阶段 1：维持在 d0，等待稳定 ----
        shooter.motor.setPower(d0);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < settleSec) {
            logger.logRow(timer.seconds(), d0, shooter.motor.getVelocity());
            idle();
        }

        // ---- 稳定段均值作为 ω0 ----
        double sum = 0;
        int n = 0;
        timer.reset();
        while (opModeIsActive() && timer.seconds() < steadyWindowSec) {
            sum += shooter.motor.getVelocity();
            n++;
            idle();
        }
        omega0 = n > 0 ? sum / n : 0;

        // ---- 阶段 2：阶跃到 dStep 并记录 ----
        shooter.motor.setPower(dStep);
        timer.reset();
        logger.log("# --- step applied here (t=0 of this segment) ---");
        int i = 0;
        while (opModeIsActive() && timer.seconds() < recordSec && i < MAX_SAMPLES) {
            double t = timer.seconds();
            double v = shooter.motor.getVelocity();
            timeArr[i] = t;
            velArr[i] = v;
            i++;
            logger.logRow(t, dStep, v);
            idle();
        }
        sampleCount = i;
        shooter.motor.setPower(0);

        if (sampleCount < 5) {
            logger.log("# 采样不足，结果无效");
            logger.close();
            return;
        }

        // ---- 稳态均值 ω(∞)：取记录末段 steadyWindowSec ----
        double tEnd = timeArr[sampleCount - 1];
        sum = 0;
        n = 0;
        for (int j = sampleCount - 1; j >= 0; j--) {
            if (timeArr[j] < tEnd - steadyWindowSec) break;
            sum += velArr[j];
            n++;
        }
        omegaSS = n > 0 ? sum / n : 0;

        // ---- 辨识 Kd 与 τ ----
        double deltaD = dStep - d0;
        double deltaW = omegaSS - omega0;
        if (Math.abs(deltaD) > 1e-6 && Math.abs(deltaW) > 1e-6) {
            kd = deltaW / deltaD; // (tick/s)/占空比

            double targetW = omega0 + 0.632 * deltaW; // t63.2% 判据
            tau = Double.NaN;
            for (int j = 0; j < sampleCount; j++) {
                if (velArr[j] >= targetW) {
                    tau = timeArr[j];
                    break;
                }
            }

            if (!Double.isNaN(tau) && kd > 0) {
                kff = 1.0 / kd;
                double tc = tRecover / ratio;
                sugKp = tau / (kd * tc);
                sugKi = 1.0 / (kd * tc);
                hasResult = true;
                logger.log(String.format(java.util.Locale.US,
                        "# omega0=%.2f omegaSS=%.2f Kd=%.6f tau=%.4f Kff=%.8f Kp=%.8f Ki=%.8f",
                        omega0, omegaSS, kd, tau, kff, sugKp, sugKi));
            }
        }
        logger.close();
    }

    private void showResult() {
        telemetry.addData("d0 / dStep", "%.3f / %.3f", d0, dStep);
        telemetry.addData("omega0 / omegaSS", "%.0f / %.0f", omega0, omegaSS);
        if (!hasResult) {
            telemetry.addLine("Result: (invalid — check d0/dStep, run again)");
            return;
        }
        telemetry.addData("Kd / tau", "%.4f / %.4f", kd, tau);
        telemetry.addData("建议 kff / kp / ki", "%.8f / %.8f / %.8f", kff, sugKp, sugKi);
        telemetry.addData("CSV", logger.getPath());
    }
}
