package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;

/**
 * 标定方法四（研究报告 6.4）：<b>掉速检测与补偿参数的无模型标定</b>。
 *
 * <p>让飞轮在目标转速下闭环稳定运行（关闭掉速补偿），采集正常工况的速度波动：
 * <pre>
 *   Δω_th = kSigma · σ_ω        （σ_ω 为稳态速度样本标准差，÷ N-1）
 *   k_th  = kRate  · max(-dω/dt)（正常工况最大减速度，仅取速度下降的分量）
 *   kBoost = kBoostRatio · PrototypeShooter.kp
 *   boostDurationSec 建议 0.1~0.5 s（需发射实测微调）
 * </pre>
 *
 * <p><b>操作</b>：Init 后 Start 自动跑一次；A 重跑，X 停转。
 * 运行前请先把方法一/二/三整定好的 kff/kp/ki 填入 {@code PrototypeShooter}。
 * 本 OpMode 运行时会临时关闭掉速补偿，<b>退出时自动恢复原阈值</b>。
 *
 * <p><b>调参</b>（Dashboard → DropDetectionTuner）：targetVelocity 目标转速；
 * settleSec 测量前稳定时长；measureSec 测量时长；kSigma 3~5；kRate 2~3；kBoostRatio 2~5。
 */
@Config
@TeleOp(name = "Tune6.4 DropDetection", group = "Tuning")
public class DropDetectionTuner extends LinearOpMode {

    // ==================== 可调参数 ====================

    /** 目标转速 (tick/s) */
    public static int targetVelocity = 3000;
    /** 测量前稳定时长 (s) */
    public static double settleSec = 2.0;
    /** 测量时长 (s) */
    public static double measureSec = 5.0;
    /** 阈值倍数 k_σ，取 3~5 */
    public static double kSigma = 5.0;
    /** 减速度倍数 k_rate，取 2~3 */
    public static double kRate = 2.5;
    /** 补偿增益倍数，取 2~5 */
    public static double kBoostRatio = 2.0;
    /** 建议补偿持续时长 (s) */
    public static double suggestBoostDurationSec = 0.3;

    private static final int MAX_SAMPLES = 4000;
    private final double[] timeArr = new double[MAX_SAMPLES];
    private final double[] velArr = new double[MAX_SAMPLES];
    private int sampleCount = 0;

    private PrototypeShooter shooter;
    private final TuningLogger logger = new TuningLogger();
    private final ElapsedTime timer = new ElapsedTime();

    // 结果
    private double stdVel = 0, maxDecel = 0, dropThreshold = 0, dropRateThreshold = 0, kBoost = 0;
    private double meanVel = 0, meanDuty = 0;
    /** 速度量化台阶 (tick/s)：由实测速度的最大公约数求得 */
    private double quantStep = 0;
    private boolean hasResult = false;

    /** 求最大减速度前的速度 EMA 平滑系数 */
    private static final double EMA_ALPHA = 0.3;

    // 进入时保存被本 OpMode 强制关闭的掉速阈值，退出时恢复，避免污染其它 OpMode
    private double origDropThreshold, origDropRateThreshold;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new PrototypeShooter(hardwareMap, telemetry);
        origDropThreshold = PrototypeShooter.dropThreshold;
        origDropRateThreshold = PrototypeShooter.dropRateThreshold;

        telemetry.addLine("Method 6.4  Drop-detection & boost calibration");
        telemetry.addLine("A: run test    X: stop motor");
        telemetry.addLine("Tune targetVelocity/settleSec/measureSec/kSigma/kRate in Dashboard");
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
        PrototypeShooter.dropThreshold = origDropThreshold;
        PrototypeShooter.dropRateThreshold = origDropRateThreshold;
        logger.close();
    }

    /** 采样稳态速度波动并给出掉速检测 / 补偿参数建议。 */
    private void runTest() {
        hasResult = false;
        sampleCount = 0;
        logger.start(hardwareMap.appContext, "drop04");
        logger.log("# drop-detection calibration, target=" + targetVelocity
                + ", settleSec=" + settleSec + ", measureSec=" + measureSec
                + ", kSigma=" + kSigma + ", kRate=" + kRate + ", kBoostRatio=" + kBoostRatio);

        // ---- 稳定段：闭环运行，关闭掉速补偿以免污染统计 ----
        shooter.setTargetVelocity(targetVelocity);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < settleSec) {
            PrototypeShooter.dropThreshold = 1e9;
            PrototypeShooter.dropRateThreshold = 1e9;
            shooter.update();
            logger.logRow(timer.seconds(), shooter.getDuty(), shooter.getVelocity());
            idle();
        }

        // ---- 测量段 ----
        timer.reset();
        int i = 0;
        while (opModeIsActive() && timer.seconds() < measureSec && i < MAX_SAMPLES) {
            PrototypeShooter.dropThreshold = 1e9;
            PrototypeShooter.dropRateThreshold = 1e9;
            shooter.update();
            double t = timer.seconds();
            timeArr[i] = t;
            velArr[i] = shooter.getVelocity();
            i++;
            logger.logRow(t, shooter.getDuty(), shooter.getVelocity());
            idle();
        }
        sampleCount = i;
        shooter.stop();
        shooter.update();

        if (sampleCount < 10) {
            logger.log("# 采样不足，结果无效");
            logger.close();
            return;
        }

        // ---- 统计：均值、标准差、最大减速度 ----
        double sum = 0;
        for (int j = 0; j < sampleCount; j++) sum += velArr[j];
        meanVel = sum / sampleCount;

        double sq = 0;
        for (int j = 0; j < sampleCount; j++) {
            double d = velArr[j] - meanVel;
            sq += d * d;
        }
        // 样本标准差（÷ N-1），与报告 6.4 脚本口径一致
        stdVel = sampleCount > 1 ? Math.sqrt(sq / (sampleCount - 1)) : 0;

        // 速度量化台阶：任何稳态精度 / 阈值讨论都必须先报出台阶
        int g = 0;
        for (int j = 0; j < sampleCount; j++) {
            int v = (int) Math.round(Math.abs(velArr[j]));
            if (v != 0) g = gcd(g, v);
        }
        quantStep = g;

        // 只统计减速度（速度下降的分量），不与加速度混淆。
        // 先对速度做 EMA 平滑再差分：相邻原始样本的读数噪声会把最大减速度显著抬高。
        maxDecel = 0;
        double ema = velArr[0], prevEma = ema, prevT = timeArr[0];
        for (int j = 1; j < sampleCount; j++) {
            ema += EMA_ALPHA * (velArr[j] - ema);
            double dt = timeArr[j] - prevT;
            double dv = ema - prevEma;
            if (dt > 1e-4 && dv < 0) {
                double decel = -dv / dt;    // 减速度取正值
                if (decel > maxDecel) maxDecel = decel;
            }
            prevEma = ema;
            prevT = timeArr[j];
        }

        dropThreshold = kSigma * stdVel;
        dropRateThreshold = kRate * maxDecel;
        kBoost = kBoostRatio * PrototypeShooter.kp;
        hasResult = true;

        logger.log(String.format(java.util.Locale.US,
                "# meanVel=%.1f sigma=%.2f step=%.1f maxDecel=%.1f dropThreshold=%.2f dropRate=%.2f kBoost=%.8f Tboost=%.2f",
                meanVel, stdVel, quantStep, maxDecel, dropThreshold, dropRateThreshold, kBoost, suggestBoostDurationSec));
        logger.close();
    }

    /** 最大公约数：用于求速度反馈的量化台阶 */
    private static int gcd(int a, int b) {
        while (b != 0) {
            int t = a % b;
            a = b;
            b = t;
        }
        return a;
    }

    private void showResult() {
        telemetry.addData("target", targetVelocity);
        telemetry.addData("mean / sigma / step", "%.0f / %.2f / %.1f", meanVel, stdVel, quantStep);
        telemetry.addData("sigma (steps)", quantStep > 1e-9 ? stdVel / quantStep : 0);
        telemetry.addData("max decel (tick/s²)", "%.1f", maxDecel);
        if (!hasResult) {
            telemetry.addLine("Result: (invalid — run again)");
            return;
        }
        telemetry.addLine("---- 建议参数 (填入 PrototypeShooter) ----");
        telemetry.addData("dropThreshold", "%.2f  (%.2f steps)",
                dropThreshold, quantStep > 1e-9 ? dropThreshold / quantStep : 0);
        telemetry.addData("dropRateThreshold", "%.2f", dropRateThreshold);
        telemetry.addData("kBoost (2~5 kp)", "%.8f", kBoost);
        telemetry.addData("boostDurationSec", "%.2f", suggestBoostDurationSec);
    }
}
