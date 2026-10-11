package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;

/**
 * 标定方法三（研究报告 6.3）：<b>继电反馈自整定</b>。
 *
 * <p>用继电器代替 PI 控制器：
 * <pre>
 *   d = dHigh  (ω* - ω > 0)
 *   d = dLow   (ω* - ω &lt; 0)
 * </pre>
 * 系统产生极限环振荡，测量振荡幅值 {@code a} 与周期 {@code Tu}，临界增益近似：
 * <pre>
 *   Ku = 4·Δd / (π·a),  Δd = dHigh - dLow
 *   Kp = 0.45·Ku     Ki = 0.54·Ku/Tu
 * </pre>
 *
 * <p>相比方法二更安全：振荡幅值由 dHigh/dLow 直接限定。
 *
 * <p><b>操作</b>：Init 后 Start 自动跑一次；A 重跑，X 停转。原始数据实时写入 CSV。
 *
 * <p><b>调参</b>（Dashboard → RelayFeedbackTuner）：targetVelocity 目标转速；dHigh/dLow 继电器
 * 输出上下限（应让稳态占空比落在两者之间）；transientSec 建立极限环的跳过时长；
 * measureSec 测量时长；hysteresis 继电器滞环 (tick/s)（默认 0，噪声大可略加；
 * 为 0 时 err=0 会保持上一拍输出，故并非严格理想继电器）。
 */
@Config
@TeleOp(name = "Tune6.3 RelayFeedback", group = "Tuning")
public class RelayFeedbackTuner extends LinearOpMode {

    // ==================== 可调参数 ====================

    /** 目标转速 (tick/s) */
    public static int targetVelocity = 3000;
    /** 继电器输出上限（占空比） */
    public static double dHigh = 0.7;
    /** 继电器输出下限（占空比） */
    public static double dLow = 0.5;
    /** 建立极限环的跳过时长 (s) */
    public static double transientSec = 2.0;
    /** 测量时长 (s) */
    public static double measureSec = 4.0;
    /** 继电器滞环 (tick/s)，默认 0 表示理想继电器 */
    public static double hysteresis = 0.0;
    /** 判定周期所需的最少过零次数 */
    public static int minZeroCrossings = 4;

    private static final int BUF = 2048;
    private final double[] bufV = new double[BUF];
    private final double[] bufT = new double[BUF];
    private int bufN = 0;
    private int bufNext = 0;

    private PrototypeShooter shooter;
    private final TuningLogger logger = new TuningLogger();
    private final ElapsedTime timer = new ElapsedTime();

    // 结果
    private double amp = 0, tu = Double.NaN, ku = 0, sugKp = 0, sugKi = 0;
    private boolean hasResult = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new PrototypeShooter(hardwareMap, telemetry);

        telemetry.addLine("Method 6.3  Relay feedback auto-tuning");
        telemetry.addLine("A: run test    X: stop motor");
        telemetry.addLine("Tune targetVelocity/dHigh/dLow/transientSec/measureSec in Dashboard");
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

    /** 执行一次继电反馈实验：跳过暂态 → 测量极限环 → 估算 Ku/Tu 与 PI。 */
    private void runTest() {
        hasResult = false;
        bufN = 0;
        bufNext = 0;
        logger.start(hardwareMap.appContext, "relay03");
        logger.log("# relay feedback, target=" + targetVelocity + ", dHigh=" + dHigh + ", dLow=" + dLow);

        double lastDuty = (dHigh + dLow) / 2.0;

        // ---- 阶段 1：建立极限环（只记录，不用于计算） ----
        timer.reset();
        while (opModeIsActive() && timer.seconds() < transientSec) {
            double v = shooter.motor.getVelocity();
            double err = targetVelocity - v;
            if (err > hysteresis) lastDuty = dHigh;
            else if (err < -hysteresis) lastDuty = dLow;
            shooter.motor.setPower(lastDuty);
            logger.logRow(timer.seconds(), lastDuty, v);
            idle();
        }

        // ---- 阶段 2：测量 ----
        timer.reset();
        while (opModeIsActive() && timer.seconds() < measureSec && bufN < BUF) {
            double t = timer.seconds();
            double v = shooter.motor.getVelocity();
            double err = targetVelocity - v;
            if (err > hysteresis) lastDuty = dHigh;
            else if (err < -hysteresis) lastDuty = dLow;
            shooter.motor.setPower(lastDuty);
            push(t, v);
            logger.logRow(t, lastDuty, v);
            idle();
        }
        shooter.motor.setPower(0);

        if (bufN < 10) {
            logger.log("# 采样不足，结果无效");
            logger.close();
            return;
        }

        amp = estimateAmplitude();
        tu = estimatePeriod();

        double deltaD = dHigh - dLow;
        if (amp > 1e-6 && !Double.isNaN(tu) && tu > 0) {
            ku = 4.0 * deltaD / (Math.PI * amp);
            sugKp = 0.45 * ku;
            sugKi = 0.54 * ku / tu;
            hasResult = true;
            logger.log(String.format(java.util.Locale.US,
                    "# a=%.2f Tu=%.4f Ku=%.8f Kp=%.8f Ki=%.8f", amp, tu, ku, sugKp, sugKi));
        }
        logger.close();
    }

    private void showResult() {
        telemetry.addData("target (tick/s)", targetVelocity);
        telemetry.addData("dHigh / dLow", "%.3f / %.3f", dHigh, dLow);
        telemetry.addLine("---- 极限环 ----");
        telemetry.addData("amplitude a (tick/s)", "%.1f", amp);
        telemetry.addData("Tu (s)", Double.isNaN(tu) ? "n/a" : String.format(java.util.Locale.US, "%.4f", tu));
        if (!hasResult) {
            telemetry.addLine("Result: (invalid — check dHigh/dLow, run again)");
            return;
        }
        telemetry.addLine("---- 建议参数 ----");
        telemetry.addData("Ku", "%.8f", ku);
        telemetry.addData("kp", "%.8f", sugKp);
        telemetry.addData("ki", "%.8f", sugKi);
    }

    // ==================== 采样缓冲与分析 ====================

    private void push(double t, double v) {
        bufV[bufNext] = v;
        bufT[bufNext] = t;
        bufNext = (bufNext + 1) % BUF;
        if (bufN < BUF) bufN++;
    }

    private double vAt(int i) {
        return bufV[(bufNext - bufN + i + 2 * BUF) % BUF];
    }

    private double tAt(int i) {
        return bufT[(bufNext - bufN + i + 2 * BUF) % BUF];
    }

    private double estimateAmplitude() {
        double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        for (int i = 0; i < bufN; i++) {
            double v = vAt(i);
            if (v < min) min = v;
            if (v > max) max = v;
        }
        return (max - min) / 2.0;
    }

    private double estimatePeriod() {
        double mean = 0;
        for (int i = 0; i < bufN; i++) mean += vAt(i);
        mean /= bufN;

        int crossings = 0;
        double firstT = Double.NaN, lastT = Double.NaN;
        for (int i = 1; i < bufN; i++) {
            double a = vAt(i - 1) - mean;
            double b = vAt(i) - mean;
            if ((a < 0 && b >= 0) || (a > 0 && b <= 0)) {
                if (Double.isNaN(firstT)) firstT = tAt(i);
                lastT = tAt(i);
                crossings++;
            }
        }
        if (crossings < minZeroCrossings || lastT <= firstT) return Double.NaN;
        int cycles = (crossings - 1) / 2;
        if (cycles < 1) return Double.NaN;
        return (lastT - firstT) / cycles;
    }
}
