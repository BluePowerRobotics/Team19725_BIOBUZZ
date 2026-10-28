package org.firstinspires.ftc.teamcode.utility.pastcalculater;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * 第 1/2 代（PIDSVA）飞轮整定：{@code d = kS + kV·ω* + kP·e + kI·∫e + kD·de/dt + kA·d(ω*)/dt}。
 *
 * <p>为什么整定方法与第 4 代不同：第 1/2 代**没有电压归一化、没有速度滤波、没有 boost**，
 * 前馈只有 {@code kS + kV·ω*} 两项，所以只标这两项 + 一组 kP/kI，kD/kA 保持 0
 * （历史调参中它们也一直是 0；要开就手改 {@link Gen1Params}）。
 *
 * <p>两个阶段（与 Tune6.5 / Tune6.1 同算法，便于和第 4 代的整定结果横向比）：
 * <ol>
 *   <li><b>kS / kV</b>：开环给若干占空比，量各自稳态转速，
 *       最小二乘拟合 {@code d_ss = kS + kV·ω*}（斜率 kV、截距 kS）。</li>
 *   <li><b>kP / kI</b>：开环占空比阶跃 {@code d0 → dStep}，辨识 {@code Kd、τ}，
 *       按 {@code kP = τ/(Kd·tc)}、{@code kI = 1/(Kd·tc)}（{@code tc = tRecover/ratio}）给建议值。</li>
 * </ol>
 *
 * <p>结果只打进遥测与 CSV（{@code t,duty,velocity,phase}），**不会**自动写回
 * {@link Gen1Params}——是否采纳由人判断（第 4 代整定也是这个规矩）。
 */
@TeleOp(name = "TuneGen1 PIDSVA 整定", group = "PastCompare")
public class TuneGen1Op extends LinearOpMode {

    // ------------------------------------------------------------ @Config

    /** 标定 kS/kV 用的开环占空比点（≥2 个；建议跨越工作区） */
    public static double[] pointDuties = {0.15, 0.25, 0.35, 0.45};
    /** 每个占空比点先等它稳定这么久 (s) */
    public static double pointSettleSec = 2.0;
    /** 稳定后取均值的窗口 (s) */
    public static double steadyWindowSec = 1.0;
    /** 每个占空比点最长等待 (s) */
    public static double pointTimeoutSec = 8.0;

    /** 阶跃前的占空比 */
    public static double d0 = 0.15;
    /** 阶跃后的占空比 */
    public static double dStep = 0.25;
    /** 阶跃记录时长 (s) */
    public static double stepRecordSec = 3.0;
    /** 期望恢复时间 (s)、tRecover/τc 比值 */
    public static double tRecover = 0.5;
    public static double ratio = 4.0;

    public static String motorName = "shooter";
    public static boolean motorReversed = true;
    /** 是否发布扁平数值遥测（由 FTC Dashboard 自带 Logging 记录） */
    public static boolean dashboardLog = true;
    public static double logPeriodMs = 10;

    /** 遥测键名（= Dashboard CSV 列名）；时间列由 Dashboard 自己加 */
    static final String CSV_HEADER = "duty,velocity,phase";

    // ------------------------------------------------------------ state

    private PastFlywheelDriver driver;
    private double lastLogSec = -1;

    private final double[] pointDutyUsed = new double[16];
    private final double[] pointOmega = new double[16];
    private int pointCount = 0;

    private double[] fit = null; // {slope=kV, intercept=kS}
    private double[] stepFit = null; // {omega0, omegaSS, Kd, tau, kff, kp, ki}

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("TuneGen1（第 1/2 代 PIDSVA）");
        telemetry.addLine(String.format(Locale.US, "点=%d 个  d0=%.2f dStep=%.2f",
                pointDuties.length, d0, dStep));
        telemetry.addLine("X 中止。结果只在遥测/CSV，不自动写回。");

        try {
            driver = new PastFlywheelDriver(hardwareMap, motorName, motorReversed,
                    new PastCalculator.Gen1());
        } catch (RuntimeException e) {
            telemetry.addLine("!! 找不到电机 " + motorName + "：" + e.getMessage());
            telemetry.update();
            sleep(3000);
            return;
        }

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        if (dashboardLog) {
            telemetry.addLine("Dash logging: ON（Dashboard Logging 记录 duty/velocity/phase）");
        }

        runPoints();
        if (opModeIsActive()) {
            runStep();
        }
        if (opModeIsActive()) {
            report();
        }

        driver.setOpenLoopPower(0);
    }

    // ------------------------------------------------------------ 阶段 1：kS / kV

    private void runPoints() {
        for (double duty : pointDuties) {
            if (!opModeIsActive() || gamepad1.x) {
                return;
            }
            double omega = measureOpenLoop(duty, pointSettleSec, pointTimeoutSec, "point");
            if (Double.isNaN(omega)) {
                continue;
            }
            if (pointCount < pointDutyUsed.length) {
                pointDutyUsed[pointCount] = duty;
                pointOmega[pointCount] = omega;
                pointCount++;
            }
            telemetry.addData("point d / omega", "%.3f / %.1f", duty, omega);
            telemetry.update();
        }

        if (pointCount >= 2) {
            double[] x = new double[pointCount];
            double[] y = new double[pointCount];
            for (int i = 0; i < pointCount; i++) {
                x[i] = pointOmega[i]; // ω
                y[i] = pointDutyUsed[i]; // d
            }
            fit = PastTuneSupport.leastSquares(x, y); // 斜率 = kV, 截距 = kS
        }
    }

    // ------------------------------------------------------------ 阶段 2：kP / kI

    private void runStep() {
        driver.resetController();
        driver.setOpenLoopPower(d0);
        sleepSec(1.0);

        driver.setOpenLoopPower(dStep);
        double startNano = System.nanoTime();
        int cap = 4000;
        double[] time = new double[cap];
        double[] velocity = new double[cap];
        int n = 0;

        while (opModeIsActive() && !gamepad1.x) {
            double t = (System.nanoTime() - startNano) / 1e9;
            if (t > stepRecordSec) {
                break;
            }
            double v = driver.getVelocity();
            if (n < cap) {
                time[n] = t;
                velocity[n] = v;
                n++;
            }
            logFrame(t, dStep, v, "step");
            telemetry.addData("step t / ω", "%.2f s / %.0f", t, v);
            telemetry.update();
            sleep(5);
        }

        stepFit = PastTuneSupport.fitStep(time, velocity, n, d0, dStep,
                steadyWindowSec, tRecover, ratio);
        driver.setOpenLoopPower(0);
    }

    // ------------------------------------------------------------ 报告

    private void report() {
        if (fit != null) {
            telemetry.addData(">>> 建议 kV (slope)", "%.8g", fit[0]);
            telemetry.addData(">>> 建议 kS (intercept)", "%.6g", fit[1]);
            telemetry.addData("fit_kV", fit[0]);
            telemetry.addData("fit_kS", fit[1]);
            telemetry.addData("fit_points", pointCount);
        } else {
            telemetry.addData("kS/kV", "拟合失败（点太少或转速没变化）");
        }

        if (stepFit != null) {
            telemetry.addData("step ω0 / ω∞", "%.0f / %.0f", stepFit[0], stepFit[1]);
            telemetry.addData("step Kd (tick/s/占空比)", "%.1f", stepFit[2]);
            telemetry.addData("step τ (s)", "%.3f", stepFit[3]);
            telemetry.addData(">>> 建议 kP", "%.8g", stepFit[5]);
            telemetry.addData(">>> 建议 kI", "%.8g", stepFit[6]);
            telemetry.addData("（kV 交叉核对 1/Kd）", "%.8g", stepFit[4]);
            telemetry.addData("step_kP", stepFit[5]);
            telemetry.addData("step_kI", stepFit[6]);
        } else {
            telemetry.addData("kP/kI", "阶跃辨识失败");
        }

        telemetry.addLine("把建议值手填进 Gen1Params（Dashboard 可改），再跑 PastCompare 验证。");
        telemetry.addData("dash log", dashboardLog ? "ON（Dashboard Logging）" : "OFF");
        telemetry.update();

        // 保持结果在屏幕上，按 X 或 STOP 退出
        while (opModeIsActive() && !gamepad1.x) {
            idle();
        }
    }

    // ------------------------------------------------------------ 工具

    /** 开环稳住后取稳态均值；超时返回 NaN。 */
    private double measureOpenLoop(double duty, double settleSec, double timeoutSec, String phase) {
        driver.setOpenLoopPower(duty);
        double start = getRuntime();
        double sum = 0;
        int n = 0;
        while (opModeIsActive() && !gamepad1.x) {
            double t = getRuntime();
            if (t - start > timeoutSec) {
                break;
            }
            double v = driver.getVelocity();
            if (t - start >= settleSec) {
                sum += v;
                n++;
            }
            logFrame(t, duty, v, phase);
            telemetry.addData("阶段", "%s d=%.3f t=%.1f/%.1f ω=%.0f",
                    phase, duty, t - start, timeoutSec, v);
            telemetry.update();
            sleep(5);
        }
        driver.setOpenLoopPower(0);
        return n > 0 ? sum / n : Double.NaN;
    }

    private void logFrame(double t, double duty, double velocity, String phase) {
        if (!dashboardLog) {
            return;
        }
        if (lastLogSec >= 0 && (t - lastLogSec) * 1000.0 < logPeriodMs) {
            return;
        }
        lastLogSec = t;
        telemetry.addData("duty", duty);
        telemetry.addData("velocity", velocity);
        telemetry.addData("phase", phaseNum(phase));
    }

    /** phase 文本 → 数字码（三份分代 Tuner 共用：1=point/settle 2=measure 3=step 4=dip 5=recover 6=dip_settle） */
    static int phaseNum(String phase) {
        switch (phase) {
            case "point":
            case "settle":
                return 1;
            case "measure":
                return 2;
            case "step":
                return 3;
            case "dip":
                return 4;
            case "recover":
                return 5;
            case "dip_settle":
                return 6;
            default:
                return 0;
        }
    }

    private void sleepSec(double sec) {
        long end = System.currentTimeMillis() + (long) (sec * 1000);
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            sleep(10);
        }
    }
}
