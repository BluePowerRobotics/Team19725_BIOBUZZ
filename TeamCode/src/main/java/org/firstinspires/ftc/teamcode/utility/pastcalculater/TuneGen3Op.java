package org.firstinspires.ftc.teamcode.utility.pastcalculater;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * 第 3 代（含 boost）飞轮整定。
 *
 * <p>第 3 代的控制律（逐行等价 0969cbd^ 版本）：
 * <pre>
 *   d = kS·sgn(ω*) + kff·ω* + kp·e + ki·∫e + boost
 *   boost = clamp(kBoost·e, 0, boostMax)，掉速触发后持续 boostDurationSec
 *   |e| &lt; iZone 才累加积分，否则清零；掉速触发时也清零
 * </pre>
 *
 * <p>三个阶段：
 * <ol>
 *   <li><b>kS / kff</b>：开环若干占空比 → 稳态转速 → 最小二乘（同 TuneGen1 阶段 1）。</li>
 *   <li><b>kp / ki</b>：开环阶跃辨识 → {@code kp = τ/(Kd·tc)}、{@code ki = 1/(Kd·tc)}。</li>
 *   <li><b>kBoost</b>：闭环跑到目标后，用 {@code dipDuty} 人为把占空比压 {@code dipSec} 秒
 *       （模拟发射掉速），量「峰值误差 / 峰值 boost / 恢复时间 / 过冲」。
 *       恢复时间比目标慢就按比例加大 kBoost（遥测给建议值，不自动写回）。</li>
 * </ol>
 */
@TeleOp(name = "TuneGen3 Boost 整定", group = "PastCompare")
public class TuneGen3Op extends LinearOpMode {

    // ------------------------------------------------------------ @Config

    public static double[] pointDuties = {0.15, 0.25, 0.35, 0.45};
    public static double pointSettleSec = 2.0;
    public static double steadyWindowSec = 1.0;
    public static double pointTimeoutSec = 8.0;

    public static double d0 = 0.15;
    public static double dStep = 0.25;
    public static double stepRecordSec = 3.0;
    public static double tRecover = 0.5;
    public static double ratio = 4.0;

    /** boost 阶段：闭环目标转速 */
    public static double boostTarget = 1900;
    /** 闭环稳定时间 (s) */
    public static double boostSettleSec = 2.0;
    /** 人为掉速：压到的开环占空比 */
    public static double dipDuty = 0.15;
    /** 掉速持续 (s) */
    public static double dipSec = 0.35;
    /** 掉速后观察多久 (s) */
    public static double recoverySec = 2.0;
    /** 期望恢复时间 (s)，用于给 kBoost 建议 */
    public static double targetRecoverySec = 0.43;

    public static String motorName = "shooter";
    public static boolean motorReversed = true;
    public static boolean logToCsv = true;
    public static double logPeriodMs = 10;

    static final String CSV_HEADER = "t,duty,velocity,error,boost,phase";

    // ------------------------------------------------------------ state

    private final PastCompareOp.TuningLoggerForPast logger = new PastCompareOp.TuningLoggerForPast();
    private PastFlywheelDriver driver;
    private PastCalculator.Gen3 calculator;

    private final double[] pointDutyUsed = new double[16];
    private final double[] pointOmega = new double[16];
    private int pointCount = 0;

    private double[] fit = null;      // {kV(=kff 方向), kS}
    private double[] stepFit = null;  // {omega0, omegaSS, Kd, tau, kff, kp, ki}

    private double dipPeakError = 0;
    private double dipPeakBoost = 0;
    private double dipRecovery = Double.NaN;
    private double dipOvershoot = 0;

    @Override
    public void runOpMode() {
        telemetry.addLine("TuneGen3（含 boost 的第 3 代）");
        telemetry.addLine(String.format(Locale.US, "点=%d 个  d0=%.2f dStep=%.2f  boost 目标=%.0f",
                pointDuties.length, d0, dStep, boostTarget));
        telemetry.addLine("X 中止。结果只在遥测/CSV，不自动写回。");

        calculator = new PastCalculator.Gen3();
        try {
            driver = new PastFlywheelDriver(hardwareMap, motorName, motorReversed, calculator);
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

        if (logToCsv) {
            logger.start(hardwareMap.appContext, "TuneGen3", CSV_HEADER);
            logger.log("# Gen3 boost: points=" + pointDuties.length
                    + " d0=" + d0 + " dStep=" + dStep + " boostTarget=" + (int) boostTarget);
        }

        runPoints();
        if (opModeIsActive() && !gamepad1.x) {
            runStep();
        }
        if (opModeIsActive() && !gamepad1.x) {
            runBoost();
        }
        if (opModeIsActive()) {
            report();
        }

        driver.setOpenLoopPower(0);
        if (logger.isOpen()) {
            logger.close();
        }
    }

    // ------------------------------------------------------------ 阶段 1

    private void runPoints() {
        for (double duty : pointDuties) {
            if (!opModeIsActive() || gamepad1.x) {
                return;
            }
            double omega = measureOpenLoop(duty, pointSettleSec, pointTimeoutSec);
            if (Double.isNaN(omega)) {
                continue;
            }
            if (pointCount < pointDutyUsed.length) {
                pointDutyUsed[pointCount] = duty;
                pointOmega[pointCount] = omega;
                pointCount++;
            }
            if (logger.isOpen()) {
                logger.log(String.format(Locale.US, "# point d=%.3f omega=%.1f", duty, omega));
            }
        }
        if (pointCount >= 2) {
            double[] x = new double[pointCount];
            double[] y = new double[pointCount];
            for (int i = 0; i < pointCount; i++) {
                x[i] = pointOmega[i];
                y[i] = pointDutyUsed[i];
            }
            fit = PastTuneSupport.leastSquares(x, y); // 斜率 = kff, 截距 = kS
        }
    }

    // ------------------------------------------------------------ 阶段 2

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
        double lastLog = -1;

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
            logFrame(t, dStep, v, 0, 0, "step", lastLog);
            lastLog = t;
            sleep(5);
        }

        stepFit = PastTuneSupport.fitStep(time, velocity, n, d0, dStep,
                steadyWindowSec, tRecover, ratio);
        driver.setOpenLoopPower(0);
    }

    // ------------------------------------------------------------ 阶段 3：boost

    private void runBoost() {
        // 闭环跑到目标
        driver.resetController();
        double lastLog = -1;
        double t0 = getRuntime();
        while (opModeIsActive() && !gamepad1.x && getRuntime() - t0 < boostSettleSec) {
            driver.update((int) boostTarget);
            double t = getRuntime() - t0;
            logFrame(t, calculator.getDuty(), driver.getVelocity(),
                    calculator.getError(), calculator.getBoostTerm(), "boost_settle", lastLog);
            lastLog = t;
            telemetry.addData("boost 阶段", "稳定中 t=%.2f/%.2f", t, boostSettleSec);
            telemetry.update();
            sleep(5);
        }
        if (!opModeIsActive() || gamepad1.x) {
            return;
        }

        // 人为掉速：开环压占空比（不调用 update，否则被闭环覆写）
        driver.setOpenLoopPower(dipDuty);
        double dipStart = getRuntime();
        dipPeakError = 0;
        dipPeakBoost = 0;
        while (opModeIsActive() && !gamepad1.x && getRuntime() - dipStart < dipSec) {
            double t = getRuntime() - t0;
            double v = driver.getVelocity();
            logFrame(t, dipDuty, v, 0, 0, "dip", lastLog);
            lastLog = t;
            telemetry.addData("boost 阶段", "掉速中 t=%.2f", t);
            telemetry.update();
            sleep(5);
        }

        // 回到闭环：观察 boost 与恢复
        double recoverStart = getRuntime();
        boolean recovered = false;
        boolean postRecovery = false;
        while (opModeIsActive() && !gamepad1.x && getRuntime() - recoverStart < recoverySec) {
            driver.update((int) boostTarget);
            double t = getRuntime() - t0;
            double e = calculator.getError();
            double boost = calculator.getBoostTerm();
            dipPeakError = Math.max(dipPeakError, e);
            dipPeakBoost = Math.max(dipPeakBoost, boost);

            if (!recovered && Math.abs(e) <= calculator.getErrorBand()) {
                recovered = true;
                dipRecovery = getRuntime() - recoverStart;
            }
            if (recovered) {
                postRecovery = true;
            }
            if (postRecovery) {
                // 过冲：恢复后超过目标的最高转速
                dipOvershoot = Math.max(dipOvershoot, driver.getVelocity() - boostTarget);
            }

            logFrame(t, calculator.getDuty(), driver.getVelocity(), e, boost, "recover", lastLog);
            lastLog = t;
            telemetry.addData("boost 阶段", "恢复中 e=%.0f boost=%.4f", e, boost);
            telemetry.update();
            sleep(5);
        }
        driver.setTargetVelocity(0);
        driver.setOpenLoopPower(0);
    }

    // ------------------------------------------------------------ 报告

    private void report() {
        if (fit != null) {
            telemetry.addData(">>> 建议 kff (slope)", "%.8g", fit[0]);
            telemetry.addData(">>> 建议 kS (intercept)", "%.6g", fit[1]);
        } else {
            telemetry.addData("kS/kff", "拟合失败");
        }

        if (stepFit != null) {
            telemetry.addData("step Kd / τ", "%.1f / %.3f s", stepFit[2], stepFit[3]);
            telemetry.addData(">>> 建议 kp / ki", "%.8g / %.8g", stepFit[5], stepFit[6]);
        } else {
            telemetry.addData("kp/ki", "阶跃辨识失败");
        }

        telemetry.addData("boost 峰值误差 / boost", "%.0f / %.4f", dipPeakError, dipPeakBoost);
        telemetry.addData("boost 恢复时间", Double.isNaN(dipRecovery) ? "未恢复" : String.format(Locale.US, "%.3f s", dipRecovery));
        telemetry.addData("boost 过冲", "%.0f tick/s", dipOvershoot);
        if (!Double.isNaN(dipRecovery) && dipRecovery > targetRecoverySec * 1.1) {
            double suggested = Gen3Params.kBoost * dipRecovery / targetRecoverySec;
            telemetry.addData(">>> 建议 kBoost（恢复偏慢）", "%.8g", suggested);
        } else if (!Double.isNaN(dipRecovery)) {
            telemetry.addData(">>> kBoost 判读", "恢复已达标，保持 %.8g", Gen3Params.kBoost);
        }
        telemetry.addData("CSV", logger.isOpen() ? logger.getPath() : "未记录");
        telemetry.update();

        if (logger.isOpen()) {
            logger.log(String.format(Locale.US,
                    "# boost peakErr=%.0f peakBoost=%.5f recovery=%s overshoot=%.0f",
                    dipPeakError, dipPeakBoost,
                    Double.isNaN(dipRecovery) ? "NaN" : String.format(Locale.US, "%.3f", dipRecovery),
                    dipOvershoot));
        }

        while (opModeIsActive() && !gamepad1.x) {
            idle();
        }
    }

    // ------------------------------------------------------------ 工具

    private double measureOpenLoop(double duty, double settleSec, double timeoutSec) {
        driver.setOpenLoopPower(duty);
        double start = getRuntime();
        double sum = 0;
        int n = 0;
        double lastLog = -1;
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
            logFrame(getRuntime(), duty, v, 0, 0, "point", lastLog);
            lastLog = t;
            telemetry.addData("阶段", "point d=%.3f t=%.1f/%.1f ω=%.0f",
                    duty, t - start, timeoutSec, v);
            telemetry.update();
            sleep(5);
        }
        driver.setOpenLoopPower(0);
        return n > 0 ? sum / n : Double.NaN;
    }

    private void logFrame(double t, double duty, double velocity, double error, double boost,
                          String phase, double lastLog) {
        if (!logger.isOpen()) {
            return;
        }
        if (lastLog >= 0 && (t - lastLog) * 1000.0 < logPeriodMs) {
            return;
        }
        logger.log(String.format(Locale.US, "%.4f,%.4f,%.2f,%.1f,%.5f,%d",
                t, duty, velocity, error, boost, TuneGen1Op.phaseNum(phase)));
    }

    private void sleepSec(double sec) {
        long end = System.currentTimeMillis() + (long) (sec * 1000);
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            sleep(10);
        }
    }
}
