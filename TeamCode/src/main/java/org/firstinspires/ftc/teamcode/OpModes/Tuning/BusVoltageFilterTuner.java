package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelParams;

/**
 * 标定方法七（第 4 代新增）：<b>母线电压低通时间常数 tauBus 的整定</b>。
 *
 * <p>第 4 代的电压归一化只乘在前馈上：
 * <pre>
 *   s = clamp(V_nominal / V_busFilt, 0.85, 1.25),   V_busFilt = EMA(V_bus, tauBus)
 * </pre>
 * tauBus 太小 → 把「单发发射造成的母线跌落」当成电量变化吃进前馈（前馈在恢复期被抬高，
 * 造出第二个满占空比），tauBus 太大 → 换电池、电量下降后前馈要很久才校正回来。
 * 因此 tauBus 由「一次发射的跌落幅度」与「电量变化的时间尺度」共同决定，本程序把前者量出来。
 *
 * <p><b>本程序做两件事</b>：
 * <ol>
 *   <li><b>A 键：稳态</b> —— 在 {@link #targetVelocity} 稳速运行，采样 {@link #sampleSec} 秒，
 *       给出母线纹波 {@code σ(V)}、稳态电压与「电压-电流」斜率（等效源内阻 R）</li>
 *   <li><b>B 键：扰动</b> —— 开始记录 {@link #launchRecordSec} 秒，期间手动发射一发（或用手压住飞轮模拟），
 *       对 5 个候选 tauBus 分别给出滤波后的最大跌落，并换算成「前馈占空比误差」
 *       {@code Δduty_ff = kff·ω*·(V_nom/V_filt - 1)}</li>
 * </ol>
 * 判定：取<b>最小</b>的 tauBus 使 {@code |Δduty_ff| ≤ maxFfDutyError}（默认 0.005 = 0.5% 占空比，
 * 约等于 kp 的 3 个量化台阶灵敏度）。默认值 1.0 s 就是这个判据在「单发跌落 3 V 左右」下算出来的。
 *
 * <p><b>操作</b>：A = 稳态测量；B = 扰动记录（先按 B 再立刻发射）；X = 停转。
 */
@Config
@TeleOp(name = "Tune6.7 BusVoltageFilter", group = "Tuning")
public class BusVoltageFilterTuner extends LinearOpMode {

    // ==================== 可调参数 ====================

    /** 测量所用目标转速 (tick/s)，应取实际发射工作点（前馈占空比越大，电压误差影响越大） */
    public static int targetVelocity = 2050;
    /** 进入目标后先稳定多久再采样 (s) */
    public static double settleSec = 2.0;
    /** 稳态采样窗口 (s) */
    public static double sampleSec = 3.0;
    /** 扰动记录时长 (s) */
    public static double launchRecordSec = 1.2;
    /** 前馈占空比误差判据：超过它说明 tauBus 太小 */
    public static double maxFfDutyError = 0.005;

    // ---- 5 个候选 tauBus (s) ----
    /** 候选 τ1 */
    public static double tau1 = 0.3;
    /** 候选 τ2 */
    public static double tau2 = 0.5;
    /** 候选 τ3 */
    public static double tau3 = 1.0;
    /** 候选 τ4 */
    public static double tau4 = 2.0;
    /** 候选 τ5 */
    public static double tau5 = 3.0;

    private static final int N_TAU = 5;
    private static final int MAX_SAMPLES = 2000;

    private final double[] taus = new double[N_TAU];
    private final double[] vf = new double[N_TAU];
    private final double[] devMax = new double[N_TAU];
    private final double[] sigmaFilt = new double[N_TAU];
    private final double[] sumFilt = new double[N_TAU];
    private final double[] sumFiltSq = new double[N_TAU];

    // 稳态统计
    private double sumV = 0, sumVSq = 0, sumI = 0, sumII = 0, sumVI = 0;
    private int steadyN = 0;
    private double busMean = 0;

    // 扰动记录
    private double baseV = 0;
    private double devPeak = 0;
    private double currentPeak = 0;

    private PrototypeShooter shooter;
    private final TuningLogger logger = new TuningLogger();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new PrototypeShooter(hardwareMap, telemetry);

        telemetry.addLine("Method 6.7  Bus voltage EMA filter calibration");
        telemetry.addLine("A: steady-state measurement    B: disturbance record (fire during it)    X: stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) runSteadyTest();
            if (gamepad1.bWasPressed()) runDisturbanceTest();
            if (gamepad1.xWasPressed()) {
                shooter.stop();
                shooter.update();
            }
            showResult();
            telemetry.update();
            idle();
        }

        shooter.stop();
        shooter.update();
        logger.close();
    }

    /** A：稳速采样，得到纹波、稳态电压与等效源内阻。 */
    private void runSteadyTest() {
        loadTaus();
        logger.start(hardwareMap.appContext, "bf07", "t,vbus,current,duty,velocity,f1,f2,f3,f4,f5");
        logger.log("# bus voltage steady test, target=" + targetVelocity
                + ", settleSec=" + settleSec + ", sampleSec=" + sampleSec
                + ", taus=" + java.util.Arrays.toString(taus));

        shooter.setTargetVelocity(targetVelocity);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < settleSec) {
            shooter.update();
            idle();
        }

        sumV = sumVSq = sumI = sumII = sumVI = 0;
        steadyN = 0;
        for (int k = 0; k < N_TAU; k++) {
            vf[k] = safeVoltage();
            sumFilt[k] = 0;
            sumFiltSq[k] = 0;
        }
        double lastT = timer.seconds();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < sampleSec) {
            double t = timer.seconds();
            double dt = t - lastT;
            lastT = t;
            double v = safeVoltage();
            double i = shooter.getCurrent();
            if (v > 0) {
                sumV += v;
                sumVSq += v * v;
                sumI += i;
                sumII += i * i;
                sumVI += v * i;
                steadyN++;
            }
            stepFilters(dt, v);
            for (int k = 0; k < N_TAU; k++) {
                sumFilt[k] += vf[k];
                sumFiltSq[k] += vf[k] * vf[k];
            }
            shooter.update();
            logger.logRowValues(t, v, i, shooter.getDuty(), shooter.getVelocity(),
                    vf[0], vf[1], vf[2], vf[3], vf[4]);
            idle();
        }
        busMean = steadyN > 0 ? sumV / steadyN : 0;
        for (int k = 0; k < N_TAU; k++) {
            sigmaFilt[k] = steadyN > 1 ? std(sumFiltSq[k], sumFilt[k], steadyN) : 0;
        }
        for (int k = 0; k < N_TAU; k++) {
            devMax[k] = 0;
        }
        devPeak = 0;
        currentPeak = 0;
        logger.close();
    }

    /** B：记录一次扰动（发射），对各候选 tauBus 量最大跌落。 */
    private void runDisturbanceTest() {
        loadTaus();
        logger.start(hardwareMap.appContext, "bf07d", "t,vbus,current,duty,velocity,f1,f2,f3,f4,f5");
        logger.log("# bus voltage disturbance record, launchRecordSec=" + launchRecordSec
                + ", taus=" + java.util.Arrays.toString(taus));

        shooter.setTargetVelocity(targetVelocity);
        double t = timer.seconds();
        double v = safeVoltage();
        baseV = v > 0 ? v : (busMean > 0 ? busMean : 12.0);
        for (int k = 0; k < N_TAU; k++) {
            vf[k] = baseV;
            devMax[k] = 0;
        }
        devPeak = 0;
        currentPeak = 0;

        timer.reset();
        double lastT = 0;
        while (opModeIsActive() && timer.seconds() < launchRecordSec) {
            t = timer.seconds();
            double dt = t - lastT;
            lastT = t;
            v = safeVoltage();
            double i = shooter.getCurrent();
            if (v > 0) {
                double dev = Math.abs(v - baseV);
                if (dev > devPeak) devPeak = dev;
                if (i > currentPeak) currentPeak = i;
            }
            stepFilters(dt, v);
            for (int k = 0; k < N_TAU; k++) {
                double d = Math.abs(vf[k] - baseV);
                if (d > devMax[k]) devMax[k] = d;
            }
            shooter.update();
            logger.logRowValues(t, v, i, shooter.getDuty(), shooter.getVelocity(),
                    vf[0], vf[1], vf[2], vf[3], vf[4]);
            idle();
        }
        logger.log(String.format(java.util.Locale.US,
                "# baseV=%.2f rawDevMax=%.3f currentPeak=%.2f filtDevMax=%s",
                baseV, devPeak, currentPeak, java.util.Arrays.toString(devMax)));
        logger.close();
    }

    private void loadTaus() {
        taus[0] = tau1;
        taus[1] = tau2;
        taus[2] = tau3;
        taus[3] = tau4;
        taus[4] = tau5;
    }

    private void stepFilters(double dt, double v) {
        if (!(v > 0)) return;
        if (!(dt > 0) || dt > 1.0) dt = 0.009;
        for (int k = 0; k < N_TAU; k++) {
            if (!(taus[k] > 0)) {
                vf[k] = v;
                continue;
            }
            double alpha = dt / (taus[k] + dt);
            vf[k] += alpha * (v - vf[k]);
        }
    }

    private double safeVoltage() {
        double v = shooter.getBusVoltage();
        return Double.isNaN(v) ? 0 : v;
    }

    private static double std(double sumSq, double sum, int n) {
        double mean = sum / n;
        double var = sumSq / n - mean * mean;
        return var > 0 ? Math.sqrt(var) : 0;
    }

    /** @return 等效源内阻 (Ω)：V = V0 - R·I 的最小二乘斜率；电流变化不足时返回 NaN */
    private double sourceResistance() {
        if (steadyN < 10) return Double.NaN;
        double denom = steadyN * sumII - sumI * sumI;
        if (Math.abs(denom) < 1e-9) return Double.NaN;
        double slope = (steadyN * sumVI - sumI * sumV) / denom;
        return -slope;
    }

    /**
     * 前馈占空比误差：{@code Δduty_ff = kff·ω*·(V_nom/V_filt - 1)}，
     * 用当前 {@link FlywheelParams#BIG} 的 kff 与 {@link #targetVelocity} 估算。
     *
     * @param vFilt 滤波后的母线电压 (V)
     */
    private double ffDutyError(double vFilt) {
        FlywheelParams p = shooter.getParams();
        double vNom = p.vBusNominal;
        if (!(vFilt > 1.0)) return Double.NaN;
        double ffDuty = p.kff * targetVelocity;
        return ffDuty * (vNom / vFilt - 1);
    }

    private void showResult() {
        telemetry.addData("target / taus", "%d / %s", targetVelocity, java.util.Arrays.toString(taus));
        telemetry.addData("bus mean / raw devMax", "%.2f V / %.2f V", busMean, devPeak);
        telemetry.addData("source resistance R", "%.3f ohm", sourceResistance());
        telemetry.addData("current peak", "%.2f A", currentPeak);
        telemetry.addLine("tau(s)  sigmaFilt(V)  filterDrop(V)  |dDuty_ff|");
        for (int k = 0; k < N_TAU; k++) {
            double vFilt = baseV - devMax[k];
            double e = ffDutyError(vFilt);
            telemetry.addLine(String.format(java.util.Locale.US, "%.1f     %.3f         %.3f          %s",
                    taus[k], sigmaFilt[k], devMax[k],
                    Double.isNaN(e) ? "n/a" : String.format(java.util.Locale.US, "%.4f", Math.abs(e))));
        }
        telemetry.addData(">>> suggested tauBus", "%.2f s", suggestedTau());
        telemetry.addData("CSV", logger.getPath());
    }

    /** @return 满足 {@link #maxFfDutyError} 的最小候选 tauBus；都不满足时返回最大者 */
    private double suggestedTau() {
        double best = taus[N_TAU - 1];
        for (int k = 0; k < N_TAU; k++) {
            double vFilt = baseV - devMax[k];
            double e = ffDutyError(vFilt);
            if (Double.isNaN(e)) continue;
            if (Math.abs(e) <= maxFfDutyError) {
                best = taus[k];
                break;
            }
        }
        return best;
    }
}
