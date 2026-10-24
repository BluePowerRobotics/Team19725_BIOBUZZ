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
 * 标定方法六（第 4 代新增）：<b>反馈速度滤波器时间常数 τV 的整定</b>。
 *
 * <p>第 4 代控制律里 PI 用的不是电机原始读数，而是经过一阶 EMA 的 {@code ωf}：
 * <pre>
 *   α = dt / (τV + dt),   ωf += α·(ω - ωf),   τV = tauVMax → tauVMin（按 |e| 调度）
 * </pre>
 * τV 的两个极端各自对应一个可测的代价：
 * <ul>
 *   <li><b>τV 越大</b> → 稳态噪声被压得越狠（{@code σ_filt/σ_raw = sqrt(α/(2-α))}），
 *       但抖动/退饱和被推迟（延迟 ≈ τV），且会把冲击的深度"吃掉"（斜坡响应是打折不是平移）</li>
 *   <li><b>τV 越小</b> → 反应快，但 PI 把量化噪声（20 tick/s 一格）当成误差去响应，
 *       稳态占空比抖动变大</li>
 * </ul>
 *
 * <p><b>本程序做两件事</b>（一次 A 键全跑完）：
 * <ol>
 *   <li><b>稳态噪声</b>：在 {@link #steadyTarget} 稳住后，统计原始速度标准差 {@code σ_raw}，
 *       并对 6 个候选 τ 各跑一个 EMA，给出各自的 {@code σ_filt}</li>
 *   <li><b>阶跃滞后</b>：目标从 {@link #steadyTarget} 阶跃 +{@link #stepDelta}，
 *       记录 6 个候选 τ 各自到达 90% 所需时间（相对原始速度的 90% 时刻）</li>
 * </ol>
 * 两个表合起来选参数：{@code tauVMax} 取「稳态 σ_filt ≤ 半个量化台阶（10 tick/s）且滞后可接受」的那个，
 * {@code tauVMin} 取「滞后影响可忽略（≈1~2 个控制周期）」的那个。
 *
 * <p>也可以不测：{@link FlywheelParams} 的默认值（30 ms / 8 ms）来自 2026-10 的离线核算
 * （见 {@code doc/现在的控制系统.md}），本程序用于现场复核与换硬件后重标。
 *
 * <p><b>操作</b>：A = 跑一次测量；X = 立即停转。
 *
 * <p><b>判读</b>
 * <ul>
 *   <li>{@code σ_raw} 通常在 10~16 tick/s（量化台阶 20 的一半量级）</li>
 *   <li>推荐 τ 见遥测最后一行：由 {@code σ_target = max(quantStep/2, 0.5%·ω)} 反解
 *       {@code α = 2r²/(1+r²)}、{@code τ = dt(1-α)/α}，其中 {@code r = σ_target/σ_raw}</li>
 *   <li>若推荐 τ 与 30 ms 差 2 倍以上，说明量化台阶/噪声特性变了（换编码器、换齿比、加滤波硬件），
 *       应同步复核 {@code quantStep}</li>
 * </ul>
 */
@Config
@TeleOp(name = "Tune6.6 VelocityFilter", group = "Tuning")
public class VelocityFilterTuner extends LinearOpMode {

    // ==================== 可调参数 ====================

    /** 稳态测量用的目标转速 (tick/s) */
    public static int steadyTarget = 2050;
    /** 稳态稳定时间 (s)：进入该目标后先跑这么久再统计 */
    public static double settleSec = 2.0;
    /** 稳态统计窗口 (s) */
    public static double windowSec = 1.5;
    /** 阶跃幅度 (tick/s)：从 steadyTarget 往上加 */
    public static int stepDelta = 300;
    /** 阶跃记录时长 (s) */
    public static double stepRecordSec = 1.5;

    // ---- 6 个候选时间常数 (s) ----
    /** 候选 τ1 */
    public static double tau1 = 0.008;
    /** 候选 τ2 */
    public static double tau2 = 0.012;
    /** 候选 τ3 */
    public static double tau3 = 0.016;
    /** 候选 τ4 */
    public static double tau4 = 0.020;
    /** 候选 τ5 */
    public static double tau5 = 0.025;
    /** 候选 τ6 */
    public static double tau6 = 0.030;

    private static final int N_TAU = 6;
    private static final int MAX_SAMPLES = 4000;
    /** 90% 判据（阶跃幅值的比例） */
    private static final double RISE_FRACTION = 0.9;

    private final double[] taus = new double[N_TAU];
    private final double[] filt = new double[N_TAU];

    // 稳态统计累加器
    private final double[] sumFilt = new double[N_TAU];
    private final double[] sumFiltSq = new double[N_TAU];
    private double sumRaw = 0, sumRawSq = 0;
    private int steadyN = 0;

    // 阶跃记录
    private final double[] stepTime = new double[MAX_SAMPLES];
    private final double[] stepVel = new double[MAX_SAMPLES];
    private final double[][] stepFilt = new double[N_TAU][MAX_SAMPLES];
    private int stepN = 0;

    // 结果
    private double sigmaRaw = 0;
    private final double[] sigmaFilt = new double[N_TAU];
    private final double[] riseTime = new double[N_TAU];
    private double velocity0 = 0, velocitySS = 0;
    private double dtMeasured = 0;
    private double suggestedTau = Double.NaN;
    private double suggestedSigma = Double.NaN;

    private PrototypeShooter shooter;
    private final TuningLogger logger = new TuningLogger();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new PrototypeShooter(hardwareMap, telemetry);

        telemetry.addLine("Method 6.6  Velocity EMA filter calibration");
        telemetry.addLine("A: run test    X: stop motor");
        telemetry.addLine("Tune steadyTarget/stepDelta/tau1..tau6 in Dashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) runTest();
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

    /** 一次完整测量：稳态噪声 → 阶跃滞后。 */
    private void runTest() {
        taus[0] = tau1;
        taus[1] = tau2;
        taus[2] = tau3;
        taus[3] = tau4;
        taus[4] = tau5;
        taus[5] = tau6;

        logger.start(hardwareMap.appContext, "vf06",
                "t,velocity,duty,vbus,f1,f2,f3,f4,f5,f6");
        logger.log("# velocity EMA filter calibration, steadyTarget=" + steadyTarget
                + ", settleSec=" + settleSec + ", windowSec=" + windowSec
                + ", stepDelta=" + stepDelta + ", taus=" + java.util.Arrays.toString(taus));

        // ---- 阶段 0：把飞轮稳到 steadyTarget，并把 6 个滤波器播种到当前速度 ----
        shooter.setTargetVelocity(steadyTarget);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < settleSec) {
            shooter.update();
            seedFilters(shooter.getVelocity());
            idle();
        }

        // ---- 阶段 1：稳态噪声统计 ----
        for (int k = 0; k < N_TAU; k++) {
            sumFilt[k] = 0;
            sumFiltSq[k] = 0;
        }
        sumRaw = 0;
        sumRawSq = 0;
        steadyN = 0;
        double lastTime = timer.seconds();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < windowSec) {
            double now = timer.seconds();
            double dt = now - lastTime;
            lastTime = now;
            double v = stepFilters(dt, shooter.getVelocity());
            if (steadyN == 0) dtMeasured = dt;
            sumRaw += v;
            sumRawSq += v * v;
            for (int k = 0; k < N_TAU; k++) {
                sumFilt[k] += filt[k];
                sumFiltSq[k] += filt[k] * filt[k];
            }
            steadyN++;
            shooter.update();
            logger.logRowValues(timer.seconds(), v, shooter.getDuty(), safeVoltage(),
                    filt[0], filt[1], filt[2], filt[3], filt[4], filt[5]);
            idle();
        }
        velocity0 = steadyN > 0 ? sumRaw / steadyN : 0;
        sigmaRaw = steadyN > 1 ? std(sumRawSq, sumRaw, steadyN) : 0;
        for (int k = 0; k < N_TAU; k++) {
            sigmaFilt[k] = steadyN > 1 ? std(sumFiltSq[k], sumFilt[k], steadyN) : 0;
        }

        // ---- 阶段 2：阶跃 +stepDelta，量各候选 τ 的 90% 滞后 ----
        stepN = 0;
        shooter.setTargetVelocity(steadyTarget + stepDelta);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < stepRecordSec && stepN < MAX_SAMPLES) {
            double t = timer.seconds();
            double v = shooter.getVelocity();
            // dt 用相邻样本的时间差（先把滤波器推进到本样本，再存本样本的滤波值）
            double dt = (stepN >= 1) ? (t - stepTime[stepN - 1]) : dtMeasured;
            stepFilters(dt, v);
            stepTime[stepN] = t;
            stepVel[stepN] = v;
            for (int k = 0; k < N_TAU; k++) {
                stepFilt[k][stepN] = filt[k];
            }
            stepN++;
            shooter.update();
            logger.logRowValues(t, v, shooter.getDuty(), safeVoltage(),
                    filt[0], filt[1], filt[2], filt[3], filt[4], filt[5]);
            idle();
        }
        shooter.setTargetVelocity(steadyTarget);
        shooter.update();

        // 阶跃末段均值作为 ω(∞)
        int tail = Math.min(stepN - 1, (int) Math.max(5, 0.2 / Math.max(dtMeasured, 1e-3)));
        if (tail > 0) {
            double s = 0;
            for (int j = stepN - 1; j > stepN - 1 - tail && j >= 0; j--) s += stepVel[j];
            velocitySS = s / tail;
        }

        // 各候选的 90% 时刻：原始速度达标时刻做基准，再算各滤波器滞后
        double rawRise = timeToReach(stepTime, stepVel, stepN, velocity0, velocitySS);
        for (int k = 0; k < N_TAU; k++) {
            double tFilt = timeToReach(stepTime, stepFilt[k], stepN, velocity0, velocitySS);
            riseTime[k] = (Double.isNaN(rawRise) || Double.isNaN(tFilt)) ? Double.NaN : tFilt - rawRise;
        }

        computeSuggestion();

        logger.log(String.format(java.util.Locale.US,
                "# sigmaRaw=%.2f sigmaFilt=%s riseLag=%s suggestedTau=%.4f",
                sigmaRaw, java.util.Arrays.toString(round(sigmaFilt)), java.util.Arrays.toString(round(riseTime)),
                suggestedTau));
        logger.close();
    }

    /** 把 6 个滤波器都播种到给定速度（进入稳态统计前调用，避免从 0 爬升污染统计）。 */
    private void seedFilters(double v) {
        for (int k = 0; k < N_TAU; k++) filt[k] = v;
    }

    /**
     * 推进 6 个候选 EMA 滤波器。
     *
     * @return 本帧的原始速度
     */
    private double stepFilters(double dt, double v) {
        if (!(dt > 0) || dt > 1.0) dt = (dtMeasured > 0 ? dtMeasured : 0.009);
        for (int k = 0; k < N_TAU; k++) {
            if (!(taus[k] > 0)) {
                filt[k] = v;
                continue;
            }
            double alpha = dt / (taus[k] + dt);
            filt[k] += alpha * (v - filt[k]);
        }
        return v;
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

    /**
     * @return 序列首次达到 {@code start + 0.9·(end-start)} 的时刻；达不到返回 NaN
     */
    private static double timeToReach(double[] t, double[] y, int n, double start, double end) {
        double target = start + RISE_FRACTION * (end - start);
        for (int i = 0; i < n; i++) {
            if (y[i] >= target) return t[i];
        }
        return Double.NaN;
    }

    /**
     * 由稳态噪声反解推荐 τ：{@code σ_filt/σ_raw = sqrt(α/(2-α))}，
     * 目标 {@code σ_target = max(quantStep/2, 0.5%·ω*)}。
     */
    private void computeSuggestion() {
        FlywheelParams p = shooter.getParams();
        double omega = Math.abs(steadyTarget);
        double sigmaTarget = Math.max(p.quantStep / 2.0, 0.005 * omega);
        suggestedSigma = sigmaTarget;
        if (sigmaRaw <= 1e-9) {
            suggestedTau = Double.NaN;
            return;
        }
        double r = Math.min(1.0, sigmaTarget / sigmaRaw);
        double alpha = 2 * r * r / (1 + r * r);
        double dt = dtMeasured > 0 ? dtMeasured : 0.009;
        suggestedTau = alpha > 1e-9 ? dt * (1 - alpha) / alpha : Double.NaN;
    }

    private static double[] round(double[] a) {
        double[] out = new double[a.length];
        for (int i = 0; i < a.length; i++) out[i] = Double.isNaN(a[i]) ? a[i] : Math.round(a[i] * 1000.0) / 1000.0;
        return out;
    }

    private void showResult() {
        telemetry.addData("steadyTarget / stepDelta", "%d / +%d", steadyTarget, stepDelta);
        telemetry.addData("dt measured (ms)", "%.1f", dtMeasured * 1000.0);
        telemetry.addData("sigma raw (tick/s)", "%.2f", sigmaRaw);
        telemetry.addData("omega0 / omegaSS", "%.0f / %.0f", velocity0, velocitySS);
        telemetry.addLine("tau(ms)  sigmaFilt  riseLag(ms)");
        for (int k = 0; k < N_TAU; k++) {
            telemetry.addLine(String.format(java.util.Locale.US, "%.0f      %.2f       %s",
                    taus[k] * 1000.0, sigmaFilt[k],
                    Double.isNaN(riseTime[k]) ? "n/a" : String.format(java.util.Locale.US, "%.0f", riseTime[k] * 1000.0)));
        }
        telemetry.addData("sigma target", "%.2f", suggestedSigma);
        telemetry.addData(">>> suggested tauVMax", "%.4f s (%s)",
                suggestedTau, Double.isNaN(suggestedTau) ? "invalid" : String.format(java.util.Locale.US, "%.1f ms", suggestedTau * 1000));
        telemetry.addData("CSV", logger.getPath());
    }
}
