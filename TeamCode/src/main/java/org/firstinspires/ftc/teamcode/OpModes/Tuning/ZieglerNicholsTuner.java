package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;

/**
 * 标定方法二（研究报告 6.2）：<b>Ziegler-Nichols 临界比例度法</b>。
 *
 * <p>只用比例控制 {@code d = Kp·(ω* - ω)}（Ki=0、前馈=0、掉速补偿关闭），从小到大增大 Kp，
 * 直到系统出现<b>等幅振荡</b>；此时 Kp 即临界增益 {@code Ku}，振荡周期即 {@code Tu}。再按 ZN 表：
 * <pre>
 *   Kp = 0.45·Ku      （占空比/(tick/s)）
 *   Ki = 0.54·Ku/Tu   （占空比/(tick/s·s)）
 * </pre>
 *
 * <p><b>操作</b>
 * <ul>
 *   <li>DPad 上 / 下 → 把 {@code PrototypeShooter.kp} 乘 / 除 1.1（也可直接在 Dashboard 里改）</li>
 *   <li>A → 锁定当前 kp 为 Ku，并用缓冲区估出的 Tu 计算建议参数</li>
 *   <li>B → 清除锁定结果与采样缓冲</li>
 * </ul>
 * OpMode 每帧强制 kff=0、ki=0 并关闭掉速补偿（<b>退出时自动恢复原值</b>）；速度样本实时写入 CSV。
 *
 * <p><b>调参</b>（Dashboard → ZieglerNicholsTuner 分组同名类）：targetVelocity 目标转速。
 */
@Config
@TeleOp(name = "Tune6.2 ZieglerNichols", group = "Tuning")
public class ZieglerNicholsTuner extends LinearOpMode {

    /** 目标转速 (tick/s) */
    public static int targetVelocity = 3000;
    /** 判定振荡所需的最少过零次数 */
    public static int minZeroCrossings = 4;
    /** 按住方向键时的 kp 连续调节周期 (ms) */
    public static long repeatPeriodMs = 150;

    private static final int BUF = 512;
    private final double[] bufV = new double[BUF];
    private final double[] bufT = new double[BUF];
    private int bufN = 0;
    private int bufNext = 0;

    /** 上一次方向键状态：-1 下 / 0 未按 / 1 上 */
    private int lastDir = 0;
    private final ElapsedTime repeatTimer = new ElapsedTime();

    private PrototypeShooter shooter;
    private final TuningLogger logger = new TuningLogger();
    private final ElapsedTime timer = new ElapsedTime();

    // 估计与锁定结果
    private double estTu = Double.NaN;
    private double estAmp = 0;
    private double estMean = 0;
    private boolean locked = false;
    private double ku = 0, sugKp = 0, sugKi = 0, tuLocked = 0;

    // 进入时保存被本 OpMode 强制改写的静态参数，退出时恢复，避免污染其它 OpMode
    private double origKff, origKi, origDropThreshold, origDropRateThreshold;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new PrototypeShooter(hardwareMap, telemetry);
        shooter.setTargetVelocity(targetVelocity);

        origKff = PrototypeShooter.kff;
        origKi = PrototypeShooter.ki;
        origDropThreshold = PrototypeShooter.dropThreshold;
        origDropRateThreshold = PrototypeShooter.dropRateThreshold;

        telemetry.addLine("Method 6.2  Ziegler-Nichols critical gain");
        telemetry.addLine("DPad U/D: x1.1 / /1.1  PrototypeShooter.kp");
        telemetry.addLine("A: lock Ku    B: reset");
        telemetry.update();

        waitForStart();
        logger.start(hardwareMap.appContext, "zn02");
        logger.log("# ZN critical-gain test, target=" + targetVelocity);
        timer.reset();

        while (opModeIsActive()) {
            // 强制 P-only 且关闭掉速补偿
            PrototypeShooter.kff = 0.0;
            PrototypeShooter.ki = 0.0;
            PrototypeShooter.dropThreshold = 1e9;
            PrototypeShooter.dropRateThreshold = 1e9;

            // 方向键增减 kp（按住则按 repeatPeriodMs 连续调节）
            int dir = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
            if (dir != lastDir) {
                if (dir != 0) PrototypeShooter.kp *= (dir > 0 ? 1.1 : 1.0 / 1.1);
                repeatTimer.reset();
            } else if (dir != 0 && repeatTimer.milliseconds() >= repeatPeriodMs) {
                PrototypeShooter.kp *= (dir > 0 ? 1.1 : 1.0 / 1.1);
                repeatTimer.reset();
            }
            lastDir = dir;

            shooter.setTargetVelocity(targetVelocity);
            shooter.update();

            double t = timer.seconds();
            push(t, shooter.getVelocity());
            logger.logRow(t, shooter.getDuty(), shooter.getVelocity());

            double[] osc = estimateOscillation();
            estTu = osc[0];
            estAmp = osc[1];
            estMean = osc[2];

            if (gamepad1.aWasPressed()) {
                if (!Double.isNaN(estTu) && estTu > 0) {
                    locked = true;
                    ku = PrototypeShooter.kp;
                    tuLocked = estTu;
                    sugKp = 0.45 * ku;
                    sugKi = 0.54 * ku / tuLocked;
                    logger.log(String.format(java.util.Locale.US,
                            "# Ku=%.8f Tu=%.4f Kp=%.8f Ki=%.8f", ku, tuLocked, sugKp, sugKi));
                }
            }
            if (gamepad1.bWasPressed()) {
                locked = false;
                bufN = 0;
                bufNext = 0;
            }

            showTelemetry();
            telemetry.update();
            idle();
        }

        shooter.motor.setPower(0);
        PrototypeShooter.kff = origKff;
        PrototypeShooter.ki = origKi;
        PrototypeShooter.dropThreshold = origDropThreshold;
        PrototypeShooter.dropRateThreshold = origDropRateThreshold;
        logger.close();
    }

    private void showTelemetry() {
        telemetry.addData("target (tick/s)", targetVelocity);
        telemetry.addData("velocity (tick/s)", "%.0f", shooter.getVelocity());
        telemetry.addData("current kp (Ku?)", "%.8f", PrototypeShooter.kp);
        telemetry.addLine("---- 振荡估计 ----");
        telemetry.addData("osc amp (tick/s)", "%.1f", estAmp);
        telemetry.addData("osc mean (tick/s)", "%.0f", estMean);
        telemetry.addData("Tu (s)", Double.isNaN(estTu) ? "n/a" : String.format(java.util.Locale.US, "%.4f", estTu));
        if (locked) {
            telemetry.addLine("---- 锁定结果 ----");
            telemetry.addData("Ku", "%.8f", ku);
            telemetry.addData("Tu (s)", "%.4f", tuLocked);
            telemetry.addData("建议 kp", "%.8f", sugKp);
            telemetry.addData("建议 ki", "%.8f", sugKi);
        }
    }

    // ==================== 采样缓冲 ====================

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

    /**
     * 由缓冲速度估计振荡周期与幅值。
     *
     * @return {Tu, amplitude, center}；Tu 为 NaN 表示当前不足以判定等幅振荡
     */
    private double[] estimateOscillation() {
        if (bufN < 10) return new double[]{Double.NaN, 0, 0};

        double mean = 0;
        for (int i = 0; i < bufN; i++) mean += vAt(i);
        mean /= bufN;

        double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        for (int i = 0; i < bufN; i++) {
            double v = vAt(i);
            if (v < min) min = v;
            if (v > max) max = v;
        }
        double amp = (max - min) / 2.0;

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
        if (crossings < minZeroCrossings || Double.isNaN(firstT) || lastT <= firstT) {
            return new double[]{Double.NaN, amp, mean};
        }
        int cycles = (crossings - 1) / 2;
        if (cycles < 1) return new double[]{Double.NaN, amp, mean};
        double tu = (lastT - firstT) / cycles;
        return new double[]{tu, amp, mean};
    }
}
