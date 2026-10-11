package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;

import java.util.ArrayList;
import java.util.Locale;

/**
 * 飞轮 PID/SVA 整定测试台 —— 按「先前馈、后反馈」流程在实机上整定 {@link PrototypeShooter} 的速度环。
 *
 * <p><b>操作</b>
 * <ul>
 *   <li>方向键左 / 右 → 切换整定模式（切换时自动停车，保留已得结论）</li>
 *   <li>A 键 → 启动 / 停止当前模式的自动流程</li>
 *   <li>B 键 → 清除数据与结论并停车</li>
 *   <li>Y 键 → 紧急停转</li>
 * </ul>
 *
 * <p><b>整定流程（先前馈，后反馈）</b>
 * <ol>
 *   <li>模式 0：kS 标定（开环）。功率从 0 以 rampRate 缓升，起转瞬间记录临界功率 → 建议 kS</li>
 *   <li>模式 1：kV 标定（开环）。依次施加 powerSteps 各档功率，平稳后记录 (转速, 功率) 样本，
 *       结束后线性回归 P = kS + kV·v → 建议 kV（斜率）与 kS（截距）</li>
 *   <li>模式 2：阶跃响应（闭环）。在 0 与 stepTarget 间自动往复，
 *       统计 90% 上升时间、超调量与整定时间，用于调 kP/kI（在 Dashboard 的 PrototypeShooter 分组修改）</li>
 *   <li>模式 3：扰动恢复（闭环）。保持发射转速，人工投球，
 *       统计转速跌落深度与恢复时间，用于复核 kP/kI</li>
 * </ol>
 *
 * <p>模式 0/1 通过 {@link PrototypeShooter#setOpenLoopPower(double)} 绕过闭环；
 * 模式 2/3 为闭环，其 PID 参数仍在 Dashboard 的 PrototypeShooter 分组实时修改。
 * 遥测中 target / velocity / error / power 曲线可加入 Dashboard plot 观察。
 *
 * <p>整定完成后按规范将结果硬编码回 {@link PrototypeShooter} 的静态字段。
 */
@Config
@TeleOp(name = "ShooterTuningTester", group = "Tests")
public class ShooterTuningTester extends LinearOpMode {

    // ==================== 可调参数 (FTC Dashboard 实时生效) ====================

    // ---- 模式 0：kS 标定 ----
    /** 开环功率上升速率 (功率/秒) */
    public static double rampRate = 0.01;
    /** 起转判定阈值 (tick/s)，取小值可减小 kS 高估 */
    public static double spinDetectThreshold = 60;
    /** 升到该功率仍未起转则放弃 (功率) */
    public static double rampPowerLimit = 1.0;

    // ---- 模式 1：kV 标定 ----
    /** 开环功率档位，应覆盖实际发射转速对应的功率范围 */
    public static double[] powerSteps = {0.25, 0.40, 0.55, 0.70};
    /** 平稳判定窗口 (秒)：相邻窗口转速差小于 settleTolerance 视为平稳 */
    public static double settleWindow = 0.5;
    /** 平稳/整定/恢复判定容差 (tick/s) */
    public static double settleTolerance = 40;
    /** 单档最长等待 (秒)，超时强制记录样本 */
    public static double maxStepWait = 8.0;

    // ---- 模式 2：阶跃响应 ----
    /** 阶跃目标转速 (tick/s) */
    public static double stepTarget = 2500;
    /** 0 与 stepTarget 往复半周期 (秒) */
    public static double cycleTime = 4.0;

    // ---- 模式 3：扰动恢复 ----
    /** 保持的发射转速 (tick/s) */
    public static double disturbTarget = 2500;
    /** 判定发生扰动的跌落阈值 (tick/s) */
    public static double dipDetect = 300;

    // ==================== 模式定义 ====================

    private static final int MODE_KS = 0;
    private static final int MODE_KV = 1;
    private static final int MODE_STEP = 2;
    private static final int MODE_DISTURB = 3;
    private static final int MODE_COUNT = 4;

    private static final String[] MODE_HINTS = {
            "0 kS calib (open loop): A=start ramp",
            "1 kV calib (open loop): A=start power steps",
            "2 step response (closed loop): A=auto 0<->target",
            "3 disturbance (closed loop): A=hold speed, feed balls"
    };

    // ==================== 运行时状态 ====================

    private PrototypeShooter shooter;
    private final ElapsedTime timer = new ElapsedTime();

    private int mode = MODE_KS;

    // 模式 0
    private boolean ksRunning = false;
    private double suggestedKS = Double.NaN;

    // 模式 1
    private boolean kvRunning = false;
    private int kvIndex = 0;
    private double kvCheckStart = 0;
    private double kvLastCheckVel = 0;
    private double kvStepStart = 0;
    /** 样本列表，每项为 {velocity (tick/s), power} */
    private final ArrayList<double[]> samples = new ArrayList<>();
    private double suggestedKV = Double.NaN;
    private double suggestedKSFromFit = Double.NaN;

    // 模式 2
    private boolean stepRunning = false;
    private double lastStepTarget = 0;
    private double stepStart = 0;
    private double stepMaxVel = 0;
    private double riseTime = -1;
    private double settleTime = -1;
    private boolean inBand = false;
    private double inBandSince = 0;
    private double lastOvershoot = Double.NaN;

    // 模式 3
    private boolean disturbRunning = false;
    private boolean disturbed = false;
    private double dipStart = 0;
    private double dipMinVel = 0;
    private double lastDip = Double.NaN;
    private double lastRecovery = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new PrototypeShooter(hardwareMap, telemetry);
        shooter.stop();

        telemetry.addLine("Dpad L/R: mode | A: run/stop | B: clear | Y: E-stop");
        telemetry.addLine("PID params: Dashboard group PrototypeShooter");
        telemetry.addLine("Flow params: Dashboard group ShooterTuningTester");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            handleButtons();

            switch (mode) {
                case MODE_KS:
                    runKsMode();
                    break;
                case MODE_KV:
                    runKvMode();
                    break;
                case MODE_STEP:
                    runStepMode();
                    break;
                case MODE_DISTURB:
                    runDisturbMode();
                    break;
            }

            setReportTelemetry();
            shooter.setTelemetry();
            telemetry.update();
        }

        shooter.stop();
        shooter.setOpenLoopPower(0);
    }

    // ==================== 按键与流程控制 ====================

    private void handleButtons() {
        boolean right = gamepad1.dpadRightWasPressed();
        boolean left = gamepad1.dpadLeftWasPressed();
        if (right || left) {
            mode = (mode + (right ? 1 : MODE_COUNT - 1)) % MODE_COUNT;
            stopSequence();
        }
        if (gamepad1.aWasPressed()) {
            toggleSequence();
        }
        if (gamepad1.bWasPressed()) {
            stopSequence();
            samples.clear();
            suggestedKS = Double.NaN;
            suggestedKV = Double.NaN;
            suggestedKSFromFit = Double.NaN;
            lastOvershoot = Double.NaN;
            lastDip = Double.NaN;
        }
        if (gamepad1.yWasPressed()) {
            stopSequence();
        }
    }

    /** 停止当前模式的流程并停车（保留已得结论） */
    private void stopSequence() {
        ksRunning = false;
        kvRunning = false;
        stepRunning = false;
        disturbRunning = false;
        disturbed = false;
        shooter.stop();
        shooter.setOpenLoopPower(0);
        shooter.resetController();
    }

    /** A 键：启动/停止当前模式的自动流程 */
    private void toggleSequence() {
        switch (mode) {
            case MODE_KS:
                ksRunning = !ksRunning;
                if (ksRunning) {
                    suggestedKS = Double.NaN;
                    timer.reset();
                }
                break;

            case MODE_KV:
                kvRunning = !kvRunning;
                if (kvRunning) {
                    if (powerSteps.length == 0) {
                        kvRunning = false;
                        break;
                    }
                    samples.clear();
                    suggestedKV = Double.NaN;
                    suggestedKSFromFit = Double.NaN;
                    kvIndex = 0;
                    timer.reset();
                    kvCheckStart = 0;
                    kvStepStart = 0;
                    kvLastCheckVel = shooter.getVelocity();
                } else {
                    shooter.setOpenLoopPower(0);
                }
                break;

            case MODE_STEP:
                stepRunning = !stepRunning;
                if (stepRunning) {
                    timer.reset();
                    lastStepTarget = 0;
                } else {
                    shooter.stop();
                }
                break;

            case MODE_DISTURB:
                disturbRunning = !disturbRunning;
                if (disturbRunning) {
                    disturbed = false;
                    timer.reset();
                } else {
                    shooter.stop();
                }
                break;
        }
    }

    // ==================== 模式 0：kS 标定（开环缓升起转） ====================

    private void runKsMode() {
        if (!ksRunning) return;

        double p = rampRate * timer.seconds();
        telemetry.addData("kS ramp power", "%.3f", p);

        if (shooter.getVelocity() >= spinDetectThreshold) {
            suggestedKS = p;
            shooter.setOpenLoopPower(0);
            ksRunning = false;
        } else if (p >= rampPowerLimit) {
            shooter.setOpenLoopPower(0);
            ksRunning = false;
        } else {
            shooter.setOpenLoopPower(p);
        }
    }

    // ==================== 模式 1：kV 标定（开环功率台阶 + 线性回归） ====================

    private void runKvMode() {
        if (!kvRunning) return;

        double p = powerSteps[kvIndex];
        shooter.setOpenLoopPower(p);
        double v = shooter.getVelocity();
        double now = timer.seconds();

        if (now - kvCheckStart >= settleWindow) {
            boolean settled = Math.abs(v - kvLastCheckVel) < settleTolerance;
            boolean timeout = now - kvStepStart >= maxStepWait;
            kvLastCheckVel = v;
            kvCheckStart = now;

            if (settled || timeout) {
                // 低于起转阈值的档位（未克服静摩擦）不参与回归
                if (v >= spinDetectThreshold) {
                    samples.add(new double[]{v, p});
                }
                kvIndex++;
                kvStepStart = now;
                if (kvIndex >= powerSteps.length) {
                    finishKv();
                }
            }
        }

        if (kvRunning) {
            telemetry.addData("kV step", "%d/%d  P=%.3f", kvIndex + 1, powerSteps.length, p);
        }
    }

    /** 对样本做最小二乘回归 P = kS + kV·v：斜率 → kV，截距 → kS */
    private void finishKv() {
        kvRunning = false;
        shooter.setOpenLoopPower(0);

        int n = samples.size();
        if (n < 2) {
            return;
        }

        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double[] s : samples) {
            sx += s[0];
            sy += s[1];
            sxx += s[0] * s[0];
            sxy += s[0] * s[1];
        }
        double denom = n * sxx - sx * sx;
        if (Math.abs(denom) < 1e-9) {
            return;
        }
        suggestedKV = (n * sxy - sx * sy) / denom;
        suggestedKSFromFit = (sy - suggestedKV * sx) / n;
    }

    // ==================== 模式 2：阶跃响应（闭环自动往复） ====================

    private void runStepMode() {
        if (!stepRunning) return;

        double now = timer.seconds();
        double target = (Math.floor(now / cycleTime) % 2 == 0) ? stepTarget : 0;

        if (target != lastStepTarget) {
            if (target > 0) {
                beginStep(now);
            } else if (lastStepTarget > 0) {
                finishStep();
            }
            lastStepTarget = target;
        }

        shooter.setTargetVelocity((int) target);
        shooter.update();

        if (target > 0) {
            trackStep(now, shooter.getVelocity());
        }
    }

    private void beginStep(double now) {
        stepStart = now;
        stepMaxVel = 0;
        riseTime = -1;
        settleTime = -1;
        inBand = false;
    }

    private void trackStep(double now, double v) {
        if (v > stepMaxVel) stepMaxVel = v;
        if (riseTime < 0 && v >= 0.9 * stepTarget) riseTime = now - stepStart;

        boolean band = Math.abs(v - stepTarget) < settleTolerance;
        if (band && !inBand) {
            inBand = true;
            inBandSince = now;
        } else if (!band && inBand) {
            inBand = false;
        }
        if (settleTime < 0 && inBand && now - inBandSince >= settleWindow) {
            settleTime = now - stepStart;
        }
    }
    
    private void finishStep() {
        lastOvershoot = stepTarget > 0 ? (stepMaxVel - stepTarget) / stepTarget * 100.0 : 0;
    }

    // ==================== 模式 3：扰动恢复（闭环保持 + 人工投球） ====================

    private void runDisturbMode() {
        if (!disturbRunning) return;

        shooter.setTargetVelocity((int) disturbTarget);
        shooter.update();

        double v = shooter.getVelocity();
        double now = timer.seconds();

        if (!disturbed) {
            if (disturbTarget - v > dipDetect) {
                disturbed = true;
                dipStart = now;
                dipMinVel = v;
            }
        } else {
            if (v < dipMinVel) dipMinVel = v;
            if (disturbTarget - v < settleTolerance) {
                disturbed = false;
                lastDip = disturbTarget - dipMinVel;
                lastRecovery = now - dipStart;
            }
        }
    }

    // ==================== 遥测 ====================

    private void setReportTelemetry() {
        telemetry.addData("Mode", MODE_HINTS[mode]);

        if (!Double.isNaN(suggestedKS)) {
            telemetry.addData(">>> Suggested kS", "%.4f", suggestedKS);
        }
        for (int i = 0; i < samples.size(); i++) {
            telemetry.addData(String.format(Locale.US, "Sample %d", i + 1),
                    "v=%.0f tick/s @ P=%.3f", samples.get(i)[0], samples.get(i)[1]);
        }
        if (!Double.isNaN(suggestedKV)) {
            telemetry.addData(">>> Suggested kV", "%.6f", suggestedKV);
            telemetry.addData(">>> Suggested kS (fit)", "%.4f", suggestedKSFromFit);
        }
        if (!Double.isNaN(lastOvershoot)) {
            telemetry.addData("Step rise(90%)", riseTime >= 0 ? String.format(Locale.US, "%.2f s", riseTime) : "-");
            telemetry.addData("Step overshoot", "%.1f %%", lastOvershoot);
            telemetry.addData("Step settle", settleTime >= 0 ? String.format(Locale.US, "%.2f s", settleTime) : "not settled");
        }
        if (!Double.isNaN(lastDip)) {
            telemetry.addData("Last disturbance", "dip=%.0f tick/s, recovery=%.2f s", lastDip, lastRecovery);
        }

        telemetry.addData("Battery", "%.2f V", getBatteryVoltage());
    }

    /** 取各 Hub 电压最小值（近似电池电压） */
    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            min = Math.min(min, sensor.getVoltage());
        }
        return min;
    }
}
