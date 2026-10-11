package org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * 原型发射机构（飞轮）控制器 —— 简化速度控制，直接输出占空比。
 *
 * <p>控制律（一阶等效模型 + 稳态前馈 + PI 反馈 + 掉速检测触发补偿）：
 * <pre>
 *   duty = Kff·ω* + Kp·err + Ki·∫err + Vboost
 *   err  = ω* - ω
 *   duty = sat(duty, dutyMin, dutyMax)
 * </pre>
 * 其中 ω* 为目标转速、ω 为实测转速，单位统一为 {@link DcMotorEx#getVelocity()} 的 tick/s；
 * 所有增益的量纲均为「占空比」，因此不读取电压传感器，{@link #calculate} 直接返回占空比。
 *
 * <p>电机置于 {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}，{@code setPower} 直接控制占空比。
 *
 * <p><b>I 项语义（上限 / 生效节点 / 清零节点）</b>
 * <ul>
 *   <li>上限：对积分项 {@code Ki·∫err} 限幅到 ±{@link #maxI}，饱和时把内部积分回算为 ±maxI/Ki，
 *       防止积分无界增长（抗饱和）。</li>
 *   <li>生效节点：仅当 {@code targetVelocity != 0} 且 {@code |err| < iZone} 时才累加积分。</li>
 *   <li>清零节点：① 停机（targetVelocity == 0）；② 误差带外（|err| ≥ iZone）；③ 触发掉速补偿时。</li>
 * </ul>
 *
 * <p><b>参数标定</b>：可依据《飞轮系统控制理论研究报告》第 6 章的黑箱辨识方法 —— 阶跃响应辨识
 * {@link #kff}/{@link #kp}/{@link #ki}，稳态速度波动定 {@link #dropThreshold}，正常工况最大减速度定
 * {@link #dropRateThreshold}，{@link #kBoost} 先取 2~5 倍 kp。
 */
@Config
public class PrototypeShooter {

    /** 电机名固定为 "shooter" */
    private static final String MOTOR_NAME = "shooter";

    public DcMotorEx motor;

    private Telemetry telemetry;

    // ==================== 可调参数 (FTC Dashboard 实时生效) ====================

    /** 稳态前馈增益，量纲为 (占空比)/(tick/s)，≈ 目标转速下的稳态占空比 / 目标转速 */
    public static double kff = 0.0002;
    /** 比例增益，量纲为 (占空比)/(tick/s) */
    public static double kp = 0.00005;
    /** 积分增益，量纲为 (占空比)/(tick/s·s) */
    public static double ki = 0.00002;

    /** I 项上限 (占空比)：对积分项 Ki·∫err 的限幅 */
    public static double maxI = 0.3;
    /** I 项生效节点 (tick/s)：|err| 小于该值时才累加积分 */
    public static double iZone = 500.0;

    /** 掉速判定阈值 (tick/s)：err 超过该值判定为掉速 */
    public static double dropThreshold = 300.0;
    /** 掉速速率阈值 (tick/s²)：dω/dt 低于其负值判定为掉速 */
    public static double dropRateThreshold = 8000.0;

    /** 掉速补偿增益，量纲为 (占空比)/(tick/s) */
    public static double kBoost = 0.0002;
    /** 掉速补偿上限 (占空比) */
    public static double boostMax = 0.4;
    /** 掉速补偿持续时长 (s)，超时后自动退出 */
    public static double boostDurationSec = 0.3;

    /** 输出下限 (占空比) */
    public static double dutyMin = 0.0;
    /** 输出上限 (占空比) */
    public static double dutyMax = 1.0;

    /** 电机方向：由飞轮装配方向决定 */
    public static boolean motorReversed = false;

    /** 目标转速 (tick/s)，0 表示停转 */
    public static int targetVelocity = 0;

    // ==================== 运行时状态 ====================

    /** 最近一次下发的占空比（供遥测） */
    private double duty = 0;

    /** 最近一次控制量各分量的值（供遥测：前馈 / 比例 / 积分 / 补偿） */
    private double lastFFTerm = 0, lastPTerm = 0, lastITerm = 0, lastBoostTerm = 0;

    /** 积分累计量 ∫err·dt (tick/s·s) */
    private double integral = 0;
    /** 上一次的实测转速 (tick/s)，用于估计 dω/dt */
    private double previousVelocity = 0;
    /** 上一次 calculate 的时间戳 (秒) */
    private double lastTime;

    /** 是否处于掉速补偿状态 */
    private boolean boostActive = false;
    /** 补偿已持续时长 (s) */
    private double boostTimer = 0;

    public PrototypeShooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        motor.setDirection(motorReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        // 软件闭环：关闭固件速度环，让 setPower 直接控制占空比
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // 飞轮惯性大，停转后让其自然滑行
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastTime = now();
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    /**
     * 设置目标转速。
     *
     * @param velocity 目标转速 (tick/s)，0 表示停转
     */
    public void setTargetVelocity(int velocity) {
        targetVelocity = velocity;
    }

    /** 停转（目标转速置 0，下一帧 update 会主动收油门并清零积分） */
    public void stop() {
        targetVelocity = 0;
    }

    /**
     * 速度环单步计算：输入目标转速与当前转速，输出占空比。
     *
     * <p>每帧只可调用一次（内部用计时器推进 dt、维护积分与补偿状态）。
     *
     * @param targetVel  目标转速 (tick/s)
     * @param currentVel 当前转速 (tick/s)
     * @return 占空比，已限幅到 [dutyMin, dutyMax]
     */
    public double calculate(double targetVel, double currentVel) {
        double currentTime = now();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        // 首帧或长时间停顿后 dt 异常，用标称循环周期兜底，避免微分项发散
        if (dt <= 0 || dt > 1.0) {
            dt = 0.02;
        }

        double error = targetVel - currentVel;
        double dVelocity = (currentVel - previousVelocity) / dt;
        previousVelocity = currentVel;

        // ---- 掉速检测与触发补偿 ----
        boolean dropDetected = targetVel != 0
                && (error > dropThreshold || dVelocity < -dropRateThreshold);
        if (dropDetected) {
            boostActive = true;
            boostTimer = 0.0;
            integral = 0.0; // 清零节点③：触发补偿时清零，避免与 boost 叠加
        }
        if (boostActive) {
            boostTimer += dt;
            if (boostTimer >= boostDurationSec || error <= 0) {
                boostActive = false; // 恢复到位或超时退出
            }
        }

        // ---- 积分：生效节点 / 清零节点 ----
        boolean iActive = targetVel != 0 && Math.abs(error) < iZone;
        if (iActive) {
            integral += error * dt; // 生效节点：目标非 0 且 |err| < iZone
        } else {
            integral = 0.0; // 清零节点①②：停机 或 |err| ≥ iZone
        }
        double iTerm = ki * integral;
        // 上限：对积分项回算限幅，防止积分饱和
        if (iTerm > maxI) {
            iTerm = maxI;
            if (ki != 0) integral = maxI / ki;
        } else if (iTerm < -maxI) {
            iTerm = -maxI;
            if (ki != 0) integral = -maxI / ki;
        }

        // ---- 前馈 + PI + 补偿，输出占空比 ----
        double boost = boostActive ? Math.max(0.0, Math.min(kBoost * error, boostMax)) : 0.0;
        lastFFTerm = kff * targetVel;
        lastPTerm = kp * error;
        lastITerm = iTerm;
        lastBoostTerm = boost;
        double out = lastFFTerm + lastPTerm + lastITerm + lastBoostTerm;
        if (out > dutyMax) out = dutyMax;
        if (out < dutyMin) out = dutyMin;

        this.duty = out;
        return out;
    }

    /**
     * 每帧调用一次：计算占空比并下发给电机。
     */
    public void update() {
        motor.setPower(calculate(targetVelocity, motor.getVelocity()));
    }

    // ==================== 标定辅助 ====================

    /**
     * 直接开环设定功率，绕过闭环（供 kS/kV 标定使用，见 ShooterTuningTester）。
     * 调用后下一帧闭环 {@link #update()} 会重新接管输出；切回闭环前请先调用 {@link #resetController()}。
     *
     * @param power 电机功率 [-1, 1]
     */
    public void setOpenLoopPower(double power) {
        motor.setPower(power);
    }

    /** 重置闭环控制器的积分与微分状态（从开环切回闭环前调用） */
    public void resetController() {
        integral = 0.0;
        previousVelocity = motor.getVelocity();
        boostActive = false;
        boostTimer = 0.0;
        lastFFTerm = 0;
        lastPTerm = 0;
        lastITerm = 0;
        lastBoostTerm = 0;
        duty = 0.0;
        lastTime = now();
    }

    // ==================== 读取 ====================

    /** @return 当前转速 (tick/s) */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /** @return 最近一次下发的占空比 */
    public double getDuty() {
        return duty;
    }

    /** @return 目标转速 (tick/s) */
    public int getTargetVelocity() {
        return targetVelocity;
    }

    /** @return 速度误差 (tick/s) */
    public double getError() {
        return targetVelocity - getVelocity();
    }

    /** @return 是否处于掉速补偿状态 */
    public boolean isBoostActive() {
        return boostActive;
    }

    /** @return 电机电流 (A) */
    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    /** 输出遥测（调用方需自行 telemetry.update()） */
    public void setTelemetry() {
        telemetry.addData("Shooter target (tick/s)", targetVelocity);
        telemetry.addData("Shooter velocity (tick/s)", "%.0f", getVelocity());
        telemetry.addData("Shooter error (tick/s)", "%.0f", getError());
        telemetry.addData("Shooter duty", "%.3f", duty);
        telemetry.addData("Shooter boost", boostActive);
        telemetry.addData("Shooter current (A)", "%.2f", getCurrent());
        // 输出分量分解，便于整定时判断各项贡献
        telemetry.addData("Shooter FF term", "%.4f", lastFFTerm);
        telemetry.addData("Shooter P term", "%.4f", lastPTerm);
        telemetry.addData("Shooter I term", "%.4f", lastITerm);
        telemetry.addData("Shooter boost term", "%.4f", lastBoostTerm);
    }
}
