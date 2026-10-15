package org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelCalculator;

/**
 * 原型发射机构（飞轮）控制器 —— 简化速度控制，直接输出占空比。
 *
 * <p><b>职责划分</b>：本类只负责「电机 + 遥测 + 参数面板」，控制律本身已抽到
 * {@link FlywheelCalculator}（位于 {@code utility/Shooter}，纯计算、无硬件依赖）。
 * 每帧通过 {@link #syncCalculator()} 把本类的 {@code @Config} 静态参数同步给计算器，
 * 因此现场整定习惯（在 Dashboard 的 {@code PrototypeShooter} 分组改参数）完全不变。
 *
 * <p>控制律（静摩擦前馈 + 一阶稳态前馈 + PI 反馈 + 掉速检测触发补偿）：
 * <pre>
 *   duty = kS·sgn(ω*) + Kff·ω* + Kp·err + Ki·∫err + Vboost
 *   err  = ω* - ω
 *   duty = sat(duty, dutyMin, dutyMax)
 * </pre>
 * 其中 ω* 为目标转速、ω 为实测转速，单位统一为 {@link DcMotorEx#getVelocity()} 的 tick/s；
 * 所有增益的量纲均为「占空比」，因此控制律不读取电压传感器，{@link #calculate} 直接返回占空比
 * （电压只作为遥测量输出，供现场判断电池状态）。
 *
 * <p>电机置于 {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}，{@code setPower} 直接控制占空比。
 *
 * <p><b>参数修改约定</b>：新增/删除控制参数时，需同步修改
 * {@link FlywheelCalculator} 的字段与 {@link #syncCalculator()}，两处缺一不可。
 *
 * <p><b>参数标定</b>：可依据《飞轮系统控制理论研究报告》第 6 章的黑箱辨识方法 —— 先用稳态工作点回归
 * {@code d_ss(ω) = kS + kff·ω} 定 {@link #kS}/{@link #kff}（工具：{@code tools/tuning/fit_steady_state_model.py}），
 * 再由阶跃/临界比例度法定 {@link #kp}/{@link #ki}，稳态速度波动定 {@link #dropThreshold}，
 * 正常工况最大减速度定 {@link #dropRateThreshold}，{@link #kBoost} 先取 2~5 倍 kp。下方默认值即 2026-09-04
 * 实机整定结果（kS=0.02945、kff=0.00038307、kp=0.00181917、ki=0.00259224，kp×一个量化台阶=0.036 已达标）。
 */
@Config
public class PrototypeShooter {

    /** 电机名固定为 "shooter" */
    private static final String MOTOR_NAME = "shooter";

    public DcMotorEx motor;

    private Telemetry telemetry;

    /** 母线电压传感器（硬件配置里的全部电压传感器，取正读数最小值，与官方 ConceptTelemetry 一致） */
    private final Iterable<VoltageSensor> voltageSensors;

    /** 速度环计算器（纯计算，本类每帧同步参数给它） */
    private final FlywheelCalculator calculator = new FlywheelCalculator();

    // ==================== 可调参数 (FTC Dashboard 实时生效) ====================

    /**
     * 静摩擦前馈 (占空比)：稳态模型 {@code d_ss(ω) = kS + kff·ω} 的截距，即「只对抗摩擦、
     * 不产生转速」的那份占空比，按 {@code kS·sgn(ω*)} 施加（ω*=0 时自动为 0）。
     * 由 2026-09-04 实测 8 个工作点回归得到（R²=0.9949，RMS 残差 0.98% 占空比）。
     */
    public static double kS = 0.02945;
    /** 稳态前馈增益，量纲为 (占空比)/(tick/s)，即稳态模型的斜率；不含静摩擦项 */
    public static double kff = 0.00038307;
    /** 比例增益，量纲为 (占空比)/(tick/s)。量化台阶自检：kp × 一个台阶(20 tick/s) 应远小于 0.05 */
    public static double kp = 0.00181917;
    /** 积分增益，量纲为 (占空比)/(tick/s·s) */
    public static double ki = 0.00259224;

    /** I 项上限 (占空比)：对积分项 Ki·err 的限幅 */
    public static double maxI = 0.3;
    /** I 项生效节点 (tick/s)：|err| 小于该值时才累加积分 */
    public static double iZone = 500.0;

    /** 掉速判定阈值 (tick/s)：err 超过该值判定为掉速 */
    public static double dropThreshold = 100.0;
    /** 掉速速率阈值 (tick/s²)：dω/dt 低于其负值判定为掉速 */
    public static double dropRateThreshold = 12000.0;

    /** 掉速补偿增益，量纲为 (占空比)/(tick/s) */
    public static double kBoost = 0.00362834;
    /** 掉速补偿上限 (占空比) */
    public static double boostMax = 0.4;
    /** 掉速补偿持续时长 (s)，超时后自动退出 */
    public static double boostDurationSec = 0.3;

    /** 输出下限 (占空比) */
    public static double dutyMin = 0.0;
    /** 输出上限 (占空比) */
    public static double dutyMax = 1.0;

    /**
     * 电机方向：由飞轮装配方向（接线极性）决定，与「整定参数」无关。
     * 当前实机为反向接线，故取 true；若重新接线/更换电机后飞轮反转，把此值改回 false。
     * 注意：方向只在构造时下发一次，Dashboard 改动需重新 Start OpMode 才生效。
     */
    public static boolean motorReversed = true;

    /** 目标转速 (tick/s)，0 表示停转 */
    public static int targetVelocity = 0;

    /**
     * 是否在标准遥测之外追加诊断行（误差 / boost 状态 / FF·P·I·boost 分量）。
     * 默认 false，保持遥测简洁（标准 5 行：目标 / 当前 / power / 电流 / 电压）；
     * 需要复盘控制量分配时在 Dashboard 里勾上即可。
     */
    public static boolean telemetryDetailed = false;

    public PrototypeShooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        this.voltageSensors = hardwareMap.voltageSensor;

        motor.setDirection(motorReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        // 软件闭环：关闭固件速度环，让 setPower 直接控制占空比
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // 飞轮惯性大，停转后让其自然滑行
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        syncCalculator();
        calculator.reset();
    }

    /**
     * 把本类的静态参数同步给计算器。
     *
     * <p>参数留在本类是为了让 FTC Dashboard 继续按 {@code PrototypeShooter} 分组显示；
     * 计算器每帧取值，因此 Dashboard 改参数依旧实时生效。
     */
    private void syncCalculator() {
        calculator.kS = kS;
        calculator.kff = kff;
        calculator.kp = kp;
        calculator.ki = ki;
        calculator.maxI = maxI;
        calculator.iZone = iZone;
        calculator.dropThreshold = dropThreshold;
        calculator.dropRateThreshold = dropRateThreshold;
        calculator.kBoost = kBoost;
        calculator.boostMax = boostMax;
        calculator.boostDurationSec = boostDurationSec;
        calculator.dutyMin = dutyMin;
        calculator.dutyMax = dutyMax;
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
     * 速度环单步计算：输入目标转速与当前转速，输出占空比（内部推进 dt）。
     *
     * <p>每帧只可调用一次（计算器内部用计时器推进 dt、维护积分与补偿状态）。
     *
     * @param targetVel  目标转速 (tick/s)
     * @param currentVel 当前转速 (tick/s)
     * @return 占空比，已限幅到 [dutyMin, dutyMax]
     */
    public double calculate(double targetVel, double currentVel) {
        syncCalculator();
        return calculator.calculate(targetVel, currentVel);
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
        calculator.reset(motor.getVelocity());
    }

    // ==================== 读取 ====================

    /** @return 当前转速 (tick/s) */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /** @return 最近一次下发的占空比 */
    public double getDuty() {
        return calculator.getDuty();
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
        return calculator.isBoostActive();
    }

    /** @return 电机电流 (A) */
    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * 读取母线电压 (V)：遍历全部电压传感器，取<b>所有正读数中的最小值</b>，
     * 与官方示例 {@code ConceptTelemetry#getBatteryVoltage()} 一致（多传感器/扩展集线器场景下更保守）。
     *
     * @return 母线电压 (V)；无有效读数时返回 {@link Double#NaN}
     */
    public double getBusVoltage() {
        double result = -1;
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = (result < 0) ? voltage : Math.min(result, voltage);
            }
        }
        return result < 0 ? Double.NaN : result;
    }

    // ==================== 遥测 ====================

    /**
     * 标准飞轮遥测（5 行）：目标转速 / 当前转速 / power(占空比) / 电流 / 电压。
     *
     * <p>飞轮相关 OpMode 统一使用这一份列表；把 {@link #telemetryDetailed} 置 true 可在其后
     * 追加误差、boost 状态与 FF·P·I·boost 分量（复盘控制量分配时用）。
     *
     * <p>调用方需自行 {@code telemetry.update()}。
     */
    public void setTelemetry() {
        telemetry.addData("Shooter target (tick/s)", targetVelocity);
        telemetry.addData("Shooter velocity (tick/s)", "%.0f", getVelocity());
        telemetry.addData("Shooter power (duty)", "%.4f", getDuty());
        telemetry.addData("Shooter current (A)", "%.2f", getCurrent());
        telemetry.addData("Shooter voltage (V)", "%.2f", getBusVoltage());

        if (telemetryDetailed) {
            telemetry.addData("Shooter error (tick/s)", "%.0f", getError());
            telemetry.addData("Shooter boost", isBoostActive());
            telemetry.addData("Shooter kS term", "%.4f", calculator.getKSTerm());
            telemetry.addData("Shooter FF term", "%.4f", calculator.getFfTerm());
            telemetry.addData("Shooter P term", "%.4f", calculator.getPTerm());
            telemetry.addData("Shooter I term", "%.4f", calculator.getITerm());
            telemetry.addData("Shooter boost term", "%.4f", calculator.getBoostTerm());
        }
    }

    /**
     * 详细遥测：标准 5 行之外再输出误差、boost 状态与 FF·P·I·boost 分量分解。
     * 供整定类程序（如 ShooterTuningTester）使用，便于判断各分量贡献。
     *
     * <p>调用方需自行 {@code telemetry.update()}。
     */
    public void setTelemetryDetailed() {
        telemetry.addData("Shooter target (tick/s)", targetVelocity);
        telemetry.addData("Shooter velocity (tick/s)", "%.0f", getVelocity());
        telemetry.addData("Shooter error (tick/s)", "%.0f", getError());
        telemetry.addData("Shooter power (duty)", "%.4f", getDuty());
        telemetry.addData("Shooter boost", isBoostActive());
        telemetry.addData("Shooter current (A)", "%.2f", getCurrent());
        telemetry.addData("Shooter voltage (V)", "%.2f", getBusVoltage());
        // 输出分量分解，便于整定时判断各项贡献
        telemetry.addData("Shooter kS term", "%.4f", calculator.getKSTerm());
        telemetry.addData("Shooter FF term", "%.4f", calculator.getFfTerm());
        telemetry.addData("Shooter P term", "%.4f", calculator.getPTerm());
        telemetry.addData("Shooter I term", "%.4f", calculator.getITerm());
        telemetry.addData("Shooter boost term", "%.4f", calculator.getBoostTerm());
    }
}
