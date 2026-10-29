package org.firstinspires.ftc.teamcode.utility.pastcalculater;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;

/**
 * 一个极简的飞轮驱动器：**一个电机 + 一个 {@link PastCalculator}**。
 *
 * <p>存在的理由：对比实验要“用第 1/3 代的计算器驱动真电机”，而 {@code PrototypeShooter}
 * 内部写死第 4 代计算器。这里自己开一个电机口，不碰 Controllers 层，也不影响正式程序。
 *
 * <p>与 {@code PrototypeShooter} 保持一致的三个约定：
 * ① 电机工作模式 {@code RUN_WITHOUT_ENCODER}，{@code setPower} 直接给占空比；
 * ② 母线电压取所有 {@code voltageSensor} 里**最小的正读数**（与官方遥测同口径）；
 * ③ {@code dt} 由 {@code nanoTime} 推进，异常值兜底 0.02 s。
 */
public final class PastFlywheelDriver {

    private static final double DEFAULT_DT = 0.02;

    /** 实际驱动的电机 */
    public final DcMotorEx motor;

    private final PastCalculator calc;
    private final List<VoltageSensor> voltageSensors = new ArrayList<>();

    private double lastNano = 0;
    private int targetVelocity = 0;

    public PastFlywheelDriver(HardwareMap hardwareMap, String motorName,
                              boolean motorReversed, PastCalculator calculator) {
        this.calc = calculator;
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);

        for (VoltageSensor s : hardwareMap.voltageSensor) {
            voltageSensors.add(s);
        }

        motor.setDirection(motorReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
    }

    /** 本驱动器的计算器（读分量用）。 */
    public PastCalculator getCalculator() {
        return calc;
    }

    public int getTargetVelocity() {
        return targetVelocity;
    }

    public void setTargetVelocity(int target) {
        this.targetVelocity = target;
    }

    public void stop() {
        targetVelocity = 0;
        motor.setPower(0);
        calc.reset(motor.getVelocity());
    }

    /** 闭环推进一帧：读转速/母线 → 计算 → 写占空比。 */
    public void update() {
        update(targetVelocity);
    }

    /** 闭环推进一帧到指定目标。 */
    public void update(int target) {
        targetVelocity = target;
        double dt = nextDt();
        motor.setPower(drive(target, motor.getVelocity(), getBusVoltage(), dt));
    }

    /**
     * 用给定输入算一步并**写进电机**（对比实验里给“当前代”用）。
     *
     * @return 本帧占空比
     */
    public double drive(int target, double velocity, double busVoltage, double dt) {
        double duty = calc.calculate(target, velocity, busVoltage, dt);
        motor.setPower(duty);
        return duty;
    }

    /** 开环给占空比（标定用；切回闭环前请 {@link #resetController()}）。 */
    public void setOpenLoopPower(double power) {
        motor.setPower(power);
    }

    /** 重新播种滤波器（从开环切回闭环、或目标大跳变时用）。 */
    public void resetController() {
        calc.reset(motor.getVelocity());
        lastNano = 0;
    }

    /** 距上一帧的时间；首次/异常返回 {@value #DEFAULT_DT}。 */
    public double nextDt() {
        long now = System.nanoTime();
        double dt = lastNano == 0 ? DEFAULT_DT : (now - lastNano) / 1e9;
        lastNano = now;
        if (dt <= 0 || dt > 1.0) {
            dt = DEFAULT_DT;
        }
        return dt;
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getCurrent() {
        return motor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
    }

    /** 所有电压传感器里最小的正读数；无有效读数返回 {@link Double#NaN}。 */
    public double getBusVoltage() {
        double min = Double.NaN;
        for (VoltageSensor s : voltageSensors) {
            double v = s.getVoltage();
            if (v > 0 && (Double.isNaN(min) || v < min)) {
                min = v;
            }
        }
        return min;
    }
}
