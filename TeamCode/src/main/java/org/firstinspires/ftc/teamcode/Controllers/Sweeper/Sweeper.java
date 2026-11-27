package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;

@Config
/**
 * 集球器（Sweeper）控制器
 *
 * 当前实现：<b>恒定功率驱动</b>。
 *   电机工作在 {@code RUN_WITHOUT_ENCODER}，每帧把目标功率直接 {@code setPower()} 下发，
 *   不做速度闭环（吸球/供球/吐球各用一个固定功率，可在 FTC Dashboard 实时调整）。
 *
 * 优点：逻辑简单、无固件速度环的饱和/振荡问题，负载（球多球少）变化时转速自然下降。
 * 代价：电池电压下降时同一功率对应的转速会变慢，需按实测电压重新标定功率值。
 *
 * 用法约定：每帧必须先调用 {@code setEat/setGiveBall/setOutput/setStop} 之一，
 * 再调用 {@link #update()} 下发；目标功率在 {@code update()} 末尾自动归零，
 * 漏调用即停车（安全默认）。
 */
public class Sweeper {
    /** 铲子舵机设备名：左右用 +L / +R 区分 */
    private static final String SHOVEL_SERVO_L = "shovelServoL";
    private static final String SHOVEL_SERVO_R = "shovelServoR";

    public DcMotorEx motor;
    /** 铲子升降舵机（左）：控制 intake 前方铲子在抬起 / 放下两个已知角度间切换 */
    public Servo shovelServoL;
    /** 铲子升降舵机（右）：与左侧同步动作，预设位置独立配置 */
    public Servo shovelServoR;

    private Telemetry telemetry;

    /** 吃球功率 [-1,1]（todo: 实机标定） */
    public static double EatPower = 0.7;
    /** 供球（送球进炮台）功率 [-1,1]（todo: 实机标定） */
    public static double GiveBallPower = 0.7;
    /** 反向吐球功率 [-1,1]（负值即反转） */
    public static double OutputPower = -0.5;

    /** 球道容量：球道计数（RobotPosition.getBallCount()）达到该值时自动停止吸球 */
    public static int BallMax = 4;

    /** 本帧目标功率 [-1,1] */
    private double targetPower = 0;
    /** 上一帧实际下发的功率，供遥测复用 */
    private double lastTargetPower = 0;

    /** 本帧目标是否为"吸球"（仅吸球受球道满容量保护，供料与吐球不受限） */
    private boolean eatCommand = false;

    /** 最近一次 update() 采到的功率，供读取与遥测复用 */
    private double cachedPower = 0;
    /** 最近一次 update() 采到的转速 (tick/s)，供读取与遥测复用 */
    private double cachedVelocity = 0;
    /** 最近一次 update() 采到的电流 (A)，供读取与遥测复用 */
    private double cachedCurrent = 0;

    public static int ForR = 0;

    /** 左铲子抬起时的舵机位置 [0,1]（todo: 实机标定） */
    public static double ShovelUpPosL = 0.5;
    /** 左铲子放下时的舵机位置 [0,1]（todo: 实机标定） */
    public static double ShovelDownPosL = 0.0;
    /** 右铲子抬起时的舵机位置 [0,1]（todo: 实机标定） */
    public static double ShovelUpPosR = 0.5;
    /** 右铲子放下时的舵机位置 [0,1]（todo: 实机标定） */
    public static double ShovelDownPosR = 0.0;

    /** 当前左铲子舵机目标位置（[0,1]） */
    private double targetShovelPosL = ShovelDownPosL;
    /** 当前右铲子舵机目标位置（[0,1]） */
    private double targetShovelPosR = ShovelDownPosR;

    public Sweeper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        this.shovelServoL = hardwareMap.get(Servo.class, SHOVEL_SERVO_L);
        this.shovelServoR = hardwareMap.get(Servo.class, SHOVEL_SERVO_R);
        setDirection();
        // 恒定功率驱动：绕过固件速度环，setPower 直接决定电机功率
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // 上电默认铲子抬起，等待操控/自动切换
        setShovelUp();
        shovelServoL.setPosition(targetShovelPosL);
        shovelServoR.setPosition(targetShovelPosR);
    }

    private void setDirection() {
        switch(ForR) {
            case 0:
                motor.setDirection(DcMotor.Direction.REVERSE);
                break;
            case 1:
                motor.setDirection(DcMotor.Direction.FORWARD);
                break;
        }
    }

    public void setEat() {
        targetPower = EatPower;
        eatCommand = true;
    }

    public void setGiveBall() {
        targetPower = GiveBallPower;
        eatCommand = false;
    }

    public void setOutput() {
        targetPower = OutputPower;
        eatCommand = false;
    }

    public void setStop() {
        targetPower = 0;
        eatCommand = false;
    }

    public void setTargetPower(double power) {
        targetPower = power;
        eatCommand = false;
    }

    /** 铲子抬起（左右同时切换到各自抬起角度） */
    public void setShovelUp() {
        targetShovelPosL = ShovelUpPosL;
        targetShovelPosR = ShovelUpPosR;
    }

    /** 铲子放下（左右同时切换到各自放下角度） */
    public void setShovelDown() {
        targetShovelPosL = ShovelDownPosL;
        targetShovelPosR = ShovelDownPosR;
    }

    /** @return 左右铲子当前目标位置是否均为「抬起」 */
    public boolean isShovelUp() {
        return Math.abs(targetShovelPosL - ShovelUpPosL) < 1e-6
                && Math.abs(targetShovelPosR - ShovelUpPosR) < 1e-6;
    }

    /**
     * 若铲子尚未抬起则抬起；已抬起时<b>不改变</b>当前目标位置（幂等）。
     *
     * <p>只写舵机位置，不动吸球电机目标速度，可在任意帧安全调用。
     */
    public void raiseShovelIfNotUp() {
        if (isShovelUp()) {
            return;
        }
        setShovelUp();
        shovelServoL.setPosition(targetShovelPosL);
        shovelServoR.setPosition(targetShovelPosR);
    }

    public void update() {
        // 球道保护：只在"吸球"（setEat）且球道已满时强制停吸。
        // 供料（setGiveBall）与反向吐球（setOutput）不受限，
        // 否则满球后会既吐不出球、也供不上料，形成互锁
        if (isBallLaneFull() && eatCommand) {
            targetPower = 0;
        }
        // 恒定功率：把目标功率直接下发给电机
        motor.setPower(targetPower);
        lastTargetPower = targetPower;
        targetPower = 0; // 每帧必须重新调用set函数，否则自动归零
        eatCommand = false;
        // 舵机保持位置：每帧刷新，保证 Dashboard 上改铲子预设位置后即时生效
        shovelServoL.setPosition(targetShovelPosL);
        shovelServoR.setPosition(targetShovelPosR);
        // 本帧唯一的硬件采样点：功率 / 转速 / 电流全部缓存，getter 与遥测只读缓存
        cachedPower = motor.getPower();
        cachedVelocity = motor.getVelocity();
        cachedCurrent = motor.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * 球道是否已达到容量（依据 RobotPosition 透传的球道计数）。
     * 未初始化 RobotPosition 时返回 false，即不做球道保护。
     */
    private boolean isBallLaneFull() {
        RobotPosition rp = RobotPosition.getInstanceOrNull();
        return rp != null && rp.getBallCount() >= BallMax;
    }

    public double getPower() {
        return cachedPower;
    }

    public double getVel() {
        return cachedVelocity;
    }

    /** @return 上一帧实际下发的功率 [-1,1] */
    public double getTargetPower() {
        return lastTargetPower;
    }

    public int getFR() {
        return ForR;
    }

    /** @return 左铲子舵机当前指令位置（[0,1]） */
    public double getShovelPosition() {
        return targetShovelPosL;
    }

    /** @return 右铲子舵机当前指令位置（[0,1]） */
    public double getShovelPositionR() {
        return targetShovelPosR;
    }

    public double getCurrent() {
        return cachedCurrent;
    }

    public void setTelemetry() {
        /*
        telemetry.addData("Sweeper Velocity", getVel());
        telemetry.addData("Sweeper Power*1000", getPower()*1000);
        telemetry.addData("Sweeper Current", getCurrent());

         */
    }
}