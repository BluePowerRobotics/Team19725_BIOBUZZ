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
 * 当前实现（固件速度闭环，推荐）：
 *   使用 REV Hub 内置速度 PIDF（RUN_USING_ENCODER + setVelocity + setVelocityPIDFCoefficients），
 *   PID 在 Hub 固件内以约 1kHz 完成，延迟低、且不占用 Robot Controller 线程。
 *
 * 未来如需软件闭环（自定义控制律：变增益/前馈/抗积分饱和等），可按以下步骤改造：
 *   1. 将 motor 切到 RUN_WITHOUT_ENCODER（让 setPower 直接控制功率）。
 *   2. 使用 utility.PID 包的 PIDSVAController + SlotConfig（或 PIDController）做速度闭环：
 *        PIDSVAController ctrl = new PIDSVAController().withSlot0(
 *            new SlotConfig()
 *                .withKP(kP).withKI(kI).withKD(kD)
 *                .withKS(kS).withKV(kV).withKA(kA)
 *                .withOutputLimits(-1.0, 1.0));
 *        // 每帧：double out = ctrl.calculate(targetVel, motor.getVelocity(), dt, true);
 *        //        motor.setPower(out);
 *   3. 具体用法参见 utility/PID/Guide.md 与 MotorPIDCore.java。
 * 注意：软件闭环延迟稍高且受 RC 线程节拍影响，仅在确需自定义控制律时切换。
 */
public class Sweeper {
    public DcMotorEx motor;
    /** 铲子升降舵机：控制 intake 前方铲子在抬起 / 放下两个已知角度间切换 */
    public Servo shovelServo;

    private Telemetry telemetry;

    public static int EatVel = 700;
    public static int GiveTheArtifactVel = 2000;
    public static int OutputVel = -500;

    /** 固件速度闭环 PIDF 系数（REV Hub 内置速度 PID，可在 FTC Dashboard 实时调参） */
    public static double VelocityP = 1.17;
    public static double VelocityI = 0.117;
    public static double VelocityD = 0.0;
    public static double VelocityF = 0.0;

    private int targetVelocity = 0;
    private int lastTargetVelocity = 0;

    /** 球道最多容纳的小球数：计数达到该值后自动停止吸球（送球 / 出球方向不受限制） */
    public static int BallMax = 4;

    /**
     * 本帧命令是否来自 {@link #setEat()}。
     * 满球保护只拦截吸球流程；送球（{@link #setGiveArtifact()}）与出球必须放行，
     * 否则满球时炮台无法把球道内的球送出去完成发射。
     */
    private boolean eatCommanded = false;

    // ==================== 吐球（Ball Eject，原独立模块合并于此） ====================

    /** 吐球时的目标转速 (tick/s，负值表示反向排出) */
    public static int EjectVel = -800;

    /** 单次吐球持续时间 (秒)，到时自动停止，防止长时间反转 */
    public static double EjectDurationSec = 0.8;

    /** 是否处于吐球状态（吐球期间优先级高于吸球/送球） */
    private boolean ejecting = false;

    /** 本次吐球的起始时刻（秒），由单调时钟提供 */
    private double ejectStartTimeSec = 0;

    /** 最近一次推进计算出的吐球剩余时间（秒） */
    private double ejectRemainingSec = 0;

    /** 最近一次 update() 采到的功率，供读取与遥测复用 */
    private double cachedPower = 0;
    /** 最近一次 update() 采到的转速 (tick/s)，供读取与遥测复用 */
    private double cachedVelocity = 0;
    /** 最近一次 update() 采到的电流 (A)，供读取与遥测复用 */
    private double cachedCurrent = 0;

    public static int ForR = 0;

    /** 铲子抬起时的舵机位置 [0,1]（todo: 实机标定） */
    public static double ShovelUpPos = 0.5;
    /** 铲子放下时的舵机位置 [0,1]（todo: 实机标定） */
    public static double ShovelDownPos = 0.0;

    /** 当前铲子舵机目标位置（[0,1]） */
    private double targetShovelPos = ShovelDownPos;

    public Sweeper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        this.shovelServo = hardwareMap.get(Servo.class, "shovelServo");
        setDirection();
        // 固件速度闭环：开启编码器速度模式并配置内置速度 PIDF
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(VelocityP, VelocityI, VelocityD, VelocityF);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // 上电默认铲子抬起，等待操控/自动切换
        setShovelUp();
        shovelServo.setPosition(targetShovelPos);
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

    /** 吸球：满球时会被 {@link #BallMax} 上限拦截并停止 */
    public void setEat() {
        targetVelocity = EatVel;
        eatCommanded = true;
    }

    /** 送球给炮台：满球时仍允许执行，用于发射道具 */
    public void setGiveArtifact() {
        targetVelocity = GiveTheArtifactVel;
        eatCommanded = false;
    }

    public void setOutput() {
        targetVelocity = OutputVel;
        eatCommanded = false;
    }

    /** 停止集球器：目标速度归零，并取消正在进行的吐球 */
    public void setStop() {
        targetVelocity = 0;
        eatCommanded = false;
        stopEject();
    }

    /**
     * 通用速度指令，不属于吸球流程，因此不受满球上限限制。
     * 需要满球自动停吸请使用 {@link #setEat()}。
     */
    public void setTargetVelocity(int velocity) {
        targetVelocity = velocity;
        eatCommanded = false;
    }

    /**
     * 开始吐球：把球道内多余的、可能影响发射或机构运行的球反向排出。
     * 单次持续 {@link #EjectDurationSec} 秒后自动结束，期间覆盖吸球/送球命令。
     */
    public void setEject() {
        setEject(now());
    }

    /**
     * 指定起始时刻触发吐球（包内可见，便于单元测试注入时间）。
     *
     * @param nowSec 单调时间（秒）
     */
    void setEject(double nowSec) {
        ejecting = true;
        ejectStartTimeSec = nowSec;
        ejectRemainingSec = EjectDurationSec;
    }

    /** 立即停止吐球（提前结束） */
    public void stopEject() {
        ejecting = false;
        ejectRemainingSec = 0;
    }

    /** @return 当前是否正在吐球 */
    public boolean isEjecting() {
        return ejecting;
    }

    /** @return 吐球剩余时间 (秒)，非吐球状态为 0 */
    public double getEjectRemainingSec() {
        return ejecting ? ejectRemainingSec : 0;
    }

    /**
     * 每帧推进一次吐球计时；到达 {@link #EjectDurationSec} 自动结束。
     *
     * @param nowSec 单调时间（秒）
     * @return 本帧是否仍在吐球
     */
    boolean updateEject(double nowSec) {
        if (!ejecting) {
            return false;
        }
        double elapsed = nowSec - ejectStartTimeSec;
        if (elapsed >= EjectDurationSec) {
            stopEject();
            return false;
        }
        ejectRemainingSec = EjectDurationSec - elapsed;
        return true;
    }

    /** @return 单调时钟（秒），供吐球计时使用 */
    private static double now() {
        return System.nanoTime() / 1e9;
    }

    /** 铲子抬起（切换到抬起角度） */
    public void setShovelUp() {
        targetShovelPos = ShovelUpPos;
    }

    /** 铲子放下（切换到放下角度） */
    public void setShovelDown() {
        targetShovelPos = ShovelDownPos;
    }

    public void update() {
        // ---- 吐球优先：推进吐球计时，吐球期间强制反向排出，覆盖本帧吸球/送球命令 ----
        boolean ejectingThisFrame = updateEject(now());

        int appliedVelocity;
        if (ejectingThisFrame) {
            appliedVelocity = EjectVel;
        } else {
            // 满球保护：球道计数由 RobotPosition 每帧更新，本类只读取其状态（不重复推进计数）。
            // 按命令来源区分两种满球场景：
            //   1. setEat（吸球）满球 → 停止吸球，避免超出容量；
            //   2. setGiveArtifact（送球）满球 → 必须放行，否则炮台无法把球送出去完成发射。
            // 出球（负速）与通用速度指令同样不受该上限拦截。
            RobotPosition robotPosition = RobotPosition.getInstanceOrNull();
            boolean ballLaneFull = robotPosition != null
                    && robotPosition.getBallCount() >= BallMax;
            if (ballLaneFull && eatCommanded) {
                targetVelocity = 0;
            }
            appliedVelocity = targetVelocity;
        }

        // 固件闭环：把目标速度下发给 Hub 内置速度 PID
        motor.setVelocity(appliedVelocity);
        lastTargetVelocity = appliedVelocity;
        targetVelocity = 0; // 每帧必须重新调用set函数，否则自动归零
        eatCommanded = false; // 命令来源同样每帧重新下发
        // 舵机保持位置：每帧刷新，保证 Dashboard 上改 ShovelUpPos/ShovelDownPos 后即时生效
        shovelServo.setPosition(targetShovelPos);
        // 本帧唯一的硬件采样点：功率 / 转速 / 电流全部缓存，getter 与遥测只读缓存
        cachedPower = motor.getPower();
        cachedVelocity = motor.getVelocity();
        cachedCurrent = motor.getCurrent(CurrentUnit.AMPS);
    }

    public double getPower() {
        return cachedPower;
    }

    public double getVel() {
        return cachedVelocity;
    }

    public int getTargetVelocity() {
        return lastTargetVelocity;
    }

    public int getFR() {
        return ForR;
    }

    /** @return 铲子舵机当前指令位置（[0,1]） */
    public double getShovelPosition() {
        return targetShovelPos;
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