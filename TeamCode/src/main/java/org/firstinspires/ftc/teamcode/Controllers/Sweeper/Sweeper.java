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

    /**
     * 球道最大容量：当 RobotPosition.getBallCount() ≥ BallMax 时，自动停止 intake 吸球，
     * 防止球道溢出。出球（OutputVel < 0）不受此限制，允许把球排空。
     * 可在 FTC Dashboard 实时调整。
     */
    public static int BallMax = 4;

    /**
     * 显式"单球出球"的持续时间（毫秒）：从触发到停转，电机以 OutputVel 反转把一颗球送出。
     * 可在 FTC Dashboard 实时调整。时间到后自动停转并恢复光电门计数。
     */
    public static int EjectDurationMs = 260;

    /** 是否正在执行显式单球出球（非阻塞状态机的运行标志） */
    private boolean ejecting = false;
    /** 本次出球的起始时刻（System.nanoTime） */
    private long ejectStartNs = 0L;
    /** 最近一次出球触发的结果提示（供遥测 / 操作手反馈） */
    private String ejectFeedback = "就绪";

    /** 固件速度闭环 PIDF 系数（REV Hub 内置速度 PID，可在 FTC Dashboard 实时调参） */
    public static double VelocityP = 1.17;
    public static double VelocityI = 0.117;
    public static double VelocityD = 0.0;
    public static double VelocityF = 0.0;

    private int targetVelocity = 0;
    private int lastTargetVelocity = 0;

    /**
     * 本帧是否处于"吸球"意图（由 setEat() 设置，唯一判定来源）。
     *
     * 为什么不能用 targetVelocity > 0 代替？
     *   EatVel = 700 和 GiveTheArtifactVel = 2000 都是正速度，但意图完全不同：
     *     - Eat：从场地吸球入球道，满球时必须停转（防止溢出 / 挤压）；
     *     - GiveArtifact：把球道内的球送入炮台发射，球道恰恰需要满球才能打连发，
     *       若因满球阻止 GiveArtifact 会直接导致"无法发射"。
     *   因此满球停转的条件必须走"意图标志"而不是"速度符号"。
     *
     *   setGiveArtifact() / setOutput() / setStop() / setTargetVelocity() 都会把 eating 置 false。
     */
    private boolean eating = false;

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

    public void setEat() {
        targetVelocity = EatVel;
        eating = true;
    }

    public void setGiveArtifact() {
        targetVelocity = GiveTheArtifactVel;
        eating = false;
    }

    public void setOutput() {
        targetVelocity = OutputVel;
        eating = false;
    }

    public void setStop() {
        targetVelocity = 0;
        eating = false;
        // 停止指令同时取消进行中的显式出球，并恢复光电门计数。
        // 球数不回补：触发时已原子扣减，中断后宁可少报也不多报（绝不出现"计数 > 实际球数"）。
        if (ejecting) {
            ejecting = false;
            RobotPosition rp = RobotPosition.getInstanceOrNull();
            if (rp != null) {
                rp.setBallCounterCounting(true);
            }
            ejectFeedback = "出球已中断（球数不回补）";
        }
    }

    public void setTargetVelocity(int velocity) {
        targetVelocity = velocity;
        // 任意直接速度指令都不算"吸球"意图，eating 置 false 交由调用方显式 setEat()
        eating = false;
    }

    /**
     * 显式"单球出球"：触发一次，从球道送出且仅送出一颗球。
     *
     * 原子性保证（对应需求 4）：
     *   1. 方法加 synchronized，配合 {@link RobotPosition#tryConsumeBall()} 内部的
     *      锁，使"检查球数 → 扣减球数 → 启动出球"成为不可分割的整体；
     *   2. 只有扣减成功才真正启动出球，杜绝"已出球却未扣减"；
     *   3. 出球期间冻结光电门计数（setBallCounterCounting(false)），
     *      杜绝球经过光电门时被再扣一次的重复扣减。
     *
     * 边界校验（对应需求 3）：球数为 0（或硬件未配置）时拒绝出球，
     * 并通过 {@link #getEjectFeedback()} 给出提示。
     *
     * 非阻塞：本方法只登记状态，实际电机输出与结束判定在 {@link #update()} 中完成，
     * 不会卡住 OpMode 主循环。
     *
     * @return true = 已受理（球数已 -1，出球进行中）；false = 被拒绝（球数为 0 / 正在出球）
     */
    public synchronized boolean ejectOne() {
        RobotPosition rp = RobotPosition.getInstanceOrNull();
        if (rp == null) {
            ejectFeedback = "无法出球：RobotPosition 未初始化";
            return false;
        }
        if (ejecting) {
            // 上一次出球尚未结束：拒绝重入，避免同一颗球被扣减两次
            ejectFeedback = "出球进行中，请稍候";
            return false;
        }
        // 原子占用一个球：检查与扣减不可分割
        if (!rp.tryConsumeBall()) {
            ejectFeedback = "球道已空（Ball=0），禁止出球";
            return false;
        }
        ejecting = true;
        ejectStartNs = System.nanoTime();
        // 冻结光电门计数，防止球经过光电门时重复扣减
        rp.setBallCounterCounting(false);
        ejectFeedback = "出球中，剩余 " + rp.getBallCount();
        return true;
    }

    /** @return 是否正在执行显式单球出球 */
    public boolean isEjecting() {
        return ejecting;
    }

    /** @return 最近一次出球触发的结果提示（用于操作手反馈 / 遥测） */
    public String getEjectFeedback() {
        return ejectFeedback;
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
        RobotPosition rp = RobotPosition.getInstanceOrNull();

        // ---- 显式单球出球状态机（优先级最高，覆盖其他速度指令） ----
        // 出球时长内保持 OutputVel 反转；到时停转并恢复光电门计数。
        // 球数已在 ejectOne() 触发瞬间原子扣减，故此处只负责"动作"，
        // 保证计数变更与出球动画在同一帧开始、同一帧结束（时序一致）。
        if (ejecting) {
            long elapsedMs = (System.nanoTime() - ejectStartNs) / 1_000_000L;
            if (elapsedMs >= EjectDurationMs) {
                ejecting = false;
                targetVelocity = 0;
                if (rp != null) {
                    rp.setBallCounterCounting(true);   // 恢复光电门计数
                }
            } else {
                targetVelocity = OutputVel;
            }
        } else if (eating && rp != null && rp.getBallCount() >= BallMax) {
            // ---- 球道满自动停止吸球（仅吸球意图生效） ----
            // 意图判定走 eating 标志，不用 targetVelocity > 0——因为 GiveTheArtifactVel（2000）
            // 也是正速度，但"送球入炮台"恰恰需要球道满球才能打连发，满球时绝不能停。
            // RobotPosition 未初始化（如 SweeperTester 单独调试）时跳过，不影响 Sweeper 本身功能。
            targetVelocity = 0;
        }

        // 固件闭环：把目标速度下发给 Hub 内置速度 PID
        motor.setVelocity(targetVelocity);
        lastTargetVelocity = targetVelocity;
        targetVelocity = 0; // 每帧必须重新调用set函数，否则自动归零
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