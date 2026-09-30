package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.PID.PIDSVAController;
import org.firstinspires.ftc.teamcode.utility.PID.SlotConfig;

/**
 * 单个发射器控制器：飞轮速度闭环 + 俯仰舵机 + 扳机舵机。
 *
 * <p>两个发射器（大球 / 小球）机械参数不同，因此超参数封装在
 * {@link Params} 中按实例传入（{@link #BIG_PARAMS} / {@link #SMALL_PARAMS}），
 * 而不是用静态裸字段——否则两个实例会共用同一份参数。
 *
 * <p><b>单位约定</b>：
 * <ul>
 *   <li>飞轮速度：{@link DcMotorEx#getVelocity()} 的默认单位 tick/s</li>
 *   <li>俯仰与扳机：舵机位置 [0, 1]（舵机空间，不一定是真实发射角度）</li>
 * </ul>
 *
 * <p><b>就绪判据</b>（{@link #update(int, double, boolean)} 的返回值）要求
 * 飞轮转速与俯仰舵机同时到位。俯仰舵机无位置反馈，因此以"目标变更后经过
 * {@link Params#pitchSettleTime} 秒"作为其到位的近似判据。
 */
@Config
public class Shooter {

    // ==================== 超参数 ====================

    /**
     * 单个发射器的机械超参数。大球与小球各持一份实例，故各字段为实例字段。
     * 全部数值均为占位默认值，必须实机标定。
     */
    public static class Params {
        // ---- 飞轮速度环 PIDSVA ----
        /** 速度环 P 增益 */
        public double kP = 0.0004;
        /** 速度环 I 增益 */
        public double kI = 0.0002;
        /** 速度环 D 增益 */
        public double kD = 0.0;
        /** 积分上限（防积分饱和） */
        public double maxI = 1.0;
        /** I 分离阈值 (tick/s)：误差小于该值才累加积分 */
        public double iZone = 3000.0;
        /** 静态摩擦前馈系数 */
        public double kS = 0.0;
        /** 速度前馈系数，量纲为 (功率)/(tick/s) */
        public double kV = 0.0002;
        /** 加速度前馈系数（当前按 0 加速度使用） */
        public double kA = 0.0;
        /** 输出下限（功率） */
        public double outputMin = -1.0;
        /** 输出上限（功率） */
        public double outputMax = 1.0;

        // ---- 机械范围 ----
        /** 俯仰舵机最小位置 */
        public double pitchMin = 0.0;
        /** 俯仰舵机最大位置 */
        public double pitchMax = 1.0;
        /** 飞轮最大转速 (tick/s)，目标速度会被钳制到 [0, speedMax] */
        public int speedMax = 3000;

        // ---- 扳机 ----
        /** 扳机释放（发射）时的舵机位置 */
        public double triggerOnPos = 1.0;
        /** 扳机关闭时的舵机位置 */
        public double triggerOffPos = 0.0;

        // ---- 装配方向 ----
        /** 飞轮电机方向：由装配方向决定，反装时置 true */
        public boolean motorReversed = false;

        // ---- 就绪判据 ----
        /** 转速到位容差 (tick/s) */
        public double speedTolerance = 100.0;
        /** 俯仰舵机动作时间 (秒)：目标变更后经过该时间即视为到位 */
        public double pitchSettleTime = 0.25;
    }

    /** 大球发射器超参数（Dashboard 可调） */
    public static Params BIG_PARAMS = new Params();

    /** 小球发射器超参数（Dashboard 可调） */
    public static Params SMALL_PARAMS = new Params();

    // ==================== 运行时状态 ====================

    public final DcMotorEx motor;
    public final Servo pitchServo;
    public final Servo triggerServo;

    private final Params params;
    private final Telemetry telemetry;
    /** 复用同一个 slot：每帧用 withXxx 刷新参数后 resetSlot（见 SlotConfig 类注释） */
    private final PIDSVAController controller = new PIDSVAController();
    private final SlotConfig slot = new SlotConfig();

    /** 上一次 update 的时间戳 (秒) */
    private double lastTime;

    /** 最近一次下发的目标转速 (tick/s，钳制后) */
    private int targetSpeed = 0;
    /** 最近一次下发的俯仰舵机位置（钳制后） */
    private double targetPitch = 0;
    /** 最近一次下发的扳机状态：true = 释放（发射） */
    private boolean triggerReleased = false;

    /** 上一次的俯仰目标，用于检测目标变更 */
    private double lastPitchCommand = Double.NaN;
    /** 俯仰目标最近一次变更的时刻 (秒) */
    private double pitchCommandTime = 0;

    /** 最近一次下发的功率（供遥测） */
    private double power = 0;

    /**
     * @param hardwareMap     硬件映射
     * @param telemetry       遥测
     * @param motorName       飞轮电机设备名
     * @param pitchServoName  俯仰舵机设备名
     * @param triggerServoName 扳机舵机设备名
     * @param params          该发射器的超参数（大球传 {@link #BIG_PARAMS}，小球传 {@link #SMALL_PARAMS}）
     */
    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, String motorName,
                   String pitchServoName, String triggerServoName, Params params) {
        this.telemetry = telemetry;
        this.params = params;
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.pitchServo = hardwareMap.get(Servo.class, pitchServoName);
        this.triggerServo = hardwareMap.get(Servo.class, triggerServoName);

        motor.setDirection(params.motorReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        // 软件速度闭环：关闭固件速度环，让 setPower 直接控制功率
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // 飞轮惯性大，停转后让其自然滑行
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controller.withSlot0(new SlotConfig()
                .withKP(params.kP).withKI(params.kI).withKD(params.kD)
                .withMaxI(params.maxI).withIZone(params.iZone)
                .withKS(params.kS).withKV(params.kV).withKA(params.kA)
                .withOutputLimits(params.outputMin, params.outputMax));

        lastTime = now();
        pitchCommandTime = lastTime;
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    /**
     * 每帧调用一次：把目标钳制到可用范围后下发舵机，并对飞轮执行速度闭环。
     *
     * @param speed   目标飞轮转速 (tick/s)，钳制到 [0, {@link Params#speedMax}]
     * @param pitch   目标俯仰舵机位置，钳制到 [{@link Params#pitchMin}, {@link Params#pitchMax}]
     * @param release 扳机是否释放（true = 释放/发射）
     * @return 飞轮转速与俯仰机构是否均已到位（即当前是否具备发射条件）
     */
    public boolean update(int speed, double pitch, boolean release) {
        double currentTime = now();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        // 首帧或长时间停顿后 dt 异常，用标称循环周期兜底，避免微分项发散
        if (dt <= 0 || dt > 1.0) {
            dt = 0.02;
        }

        // 钳制到可用范围
        targetSpeed = Math.max(0, Math.min(params.speedMax, speed));
        targetPitch = Math.max(params.pitchMin, Math.min(params.pitchMax, pitch));
        triggerReleased = release;

        // 舵机下发
        pitchServo.setPosition(targetPitch);
        triggerServo.setPosition(release ? params.triggerOnPos : params.triggerOffPos);

        // 俯仰目标变更时重新计时，用于近似判断舵机是否到位
        if (Double.isNaN(lastPitchCommand) || Math.abs(targetPitch - lastPitchCommand) > 1e-6) {
            lastPitchCommand = targetPitch;
            pitchCommandTime = currentTime;
        }

        // 飞轮速度闭环
        double currentVelocity = motor.getVelocity();

        // 每帧刷新参数，使 Dashboard 上的改动即时生效
        slot.withKP(params.kP).withKI(params.kI).withKD(params.kD)
                .withMaxI(params.maxI).withIZone(params.iZone)
                .withKS(params.kS).withKV(params.kV).withKA(params.kA)
                .withOutputLimits(params.outputMin, params.outputMax);
        controller.resetSlot(slot);

        // VelCycle = true：目标转速作为前馈速度项，由 kS/kV 换算成基础功率
        power = controller.calculate(targetSpeed, currentVelocity, dt, true);
        motor.setPower(power);

        return isReady();
    }

    /**
     * 停止发射：飞轮目标归零、扳机复位。
     * 俯仰舵机保持当前位置不变（不改动 {@code targetPitch}）。
     */
    public void stop() {
        targetSpeed = 0;
        triggerReleased = false;
        triggerServo.setPosition(params.triggerOffPos);
        motor.setPower(0);
        power = 0;
        controller.reset();
    }

    // ==================== 读取 ====================

    /**
     * @return 飞轮转速与俯仰机构是否均已到位。飞轮需在容差内达到非零目标转速
     *         （目标为 0 时视为不可发射），俯仰舵机以动作时间近似判断。
     */
    public boolean isReady() {
        boolean speedReady = targetSpeed > 0
                && Math.abs(targetSpeed - motor.getVelocity()) < params.speedTolerance;
        boolean pitchReady = now() - pitchCommandTime >= params.pitchSettleTime;
        return speedReady && pitchReady;
    }

    /** @return 当前飞轮转速 (tick/s) */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /** @return 最近一次下发的目标转速 (tick/s) */
    public int getTargetSpeed() {
        return targetSpeed;
    }

    /** @return 最近一次下发的俯仰舵机位置 */
    public double getTargetPitch() {
        return targetPitch;
    }

    /** @return 最近一次下发的扳机状态 */
    public boolean isTriggerReleased() {
        return triggerReleased;
    }

    /** @return 最近一次下发的功率 */
    public double getPower() {
        return power;
    }

    /** 输出遥测（调用方需自行 telemetry.update()） */
    public void setTelemetry() {
        telemetry.addData("Shooter speed target (tick/s)", targetSpeed);
        telemetry.addData("Shooter speed current (tick/s)", "%.0f", motor.getVelocity());
        telemetry.addData("Shooter power", "%.3f", power);
        telemetry.addData("Shooter pitch", "%.3f", targetPitch);
        telemetry.addData("Shooter trigger released", triggerReleased);
        telemetry.addData("Shooter ready", isReady());
    }
}