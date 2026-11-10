package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;

/**
 * 集球器（自包含）—— 对应架构图中的「Sweeper」节点。
 *
 * <p>本类自包含实现「固件速度闭环 + 铲子舵机」，并在其上叠加架构图的跨层同步逻辑：<b>满仓停吸</b>
 * ——每帧下发速度前，从 {@link RobotPosition} 读 {@code BallCount}，满仓（≥ {@link #FULL_COUNT}）且
 * 本帧意图为吸球时强制归零；给工件（{@link #setGiveArtifact()}）与出球（{@link #setOutput()}）不受限制。
 *
 * <p><b>数据同步方向</b>：Sweeper →（只读）→ RobotPosition.getBallCount()。
 * 不传 {@link RobotPosition} 的构造方式（两参数）用于不关心球道计数的旧调用方，此时不做满仓保护。
 *
 * <p><b>注意</b>：{@code update()} 每帧会把 targetVelocity 归零，因此上层必须每帧重新调用
 * {@link #setEat()} / {@link #setOutput()} / {@link #setStop()} 设置意图。
 */
@Config
public class Sweeper {

    /** 集球电机（固件速度闭环） */
    private final DcMotorEx motor;
    /** 铲子升降舵机 */
    private final Servo shovelServo;
    /** 只读依赖：状态估计器（取球道满仓状态）；为 null 时不做满仓保护 */
    private final RobotPosition robotPosition;
    private final Telemetry telemetry;

    // ==================== 超参数（Dashboard 实时生效） ====================

    /** 吸球转速 (tick/s) */
    public static int EatVel = 700;
    /** 给工件转速 (tick/s) */
    public static int GiveTheArtifactVel = 2000;
    /** 倒球转速 (tick/s，负值) */
    public static int OutputVel = -500;

    /** 球道满仓球数：BallCount 达到该值即停止吸球（Dashboard 可调） */
    public static int FULL_COUNT = 4;

    /** 固件速度闭环 PIDF 系数 */
    public static double VelocityP = 1.17;
    public static double VelocityI = 0.117;
    public static double VelocityD = 0.0;
    public static double VelocityF = 0.0;

    /** 电机方向开关：0 = REVERSE，1 = FORWARD */
    public static int ForR = 0;

    /** 铲子抬起时的舵机位置 [0,1]（todo: 实机标定） */
    public static double ShovelUpPos = 0.5;
    /** 铲子放下时的舵机位置 [0,1]（todo: 实机标定） */
    public static double ShovelDownPos = 0.0;

    // ==================== 运行时状态 ====================

    private int targetVelocity = 0;
    private int lastTargetVelocity = 0;

    /** 本帧是否为「吸球」意图：只有吸球才受满仓停吸保护，给工件/出球不受限制 */
    private boolean eatIntent = false;

    private double cachedPower = 0;
    private double cachedVelocity = 0;
    private double cachedCurrent = 0;

    /** 当前铲子舵机目标位置（[0,1]） */
    private double targetShovelPos = ShovelDownPos;

    /** 不关心球道计数的调用方：不做满仓停吸保护 */
    public Sweeper(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null);
    }

    public Sweeper(HardwareMap hardwareMap, Telemetry telemetry, RobotPosition robotPosition) {
        this.telemetry = telemetry;
        this.robotPosition = robotPosition;
        this.motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        this.shovelServo = hardwareMap.get(Servo.class, "shovelServo");

        setDirection();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(VelocityP, VelocityI, VelocityD, VelocityF);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setShovelUp();
        shovelServo.setPosition(targetShovelPos);
    }

    private void setDirection() {
        switch (ForR) {
            case 0:
                motor.setDirection(DcMotor.Direction.REVERSE);
                break;
            case 1:
                motor.setDirection(DcMotor.Direction.FORWARD);
                break;
            default:
                throw new IllegalArgumentException("ForR must be 0 (REVERSE) or 1 (FORWARD), got: " + ForR);
        }
    }

    /** 吸球意图（intake，正速度，计数方向 +1）；满仓时会被 {@link #update()} 拦截 */
    public void setEat() {
        targetVelocity = EatVel;
        eatIntent = true;
    }

    /** 给工件意图（正向送球，不受满仓停吸保护） */
    public void setGiveArtifact() {
        targetVelocity = GiveTheArtifactVel;
        eatIntent = false;
    }

    /** 倒球意图（output，负速度，计数方向 -1） */
    public void setOutput() {
        targetVelocity = OutputVel;
        eatIntent = false;
    }

    /** 停止意图 */
    public void setStop() {
        targetVelocity = 0;
        eatIntent = false;
    }

    /** 直接设置目标速度（tick/s），按正负判断是否需要满仓保护 */
    public void setTargetVelocity(int velocity) {
        targetVelocity = velocity;
        eatIntent = velocity > 0;
    }

    /** 铲子抬起 */
    public void setShovelUp() {
        targetShovelPos = ShovelUpPos;
    }

    /** 铲子放下 */
    public void setShovelDown() {
        targetShovelPos = ShovelDownPos;
    }

    /**
     * 每帧调用：先施加满仓停吸保护，再下发速度与舵机。
     */
    public void update() {
        // 满仓停吸：只有「吸球」意图且 BallCount 达到 FULL_COUNT 时强制停吸；
        // 给工件 / 出球方向不受限制
        if (eatIntent && robotPosition != null && robotPosition.getBallCount() >= FULL_COUNT) {
            targetVelocity = 0;
        }

        motor.setVelocity(targetVelocity);
        lastTargetVelocity = targetVelocity;
        targetVelocity = 0; // 每帧必须重新调用 set 函数，否则自动归零
        eatIntent = false;

        shovelServo.setPosition(targetShovelPos);

        cachedPower = motor.getPower();
        cachedVelocity = motor.getVelocity();
        cachedCurrent = motor.getCurrent(CurrentUnit.AMPS);
    }

    /** @return 本帧实际下发的目标速度（满仓保护后），供遥测 */
    public int getTargetVelocity() {
        return lastTargetVelocity;
    }

    public double getPower() {
        return cachedPower;
    }

    public double getVel() {
        return cachedVelocity;
    }

    public double getCurrent() {
        return cachedCurrent;
    }

    public int getFR() {
        return ForR;
    }

    public double getShovelPosition() {
        return targetShovelPos;
    }

    /** 输出遥测（只读本帧缓存；调用方需自行 telemetry.update()） */
    public void setTelemetry() {
        if (telemetry == null) {
            return;
        }
        telemetry.addData("Sweeper target vel", lastTargetVelocity);
        telemetry.addData("Sweeper power", "%.3f", cachedPower);
        telemetry.addData("Sweeper vel", "%.0f", cachedVelocity);
        telemetry.addData("Sweeper current", "%.2f", cachedCurrent);
        telemetry.addData("Shovel pos", "%.3f", targetShovelPos);
    }

    /** 停止集球器（退出时调用） */
    public void stop() {
        setStop();
        update();
    }
}
