package org.firstinspires.ftc.teamcode.Processors.RobotPosition;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.AdaptiveEKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.Sensors.BallCounter;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.utility.filter.EMA;
//负责除视觉以外的所有状态估计
@Config
public class RobotPosition {
    static MecanumDrive drive;
    HardwareMap hardwareMap;

    /** 融合定位器：Pinpoint 速度 + Limelight 视觉 + 自适应 EKF（权威位姿来源） */
    private AdaptiveEKFLocalizer fusionLocalizer;

    /**
     * HIVE 状态跟踪值：MT1 有观测时更新为观测值，无观测 / 解算被拒时保持上一状态。
     * 初值取 MIDDLE（HIVE 水平），跟踪值不会为 {@link MT1Localizer.HiveState#UNKNOWN}。
     */
    private MT1Localizer.HiveState hiveState = MT1Localizer.HiveState.MIDDLE;

    /** IMU，复用 Road Runner 已初始化的实例（设备名 "imu"），仅用于 resetYaw() */
    private IMU imu;

    // ==================== 球道计数（BallCounter 透传） ====================

    /** 对射光电门在硬件配置中的设备名 */
    private static final String BREAK_BEAM_DEVICE_NAME = "breakBeam";

    /**
     * 球道计数器（基于对射光电门的上升沿检测）。
     * 透传对象：本类只负责每帧调用其 update() / reset()，并把 getCount() 暴露给上层。
     * 若硬件未配置（设备名不存在），此字段为 null，所有球道计数接口安全降级（计数恒为 0）。
     */
    private BallCounter ballCounter;

    /**
     * 球道计数方向（每帧由上层根据 Sweeper 模式设置）：
     * {@link BallCounter#MODE_INTAKE} = 进入模式（球挡光 +1），
     * {@link BallCounter#MODE_OUTPUT} = 出球模式（球挡光 -1）。
     */
    private int ballCounterMode = BallCounter.MODE_INTAKE;

    public Pose2d currentPose;
    public PoseVelocity2d currentVelocity2d;

    private static RobotPosition instance;

    public static RobotPosition getInstance(){
        if(instance==null){
            throw new IllegalStateException("RobotPosition not initialized, call setInstance first");
        }
        return instance;
    }

    /**
     * @return 已初始化的实例；未初始化时返回 null。
     *         供可视化等「可选」功能安全访问，避免 {@link #getInstance()} 抛异常。
     */
    public static RobotPosition getInstanceOrNull(){
        return instance;
    }
    private RobotPosition(){
    }

;
    /**
     * 按红方初始化（Limelight pipeline 0）。
     */
    public static RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose) {
        return RobotPositioninit(hardwareMap, initpose, TeamColor.RED);
    }

    /**
     * @param hardwareMap 硬件映射
     * @param initpose    初始位姿
     * @param teamColor   队伍颜色: 红方加载 Limelight pipeline 0, 蓝方加载 pipeline 1
     */
    public static RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose,
                                                  TeamColor teamColor) {

        instance=new RobotPosition();
        instance.hardwareMap = hardwareMap;

        instance.currentPose = initpose != null ? initpose : new Pose2d(0,0,0);

        // ---- 先构造融合定位器：Pinpoint 速度预测 + Limelight 视觉更新 + 自适应 Q/R ----
        // 队伍颜色决定 Limelight pipeline（红 0 / 蓝 1），由 MT1Localizer 按颜色切换。
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        instance.fusionLocalizer = new AdaptiveEKFLocalizer(
                hardwareMap, limelight, "imu", instance.currentPose, false, teamColor);

        // ---- 把融合定位器注入 RoadRunner，作为 MecanumDrive 唯一的定位器 ----
        // 位姿与速度都直接来自 EKF，不再需要外部 setPose 回写；
        // 定位器每帧仅由 RobotPosition.update() 调用一次 drive.updatePoseEstimate() 推进，
        // 轨迹 / 转向 Action 内部改为复用该帧结果。
        instance.drive = new MecanumDrive(hardwareMap, instance.fusionLocalizer);
        // IMU 复用 Road Runner 已初始化的实例
        instance.imu = instance.drive.lazyImu.get();

        // ---- 球道计数器 ----
        // BallCounter 内部自己创建 BreakBeamSensor，RobotPosition 只传入 HardwareMap + 设备名，
        // 严格遵循"RobotPosition → BallCounter → BreakBeam → 光电门"的层级。
        // 硬件可能未配置（调试用 OpMode），故用 try-catch 降级：
        // 创建失败时 ballCounter 保持 null，球道计数接口返回 0 且不抛异常。
        try {
            instance.ballCounter = new BallCounter(hardwareMap, BREAK_BEAM_DEVICE_NAME);
        } catch (Exception e) {
            instance.ballCounter = null;
        }

        return instance;
    }

    /**
     * 重置机器人位姿。
     * MecanumDrive 的 localizer 就是融合定位器本身，因此一次 setPose
     * 即同时复位 EKF 状态与内部 Pinpoint 里程计。
     *
     * @param pose 目标位姿（真实位姿）
     */
    public void ResetPoseTo(Pose2d pose) {
        drive.localizer.setPose(pose);
        currentPose = pose;
    }

    // 每帧调用：推进融合定位器并返回当前位姿
    public Pose2d update() {
        // 唯一推进点：每帧只调用一次，避免同一帧的视觉观测被重复计入 EKF
        // （MecanumDrive 的轨迹 / 转向 Action 内部已改为复用本帧结果）
        currentVelocity2d = drive.updatePoseEstimate();

        // HIVE 状态跟踪：MT1 成功解出倾角观测时更新，否则保持上一状态
        MT1Localizer mt1 = fusionLocalizer.getMT1();
        if (mt1.isHiveEstimated()) {
            hiveState = mt1.getHiveState();
        }

        currentPose = drive.localizer.getPose();

        // ---- 球道计数（透传）：每帧推进一次 BallCounter ----
        // 方向由 ballCounterMode 决定（上层每帧根据 Sweeper 模式设置）；
        // ballCounter 为 null 时（硬件未配置）直接跳过，不影响位姿估计。
        if (ballCounter != null) {
            ballCounter.update(ballCounterMode);
        }

        return currentPose;
    }

    // ==================== 球道计数（BallCounter 透传接口） ====================

    /**
     * 设置球道计数方向。
     * 应在每帧根据 Sweeper 的当前模式调用：
     *   Sweeper 吃球（intake）→ {@link BallCounter#MODE_INTAKE}（球挡光 +1）
     *   Sweeper 出球（output）→ {@link BallCounter#MODE_OUTPUT}（球挡光 -1）
     *
     * @param mode {@link BallCounter#MODE_INTAKE} 或 {@link BallCounter#MODE_OUTPUT}
     */
    public void setBallCounterMode(int mode) {
        if (mode != BallCounter.MODE_INTAKE && mode != BallCounter.MODE_OUTPUT) {
            throw new IllegalArgumentException(
                    "mode must be MODE_INTAKE (0) or MODE_OUTPUT (1), got: " + mode);
        }
        this.ballCounterMode = mode;
    }

    /**
     * 清空球道计数并复位光电门边缘检测状态。
     * 由上层在以下时机调用：
     *   1. 炮台发射完毕（球已射出，球道清空）；
     *   2. 操作手手动清零。
     * <b>不读取硬件</b>：复用 BallCounter.reset() 的约定，以最近一帧遮挡状态为基准。
     */
    public void resetBallCount() {
        if (ballCounter != null) {
            ballCounter.reset();
        }
    }

    /**
     * @return 当前球道内累计的小球个数（始终 ≥ 0）。
     *         硬件未配置（ballCounter == null）时返回 0。
     */
    public int getBallCount() {
        return (ballCounter != null) ? ballCounter.getCount() : 0;
    }

    /**
     * @return 球道计数器实例，供需要直接访问（如调试遥测）的上层使用；
     *         硬件未配置时返回 null。
     */
    public BallCounter getBallCounter() {
        return ballCounter;
    }


    public Pose2d getPose2d(){        return currentPose;    }

    public double getX(){     return currentPose.position.x;    }
    public double getY(){   return currentPose.position.y;    }
    public double getTheta(){ return currentPose.heading.toDouble();    }
    /** @return 场坐标系 X 轴线速度 (英寸/秒)。输入为体坐标系速度，按 R(θ) 旋转到场坐标系。 */
    public double getVx(){
        double vxBody = currentVelocity2d.linearVel.x;
        double vyBody = currentVelocity2d.linearVel.y;
        double theta = getTheta();
        return vxBody * Math.cos(theta) - vyBody * Math.sin(theta);
    }
    /** @return 场坐标系 Y 轴线速度 (英寸/秒)。输入为体坐标系速度，按 R(θ) 旋转到场坐标系。 */
    public double getVy(){
        double vxBody = currentVelocity2d.linearVel.x;
        double vyBody = currentVelocity2d.linearVel.y;
        double theta = getTheta();
        return vxBody * Math.sin(theta) + vyBody * Math.cos(theta);
    }
    public MecanumDrive getDrive(){return drive;}
    /** @return 自适应 EKF 融合定位器（可读取 MT1 HIVE 倾角/状态观测与 Q、R 调试量） */
    public AdaptiveEKFLocalizer getFusionLocalizer(){return fusionLocalizer;}

    /**
     * @return 当前跟踪的 HIVE 状态 (MIDDLE / AUDIENCE_UP / AUDIENCE_DOWN)。
     *         在 {@link #update()} 中于 MT1 成功解出倾角观测时更新为观测值，
     *         无观测 / 解算被拒时保持上一状态，永不为 UNKNOWN。
     */
    public MT1Localizer.HiveState getHiveState(){return hiveState;}
    public double getOmega(){return currentVelocity2d.angVel;}

    // ---- IMU 功能（原 IMUSensor.java 合并于此） ----

    /**
     * 最近一帧的 IMU 的 yaw/pitch/roll 角。
     * 由 {@code AdaptiveEKFLocalizer.update()} 每帧采样一次后上传，此处不读硬件。
     */
    public YawPitchRollAngles getYawPitchRollAngles() {
        return fusionLocalizer.getImuAngles();
    }

    /**
     * 获取指定单位的 yaw（航向角），取自 {@code AdaptiveEKFLocalizer} 最近一帧的 IMU 采样。
     * @param angleUnit 角度单位
     */
    public double getYaw(AngleUnit angleUnit) {
        return fusionLocalizer.getImuAngles().getYaw(angleUnit);
    }

    /**
     * 重置 IMU yaw 为 0。
     * 注意：Road Runner 定位依赖 IMU yaw 计算航向增量，运行中调用会破坏位姿估计，仅应在初始化/标定时使用。
     */
    public void resetYaw() {
        imu.resetYaw();
    }

}