package org.firstinspires.ftc.teamcode.Processors.RobotPosition;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public Pose2d currentPose;
    public PoseVelocity2d currentVelocity2d;

    private static RobotPosition instance;

    // ==================== 球道计数（BallChannel，可选） ====================

    /** 本帧的计数方向：{@link BallCounter#MODE_INTAKE}（+1）/ {@link BallCounter#MODE_OUTPUT}（-1） */
    private int ballMode = BallCounter.MODE_INTAKE;

    /** 球道计数器：首次用到球道功能时才创建（避免未配置光电门的机器人初始化失败） */
    private BallCounter ballCounter;

    /** 是否启用定位部分：球道专用实例为 false（不碰 Limelight / 里程计） */
    private boolean localizationEnabled = false;

    /**
     * 只用于球道计数的轻量实例：不启动 Limelight / IMU / 里程计，仅提供球道相关接口。
     * 供纯球道调试的 OpMode 使用；正式比赛请用 {@link #RobotPositioninit}。
     *
     * @param hardwareMap 硬件映射
     * @return 未注册为单例的独立实例
     */
    public static RobotPosition initBallChannelOnly(HardwareMap hardwareMap) {
        RobotPosition position = new RobotPosition();
        position.hardwareMap = hardwareMap;
        position.currentPose = new Pose2d(0, 0, 0);
        // 速度置零而不是留 null：避免球道专用实例误调 getVx()/getVy()/getOmega() 时 NPE
        position.currentVelocity2d = new PoseVelocity2d(new Vector2d(0, 0), 0);
        return position;
    }

    /** 懒创建球道计数器（先创建下属 BallCounter，其内部再创建 BreakBeamSensor） */
    private BallCounter ballCounter() {
        if (ballCounter == null) {
            ballCounter = new BallCounter(hardwareMap);
        }
        return ballCounter;
    }

    /**
     * 提前创建球道计数器：在 OpMode 的 init 阶段调用，使「光电门未配置」在 init 就报错，
     * 而不是等到主循环第一帧（比赛中才开始崩）。
     */
    public void ensureBallChannel() {
        ballCounter();
    }

    /** 设置本帧计数方向（{@link BallCounter#MODE_INTAKE} / {@link BallCounter#MODE_OUTPUT}） */
    public void setBallMode(int mode) {
        ballMode = mode;
    }

    /** @return IsBlocked：光电门当前是否遮挡 */
    public boolean isBlocked() {
        return ballCounter != null && ballCounter.isBlocked();
    }

    /** @return BallCount：当前累计球数（未启用球道时为 0） */
    public int getBallCount() {
        return ballCounter == null ? 0 : ballCounter.getCount();
    }

    /** resetBallCount：只清计数，不动光电门滤波与边缘基准（炮台发射完毕 / 手动轻量清空） */
    public void resetBallCount() {
        if (ballCounter != null) {
            ballCounter.clearCount();
        }
    }

    /** 深复位：计数归零，同时重置光电门 EMA 滤波与边缘基准 */
    public void resetBallChannel() {
        if (ballCounter != null) {
            ballCounter.reset();
        }
    }

    /** 输出球道状态遥测（调用方需自行 telemetry.update()） */
    public void setBallTelemetry(Telemetry telemetry) {
        telemetry.addData("Ball count", getBallCount());
        telemetry.addData("Ball blocked", isBlocked() ? 1 : 0);
    }

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
        instance.localizationEnabled = true;

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
        // reset 边由本类发起：位姿重定位时同步深复位球道计数（图中 RobotPosition → BallCounter）
        resetBallChannel();
    }

    // 每帧调用：推进融合定位器与球道计数，返回当前位姿
    public Pose2d update() {
        // 唯一推进点：每帧只调用一次，避免同一帧的视觉观测被重复计入 EKF
        // （MecanumDrive 的轨迹 / 转向 Action 内部已改为复用本帧结果）
        // 球道专用实例（initBallChannelOnly）没有 drive，跳过定位部分
        if (localizationEnabled && drive != null) {
            currentVelocity2d = drive.updatePoseEstimate();

            // HIVE 状态跟踪：MT1 成功解出倾角观测时更新，否则保持上一状态
            MT1Localizer mt1 = fusionLocalizer.getMT1();
            if (mt1.isHiveEstimated()) {
                hiveState = mt1.getHiveState();
            }

            currentPose = drive.localizer.getPose();
        }

        // 球道计数：与定位共用同一条「更新」边（OpMode → RobotPosition → BallCounter）
        if (ballCounter != null) {
            ballCounter.update(ballMode);
        }
        return currentPose;
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