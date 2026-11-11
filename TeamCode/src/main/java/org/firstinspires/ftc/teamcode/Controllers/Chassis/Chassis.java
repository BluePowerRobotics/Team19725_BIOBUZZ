 package org.firstinspires.ftc.teamcode.Controllers.Chassis;

 import com.acmerobotics.dashboard.config.Config;
 import com.acmerobotics.roadrunner.Pose2d;
 import com.acmerobotics.roadrunner.PoseVelocity2d;
 import com.acmerobotics.roadrunner.Vector2d;
 import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

 import org.firstinspires.ftc.robotcore.external.Telemetry;
 import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
 import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
 import org.firstinspires.ftc.teamcode.utility.ActionRunner;
 import org.firstinspires.ftc.teamcode.Parameter.HypParams;
 import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
/*
todo：
1. 根据定位与目标位置，实现一键转向目标瞄准
2. 瞄准状态下，一操右摇杆失效，
   有头时，左摇杆前后控制车与球门连线上的前进后退，左摇杆左右控制底盘绕球门旋转（始终指向球门）
   无头时，左摇杆前后控制沿场地的前后平移，左摇杆左右控制沿场地的左右平移，但始终指向球门。
 */
@Config
public class Chassis {

    private final MecanumDrive drive;
    private final ActionRunner actionRunner;
    private boolean useNoHeadMode = HypParams.InitialUseNoHeadMode;
    private final Telemetry telemetry;
    private double lastKx = 0, lastKy = 0, lastKomega = 0;

    /** 刹车舵机在 Robot Configuration 中的设备名 */
    private static final String BRAKE_SERVO_NAME = "brakeServo";

    /** 刹车抬起（不刹车）的舵机位置 [0,1]（todo: 实机标定） */
    public static double BrakeUpPos = 1.0;
    /** 刹车放下（刹车）的舵机位置 [0,1]（todo: 实机标定） */
    public static double BrakeDownPos = 0.0;

    /** 底盘刹车舵机：抬起 = 可自由移动，放下 = 压住地面刹车 */
    private final Servo brakeServo;

    /** 本帧是否请求停止底盘（由 {@link #stop()} 置位，update 消费） */
    private boolean stopRequested;

    /**
     * 本帧是否请求放下刹车（由 {@link #stop(boolean)} 且参数为 true 时置位，update 消费）。
     * 未置位时 update 会把刹车抬起。
     */
    private boolean brakeRequested;

    /** 瞄准模式航向环 P 增益 (rad/s per rad)，Dashboard 可调 */
    public static double aimKp = 3.0;
    /** 瞄准模式角速度指令上限 (rad/s)，防止过冲 */
    public static double aimMaxOmega = Math.PI;

    private final TeamColor teamColor;

    public Chassis(HardwareMap hardwareMap, TeamColor teamColor, ActionRunner actionRunner, Telemetry telemetry, boolean isTeleOp) {
        this.teamColor = teamColor;
        Pose2d initPose;
        if (isTeleOp) {
            initPose = (teamColor == TeamColor.RED) ?
                    HypParams.StopPoseRed : HypParams.StopPoseBlue;
        } else {
            initPose = (teamColor == TeamColor.RED) ?
                    HypParams.startPoseRed : HypParams.startPoseBlue;
        }
        RobotPosition.RobotPositioninit(hardwareMap, initPose, teamColor);
        this.drive = RobotPosition.getInstance().getDrive();
        this.actionRunner = actionRunner;
        this.telemetry = telemetry;
        this.brakeServo = hardwareMap.get(Servo.class, BRAKE_SERVO_NAME);
        // 上电默认抬起刹车，等待上层显式 stop(true) 放下
        this.brakeServo.setPosition(BrakeUpPos);

    }

    public Chassis(HardwareMap hardwareMap, TeamColor teamColor, ActionRunner actionRunner, Telemetry telemetry, Pose2d startPose) {
        this.teamColor = teamColor;
        RobotPosition.RobotPositioninit(hardwareMap, startPose, teamColor);
        this.drive = RobotPosition.getInstance().getDrive();
        this.actionRunner = actionRunner;
        this.telemetry = telemetry;
        this.brakeServo = hardwareMap.get(Servo.class, BRAKE_SERVO_NAME);
        // 上电默认抬起刹车，等待上层显式 stop(true) 放下
        this.brakeServo.setPosition(BrakeUpPos);
    }

    public void setUseNoHeadMode(boolean useNoHeadMode){
        this.useNoHeadMode = useNoHeadMode;
    }
    public void exchangeUseNoHeadMode(){
        useNoHeadMode = !useNoHeadMode;
    }
    public boolean getUseNoHeadMode(){
        return useNoHeadMode;
    }
    /**
     * 请求停止底盘（每帧调用）：只登记停止请求，<b>不操作任何硬件</b>。
     * 零速与刹车舵机的实际下发统一在 {@code update} 内完成。
     */
    public void stop(){
        stopRequested = true;
    }

    /**
     * 请求停止底盘，并可按需放下刹车：只登记请求，<b>不操作任何硬件</b>。
     *
     * <p>刹车是"每帧请求"式语义：只有本帧调用过 {@code stop(true)}，
     * 下一次 {@link #update(double, double, double)} / {@link #update(double, double, Pose2d)}
     * 才会把刹车放下；否则 update 会把刹车抬起。
     * 因此需要持续刹车时应<b>每帧</b>调用 {@code stop(true)}。
     *
     * @param brake true = 本帧停止底盘并放下刹车；false = 只停止底盘
     */
    public void stop(boolean brake){
        stopRequested = true;
        if (brake) {
            brakeRequested = true;
        }
    }

    public void update(double Kx, double Ky, double Komega){
        // 消费本帧的 stop / 刹车请求：stop() 只登记，硬件只在这里下发
        boolean stopped = stopRequested;
        boolean brake = brakeRequested;
        stopRequested = false;
        brakeRequested = false;

        lastKx = Kx;
        lastKy = Ky;
        lastKomega = Komega;
        if(!stopped && !actionRunner.isBusy()){
            // 摇杆 → 底盘速度映射：Ky/Kx 取反以匹配 FTC SDK 手柄惯例（上推为负、右推为正）
            // 官方 SDK: forward = -gamepad1.left_stick_y, strafe = gamepad1.left_stick_x
            // Road Runner: PoseVelocity2d.y 正值 = 向左横移，故 strafe 也需取反
            double forwardVel = -Ky;
            double strafeVel = -Kx;
            double omega = -Komega;
            if(useNoHeadMode){
                // 操作手基础朝向：BLUE 面向 -pi/2 (y-为前), RED 面向 pi/2 (y+为前)
                double driverHeading = (teamColor == TeamColor.RED) ? Math.PI / 2 : -Math.PI / 2;
                // 摇杆输入在操作手主观坐标系中，旋转到场地坐标系后再旋转到机器人坐标系
                // 复合效果等价于用 (theta - driverHeading) 替代原 theta
                double theta = RobotPosition.getInstance().getTheta() - driverHeading;
                double cos = Math.cos(theta);
                double sin = Math.sin(theta);
                // 将操作手坐标系速度旋转到机器人坐标系
                double forwardRobot = forwardVel * cos + strafeVel * sin;
                double strafeRobot = -forwardVel * sin + strafeVel * cos;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardRobot, strafeRobot), omega));
            }
            else{
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardVel, strafeVel), omega));
            }
        } else {
            // 本帧有 stop 请求，或 Action 运行期间手柄输入被屏蔽：主动刹停，
            // 避免电机维持上一帧功率；Action 的实际驱动由 ActionRunner.update() 下发
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
        brakeServo.setPosition(brake ? BrakeDownPos : BrakeUpPos);
    }

    /**
     * 瞄准模式重载：右摇杆失效，航向自动锁定目标，左摇杆只控制平移。
     *
     * <p>目标方位角由定位位姿解算 des = atan2(targetY - y, targetX - x)，
     * 经 P 环 ({@link #aimKp} / {@link #aimMaxOmega}) 输出角速度指令，且始终走最短转向方向。
     *
     * <ul>
     *   <li><b>有头</b>（useNoHeadMode=false）：车头已锁定目标，机器人前方即"车-球门连线"方向，
     *       左摇杆前后 = 沿连线前进后退，左摇杆左右 = 绕球门旋转（沿连线切向）；</li>
     *   <li><b>无头</b>（useNoHeadMode=true）：左摇杆前后/左右 = 沿场地前后/左右平移，
     *       摇杆方向由操作手坐标系旋转到机器人坐标系，同时车头始终指向目标。</li>
     * </ul>
     *
     * @param Kx         左摇杆左右（FTC 惯例：右推为正）
     * @param Ky         左摇杆前后（FTC 惯例：上推为负）
     * @param targetPose 目标位置（仅取 x/y，heading 忽略）
     */
    public void update(double Kx, double Ky, Pose2d targetPose){
        // 消费本帧的 stop / 刹车请求：stop() 只登记，硬件只在这里下发
        boolean stopped = stopRequested;
        boolean brake = brakeRequested;
        stopRequested = false;
        brakeRequested = false;

        lastKx = Kx;
        lastKy = Ky;
        if(!stopped && !actionRunner.isBusy()){
            double forwardVel = -Ky;
            double strafeVel = -Kx;
            // 右摇杆失效：角速度由"始终指向目标"的航向环给出
            double omega = aimOmega(targetPose);
            if(useNoHeadMode){
                // 无头：摇杆输入在操作手/场地坐标系中，旋转到机器人坐标系
                double driverHeading = (teamColor == TeamColor.RED) ? Math.PI / 2 : -Math.PI / 2;
                double theta = RobotPosition.getInstance().getTheta() - driverHeading;
                double cos = Math.cos(theta);
                double sin = Math.sin(theta);
                double forwardRobot = forwardVel * cos + strafeVel * sin;
                double strafeRobot = -forwardVel * sin + strafeVel * cos;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardRobot, strafeRobot), omega));
            }
            else{
                // 有头：车头锁定目标后，机器人前方即指向球门的方向
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardVel, strafeVel), omega));
            }
        } else {
            // 本帧有 stop 请求，或 Action 运行期间手柄输入被屏蔽：主动刹停（同普通重载）
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
        brakeServo.setPosition(brake ? BrakeDownPos : BrakeUpPos);
    }

    /**
     * todo：P控制不够精确，改成PD或PID
     * 解算"始终指向目标"的角速度指令：目标在场地坐标系中的方位角
     * des = atan2(targetY - y, targetX - x)，与当前航向求最短转向误差后做 P 控制并限幅。
     *
     * @param targetPose 目标位置（仅取 x/y）
     * @return 角速度指令 (rad/s，逆时针为正)
     */
    private double aimOmega(Pose2d targetPose){
        RobotPosition robotPosition = RobotPosition.getInstance();
        double desiredHeading = Math.atan2(
                targetPose.position.y - robotPosition.getY(),
                targetPose.position.x - robotPosition.getX());
        // 误差归一化到 [-π, π)，保证始终走最短转向方向
        double error = Math.atan2(
                Math.sin(desiredHeading - robotPosition.getTheta()),
                Math.cos(desiredHeading - robotPosition.getTheta()));
        double omega = aimKp * error;
        return Math.max(-aimMaxOmega, Math.min(aimMaxOmega, omega));
    }

    public void telemetry(){
        /*
        telemetry.addData("X",RobotPosition.getInstance().getX());
        telemetry.addData("Y",RobotPosition.getInstance().getY());
        telemetry.addData("Heading",Math.toDegrees(RobotPosition.getInstance().getTheta()));
        telemetry.addData("useNoHeadMode", useNoHeadMode);
        telemetry.addData("HeadingPID_kP", headingPID.getKP());
        telemetry.addData("HeadingPID_kI", headingPID.getKI());
        telemetry.addData("HeadingPID_kD", headingPID.getKD());
        /*
        telemetry.addData("lfP",drive.leftFront.getPower());
        telemetry.addData("rfP",drive.rightFront.getPower());
        telemetry.addData("lbP",drive.leftBack.getPower());
        telemetry.addData("rbP",drive.rightBack.getPower());
        */
        telemetry.addData("Vx",RobotPosition.getInstance().getVx());
        telemetry.addData("Vy",RobotPosition.getInstance().getVy());
        telemetry.addData("Omega",Math.toDegrees(RobotPosition.getInstance().getOmega()));
        /*
        telemetry.addData("lfV",drive.leftFront.getVelocity());
        telemetry.addData("rfV",drive.rightFront.getVelocity());
        telemetry.addData("lbV",drive.leftBack.getVelocity());
        telemetry.addData("rbV",drive.rightBack.getVelocity());
        
        telemetry.addData("Kx", lastKx);
        telemetry.addData("Ky", lastKy);
        telemetry.addData("Komega", lastKomega);
        */
        
    }
}