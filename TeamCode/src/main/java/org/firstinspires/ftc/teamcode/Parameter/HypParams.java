package org.firstinspires.ftc.teamcode.Parameter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

@Config
/**
 * 全局超参数配置类
 * 用于集中管理机器人控制系统中的所有超参数，超参应为无需拟合的常量，如工程参数等
 */
public class HypParams {
    /**
     * 底盘最大速度（英寸/秒）
     */
    public static double maxV = 2.0;

    /**
     * 底盘最大角速
     */
    public static double maxOmega = Math.PI;

    /**
     * 底盘半长（英寸）：底盘中心到最后端 / 最前端中点的距离。
     * 用于在「最后端中点坐标」与「底盘中心坐标」之间换算，以及由 intake 目标点反推中心位姿。
     * todo:实机测量标定（取值取自 master 分支 52ad315）
     */
    public static double chassisHalfLengthIn = 9.0;

    /**
     * todo:红队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseRed = new Pose2d(-41.3, 55,0);

    /**
     * todo:蓝队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseBlue = new Pose2d(-41.3, -55, 0);
    /**
     * todo:蓝队自动停车姿态（单位：英寸，弧度）
     */
    public static Pose2d StopPoseBlue = new Pose2d(0, -24, Math.PI);
    /**
     * todo:红队自动停车姿态（单位：英寸，弧度）
     */
    public static Pose2d StopPoseRed = new Pose2d(0, 24, Math.PI);

    /**
     * todo:红队重置姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d ResetPoseRed = new Pose2d(63, -60.7, -Math.PI/2);

    /**
     * todo:蓝队重置姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d ResetPoseBlue = new Pose2d(63, 60.7, Math.PI/2);

    /**
     * 初始操控模式标志
     * true=无头模式（场心地坐标系），false=有头模式（机器人坐标系）
     */
    public static boolean InitialUseNoHeadMode = false;

    /**
     * todo:停车时间阈值（单位：毫秒）
     * 自动阶段剩余时间小于此值时执行停车
     */
    public static long PARK_TIME_THRESHOLD_MS = 3000;

    /**
     * 自动阶段总时长（单位：毫秒）
     */
    public static long AUTONOMOUS_DURATION_MS = 30000;
}