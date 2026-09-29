package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;

/**
 * 底盘几何换算工具：在「底盘中心」与「底盘最后端中点 / 前端 intake 目标点」之间进行坐标换算。
 *
 * <p>约定（与 RoadRunner 场地坐标系一致）：
 * <ul>
 *   <li>底盘中心 O = (cx, cy)，朝向 θ（前进方向单位向量 (cosθ, sinθ)）</li>
 *   <li>底盘半长 L = {@link HypParams#chassisHalfLengthIn}（中心到最后端 / 最前端中点的距离）</li>
 *   <li>最后端中点 R = O - L·(cosθ, sinθ)</li>
 *   <li>最前端中点（intake 位置）F = O + L·(cosθ, sinθ)</li>
 * </ul>
 */
public class ChassisGeometry {

    /**
     * 由底盘最后端中点的坐标推算底盘中心坐标。
     * 中心 = 最后端中点 + 半长 · (cosθ, sinθ)。
     *
     * @param rearX   最后端中点 x（英寸）
     * @param rearY   最后端中点 y（英寸）
     * @param heading 机器人朝向（弧度）
     * @return 底盘中心坐标
     */
    public static Vector2D centerFromRearMidpoint(double rearX, double rearY, double heading) {
        double l = HypParams.chassisHalfLengthIn;
        return new Vector2D(rearX + l * Math.cos(heading), rearY + l * Math.sin(heading));
    }

    /**
     * 由底盘中心坐标推算最后端中点坐标。
     * 最后端中点 = 中心 - 半长 · (cosθ, sinθ)。
     */
    public static Vector2D rearMidpointFromCenter(double cx, double cy, double heading) {
        double l = HypParams.chassisHalfLengthIn;
        return new Vector2D(cx - l * Math.cos(heading), cy - l * Math.sin(heading));
    }

    /**
     * 由底盘中心坐标推算最前端中点（intake 安装位置）坐标。
     * 前端中点 = 中心 + 半长 · (cosθ, sinθ)。
     */
    public static Vector2D frontMidpointFromCenter(double cx, double cy, double heading) {
        double l = HypParams.chassisHalfLengthIn;
        return new Vector2D(cx + l * Math.cos(heading), cy + l * Math.sin(heading));
    }

    /**
     * 根据 intake 目标点与机器人当前中心位置，计算机器人应到达的目标底盘中心位姿。
     *
     * <p>目标同时满足：
     * <ol>
     *   <li>前端 intake 恰好到达 {@code (intakeX, intakeY)}；</li>
     *   <li>机器人朝向面朝该取球点。</li>
     * </ol>
     * 以「当前中心 → 取球点」的方向作为逼近航向 θ，则目标中心
     * C = 取球点 - 半长·(cosθ, sinθ)。此时由 C 指向取球点的方向恰为 θ，
     * 因此 intake 到位且朝向正确，二者自洽。
     *
     * @param intakeX   intake 目标点 x（英寸）
     * @param intakeY   intake 目标点 y（英寸）
     * @param currentCx 当前底盘中心 x（用于确定逼近航向）
     * @param currentCy 当前底盘中心 y
     * @return 目标底盘中心位姿（位置 + 朝向）
     */
    public static Pose2d centerPoseFromIntakeTarget(double intakeX, double intakeY,
                                                    double currentCx, double currentCy) {
        double heading = Math.atan2(intakeY - currentCy, intakeX - currentCx);
        double l = HypParams.chassisHalfLengthIn;
        double cx = intakeX - l * Math.cos(heading);
        double cy = intakeY - l * Math.sin(heading);
        return new Pose2d(cx, cy, heading);
    }
}
