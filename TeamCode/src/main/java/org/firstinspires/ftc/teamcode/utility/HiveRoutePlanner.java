package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;

import java.util.ArrayList;
import java.util.List;

/**
 * Hive 穿越 / 绕行路线规划。
 *
 * <p>按队伍颜色与「是否从 Hive 正下方通过」，为一段「起点位姿 → 终点位姿」的移动生成一组
 * 中转途经点，使<b>整机外接矩形</b>（{@link ChassisGeometry#footprintPolygon(Pose2d)}，
 * 尺寸由 {@link HypParams#chassisHalfLengthIn} / {@link HypParams#chassisHalfWidthIn} 决定）
 * 满足几何约束：
 *
 * <ul>
 *   <li><b>通过 Hive 正下方</b>（{@code throughHive == true}）：轨迹中途存在整机完全落入
 *       Hive 矩形（{@link HypParams#HiveRectRed} / {@link HypParams#HiveRectBlue}）的时刻；</li>
 *   <li><b>绕开 Hive</b>（{@code throughHive == false}）：中转路线整机始终位于矩形之外；</li>
 *   <li>两者都<b>不得与 Hive 禁线</b>（{@link HypParams#HiveForbiddenLineRedA} 等）
 *       发生任何接触。</li>
 * </ul>
 *
 * <p><b>几何策略</b>：中转途经点统一采用航向 {@link #TRANSIT_HEADING}（面向 +x），使底盘
 * 外接矩形在横/纵车道内轴对齐；车道位置另按「外接矩形对角半长 + 安全间隙」外扩，
 * 从而在航向旋转的段（进/出车道）也远离禁线。
 *
 * <p><b>约束边界说明</b>：
 * <ul>
 *   <li>起点、终点分别位于发射区 / 取球点，可能本身就贴着 Hive 矩形边缘（发射区与矩形相邻），
 *       因此「整机完全位于矩形之外」只对<b>中转途经点及其间的车道段</b>成立，
 *       首段（起点 → 首个途经点）与末段（途经点 → 终点）的位姿由起终点决定；</li>
 *   <li>车道取值已按 {@link HypParams#hiveAvoidClearanceIn} 留出安全间隙，需实机标定。</li>
 * </ul>
 */
public final class HiveRoutePlanner {

    /** 中转途经点统一采用的航向（弧度）：面向 +x，使底盘外接矩形轴对齐 */
    public static final double TRANSIT_HEADING = 0.0;

    private HiveRoutePlanner() {
    }

    /** 队伍对应的 y 轴符号：红队 -1，蓝队 +1。 */
    public static int ySign(TeamColor team) {
        return team == TeamColor.RED ? -1 : 1;
    }

    /** 队伍对应的 Hive 正下方矩形区域。 */
    public static ConvexPolygon hiveRect(TeamColor team) {
        return team == TeamColor.RED ? HypParams.HiveRectRed : HypParams.HiveRectBlue;
    }

    /**
     * 队伍对应的 Hive 禁线两端点。
     *
     * @return 长度为 2 的数组 {@code [A, B]}，表示线段 A-B
     */
    public static Vector2D[] forbiddenLine(TeamColor team) {
        if (team == TeamColor.RED) {
            return new Vector2D[]{HypParams.HiveForbiddenLineRedA, HypParams.HiveForbiddenLineRedB};
        }
        return new Vector2D[]{HypParams.HiveForbiddenLineBlueA, HypParams.HiveForbiddenLineBlueB};
    }

    /** 底盘外接矩形对角半长：任意航向下外接矩形在各轴向外扩的最大半长。 */
    public static double footprintDiagonalHalf() {
        double l = HypParams.chassisHalfLengthIn;
        double w = HypParams.chassisHalfWidthIn;
        return Math.sqrt(l * l + w * w);
    }

    /** Hive 矩形在 x 方向上的半宽（红/蓝均为 24）。 */
    public static double hiveRectHalfWidthX(TeamColor team) {
        double m = 0;
        for (Vector2D v : hiveRect(team).getVertices()) {
            m = Math.max(m, Math.abs(v.getX()));
        }
        return m;
    }

    /** Hive 矩形中离场地中心较远的那条边界的 |y|（红/蓝均为 25）。 */
    public static double hiveRectOuterAbsY(TeamColor team) {
        double m = 0;
        for (Vector2D v : hiveRect(team).getVertices()) {
            m = Math.max(m, Math.abs(v.getY()));
        }
        return m;
    }

    /** Hive 矩形中离场地中心较近的那条边界的 |y|（红/蓝均为 1）。 */
    public static double hiveRectInnerAbsY(TeamColor team) {
        double m = Double.MAX_VALUE;
        for (Vector2D v : hiveRect(team).getVertices()) {
            m = Math.min(m, Math.abs(v.getY()));
        }
        return m;
    }

    /**
     * 沿 Hive 内侧（正下方）通过的横向车道 y：取「禁线 |y|」与「矩形内侧边界 |y|」的中点。
     * 红队 {@code -12.5}，蓝队 {@code +12.5}。
     *
     * <p>在 {@link #TRANSIT_HEADING} 航向下，该车道上的整机外接矩形完全位于矩形内，
     * 且不触及禁线。
     */
    public static double traverseY(TeamColor team) {
        double lineAbs = Math.abs(forbiddenLine(team)[0].getY());
        return ySign(team) * (lineAbs + hiveRectInnerAbsY(team)) / 2.0;
    }

    /**
     * Hive 矩形两侧的纵向车道 x。
     *
     * <p>取值 {@code side * (矩形半宽 + 外接矩形对角半长 + 安全间隙)}：即使航向在
     * 进入 / 离开车道时旋转，外接矩形在该车道上也整体位于禁线之外。
     *
     * @param side x 侧符号：{@code >=0} 取 +1，否则取 -1
     */
    public static double approachX(TeamColor team, int side) {
        int s = (side >= 0) ? 1 : -1;
        return s * (hiveRectHalfWidthX(team) + footprintDiagonalHalf() + HypParams.hiveAvoidClearanceIn);
    }

    /**
     * 绕行 / 离场用的外侧车道 y：{@code ±(矩形外边界|y| + 对角半长 + 安全间隙)}。
     * 红队约 {@code -39.7}，蓝队约 {@code +39.7}。
     *
     * <p>在该车道上，无论航向如何旋转，整机外接矩形都位于矩形之外且不触及禁线。
     */
    public static double outerLaneY(TeamColor team) {
        return ySign(team)
                * (hiveRectOuterAbsY(team) + footprintDiagonalHalf() + HypParams.hiveAvoidClearanceIn);
    }

    /** 给定位姿下，整机外接矩形是否完全位于 Hive 矩形内（含边界）。 */
    public static boolean isFootprintInsideHiveRect(TeamColor team, Pose2d pose) {
        return hiveRect(team).Contains(ChassisGeometry.footprintPolygon(pose));
    }

    /** 给定位姿下，整机外接矩形是否与 Hive 禁线发生接触。 */
    public static boolean isFootprintTouchingForbiddenLine(TeamColor team, Pose2d pose) {
        Vector2D[] line = forbiddenLine(team);
        return ChassisGeometry.footprintPolygon(pose).IntersectsSegment(line[0], line[1]);
    }

    /**
     * 生成中转途经点列表：按顺序依次为各中转点，最后一项为 {@code finalPose}。
     *
     * <p>调用方可直接把返回列表逐点喂给 {@code TrajectoryActionBuilder.strafeToLinearHeading}。
     *
     * @param team       队伍颜色
     * @param start      起点位姿（仅用于判定起步的 x 侧）
     * @param finalPose  终点位姿（含目标航向）
     * @param throughHive true=从 Hive 正下方通过；false=绕开 Hive
     * @return 途经点列表（末项为 {@code finalPose}，已去除相邻重复点）
     */
    public static List<Pose2d> waypoints(TeamColor team, Pose2d start, Pose2d finalPose, boolean throughHive) {
        double hy = traverseY(team);
        double outerY = outerLaneY(team);
        int startSide = start.position.x >= 0 ? 1 : -1;
        int finalSide = finalPose.position.x >= 0 ? 1 : -1;
        double finalHeading = finalPose.heading.toDouble();

        List<Pose2d> wps = new ArrayList<>();
        if (throughHive) {
            // 进入内车道 → 穿过矩形中央（整机完全入内）→ 退出到另一侧内车道 → 转入终点的外侧车道 → 终点
            wps.add(new Pose2d(approachX(team, startSide), hy, TRANSIT_HEADING));
            wps.add(new Pose2d(0.0, hy, TRANSIT_HEADING));
            wps.add(new Pose2d(approachX(team, finalSide), hy, TRANSIT_HEADING));
            wps.add(new Pose2d(approachX(team, finalSide), finalPose.position.y, finalHeading));
        } else {
            // 起步就到外侧车道 → 沿外侧车道绕过矩形 → 终点
            wps.add(new Pose2d(approachX(team, startSide), outerY, TRANSIT_HEADING));
            wps.add(new Pose2d(approachX(team, finalSide), outerY, TRANSIT_HEADING));
            wps.add(new Pose2d(approachX(team, finalSide), finalPose.position.y, finalHeading));
        }
        wps.add(finalPose);
        return dedupConsecutive(wps);
    }

    /** 去除相邻重复（位置相同）的途经点 */
    private static List<Pose2d> dedupConsecutive(List<Pose2d> wps) {
        List<Pose2d> out = new ArrayList<>();
        for (Pose2d p : wps) {
            if (!out.isEmpty() && samePosition(out.get(out.size() - 1), p)) {
                continue;
            }
            out.add(p);
        }
        return out;
    }

    private static boolean samePosition(Pose2d a, Pose2d b) {
        return Math.abs(a.position.x - b.position.x) < 1e-9
                && Math.abs(a.position.y - b.position.y) < 1e-9;
    }
}
