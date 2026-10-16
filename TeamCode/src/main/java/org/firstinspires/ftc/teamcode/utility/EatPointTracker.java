package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;

import java.util.HashSet;
import java.util.Set;

/**
 * 取球点访问状态：跨 Action 共享「本次自动阶段是否已经到过某个取球点」的记账。
 *
 * <p><b>用途</b>：{@code GoToUnvisitedEatPoint*Action} 需要前往「此前未到达过的吸球点」，
 * 因此把已访问的取球点记在这里，由这些 Action 构造时查询、完成时写入。
 *
 * <p><b>生命周期约定</b>
 * <ul>
 *   <li>自动阶段开始（{@code waitForStart()} 之后）调用 {@link #clear()} 清空上一局遗留的记账；</li>
 *   <li>Action 完成时调用 {@link #markVisited(int)} 标记该取球点已访问；</li>
 *   <li>后续 Action 构造时用 {@link #nearestUnvisitedIndex(TeamColor, double, double)} 选择目标。</li>
 * </ul>
 *
 * <p><b>队伍坐标约定</b>：共享取球点 {@link HypParams#EatPoint1} / {@link HypParams#EatPoint2}
 * 位于场地 -y 侧（红队一侧）。红队直接使用这两个点；蓝队取其<b>中心对称点</b>
 * {@code (x,y)->(-x,-y)}（位于 +y 侧，与蓝队发射区同侧）。红蓝两队对同一个取球点序号
 * 共享同一条「已访问」记账（即蓝队到过中心对称点后，该序号对红队也视为已访问）。
 */
public final class EatPointTracker {

    /** 取球点序号：第一个取球点 */
    public static final int FIRST = 1;
    /** 取球点序号：第二个取球点 */
    public static final int SECOND = 2;

    /** 已访问过的取球点序号集合 */
    private static final Set<Integer> visited = new HashSet<>();

    private EatPointTracker() {
    }

    /** 自动阶段开始时清空记账，避免读到上一局的残留。 */
    public static void clear() {
        visited.clear();
    }

    /**
     * 标记某个取球点已访问。
     *
     * @param pointIndex 取球点序号：{@link #FIRST} 或 {@link #SECOND}
     */
    public static void markVisited(int pointIndex) {
        visited.add(pointIndex);
    }

    /**
     * @param pointIndex 取球点序号
     * @return 该取球点是否已访问过
     */
    public static boolean isVisited(int pointIndex) {
        return visited.contains(pointIndex);
    }

    /** 是否两个取球点都已被访问过。 */
    public static boolean allVisited() {
        return visited.contains(FIRST) && visited.contains(SECOND);
    }

    /**
     * 返回某队某个取球点的场地坐标。
     *
     * <p>红队为共享的 {@link HypParams#EatPoint1} / {@link HypParams#EatPoint2}；
     * 蓝队取其中心对称点 {@code (x,y)->(-x,-y)}。
     *
     * @param team       队伍颜色
     * @param pointIndex 取球点序号：{@link #FIRST} 或 {@link #SECOND}
     * @return 取球点场地坐标（英寸）
     */
    public static Vector2D eatPoint(TeamColor team, int pointIndex) {
        Vector2D p = (pointIndex == FIRST) ? HypParams.EatPoint1 : HypParams.EatPoint2;
        if (team == TeamColor.RED) {
            return new Vector2D(p);
        }
        return new Vector2D(-p.getX(), -p.getY());
    }

    /**
     * 选择「此前未到达过」的取球点序号：在未访问的取球点中取距 {@code (x, y)} 最近者；
     * 若两个都已访问，则退回取距 {@code (x, y)} 最近者（保证总有返回）。
     *
     * @param team 队伍颜色（决定取球点坐标是否取中心对称）
     * @param x    当前底盘中心 x（英寸）
     * @param y    当前底盘中心 y（英寸）
     * @return 选中的取球点序号
     */
    public static int nearestUnvisitedIndex(TeamColor team, double x, double y) {
        int bestUnvisited = -1;
        double bestUnvisitedDist = Double.MAX_VALUE;
        int bestAny = FIRST;
        double bestAnyDist = Double.MAX_VALUE;

        for (int idx : new int[]{FIRST, SECOND}) {
            Vector2D p = eatPoint(team, idx);
            double d = Math.hypot(p.getX() - x, p.getY() - y);
            if (d < bestAnyDist) {
                bestAnyDist = d;
                bestAny = idx;
            }
            if (!isVisited(idx) && d < bestUnvisitedDist) {
                bestUnvisitedDist = d;
                bestUnvisited = idx;
            }
        }
        return (bestUnvisited >= 0) ? bestUnvisited : bestAny;
    }
}
