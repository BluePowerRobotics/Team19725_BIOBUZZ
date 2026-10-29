package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.utility.EatPointTracker;

/**
 * 从所在发射区域出发，<b>绕开 Hive</b>（经非 Hive 正下方，沿矩形外侧）前往此前未到达过的吸球点。
 *
 * <p>终点为 {@link EatPointTracker#nearestUnvisitedIndex} 选出的未访问取球点；
 * 红队使用共享取球点，蓝队使用其中心对称点（位于蓝队 +y 一侧）。
 * 到达后由 {@link #onComplete()} 把该取球点标记为已访问。
 *
 * <p>「整机位于 Hive 矩形之外」及「不与禁线接触」由 {@link HiveRouteAction} /
 * {@code HiveRoutePlanner} 保证。
 */
public class GoToUnvisitedEatPointAvoidHiveAction extends HiveRouteAction {

    private final int pointIndex;

    /**
     * @param teamColor 队伍颜色
     */
    public GoToUnvisitedEatPointAvoidHiveAction(TeamColor teamColor) {
        super(teamColor, false, unvisitedEatPointTargetPose(teamColor));
        this.pointIndex = unvisitedEatPointIndex(teamColor);
    }

    @Override
    protected void onComplete() {
        EatPointTracker.markVisited(pointIndex);
    }

    /** @return 本 Action 前往的取球点序号 */
    public int pointIndex() {
        return pointIndex;
    }
}
