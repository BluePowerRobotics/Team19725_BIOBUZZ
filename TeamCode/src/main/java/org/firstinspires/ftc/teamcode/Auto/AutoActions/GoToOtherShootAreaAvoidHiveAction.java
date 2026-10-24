package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;

/**
 * 从所在发射区域出发，<b>绕开 Hive</b>（经非 Hive 正下方，沿矩形外侧）前往<b>另一个发射区域</b>。
 *
 * <p>「所在发射区域」按当前位姿自动判断；终点为另一个发射区域内距离机器人最近的点，
 * 航向指向调用方指定的单个球门（与手动阶段自动瞄准一致，精确瞄准由 {@link AutoAimAction} 完成）。
 *
 * <p>「整机位于 Hive 矩形之外」及「不与禁线接触」由 {@link HiveRouteAction} /
 * {@code HiveRoutePlanner} 保证。
 */
public class GoToOtherShootAreaAvoidHiveAction extends HiveRouteAction {

    private final int targetAreaIndex;

    /**
     * @param teamColor 队伍颜色
     * @param goal      预瞄准的单个球门目标点（与手动瞄准传入 Chassis 的目标点一致）
     */
    public GoToOtherShootAreaAvoidHiveAction(TeamColor teamColor, Pose2d goal) {
        super(teamColor, false, otherShootAreaTargetPose(teamColor, goal));
        this.targetAreaIndex = otherShootAreaIndex(teamColor);
    }

    /** @return 本 Action 前往的发射区域序号 */
    public int targetAreaIndex() {
        return targetAreaIndex;
    }
}
