package org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;

/**
 * 从当前位置出发，<b>经 Hive 正下方</b>前往<b>距离机器人最近的发射区域</b>，
 * 终点为该区域内距离机器人最近的可发射点。
 *
 * <p>与 {@link GoToNearestShootAreaAvoidHiveAction} 的唯一区别是 Hive 通过方式：
 * 本类走 Hive 正下方的内车道，后者走矩形外侧。
 *
 * <p>「最近的发射区域」与目标位姿的判定完全复用
 * {@link GoToNearestShootAreaAvoidHiveAction#nearestShootAreaTargetPose}。
 */
public class GoToNearestShootAreaViaHiveAction extends HiveRouteAction {

    private final int targetAreaIndex;

    /**
     * @param teamColor 队伍颜色
     * @param goal      预瞄准的单个球门目标点
     */
    public GoToNearestShootAreaViaHiveAction(TeamColor teamColor, Pose2d goal) {
        this(teamColor, goal, null);
    }

    /**
     * @param teamColor 队伍颜色
     * @param goal      预瞄准的单个球门目标点
     * @param sweeper   集球器；到达前 {@link HiveRouteAction#shovelLeadMs} 自动放下铲子，可为 null
     */
    public GoToNearestShootAreaViaHiveAction(TeamColor teamColor, Pose2d goal, Sweeper sweeper) {
        super(teamColor, true,
                GoToNearestShootAreaAvoidHiveAction.nearestShootAreaTargetPose(teamColor, goal),
                sweeper);
        this.targetAreaIndex = currentShootAreaIndex(teamColor);
    }

    /** @return 本 Action 前往的发射区域序号 */
    public int targetAreaIndex() {
        return targetAreaIndex;
    }
}
