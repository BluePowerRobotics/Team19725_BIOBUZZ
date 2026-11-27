package org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 从当前位置出发，<b>绕开 Hive</b>前往<b>距离机器人最近的发射区域</b>，
 * 终点为该区域内距离机器人最近的可发射点。
 *
 * <p>「最近的发射区域」由 {@link HiveRouteAction#currentShootAreaIndex} 判定：
 * 已在某区域内则取该区域，否则比较到两个区域边界的最近距离取近者。
 *
 * <p>航向指向调用方指定的单个球门（与手动阶段自动瞄准同一公式），
 * 精确瞄准由后续 {@link AutoAimAction} 完成。
 */
public class GoToNearestShootAreaAvoidHiveAction extends HiveRouteAction {

    private final int targetAreaIndex;

    /**
     * @param teamColor 队伍颜色
     * @param goal      预瞄准的单个球门目标点
     */
    public GoToNearestShootAreaAvoidHiveAction(TeamColor teamColor, Pose2d goal) {
        this(teamColor, goal, null);
    }

    /**
     * @param teamColor 队伍颜色
     * @param goal      预瞄准的单个球门目标点
     * @param sweeper   集球器；到达前 {@link HiveRouteAction#shovelLeadMs} 自动放下铲子，可为 null
     */
    public GoToNearestShootAreaAvoidHiveAction(TeamColor teamColor, Pose2d goal, Sweeper sweeper) {
        super(teamColor, false, nearestShootAreaTargetPose(teamColor, goal),
                currentShootAreaIndex(teamColor), sweeper);
        this.targetAreaIndex = currentShootAreaIndex(teamColor);
    }

    /** @return 本 Action 前往的发射区域序号 */
    public int targetAreaIndex() {
        return targetAreaIndex;
    }

    /**
     * 「最近发射区域内离机器人最近的点」位姿，航向指向给定球门。
     *
     * @param teamColor 队伍颜色
     * @param goal      预瞄准的单个球门目标点
     */
    static Pose2d nearestShootAreaTargetPose(TeamColor teamColor, Pose2d goal) {
        ConvexPolygon area = shootArea(teamColor, currentShootAreaIndex(teamColor));
        Pose2d current = RobotPosition.getInstance().getPose2d();
        Vector2D nearest = area.NearestVectorFrom(current.position.x, current.position.y);
        double x = current.position.x + nearest.getX();
        double y = current.position.y + nearest.getY();
        double heading = Math.atan2(goal.position.y - y, goal.position.x - x);
        return new Pose2d(x, y, heading);
    }
}
