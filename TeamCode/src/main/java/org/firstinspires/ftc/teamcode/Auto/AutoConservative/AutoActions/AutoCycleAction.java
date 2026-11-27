package org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.InstantAction;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.DeferredAction;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HiveRoutePlanner;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.ArrayList;
import java.util.List;

/**
 * 【保守版】把自动阶段的一个完整周期合成为 <b>一个 Action</b>。
 *
 * <p>周期内容（与现 {@code AutoBlue} 状态机等价的顺序，仍然在取球点停车吃球）：
 * <pre>
 *   前往未访问取球点（绕开 Hive） → 吃球 → 经 Hive 正下方前往指定发射区域 → （可选）自动瞄准
 * </pre>
 *
 * <p><b>targetShootAreaIndex</b> 取值：
 * <ul>
 *   <li>{@code 0}（默认）：用 {@link HiveRouteAction#otherShootAreaTargetPose}，即「另一发射区域」
 *       （与机器人当前所在区域相反，维持原逻辑）；</li>
 *   <li>{@link GoToShootAreaAction#FIRST}（1）：固定去<b>第一发射区域</b>；</li>
 *   <li>{@link GoToShootAreaAction#SECOND}（2）：固定去<b>第二发射区域</b>。</li>
 * </ul>
 */
@Config
public class AutoCycleAction implements Action {

    /** 是否在周期末尾追加一次自动瞄准（原地转向对齐球门） */
    public static boolean includeAim = true;

    /** 队伍颜色 */
    private final TeamColor teamColor;

    /** 集球器 */
    private final Sweeper sweeper;

    /** 炮台控制器（用于自动瞄准与发射） */
    private final Turret turret;

    /** 预瞄准的球门目标点（用于发射区域终点航向，与 {@code Chassis} 自动瞄准一致） */
    private final Pose2d goal;

    /**
     * 目标发射区域序号：
     * 0 = otherShootArea（默认，与当前区域相反）；
     * 1 = FIRST（第一发射区域）；
     * 2 = SECOND（第二发射区域）。
     */
    private final int targetShootAreaIndex;

    /** 首次 run() 时构建的组合 Action */
    private Action impl;

    /**
     * 默认构造：去「另一发射区域」（与当前所在区域相反）。
     */
    public AutoCycleAction(TeamColor teamColor, Sweeper sweeper, Turret turret, Pose2d goal) {
        this(teamColor, sweeper, turret, goal, 0);
    }

    /**
     * 旧 boolean 兼容构造。
     *
     * @param goFirstShootArea true=第一发射区域（FIRST=1），false=另一发射区域（0，默认）
     */
    public AutoCycleAction(TeamColor teamColor, Sweeper sweeper, Turret turret, Pose2d goal,
                           boolean goFirstShootArea) {
        this(teamColor, sweeper, turret, goal, goFirstShootArea ? GoToShootAreaAction.FIRST : 0);
    }

    /**
     * @param teamColor            队伍颜色
     * @param sweeper              集球器控制器
     * @param turret               炮台控制器
     * @param goal                 预瞄准的单个球门目标点
     * @param targetShootAreaIndex 目标发射区域序号：0=otherShootArea，1=FIRST，2=SECOND
     */
    public AutoCycleAction(TeamColor teamColor, Sweeper sweeper, Turret turret, Pose2d goal,
                           int targetShootAreaIndex) {
        this.teamColor = teamColor;
        this.sweeper = sweeper;
        this.turret = turret;
        this.goal = goal;
        this.targetShootAreaIndex = targetShootAreaIndex;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (impl == null) {
            impl = build();
        }
        packet.put("AutoCycle", "Running");
        return impl.run(packet);
    }

    /**
     * 组合周期。每个移动段都延迟到执行时再构造，避免用旧位姿规划。
     */
    private Action build() {
        List<Action> steps = new ArrayList<>();

        steps.add(new DeferredAction(() -> new GoToUnvisitedEatPointAvoidHiveAction(teamColor, sweeper)));
        steps.add(new EatAction(sweeper));
        // 吃球结束、前往发射区域前抬起铲子（已抬起则不改变当前铲子位置）
        steps.add(new InstantAction(() -> sweeper.raiseShovelIfNotUp()));

        // 计算回程目标发射区域
        Pose2d shootTarget;
        switch (targetShootAreaIndex) {
            case GoToShootAreaAction.FIRST:
                shootTarget = shootAreaTargetPose(teamColor, GoToShootAreaAction.FIRST, goal);
                break;
            case GoToShootAreaAction.SECOND:
                shootTarget = shootAreaTargetPose(teamColor, GoToShootAreaAction.SECOND, goal);
                break;
            default:
                shootTarget = HiveRouteAction.otherShootAreaTargetPose(teamColor, goal);
        }
        final Pose2d finalShootTarget = shootTarget;
        steps.add(new DeferredAction(() -> buildLegToShootArea(finalShootTarget)));

        if (includeAim) {
            MecanumDrive drive = RobotPosition.getInstance().getDrive();
            // 到达发射区后先检查抬升侧：不在抬升侧则自动改去另一发射区域再瞄准发射
            steps.add(new LiftedSideAimShootAction(teamColor, drive, turret, false, true));
        }

        return new SequentialAction(steps);
    }

    /** 经 Hive 正下方前往指定目标位姿（航向指向球门）。 */
    private Action buildLegToShootArea(Pose2d target) {
        MecanumDrive drive = RobotPosition.getInstance().getDrive();
        Pose2d start = RobotPosition.getInstance().getPose2d();
        TrajectoryActionBuilder builder = drive.actionBuilder(start, MecanumDrive.PARAMS.maxWheelVel);
        for (Pose2d wp : HiveRoutePlanner.waypoints(teamColor, start, target, true)) {
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }
        return builder.build();
    }

    /** 指定发射区域内距当前位置最近的点，航向指向调用方指定的单个球门。 */
    private static Pose2d shootAreaTargetPose(TeamColor teamColor, int areaIndex, Pose2d goal) {
        ConvexPolygon area = HiveRouteAction.shootArea(teamColor, areaIndex);
        Pose2d current = RobotPosition.getInstance().getPose2d();
        Vector2D nearest = area.NearestVectorFrom(current.position.x, current.position.y);
        double x = current.position.x + nearest.getX();
        double y = current.position.y + nearest.getY();
        double heading = Math.atan2(goal.position.y - y, goal.position.x - x);
        return new Pose2d(x, y, heading);
    }
}
