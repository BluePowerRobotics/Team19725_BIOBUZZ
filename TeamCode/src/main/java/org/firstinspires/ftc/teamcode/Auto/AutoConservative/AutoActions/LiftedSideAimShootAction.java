package org.firstinspires.ftc.teamcode.Auto.AutoConservative.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HiveRoutePlanner;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.List;

/**
 * 【到达发射区后的抬升侧检查】先判断当前所在发射区域是否位于抬升侧，再决定瞄准发射还是改道。
 *
 * <p>本 Action 用于替换状态机 {@code AIM_SHOOT} 阶段（以及周期 Action 末尾）的纯瞄准动作，
 * 在<b>首次执行的一帧</b>按当时的实时位姿与实时球门做出判断：
 *
 * <ol>
 *   <li>用 {@link HiveRouteAction#currentShootAreaIndex} 判断机器人当前所在的发射区域；</li>
 *   <li><b>在抬升侧</b>（该区域最靠近机器人一侧的 x 符号与
 *       {@link RobotPosition#getGoalPose()} 的 x 符号相同）→ 直接瞄准发射；</li>
 *   <li><b>不在抬升侧</b> → 前往<b>另一个</b>发射区域内距机器人最近的可发射点
 *       （航向指向实时球门），到位后再瞄准发射。</li>
 * </ol>
 *
 * <p><b>抬升侧约定</b>与全功能版一致：发射区域 {@code FIRST} 位于 +x 侧、
 * {@code SECOND} 位于 -x 侧；球门 {@code x >= 0} 视为 AudienceUp 侧抬升。
 * 判定时取区域边界上距机器人最近点的 x 符号，与球门 x 符号比较，相同即「在抬升侧」。
 *
 * <p><b>改道路径</b>由 {@link HiveRoutePlanner#waypoints} 生成，{@code throughHive}
 * 与调用方程序原有的路线风格保持一致（绕开 / 经 Hive 正下方）。
 *
 * <p>瞄准发射沿用原有两类动作：{@code chassisAim = false} 用 {@link AutoAimAction}
 * （底盘不动、炮台独立瞄准），{@code true} 用 {@link AutoTurretAimShootWithChassisAction}
 * （底盘辅助转向）。
 */
public class LiftedSideAimShootAction implements Action {

    /** 队伍颜色 */
    private final TeamColor teamColor;
    /** 底盘 */
    private final MecanumDrive drive;
    /** 炮台控制器 */
    private final Turret turret;
    /** 瞄准阶段是否让底盘参与转向 */
    private final boolean chassisAim;
    /** 改道时是否从 Hive 正下方通过 */
    private final boolean throughHive;

    /** 首次 run() 时构建的具体动作 */
    private Action impl;

    /**
     * @param teamColor   队伍颜色
     * @param drive       底盘
     * @param turret      炮台控制器
     * @param chassisAim  瞄准阶段底盘是否参与转向
     * @param throughHive 改道时是否从 Hive 正下方通过
     */
    public LiftedSideAimShootAction(TeamColor teamColor, MecanumDrive drive, Turret turret,
                                    boolean chassisAim, boolean throughHive) {
        this.teamColor = teamColor;
        this.drive = drive;
        this.turret = turret;
        this.chassisAim = chassisAim;
        this.throughHive = throughHive;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (impl == null) {
            impl = build(packet);
        }
        return impl.run(packet);
    }

    /** 首次执行时按实时位姿与实时球门决定「原地瞄准」或「先去另一发射区再瞄准」。 */
    private Action build(TelemetryPacket packet) {
        Pose2d current = RobotPosition.getInstance().getPose2d();
        Pose2d goal = RobotPosition.getInstance().getGoalPose();
        int areaIndex = HiveRouteAction.currentShootAreaIndex(teamColor);
        boolean onLiftedSide = onLiftedSide(areaIndex, goal, current);

        packet.put("LiftedAim area", areaIndex);
        packet.put("LiftedAim onLiftedSide", onLiftedSide);

        if (onLiftedSide) {
            return aim();
        }

        // 不在抬升侧：前往另一个发射区域内距当前位置最近的可发射点
        int otherIndex = (areaIndex == HiveRouteAction.FIRST)
                ? HiveRouteAction.SECOND : HiveRouteAction.FIRST;
        ConvexPolygon area = HiveRouteAction.shootArea(teamColor, otherIndex);
        Vector2D nearest = area.NearestVectorFrom(current.position.x, current.position.y);
        double x = current.position.x + nearest.getX();
        double y = current.position.y + nearest.getY();
        double heading = Math.atan2(goal.position.y - y, goal.position.x - x);
        Pose2d target = new Pose2d(x, y, heading);
        packet.put("LiftedAim divertToArea", otherIndex);

        return new SequentialAction(buildMove(current, target), aim());
    }

    /**
     * 目标发射区域是否位于实时球门所在的一侧（即「在抬升侧」）。
     *
     * @param areaIndex 发射区域序号
     * @param goal      实时球门
     * @param current   当前位姿
     */
    private boolean onLiftedSide(int areaIndex, Pose2d goal, Pose2d current) {
        ConvexPolygon area = HiveRouteAction.shootArea(teamColor, areaIndex);
        Vector2D nearest = area.NearestVectorFrom(current.position.x, current.position.y);
        double areaX = current.position.x + nearest.getX();
        return (areaX > 0) == (goal.position.x > 0);
    }

    /** 按调用方选定的瞄准方式创建瞄准发射动作。 */
    private Action aim() {
        return chassisAim
                ? new AutoTurretAimShootWithChassisAction(drive, turret)
                : new AutoAimAction(drive, turret);
    }

    /** 用 {@link HiveRoutePlanner} 的途经点串成一条以最大轮速为首尾速度的单段轨迹。 */
    private Action buildMove(Pose2d start, Pose2d target) {
        List<Pose2d> waypoints = HiveRoutePlanner.waypoints(teamColor, start, target, throughHive);
        TrajectoryActionBuilder builder = drive.actionBuilder(start, MecanumDrive.PARAMS.maxWheelVel);
        for (Pose2d wp : waypoints) {
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }
        return builder.build();
    }
}
