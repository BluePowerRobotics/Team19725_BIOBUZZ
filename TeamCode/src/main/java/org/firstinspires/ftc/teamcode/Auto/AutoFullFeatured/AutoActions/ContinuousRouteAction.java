package org.firstinspires.ftc.teamcode.Auto.AutoFullFeatured.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.EatPointTracker;
import org.firstinspires.ftc.teamcode.utility.HiveRoutePlanner;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.List;

/**
 * 【激进版】把「前往取球点 + 路过式吃球 + 前往另一发射区域」合成 <b>一个 Action</b>，
 * 且全程<b>只有一条轨迹</b>，途经点之间不减速到零。
 *
 * <p><b>为什么必须是一条 builder 链</b>：
 * {@link MecanumDrive#actionBuilder} 传入 {@code beginEndVel = 0}，而 Road Runner 的
 * {@code profile()} 用同一个值作为起点和终点速度——因此每一份
 * {@code actionBuilder(...).build()} 出来的轨迹<b>必然以 v = 0 结束</b>。
 * 任何 Action 组合器（{@code SequentialAction} / {@code ParallelAction}）都无法消除
 * 两条独立轨迹之间的物理停车。只有把全部途经点串进同一个 {@code TrajectoryActionBuilder}，
 * 才会共用一条时间最优 profile：途经点处按速度/加速度约束通过，不停车。
 *
 * <p><b>吃球怎么做到「不停车」</b>：不再把取球点当终点，而是当<b>途经点</b>。
 * 到达<b>原吸球点</b>时用 {@code afterDisp(0, eatSeq)} 注册标记，Road Runner 会把该 Action
 * 包进 {@code ParallelAction} 与轨迹<b>并行执行</b>——轨迹继续跑，不等待。
 * 随后机器人沿<b>场地边框外法线</b>方向平移 {@link #outwardEatShiftIn} 英寸到<b>现吸球点</b>，
 * 全程边移动边吸球；吸球最长 {@link #maxEatDurationMs}，到时即停（即使未到现吸球点）。
 *
 * <p><b>为什么标记值必须持续多帧</b>：{@code Sweeper.update()} 每帧会把目标速度归零
 * （见 {@code Sweeper#update}），因此一次性 {@code InstantAction} 无效；
 * 这里直接复用 {@code EatAction}（每帧 {@code setEat() + update()}）。
 *
 * <p><b>已知代价 / 实机注意</b>：
 * <ul>
 *   <li>吃球时长由 {@link #eatDurationMs} 控制，应约等于「触发点到路线结束」的行驶时间；
 *       若明显长于剩余轨迹时间，轨迹跑完后 {@code ParallelAction} 仍会等它结束——
 *       车已停在发射区但整段 Action 未结束，会推迟下一周期。</li>
 *   <li>「路过式」要求集球器能在行驶中把球吃进去；若必须静止对准取球点，
 *       请改用保守版 {@link AutoCycleAction}。</li>
 *   <li>{@code startHiveThroughHive} 控制第二段是否走 Hive 正下方（与现主程序一致为 true）。</li>
 * </ul>
 *
 * <p><b>接入方式</b>（不在本类中修改主程序，供 {@code AutoBlue} 后续替换状态机）：
 * <pre>
 * actionRunner.add(new ContinuousRouteAction(teamColor, sweeper, turret, goal));
 * </pre>
 * 注意本类同样在<b>首次 run()</b> 时以当时位姿规划，可直接放入队列。
 */
@Config
public class ContinuousRouteAction implements Action {

    /**
     * 现吸球点相对原吸球点沿场地边框外法线的平移距离 (in)。
     * 原吸球点（intake 对准取球点）作为<b>开始吸球的初始位置</b>，
     * 之后机器人沿垂直边框方向向外移动该距离，边移动边吸球。
     */
    public static double outwardEatShiftIn = 3.0;

    /** 最长吸球时间 (ms)：到时立即停转，即使尚未到达现吸球点 */
    public static long maxEatDurationMs = 1200;

    /** 第二段是否经 Hive 正下方（true=通过，false=绕开） */
    public static boolean secondLegThroughHive = true;

    /** 是否在路线末尾追加自动瞄准 */
    public static boolean includeAim = true;

    /** 队伍颜色 */
    private final TeamColor teamColor;

    /** 集球器 */
    private final Sweeper sweeper;

    /** 炮台控制器（用于自动瞄准与发射） */
    private final Turret turret;

    /** 预瞄准的球门目标点 */
    private final Pose2d goal;

    /** 首次 run() 时构建的组合 Action */
    private Action impl;

    /** 原吸球点对应的底盘中心位姿（开始吸球的位置），{@link #build()} 中写入 */
    private Pose2d eatTargetPose;

    /** 保证「放下铲子」只下发一次 */
    private boolean shovelDropped = false;

    /**
     * @param teamColor 队伍颜色
     * @param sweeper   集球器控制器
     * @param turret    炮台控制器
     * @param goal      预瞄准的单个球门目标点
     */
    public ContinuousRouteAction(TeamColor teamColor, Sweeper sweeper, Turret turret, Pose2d goal) {
        this.teamColor = teamColor;
        this.sweeper = sweeper;
        this.turret = turret;
        this.goal = goal;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (impl == null) {
            impl = build();
        }

        // 到达原吸球点（吸球起始点）前 shovelLeadMs 放下铲子
        if (!shovelDropped && eatTargetPose != null && HiveRouteAction.shovelDue(eatTargetPose)) {
            sweeper.setShovelDown();
            sweeper.update();
            shovelDropped = true;
            packet.put("ContinuousRoute shovel", "Down");
        }

        packet.put("ContinuousRoute", "Running");
        return impl.run(packet);
    }

    /**
     * 以首次执行时的位姿为起点，把两段路线与吃球标记串成一条 builder 链。
     *
     * <p>吸球段：到达<b>原吸球点</b>即启动吸球（并行，不打断轨迹），
     * 随后沿场地边框外法线方向平移 {@link #outwardEatShiftIn} 英寸到<b>现吸球点</b>，
     * 全程边移动边吸；吸球最长持续 {@link #maxEatDurationMs}，到时立即停转，
     * 即使尚未到达现吸球点——因此吸球<b>不需要底盘静止</b>。
     */
    private Action build() {
        Pose2d start = RobotPosition.getInstance().getPose2d();
        MecanumDrive drive = RobotPosition.getInstance().getDrive();

        // 原吸球点（intake 对准原取球点）与另一发射区域
        Pose2d eatTarget = HiveRouteAction.unvisitedEatPointTargetPose(teamColor);
        int eatIndex = HiveRouteAction.unvisitedEatPointIndex(teamColor);
        Pose2d shootTarget = HiveRouteAction.otherShootAreaTargetPose(teamColor, goal);
        eatTargetPose = eatTarget;

        // 现吸球点：原吸球点沿场地边框外法线平移 outwardEatShiftIn 英寸
        Pose2d shiftedTarget = shiftOutward(eatTarget, eatShiftNormal(eatIndex));

        // 两段途经点：第一段绕开 Hive 到原吸球点，第二段从现吸球点经 Hive 正下方（可配）
        List<Pose2d> leg1 = HiveRoutePlanner.waypoints(teamColor, start, eatTarget, false);
        List<Pose2d> leg2 = HiveRoutePlanner.waypoints(
                teamColor, shiftedTarget, shootTarget, secondLegThroughHive);

        TrajectoryActionBuilder builder = drive.actionBuilder(start);

        // ---- 第一段：除末项（= 原吸球点）外全部作为途经点逐个加入，共用一条 profile ----
        for (int i = 0; i < leg1.size() - 1; i++) {
            Pose2d wp = leg1.get(i);
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }

        // ---- 吃球并行段：到达原吸球点时启动，最长 maxEatDurationMs 后停转 ----
        Action eatSeq = new SequentialAction(
                new EatAction(sweeper, maxEatDurationMs),
                new InstantAction(() -> EatPointTracker.markVisited(eatIndex)));

        // ---- 原吸球点：开始吸球的初始位置（作为途经点，不减速）----
        builder = builder
                .strafeToLinearHeading(eatTarget.position, eatTarget.heading.toDouble())
                .afterDisp(0.0, eatSeq);

        // ---- 现吸球点：沿垂直场地边框方向外移，边移动边吸 ----
        builder = builder
                .strafeToLinearHeading(shiftedTarget.position, shiftedTarget.heading.toDouble());

        // ---- 第二段：从现吸球点前往另一发射区域，仍属同一条 builder 链 ----
        for (Pose2d wp : leg2) {
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }

        // ---- 构建：以上全部途经点合成唯一一条轨迹（beginEndVel = 0，只在最末端停车）----
        Action route = builder.build();
        if (includeAim) {
            route = new SequentialAction(route, new AutoAimAction(drive, turret));
        }
        return route;
    }

    /**
     * 取球点所在场地边框的外法线方向：比较 |x| 与 |y|，取更接近边界的那一轴，
     * 符号与该坐标符号一致。例如红队 {@code (24,-72)} → {@code (0,-1)}，
     * {@code (-72,-24)} → {@code (-1,0)}。
     *
     * @param eatIndex 取球点序号（{@link HiveRouteAction#FIRST} / {@link HiveRouteAction#SECOND}）
     */
    private Vector2D eatShiftNormal(int eatIndex) {
        Vector2D p = EatPointTracker.eatPoint(teamColor, eatIndex);
        if (Math.abs(p.getY()) >= Math.abs(p.getX())) {
            return new Vector2D(0.0, Math.signum(p.getY()));
        }
        return new Vector2D(Math.signum(p.getX()), 0.0);
    }

    /**
     * 把底盘中心位姿沿给定方向平移 {@link #outwardEatShiftIn} 英寸，航向保持不变。
     */
    private static Pose2d shiftOutward(Pose2d pose, Vector2D normal) {
        return new Pose2d(
                pose.position.x + outwardEatShiftIn * normal.getX(),
                pose.position.y + outwardEatShiftIn * normal.getY(),
                pose.heading.toDouble());
    }
}
