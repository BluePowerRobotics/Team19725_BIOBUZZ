package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HiveRoutePlanner;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.ArrayList;
import java.util.List;

/**
 * 「前往发射区域」类 Action 的公共行驶内核：把一条既定路径按里程三等分拆成三段，
 * 在<b>两个三等分点</b>处让底盘在行进中把车头指向<b>最近一次有效判定的 Hive 一侧</b>
 * （以 AudienceUp 侧为正例：{@link RobotPosition#getGoalPose()} 在 +x 侧 ⇒ 面向 AudienceUp 面，
 * 否则面向 AudienceDown 面；瞄准点取该面上距机器人最近的点），
 * 并在每次经过检查点时用实时球门（{@link RobotPosition#getGoalPose()}）判断
 * 「是否正在前往抬升侧」：
 *
 * <ul>
 *   <li><b>是</b> → 继续沿原路径以最短时间前往目标发射位置；</li>
 *   <li><b>不是</b> → 立即放弃剩余原路径，以最短时间改道前往<b>另一个发射区域</b>内
 *       距当前位置最近的可发射点（航向指向实时球门）。</li>
 * </ul>
 *
 * <p><b>抬升侧判定</b>：{@code getGoalPose().position.x > 0} 视为场地的 +x 侧抬升
 * （AudienceUp），{@code < 0} 视为 -x 侧抬升（AudienceDown）。把目标发射区域边界上
 * 距机器人最近点的 x 符号与球门 x 符号比较，相同即「在抬升侧」。
 *
 * <p><b>最短时间</b>：每一段都以 {@link MecanumDrive.Params#maxWheelVel} 作为首尾速度
 * （{@code actionBuilder(start, maxWheelVel)}），段间不停车。
 *
 * <p><b>路径来源</b>：第一、二段直接取调用方给定的既定路径（已含 Hive 绕行 / 穿越途经点），
 * 只在三等分点处插入航向检查点，位置不变；第三段为检查点之后的剩余原路径；
 * 仅在改道时才用 {@link HiveRoutePlanner#waypoints} 重新规划一段到另一个发射区域的路线。
 */
public class ShootAreaApproachAction implements Action {

    /** 判定「同一点」的位置容差（英寸） */
    private static final double EPS = 1e-6;

    private final TeamColor teamColor;
    /** 构造时计划前往的发射区域序号（{@link HiveRouteAction#FIRST} / {@link HiveRouteAction#SECOND}） */
    private final int targetAreaIndex;
    /** 改道时是否从 Hive 正下方通过 */
    private final boolean replanThroughHive;

    /** 第一段：起点 → 第一个三等分点 */
    private final List<Pose2d> firstLeg = new ArrayList<>();
    /** 第二段：第一个三等分点 → 第二个三等分点 */
    private final List<Pose2d> secondLeg = new ArrayList<>();
    /** 第三段：第二个三等分点 → 原定终点 */
    private final List<Pose2d> thirdLeg = new ArrayList<>();

    /** 当前执行到第几段（0/1/2） */
    private int legIndex = 0;
    /** 当前正在执行的分段轨迹（懒构造） */
    private Action currentLeg;
    /** 改道后替换第三段的分段轨迹（为 null 表示不改道） */
    private Action divertLeg;
    /** 是否已发出改道 */
    private boolean diverted = false;

    /**
     * @param teamColor         队伍颜色
     * @param startPose         起点位姿（当前位置）
     * @param baseWaypoints     既定路径的途经点，末项为终点位姿（不含起点）
     * @param targetAreaIndex   构造时计划前往的发射区域序号
     * @param replanThroughHive 改道时是否从 Hive 正下方通过
     */
    public ShootAreaApproachAction(TeamColor teamColor, Pose2d startPose, List<Pose2d> baseWaypoints,
                                   int targetAreaIndex, boolean replanThroughHive) {
        this.teamColor = teamColor;
        this.targetAreaIndex = targetAreaIndex;
        this.replanThroughHive = replanThroughHive;
        splitLegs(startPose, baseWaypoints);
    }

    // ==================== 构造期：按里程三等分拆段 ====================

    private void splitLegs(Pose2d startPose, List<Pose2d> baseWaypoints) {
        List<Pose2d> nodes = new ArrayList<>();
        nodes.add(startPose);
        nodes.addAll(baseWaypoints);

        double total = 0;
        for (int i = 0; i + 1 < nodes.size(); i++) {
            total += distance(nodes.get(i), nodes.get(i + 1));
        }
        double firstTarget = total / 3.0;
        double secondTarget = total * 2.0 / 3.0;

        List<Pose2d> current = firstLeg;
        addIfMoved(current, startPose);

        double travelled = 0;
        int nextCheckpoint = 1;
        for (int i = 0; i + 1 < nodes.size(); i++) {
            Pose2d a = nodes.get(i);
            Pose2d b = nodes.get(i + 1);
            double segmentLength = distance(a, b);
            if (segmentLength < EPS) {
                continue;
            }
            while (nextCheckpoint <= 2) {
                double target = (nextCheckpoint == 1) ? firstTarget : secondTarget;
                if (travelled + segmentLength + EPS < target) {
                    break;
                }
                double f = clamp((target - travelled) / segmentLength);
                Pose2d checkpoint = HiveRoutePlanner.hiveSideFacingPose(teamColor,
                        a.position.x + f * (b.position.x - a.position.x),
                        a.position.y + f * (b.position.y - a.position.y),
                        audienceUpSide());
                addIfMoved(current, checkpoint);
                current = (nextCheckpoint == 1) ? secondLeg : thirdLeg;
                addIfMoved(current, checkpoint);
                nextCheckpoint++;
            }
            addIfMoved(current, b);
            travelled += segmentLength;
        }
    }

    // ==================== 运行期：逐段执行 + 检查点判定 ====================

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("ShootApproach leg", legIndex + 1);
        packet.put("ShootApproach diverted", diverted);

        if (currentLeg == null) {
            if (legIndex == 0) {
                currentLeg = buildLeg(firstLeg, false, true);
            } else if (legIndex == 1) {
                currentLeg = buildLeg(secondLeg, true, true);
            } else if (diverted) {
                currentLeg = divertLeg;
            } else {
                currentLeg = buildLeg(thirdLeg, true, false);
            }
            if (currentLeg == null) {
                return false;
            }
        }

        if (currentLeg.run(packet)) {
            return true;
        }

        currentLeg = null;
        if (legIndex == 0) {
            if (onLiftedSide(targetAreaIndex)) {
                legIndex = 1;
            } else {
                divert();
            }
            return true;
        }
        if (legIndex == 1) {
            if (!onLiftedSide(targetAreaIndex)) {
                divert();
            } else {
                legIndex = 2;
            }
            return true;
        }
        return false;
    }

    /** 改道：以最短时间前往另一个发射区域内距当前位置最近的可发射点。 */
    private void divert() {
        int otherIndex = (targetAreaIndex == HiveRouteAction.FIRST)
                ? HiveRouteAction.SECOND : HiveRouteAction.FIRST;
        Pose2d target = nearestShootPose(otherIndex);
        Pose2d current = RobotPosition.getInstance().getPose2d();
        List<Pose2d> waypoints = HiveRoutePlanner.waypoints(teamColor, current, target, replanThroughHive);
        divertLeg = buildLeg(withStart(current, waypoints), false, false);
        diverted = true;
        legIndex = 2;
    }

    /**
     * 目标发射区域是否位于实时球门所在的一侧（即「正在前往抬升侧」）。
     *
     * @param areaIndex 发射区域序号
     */
    private boolean onLiftedSide(int areaIndex) {
        Pose2d current = RobotPosition.getInstance().getPose2d();
        ConvexPolygon area = HiveRouteAction.shootArea(teamColor, areaIndex);
        Vector2D nearest = area.NearestVectorFrom(current.position.x, current.position.y);
        double areaX = current.position.x + nearest.getX();
        Pose2d goal = RobotPosition.getInstance().getGoalPose();
        return (areaX > 0) == (goal.position.x > 0);
    }

    /** 指定发射区域内距当前位置最近的可发射点，航向指向实时球门。 */
    private Pose2d nearestShootPose(int areaIndex) {
        Pose2d current = RobotPosition.getInstance().getPose2d();
        ConvexPolygon area = HiveRouteAction.shootArea(teamColor, areaIndex);
        Vector2D nearest = area.NearestVectorFrom(current.position.x, current.position.y);
        double x = current.position.x + nearest.getX();
        double y = current.position.y + nearest.getY();
        Pose2d goal = RobotPosition.getInstance().getGoalPose();
        return new Pose2d(x, y, Math.atan2(goal.position.y - y, goal.position.x - x));
    }

    // ==================== 工具 ====================

    /**
     * 用某一段的节点列表构建分段轨迹；节点不足或位置未变时返回 {@code null}。
     *
     * @param startsAtCheckpoint 首节点是否为三等分点检查点（是则按最近一次有效判定重算航向）
     * @param endsAtCheckpoint   末节点是否为三等分点检查点（同上）
     */
    private Action buildLeg(List<Pose2d> nodes, boolean startsAtCheckpoint, boolean endsAtCheckpoint) {
        if (nodes.size() < 2) {
            return null;
        }
        List<Pose2d> poses = new ArrayList<>(nodes);
        if (startsAtCheckpoint) {
            poses.set(0, faceHiveSide(poses.get(0)));
        }
        if (endsAtCheckpoint) {
            int last = poses.size() - 1;
            poses.set(last, faceHiveSide(poses.get(last)));
        }
        MecanumDrive drive = RobotPosition.getInstance().getDrive();
        TrajectoryActionBuilder builder =
                drive.actionBuilder(poses.get(0), MecanumDrive.PARAMS.maxWheelVel);
        for (int i = 1; i < poses.size(); i++) {
            Pose2d pose = poses.get(i);
            builder = builder.strafeToLinearHeading(pose.position, pose.heading.toDouble());
        }
        return builder.build();
    }

    /** 把位姿的航向改为「面向最近一次有效判定的 Hive 一侧」。 */
    private Pose2d faceHiveSide(Pose2d pose) {
        return HiveRoutePlanner.hiveSideFacingPose(
                teamColor, pose.position.x, pose.position.y, audienceUpSide());
    }

    /**
     * 最近一次有效判定是否为 AudienceUp 侧（球门在 +x 侧即视为该侧抬升）。
     * <b>以 AudienceUp 侧为正例</b>：{@code x >= 0}（含未判定时的默认值）按 Up 处理。
     */
    private static boolean audienceUpSide() {
        return RobotPosition.getInstance().getGoalPose().position.x >= 0;
    }

    /** 在途经点列表最前面补上起点位姿。 */
    private static List<Pose2d> withStart(Pose2d start, List<Pose2d> waypoints) {
        List<Pose2d> nodes = new ArrayList<>();
        nodes.add(start);
        nodes.addAll(waypoints);
        return nodes;
    }

    /** 仅在位置与上一个节点不同时追加（避免 Road Runner 出现零长度段）。 */
    private static void addIfMoved(List<Pose2d> nodes, Pose2d pose) {
        if (!nodes.isEmpty()) {
            Pose2d last = nodes.get(nodes.size() - 1);
            if (Math.abs(last.position.x - pose.position.x) < EPS
                    && Math.abs(last.position.y - pose.position.y) < EPS) {
                return;
            }
        }
        nodes.add(pose);
    }

    private static double distance(Pose2d a, Pose2d b) {
        return a.position.minus(b.position).norm();
    }

    private static double clamp(double v) {
        return Math.max(0, Math.min(1, v));
    }
}
