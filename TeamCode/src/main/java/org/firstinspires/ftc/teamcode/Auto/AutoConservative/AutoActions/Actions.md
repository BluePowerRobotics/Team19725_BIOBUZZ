# AutoActions 使用说明

> **本目录是 `Auto/AutoFullFeatured/AutoActions/` 的保守备份（`AutoConservative`）**：
> 其中**不含**「前往发射区域途中在路径三等分点检查抬升侧、必要时改道」的逻辑，
> 所有「前往发射区域」的移动 Action 均为**单段轨迹直达终点**。
> 原目录 `Auto/AutoFullFeatured/AutoActions/` 的代码不受影响。

本目录（`Auto/AutoConservative/AutoActions/`）汇总自动阶段可入队执行的 Road Runner `Action`。
所有移动类 Action 的位姿来源统一为 **`AdaptiveEKFLocalizer` 融合定位系统**
（Pinpoint 里程计 + Limelight MT1 视觉的自适应 EKF 融合位姿）：

- 构造时通过 `RobotPosition.getInstance().getPose2d()` 取起点位姿（轨迹在构造时刻固化）；
- 执行时轨迹跟随内部持续调用融合定位器推进；
- `AutoAimAction` 每帧直接读 `AdaptiveEKFLocalizer.getPose()`；
- 组合类 Action（`AutoCycleAction` / `ContinuousRouteAction`）在**首次 `run()` 时**才构造
  `SequentialAction` 并规划轨迹（`AutoCycleAction` 的移动段经 `utility/DeferredAction` 包装），
  保证仍以**执行时的真实位姿**规划。

## 通用约定

1. **串行队列执行**：由 `utility/ActionRunner`（`add` / `update` / `isBusy` / `clear`）调度，
   OpMode 主循环中每帧调用 `actionRunner.update()`。
2. **非阻塞**：每个 Action 的 `run(packet)` 每帧执行一次，返回 `true` 表示未完成，
   返回 `false` 表示结束（队列自动切换下一个 Action）。
3. **构造时刻固化**：轨迹类 Action 应在**入队前、机器人位置基本确定后**再 new，
   不要提前构造后跨多个状态复用。
4. **Dashboard 调参**：带 `@Config` 的 Action，其 `public static` 参数可在 FTC Dashboard
   运行时实时修改，无需重新部署。
5. **单位**：坐标英寸，角度弧度（`strafeToLinearHeading`），时间毫秒。
6. **时间最优**：所有移动类 Action 都必须以最短时间走完整条路径——
   起步冲到最大速度、中段巡航、末端最快停止（梯形速度曲线）。
   实现方式：**一段移动只用一个 `actionBuilder` 链**，中间点作为链式途经点
   （段间速度连续、不中途停车）；约束来源为 `MecanumDrive.Params` 五参数。
   原理、标定与验证方法见
   [`RoadRunner/时间最优运动.md`](../../../RoadRunner/时间最优运动.md)。
7. **多段组合**（`SequentialAction` / `ParallelAction`）：可把一个完整周期合成单个 Action 入队，
   并用 `utility/DeferredAction` 包装移动段，使其仍在**执行时**才构造（避免用旧位姿规划）。
   但需注意：`MecanumDrive.actionBuilder` 的 `beginEndVel = 0`，Road Runner 的 `profile()`
   用同一个值同时作为**起点与终点速度**，因此**每一份 `build()` 出来的轨迹必然以 v=0 结束**——
   任何 Action 组合器都无法消除两条独立轨迹之间的物理停车。
   「周期组合」见第 13 / 14 节——两者都是「到点停车吃球」，区别只在投递粒度。
8. **铲子提前放下**：前往取球点的移动 Action（`GoToUnvisitedEatPoint*` / `GoToEatPointAction` /
   `GoToNearestEatPointAction` / `ContinuousRouteAction`）接收 `Sweeper`，每帧用
   「剩余直线距离 ÷ 线速度」估算 ETA，当 ETA ≤ `HiveRouteAction.shovelLeadMs`
   （默认 1000 ms）时立即下发 `setShovelDown()`，保证到位时铲子已经落下；
   `EatAction` 首次 `run()` 再兜底一次。不传 `Sweeper` 的重载保持旧行为（不提前放铲）。

## Action 一览

| Action | 作用 | 构造参数 |
|--------|------|---------|
| `GoToStopPose` | 前往停车位姿并停车 | `(MecanumDrive drive, Pose2d stopPose)` |
| `GoToShootAreaAction` | **保守版单段直达轨迹**：抵达指定发射区域内离机器人最近的点，车头预瞄指定球门；**不经过**全功能版的 `ShootAreaApproachAction` 三等分点检查与抬升侧改道 | `(TeamColor, int areaIndex, Pose2d goal)` 或 `(TeamColor, areaIndex, goal, throughHive)`（4 参传是否经 Hive 正下方） |
| `GoToShootAreaNearestVerticesAction` | 单段连续轨迹依次经过两个发射区域各自距原点最近的顶点 | `(TeamColor, Pose2d goal)` |
| `GoToEatPointAction` | 到指定取球点（intake 到位、车头朝取球点） | `(int pointIndex)` 或 `(int, Sweeper)` |
| `GoToNearestEatPointAction` | 自动选近的取球点并前往 | 无 或 `(Sweeper)` |
| `GoToUnvisitedEatPointViaHiveAction` | 经 Hive 正下方前往此前未访问的取球点 | `(TeamColor)` 或 `(TeamColor, Sweeper)` |
| `GoToOtherShootAreaViaHiveAction` | 经 Hive 正下方前往另一发射区域并预瞄球门 | `(TeamColor, Pose2d goal)` |
| `GoToUnvisitedEatPointAvoidHiveAction` | 绕开 Hive（外侧）前往此前未访问的取球点 | `(TeamColor)` 或 `(TeamColor, Sweeper)` |
| `GoToOtherShootAreaAvoidHiveAction` | 绕开 Hive（外侧）前往另一发射区域并预瞄球门 | `(TeamColor, Pose2d goal)` |
| `GoToNearestShootAreaAvoidHiveAction` | 绕开 Hive（外侧）前往**最近**发射区域内离机器人最近的可发射点 | `(TeamColor, Pose2d goal)` 或 `(TeamColor, Pose2d goal, Sweeper)` |
| `GoToNearestShootAreaViaHiveAction` | 经 Hive 正下方前往**最近**发射区域内离机器人最近的可发射点 | `(TeamColor, Pose2d goal)` 或 `(TeamColor, Pose2d goal, Sweeper)` |
| `AutoAimAction` | 底盘不动，炮台自动瞄准并发射（Turret 自动扣扳机） | `(MecanumDrive, Turret)` |
| `AutoTurretAimShootWithChassisAction` | 底盘主动转向球门辅助瞄准，炮台自动瞄准并发射 | `(MecanumDrive, Turret)` |
| `LiftedSideAimShootAction` | 到达发射区后先判断抬升侧：在抬升侧则瞄准发射；否则先去另一发射区域最近点再瞄准发射 | `(TeamColor, MecanumDrive, Turret, boolean chassisAim, boolean throughHive)` |
| `EatAction` | 底盘原地不动（每帧驱动归零），集球器吃球（默认 2.5 s）后停转 | `(Sweeper)` 或 `(Sweeper, long durationMs)` |
| `AdaptiveParkAction` | T-3s 检测整机是否接触场地边框，接触则开到安全位置停靠，否则原地停止 | `(MecanumDrive, ElapsedTime matchTime, long durationMs, long leadTimeMs)` |
| `AutoCycleAction` | **保守版**：把「去取球点 → 吃球 → 去指定发射区 →（可选）瞄准」合成一个 Action（取球点仍停车）；**单段直达**，无三等分点检查 | `(TeamColor, Sweeper, Turret, Pose2d goal)` 或 `(…, int targetShootAreaIndex)`（0=otherShootArea /默认；1=FIRST；2=SECOND）；旧 boolean `goFirstShootArea` 兼容构造保留（true→1，false→0） |
| `ContinuousRouteAction` | **合成版**：轨迹1（起点→取球点，绕开 Hive）→ 停车吃球 → 轨迹2（取球点→指定发射区，单段直达无三等分点检查）→（可选）瞄准；**两段轨迹之间停车** | `(TeamColor, Sweeper, Turret, Pose2d goal)` 或 `(…, int targetShootAreaIndex)`（同 AutoCycleAction）；旧 boolean 兼容构造保留 |

> 保守备份中，所有 **前往发射区域** 的移动 Action（`GoToShootAreaAction`、`GoToShootAreaNearestVerticesAction`、
> `GoToOtherShootArea{Avoid,Via}HiveAction`、`GoToNearestShootArea{Avoid,Via}HiveAction`）
> 均为**单段轨迹直达终点**，不经过三等分点检查；原 `ShootAreaApproachAction` 不包含在本备份中。

---

## 1. GoToStopPose

**作用**：用 `strafeToLinearHeading` 从当前融合位姿一次性平移旋转到停车位姿。

**构造**：`new GoToStopPose(drive, HypParams.StopPoseRed)`
（蓝队用 `HypParams.StopPoseBlue`）

**完成条件**：Road Runner 轨迹执行完毕。

## 2. GoToShootAreaAction

**作用**：
1. 按队伍颜色与序号选择发射区域（`ConvexPolygon`）：
   `FIRST=1` / `SECOND=2`，非法序号抛 `IllegalArgumentException`；
2. 用 `ConvexPolygon.NearestVectorFrom` 求多边形上离机器人最近的点作为目标位置；
3. 航向指向调用方传入的**单个具体球门**（与手动阶段
   `Chassis#update(double, double, Pose2d)` / `Chassis#aimOmega` 完全一致）；
4. `strafeToLinearHeading` 到位。精确瞄准由后续 `AutoAimAction` 完成。

**构造**：
```java
new GoToShootAreaAction(TeamColor.RED,
                        GoToShootAreaAction.FIRST,
                        HypParams.RedAudienceUp);
```

## 3. GoToShootAreaNearestVerticesAction

**作用**：
1. 计算机器人到本颜色两个发射区域边界的最近距离，确定最近区域与另一区域；
2. 分别取两个多边形中距场地原点最近的顶点，航向指向传入的具体球门；
3. 在**同一个** `actionBuilder` 链上用两次 `strafeToLinearHeading` 构建**单段连续轨迹**：
   起点 → 最近区域顶点 → 另一区域顶点；两个顶点均为途经点，机器人在巡航中依次通过
   （**不中途停车**），以最短时间完成整条路径。

**构造**：
```java
new GoToShootAreaNearestVerticesAction(TeamColor.RED, HypParams.RedAudienceUp);
```

## 4. GoToEatPointAction

**作用**：取球点为底盘前端 intake 的目标点（`HypParams.EatPoint1=(24,-72)`、
`EatPoint2=(-72,-24)`，场地固定坐标不分红蓝）。构造时：
1. 用 `ChassisGeometry.centerPoseFromIntakeTarget` 由 intake 目标点反推底盘中心位姿
   ——前端 intake 恰好落在取球点，且车头面朝取球点；
2. `strafeToLinearHeading` 到位。到位后通常入队 `EatAction`。

**铲子**：传入 `Sweeper` 时，到达终点前 `HiveRouteAction.shovelLeadMs`（默认 1000 ms）
自动 `setShovelDown()`，保证到位即可吸球（详见第 15 节）。

**构造**：`new GoToEatPointAction(GoToEatPointAction.FIRST)`（或 `SECOND`，非法序号抛异常）；
推荐 `new GoToEatPointAction(GoToEatPointAction.FIRST, sweeper)`。

## 5. GoToNearestEatPointAction

**作用**：构造时计算底盘中心到 `EatPoint1` / `EatPoint2` 的直线距离，自动选近者，
其余同 `GoToEatPointAction`。

**构造**：`new GoToNearestEatPointAction()`；推荐 `new GoToNearestEatPointAction(sweeper)`。

## Hive 路线几何约束（`GoTo*ViaHive` / `GoTo*AvoidHive` 共用）

以下 4 个 Action 均继承 `HiveRouteAction` 基类，由 `utility/HiveRoutePlanner` 生成途经点，
保证对外接矩形（整机轮廓）的几何约束：

- **Hive 正下方矩形**：红队 x∈[-24,24]、y∈[-25,-1]（`HypParams.HiveRectRed`）；
  蓝队为**中心对称**区域 x∈[-24,24]、y∈[1,25]（`HypParams.HiveRectBlue`）。
- **Hive 禁线**：红队为 (24,-24)–(-24,-24) 两点间线段（`HypParams.HiveForbiddenLineRedA/B`）；
  蓝队为 (24,24)–(-24,24)（`HypParams.HiveForbiddenLineBlueA/B`）。
- **整机轮廓**：底盘外接矩形，半长 / 半宽为 `HypParams.chassisHalfLengthIn` /
  `chassisHalfWidthIn`，由 `ChassisGeometry.footprintPolygon` 生成。
- **通过 Hive 正下方**（`throughHive=true`）：轨迹中途存在整机外接矩形**完全落入矩形内**
  的时刻（`HiveRoutePlanner.isFootprintInsideHiveRect`）。
- **绕开 Hive**（`throughHive=false`）：中转路线整机始终位于矩形之外。

两者都**不得与 Hive 禁线接触**（`HiveRoutePlanner.isFootprintTouchingForbiddenLine`）。

**规划策略**：中转途经点航向统一为 0（面向 +x），使外接矩形轴对齐；内车道
`y=±12.5`（禁线与矩形内侧边界的中点），纵向车道 `x=±(24 + 对角半长 + 间隙)`、
外侧车道 `y=±(25 + 对角半长 + 间隙)`（对角半长 = √(半长²+半宽²)，间隙
`HypParams.hiveAvoidClearanceIn`）。

**约束边界**：起点、终点分别位于发射区域 / 取球点，可能本身就贴着矩形边缘
（发射区域与矩形相邻），因此「整机完全位于矩形之外」仅对**中转途经点及其间的车道段**
成立。车道取值与间隙需**实机标定**。

**队伍对称**：蓝队全部几何为中心对称——取球点取 `(x,y)->(-x,-y)`、矩形与禁线为红队的
中心对称、车道 y 取反、纵向车道 x 相同。

## 6. GoToUnvisitedEatPointViaHiveAction

**作用**：从机器人所在发射区域出发，**经 Hive 正下方**前往此前未到达过的吸球点。
1. 「未访问取球点」由 `utility/EatPointTracker` 共享状态决定：在未访问的取球点中选距
   机器人最近者（两个都已访问时退回取最近者）；红队用共享取球点，蓝队用其中心对称点；
2. 由 `ChassisGeometry.centerPoseFromIntakeTarget` 反推使 intake 到位、车头朝取球点的中心位姿；
3. 途经点由 `HiveRoutePlanner.waypoints(..., throughHive=true)` 生成；
4. 到达后 `onComplete()` 调用 `EatPointTracker.markVisited(pointIndex)`。

**构造**：`new GoToUnvisitedEatPointViaHiveAction(TeamColor.RED)`；
推荐 `new GoToUnvisitedEatPointViaHiveAction(TeamColor.RED, sweeper)`（传 `Sweeper` 才会提前放铲）。
**访问记账**：由 `AutoRed` / `AutoBlue` 在自动阶段开始时调用 `EatPointTracker.clear()` 清空。

## 7. GoToOtherShootAreaViaHiveAction

**作用**：从机器人所在发射区域出发，**经 Hive 正下方**前往**另一个发射区域**。
1. 「所在发射区域」按当前位姿自动判断（在位姿内取该区域；否则取边界最近者）；
2. 终点为另一发射区域内距机器人最近的点，航向指向调用方指定的单个球门
   （与手动阶段自动瞄准一致，精确瞄准由后续 `AutoAimAction` 完成）；
3. 途经点由 `HiveRoutePlanner.waypoints(..., throughHive=true)` 生成。

**构造**：`new GoToOtherShootAreaViaHiveAction(TeamColor.RED, HypParams.RedAudienceUp)`。

## 8. GoToUnvisitedEatPointAvoidHiveAction

**作用**：从机器人所在发射区域出发，**绕开 Hive**（经非 Hive 正下方，沿矩形外侧）前往
此前未到达过的吸球点。终点选择同 `GoToUnvisitedEatPointViaHiveAction`，路径由
`HiveRoutePlanner.waypoints(..., throughHive=false)` 生成（沿外侧车道 `y=±39.7` 绕过矩形）。
**构造**：`new GoToUnvisitedEatPointAvoidHiveAction(TeamColor.RED)`；
推荐 `new GoToUnvisitedEatPointAvoidHiveAction(TeamColor.RED, sweeper)`（传 `Sweeper` 才会提前放铲）。

## 9. GoToOtherShootAreaAvoidHiveAction

**作用**：从机器人所在发射区域出发，**绕开 Hive**（外侧）前往**另一个发射区域**，其余同
`GoToOtherShootAreaViaHiveAction`，仅路径模式不同。
**构造**：`new GoToOtherShootAreaAvoidHiveAction(TeamColor.RED, HypParams.RedAudienceUp)`。

## 9.5 GoToNearestShootAreaAvoidHiveAction / GoToNearestShootAreaViaHiveAction

**作用**：前往**距离机器人最近的发射区域**，终点为该区域内距机器人最近的可发射点，
航向指向调用方指定的单个球门（精确瞄准仍由后续 `AutoAimAction` 完成）。

1. 「最近的发射区域」由 `HiveRouteAction.currentShootAreaIndex` 判定：机器人已在某区域内则取该区域，
   否则比较到两个区域边界的最近距离、取近者——等价于 `GoToShootAreaAction` 里 `areaIndex` 的自动版；
2. 终点 = `shootArea.NearestVectorFrom(当前位姿)` 得到的边界最近点（已在区域内时即为原地）；
3. 两个类的唯一区别是 Hive 通行方式：
   - `GoToNearestShootAreaAvoidHiveAction` → `throughHive = false`，沿矩形外侧车道绕行；
   - `GoToNearestShootAreaViaHiveAction` → `throughHive = true`，从 Hive 正下方内车道通过。

**构造**：`new GoToNearestShootAreaAvoidHiveAction(TeamColor.RED, goal)` /
`new GoToNearestShootAreaViaHiveAction(TeamColor.RED, goal)`；
可选第三参 `Sweeper`（传了才会提前放铲）。

> ⚠️ 与 `GoToOtherShootArea*Action` 的区别：后者去的是**另一个**（通常是更远的）发射区，
> 本组去的是**最近的**发射区。

## 10. AutoAimAction（底盘不动 · 炮台自动瞄准与发射）

**作用**：底盘全程保持静止（`setDrivePowers(0,0,0)`），由 {@link Turret} 独立完成
**平转 + 俯仰 + 飞轮加速 + 自动扣扳机**的完整瞄准与发射流程。

**瞄准与发射主体**：`Turret.update(allowFire, preload, false, false)`
- 内部从 `RobotPosition.getGoalPose()` 读取球门（已由 HIVE 抬升侧自动选择 Up/Down）；
- 平转到位 + 飞轮到位 + 俯仰到位时，由 `TriggerGate` 自动扣扳机并保持 `fireHoldTime`；
- 返回 `FireReady(big, small)` 表示两个发射器是否就绪。

**完成条件**（任一满足即返回 `false`）：
- 大球或小球扳机从「释放」回到「关闭」（`isBigFiring` / `isSmallFiring` 的**下降沿**）—— 一次发射完成；
- 超过 `timeoutMs` 毫秒。

结束时自动 `turret.stop()`（停飞轮、关扳机），底盘保持 0 速度。

> ⚠️ 本 Action **不再判断底盘航向是否对准**——发射权完全交给 Turret 的就绪判据，
> 只要炮台到位且机构就绪就会发射。超时与成功都返回 `false`，遥测 `AutoAim = Timeout / Fired` 可区分。

**Dashboard 可调参数**：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `allowFire` | `true` | 是否允许 Turret 扣扳机（`false` 时只预载不发射，便于单独调试瞄准） |
| `preload` | `true` | 是否启用预载（飞轮按预载转速待命，缩短响应） |
| `timeoutMs` | 5000 | 超时强制结束 (ms) |

**构造**：`new AutoAimAction(drive, turret)`。

## 10.5 AutoTurretAimShootWithChassisAction（底盘旋转辅助瞄准与发射）

**作用**：与 {@link AutoAimAction} 的区别在于**底盘主动转向球门**，使炮台需要旋转的角度最小，
缩短炮台到达目标位置的总时间。

**控制分工**：
- **底盘**：PD 控制角速度，使车头朝向球门
  `desiredHeading = atan2(goalY-y, goalX-x)`，误差归一化到 `[-π, π]`，死区内输出 0；
- **炮台**：`Turret.update(...)` 自动瞄准并发射（同 {@link AutoAimAction}）。

底盘转向把目标拉到炮台行程中心附近，炮台只需补偿剩余小角度，总响应最快。
本 Action 不判断底盘是否已对准——发射权同样交给 Turret。

**完成条件**：同 {@link AutoAimAction}（发射完成下降沿或超时）。

**Dashboard 可调参数**：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `allowFire` | `true` | 是否允许扣扳机 |
| `preload` | `true` | 是否启用预载 |
| `yawKp` | 3.0 | 底盘转向环 P (rad/s per rad) |
| `yawKd` | 0.15 | 底盘转向环 D |
| `maxOmega` | π | 底盘角速度上限 (rad/s) |
| `headingTol` | 1.5° | 底盘转向死区 (rad) |
| `timeoutMs` | 6000 | 超时强制结束 (ms) |

**构造**：`new AutoTurretAimShootWithChassisAction(drive, turret)`。

## 10.7 LiftedSideAimShootAction（抬升侧检查后再瞄准发射）

**作用**：替换状态机 `AIM_SHOOT` 阶段与周期 Action 末尾的纯瞄准动作。首次 `run()` 时按
**当时的实时位姿与实时球门**判断：

1. `HiveRouteAction.currentShootAreaIndex` 得出机器人所在发射区域（`FIRST` / `SECOND`）；
2. **在抬升侧**（该区域边界上距机器人最近点的 x 符号与 `RobotPosition.getGoalPose().position.x`
   符号相同）→ 直接执行瞄准发射；
3. **不在抬升侧** → 用 `HiveRoutePlanner.waypoints` 规划一段到**另一个**发射区域内
   距当前位置最近可发射点的轨迹（航向指向实时球门），到位后再瞄准发射。

**构造**：`new LiftedSideAimShootAction(teamColor, drive, turret, chassisAim, throughHive)`；
`chassisAim = false` 用 `AutoAimAction`（底盘不动），`true` 用
`AutoTurretAimShootWithChassisAction`（底盘辅助转向）；`throughHive` 与所在程序的
Hive 通行风格保持一致（8 个 OpMode 中 `*ViaHive` 传 `true`，其余传 `false`）。

遥测键：`LiftedAim area`（所在区域）、`LiftedAim onLiftedSide`（是否在抬升侧）、
`LiftedAim divertToArea`（改道目标区域，仅在改道时出现）。

## 11. EatAction

**作用**：**底盘原地不动**——每帧先
`drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))` 把驱动归零，
再 `sweeper.setEat()` + `sweeper.update()`（Sweeper 每帧自动归零目标，
必须每帧重新置位），持续到时后 `setStop()` 并结束。

**铲子**：首次 `run()` **无条件** `sweeper.setShovelDown()`——EatAction 必传 `Sweeper`，
因此吃球开始时铲子一定处于放下位置。

**Dashboard 可调参数**：`eatDurationMs`（默认 2500 ms = 2.5 s，可实时调整）；
吃球速度 `Sweeper.EatVel`（默认 700 tick/s）在 `Sweeper` 类中调整。

**构造**：
```java
Sweeper sweeper = new Sweeper(hardwareMap, telemetry);
new EatAction(sweeper);                       // 用 eatDurationMs 默认时长（2.5 s）
new EatAction(sweeper, 1500);                 // 指定吃球时长
```

## 12. AdaptiveParkAction（自适应停靠）

**作用**：自动阶段结束前 **3 秒**（T-3s）检测整机外接矩形是否接触场地边框，并据此决定
**本局停靠点**。停靠点由当时位姿实时解算，**不是固定点**。

**场地边界**：12 ft × 12 ft 场地以原点为中心，边界为 `|x| = |y| = 72 in`
（`HypParams.fieldHalfSizeIn`）。

**判定与动作**：

| 检测结果 | 动作 | 停靠点 |
|---------|------|--------|
| 四角任一到达/越过 ±72 in（接触） | 沿 `strafeToLinearHeading` 轨迹开到最小平移解 | 解算出的安全位姿（动态） |
| 四角全部在边界内（不接触） | `setDrivePowers(0,0,0)` 原地停止 | 检测时刻的当前位姿 |

**最小平移解**（`safeParkPose`）：设底盘半长 / 半宽为 `L` / `W`，航向 `θ`，求旋转矩形的
轴对齐半投影

```
ex = |L·cosθ| + |W·sinθ|
ey = |L·sinθ| + |W·cosθ|
```

再把中心夹到 `[-limit+ex, limit-ex]`（y 同理），其中
`limit = fieldHalfSizeIn - parkBorderMarginIn`。这样得到的**轴对齐包围盒**完全落在安全区内，
且位移最小、**不改变航向**。

**执行时序**：

1. 未到 T-3s：每帧保持电机停止并返回 `true`（可提前入队，作为队列最后一个元素）；
2. 到 T-3s：检测一次，记录 `borderContact` 与 `parkPose`；
3. 接触 → 建轨迹并跟随；不接触 → 停车并立刻返回 `false`；
4. 轨迹结束后再次停车并返回 `false`。

**构造**（与 OpMode 共用同一个比赛计时器）：

```java
ElapsedTime matchTime = new ElapsedTime();   // START 后创建

actionRunner.clear();
actionRunner.add(new AdaptiveParkAction(
        drive, matchTime,
        HypParams.AUTONOMOUS_DURATION_MS,
        HypParams.PARK_TIME_THRESHOLD_MS));
```

**参数说明**（均在 `Parameter/HypParams.java`，`@Config` 可在 Dashboard 实时调整）：

| 参数 | 默认值 | 单位 | 含义 |
|------|--------|------|------|
| `AUTONOMOUS_DURATION_MS` | 30000 | ms | 自动阶段总时长，作为倒计时基准 |
| `PARK_TIME_THRESHOLD_MS` | 3000 | ms | 停靠触发阈值（剩余时间 ≤ 该值即检测） |
| `fieldHalfSizeIn` | 72.0 | in | 场地半边长（12 ft × 12 ft → ±72） |
| `parkBorderMarginIn` | 3.0 | in | 停靠时整机外接盒与边框保留的安全间隙 |
| `chassisHalfLengthIn` / `chassisHalfWidthIn` | 见 HypParams | in | 整机轮廓半长 / 半宽，用于外接矩形 |

**遥测键**（FTC Dashboard）：

| 键 | 含义 |
|----|------|
| `AdaptivePark` | 阶段文字（`waiting until T-3s` / `no border contact -> stop` / `border contact -> moving inside`） |
| `AdaptivePark remaining(ms)` | 等待阶段的剩余时间 |
| `AdaptivePark park x` / `park y` / `park heading(deg)` | 本局停靠点 |

**对外查询**：`borderContact()`、`parkPose()`。

**注意**：

- 位姿来自融合定位，本 Action **不推进定位器**（由 OpMode 每帧唯一一次 `update()`）。
- 判据依赖定位精度；若实车贴边仍有压线风险，调大 `parkBorderMarginIn`（如 4~5 in）。
- 极端情况下（外接盒大于安全区）退回场地中心 `(0,0)`，航向不变。

## 13. AutoCycleAction（保守版：周期组合）

**作用**：把自动阶段的一个完整周期合成为**一个** Action 入队：

```
GoToUnvisitedEatPointAvoidHiveAction → EatAction
  → GoToOtherShootAreaViaHiveAction →（可选）LiftedSideAimShootAction
```

取球点仍是终点，车会**停稳后**再吃球——与现主程序状态机语义一致，只是把「逐段投递」变成「一次投递」。

**相对逐段入队的收益**：

| 问题 | 逐段入队 | 组合后 |
|------|----------|--------|
| 段间空档 | `ActionRunner` **每帧最多启动一个 Action**，交接处有 1 帧（约 20 ms）空档，该帧没有任何代码给底盘下发电机指令 | `SequentialAction` 是 `tailrec`，上一子 Action 返回 false 的**同一帧**接下一段 |
| 队列状态抖动 | `isBusy()` 在段间短暂为 false，`Chassis` 可能误判「队列空闲」而放行手柄或主动刹停 | 整个周期 `isBusy()` 恒为 true |
| 旧位姿规划 | 每段在轮到自己时才 new，位姿正确 | 若把子 Action 提前 new，则会用**上一段开始前**的旧位姿规划；用 `DeferredAction` 保证仍是执行时构造 |

**实现要点**：

```java
List<Action> steps = new ArrayList<>();
steps.add(new DeferredAction(() -> new GoToUnvisitedEatPointAvoidHiveAction(teamColor, sweeper)));
steps.add(new EatAction(sweeper));
steps.add(new DeferredAction(() -> new GoToOtherShootAreaViaHiveAction(teamColor, goal)));
if (includeAim) steps.add(new LiftedSideAimShootAction(teamColor, drive, turret, false, true));
return new SequentialAction(steps);
```

**Dashboard 可调参数**：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `includeAim` | `true` | 是否在周期末尾追加 `LiftedSideAimShootAction`（内部先查抬升侧） |

**构造 / 接入**：

```java
actionRunner.add(new AutoCycleAction(teamColor, sweeper, turret, HypParams.BlueAudienceUp));
```

**注意**：本 Action 只能消除调度空档，**不能**消除轨迹之间的减速到零（吃球段与移动段仍是独立轨迹）。
第 14 节的 `ContinuousRouteAction` 同样是「到点停车吃球」，两者的区别只是投递粒度。

**状态**：尚未接入 `AutoRed` / `AutoBlue`，主程序仍按原状态机逐段投递。

## 14. ContinuousRouteAction（合成版：轨迹1 → 停车吃球 → 轨迹2 → 可选瞄准）

**作用**：把「前往取球点 + **停车吃球** + 前往另一发射区域 +（可选）瞄准发射」合成**一个** Action，
内部由 `SequentialAction` 依次串联：轨迹 1 → `EatAction` → `markVisited` → 轨迹 2 →（可选）`LiftedSideAimShootAction`。
**两段轨迹之间必然停车吃球**。类别名仍为 `ContinuousRouteAction`，但已不再是「单条轨迹不停车」。

**两段轨迹与停车**：

`MecanumDrive.actionBuilder(start, beginEndVel)` 的 `beginEndVel` 在 Road Runner 的 `profile()` 中
同时作为**起点与终点速度**。本类两段轨迹都用 `actionBuilder(start, PARAMS.maxWheelVel)` 构建，
但每一份 `build()` 仍是独立轨迹，中间还夹着 `EatAction`（每帧驱动归零）——
因此到点停车是**设计行为**，任何 `SequentialAction` / `ParallelAction` 组合都无法消除。

**路线构成**：

```
start → leg1 途经点（绕开 Hive） → 取球点（intake 对准）【停车吃球 → markVisited】
      → leg2 途经点（secondLegThroughHive ? 经 Hive 正下方 : 绕外侧） → 发射区 →（可选）LiftedSideAimShootAction
```

- **取球点**：`HiveRouteAction.unvisitedEatPointTargetPose` 给出的底盘中心位姿（intake 恰好对准取球点）；
- **去程**：固定 `throughHive = false`，由 `HiveRoutePlanner.waypoints` 生成绕开 Hive 的外侧车道；
- **回程**：由 `secondLegThroughHive` 决定是否经 Hive 正下方，几何约束同第 6~9 节；
- **访问标记**：吃球结束之后才 `EatPointTracker.markVisited(eatIndex)`（不再在贴近取球点时提前标记）。

**吃球段**：

`EatAction` 每帧先把底盘驱动归零、再 `setEat()` + `update()`，因此吃球期间**底盘原地不动**；
默认吃 `eatDurationMs`（2500 ms）后 `setStop()` 并返回 false。吃球结束后才经
`InstantAction(() -> EatPointTracker.markVisited(eatIndex))` 标记该取球点已访问。

**铲子**：`run()` 每帧用 `HiveRouteAction.shovelDue(eatTargetPose)` 估算 ETA，
剩余时间 ≤ `HiveRouteAction.shovelLeadMs`（默认 1000 ms）时提前 `setShovelDown()`；
`EatAction` 首次 `run()` 还会无条件再放一次。

**必须知道的代价**：

| 现象 | 原因 | 规避 |
|------|------|------|
| 单周期变长 | 两段轨迹之间必然停车吃球（默认 2.5 s） | 按实际进球效果下调 `EatAction.eatDurationMs`，或选择更短的返程车道 |
| 段间两次加减速 | 轨迹 1 与轨迹 2 是两份独立的 `build()`，各自以 v=0 结束 | 属设计行为，Action 组合无法消除 |
| 取球点被重复选择 | 访问标记在吃球**结束**后才写入 | 若中途被 `actionRunner.clear()` 打断，该点不会被标记，下一周期可能再次前往 |
| 空跑一个取球点 | `unvisitedEatPointIndex` 按「最近的未访问点」选择 | 两个点都已访问时会退回最近者，属预期行为 |

**Dashboard 可调参数**：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `secondLegThroughHive` | `true` | 第二段是否经 Hive 正下方（`false` 为绕外侧） |
| `includeAim` | `true` | 路线末尾是否追加 `LiftedSideAimShootAction`（内部先查抬升侧） |

> 吃球时长不在本类：改 `EatAction.eatDurationMs`（默认 2500 ms = 2.5 s，Dashboard 可调）。

**构造 / 接入**：

```java
// goal 应取当前球门（HIVE 抬升侧自动 Up/Down），见各 OpMode 的 currentGoal()
actionRunner.add(new ContinuousRouteAction(teamColor, sweeper, turret,
                                           RobotPosition.getInstance().getGoalPose()));
```

**状态**：已接入 `AutoRedContinuous` / `AutoBlueContinuous`（见「自动阶段主程序（组合 Action 循环版）」）。

## 15. 铲子与吃球策略（提前放铲 / 到点停车吃球）

本节汇总与吃球相关的两项机制：**铲子提前放下** 与 **吃球时底盘锁死（到点停车吃球）**。

### 15.1 铲子提前放下

**为什么需要**：`Sweeper` 构造时默认 `setShovelUp()`，而 `setEat()` 只控制吸球电机、
不控制铲子舵机。此前所有吃球 Action 都只调 `setEat()`，铲子始终抬起——球无法被铲入。
本版在「前往取球点」的移动 Action 中统一加入放铲逻辑。

**实现**：`HiveRouteAction.shovelDue(Pose2d finalPose)` 每帧估算 ETA：

```java
double remaining = RobotPosition.getInstance().getPose2d()
        .position.minus(finalPose.position).norm();
double speed = RobotPosition.getInstance().getDrive().getLastVelocity().linearVel.norm();
double v = Math.max(speed, MecanumDrive.PARAMS.maxWheelVel * 0.5);   // 起步/拥堵兜底
boolean due = remaining / v <= HiveRouteAction.shovelLeadMs / 1000.0; // 默认 1.0 s
```

`due` 为真且尚未放铲时，调用一次 `sweeper.setShovelDown(); sweeper.update();` 并置位
`shovelDropped`（保证只下发一次）。`setShovelDown()` 只改 `targetShovelPos`（持久状态），
之后每帧 `Sweeper.update()` 都会保持该目标位置。

**覆盖范围**：

| Action | 放铲依据的终点 | 备注 |
|--------|---------------|------|
| `GoToUnvisitedEatPointViaHiveAction` | 未访问取球点中心位姿 | 需传 `Sweeper` |
| `GoToUnvisitedEatPointAvoidHiveAction` | 同上 | 需传 `Sweeper` |
| `GoToEatPointAction` | 指定取球点中心位姿 | 需传 `Sweeper` |
| `GoToNearestEatPointAction` | 自动选中的取球点中心位姿 | 需传 `Sweeper` |
| `ContinuousRouteAction` | 取球点底盘中心位姿（轨迹 1 终点） | 内部自带 `Sweeper` |
| `EatAction` | —— | 首次 `run()` 无条件 `setShovelDown()` 兜底 |
| `GoToOtherShootArea*` | —— | 去发射区，无 `Sweeper`、不放铲 |

**参数**：

| 参数 | 位置 | 默认 | 含义 |
|------|------|------|------|
| `shovelLeadMs` | `HiveRouteAction` | 1000 ms | 提前放铲量；`@Config` 可 Dashboard 实时调 |

> ⚠️ 该 ETA 是**直线距离 ÷ 线速度**的粗略估计，轨迹绕行时偏乐观。
> 若实机发现放铲偏晚，把 `shovelLeadMs` 调到 1300~1500 ms；
> 若太早（铲子刮地、影响穿过 Hive），调到 700~800 ms。

### 15.2 ContinuousRouteAction 的到点停车吃球

**为什么**：本类最初把吃球与轨迹并行（`afterDisp` 触发、边移动边吸），实测难以保证进球，
因此改为**到点停车吃**：轨迹 1 把 intake 送到取球点 → `EatAction`（每帧驱动归零，默认 2.5 s）
→ `markVisited` → 轨迹 2 前往另一发射区。

**时序**：

```
轨迹1：start → 绕开 Hive 的途经点 → 取球点（intake 对准）
吃球：EatAction 停车吃 2.5 s（底盘驱动每帧归零）→ markVisited
轨迹2：取球点 →（经 Hive 正下方 / 绕外侧，由 secondLegThroughHive 决定）→ 发射区
（可选）LiftedSideAimShootAction（先查抬升侧，必要时改道另一发射区）
```

**代价**：两段轨迹之间必然多出约 2.5 s 的静止吃球时间，以及两次 v=0 的加减速；
换来的是对齐质量与进球稳定性。

**参数**：

| 参数 | 默认 | 单位 | 含义 |
|------|------|------|------|
| `EatAction.eatDurationMs` | 2500 | ms | 停车吃球时长（Dashboard 可调） |
| `secondLegThroughHive` | `true` | — | 第二段是否经 Hive 正下方 |
| `includeAim` | `true` | — | 末尾是否追加 `LiftedSideAimShootAction` |

**调参建议**：

- 进球不足 → 增大 `EatAction.eatDurationMs`（先试 3000 ms），并确认铲子已在到位前放下；
- 周期偏长 → 缩短 `EatAction.eatDurationMs`，或把 `secondLegThroughHive` 切到更短的一侧；
- 取球点被重复前往 → 访问标记现在由吃球段结束后写入，若中途被 `actionRunner.clear()` 打断则不会记账。

### 15.3 其他 Action 保持不变

`AutoCycleAction` 与逐段状态机的吃球流程同样是
**移动到位 → 停车 → `EatAction` 静止吃球**，与 15.2 一致；区别仅在于「合成一个 Action 投递」还是「逐段投递」。

## 典型串联顺序

```
GoToShootAreaAction → AutoAimAction →（发射）
GoToEatPointAction / GoToNearestEatPointAction → EatAction
GoToOtherShootAreaViaHiveAction / GoToOtherShootAreaAvoidHiveAction → AutoAimAction →（发射）
GoToNearestShootArea{Avoid,Via}HiveAction → LiftedSideAimShootAction →（发射，必要时先改道另一发射区）
GoToUnvisitedEatPointViaHiveAction / GoToUnvisitedEatPointAvoidHiveAction → EatAction
剩余时间 ≤ PARK_TIME_THRESHOLD_MS → ActionRunner.clear() + AdaptiveParkAction
```

**整周期组合（可选，见 13 / 14 节）**：

```java
actionRunner.add(new AutoCycleAction(teamColor, sweeper, turret, goal));        // 保守版：逐段用 DeferredAction，取球点停车吃球
actionRunner.add(new ContinuousRouteAction(teamColor, sweeper, turret, goal));  // 合成版：轨迹1 → 停车吃球 → 轨迹2
```

两者都只需在 `actionRunner` 空闲时投递即可，后续周期重复投递。

## 自动阶段主程序（四组 × 红蓝，共 8 个 OpMode）

8 个 OpMode 各自是完整的独立 `LinearOpMode`（私有 `AutoState` 枚举 + `switch` 状态机），
只在两个维度上不同：**Hive 通行方式** 与 **瞄准时底盘是否参与转向**。

### 分组

| 组 | Hive 通行 | 瞄准 | 红队 | 蓝队 |
|----|-----------|------|------|------|
| 1 | 绕开 Hive（外侧车道） | 底盘不动 · `LiftedSideAimShootAction`（内用 `AutoAimAction`） | `AutoRed` | `AutoBlue` |
| 2 | 经 Hive 正下方（内车道） | 底盘不动 · `LiftedSideAimShootAction`（内用 `AutoAimAction`） | `AutoRedViaHive` | `AutoBlueViaHive` |
| 3 | 绕开 Hive（外侧车道） | 底盘辅助转向 · `LiftedSideAimShootAction`（内用 `AutoTurretAimShootWithChassisAction`） | `AutoRedChassisAim` | `AutoBlueChassisAim` |
| 4 | 经 Hive 正下方（内车道） | 底盘辅助转向 · `LiftedSideAimShootAction`（内用 `AutoTurretAimShootWithChassisAction`） | `AutoRedChassisAimViaHive` | `AutoBlueChassisAimViaHive` |

| 项目 | 取值 |
|------|------|
| 队伍 | `TeamColor.RED` / `TeamColor.BLUE` |
| 起始位姿 | `HypParams.startPoseRed` / `HypParams.startPoseBlue` |
| 预瞄准球门 | `currentGoal()` → `RobotPosition.getGoalPose()`（八者同源） |

### 状态机（8 个 OpMode 共用）

```
START_SHOOT    → 瞄准发射 Action，连打 START_SHOTS 发（底盘不动或底盘辅助转向）
TO_EAT_POINT   → GoToUnvisitedEatPointAvoidHiveAction / ViaHiveAction（自动选最近未访问取球点）
EATING         → EatAction
TO_SHOOT_AREA  → 三层优先级：
                 1. AREA_CYCLE_MODE ≠ OFF → 默认 SECOND，按枚举（EVEN/ODD/EVERY）决定切 FIRST 还是 SECOND，投 GoToShootAreaAction(FIRST/SECOND)
                 2. cyclesDone ≥ 1 且 SECOND_CYCLE_FIRST_AREA=true → 投 GoToShootAreaAction(FIRST)
                 3. 否则 → 投 GoToShootAreaAction(SECOND)（默认第二发射区域）
                 （保守版均为**单段直达轨迹**，无三等分点检查——初始目标确定后行驶中不会改道）
AIM_SHOOT      → LiftedSideAimShootAction（先查抬升侧；不在则改道另一发射区再瞄准发射 1 发）
CYCLE_END      → cyclesDone++；未满 CYCLES 回 TO_EAT_POINT，满则转 PARKING
PARKING        → AdaptiveParkAction
STOP
```

> 与旧版的三点差异：① 起点先瞄准发射 `START_SHOTS` 发；
> ② 去发射区改用**最近**发射区（`GoToNearestShootArea*`），不再去「另一个」发射区；
> ③ 循环 `CYCLES` 次后主动停靠，不再无限循环到时间耗尽。

> 另：`AIM_SHOOT` 使用 `LiftedSideAimShootAction` —— 到位后先查是否在抬升侧，
> **不在**则自动改道另一发射区域最近可发射点，再自动瞄准发射（见 10.7）。

### Dashboard 可调参数

| 参数 | 默认 | 含义 |
|------|------|------|
| `START_SHOTS` | 4 | 起点连续发射的发数（= 预载球数） |
| `CYCLES` | 2 | 「取球 → 吃球 → 发射区 → 瞄准发射」的循环次数 |
| `SECOND_CYCLE_FIRST_AREA` | `false` | 第二次及以后循环的「前往发射点」改为去**第一发射区域**；init 阶段还可按 gamepad1.Y 切换，遥测实时显示 |
| `AREA_CYCLE_MODE` | `0` | 循环数驱动发射区域切换模式，枚举：`0=OFF`（默认，所有循环默认去第二发射区域）；`1=EVEN`（偶数次末尾切第一区域）；`2=ODD`（奇数次末尾切第一区域）；`3=EVERY`（每次都切，交替模式）；init 阶段还可按 gamepad1.A 循环切换，遥测实时显示 |

前两个是 `public static int`，后两个分别是 `public static boolean` 和 `public static int`，且类上有 `@Config`，可在 Dashboard 的 Config 页实时修改。

> ⚠️ `AutoAimAction` 以「扳机下降沿」或超时结束，一次只保证一发。
> 若实际预载不足 `START_SHOTS` 球，多出的发会各自空等到超时，建议按真实预载数设置。

> 所有发射区域选择开关的**优先级**：`AREA_CYCLE_MODE ≠ OFF` > `SECOND_CYCLE_FIRST_AREA` > 默认 SECOND。
> `AREA_CYCLE_MODE` 与 `utility/AutoStartOptions` 里对应按键（gamepad1.A / Y）合并生效
> （OpMode helper 先读 `AutoStartOptions`，无效时回退到 OpMode 静态字段）。
> 详见「发射区域选择开关（三级优先级）」小节。

> ⚠️ 保守版关键差异：所有行驶段都是**单段直达轨迹**，没有全功能版 `ShootAreaApproachAction` 的
> 三等分点抬升侧检查与行驶中改道——初始目标确定后，整段轨迹一直去到该目标，不会中途改道。

### 半区约定

| 半区 | 几何 |
|------|------|
| 红队半区 | `y < 0` |
| 蓝队半区 | `y > 0` |

「非我方联盟半区」= 对方的半区（红队为 `y > 0`，蓝队为 `y < 0`）。
各 OpMode 把它作为遥测项 `In Forbidden Half` 上报，**不在规划层做绕行**。

> 按此约定，红队两个吃球点 `(24,-72)`、`(-72,-24)`、两个发射区域（`y ∈ [-30,-3]`）、
> Hive 正下方矩形（`y ∈ [-25,-1]`）与 `HiveRoutePlanner` 的全部车道
> （内侧 `y = -12.5`、外侧 `y = -39.7`）均落在 `y < 0`，属于红队本方半区，与「不走非我方联盟半区」一致；
> 蓝队为中心对称（`y > 0`），同样一致。
>
> ⚠️ 例外：`HypParams.startPoseRed = (-41.3, 55)` 与 `StopPoseRed = (0, 24)` 落在 `y > 0`，
> 蓝队的 `startPoseBlue = (-41.3, -55)` 与 `StopPoseBlue = (0, -24)` 落在 `y < 0`，
> 按本约定属于各自队伍的非我方联盟半区。若起点/停靠点必须在己方半区，需核对 `HypParams` 中
> 两者的 `y` 符号。

### 预瞄准球门必须与炮台瞄准同源

> ⚠️ **预瞄准球门必须与炮台瞄准同源**。`AutoAimAction` /
> `AutoTurretAimShootWithChassisAction` 内部通过 `Turret.update(...)` 读取
> **`RobotPosition.getGoalPose()`**（由 HIVE 抬升侧自动在 AudienceUp / AudienceDown 之间切换）。
> 因此移动类 Action 传入的 `goal` 也必须实时取 `RobotPosition.getInstance().getGoalPose()`，
> 各 OpMode 统一封装为私有方法 `currentGoal()`；
> 若写死 `HypParams.RedAudienceUp`，当 HIVE 判定为 `AUDIENCE_DOWN` 时
> 车头会朝向上侧球门、炮台需额外转约 180° 才能发射，白白损失周期时间。

### 停靠的硬保障

主循环每帧计算 `remainingMs`；一旦
`remainingMs <= PARK_TIME_THRESHOLD_MS` 且尚未投递过停靠动作，立即
`actionRunner.clear()` 中断**正在执行**的任意 Action，再投递 `AdaptiveParkAction`，
并用 `parkScheduled` 保证只投递一次。因此即使上一段长轨迹尚未跑完，T-3s 也一定会转入停靠。
另外 `CYCLE_END` 跑满 `CYCLES` 次时也会立即转入同一段停靠流程。

### 盟友禁区守卫

每帧查询 `AutoParams` 中当前时间窗生效的盟友活动区，整机轮廓一旦闯入，
立即清空队列并改投「开到最近安全位姿」的脱困动作，脱离后自动恢复状态机。

### 结束收尾

`RobotStateStore.savePose(...)` 记录结束位姿，`sweeper.setStop()` +
`turret.stop()` + `chassis.stop()` + `chassis.update(0,0,0)` 确保底盘、集球器与炮台停稳。

### 选型建议

| 场景 | 推荐 |
|------|------|
| Hive 外侧车道空闲、追求稳定 | 第 1 / 3 组（`*AvoidHive`） |
| 外侧车道被占、需要穿 Hive 正下方 | 第 2 / 4 组（`*ViaHive`） |
| 炮台平转是瞄准瓶颈 | 第 3 / 4 组（`*ChassisAim*`） |

## 自动阶段主程序（组合 Action 循环版）

这 4 个 OpMode 把「取球 + 吃球 + 移动 + 瞄准发射」合成为**单一周期 Action** 并循环执行，
状态机简化为 `CYCLE → CYCLE → … → PARKING → STOP`，其余逻辑（停靠、盟友禁区、收尾）
与 {@link AutoRed} / {@link AutoBlue} 完全一致。

### OpMode 一览

| OpMode | 队伍 | 周期 Action | 特点 |
|--------|------|-------------|------|
| `AutoRedCycle` | 红 | {@link AutoCycleAction} | 保守版：取球点停车吃球，消除段间 1 帧调度空档 |
| `AutoBlueCycle` | 蓝 | {@link AutoCycleAction} | 同上 |
| `AutoRedContinuous` | 红 | {@link ContinuousRouteAction} | 合成版：轨迹1 → 停车吃球 → 轨迹2 →（可选）`LiftedSideAimShootAction` |
| `AutoBlueContinuous` | 蓝 | {@link ContinuousRouteAction} | 同上 |

### 状态机（4 个 OpMode 共用）

```
CYCLE   → AutoCycleAction / ContinuousRouteAction（队列空闲即投递下一周期）
T-3s    → actionRunner.clear() + AdaptiveParkAction
PARKING → AdaptiveParkAction
STOP
```

- `CYCLE` 状态下：动作队列空闲且未在脱困时，立即 `actionRunner.add(new 周期Action(...))`；
- T-3s：`actionRunner.clear()` 中断正在执行的周期，投递 `AdaptiveParkAction`，状态切到 `PARKING`；
- `PARKING` 完成 → `STOP`。

### 构造与关键参数

**AutoCycleAction 构造**：
- `new AutoCycleAction(teamColor, sweeper, turret, goal)` —— 4 参，内部 `targetShootAreaIndex=0`，走 `HiveRouteAction.otherShootAreaTargetPose` 原逻辑
- `new AutoCycleAction(teamColor, sweeper, turret, goal, int targetShootAreaIndex)` —— 5 参（int）：0=otherShootArea /默认；1=FIRST（第一发射区域）；2=SECOND（第二发射区域）；OpMode 里的 AREA_CYCLE_MODE 逻辑把算好的 FIRST/SECOND 传进来
- 旧的 boolean 构造 `(teamColor, sweeper, turret, goal, boolean goFirstShootArea)` 保留做兼容（true→1，false→0）

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `includeAim` | `true` | 是否在周期末尾追加 `LiftedSideAimShootAction`（内部先查抬升侧） |

**ContinuousRouteAction 构造**：
- `new ContinuousRouteAction(teamColor, sweeper, turret, goal)` —— 4 参，同 AutoCycleAction 默认
- `new ContinuousRouteAction(teamColor, sweeper, turret, goal, int targetShootAreaIndex)` —— 5 参（int），同 AutoCycleAction
- 旧的 boolean 构造保留兼容

| 参数 | 默认值 | 单位 | 含义 |
|------|--------|------|------|
| `secondLegThroughHive` | `true` | — | 第二段是否经 Hive 正下方（`false`=绕外侧） |
| `includeAim` | `true` | — | 路线末尾是否追加 `LiftedSideAimShootAction` |

> ⚠️ 吃球时长由 `EatAction.eatDurationMs`（默认 2500 ms）控制，不在本类；
> 两段轨迹之间是**停车吃球**而非行驶中吃球，估算单周期耗时请按
> 「轨迹1 + 2.5 s + 轨迹2 +（可选）瞄准」累加。

### 用法示例

**在 Cycle 与 Continuous 之间切换**（以红队为例）：

```java
// 保守版（取球点停车吃球，最稳妥）
actionRunner.add(new AutoCycleAction(teamColor, sweeper, turret, goal));

// 合成版（轨迹1 → 停车吃球 → 轨迹2 → 可选瞄准）
actionRunner.add(new ContinuousRouteAction(teamColor, sweeper, turret, goal));
```

**临时关闭瞄准发射，只跑取球+移动周期**（Dashboard Config 页）：

```java
AutoCycleAction.includeAim = false;        // 或 ContinuousRouteAction.includeAim = false
```

**调整吃球时长 / 返程车道**（Dashboard Config 页）：

```java
EatAction.eatDurationMs = 3000;                      // 停车吃球 3 s
ContinuousRouteAction.secondLegThroughHive = false;  // 返程改走外侧车道
```

### 选型建议

| 场景 | 推荐 OpMode |
|------|-------------|
| 想逐段投递、便于观察每个 Action | `AutoRedCycle` / `AutoBlueCycle` |
| 希望整个周期一次入队、减少调度空档 | `AutoRedContinuous` / `AutoBlueContinuous` |
| 不确定 / 首次联调 | 先 `Cycle`，稳定后再试 `Continuous` |

所有周期 Action 都在**首次 `run()` 时**以当时真实位姿规划轨迹，因此可直接入队，
无需像逐段状态机那样处理段间位姿刷新。

## 发射区域选择开关（三级优先级）

所有自动 OpMode 的「前往发射点」默认都去 **第二发射区域（SECOND）**，以下三级开关依次覆盖：

> ⚠️ **保守版关键差异**：所有行驶段都是**单段直达轨迹**，没有全功能版的
> `ShootAreaApproachAction` 三等分点抬升侧检查与行驶中改道。初始目标确定后，
> 整段轨迹一直去到该目标，不会中途改道。行驶中的抬升侧检查**只在** `AIM_SHOOT`
> 阶段由 `LiftedSideAimShootAction`（10.7 节）执行。

### 开关 1：`AREA_CYCLE_MODE`（循环数驱动切换 —— **最高优先级**）

**枚举值**（`public static int`，Dashboard 可实时改；init 时按 **gamepad1.A** 循环切换 OFF→EVEN→ODD→EVERY→OFF）：

| 值 | 名称 | 行为（默认 SECOND，满足条件时 FIRST） |
|----|------|------|
| 0 | `OFF` | 关闭，不生效（交由开关 2 或默认 SECOND） |
| 1 | `EVEN` | **偶数次**循环末尾切到第一发射区域（奇数次保持第二区域） |
| 2 | `ODD` | **奇数次**循环末尾切到第一发射区域（偶数次保持第二区域） |
| 3 | `EVERY` | 每次循环末尾都切（**交替模式**：第一次 SECOND → 第二次 FIRST → 第三次 SECOND → …） |

**三套「当前循环数」获取**（不同程序族计数起点不同）：

| 程序族 | 字段 | 第 N 次循环时的值 | 偶数判定 |
|--------|------|-------------------|---------|
| 8 个状态机 OpMode | `cyclesDone`（初始 0，`CYCLE_END` 后 ++） | `N-1` | `(cyclesDone + 1) % 2 == 0` |
| Cycle / Continuous | `cycleCount`（初始 1，投完 ++） | `N` | `cycleCount % 2 == 0` |
| AutoRoutine（4 个） | `round`（初始 1，AIM 后置 2） | `N` | `round % 2 == 0` |

**实现方式**：
- **状态机** `case TO_SHOOT_AREA`：算好 FIRST/SECOND 后投 `GoToShootAreaAction(teamColor, targetArea, goal, throughHive)`；
  ViaHive 组 throughHive=true、AvoidHive 组 throughHive=false。**保守版单段直达**——行驶中不会改道，
  初始目标是 FIRST 就一直去到 FIRST，是 SECOND 就一直去到 SECOND。
- **Cycle / Continuous 组合 Action**：OpMode 算出 FIRST/SECOND 后把 `targetShootAreaIndex`（1=FIRST, 2=SECOND）
  传给 `AutoCycleAction` / `ContinuousRouteAction` 的 5 参 int 构造，内部 `HiveRouteAction.shootArea`
  得到对应区域最近点（朝向实时球门）。保守版没有 ShootAreaApproachAction，行驶中不改道。
- **AutoRoutine** `case TO_SHOOT_2`：算好后用固定点 `HypParams.ShootFirstAreaRed/Blue` 或 `ShootSecondAreaRed/Blue`，
  其余同原逻辑（AIM_SHOOT 后转入下一轮或停靠）。

### 开关 2：`SECOND_CYCLE_FIRST_AREA`（上一轮兼容 —— **次优先级**）

**开关来源**（两处默认 `false`，合并生效）：

1. 各 OpMode 的 `@Config public static boolean SECOND_CYCLE_FIRST_AREA = false`（FtcDashboard 实时改）；
2. `utility/AutoStartOptions.secondCycleFirstArea()`（waitForStart 前按 **gamepad1.Y** 切换，`reset()` 每局复位）。

OpMode 内 helper 返回 `OpMode.SECOND_CYCLE_FIRST_AREA || AutoStartOptions.secondCycleFirstArea()`，遥测键
`2nd Cycle First Area (gamepad1.Y)` 实时显示合并值。

**判定**：`AREA_CYCLE_MODE == OFF` 时，`cyclesDone >= 1`（状态机）/ `cycleCount >= 2`（Cycle/Continuous）/ `round >= 2`（AutoRoutine）
时改去 FIRST，否则保持 SECOND。

### 开关 3：默认 SECOND（**最低优先级**）

两个开关都关时，所有循环统一去 **第二发射区域（SECOND）**，不再按就近选或按抬升侧改道。
**保守版在行驶中也不会改道**（没有 ShootAreaApproachAction）——初始目标确定后整段轨迹直达该目标。
行驶中的抬升侧检查**只在** `AIM_SHOOT` 阶段由 `LiftedSideAimShootAction`（10.7 节）执行，
与本开关的初始目标选择不冲突。

### 按键一览（waitForStart 前）

| 按键 | 开关 | 循环方式 |
|------|------|---------|
| **gamepad1.X** | `startShoot`（开局是否先发射） | 按下沿翻转 |
| **gamepad1.Y** | `SECOND_CYCLE_FIRST_AREA`（第二次起改 FIRST） | 按下沿翻转 |
| **gamepad1.A** | `AREA_CYCLE_MODE`（循环数驱动切换） | 按下沿循环 OFF→EVEN→ODD→EVERY→OFF |

全部遥测实时显示当前值，`reset()` 每局复位。

## 参数配置：AutoParams（盟友活动区 / 我方禁止活动区）

**文件**：`Parameter/AutoParams.java`（`@Config`，与 `HypParams` 同级）

**作用**：我方联盟只有 **1 名队友**，登记其在「我方区域」自动阶段依次占据的 **5 块**活动范围，
作为**我方在该时段内的禁止活动区域**，避免自动阶段抢位 / 相撞。

**关键语义**：每个禁区 = **一块凸多边形区域 + 一个时间窗**；
时间窗之外该区域**不构成限制**，因此同一块场地在不同时段可被不同活动范围复用。

### 全局参数

| 参数 | 默认值 | 单位 | 含义 |
|------|--------|------|------|
| `useAllyZones` | `true` | — | 总开关；`false` 时所有查询恒返回「不受限」 |
| `allyZoneClearanceIn` | `2.0` | in | 安全外扩量：整机轮廓距禁区边界小于该值即判闯入，用于吸收定位误差 |
| `allyZoneEscapeMarginIn` | `1.0` | in | 脱困余量：脱困目标点停在「禁区边界 + clearance + 本值」之外，避免贴边反复触发 |

### 区域与时间窗

所有区域按**红队视角**以场地坐标（英寸）填写，蓝队由原点中心对称自动生成：

| 字段 | 区域名称 | 范围（红队） | 时间窗 |
|------|----------|--------------|--------|
| `ALLY_ZONE_1` | `AllyRange1-StartCorner` | `x∈[-70,-40]`、`y∈[40,60]` | 0 – 6000 ms |
| `ALLY_ZONE_2` | `AllyRange2-MidLeft` | `x∈[-20,10]`、`y∈[30,50]` | 3000 – 12000 ms |
| `ALLY_ZONE_3` | `AllyRange3-RightStart` | `x∈[25,55]`、`y∈[35,55]` | 0 – 9000 ms |
| `ALLY_ZONE_4` | `AllyRange4-NearHive` | `x∈[-15,15]`、`y∈[8,28]` | 8000 – 20000 ms |
| `ALLY_ZONE_5` | `AllyRange5-RightLow` | `x∈[40,68]`、`y∈[5,25]` | 12000 – 30000 ms |
| `ALLY_ZONES_RED` | — | 上述 5 个 | 各自时间窗 |
| `ALLY_ZONES_BLUE` | — | 上述 5 个的原点中心对称 | 各自时间窗（不变） |

> 以上坐标与时间窗均为**占位默认值**，必须按队友实际跑位与战术安排修改。
>
> **增加范围数量**：新增 `ALLY_ZONE_6` 字段（红队视角坐标）→ 追加到 `ALLY_ZONES_RED` 的
> `Arrays.asList(...)` 末尾即可；`ALLY_ZONES_BLUE` 与全部查询 / 判定 / 脱困逻辑自动适配，无需改动。
> 唯一例外：`nearestSafePose` 对重叠禁区最多迭代 4 次脱困，若同一时刻生效且彼此重叠的禁区可能超过 4 个，
> 需调大该迭代上限。**减少数量**：删除字段并同步从列表移除。

**红蓝镜像**：`ALLY_ZONES_BLUE` 由 `area.inRelative(0, 0, Math.PI)` 生成，
即 `(x, y) → (-x, -y)`，与 `HypParams` 中发射区域、Hive 矩形的红蓝约定一致。

**时间基准**：`startMs` / `endMs` 为**相对自动阶段 START 的毫秒数**，与 OpMode 中的
`ElapsedTime matchTime` 同一基准；自动阶段总时长见 `HypParams.AUTONOMOUS_DURATION_MS`。

### 查询接口

| 方法 | 返回 | 用途 |
|------|------|------|
| `zonesFor(teamColor)` | `List<AllyZone>` | 全量禁区（不按时间过滤） |
| `activeZones(teamColor, matchMs)` | `List<AllyZone>` | 该时刻**生效**的禁区 |
| `activeZoneNames(teamColor, matchMs)` | `List<String>` | 生效禁区名称（遥测 / 日志） |
| `isForbidden(teamColor, pose, matchMs)` | `boolean` | 整机是否闯入任一生效禁区 |
| `firstConflictingZone(teamColor, pose, matchMs)` | `AllyZone` / `null` | 冲突来源，供决定绕行方向 |
| `nearestSafePose(teamColor, pose, matchMs)` | `Pose2d` | 脱离禁区的**最小位移**安全位姿（含安全余量、自动夹在场地内） |

`AllyZone` 自身提供：`activeAt(matchMs)`、`contains(x, y)`、`distanceTo(x, y)`、`mirrored()`。

**判定精度**：`isForbidden` 取 `ChassisGeometry.footprintCorners(pose)` 的**整机外接矩形四个角点**
逐点判定，任一角点到禁区距离 ≤ `allyZoneClearanceIn` 即判闯入——
判定的是**整机轮廓**而非单纯中心点。

### 已内置到自动 OpMode 的守卫（无需额外调用）

`AutoRed` / `AutoBlue` 主循环**每帧**执行下列守卫，因此禁区是**自动生效**的：

```java
long matchMs = (long) matchTime.milliseconds();

// 停靠阶段优先，不再检查禁区
Pose2d pose = RobotPosition.getInstance().getPose2d();
AutoParams.AllyZone conflict = parkScheduled || !AutoParams.useAllyZones
        ? null
        : AutoParams.firstConflictingZone(teamColor, pose, matchMs);

if (conflict != null && !zoneEscapeActive) {
    zoneEscapeActive = true;
    actionRunner.clear();                                       // 中断当前动作
    actionRunner.add(new GoToStopPose(                          // 改投脱困动作
            drive, AutoParams.nearestSafePose(teamColor, pose, matchMs)));
} else if (conflict == null) {
    zoneEscapeActive = false;                                   // 已脱离，恢复常规循环
}

// 脱困期间暂停常规动作投递
if (!actionRunner.isBusy() && !zoneEscapeActive) { /* 状态机投递 */ }
```

行为要点：

1. **闯入即中断**：`actionRunner.clear()` 会打断正在执行的轨迹 Action，不等其跑完；
2. **脱困目标动态求解**：`nearestSafePose()` 以禁区包围盒为界，把底盘中心推到最近盒外位置
   （航向不变、位移最小），并额外留出 `allyZoneClearanceIn + allyZoneEscapeMarginIn`，
   结果还要夹在场地内，不会为了躲禁区而开出边框；
3. **只触发一次**：`zoneEscapeActive` 置位后不再重复投递；
4. **自动恢复**：脱困后 `conflict == null`，标志清零，状态机从原状态继续投递动作；
5. **与停靠互斥**：`parkScheduled == true` 后守卫停用，停靠逻辑独占控制权。

遥测新增三行：`Active Zones`（生效禁区名）、`In Forbidden`（是否闯入）、`Escaping`（是否在脱困）。

### 其他用法示例

**1）遥测显示当前生效的禁区（Dashboard / Driver Station）：**

```java
long matchMs = (long) matchTime.milliseconds();
telemetry.addData("Active Zones", AutoParams.activeZoneNames(teamColor, matchMs));
telemetry.addData("In Forbidden", AutoParams.isForbidden(
        teamColor, RobotPosition.getInstance().getPose2d(), matchMs));
```

**2）规划前检查目标点是否落在禁区内（提前绕行，而不是闯进去再脱困）：**

```java
AutoParams.AllyZone conflict = AutoParams.firstConflictingZone(teamColor, targetPose, matchMs);
if (conflict != null) {
    // targetPose 在该时段属于队友活动范围，需要改点或等待时间窗结束
    telemetry.addData("Blocked by", conflict.name + " " + conflict);
}
```

**3）临时关闭约束做对比试验（Dashboard Config 页）：**

```java
AutoParams.useAllyZones = false;   // 等价于在 Dashboard 取消勾选
```

### 修改注意

- 区域与时间窗在**类加载时**构建为不可变列表，修改后需**重新运行 OpMode** 生效；
  `useAllyZones`、`allyZoneClearanceIn`、`allyZoneEscapeMarginIn` 三个标量可在 Dashboard Config 页实时调整。
- 禁区判定依赖定位精度，实测若出现「压线」，优先调大 `allyZoneClearanceIn`（3~4 in）。
- 时间窗写反（`startMs > endMs`）时构造器会自动交换，不会抛异常。

## 实机测试

每个 Action 都有对应的实机测试 OpMode，位于
`Auto/AutoConservative/AutoOpModes/`（Driver Station 中组名 **ActionTest**）：

| OpMode | 被测 Action |
|--------|------------|
| `Test: GoToStopPose` | `GoToStopPose` |
| `Test: GoToShootArea` | `GoToShootAreaAction` |
| `Test: GoToShootAreaVertices` | `GoToShootAreaNearestVerticesAction` |
| `Test: GoToEatPoint` | `GoToEatPointAction` |
| `Test: GoToNearestEatPoint` | `GoToNearestEatPointAction` |
| `Test: GoToUnvisitedEatPointViaHive` | `GoToUnvisitedEatPointViaHiveAction` |
| `Test: GoToOtherShootAreaViaHive` | `GoToOtherShootAreaViaHiveAction` |
| `Test: GoToUnvisitedEatPointAvoidHive` | `GoToUnvisitedEatPointAvoidHiveAction` |
| `Test: GoToOtherShootAreaAvoidHive` | `GoToOtherShootAreaAvoidHiveAction` |
| `Test: AutoAim` | `AutoAimAction` |
| `Test: Eat` | `EatAction` |

> `AdaptiveParkAction` 没有独立测试 OpMode：它的触发依赖比赛计时与自动阶段完整流程，
> 请通过 `AutoRed` / `AutoBlue` 实机联调验证（重点观察 T-3s 是否停车、接触时是否向场内移动、
> Dashboard 的 `AdaptivePark park x/y` 是否为场内位置）。
>
> `AutoCycleAction` / `ContinuousRouteAction` 同样暂无独立测试 OpMode：它们由上表 Action 组合而成，
> 建议先逐个验证被组合的 Action，再在 `AutoBlue` 中接入联调；也可参照 `ActionTestBase`
> 新增一个只运行单个组合 Action 的测试 OpMode。

**Init 阶段按键选择**：

- `A` / `B`：红队 / 蓝队（决定起始位姿、Limelight pipeline、球门颜色）；
- 十字键上 / 下：切换 `FIRST` / `SECOND`（仅发射区域、取球点测试显示）；
- `X`：切换瞄准球门 AudienceUp / AudienceDown（仅发射区域类测试显示）。

按下 START 后自动构造并执行该 Action，完成后 OpMode 自动结束；
Dashboard 与 Driver Station 遥测区可观察目标位姿、误差、Action 状态。
运行前请将机器人放到与所选起始位姿一致的位置和朝向上。
