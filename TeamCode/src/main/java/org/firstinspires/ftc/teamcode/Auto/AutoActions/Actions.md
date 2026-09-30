# AutoActions 使用说明

本目录（`Auto/AutoActions/`）汇总自动阶段可入队执行的 Road Runner `Action`。
所有移动类 Action 的位姿来源统一为 **`AdaptiveEKFLocalizer` 融合定位系统**
（Pinpoint 里程计 + Limelight MT1 视觉的自适应 EKF 融合位姿）：

- 构造时通过 `RobotPosition.getInstance().getPose2d()` 取起点位姿（轨迹在构造时刻固化）；
- 执行时轨迹跟随内部持续调用融合定位器推进；
- `AutoAimAction` 每帧直接读 `AdaptiveEKFLocalizer.getPose()`。

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

## Action 一览

| Action | 作用 | 构造参数 |
|--------|------|---------|
| `GoToStopPose` | 前往停车位姿并停车 | `(MecanumDrive drive, Pose2d stopPose)` |
| `GoToShootAreaAction` | 抵达指定发射区域内离机器人最近的点，车头预瞄指定球门 | `(TeamColor, int areaIndex, Pose2d goal)` |
| `GoToShootAreaNearestVerticesAction` | 依次到两个发射区域各自距原点最近的顶点 | `(TeamColor, Pose2d goal)` |
| `GoToEatPointAction` | 到指定取球点（intake 到位、车头朝取球点） | `(int pointIndex)` |
| `GoToNearestEatPointAction` | 自动选近的取球点并前往 | 无 |
| `AutoAimAction` | 原地 PD 转向，自动锁定当前 HIVE 侧球门 | `(TeamColor)` |
| `EatAction` | 集球器以吃球速度运转指定时长后停转 | `(Sweeper)` 或 `(Sweeper, long durationMs)` |

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
3. 先驶向最近区域的顶点，停稳后再驶向另一区域的顶点（两段轨迹依次执行）。

**注意**：第二段轨迹以第一段名义终点为起点构造，故第一段有跟踪误差时第二段起点
与实际位姿存在小幅偏差。

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

**构造**：`new GoToEatPointAction(GoToEatPointAction.FIRST)`（或 `SECOND`，非法序号抛异常）。

## 5. GoToNearestEatPointAction

**作用**：构造时计算底盘中心到 `EatPoint1` / `EatPoint2` 的直线距离，自动选近者，
其余同 `GoToEatPointAction`。

**构造**：`new GoToNearestEatPointAction()`。

## 6. AutoAimAction

**作用**：每帧读融合位姿，依据 `RobotPosition.getHiveState()` 选择球门
（`AUDIENCE_UP→Up`、`AUDIENCE_DOWN→Down`、`MIDDLE→保持上次锁定，无历史默认 Up`），
原地转向使车头对准球门，仅下发角速度、平移为 0。

**控制律**：期望航向 `des = atan2(goalY-y, goalX-x)`，误差归一化到 `[-π,π]`
后做 PD：`omega = Kp·e + Kd·de/dt`，死区内输出 0，输出限幅。

**完成条件**：航向误差与实际角速度同时小于容差并持续 `settleMs` 毫秒；
超过 `timeoutMs` 强制结束（交上层状态机处理）。

**Dashboard 可调参数**：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `aimKp` | 3.0 | 航向环 P |
| `aimKd` | 0.15 | 航向环 D |
| `aimMaxOmega` | π | 角速度上限 (rad/s) |
| `headingTol` | 2° | 航向收敛容差 |
| `omegaTol` | 0.15 | 角速度收敛容差 (rad/s) |
| `settleMs` | 200 | 收敛持续时间 |
| `timeoutMs` | 3000 | 超时强制结束 |

**构造**：`new AutoAimAction(TeamColor.RED)`。

## 7. EatAction

**作用**：每帧 `sweeper.setEat()` + `sweeper.update()`（Sweeper 每帧自动归零目标，
必须每帧重新置位），持续到时后 `setStop()` 并结束。

**Dashboard 可调参数**：`eatDurationMs`（默认 2000 ms，待实机标定）；
吃球速度 `Sweeper.EatVel`（默认 700 tick/s）在 `Sweeper` 类中调整。

**构造**：
```java
Sweeper sweeper = new Sweeper(hardwareMap, telemetry);
new EatAction(sweeper);                       // 用 eatDurationMs 默认时长
new EatAction(sweeper, 1500);                 // 指定时长
```

## 典型串联顺序

```
GoToShootAreaAction → AutoAimAction →（发射）
GoToEatPointAction / GoToNearestEatPointAction → EatAction
任意阶段超时 → ActionRunner.clear() + GoToStopPose
```

## 实机测试

每个 Action 都有对应的实机测试 OpMode，位于
`Auto/AutoOpModes/`（Driver Station 中组名 **ActionTest**）：

| OpMode | 被测 Action |
|--------|------------|
| `Test: GoToStopPose` | `GoToStopPose` |
| `Test: GoToShootArea` | `GoToShootAreaAction` |
| `Test: GoToShootAreaVertices` | `GoToShootAreaNearestVerticesAction` |
| `Test: GoToEatPoint` | `GoToEatPointAction` |
| `Test: GoToNearestEatPoint` | `GoToNearestEatPointAction` |
| `Test: AutoAim` | `AutoAimAction` |
| `Test: Eat` | `EatAction` |

**Init 阶段按键选择**：

- `A` / `B`：红队 / 蓝队（决定起始位姿、Limelight pipeline、球门颜色）；
- 十字键上 / 下：切换 `FIRST` / `SECOND`（仅发射区域、取球点测试显示）；
- `X`：切换瞄准球门 AudienceUp / AudienceDown（仅发射区域类测试显示）。

按下 START 后自动构造并执行该 Action，完成后 OpMode 自动结束；
Dashboard 与 Driver Station 遥测区可观察目标位姿、误差、Action 状态。
运行前请将机器人放到与所选起始位姿一致的位置和朝向上。
