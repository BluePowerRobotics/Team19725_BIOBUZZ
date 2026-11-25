# AutoRoutine 使用说明

> **本目录是「规定路线」自动程序**：所有取球点 / 发射区域 / 吸球起终点 / 停靠位姿
> 全部取自 `Parameter/HypParams` 的**固定值**，不含任何动态选点。
>
> 复用 `Auto/AutoFullFeatured/AutoActions/` 中的通用 Action（`AutoAimAction`、`EatAction`、
> `GoToStopPose`），并只在本目录内实现两个自己的移动 Action。

## 目录定位

| 项目 | 说明 |
|------|------|
| 移动方式 | `HiveRoutePlanner.waypoints(..., false)` 沿**外侧车道绕开 Hive**，单条轨迹一次走完 |
| 取球点 | 固定 `HypParams.EatPoint1` / `EatPoint2`，按 `round` 选择 |
| 发射区域 | 固定 `HypParams.ShootSecondAreaRed/Blue`（默认）或 `ShootFirstAreaRed/Blue`（开关开启时） |
| 停靠 | 固定 `HypParams.StopPoseRed` / `StopPoseBlue` |
| **不包含** | 盟友活动禁区（`AutoParams`）、取球点访问记账（`EatPointTracker`）、就近发射区选择、行驶途中三等分点改道 |

## 通用约定

1. **串行队列执行**：由 `utility/ActionRunner`（`add` / `update` / `isBusy` / `clear`）调度，
   OpMode 主循环每帧调用 `actionRunner.update()`。
2. **非阻塞**：每个 Action 的 `run(packet)` 每帧调用一次，返回 `true` = 未完成，`false` = 结束。
3. **构造时刻固化**：`RoutineMoveAction` 在**构造时**取起点位姿并规划整条轨迹，
   因此必须在「入队前、机器人位置基本确定后」再 new，不可提前构造后复用。
   `SweepMoveAction` 例外，它把轨迹延迟到**首次 `run()`** 构建。
4. **单位**：坐标英寸，角度弧度，时间毫秒。
5. **时间最优**：移动类 Action 均以 `MecanumDrive.PARAMS.maxWheelVel` 规划，
   梯形速度曲线（起步→巡航→最快停止），途经点作为链式途经点、段间不停车。
6. **铲子提前放下**：传入 `Sweeper` 的移动 Action，按「剩余直线距离 ÷ 线速度」估算 ETA，
   当 ETA ≤ `HiveRouteAction.shovelLeadMs`（默认 1000 ms）时下发 `setShovelDown()`。

## Action 一览（本目录自有）

| Action | 作用 | 构造参数 |
|--------|------|---------|
| `RoutineMoveAction` | 一段固定行驶：当前位姿 → 目标位姿，途经点按**外侧车道绕开 Hive** 一次性固化；传入 `Sweeper` 时提前放铲 | `(TeamColor teamColor, Pose2d targetPose, Sweeper sweeper)`，`sweeper` 可为 `null` |
| `SweepMoveAction` | 「边前往吸球终点、边吸球」：**单条直线段**（不绕 Hive），第一帧放铲 + 开启集球器，到位后停止吸球 | `(Pose2d targetPose, Sweeper sweeper)` |

### RoutineMoveAction 细节

- 构造时调用 `HiveRoutePlanner.waypoints(teamColor, start, targetPose, false)` 获取途经点，
  再用**一个** `actionBuilder`（起点速度 0、终点速度 0）链式 `strafeToLinearHeading`，
  整段轨迹不中途停车。
- 每帧若 `sweeper != null` 且 `HiveRouteAction.shovelDue(targetPose)` 为真，
  立即 `setShovelDown() + update()`（只下发一次）。
- 遥测键：`Routine target x` / `Routine target y` / `Routine target heading(deg)`，
  放铲时额外输出 `Routine shovel = Down`。

### SweepMoveAction 细节

- **首次 `run()`** 才以当时融合位姿构建单条 `strafeToLinearHeading` 轨迹（不经过
  `HiveRoutePlanner`，直线段），因此可直接入队、跨状态复用。
- 第一帧 `setShovelDown()`，此后每帧 `setEat() + update()`。
- 轨迹结束那一帧 `setStop() + update()` 并返回 `false`。
- 遥测键：`Sweep target x` / `Sweep target y`。

## 复用的 AutoFullFeatured Action

| Action | 在 AutoRoutine 中的用途 |
|--------|------------------------|
| `AutoAimAction(MecanumDrive, Turret)` | 底盘不动、炮台自动瞄准并扣扳机（一发）；用于开局连发与每轮发射 |
| `EatAction(Sweeper)` | 底盘原地不动，集球器吃球（默认 2.5 s）后停转 |
| `GoToStopPose(MecanumDrive, Pose2d)` | 前往固定停靠位姿并停车；T-3s 或路线跑完时使用 |
| `HiveRouteAction.shovelLeadMs` / `shovelDue(Pose2d)` | `RoutineMoveAction` 的提前放铲判据（静态成员，Dashboard 可调） |
| `HiveRoutePlanner.waypoints(...)` | `RoutineMoveAction` 的外侧绕 Hive 途经点 |

## OpMode 一览

| OpMode | group | 状态数 | 说明 |
|--------|-------|--------|------|
| `AutoRedRoutine` | `AutoRoutine` | 7 | 红队规定路线：吃球 1 → 发射 → 吃球 2 → 发射 → 停靠 |
| `AutoBlueRoutine` | `AutoRoutine` | 7 | 蓝队同上 |
| `AutoRedTakeGroundball` | `AutoRoutine` | 10 | 红队在 7 状态路线**之后**追加地面吸球阶段 |
| `AutoBlueTakeGroundball` | `AutoRoutine` | 10 | 蓝队同上 |

---

## 1. 状态机（7 状态：`AutoRedRoutine` / `AutoBlueRoutine`）

```
START_SHOOT ──(发完 START_SHOTS 或开局关闭)──> TO_EAT ──> EATING ──> TO_SHOOT_2 ──> AIM
                                                  ↑                                  │
                                                  └────── round==1（置 round=2）───────┤
                                                                                     │
                        round==2 ──> PARKING ──> STOP                                │
```

| 状态 | 动作 | 下一状态 |
|------|------|---------|
| `START_SHOOT` | `AutoAimAction`，每帧投一发，`shotsFired` 递增；达到 `START_SHOTS` 或开局被关闭时结束 | `TO_EAT` |
| `TO_EAT` | `RoutineMoveAction(teamColor, fixedEatPose(round==1 ? EatPoint1 : EatPoint2), sweeper)` | `EATING` |
| `EATING` | `EatAction(sweeper)` | `TO_SHOOT_2` |
| `TO_SHOOT_2` | 按 `AREA_CYCLE_MODE` / `SECOND_CYCLE_FIRST_AREA` 选第一或第二发射区域的固定点，`RoutineMoveAction(..., null)` | `AIM` |
| `AIM` | `AutoAimAction`；`round==1` 时置 `round=2` 并回到取球流程，否则进入停靠 | `TO_EAT` / `PARKING` |
| `PARKING` | 首次进入调用 `startParking()`（`clear()` 后投 `GoToStopPose`），已停靠则结束 | `STOP` |
| `STOP` | 退出主循环 | — |

**位姿计算**（均为固定值，不做就近选点）：

- `fixedEatPose(Vector2D eatPoint)`：`ChassisGeometry.centerPoseFromIntakeTarget(...)`，
  按「前端 intake 落在取球点」反推底盘中心位姿，车头朝向该点。
- `fixedShootPose(ConvexPolygon area)`：取该区域中**距场地原点 (0,0) 最近**的顶点作为停靠点，
  航向 `atan2(fixedGoal.y - y, fixedGoal.x - x)`（`fixedGoal` = `RedAudienceUp` / `BlueAudienceUp`）。

**提前停靠**：主循环每帧计算 `remainingMs`，当 ≤ `HypParams.PARK_TIME_THRESHOLD_MS` 时
`startParking()`：`actionRunner.clear()` + `GoToStopPose(drive, stopPose)`，丢弃未完成的路线。

## 2. 状态机（10 状态：`AutoRedTakeGroundball` / `AutoBlueTakeGroundball`）

在前述 7 状态基础上，第二轮 `AIM` 结束后**不直接停靠**，而是进入地面吸球：

```
... AIM ──(round==2)──> TAKE_GROUNDBALL ──> TO_SWEEP_START ──> SWEEPING ──> PARKING ──> STOP
```

| 状态 | 动作 | 下一状态 |
|------|------|---------|
| `TAKE_GROUNDBALL` | `setShovelDown() + setEat() + update()`，原地进入吸球姿态（不移动） | `TO_SWEEP_START` |
| `TO_SWEEP_START` | `RoutineMoveAction(teamColor, groundSweepStart, null)`（外侧绕 Hive） | `SWEEPING` |
| `SWEEPING` | `SweepMoveAction(groundSweepEnd, sweeper)`：直线吸球，到位停吸 | `PARKING` |

其余状态与 7 状态版本完全一致。

## 3. 位姿来源（`Parameter/HypParams`）

| 参数 | 红队 | 蓝队 | 用途 |
|------|------|------|------|
| `startPoseRed/Blue` | `(-41.3, -55, 0)` | `(-41.3, 55, 0)` | 起始位姿 / 定位初始化 |
| `StopPoseRed/Blue` | `(0, -24, π)` | `(0, 24, π)` | 固定停靠位姿 |
| `RedAudienceUp` / `BlueAudienceUp` | `(17.7, -12.75, 0)` | `(17.7, 12.75, 0)` | 固定预瞄准球门（规定路线不动态切换） |
| `EatPoint1` / `EatPoint2` | `(24, -72)` / `(-72, -24)` | 同左（两队共用） | 取球点 1 / 2（`Vector2D`） |
| `ShootFirstAreaRed/Blue` | 凸多边形 | 凸多边形 | 第一发射区域（`ConvexPolygon`） |
| `ShootSecondAreaRed/Blue` | 凸多边形 | 凸多边形 | 第二发射区域（`ConvexPolygon`） |
| `GroundSweepStartRed/Blue` | `(36, -48, π)` | `(-36, 48, 0)` | 地面吸球起点 |
| `GroundSweepEndRed/Blue` | `(-36, -48, π)` | `(36, 48, 0)` | 地面吸球终点 |

> 吸球起终点坐标目前为**占位值，需实机标定**。

## 4. Dashboard 可调参数

| 参数 | 默认 | 含义 |
|------|------|------|
| `START_SHOTS` | 4 | 起点连续发射的发数（= 预载球数） |
| `SECOND_CYCLE_FIRST_AREA` | `false` | 第二次及以后循环的「前往发射点」改为去**第一发射区域**；init 阶段可按 gamepad1.Y 切换 |
| `AREA_CYCLE_MODE` | `0` | 循环数驱动发射区域切换：`0=OFF`（默认全去第二区域）、`1=EVEN`（偶数轮去第一区域）、`2=ODD`（奇数轮去第一区域）、`3=EVERY`（交替：第 1 轮第二、第 2 轮第一、第 3 轮第二…）；init 阶段可按 gamepad1.A 循环切换 |

三个字段均为 `public static`，类上有 `@Config`，可在 FTC Dashboard 实时修改。

**优先级**：`AREA_CYCLE_MODE ≠ OFF` > `SECOND_CYCLE_FIRST_AREA` > 默认第二发射区域。
其中 `AREA_CYCLE_MODE` 与 gamepad1.A、`SECOND_CYCLE_FIRST_AREA` 与 gamepad1.Y 分别合并生效
（helper 先读 `AutoStartOptions`，无效时回退到 OpMode 静态字段）。

**循环数判定**：`round` 初始 1，第一轮 `AIM` 后置 2；因此 `round % 2 == 0` 即「偶数轮」。

> ⚠️ `AutoAimAction` 以「扳机下降沿」或超时结束，一次只保证一发。
> 若实际预载不足 `START_SHOTS` 球，多出的发会各自空等到超时，建议按真实预载数设置。

## 5. 按键（`waitForStart` 之前）

| 按键 | 开关 | 切换方式 |
|------|------|---------|
| **gamepad1.X** | 开局是否先发射（默认开） | 按下沿翻转 |
| **gamepad1.Y** | 第二次起改去第一发射区域（默认关） | 按下沿翻转 |
| **gamepad1.A** | `AREA_CYCLE_MODE`（默认 OFF） | 按下沿循环 OFF→EVEN→ODD→EVERY→OFF |

init 阶段遥测实时显示三项当前值；`AutoStartOptions.reset()` 每局复位，避免上一局选择残留。

## 6. 与其他自动程序族的差异

| 对比项 | `AutoRoutine`（本目录） | `AutoFullFeatured` | `AutoConservative` |
|--------|------------------------|--------------------|--------------------|
| 取球点选择 | 固定 1 → 2（按 `round`） | 未访问点记账 + 就近 | 同全功能版 |
| 发射区域选择 | 固定 SECOND（可开关切 FIRST） | 就近 + 抬升侧改道 | 单段直达 + 抬升侧检查 |
| 行驶中三等分点改道 | ❌ 无 | ✅ 有（`ShootAreaApproachAction`） | ❌ 无 |
| 盟友禁区守卫 | ❌ 无 | ✅ 有 | ✅ 有 |
| 时间不够时停靠 | ✅ `PARK_TIME_THRESHOLD_MS` | ✅ `AdaptiveParkAction`（自适应） | ✅ 自适应 |
| 地面吸球 | ✅（`*TakeGroundball`） | ❌ | ❌ |

## 7. 已知限制

1. **吸球起终点为占位值**：`GroundSweepStartRed/Blue`、`GroundSweepEndRed/Blue`
   需按实机场地与机械尺寸标定后再用于比赛。
2. **`SweepMoveAction` 为直线段**：不绕 Hive；若起点到终点连线穿过 Hive，
   需先调整 `GroundSweepStart` 使其位于 Hive 同侧可用通道。
3. **固定球门**：`fixedGoal` 在构造时固定为 `AudienceUp`，不支持 HIVE 抬升侧动态切换；
   若比赛现场 HIVE 状态与假设不符，需改 `fixedGoal` 或增加判定。
4. **`RoutineMoveAction` 构造即规划**：若在机器人尚未到达前就提前 new，
   会以旧位姿规划出错误轨迹；必须遵守「入队前构造」的约定。
