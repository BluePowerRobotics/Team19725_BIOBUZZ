# 集球器（Sweeper）模块说明

## 1. 功能概览

`Controllers.Sweeper.Sweeper` 统一管理 intake 电机与铲子舵机，提供四种命令：

| 命令 | 目标速度 | 用途 | 满球时 |
|---|---|---|---|
| `setEat()` | `EatVel`(700) | 从场地吸球 | **停止吸球**（防止溢出） |
| `setGiveArtifact()` | `GiveTheArtifactVel`(2000) | 把球道内的球送入炮台 | **放行**（否则无法发射） |
| `setOutput()` | `OutputVel`(-500) | 反向出球 | 放行（负速不受限） |
| `setEject()` | `EjectVel`(-800) | **吐球**：排出多余、影响发射或机构运行的球 | 吐球优先，覆盖当帧其他命令 |

命令均为**每帧重新下发**：`update()` 用完即把 `targetVelocity` 与命令来源复位。

## 2. 吐球（原 BallEjector 已合并进本模块）

### 2.1 状态流转

```
IDLE ──setEject(now)──▶ EJECTING ──累计 ≥ EjectDurationSec──▶ IDLE
                          │  ▲
                          │  └── setEject(now) 重新计时
                          └───── stopEject() / setStop() ──▶ IDLE
```

- 吐球期间 `update()` 强制下发 `EjectVel`，忽略本帧吸球/送球命令；
- 到时自动结束，避免反向驱动无限持续；
- `setStop()` 会同时取消吐球（比赛结束收尾不会残留反转）。

### 2.2 接口

| 方法 | 说明 |
|---|---|
| `setEject()` | 触发一次吐球（内部单调时钟计时） |
| `stopEject()` | 立即停止吐球 |
| `isEjecting()` | 是否正在吐球 |
| `getEjectRemainingSec()` | 剩余时间（秒），非吐球为 0 |
| `setEject(double nowSec)` / `updateEject(double nowSec)` | 包内可见，供单元测试注入时间 |

### 2.3 参数（FTC Dashboard 可调）

| 参数 | 默认值 | 含义 |
|---|---|---|
| `Sweeper.EatVel` | 700 | 吸球速度 |
| `Sweeper.GiveTheArtifactVel` | 2000 | 送球速度 |
| `Sweeper.OutputVel` | -500 | 出球速度 |
| `Sweeper.EjectVel` | -800 | 吐球速度 |
| `Sweeper.EjectDurationSec` | 0.8 s | 单次吐球持续时间 |
| `Sweeper.BallMax` | 4 | 球道容量（吸球上限） |

## 3. 更新链路

```
OpMode（TeleOpRed/Blue、SweeperTester）
 ├─ Sweeper.setEat()/setGiveArtifact()/setOutput()/setEject()
 ├─ Sweeper.update()                 // 每帧：吐球计时 → 满球规则 → 下发速度/舵机 → 缓存采样
 └─ RobotPosition.update()
      └─ BallCounter.update() → BreakBeam.update()
```

- 吐球期间上层把球道计数方向设为 `MODE_OUTPUT`，排出的球逐个 -1；
- 送球/发射不受满球限制；发射完成由 `Turret` 清零；操作手可手动清空。

## 4. 操作方式

- **TeleOpRed / TeleOpBlue**：一操 `B` 吐球（瞬时按键），左/右保险杠吸球/出球，
  二操 `A` 手动清空计数；遥测显示 `Ball Count` 与 `Ejecting (剩余秒)`。
- **SweeperTester**：`A` 吸球、`B` 送球、`Y` 出球、`X` 停止（取消吐球）、
  `Right Bumper` 吐球；十字键控制铲子。

## 5. 边界与错误处理

| 场景 | 行为 |
|---|---|
| 吐球中再次触发 | 重新计时，不会提前结束 |
| `EjectDurationSec ≤ 0` | 下一次 `update` 立即结束，不会无限反转 |
| 吐球中收到吸球/送球命令 | 本帧被吐球覆盖，结束后恢复命令语义 |
| `RobotPosition` 未初始化 | 满球判断跳过，吐球不受影响 |
| 比赛结束 / 显式停止 | `setStop()` 取消吐球并归零 |

## 6. 测试

- `SweeperTest`（11 项）：满球停吸、满球送球放行、未满吸球、满球出球；
  吐球优先级、停止取消吐球、默认空闲、持续计时、到时自动结束、重入计时、零时长边界。
- `:TeamCode:compileDebugJavaWithJavac` + `:TeamCode:testDebugUnitTest` 全量通过。
