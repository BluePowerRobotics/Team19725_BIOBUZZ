# Shooter 工具库目录说明

## 概述

本目录存放**发射机构（飞轮）**相关的纯计算类。这些类不持有任何硬件引用、不读遥测、不依赖 `Controllers/`，
只做「输入 → 输出」的数值计算，因此可以离线复算与单独验证。

控制器（电机、硬件初始化、遥测、参数面板）位于
`Controllers/Shooter/PrototypeShooter/PrototypeShooter.java`，它每帧把参数同步给本目录的计算器。

## 文件说明

| 文件 | 功能说明 |
|------|----------|
| `FlywheelCalculator.java` | 飞轮速度环计算器：`(目标转速 ω*, 当前转速 ω, dt) → 占空比 duty`。含静摩擦前馈 + 稳态前馈 + PI 反馈 + 掉速检测触发补偿，以及积分抗饱和、掉速检测「武装」规则等状态机 |

## 控制律

```
duty = kS·sgn(ω*) + Kff·ω* + Kp·err + Ki·∫err + boost
err  = ω* - ω
duty = sat(duty, dutyMin, dutyMax)
```

- 单位统一为 `DcMotorEx#getVelocity()` 的 tick/s；所有增益量纲为「占空比」（不读取电压传感器）。
- `kS`（静摩擦前馈）：稳态实测占空比与转速的关系不过原点，而是 `d_ss(ω) = kS + kff·ω`。
  `kS` 是静摩擦、同步带预紧、齿槽转矩对应的那份占空比（只对抗摩擦、不产生转速），按 `kS·sgn(ω*)` 施加；
  `sgn` 不可省——`ω* = 0` 时必须为 0，否则停转指令下飞轮会自己转起来。
  不拆 `kS` 的后果：低转速欠给、高转速过给，差额全压给积分项，表现为积分长期停在非零值。
- `boost = kBoost·err` 仅在检测到掉速时生效（限幅 `[0, boostMax]`，持续 `boostDurationSec` 后退出）。
- I 项：仅当目标非 0 且 `|err| < iZone` 时累加；停机、误差带外、触发补偿时清零；积分项限幅 ±`maxI` 并回算积分（抗饱和）。

## 使用方式

### 1. 由控制器调用（常规）

```java
PrototypeShooter shooter = new PrototypeShooter(hardwareMap, telemetry);
shooter.setTargetVelocity(1700);
// 每帧一次
shooter.update();        // 内部 syncCalculator() 后调用 calculator.calculate(target, current)
```

参数留在 `PrototypeShooter` 的 `@Config` 静态字段上（FTC Dashboard 分组不变），
由 `PrototypeShooter#syncCalculator()` 每帧同步给计算器 —— **新增参数时两处都要改**。

### 2. 离线复算（显式 dt）

```java
FlywheelCalculator calc = new FlywheelCalculator();
calc.kS = 0.02945; calc.kff = 0.00038307;
calc.kp = 0.00181917; calc.ki = 0.00259224;
calc.reset();
for (int i = 0; i < n; i++) {
    double duty = calc.calculate(target[i], velocity[i], 0.0098); // 固定步长，可重复
}
```

三参数版本 `calculate(target, velocity, dt)` 不读写内部计时器，适合回归测试与整定数据复算；
两参数版本 `calculate(target, velocity)` 自行用单调时钟推进 dt（异常时用 `DEFAULT_DT = 0.02 s` 兜底）。

### 3. 重置

从开环标定切回闭环前调用 `reset()` / `reset(currentVelocity)`，清积分、清补偿状态、解除武装并刷新计时器。

## 注意事项

1. **每帧只调用一次** `calculate(...)`：它依赖内部计时器推进 dt、维护积分与补偿状态。
2. **掉速检测武装规则**：只有误差回到 `dropThreshold` 以内（进入稳态）才允许触发补偿，目标一改立即解除武装 ——
   这样从 0 拉起时的正常加速段不会被误判成掉速。
3. **量化台阶**：速度反馈存在量化台阶（本机 20 tick/s）。`kp × 一个台阶` 应远小于 0.05，
   否则反馈跳一格就会让占空比大幅跳变，出现抖振（历史故障值 0.381）。
4. **停转语义**：`targetVelocity = 0` 时前馈为 0、积分清零、占空比收到 `dutyMin`，飞轮靠 `FLOAT` 惯性滑行。
