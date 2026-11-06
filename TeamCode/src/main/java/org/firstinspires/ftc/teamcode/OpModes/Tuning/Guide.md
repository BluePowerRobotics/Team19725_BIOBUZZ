
# 飞轮参数整定 OpMode 使用指南（SimpleShooter）

> SimpleShooter 分支已删除速度滤波与母线电压滤波；`Tune6.6 VelocityFilter`、
> `Tune6.7 BusVoltageFilter` 已随对应滤波器一起删除；对应章节已从本文移除。

本目录提供 **3 个** 可用的整定辅助 OpMode，全部只用「占空比输入 + 速度输出 + 母线电压」，
不需要测量 `R/L/Ke/Kt/J/B`。对应《飞轮系统控制理论研究报告》第 6 章，
控制律定义见 `doc/现在的控制系统.md`。

标定目标是 [`FlywheelParams`](../../utility/Shooter/FlywheelParams.java) 里的参数
（大球 `FlywheelParams.BIG`、小球 `FlywheelParams.SMALL`），**不再改 `PrototypeShooter` 的静态字段**。

| 编号 | 类名 | Driver Station 名称 | 标定对象 |
|---|---|---|---|
| 6.1 | `StepResponseTuner` | `Tune6.1 StepResponse` | `kS/kff/kp/ki`（模型式初值） |
| 6.3 | `RelayFeedbackTuner` | `Tune6.3 RelayFeedback` | `kp/ki`（临界比例度复核） |
| 6.5 | `FeedforwardTuner` | `Tune6.5 Feedforward` | `kS/kff`（多点稳态拟合） |



另有 `ShooterTuningTester`（开环逐级功率扫描，`kS/kff` 的第二种来源）与
`TuningLogger`（CSV 写入工具，不是 OpMode）。

> SimpleShooter 另删除 `Tune6.6/Tune6.7`；第 4 代也不使用 boost / 掉速状态机，故无需 `Tune6.4 DropDetection`。

---

## 通用说明

- **分组**：Driver Station 分组 `Tuning`，名称前缀 `Tune6.x`。
- **调参入口**：FTC Dashboard → Config。每个 OpMode 有自己的分组；被标定的参数在
  `FlywheelParams`（`BIG`/`SMALL`）分组里。
- **原始数据**：每次运行把采样写入 CSV：`<RC 外部存储>/tuning/<前缀>_<时间戳>.csv`，
  用 `adb pull` 取出后配合 `tools/tuning/` 的脚本分析（见 `tools/tuning/PLOT_GUIDE.md`）。
- **CSV 格式**：以 `#` 开头的注释行记录本次实验参数与结论，随后是表头行；
  默认表头 `t,duty,velocity`，新程序用更宽的表头（见各方法「输出」）。
  解析时 `skiprows` 跳过注释行、`comments='#'`（`numpy.loadtxt(..., comments='#')` /
  `pandas.read_csv(..., comment='#')`）。时间轴严格单调，单位秒。
- **速度单位**：统一 `DcMotorEx#getVelocity()` 的 **tick/s**。
- **量化台阶**：实测 **20 tick/s**。任何稳态精度/阈值讨论都必须先报「相当于几个台阶」；
  参数里 `quantStep=20` 就是它的固化值。若换了电机/齿比/滤波，必须重新测台阶。
- **安全**：飞轮转速高，测试前取走球与异物；开环测试务必让 `d0/dHigh/dLow` 在安全转速内。
  所有 OpMode 都可用 **X 键**立即停转。
- **参数污染**：6.1/6.3/6.5 会临时强改被测参数（如置 `kff=0`、`ki=0`）以隔离环节，
  **退出时自动恢复原值**。V4 起这些临时改动作用在 `FlywheelParams` 实例上，
  重启 OpMode 即恢复（若手动在 Dashboard 改过，注意别和整定值混在一起）。

---

## 方法一：阶跃响应辨识（`Tune6.1 StepResponse`）

> 一次给出 `Kd / τ / kff / kp / ki` 的模型式初值，是最快的起点。

### 原理

占空比 `d0` 稳定运行后阶跃到 `dStep`，速度响应近似
`ω(t) = ω0 + Kd·Δd·(1 − e^(−t/τ))`，读 `Kd = Δω/Δd`、`τ = t63.2%`，零极点对消得
`kff = 1/Kd`、`kp = τ/(Kd·τc)`、`ki = 1/(Kd·τc)`，`τc = tRecover/ratio`。

### 可调参数（Dashboard → StepResponseTuner）

| 参数 | 含义 | 建议 |
|---|---|---|
| `d0` | 阶跃前占空比 | 让飞轮稳定转起来（如 0.15） |
| `dStep` | 阶跃后占空比 | 与 d0 拉开明显差距（如 0.25） |
| `settleSec` | 阶跃前稳定时长 | 2 s |
| `recordSec` | 阶跃后记录时长 | 3 s（≥4τ） |
| `steadyWindowSec` | 稳态均值窗口 | 0.5 s |
| `tRecover` | 期望恢复时间 | 0.5 s |
| `ratio` | tRecover/τc | 3~5 |

### 操作

1. Start → 自动跑一次（`d0` 稳定 → 阶跃 → 记录 → 计算）。
2. **A** 重跑，**X** 停转。
3. 把建议值填进 `FlywheelParams.BIG`（注意：`kp` 见下方「注意」）。

### 输出

`Kd (tick/s per duty)`、`τ (s)`、`kff`、`kp`、`ki`。

### 注意

- **模型式给的 `kp` 一定偏小**：相角裕度/内模公式给出 6.1e-4~9.8e-4，
  而量化判据允许到 `0.05/quantStep = 0.0025`（当前 0.00182）。经验做法是
  **在模型值上乘 2~3 倍**，再用 `kp×台阶 ≤ 0.05` 卡上限、用继电法（6.3）交叉验证。
- `τ` 由 63.2% 上升时间估计，噪声大时多跑几次取中位数。
- `dStep` 与 `d0` 太接近、或飞轮未真正转起来，会判 invalid。

---

## 方法二：继电反馈自整定（`Tune6.3 RelayFeedback`）

> 自动产生极限环，比手动加 Kp 安全得多；是 `kp/ki` 的**第二种**整定方式。

### 原理

用继电器代替控制器：`err>0 → dHigh`，`err<0 → dLow`。进入极限环后测幅值 `a`、周期 `Tu`，
则

```text
Ku = 2·Δd / (π·a)        Δd = dHigh − dLow，a = 极限环峰值（相对设定点）
Kp = 0.45·Ku ,  Ki = 0.54·Ku/Tu
```

> **旧数据修正**：2026-10-22 之前本程序写作 `Ku = 4·Δd/(π·a)`，把全幅 `Δd` 当成半幅使用，
> 结果高估 2 倍（样例 `dHigh=0.7, dLow=0.5, a=165` → 旧值 1.543e-3，正确值 7.716e-4）。
> **凡是此前用旧版本跑出的 `Ku/Kp/Ki` 都要除以 2 再参考。**

### 可调参数（Dashboard → RelayFeedbackTuner）

| 参数 | 含义 | 建议 |
|---|---|---|
| `targetVelocity` | 目标转速 | 典型工作转速（2050） |
| `dHigh` / `dLow` | 继电器输出上下限 | 让稳态占空比落在两者之间（如 0.7 / 0.5） |
| `transientSec` | 跳过建立极限环的暂态 | 2 s |
| `measureSec` | 测量时长 | 4 s（≥数个周期） |
| `hysteresis` | 继电器滞环 (tick/s) | 默认 0；噪声大可略加（建议 ≤ 台阶） |

### 操作

1. Start → 自动跑一次（建立极限环 → 测量 → 计算）。
2. **A** 重跑，**X** 停转。
3. 读建议 `Ku/kp/ki`。

### 输出

幅值 `a`、`Tu (s)`、`Ku`、建议 `kp`、建议 `ki`。

### 注意

- `Tu` 有效的前提是稳定极限环（`transientSec` 足够长）。
- `a` 太小时 `Ku` 会被噪声放大，判 invalid：拉开 `dHigh/dLow` 后重试。
- 继电法给的是「临界增益」，对飞轮这种带饱和的执行器通常再取 `0.3~0.45×Ku` 更稳。

---

## 方法三：无模型前馈增益标定（`Tune6.5 Feedforward`）

> `kS` 与 `kff` 的**首选**方法；也是判断「积分是否长期负重」的唯一现场手段。

### 原理

闭环稳定运行时记录稳态占空比 `d_ss`，由
`d_ss(ω*) = kS + kff·ω*`
**多点线性拟合**：斜率 = `kff`、截距 = `kS`。本 OpMode 强制 `kff=0`、`kS=0`，
由 PI 提供全部稳态占空比，避免前馈把读数掩盖掉。

### 可调参数（Dashboard → FeedforwardTuner）

| 参数 | 含义 | 建议 |
|---|---|---|
| `targetVelocity` | 初始目标转速 | 典型工作转速（1700） |
| `velocityStep` | 目标转速步长 | 200 tick/s |
| `settleSec` | 测量前稳定时长 | **≥2 s**（短了会吃到暂态） |
| `windowSec` | 取均值窗口 | 1.5 s |

### 操作

1. **前置**：`kp/ki` 已整定（非 0），否则根本到不了目标转速。
2. Start 后用 **DPad 上/下** 设定目标转速；**每档至少停 2 s**（历史教训：
   0.57~0.62 s 的停留把 `kS` 高估到 0.13）。
3. 速度稳定后按 **A** 记录一点（自动等 `settleSec` 后取 `windowSec` 均值）。
4. 覆盖 **1700 / 2050 / 2250 各 3~4 点**，共 9~12 点。
5. 遥测显示 `Kff (origin fit, only if kS=0)`（过原点斜率）与 `Kff (fit) / kS (intercept)`
   （含截距拟合，**推荐**）。**B** 清空测量点。

### 输出

每点 `ω / d_ss / Kff`、过原点斜率、线性拟合斜率与截距（= kS）。

### 注意

- **单点拟合是高估陷阱**：`kS≈0.03` 时在 1700 tick/s 只用一点过原点拟合，会把 `kff` 高估约 4.6%。
  必须多点、必须拟合斜率。
- 稳态占空比接近 1（饱和）的点无效，降低目标转速重测。
- 若截距为负或 >0.06，说明机械/电气状态变了（脏污、轴承、电池），先查机械再改参数。

---

## 推荐整定顺序

1. **准备**：取走异物；确认 `quantStep` 仍是 20（换过电机/齿比就重测）。
2. **稳态前馈**（6.5）：≥9 点重标 `kS`、`kff`。**这一步决定积分是否长期负重，必须先做。**
3. **快速性**（6.1 → 6.3）：6.1 给模型初值（`kp` 乘 2~3），6.3 给临界增益复核，
   最终 `kp` 用 `kp×台阶 ≤ 0.05` 卡上限、用实测 `duty_sd` 定稿。
4. **发射实测**：带球 10~15 发，看「跌幅 / 恢复时间 / 过冲 / 饱和占比 / 积分回落」。
5. **两种电量对比**：满电 12.9~13.1 V 与放电 ~10.5 V，同目标各 30 s，`duty·V_bus` 差 ≤3%。

---

## 参数速查

| `FlywheelParams` 参数 | 单位 | 来源 |
|---|---|---|
| `kS` | 占空比 | 6.5（拟合截距）、`ShooterTuningTester`（开环截距） |
| `kff` | 占空比/(tick/s) | 6.5（拟合斜率）、6.1（=1/Kd） |
| `kp` | 占空比/(tick/s) | 6.1×2~3、6.3（0.45·Ku）、量化上限 `0.05/quantStep` |
| `ki` | 占空比/(tick/s·s) | 6.1（=1/(Kd·τc)）、6.3（0.54·Ku/Tu） |
| `maxI` | 占空比 | 经验值，防积分饱和（0.3） |
| `iZone` | tick/s | 积分门限，BIG / SMALL 独立整定 |
| `vBusNominal` | V | 电池额定（12.0；满电实测 12.9~13.1） |
| `settleFrac` / `settleFrames` | — | 算出来的，不整定（2% / 10 帧） |

> 全部增益单位为「占空比」。第 4 代的电压归一化只乘前馈，
> 因此**增益量纲不变**，不需要像「整条输出 × V_nom/V_bus」那样换算成 V 量纲。

---

---

## 附录 B：分代整定程序（不在本目录）

历史各代飞轮控制律的整定程序**故意不放在这里**，而是与它们各自的计算器放在一起：
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/pastcalculater/`

| 程序 | 整定对象 | 阶段 |
|---|---|---|
| `TuneGen1Op` | 第 1/2 代 PIDSVA（`Gen1Params`） | ①开环多点 → `kS`(截距)/`kV`(斜率)；②开环阶跃 → `kP`/`kI` |
| `TuneGen3Op` | 第 3 代 boost（`Gen3Params`） | ①②同上给 `kS`/`kff`/`kp`/`ki`；③人为掉速 → 判读并给 `kBoost` 建议 |
| `TuneGen4Op` | 第 4 代（`Gen4Params`） | **不做辨识**，只按 §2.12 闭环验收（稳态量 + 掉速恢复 + `isSettled` 行为） |

为什么分开放：这三份要共用 `utility/pastcalculater` 的 `PastCalculator` / `PastFlywheelDriver` /
`PastTuneSupport`，而本目录（`OpModes/Tuning`）的 3 个 Tuner 只服务**当前代**、
参数唯一出处是 `FlywheelParams`。两套参数容器不要混：`FlywheelParams` 是实例字段（Dashboard 改不动），
`GenXParams` 是全 `static`（Dashboard 能改）。用法见 `pastcalculater/GUIDE.md`。

SimpleShooter 整定仍走本目录：`kS/kff → 6.5`、`kp/ki → 6.1(+6.3 复核)`、`iZone` 按闭环表现直接调；
`TuneGen4Op` 只用于「改完参数后确认闭环达标」。

