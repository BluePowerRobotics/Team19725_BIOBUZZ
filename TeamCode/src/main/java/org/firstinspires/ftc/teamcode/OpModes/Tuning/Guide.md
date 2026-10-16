# 飞轮无模型标定 OpMode 使用指南

本目录提供 5 个调参辅助 OpMode，对应《飞轮系统控制理论研究报告》第 6 章
「无电气/机械参数标定控制参数的方法」。全部方法只利用**占空比输入 + 速度输出**，
无需测量 `R/L/Ke/Kt/J/B` 等参数。

标定的目标对象是 [PrototypeShooter.java](../../Controllers/Shooter/PrototypeShooter/PrototypeShooter.java)
的参数：`kff / kp / ki / maxI / iZone / dropThreshold / dropRateThreshold / kBoost / boostDurationSec`。

---

## 通用说明

- **分组**：所有 OpMode 的 Driver Station 分组为 `Tuning`，名称前缀 `Tune6.x`。
- **调参入口**：FTC Dashboard → Config。每个 OpMode 有自己的分组；`PrototypeShooter` 参数在
  `PrototypeShooter` 分组里（方法二需要在这里改 `kp`）。
- **原始数据**：每个 OpMode 都会把 `(时间, 占空比, 转速)` 采样写入 CSV：
  `<RC 外部存储>/tuning/<方法名>_<时间戳>.csv`，可用 `adb pull` 或 RC 文件管理器取出，
  配合报告第 6 章的 Python 脚本离线复核。方法一会把 CSV 路径显示在遥测里。
- **CSV 格式**：文件以 `#` 开头的注释行记录本次实验的参数与结论，随后是表头 `t,duty,velocity`；
  数据列序固定为「时间(s), 占空比, 转速(tick/s)」，时间轴严格单调。解析时跳过 `#` 行即可
  （`numpy.loadtxt(..., comments='#', skiprows=1)` 或 `pandas.read_csv(..., comment='#')`）。
- **掉速检测的「武装」规则**：`PrototypeShooter` 只有在误差回到 `dropThreshold` 以内（即已进入稳态）
  之后才允许触发掉速补偿，目标转速一旦改变立刻重新解除武装——否则从 0 拉起时的正常加速段会被
  误判为掉速，使 boost 全程挂着并在到速后造成过冲。因此方法四测量「正常波动」前，务必先让系统真正到速。
- **速度单位**：统一为 `DcMotorEx#getVelocity()` 的 **tick/s**（对应报告里的 ω）。
- **安全**：飞轮转速高，测试时先取下小球/异物；开环测试（方法一/三）务必保证 `d0/dHigh/dLow`
  在安全转速范围内。所有 OpMode 都可用 **X 键**立即停转。
- **参数污染**：方法二/四/五会临时强改 `PrototypeShooter` 的静态增益（如 `kff/ki` 与掉速阈值）
  以隔离被测环节，OpMode **退出时会自动恢复原值**，不会影响同一上电会话中后续 OpMode。
  唯一例外：方法二对 `kp` 的调节是用户主动整定行为，退出时**不还原**（便于直接沿用）。

---

## 方法一：阶跃响应辨识（`Tune6.1 StepResponse`）

> 报告 6.1，最推荐。一次性给出 `Kd / τ / kff / kp / ki`。

### 原理
以占空比 `d0` 稳定运行后阶跃到 `dStep`，速度响应近似
`ω(t) = ω0 + Kd·Δd·(1 - e^(-t/τ))`，据此读 `Kd = Δω/Δd`、`τ = t63.2%`，再按零极点对消算：
`kff = 1/Kd`，`kp = τ/(Kd·τc)`，`ki = 1/(Kd·τc)`，其中 `τc = tRecover/ratio`。

### 可调参数（Dashboard → StepResponseTuner）
| 参数 | 含义 | 建议 |
|---|---|---|
| `d0` | 阶跃前占空比 | 让飞轮稳定转起来（如 0.15） |
| `dStep` | 阶跃后占空比 | 与 d0 拉开明显差距（如 0.25） |
| `settleSec` | 阶跃前稳定时长 | 2 s |
| `recordSec` | 阶跃后记录时长 | 3 s（≥ 4τ） |
| `steadyWindowSec` | 取稳态均值窗口 | 0.5 s |
| `tRecover` | 期望恢复时间 | 0.5 s |
| `ratio` | tRecover/τc | 3~5 |

### 操作
1. Init 后 Start → **自动跑一次**（d0 稳定 → 阶跃 → 记录 → 计算）。
2. 按 **A** 重跑；按 **X** 停转。
3. 读取遥测的「建议参数」，填入 `PrototypeShooter`。

### 输出
`Kd (tick/s per duty)`、`τ (s)`、`kff`、`kp`、`ki`。

### 注意
- 若 `dStep` 与 `d0` 太接近或飞轮未真正转起来，结果会判为 invalid。
- `τ` 由 63.2% 上升时间估计，采样噪声大时可多跑几次取中位数。

---

## 方法二：Ziegler-Nichols 临界比例度法（`Tune6.2 ZieglerNichols`）

> 报告 6.2。手动增大 Kp 直到等幅振荡，读出 `Ku / Tu`。

### 原理
P-only 控制 `d = Kp·(ω*-ω)`（OpMode 强制 `kff=0、ki=0` 并关闭掉速补偿）；
增大 Kp 至等幅振荡，则当前 Kp 即 `Ku`，振荡周期为 `Tu`。ZN 表：
`Kp = 0.45·Ku`，`Ki = 0.54·Ku/Tu`。

### 可调参数（Dashboard → ZieglerNicholsTuner）
| 参数 | 含义 | 建议 |
|---|---|---|
| `targetVelocity` | 目标转速 | 典型工作转速 |
| `minZeroCrossings` | 判定周期的最少过零次数 | 4 |
| `warmupSec` | 改变 kp 后跳过暂态、不采样的时长 | 3 s |

### 操作
1. Start 后飞轮以 P-only 运行。
2. **DPad 上/下** 把 `PrototypeShooter.kp` ×1.1 / ÷1.1（或直接在 Dashboard 的
   `PrototypeShooter` 分组里改），直到出现**不衰减的等幅振荡**。
3. 观察遥测里的 `osc amp`、`Tu`；确认稳定后按 **A** 锁定：以当前 kp 作为 `Ku`，
   用缓冲估出的 `Tu` 计算建议 `kp/ki`。
4. 按 **B** 清除锁定结果与采样缓冲。

### 输出
`Ku`、`Tu (s)`、建议 `kp`、建议 `ki`。

### 注意
- `Ku` 与 `kp` 同量纲（占空比/(tick/s)），`ki` 单位为占空比/(tick/s·s)，可直接填入。
- 每次 kp 变化（手柄或 Dashboard 直接改）都会**清空采样缓冲并重新等待 `warmupSec`**，避免把上一次
  kp 的响应和启动峰值算进幅值/周期；遥测显示 `(settling)` 时不要读振荡结果。
- 若只有稳态误差、无振荡，说明 Kp 还不够大；若振荡幅值失控，立即 X 停转。
- 锁定前请确认 `osc amp` 近似恒定（等幅）。

---

## 方法三：继电反馈自整定（`Tune6.3 RelayFeedback`）

> 报告 6.3。自动产生极限环，比方法二楼安全。

### 原理
用继电器代替控制器：`err>0 → dHigh`，`err<0 → dLow`。系统进入极限环，
测幅值 `a` 与周期 `Tu`，则 `Ku = 4·Δd/(π·a)`（`Δd = dHigh-dLow`），
再 `Kp = 0.45·Ku`、`Ki = 0.54·Ku/Tu`。

### 可调参数（Dashboard → RelayFeedbackTuner）
| 参数 | 含义 | 建议 |
|---|---|---|
| `targetVelocity` | 目标转速 | 典型工作转速 |
| `dHigh` / `dLow` | 继电器输出上下限 | 让稳态占空比落在两者之间（如 0.7 / 0.5） |
| `transientSec` | 跳过建立极限环的暂态 | 2 s |
| `measureSec` | 测量时长 | 4 s（≥ 数个周期） |
| `hysteresis` | 继电器滞环 (tick/s) | 默认 0；信号噪声大可略加 |

### 操作
1. Init 后 Start → **自动跑一次**（建立极限环 → 测量 → 计算）。
2. 按 **A** 重跑；按 **X** 停转。
3. 读取建议 `Ku/kp/ki`。

### 输出
幅值 `a`、`Tu (s)`、`Ku`、建议 `kp`、建议 `ki`。

### 注意
- `Tu` 有效的前提是已进入稳定的极限环（跳过 `transientSec` 足够长）。
- 若 `a` 太小或周期无效，结果判为 invalid：调大 `dHigh/dLow` 差距后重试。

---

## 方法四：掉速检测与补偿参数标定（`Tune6.4 DropDetection`）

> 报告 6.4。标定 `dropThreshold / dropRateThreshold / kBoost / boostDurationSec`。

### 原理
让系统闭环稳定运行（本 OpMode **关闭掉速补偿**以测纯正常波动），统计：
`Δω_th = kSigma·σ_ω`（σ_ω 为**样本标准差**，÷ N-1）；
`k_th = kRate·max(-dω/dt)`（仅取**速度下降**的分量即最大减速度）；
`kBoost = kBoostRatio·kp`；`boostDurationSec` 建议 0.1~0.5 s（发射实测微调）。

### 可调参数（Dashboard → DropDetectionTuner）
| 参数 | 含义 | 建议 |
|---|---|---|
| `targetVelocity` | 目标转速 | 典型工作转速 |
| `settleSec` | 测量前稳定时长 | 2 s |
| `measureSec` | 测量时长 | 5 s |
| `kSigma` | σ 的倍数 | 3~5 |
| `kRate` | 最大减速度倍数 | 2~3 |
| `kBoostRatio` | kBoost/kp 倍数 | 2~5 |
| `suggestBoostDurationSec` | 补偿时长建议值 | 0.3 s |

### 操作
1. **前置**：先把方法一/二/三整定好的 `kff/kp/ki` 填入 `PrototypeShooter`。
2. Init 后 Start → **自动跑一次**；按 **A** 重跑；**X** 停转。
3. 读取建议参数并填入 `PrototypeShooter`。

### 注意
- `kBoost` 由当前 `PrototypeShooter.kp` 按倍数给出，因此需先定好 kp。
- 遥测会同时给出速度**量化台阶**（实测速度的最大公约数）与 σ 相当于几个台阶。若
  `dropThreshold` 只相当于 1~2 个台阶，说明阈值被量化噪声主导，需保守取值或提高目标转速——
  任何稳态精度/阈值的讨论都必须先报出台阶。
- `maxDecel` 先对速度做 EMA 平滑再差分，避免相邻样本的读数噪声把 `dropRateThreshold` 抬得过高。
- 本方法只统计「正常工况」波动；真实掉速（发射）后的恢复需用带补偿的系统实测微调 `kBoost/boostDurationSec`。

---

## 方法五：无模型前馈增益标定（`Tune6.5 Feedforward`）

> 报告 6.5。标定 `kff`。

### 原理
闭环稳定运行时记录稳态占空比 `d_ss`，则 `Kff = d_ss/ω*`。本 OpMode **强制 kff=0**，
由 PI 提供全部稳态占空比，避免前馈饱和导致读数失真。可在多个转速下各测一点做线性拟合。

### 可调参数（Dashboard → FeedforwardTuner）
| 参数 | 含义 | 建议 |
|---|---|---|
| `targetVelocity` | 初始目标转速 | 典型工作转速 |
| `velocityStep` | 目标转速步长 | 200 tick/s |
| `settleSec` | 测量前稳定时长 | 2 s |
| `windowSec` | 取均值窗口 | 1.5 s |

### 操作
1. **前置**：`kp/ki` 必须已整定（非 0），否则系统到不了目标转速。
2. Start 后，用 **DPad 上/下** 设定目标转速。
3. 待速度稳定后按 **A** 记录一点（自动等待 `settleSec` 后取 `windowSec` 均值）。
4. 改变目标转速，重复步骤 3，记录多点。
5. 遥测显示 `Kff (through origin)`（过原点斜率，推荐）与 `Kff (linear fit slope)`（含截距拟合，
   可反映静摩擦）。按 **B** 清空测量点。

### 输出
每点的 `ω / d_ss / Kff`、过原点斜率 Kff、线性拟合斜率与截距。

### 注意
- 若稳态占空比接近 1（饱和），该点无效，请降低目标转速。
- 过原点斜率适合直接填入 `kff`；若拟合截距明显非 0，说明存在静摩擦，可考虑用截距补偿 `kS`（当前控制器未含该项）。

---

## 推荐标定顺序（报告 6.6）

1. **准备**：取下异物，确保飞轮可安全低速运行。
2. **阶跃辨识**（方法一）→ 得到 `Kd / τ` 及初版 `kff / kp / ki`，填入 `PrototypeShooter`。
3. **PI 微调**：若阶跃法结果不理想，用方法二（ZN）或方法三（继电）复核 `kp / ki`，取较保守者。
4. **前馈标定**（方法五）→ 用实测稳态占空比修正 `kff`。
5. **掉速参数**（方法四）→ 得到 `dropThreshold / dropRateThreshold / kBoost`，`boostDurationSec` 先取 0.3 s。
6. **发射实测**：真实发射小球，观察掉速与恢复，微调 `kBoost / boostDurationSec`。

---

## 参数速查

| PrototypeShooter 参数 | 单位 | 来源方法 |
|---|---|---|
| `kff` | 占空比/(tick/s) | 6.1（=1/Kd）、6.5（=d_ss/ω*） |
| `kp` | 占空比/(tick/s) | 6.1、6.2、6.3 |
| `ki` | 占空比/(tick/s·s) | 6.1、6.2、6.3 |
| `maxI` | 占空比 | 经验值（防积分饱和） |
| `iZone` | tick/s | 经验值（积分生效误差带） |
| `dropThreshold` | tick/s | 6.4 |
| `dropRateThreshold` | tick/s² | 6.4 |
| `kBoost` | 占空比/(tick/s) | 6.4（=倍数·kp） |
| `boostDurationSec` | s | 6.4 + 发射实测 |

> 队列中的增益单位均为「占空比」，因此本套方法与 `PrototypeShooter` 不读取电压传感器一致。
> 若后续改为电压补偿（`d=V/Vdc`），需把增益改为 V 量纲并乘以 `Vdc`。
