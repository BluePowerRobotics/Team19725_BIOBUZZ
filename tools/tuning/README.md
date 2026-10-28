# 飞轮标定数据离线分析（tools/tuning）

把 RC 上导出的标定 CSV 与 FTC Dashboard 遥测日志变成**图 + 复算过的参数**。
与机器人端使用**同一套公式**，因此离线结果可以和端上遥测互相印证；离线还能多做两件端上做不到的事：
最小二乘拟合 τ（而不是只读 t63.2%）和多点回归 Kff（而不是只取单点）。

对应《飞轮系统控制理论研究报告》第 6 章；端上怎么标定见
[`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Tuning/Guide.md`](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Tuning/Guide.md)。

> **看图先读 [PLOT_GUIDE.md](PLOT_GUIDE.md)** —— 每个脚本产出的每一张 PNG 的文件名、横纵轴、
> 看什么、判据「好/合格/差」三档数字、常见误判、按参数整定的查图表，都在那里。
> 本 README 只讲脚本用途、命令与数据格式。

---

## 0. 脚本一览

| 脚本 | 输入 | 干什么 | 图片产物（一图一文件） |
|---|---|---|---|
| `analyze_tuning.py` | Tune6.x 标定 CSV（`t,duty,velocity`） | 五种方法的参数复算 + 报告 | 每个输入 2 张（见 §2） |
| `analyze_tuning.m` | 同上 | MATLAB 版，同口径同图名（对照用） | 同上 |
| `analyze_dashboard_log.py` | FTC Dashboard 原始 CSV | 清洗 + 分段统计 + 诊断 | 9 张（见 §3） |
| `analyze_step_response.py` | clean 表 | 多次阶跃的稳态/恢复/超调汇总 | 4 张 |
| `analyze_launch_events.py` | FTC Dashboard 原始 CSV | 发射掉速逐发分析 | 每发 1 张 + 3 张统计 + 1 份事件 CSV |
| `plot_speed.m` | clean 表 | MATLAB 转速专图 + 诊断图 | 6 张 |
| `plot_flywheel.py` | `flywheel/*.csv` 台架表 | 三阶段开环/闭环横向对比 | 10 张 |
| `plot_csv.py` | **任意 CSV**（Dashboard 原始 / Dashboard Logging 导出 / 旧的 `TuningLogger` 文件） | 通用兜底：列名归一化 + 时间轴修复 + 逐列统计，**不假设格式** | **每列 1 张 + 1 份 `*_summary.txt`** |
| `export_clean_table.py` | 原始日志 | 生成 `<名字>_clean.csv`（按行序重建时间轴） | 无（只出表） |
| `fit_steady_state_model.py` | clean 表 | 拟合 `d_ss = kS + kff·ω` | 无（只出表/文本） |
| `make_samples.py` | 无 | 生成 5 组真值已知的合成 CSV（自检用） | 无 |
| `check_plot_data.py` | `out/` 下的表与图 | 数据口径一致性自检（与 MATLAB 统计交叉核对） | 无 |

| 方法 | OpMode | 文件名前缀 | 输出 |
|---|---|---|---|
| 6.1 阶跃响应辨识 | `Tune6.1 StepResponse` | `step01_*.csv` | Kd、τ、kff、kp、ki |
| 6.2 Ziegler-Nichols | `Tune6.2 ZieglerNichols` | `zn02_*.csv` | Ku、Tu、kp、ki |
| 6.3 继电反馈 | `Tune6.3 RelayFeedback` | `relay03_*.csv` | a、Tu、Ku、kp、ki |
| 6.4 掉速检测 | `Tune6.4 DropDetection` | `drop04_*.csv` | σ、量化台阶、maxDecel、Δω_th、k_th、kBoost |
| 6.5 前馈标定 | `Tune6.5 Feedforward` | `ff05_*.csv` | 多点回归 Kff（斜率）+ kS（截距） |

> **第 4 代现状**：`Tune6.2 ZieglerNichols` 与 `Tune6.4 DropDetection` 的 OpMode **已删除**
> （boost 已从控制律移除、6.2 与 6.3 方法重复），`analyze_tuning.py` 仍保留这两种方法的解析，
> 只为**复核历史 `zn02_/drop04_` CSV**。现行 5 个 Tuner 见
> [`OpModes/Tuning/Guide.md`](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Tuning/Guide.md)。
> 新增的 `Tune6.6 VelocityFilter` / `Tune6.7 BusVoltageFilter` 的 CSV 前缀为 `vf06_` / `bf07_`。

---

## 1. 取数据

**第 4 代的验证工具（`FlywheelRun` / `VoltageSweep` / `PastCompare` / `TuneGen1/3/4`）不写自定义
CSV**：它们把每列数据作为扁平遥测键发出（各 OpMode 已绑 `MultipleTelemetry`），
在 **FTC Dashboard 的 Logging 面板**点 Start/Stop 记录，再在面板里下载 CSV 即可
（列名 = 遥测键名，时间列由 Dashboard 加）。

只有旧的 Tune6.x 标定 OpMode 仍把 CSV 写到 RC 外部存储的 `tuning/` 目录：

```
adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/tuning ./tuning_data
```

（或用 RC 上的文件管理器 / USB 拷贝；方法一还会在遥测里显示完整路径。）

FTC Dashboard 日志与台架表另行导出，放到任意目录即可。

---

## 2. Python 版（主用）：Tune6.x 标定 CSV

依赖：Python 3 + numpy + matplotlib（本机已具备：3.11.5 / 2.2.6 / 3.10.8；**不需要 scipy**）。

```bash
python tools/tuning/analyze_tuning.py ./tuning_data                 # 分析目录下全部 CSV（含子目录）
python tools/tuning/analyze_tuning.py step01_1700.csv --out ./out   # 单个文件 + 指定输出目录
python tools/tuning/analyze_tuning.py ./tuning_data --no-plot       # 只出报告（无图形环境）
python tools/tuning/analyze_tuning.py zn02_1.csv --ku 9e-5          # 手工指定 Ku
python tools/tuning/analyze_tuning.py drop04_1.csv --kp 5e-5        # 计算 kBoost 需要 kp
```

目录里混入非标定 CSV（比如 Dashboard 日志）时，脚本会打印
`[跳过] 不是 t,duty,velocity 三列标定 CSV：…` 并忽略，不会报错。

每个输入产出（**图名按方法不同**）：

| 方法 | 图 |
|---|---|
| 6.1 `step01` | `<名字>_step_velocity.png`（转速 + 拟合 + t63.2%）、`<名字>_step_duty.png`（开环阶跃） |
| 6.2 `zn02` | `<名字>_velocity.png`（尾部窗 + ±a）、`<名字>_duty.png` |
| 6.3 `relay03` | `<名字>_velocity.png`（极限环）、`<名字>_duty.png`（dHigh/dLow） |
| 6.4 `drop04` | `<名字>_velocity.png`、`<名字>_duty.png` |
| 6.5 `ff05` | `<名字>_velocity.png`、`<名字>_ff_regression.png`（`d_ss(ω)` 回归） |

外加 `<名字>_report.txt`（结论文本，同时打印到终端）。

---

## 3. Python 版：Dashboard 日志 / 阶跃 / 发射 / 台架

```bash
# ★ 通用兜底（第 4 代新增）：任意 CSV → 每列一张图 + 汇总表，不用先判断格式
python tools/tuning/plot_csv.py .\FlywheelRun_log.csv --out ./tools/tuning/out
python tools/tuning/plot_csv.py cmp.csv --group gen            # 多代对比：图内按代分线
python tools/tuning/plot_csv.py sweep.csv --segment segment_id # 电压扫描：分段着色 + 逐段表

# Dashboard 原始日志 → 清洗 + 分段统计 + 9 张图（--tail 稳态窗口，默认 1.0 s）
python tools/tuning/analyze_dashboard_log.py ./raw/log4.csv --out ./tools/tuning/out

# 原始日志 → clean 表（后面三个脚本的输入）
python tools/tuning/export_clean_table.py ./raw/log4.csv --out ./tools/tuning/out/clean

# 多次阶跃汇总（--band-frac 0.02 / --band-steps 2.0 / --tail 1.0）
python tools/tuning/analyze_step_response.py tools/tuning/out/clean/log4_clean.csv --out tools/tuning/out

# 发射掉速逐发（吃**原始日志**，不能喂 clean 表；--target-band 0.02 / --min-target 500）
python tools/tuning/analyze_launch_events.py ./raw/log4.csv --out ./tools/tuning/out

# 台架三阶段对比（--window 稳态窗口，默认 2.0 s）
python tools/tuning/plot_flywheel.py ./flywheel_data --out ./tools/tuning/out
```

产物图名（`<stem>` = 输入文件名去扩展名）：

| 脚本 | 图 |
|---|---|
| `analyze_dashboard_log.py` | `<stem>_velocity.png` `<stem>_error.png` `<stem>_duty.png` `<stem>_current.png` `<stem>_voltage.png`（仅当日志带 voltage 列）`<stem>_zoom_velocity.png` `<stem>_zoom_duty.png` `<stem>_scatter_p_vs_error.png` `<stem>_scatter_duty_vs_velocity.png` |
| `analyze_step_response.py` | `<stem>_step_velocity.png` `<stem>_step_duty.png` `<stem>_step_current.png` `<stem>_step_aligned.png` + `<stem>_step_report.txt` |
| `analyze_launch_events.py` | `<stem>_launch_velocity.png` `<stem>_launch_duty.png` `<stem>_launch_boost.png` `<stem>_launch_001.png…_NNN.png`（每发一张）`<stem>_launch_stats_depth_vs_recover.png` `<stem>_launch_stats_depth_hist.png` `<stem>_launch_stats_recover_hist.png` + `<stem>_launch_events.csv`（事件总表）+ `<stem>_launch_report.txt` |
| `plot_flywheel.py` | `flywheel_compare_{velocity,duty,voltage}.png`、`flywheel_sweep_{velocity,voltage,kd}.png`、`flywheel_bench_{velocity,sd,duty,voltage}.png` |

---

## 4. MATLAB 版（对照）

```matlab
cd tools/tuning
analyze_tuning('C:\path\to\tuning_data')                       % 目录
analyze_tuning('step01_1.csv','zn02_1.csv')                    % 多文件
analyze_tuning(DIR, '-out', OUTDIR, '-noplot')                 % 只出报告
analyze_tuning(F, '-window', 4, '-ku', 9e-5, '-kp', 5e-5)      % 覆盖分析参数
plot_speed('C:\path\to\log4_clean.csv', 'C:\out')              % 转速专图 + 诊断图
```

选项：`-out DIR`、`-noplot`、`-window S`、`-ku K`、`-kp K`、`-alpha A`。

- 输出**图名与 Python 版完全一致**，便于逐张对照：
  `step01` → `_step_velocity` / `_step_duty`；`zn02`/`relay03`/`drop04` → `_velocity` / `_duty`；
  `ff05` → `_velocity` / `_ff_regression`。
- `plot_speed.m` 另外用 `_speed_` / `_diag_` 区分图组：
  `_speed_velocity.png` `_speed_error.png` `_diag_velocity.png` `_diag_error.png`
  `_diag_duty.png` `_diag_current.png`，统计文本写 `plot_speed_stats.txt`。
- MATLAB 需要写 `%APPDATA%\MathWorks` 与临时目录，因此 DSH 沙箱必须是 `danger-full-access`，
  否则 MATLAB 直接启动失败（退出码 `-1073741824` 家族，如 `-1073741502` = `0xC0000142`）。

---

## 5. 产物图命名规则（所有脚本统一）

```
<前缀>_<物理量>.png
```

- `<前缀>` = 输入 CSV 文件名去扩展名：`log4_clean.csv` → `log4_clean`，`step01_1700.csv` → `step01_1700`；
  台架图前缀固定为 `flywheel`。
- `<物理量>` 自解释：`velocity` / `error` / `duty` / `current` / `voltage` / `zoom_*` /
  `scatter_*` / `step_*` / `launch_*` / `ff_regression` / `bench_*`。
- **一张图只画一件事**（不再有多 subplot 拼图）；尺寸 `figsize=(9, 4)`，
  保存 `dpi=150, bbox_inches='tight'`。
- 每张图自带标题（含目标转速/时间窗）、带单位的 x/y 轴标签、grid，
  关键数字用 `annotate` / 图例画在图上。
- 完整清单与判读方法见 **[PLOT_GUIDE.md](PLOT_GUIDE.md)**。

---

## 6. CSV 格式

### 6.1 标定 CSV（Tune6.x）

```
# flywheel tuning data (step01)
# step response, d0=0.15, dStep=0.25, settleSec=2.0, ..., tRecover=0.5, ratio=4.0
t,duty,velocity
0.0000,0.1500,1449.00
...
# --- step applied here (t=0 of this segment) ---
0.0000,0.2500,1466.00
...
```

- 以 `#` 开头的是注释行，记录本次实验的参数与结论（形如 `key=value`），脚本会解析它；
- 表头固定 `t,duty,velocity`：时间 (s)、占空比、转速 (tick/s，即 `DcMotorEx#getVelocity()`)；
- 解析方式：`numpy.loadtxt(f, comments='#', skiprows=1)` 或 `pandas.read_csv(f, comment='#')`；
- **step01 的 `t` 在阶跃处重新从 0 开始**（注释行 `# --- step applied here ---` 标出该位置），
  脚本按该标记分段处理；
- 判断一个 CSV 是不是标定表：第一个非注释行的逗号列数必须是 3（`analyze_tuning.py`
  就是这么筛的，自动跳过 Dashboard 日志）。

### 6.2 clean 表（`export_clean_table.py` 产出）

`#` 元数据行 + 表头 + 数据行，列：
`t,target,velocity,error,duty,ff,i,p,boost,boost_term,current`（原始日志有电压时再加 `voltage`）。

- `t` 按**行序**重建的均匀时间轴（原始 `time` 列毫秒字段是坏的，不能直接用）；
- `error` 统一重算为 `target − velocity`（不用日志里那一列，避免与速度不同帧）；
- 两种读法都对：`pandas.read_csv(f, comment='#')`；
  MATLAB `readtable(f,'CommentStyle','#')`。

---

## 7. 判读要点（踩过的坑）

1. **先看量化台阶**：`drop04` 会报出速度反馈的量化台阶（实测速度的最大公约数）与 σ 相当于几个台阶。
   若 `Δω_th` 只相当于 1~2 个台阶，阈值就被量化噪声主导——报告里会给出 `⚠` 警告。
   本机 log2/log3/log4 的台阶都是 **20 tick/s**，但**不要跨日志套用**。
2. **τ 取哪个**：端上用 t63.2% 判据，离散采样下偏大；离线最小二乘更准，两者差距大时以最小二乘为准。
3. **`ff05` / 拟合截距 = `kS`**：现行控制律**已经包含 `kS`（静摩擦/常数偏置）项**，
   线性拟合截距就是它的实测值，直接填进 `kS` 即可；过原点斜率与拟合斜率会略有差异，
   两者都打印出来便于对照。截距 ≤ 0.005 时可不加。
4. **`zn02` 必须近似等幅**：只有等幅振荡时当前 Kp 才是 Ku；幅值还在增长说明还没到临界点，需继续加 Kp。
5. **一致性核对**：`drop04` 报告里会列端上给出的 `sigma/maxDecel/dropThreshold/...`，
   与离线复算值应基本一致；不一致说明端上那段代码或采样口径变了。
6. **单位**：增益单位统一为「占空比」，即
   `kS` = 占空比，`kff` = 占空比/(tick/s)，`kp` = 占空比/(tick/s)，`ki` = 占空比/(tick/s·s)；
   控制器不读电压传感器，若后续改成 `d=V/Vdc`，这些增益需要乘 `Vdc` 换成电压量纲。
7. **稳态段要停够 2 s**、**时间轴要重建**、**5 点滑动中位数别用在阶跃处** —— 详见 PLOT_GUIDE §9。

---

## 8. 合成样例自检

没有机器人时也可以验证脚本本身。`make_samples.py` 会生成 5 组格式完全一致的合成 CSV，真值已知：

```bash
python tools/tuning/make_samples.py ./samples
python tools/tuning/analyze_tuning.py ./samples --out ./samples
```

复算值应当还原真值（实测结果）：

| 方法 | 真值 | 复算值 |
|---|---|---|
| step01 | Kd=8000、τ=0.30 s | Kd=7984.2、τ(最小二乘)=0.2999 s、τ(t63.2%)=0.32 s |
| zn02 | Tu=0.42 s、Ku=9e-5（锁定值） | Tu=0.4382 s、Ku 取自锁定行 → kp=4.05e-5、ki=1.11e-4 |
| relay03 | Δd=0.2、a≈180、Tu=0.55 s | Δd=0.2000、a=165.0、Tu=0.5967 s → Ku=1.54e-3 |
| drop04 | σ≈15、台阶=1 | σ=15.51、台阶=1.0、maxDecel=600.6、Δω_th=77.57 |
| ff05 | kff=1.2e-4、截距 0.012 | 过原点 1.2533e-4；线性拟合斜率 1.2000e-4、截距 0.0120 |

（MATLAB 版对同一批样例给出相同口径的结果，见 §4。）

## 9. 数据自检

```bash
python tools/tuning/check_plot_data.py
```

检查 `out/clean/log4_clean.csv` 的列名与口径（`t` 严格单调、`error = target − velocity`、
量化台阶 = 20、最高转速 ≈ 2400、`kp` 量级），并把 MATLAB 版 `plot_speed_stats.txt` 里的
统计值与 Python 复算交叉核对。图还没生成时只打印 `[跳过]`，不算失败。
