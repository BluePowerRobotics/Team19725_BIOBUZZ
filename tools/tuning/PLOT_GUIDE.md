# 飞轮图表判读指南（PLOT_GUIDE）

本文件是 `tools/tuning/` 全部**图片产物**的清单与判读手册。
脚本总览、取数流程、CSV 格式见 [README.md](README.md)；端上标定操作见
[`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Tuning/Guide.md`](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Tuning/Guide.md)。

> **本版变化**：所有脚本改成 **一图一文件** —— 每个物理量 / 每个判据单独一张 PNG，
> 不再把 2×2、1×3、4×1 subplot 拼成一张大图。文件名后缀就是这张图回答的问题。
> 判据阈值**逐条对着当前源码核对过**（§7.1 标了文件名与行号），与旧版说明不同处以源码为准。

---

## 0. 总则（所有图共用）

1. **一图一文件**：想看多个量就把几张 PNG 并排贴在报告里。
2. **命名规则**：`<前缀>_<物理量>.png`。前缀 = 输入 CSV 文件名去扩展名
   （`log4_clean.csv` → `log4_clean`，`step01_1700.csv` → `step01_1700`）；
   台架图前缀固定为 `flywheel`；MATLAB 的 `plot_speed.m` 用 `_speed_` / `_diag_` 区分图组。
3. **每张图自带**：标题（含目标转速 / 时间窗）、x 轴标签与单位、y 轴标签与单位、grid，
   以及**画在图上**的关键数字（`ax.annotate` / 图例），如稳态 sd、极限环周期与幅值、
   跌幅、恢复时间。判读时不必回头看终端输出。
4. 保存参数统一 `dpi=150, bbox_inches='tight'`；单图 `figsize=(9, 4)`。
   MATLAB 用 `exportgraphics(fig, png, 'Resolution', 150)`。
5. 终端统计与 `*_report.txt` **保持原样**，图只是把同一批数字可视化。
   **本次拆图没有改动任何解析口径、判据阈值与单位。**

数据来源与列名：

| 来源 | 文件 | 谁产出 | 有电压列吗 |
|---|---|---|---|
| FTC Dashboard 原始导出 | 手动导出 `*.csv` | Dashboard 遥测记录 | 有 |
| clean 表 | `*_clean.csv`（11 列 + 可选 `voltage`） | `export_clean_table.py` | 视原始日志 |
| 整定 OpMode CSV | `tuning/<前缀>_<时间戳>.csv`（`t,duty,velocity` 三列） | `TuningLogger`（RC 外部存储 `/tuning/`） | 无 |
| 台架 CSV | `flywheel/*.csv`（`t,power,velocity,busV,motorV[,mode]`） | 台架测试程序 | 有（busV） |

统一约定：时间列单位秒、严格单调；速度 tick/s；占空比 0~1。
**判读前先确认量化台阶**（本机 log4/log3 为 **20 tick/s**）：所有「稳态误差、极限环幅值、
过冲、阈值」都要换算成台阶数，否则会把 1 个台阶的正常抖动当成故障。

---

## 1. `plot_flywheel.py` —— 台架三段式开环/闭环对比（10 张）

输入：`flywheel/*.csv`。`fly1_*.csv`（无 mode 列）= 阶段①无控制器扫描；
带 mode 列的 `fly*.csv` = 阶段②/③对照（mode 0 = 主控制、mode 1 = 对照组）。
稳态统计窗口 `--window` 默认 **2.0 s**。

| 文件名 | 横轴 | 纵轴 | 画的是什么 | 看什么 | 判据 |
|---|---|---|---|---|---|
| `flywheel_compare_velocity.png` | 时间 (s) | 转速 (tick/s) | 多个 mode / 文件在同一 setpoint 下的转速曲线 | 曲线重合度、稳态带宽度、启动快慢 | 稳态 sd < 1 台阶（20 tick/s）为好；两模式差 > 2% 才有意义 |
| `flywheel_compare_duty.png` | 时间 (s) | 占空比 (0~1) | 同一批 run 的占空比（图上画 0 / 1 红线） | 抖动幅度、是否贴顶 | 稳态 duty sd ≤ 0.01 好 / ≤ 0.03 合格 / > 0.05 差（量化主导） |
| `flywheel_compare_voltage.png` | 时间 (s) | 母线电压 (V) | 母线电压 + 11.5~13.0 V 绿带 | 跌落深度与恢复快慢 | 落在绿带内为好；长期 < 11.5 V 说明电池弱/线阻大，此时比较增益不公平 |
| `flywheel_sweep_velocity.png` | 占空比 power (0~1) | 转速 (tick/s) | 阶段①每档的稳态转速均值，误差棒 = 该档 sd | 是否单调、有无平台/掉落（限流）、极限点 | 单调上升为好；末端变平 = 输出到顶，别再往上加 power |
| `flywheel_sweep_voltage.png` | 占空比 power (0~1) | 电压 (V) | 母线电压与「母线电压×占空比」（端电压） | 大功率段跌多少、电池内阻水平 | busV ≥ 11.5 V 为好；< 11.0 V 时 kff 会系统性偏小 |
| `flywheel_sweep_kd.png` | 占空比 power (0~1) | 转速 (tick/s) | 对 (power, velocity) 的线性拟合，标出 Kd 与截距 | 斜率 = Kd（`kff = 1/Kd`）、截距 = 静摩擦 | 明显弯曲的功率段 = 已到电机能力上限，不要在**那段**取 Kd |
| `flywheel_bench_velocity.png` | 基准表 tag（阶段①极限 / ②PID vs 开环 / ③控制律 vs PID） | 稳态转速 v (tick/s) | `# bench` 行记录的 v 柱状对比 | 后一阶段是否比前一阶段更快/更稳 | 差值 > 50 tick/s 才算真实差异（以下属量化噪声） |
| `flywheel_bench_sd.png` | 同上 | 转速 sd_v (tick/s) | 各阶段稳态转速标准差 | 控制质量趋势，越小越稳 | ≤ 20 tick/s（1 台阶）优秀 / ≤ 40（2 台阶）合格 / > 40 差 |
| `flywheel_bench_duty.png` | 同上 | 稳态占空比 p (0~1) | 各阶段的控制代价 | 更稳是否付出了更大占空比 | 同转速下 duty 更低者优；接近 1.0 说明没余量 |
| `flywheel_bench_voltage.png` | 同上 | 稳态母线电压 busV (V) | 各阶段的供电水平 | 阶段间电池状态是否可比 | 差异 > 0.3 V 时两阶段数据不可直接比较 |

---

## 2. `analyze_dashboard_log.py` —— 整段 Dashboard 遥测（9 张，最常用）

命令行：`python analyze_dashboard_log.py <原始日志.csv> --out ./out [--tail 1.0] [--no-plot]`
（**不吃 clean 表**，clean 表交给 `analyze_step_response.py` / `analyze_launch_events.py` / `plot_speed.m`）。
时间轴按**行序重建**，台阶由速度 gcd 求出，末尾「目标未变但速度归零」的收尾帧会被剔除。

| 文件名 | 横轴 | 纵轴 | 画的是什么 | 看什么 | 判据 |
|---|---|---|---|---|---|
| `<stem>_velocity.png` | 时间 (s) | 转速 (tick/s) | 目标（虚线）vs 实测（实线），目标变化处画灰虚线 | 跟踪形状、延迟、掉速坑 | 稳态段应贴在目标线上（≤ 2 台阶） |
| `<stem>_error.png` | 时间 (s) | 误差 e = ω* − ω (tick/s) | 误差全程 + ±1 台阶（红点线）+ ±2% 最大目标（绿虚线） | 稳态误差是否在带内、换速后的波动 | 稳态 \|e\| ≤ 2% 目标为合格，> 2% 报 `[警告]` |
| `<stem>_duty.png` | 时间 (s) | 占空比 (0~1) | duty 与 FF / P / I 分量、FF+P+I 叠加（黑点线） | 黑点线是否越过 1.0（被限幅截掉）、谁在主导 | duty ≥ 0.999 的帧占比 > 5% → `[警告]`（饱和，加大 kp 没用） |
| `<stem>_current.png` | 时间 (s) | 电流 (A) | 电机电流，绿色阴影 = 掉速补偿 boost 生效段 | boost 是否在该出现时出现 | 全程 boost 触发 < 0.1% → `[提示]`（补偿从未被验证） |
| `<stem>_voltage.png` | 时间 (s) | 母线电压 (V) | 母线电压 + 11.5~13.0 V 绿带（**仅当日志带 voltage 列才生成**） | 跌落深度、恢复形状 | 绿带内为好；跌落 > 0.5 V 且恢复 > 0.3 s → 调 τ_bus（§10） |
| `<stem>_zoom_velocity.png` | 时间 (s，段内相对) | 转速 (tick/s) | 最后一个稳态段末尾 2 s 放大，画 ±1/2 台阶网格 | 转速是否只在相邻台阶间跳（量化极限环） | 只跳 1 格 = 已到量化极限；跳 2 格以上说明仍有真实振荡 |
| `<stem>_zoom_duty.png` | 时间 (s，段内相对) | 占空比 (0~1) | 同一窗口的占空比 | 转速只跳一格、duty 却大范围跳 → kp 相对台阶过大 | `kp×台阶 ≥ 0.05` → `[警告]`；`≥ 0.20` → `[严重]` |
| `<stem>_scatter_p_vs_error.png` | 误差 e (tick/s) | P term (占空比) | P 项对误差的散点 | 斜率 = kp；点呈台阶状 = 量化主导；是否贴到限幅 | 应落在一条过原点直线上；出现大跨度水平段 = 误差被量化吃掉 |
| `<stem>_scatter_duty_vs_velocity.png` | 占空比 (0~1) | 转速 (tick/s) | 转速对占空比的散点 | 斜率 ≈ Kd；右端下垂 = 饱和；左侧是否偏离直线 | 线性段斜率稳定；下垂段不可用于估 Kd |

> 终端统计里给出逐段 `v_sd` / 误差% / duty sd / 饱和% / P 项幅度，以及极限环的
> 过零周期（`zc_period`，需 ≥ 5 次过零，否则 `nan`）与幅值（换算成台阶数）。

---

## 3. `analyze_step_response.py` —— 多次阶跃的汇总与对齐（4 张）

输入：clean 表。参数：`--band-frac` 默认 **0.02**、`--band-steps` 默认 **2.0**、
`--tail` 默认 **1.0 s**；恢复带 `band = max(0.02·目标, 2·台阶)`，需**连续 10 帧**在带内
才算进入稳态（`settle_index(hold=10, limit_sec=20)`）；分段间隔 `gap=0.05 s`。

| 文件名 | 横轴 | 纵轴 | 画的是什么 | 看什么 | 判据 |
|---|---|---|---|---|---|
| `<stem>_step_velocity.png` | 相对阶跃时刻的时间 (s) | 转速 (tick/s) | 最大一次向上阶跃窗口：实测、目标、±band、t90 与进带时刻竖线、占空比/电压注记 | 起步斜率、到速时间、是否超调 | 超调中位 > +5% 说明 kp 偏大 |
| `<stem>_step_duty.png` | 相对阶跃时刻的时间 (s) | 占空比 (0~1) | 同一窗口的占空比 | 是否顶到 1.0 并长期保持 | 饱和占比中位 > 50% = 加速段全油门，属正常；**恢复期还饱和**则加大增益无用 |
| `<stem>_step_current.png` | 相对阶跃时刻的时间 (s) | 电流 (A) | 同一窗口的电机电流 | 加速电流峰值、饱和时是否还有余量 | 长期贴限流值 → 受电源/电机限制，不是控制参数问题 |
| `<stem>_step_aligned.png` | 相对阶跃时刻的时间 (s) | 转速 (tick/s) | 全部向上阶跃对齐叠加 | 形状一致性、迟滞、个别离群 | 曲线束越窄越好；明显分叉 = 不同工作点的 Kd/静摩擦差别大（需要 kS + 分段 kff） |

报告结论（终端 + `*_step_report.txt`）：转速 sd **< 1 台阶为优秀**；占空比 sd **> 0.05**
说明增益相对台阶过大；kS **> 0.005** 建议拆分 kS；工作点跨度 **< 300 tick/s** 时
kS/kff 无法分离（只报过原点 kff）。

---

## 4. `analyze_launch_events.py` —— 发射掉速逐发分析（每发 1 张 + 3 张统计 + 1 份 CSV）

输入：**原始 Dashboard CSV**（与 `analyze_dashboard_log.py` 同一读法，直接复用其 `load` /
`rebuild_time`，所以**不能直接喂 clean 表**）。参数：`--target-band` 默认 **0.02**（比例，不是 tick 数）、`--min-target` 默认 **500**。
事件判据：跌幅门限 `delta = max(100, 6%·ω*)`（源码 `max(100.0, 3.0 * target_band * tv)`），
恢复带 `band = max(2%·目标, 1·台阶)`，需连续 10 帧；`|ω*| < 500` 或事件 < 2 s 的段跳过。

| 文件名 | 横轴 | 纵轴 | 画的是什么 | 看什么 | 判据 |
|---|---|---|---|---|---|
| `<stem>_launch_velocity.png` | 时间 (s) | 转速 (tick/s) | 全程转速 + 目标，每个掉速事件区间着色并编号 | 一共几发、每发跌多深、恢复多久 | 跌幅中位 < 10% ω* 好 / 10~20% 合格 / > 20% 差 |
| `<stem>_launch_duty.png` | 时间 (s) | 占空比 (0~1) | 全程 duty 与 I / P 分量 | 恢复期是否顶到 1.0（无余量） | 过半事件恢复期 duty ≥ 0.999 → `[警告]` |
| `<stem>_launch_boost.png` | 时间 (s) | 补偿（×100 显示） | boost 标志与 boost term | boost 有没有在该出现时出现、峰值是否贴限幅 | 参与率 < 60% → `[警告]`；峰值 ≥ 0.399（boostMax = 0.400）→ `[警告]` |
| `<stem>_launch_001.png` … `<stem>_launch_NNN.png` | 相对事件起点的时间 (s) | 转速 (tick/s) | **每一次事件单独一张**：转速 + 目标 + 占空比/I 注记 | 单发细节：跌落速度、反弹形状、恢复后是否欠冲 | 恢复 < 0.3 s 好 / 0.3~0.5 s 合格 / > 0.5 s 差（中位 > 0.4 s 报警告） |
| `<stem>_launch_stats_depth_vs_recover.png` | 跌幅 (tick/s) | 恢复时间 (s) | 每发一个点，颜色 = boost 占比% | 跌幅与恢复时间是否正相关；boost 高的点是否恢复更快 | 若 boost 占比高却没改善 → 补偿方向/强度有问题 |
| `<stem>_launch_stats_depth_hist.png` | 跌幅 (tick/s) | 事件数 | 跌幅分布直方图 | 是否集中、有无双峰 | 双峰 = 两种工况（单发/连发），必须分开标定 |
| `<stem>_launch_stats_recover_hist.png` | 恢复时间 (s) | 事件数 | 恢复时间分布直方图 | 中位与长尾 | 中位 > 0.4 s → `[警告]`；右拖长尾 = 偶发严重掉速 |
| `<stem>_launch_events.csv` | — | — | **事件总表**（不是图）：`idx,target,t0_s,recover_s,depth_tick_s,depth_pct,vmin,duty_max,is_saturated,i_min,p_max,current_max,boost_frac,boost_max,overshoot` | 可直接贴进报告做逐发对照 | — |

> 恢复后过冲中位 < −50 tick/s → `[提示]` 欠冲（恢复过慢）。负值 = 没冲过头。

---

## 5. `analyze_tuning.py` / `analyze_tuning.m` —— Tune6.x 标定 CSV（每个输入 2 张图 + 报告）

```bash
python tools/tuning/analyze_tuning.py ./tuning_data --out ./out [--no-plot]
```
MATLAB 同口径、**图名与 Python 完全一致**。

| 方法 | 文件名 | 横轴 | 纵轴 | 画的是什么 | 看什么 |
|---|---|---|---|---|---|
| 6.1 step01 | `<stem>_step_velocity.png` | 时间 (s)，0 = 阶跃时刻 | 转速 (tick/s) | 阶跃前后转速 + 最小二乘一阶拟合曲线 + t63.2% 竖线 + 图内注记（Δd、Δω、Kd、kff、两个 τ、τc） | 拟合是否贴合实测；t63.2% 与最小二乘 τ 是否接近 |
| 6.1 step01 | `<stem>_step_duty.png` | 时间 (s)，0 = 阶跃时刻 | 占空比 (0~1) | 开环阶跃的 d0 / dStep | 阶跃是否干净（瞬时跳变、无斜坡） |
| 6.2 zn02 | `<stem>_velocity.png` | 时间 (s) | 转速 (tick/s) | 尾部窗内转速 + 中心线 + ±a 线 + 注记（a、Tu、Ku） | **必须近似等幅**；幅值还在长 = 没到临界点，继续加 kp |
| 6.2 zn02 | `<stem>_duty.png` | 时间 (s) | 占空比 (0~1) | 输出波形 | 输出是否也在等幅振荡 |
| 6.3 relay03 | `<stem>_velocity.png` | 时间 (s) | 转速 (tick/s) | 极限环转速 + 中心/幅值线 | 是否稳定极限环、周期是否稳定 |
| 6.3 relay03 | `<stem>_duty.png` | 时间 (s) | 占空比 (0~1) | dHigh / dLow 参考线 + Δd 注记 | 继电器是否真的只有两档 |
| 6.4 drop04 | `<stem>_velocity.png` | 时间 (s) | 转速 (tick/s) | 测量段转速 | 稳态带宽度 → σ；台阶数 = σ/step |
| 6.4 drop04 | `<stem>_duty.png` | 时间 (s) | 占空比 (0~1) | 测量段占空比 | 是否平稳无大幅波动 |
| 6.5 ff05 | `<stem>_velocity.png` | 时间 (s) | 转速 (tick/s) | 全程转速 | 各稳态点是否真的稳住了 |
| 6.5 ff05 | `<stem>_ff_regression.png` | 转速 ω (tick/s) | 稳态占空比 d_ss (0~1) | 测量点 + 过原点 kff 线 + 含截距线性拟合线 + 注记（斜率 = kff、截距 = kS） | 点是否成直线；截距明显非 0 = 静摩擦项 kS |
| 未识别 | `<stem>_velocity.png` / `<stem>_duty.png` | 时间 (s) | 转速 / 占空比 | 只出基本曲线 | 确认 CSV 注释行里有没有 `method=` 关键字 |

判据（源码常量）：
- **6.1**：`kff = 1/Kd`、`kp = τ/(Kd·τc)`、`ki = 1/(Kd·τc)`，`τc = tRecover/ratio = 0.5/4.0 = 0.125 s`；
  稳态窗 `steadyWindowSec` 默认 0.5 s。Kd 明显偏小 → 加大 Δd 重跑。
- **6.2/6.3**：`kp = 0.45·Ku`、`ki = 0.54·Ku/Tu`；继电器 `Ku = 4Δd/(πa)`。
- **6.4**：`Δω_th = k_σ·σ`（k_σ 默认 5.0）、`k_th = k_rate·maxDecel`（k_rate 默认 2.5）、
  `kBoost = 2·kp`；**Δω_th 折算不足 3 个量化台阶 → `⚠` 警告**（阈值被量化噪声主导）。
- **6.5**：`kff_origin = Σ(ω·d_ss)/Σ(ω²)` 与 `np.polyfit` 斜率并列给出，**截距即 kS**。

> 五种方法（step01 / zn02 / relay03 / drop04 / ff05）**当前都还在脚本里**，靠文件名前缀或
> 注释行关键字识别；不是 t,duty,velocity 三列的 CSV 会被打印 `[跳过]` 后忽略。

---

## 6. `plot_speed.m` —— MATLAB 转速专图与诊断（6 张）

输入：clean 表。**图名与 Python 版一致，另用 `_speed_` / `_diag_` 区分图组**：

| 文件名 | 横轴 | 纵轴 | 画的是什么 | 看什么 |
|---|---|---|---|---|
| `<stem>_speed_velocity.png` | 时间 (s) | 转速 (tick/s) | 目标（红虚线）vs 实测（实线）+ ±1/2/3 台阶灰网格 + 目标变化竖线 + 掉速/发射区间橙色阴影 | 稳态贴线程度、掉速事件的次数与位置 |
| `<stem>_speed_error.png` | 时间 (s) | 误差 = 目标 − 实测 (tick/s) | 误差 + ±2% 最大目标（红点线）+ ±1 台阶（绿点线） | \|e\| 是否长期在带内 |
| `<stem>_diag_velocity.png` | 时间 (s) | 转速 (tick/s) | 目标 vs 实测（诊断副本，一张图只讲一件事） | 快速定位异常时段 |
| `<stem>_diag_error.png` | 时间 (s) | 误差 (tick/s) | 误差 + ±1 台阶线 | 台阶状 = 量化；锯齿状 = 增益过大 |
| `<stem>_diag_duty.png` | 时间 (s) | 占空比 (0~1) | duty 与 FF / P / I 分量、FF+P+I 叠加 | 谁在主导输出、是否被 1.0 截掉 |
| `<stem>_diag_current.png` | 时间 (s) | 电流 (A) | 电流 + boost 生效阴影 | 电流余量、boost 时序 |

`plot_speed.m` 的判据：`band = max(2%·目标, 1·台阶)`、`delta = max(100, 3%·目标)`、
事件段 `|ω*| < 500` 或时长 < 2 s 跳过、恢复需连续 10 帧在带内、稳态窗 `-tail` 默认 **2.0 s**
（且不超过该段时长一半）。统计文本写到固定文件 **`plot_speed_stats.txt`**。

> 注：`plot_speed.m` 的事件跌幅门限是 **3%·ω\***，而 Python 的 `analyze_launch_events.py`
> 是 **6%·ω\***（`3.0 * target_band`，target_band 默认 0.02）。两个脚本事件集合因此**可能不同**，
> 这是既有差异，本次拆图未改；跨脚本对数时以上面的公式各自核对。

---

## 7. 判据阈值总表

### 7.1 代码里真实存在的判据（改脚本就会改行为，改动务必同步本文件）

| 量 | 好 | 合格 | 差 | 出处 |
|---|---|---|---|---|
| 稳态误差 \|e\| | ≤ 2% 目标 | — | > 2% 目标 → `[警告]` | `analyze_dashboard_log.py:403` |
| `kp×台阶`（占空比摆动） | < 0.05 | 0.05 ~ 0.20 | ≥ 0.20 → `[严重]`（0.05~0.20 为 `[警告]`） | `analyze_dashboard_log.py:390-399` |
| 占空比饱和占比（duty ≥ 0.999） | ≤ 5% | — | > 5% → `[警告]` | `analyze_dashboard_log.py:410` |
| boost 触发占比 | ≥ 0.1% | — | < 0.1% → `[提示]`（补偿未验证） | `analyze_dashboard_log.py:407` |
| 发射事件 boost 参与率 | ≥ 60% | — | < 60% → `[警告]` | `analyze_launch_events.py:171` |
| boost term 峰值 | < 0.399 | — | ≥ 0.399 → `[警告]`（boostMax = 0.400） | `analyze_launch_events.py:173` |
| 恢复期顶到 100% 的事件占比 | ≤ 50% | — | > 50% → `[警告]` | `analyze_launch_events.py:175` |
| 发射恢复时间中位 | < 0.3 s | 0.3 ~ 0.4 s | > 0.4 s → `[警告]` | `analyze_launch_events.py:177` |
| 恢复后过冲中位 | ≥ −50 tick/s | — | < −50 → `[提示]`（欠冲） | `analyze_launch_events.py:179` |
| 稳态转速 sd | < 1 个台阶（log4 为 20 tick/s） | 1 ~ 2 台阶 | > 2 台阶 | `analyze_step_response.py:277`（只打印，不报警） |
| 稳态占空比 sd | ≤ 0.05 | — | > 0.05（打印「增益相对台阶过大」） | `analyze_step_response.py:281` |
| 拟合截距 kS | ≤ 0.005 | — | > 0.005 → 建议拆分 kS | `analyze_step_response.py:302` |
| 工作点转速跨度 | ≥ 300 tick/s 才能分离 kS/kff | — | < 300 → 只报过原点 kff | `analyze_step_response.py:299` |
| drop04 阈值台阶数 | ≥ 3 个台阶 | — | < 3 → `⚠` 警告 | `analyze_tuning.py:663` |
| 阶跃恢复带 | `band = max(2%·目标, 2·台阶)`，连续 10 帧 | — | 带外即未恢复 | `analyze_step_response.py` / `plot_speed.m` |
| 发射事件跌幅门限 | `delta = max(100, 6%·ω*)` | — | 低于门限不算事件 | `analyze_launch_events.py` |

### 7.2 仅供目视参考的经验带（**代码里没有数值判据**，不要当成通过/不通过的红线）

| 量 | 参考带 | 说明 |
|---|---|---|
| 母线电压 | **11.5 ~ 13.0 V** | 图上画的绿带就是这个区间（标称 12.8 V）。它是**画图参考带**，不是任何脚本的报警阈值；长期 < 11.5 V 先换电池/查线阻，再谈调参 |
| 电流峰 | 本机实测约 9.0 A（加速段） | 用于判断电源/电机是否有余量，非阈值 |
| 量化台阶 | 由速度 gcd 实测（log4/log3 为 **20 tick/s**，合成样例为 1 tick/s） | 不同日志不同，**不要跨日志套用数字** |
| 增益量纲 | `kS` = 占空比；`kff` = 占空比/(tick/s)；`kp` = 占空比/(tick/s)；`ki` = 占空比/(tick/s·s) | 控制器不读电压传感器；若以后改成 `d = V/Vdc`，全部增益要乘 Vdc 换成电压量纲 |

---

## 8. 出图流程（一条命令到全部图）

```bash
# 0) 取数（一次性）
adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/tuning ./tuning_data

# 1) Tune6.x 标定 CSV → 每个文件 2 张图 + 报告（Python 主用）
python tools/tuning/analyze_tuning.py ./tuning_data --out ./tools/tuning/out

# 2) 同一批 CSV 用 MATLAB 复核（图名一致，便于对照）
#    analyze_tuning('C:\...\tuning_data', '-out', 'tools/tuning/out')

# 3) Dashboard 原始日志 → 9 张图 + 报告
python tools/tuning/analyze_dashboard_log.py ./raw/log4.csv --out ./tools/tuning/out

# 4) 原始日志 → clean 表（后续三个脚本的输入）
python tools/tuning/export_clean_table.py ./raw/log4.csv --out ./tools/tuning/out/clean

# 5) 多次阶跃统计 → 4 张图 + 报告
python tools/tuning/analyze_step_response.py tools/tuning/out/clean/log4_clean.csv --out tools/tuning/out

# 6) 发射掉速逐发分析 → 每发 1 张 + 3 张统计 + 事件总表 CSV（吃**原始日志**）
python tools/tuning/analyze_launch_events.py ./raw/log4.csv --out ./tools/tuning/out

# 7) 台架三段式对比 → 10 张图
python tools/tuning/plot_flywheel.py ./flywheel_data --out ./tools/tuning/out

# 8) 没有机器人时先跑合成样例自检（5 组真值已知的 CSV）
python tools/tuning/make_samples.py ./samples
python tools/tuning/analyze_tuning.py ./samples --out ./samples

# 9) 数据一致性自检（clean 表口径 + 与 MATLAB 统计交叉核对）
python tools/tuning/check_plot_data.py
```

产物全部落在 `--out` 指定目录（默认与输入同级）。`--no-plot` 只出报告不出图（无图形环境时用）。

---

## 9. 常见误判（踩过的坑）

1. **时间列是坏的，必须按行序重建均匀时间轴。**
   Dashboard 导出的 `time` 毫秒字段会倒退/丢帧。脚本都用 `rebuild_time` / `export_clean_table.py`
   按**行序**重建均匀 `dt`，并打印倒退处数。拿原始时间列直接画图会看到锯齿暴跌，
   算出来的 sd 与周期全错。
2. **稳态段必须停 ≥ 2 s，否则是暂态污染。**
   稳态窗口取「每段末尾 1~2 s」。某档只停 0.5 s 时，那段「稳态」里全是换速暂态，
   sd / 误差 / 极限环幅值都不可信。`fit_steady_state_model.py --min-dwell` 默认 **2.0 s**
   就是在筛这个；`analyze_dashboard_log.py` 会直接跳过时长 < 0.4 s 或帧数 < 10 的段。
3. **5 点滑动中位数不要用在阶跃处**（`export_clean_table.py` / `fit_steady_state_model.py`
   里的平滑）：中位数滤波会把阶跃抹成斜坡，t63.2%、恢复时间、超调全部失真。
   要平滑就只在**稳态段内部**做，或先切段再平滑。
4. **不要用饱和段估计 Kd**：占空比顶到 1.0 的区段转速不再随 duty 线性增长
   （`_scatter_duty_vs_velocity.png` 右下垂的那段），在那里取斜率会把 Kd 估小。
5. **不要跨日志套用「20 tick/s 台阶」**：台阶是速度反馈分辨率与滤波 τV 的产物，
   换固件/换滤波都可能变；每次都看报告里的 `速度量化台阶（gcd）= …`。
6. **工作点少的 ff05 别信 kS**：只有 2 个工作点时只能报过原点 kff；
   跨度 < 300 tick/s 时 kS 与 kff 强耦合，拟合截距没有物理意义。
7. **`zn02` 幅值还在增长就不是 Ku**：只有等幅振荡时的 kp 才是临界增益。
8. **boost 峰值贴到 0.400 时，加 kBoost 不再有效**：那是限幅在起作用，
   要先提高 `boostMax` 或改检测阈值。
9. **`out/` 里图的覆盖率取决于日志列**：例如 `log4_clean.csv` 只有 11 列（无电压），
   就不会生成 `<stem>_voltage.png`——这是正常行为，不是脚本坏了。
10. **一次实验只改一个参数**：同时改 kp 和 τV 后，duty 抖动和误差形态一起变，
    图上看不出是谁的功劳。
11. **`plot_speed.m` 与 `analyze_launch_events.py` 的事件门限不同**（3% vs 6% ω*），
    事件计数对不上时先查这个，不是数据坏了。

---

## 10. 按参数整定查图表

| 要整定的参数 | 先看哪张图 | 看什么 | 怎么调 |
|---|---|---|---|
| **kS**（静摩擦前馈） | `<stem>_ff_regression.png`、`<stem>_step_aligned.png` | 线性拟合截距是否明显非 0；低转速工作点是否系统性偏低 | kS = 截距（占空比）。截距 ≤ 0.005 可不加 |
| **kff**（速度前馈） | `<stem>_ff_regression.png`、`flywheel_sweep_kd.png` | 过原点线的斜率；Kd 是否线性 | `kff = 过原点斜率 = 1/Kd`；饱和段功率不要参与取 kff |
| **kp**（比例） | `<stem>_error.png`、`<stem>_scatter_p_vs_error.png`、`<stem>_step_duty.png` | kp×台阶是否 ≥ 0.05 / 0.20；误差是否锯齿；阶跃是否超调 | 从 `0.45·Ku` 起；抖动就降到 kp×1 台阶 < 0.05~0.1；超调大也降 kp |
| **ki**（积分） | `<stem>_duty.png` 的 I 分量、`<stem>_step_velocity.png` 的稳态残差 | I 是否长期单调爬升（缺 kS）；稳态误差是否被慢慢吃掉 | 稳态误差 > 2% 且 I 在爬 → 先补 kS 再加 ki；I 来回穿 → ki 偏大 |
| **速度滤波 τV** | `<stem>_duty.png` 抖动、`<stem>_zoom_duty.png` | duty 抖动主频与幅度；转速只跳一格而 duty 大幅横跳 = 滤波不足 | 抖动主频接近量化噪声 → 加大 τV；加到 t63.2% 明显变长就停 |
| **母线电压滤波 τ_bus** | `<stem>_voltage.png`、`flywheel_compare_voltage.png` | 每次发射的电压跌落深度与恢复时间；补偿是否滞后 | 恢复慢且补偿滞后 → 减小 τ_bus；电压毛刺导致增益抖动 → 加大 τ_bus |
| **kBoost / Δω_th**（掉速补偿） | `<stem>_launch_*.png`、`<stem>_launch_stats_*.png` | boost 参与率、峰值是否贴 0.400、恢复中位、跌幅分布 | 参与率低 → 降 Δω_th；峰值贴限幅 → 提高 boostMax 或查 kBoost；恢复仍慢且 duty 已饱和 → 受电源/电机限制，调参无用 |
| **整体验证** | `flywheel_bench_*.png` | 三阶段 v / sd / p / busV 的趋势 | sd 下降而 duty 没涨 = 有效改进；sd 没变说明瓶颈不在这几个增益上 |

---

## 11. 历史基线（第 3 代 13 发实测，作为回归参照）

这些数字来自早期整定批次，**不是当前阈值**，但可作为「改完有没有退化」的锚点：

| 量 | 第 3 代实测 | 判读 |
|---|---|---|
| 发射跌深 | 270 ~ 290 tick/s | 物理决定（压轮/球的摩擦），调参改不了 |
| 恢复时间中位 | 0.428 s（散布 0.25~0.64 s） | 第 4 代目标 ≤ 0.43 s（不劣化 5% 以内） |
| 恢复后过冲 | +50 ~ +90 tick/s | 第 4 代目标 ≤ +20 |
| 恢复期饱和 | 13/13 顶到 1.0 | 跌深大于退饱和门限时必然，属正常 |
| 2250 段 duty 饱和占比 | 74.9% | 该段一直贴顶，此时 kp 再大也没用 |
| 稳态 duty sd | 0.0182 ~ 0.020 | 第 4 代目标 ≤ 0.010（被速度滤波压住） |
| 量化极限环 | 28 ms / 0.50 台阶（好）↔ 51 ms / 1.50 台阶（kp 过大） | 周期 × 幅值一起看 |
