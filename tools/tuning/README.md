# 飞轮标定数据离线分析（tools/tuning）

把 RC 上 `OpModes/Tuning` 五个标定 OpMode 导出的 CSV 变成**图 + 复算过的参数**。
与机器人端使用**同一套公式**，因此离线结果可以和端上遥测互相印证；离线还能多做两件端上做不到的事：
最小二乘拟合 τ（而不是只读 t63.2%）和多点回归 Kff（而不是只取单点）。

对应《飞轮系统控制理论研究报告》第 6 章；端上怎么标定见
[`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Tuning/Guide.md`](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Tuning/Guide.md)。

| 方法 | OpMode | 文件名前缀 | 输出 |
|---|---|---|---|
| 6.1 阶跃响应辨识 | `Tune6.1 StepResponse` | `step01_*.csv` | Kd、τ、kff、kp、ki |
| 6.2 Ziegler-Nichols | `Tune6.2 ZieglerNichols` | `zn02_*.csv` | Ku、Tu、kp、ki |
| 6.3 继电反馈 | `Tune6.3 RelayFeedback` | `relay03_*.csv` | a、Tu、Ku、kp、ki |
| 6.4 掉速检测 | `Tune6.4 DropDetection` | `drop04_*.csv` | σ、量化台阶、maxDecel、Δω_th、k_th、kBoost |
| 6.5 前馈标定 | `Tune6.5 Feedforward` | `ff05_*.csv` | 多点回归 Kff |

## 1. 取出数据

OPmode 结束会把 CSV 写到 RC 外部存储的 `tuning/` 目录：

```
adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/tuning ./tuning_data
```

（或用 RC 上的文件管理器 / USB 拷贝；方法一还会在遥测里显示完整路径。）

## 2. Python 版（主用）

依赖：Python 3 + numpy + matplotlib（本机已具备：3.11.5 / 2.2.6 / 3.10.8；**不需要 scipy**）。

```bash
python tools/tuning/analyze_tuning.py ./tuning_data                 # 分析目录下全部 CSV
python tools/tuning/analyze_tuning.py step01_1700.csv --out ./out   # 单个文件 + 指定输出目录
python tools/tuning/analyze_tuning.py ./tuning_data --no-plot       # 只出报告（无图形环境）
python tools/tuning/analyze_tuning.py zn02_1.csv --ku 9e-5          # 手工指定 Ku
python tools/tuning/analyze_tuning.py drop04_1.csv --kp 5e-5        # 计算 kBoost 需要 kp
```

每个输入产出：

- `<名字>_report.txt`：结论文本（同时打印到终端）
- `<名字>_analysis.png`：曲线图（转速 + 占空比；ff05 第二幅画成 (ω, d_ss) 回归图）

## 3. MATLAB 版（对照）

```matlab
cd tools/tuning
analyze_tuning('C:\path\to\tuning_data')                       % 目录
analyze_tuning('step01_1.csv','zn02_1.csv')                    % 多文件
analyze_tuning(DIR, '-out', OUTDIR, '-noplot')                 % 只出报告
analyze_tuning(F, '-window', 4, '-ku', 9e-5, '-kp', 5e-5)      % 覆盖分析参数
```

选项：`-out DIR`、`-noplot`、`-window S`、`-ku K`、`-kp K`、`-alpha A`。
输出与 Python 版同名（`_report.txt` / `_analysis.png`）。MATLAB 需要写 `%APPDATA%\MathWorks`
与临时目录，因此 DSH 沙箱必须是 `danger-full-access`，否则 MATLAB 直接启动失败
（退出码 `-1073741502` = `0xC0000142`）。

## 4. CSV 格式

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

- 以 `#` 开头的是注释行，记录本次实验的参数与结论（形如 `key=value`），两个脚本都会解析它；
- 表头固定 `t,duty,velocity`：时间 (s)、占空比、转速 (tick/s，即 `DcMotorEx#getVelocity()`)；
- 解析方式：`numpy.loadtxt(f, comments='#', skiprows=1)` 或 `pandas.read_csv(f, comment='#')`；
- **step01 的 `t` 在阶跃处重新从 0 开始**（注释行 `# --- step applied here ---` 标出该位置），
  两个脚本都按该标记分段处理。

## 5. 判读要点（踩过的坑）

1. **先看量化台阶**：`drop04` 会报出速度反馈的量化台阶（实测速度的最大公约数）与 σ 相当于几个台阶。
   若 `Δω_th` 只相当于 1~2 个台阶，说明阈值被量化噪声主导——报告里会给出 `⚠` 警告。
2. **τ 取哪个**：端上用 t63.2% 判据，离散采样下偏大；离线最小二乘更准，两者差距大时以最小二乘为准。
3. **`ff05` 看截距**：线性拟合截距明显非 0 → 存在静摩擦，当前控制律（前馈 + PI + 掉速补偿）没有 `kS` 项，
   可考虑加常数偏置，否则低转速段误差会偏大。
4. **`zn02` 必须近似等幅**：只有等幅振荡时当前 Kp 才是 Ku；幅值还在增长说明还没到临界点，需继续加 Kp。
5. **一致性核对**：`drop04` 报告里会列出端上给出的 `sigma/maxDecel/dropThreshold/...`，
   与离线复算值应基本一致；不一致说明端上那段代码或采样口径变了。
6. **单位**：增益单位统一为「占空比」，即
   `kff` = 占空比/(tick/s)，`kp` = 占空比/(tick/s)，`ki` = 占空比/(tick/s·s)；
   控制器不读电压传感器，若后续改成 `d=V/Vdc`，这些增益需要乘 `Vdc` 换成电压量纲。

## 6. 合成样例自检

没有机器人时也可以验证脚本本身。`make_samples.py` 会生成 5 组格式完全一致的合成 CSV，真值已知：

```bash
python tools/tuning/make_samples.py ./samples
python tools/tuning/analyze_tuning.py ./samples
```

复算值应当还原真值（实测结果）：

| 方法 | 真值 | 复算值 |
|---|---|---|
| step01 | Kd=8000、τ=0.30 s | Kd=7984.2、τ(最小二乘)=0.2999 s、τ(t63.2%)=0.32 s |
| zn02 | Tu=0.42 s、Ku=9e-5（锁定值） | Tu=0.4382 s、Ku 取自锁定行 → kp=4.05e-5、ki=1.11e-4 |
| relay03 | Δd=0.2、a≈180、Tu=0.55 s | Δd=0.2000、a=165.0、Tu=0.5967 s → Ku=1.54e-3 |
| drop04 | σ≈15、台阶=1 | σ=15.51、台阶=1.0、maxDecel=600.6、Δω_th=77.57 |
| ff05 | kff=1.2e-4、截距 0.012 | 过原点 1.2533e-4；线性拟合斜率 1.2000e-4、截距 0.0120 |

（MATLAB 版对同一批样例给出相同口径的结果，见 §3。）
