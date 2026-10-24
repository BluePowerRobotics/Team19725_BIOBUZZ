#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞轮三阶段调试数据出图 / 对比（配合 OpModes/Flywheel 的四个 OpMode）。

读取 `<RC 外部存储>/flywheel/*.csv`（列：t,power,velocity,busV,motorV[,mode]），自动识别：
  fly1_*  = 阶段① 无控制器功率扫描（sweep）      → 画 速度-功率 曲线 + 极限点
  fly2_*  = 阶段② 单 PID ⇄ 开环对照（含 mode 列） → 画 时间序列（按 mode 分色）
  fly3_*  = 阶段③ 现有控制律 ⇄ 纯 PID 对照        → 同上

输出（默认写到 --out 目录；**一图一文件**，命名规则 `flywheel_<子图>_<物理量>.png`）：
  flywheel_compare_velocity.png  所有对照文件的速度时间序列叠加（跨文件比较）
  flywheel_compare_duty.png      所有对照文件的占空比时间序列叠加
  flywheel_compare_voltage.png   所有对照文件的母线电压时间序列叠加（含 11.5~13 V 正常带）
  flywheel_sweep_velocity.png    阶段① 速度-功率 扫描曲线（误差棒=各级 sd，标出极限点）
  flywheel_sweep_voltage.png     阶段① 母线电压-功率 与 端电压≈busV×power
  flywheel_bench_velocity.png    基准表 v 对比柱状图（三阶段横向对比）
  flywheel_bench_sd.png          基准表 sd_v 对比柱状图（越小越稳）
  flywheel_bench_duty.png        基准表 p 对比柱状图（控制代价）
  并在终端打印各文件、各模式的稳态统计表。

用法：
    python plot_flywheel.py ./flywheel_data
    python plot_flywheel.py fly1_1.csv fly2_1.csv fly3_1.csv --out ./out
    python plot_flywheel.py ./flywheel_data --window 2.0 --no-plot
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import sys
from dataclasses import dataclass, field

try:
    import numpy as np
except ImportError:
    print("需要 numpy：pip install numpy", file=sys.stderr)
    raise

BENCH_PREFIX = "# bench,"


@dataclass
class Series:
    path: str
    name: str
    kind: str                      # sweep / compare / generic
    header: list = field(default_factory=list)
    t: "np.ndarray" = None
    power: "np.ndarray" = None
    vel: "np.ndarray" = None
    bus: "np.ndarray" = None
    motor: "np.ndarray" = None
    mode: "np.ndarray" = None
    notes: list = field(default_factory=list)
    bench: list = field(default_factory=list)    # [(tag, v, p, sdV, sdP, busV)]


def read_csv(path: str) -> Series:
    s = Series(path=path, name=os.path.splitext(os.path.basename(path))[0], kind="generic")
    cols = {"t": [], "power": [], "velocity": [], "busV": [], "motorV": [], "mode": []}
    ncol = 5
    with open(path, "r", encoding="utf-8-sig", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                if line.startswith(BENCH_PREFIX):
                    p = [x.strip() for x in line.split(",")]
                    if len(p) >= 7:
                        try:
                            s.bench.append((p[1], float(p[2]), float(p[3]),
                                            float(p[4]), float(p[5]), float(p[6])))
                        except ValueError:
                            pass
                else:
                    s.notes.append(line.lstrip("#").strip())
                continue
            parts = [x.strip() for x in line.split(",")]
            if len(parts) < 5:
                continue
            try:
                vals = [float(x) for x in parts]
            except ValueError:
                s.header = parts
                ncol = len(parts)
                continue
            cols["t"].append(vals[0])
            cols["power"].append(vals[1])
            cols["velocity"].append(vals[2])
            cols["busV"].append(vals[3])
            cols["motorV"].append(vals[4])
            cols["mode"].append(vals[5] if len(vals) > 5 else 0.0)

    s.t = np.asarray(cols["t"], float)
    s.power = np.asarray(cols["power"], float)
    s.vel = np.asarray(cols["velocity"], float)
    s.bus = np.asarray(cols["busV"], float)
    s.motor = np.asarray(cols["motorV"], float)
    s.mode = np.asarray(cols["mode"], float) if ncol > 5 else None

    low = s.name.lower()
    if low.startswith("fly1"):
        s.kind = "sweep"
    elif s.mode is not None and len(np.unique(s.mode)) >= 1 and low.startswith("fly"):
        s.kind = "compare"
    elif s.mode is not None:
        s.kind = "compare"
    return s


def segments(s: Series):
    """把对照文件按 mode 切成若干连续段，返回 [(mode, i0, i1), ...]。"""
    if s.mode is None or len(s.mode) == 0:
        return [(0.0, 0, len(s.t))]
    out = []
    start = 0
    for i in range(1, len(s.mode)):
        if s.mode[i] != s.mode[i - 1]:
            out.append((float(s.mode[i - 1]), start, i))
            start = i
    out.append((float(s.mode[-1]), start, len(s.mode)))
    return out


def steady_stats(s: Series, window: float, mode_value=None):
    """取（某个 mode 的）最后 window 秒，返回 (n, meanV, sdV, meanP, sdP, meanBus)。"""
    if s.mode is not None and mode_value is not None:
        idx = np.nonzero(s.mode == mode_value)[0]
    else:
        idx = np.arange(len(s.t))
    if len(idx) == 0:
        return None
    t = s.t[idx]
    sel = idx[t >= (t[-1] - window)] if window > 0 else idx
    if len(sel) < 3:
        return None
    v = s.vel[sel]
    p = s.power[sel]
    b = s.bus[sel]
    return (len(sel), float(np.mean(v)), float(np.std(v, ddof=1)),
            float(np.mean(p)), float(np.std(p, ddof=1)), float(np.nanmean(b)))


def print_table(series_list, window: float):
    print("=" * 96)
    print(f"{'文件':<28}{'模式':<12}{'n':>6}{'v_mean':>10}{'sd_v':>9}{'p_mean':>9}{'sd_p':>9}{'busV':>8}")
    print("-" * 96)
    for s in series_list:
        if s.kind == "sweep":
            # 扫描文件没有单一工作点，整段统计无意义（看 flywheel_sweep.png）
            print(f"{s.name[:27]:<28}{'扫描(见sweep图)':<12}{len(s.t):>6}{'-':>10}{'-':>9}{'-':>9}{'-':>9}{'-':>8}")
            continue
        modes = [None] if s.mode is None else sorted(set(float(m) for m in s.mode))
        for m in modes:
            st = steady_stats(s, window, m)
            if st is None:
                continue
            label = "-" if m is None else ("主控制" if m == 0 else "对照")
            print(f"{s.name[:27]:<28}{label:<12}{st[0]:>6}{st[1]:>10.0f}{st[2]:>9.1f}"
                  f"{st[3]:>9.4f}{st[4]:>9.4f}{st[5]:>8.2f}")
    print("=" * 96)


def print_bench(series_list):
    rows = []
    for s in series_list:
        rows.extend(s.bench)
    if not rows:
        return
    print("\n基准表（Y 键 / 阶段①自动记录）—— 三阶段横向对比：")
    print(f"{'tag':<20}{'v':>9}{'p':>9}{'sd_v':>8}{'sd_p':>9}{'busV':>8}")
    print("-" * 66)
    for tag, v, p, sdv, sdp, bus in rows:
        print(f"{tag[:19]:<20}{v:>9.0f}{p:>9.4f}{sdv:>8.1f}{sdp:>9.4f}{bus:>8.2f}")
    print("-" * 66)
    print("读法：v 越接近目标越好；sd_v 越小越稳；p 是控制代价；sd_p 大说明占空比在抖。\n")


# ---------------------------------------------------------------------------
# 绘图
# ---------------------------------------------------------------------------

def setup_mpl(show: bool):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def save_fig(plt, fig, path, dpi=150):
    """统一的存图口径：一图一文件、dpi=150、bbox_inches='tight'。"""
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return path


#: 三项"对比"物理量的出图规格：(文件名后缀, y 轴标签, 标题中段, 取数函数)
COMPARE_SPECS = (
    ("velocity", "转速 (tick/s)", "转速 ω", lambda s: s.vel),
    ("duty", "占空比 (power, 0~1)", "占空比 power", lambda s: s.power),
    ("voltage", "母线电压 (V)", "母线电压 busV", lambda s: s.bus),
)


def plot_compare(plt, series_list, out_dir):
    """飞轮对照：速度 / 占空比 / 母线电压 —— **每个物理量各出一张 PNG**。"""
    comp = [s for s in series_list if s.kind == "compare"]
    if not comp:
        return []
    styles = ["-", "--", "-.", ":"]
    colors = {0.0: "#1f77b4", 1.0: "#d62728"}
    made = []
    for key, ylab, title, getter in COMPARE_SPECS:
        fig, ax = plt.subplots(figsize=(9, 4))
        for k, s in enumerate(comp):
            arr = getter(s)
            for m, i0, i1 in segments(s):
                if i1 - i0 < 3:
                    continue
                c = colors.get(m, "#7f7f7f")
                lab = f"{s.name[:22]} mode={int(m)}"
                ax.plot(s.t[i0:i1], arr[i0:i1], color=c, ls=styles[k % 4], lw=1.2, label=lab)
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel(ylab)
        ax.set_title(f"飞轮对照 — {title}（按 mode 分色；实线/虚线区分文件）")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, ncol=2, loc="best")
        if key == "duty":
            ax.axhline(1.0, color="r", lw=1.0)
            ax.axhline(0.0, color="r", lw=1.0)
            ax.annotate("红实线 = 限幅上下限 0 / 1", xy=(0.01, 0.02), xycoords="axes fraction",
                        fontsize=8, color="r", va="bottom")
        if key == "voltage":
            ax.axhspan(11.5, 13.0, color="green", alpha=0.08)
            ax.annotate("绿带 = 正常母线区间 11.5~13.0 V", xy=(0.01, 0.98),
                        xycoords="axes fraction", fontsize=8, color="green", va="top")
        made.append(save_fig(plt, fig, os.path.join(out_dir, f"flywheel_compare_{key}.png")))
    return made


def sweep_levels(s: Series):
    """按占空比档位分组，返回 (ps, vs, sds, buses) —— 每级的均值/标准差/母线电压。"""
    p = s.power
    levels = sorted(set(np.round(p, 4)))
    ps, vs, sds, buses = [], [], [], []
    for lv in levels:
        sel = np.abs(p - lv) < 1e-6
        if np.count_nonzero(sel) < 3:
            continue
        ps.append(lv)
        vs.append(float(np.mean(s.vel[sel])))
        sds.append(float(np.std(s.vel[sel], ddof=1)))
        buses.append(float(np.nanmean(s.bus[sel])))
    return ps, vs, sds, buses


def plot_sweep(plt, series_list, out_dir):
    """阶段① 扫描：速度-功率 / 电压-功率 —— **每张图一个物理量**。"""
    sw = [s for s in series_list if s.kind == "sweep"]
    if not sw:
        return []
    made = []

    # ① 速度-功率
    fig, ax = plt.subplots(figsize=(9, 4))
    for s in sw:
        ps, vs, sds, _ = sweep_levels(s)
        if not ps:
            continue
        ax.errorbar(ps, vs, yerr=sds, marker="o", ms=4, capsize=3, lw=1.2, label=s.name[:24])
        imax = int(np.argmax(vs))
        ax.annotate(f"极限 {vs[imax]:.0f} tick/s @ power={ps[imax]:.2f}",
                    xy=(ps[imax], vs[imax]),
                    xytext=(ps[imax] * 0.55, vs[imax] * 0.95),
                    arrowprops=dict(arrowstyle="->", color="k", lw=1), fontsize=9)
    ax.set_xlabel("占空比 power (0~1)")
    ax.set_ylabel("转速 (tick/s)")
    ax.set_title("阶段① 无控制器扫描 — 速度-功率（误差棒 = 该档位的 sd）")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    made.append(save_fig(plt, fig, os.path.join(out_dir, "flywheel_sweep_velocity.png")))

    # ② 电压-功率
    fig, ax = plt.subplots(figsize=(9, 4))
    for s in sw:
        ps, _, _, buses = sweep_levels(s)
        if not ps:
            continue
        ax.plot(ps, buses, "o-", ms=4, lw=1.2, label=f"{s.name[:18]} 母线 busV")
        ax.plot(ps, np.array(ps) * np.array(buses), "s--", ms=4, lw=1.0,
                label=f"{s.name[:18]} 端电压≈busV×power")
    ax.axhspan(11.5, 13.0, color="green", alpha=0.08)
    ax.set_xlabel("占空比 power (0~1)")
    ax.set_ylabel("电压 (V)")
    ax.set_title("阶段① 无控制器扫描 — 母线电压 / 端电压 vs 功率")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    ax.annotate("绿带 = 正常母线区间 11.5~13.0 V", xy=(0.01, 0.98), xycoords="axes fraction",
                fontsize=8, color="green", va="top")
    made.append(save_fig(plt, fig, os.path.join(out_dir, "flywheel_sweep_voltage.png")))
    return made


def sweep_power_model(plt, series_list, out_dir):
    """阶段① 补充图：由扫描反算的 Kd 与 kff（转速/占空比 → 前馈增益）。"""
    sw = [s for s in series_list if s.kind == "sweep"]
    if not sw:
        return []
    made = []
    fig, ax = plt.subplots(figsize=(9, 4))
    for s in sw:
        ps, vs, _, _ = sweep_levels(s)
        if len(ps) < 2:
            continue
        kd, p0 = np.polyfit(ps, vs, 1)      # vs = kd*power + p0
        xx = np.linspace(0, max(ps) * 1.05, 50)
        ax.plot(xx, kd * xx + p0, "-", lw=1.2,
                label=f"{s.name[:18]} Kd≈{kd:.0f} (tick/s)/duty, 截距 {p0:.0f}")
        ax.plot(ps, vs, "o", ms=4)
    ax.set_xlabel("占空比 power (0~1)")
    ax.set_ylabel("转速 (tick/s)")
    ax.set_title("阶段① 扫描线性化 — 斜率即 Kd（kff = 1/Kd）")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    made.append(save_fig(plt, fig, os.path.join(out_dir, "flywheel_sweep_kd.png")))
    return made


BENCH_SPECS = (
    ("velocity", 1, "稳态转速均值 v (tick/s)", "#1f77b4", "v"),
    ("sd", 3, "转速标准差 sd_v (tick/s) —— 越小越稳", "#d62728", "sd_v"),
    ("duty", 2, "稳态占空比 p (0~1) —— 控制代价", "#2ca02c", "p"),
    ("voltage", 5, "稳态母线电压 busV (V) —— 正常 11.5~13.0 V", "#ff7f0e", "busV"),
)


def plot_bench(plt, series_list, out_dir):
    """基准表（# bench 行）横向对比：v / sd_v / p —— **每项各出一张柱状图**。"""
    rows = []
    for s in series_list:
        rows.extend(s.bench)
    if not rows:
        return []
    # 去掉重复 tag（后出现者覆盖），保持顺序
    seen = {}
    order = []
    for r in rows:
        if r[0] not in seen:
            order.append(r[0])
        seen[r[0]] = r
    rows = [seen[k] for k in order]

    tags = [r[0] for r in rows]
    x = np.arange(len(tags))
    made = []
    for key, col, title, color, ylab in BENCH_SPECS:
        data = [r[col] for r in rows]
        fig, ax = plt.subplots(figsize=(9, 4))
        bb = ax.bar(x, data, color=color, width=0.6)
        for rect, val in zip(bb, data):
            ax.text(rect.get_x() + rect.get_width() / 2, val,
                    f"{val:.0f}" if abs(val) > 10 else f"{val:.4f}",
                    ha="center", va="bottom", fontsize=7)
        ax.set_xticks(x)
        ax.set_xticklabels(tags, fontsize=8, rotation=20, ha="right")
        ax.set_xlabel("基准表 tag（阶段①极限 / ②PID vs 开环 / ③控制律 vs PID）")
        ax.set_ylabel(ylab)
        ax.set_title(f"三阶段基准对比 — {title}")
        ax.grid(alpha=0.3, axis="y")
        made.append(save_fig(plt, fig, os.path.join(out_dir, f"flywheel_bench_{key}.png")))
    return made


def collect(paths):
    files = []
    for p in paths:
        if os.path.isdir(p):
            files.extend(sorted(glob.glob(os.path.join(p, "*.csv"))))
        elif any(c in p for c in "*?"):
            files.extend(sorted(glob.glob(p)))
        else:
            files.append(p)
    out = []
    for f in files:
        if f not in out:
            out.append(f)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="飞轮三阶段调试数据出图与对比")
    ap.add_argument("inputs", nargs="+", help="CSV、目录或通配符")
    ap.add_argument("--out", default=None, help="输出目录（默认第一个输入所在目录）")
    ap.add_argument("--window", type=float, default=2.0, help="稳态统计窗口 (s)，默认 2.0")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--show", action="store_true")
    cfg = ap.parse_args(argv)

    files = [f for f in collect(cfg.inputs) if os.path.isfile(f)]
    if not files:
        print("没有找到输入 CSV", file=sys.stderr)
        return 2

    series_list = [read_csv(f) for f in files]
    out_dir = cfg.out or os.path.dirname(os.path.abspath(files[0]))
    os.makedirs(out_dir, exist_ok=True)

    print_table(series_list, cfg.window)
    print_bench(series_list)

    if not cfg.no_plot:
        try:
            plt = setup_mpl(cfg.show)
        except ImportError as e:
            print(f"[跳过绘图] {e}", file=sys.stderr)
            return 0
        made = []
        made += plot_compare(plt, series_list, out_dir)
        made += plot_sweep(plt, series_list, out_dir)
        made += sweep_power_model(plt, series_list, out_dir)
        made += plot_bench(plt, series_list, out_dir)
        for m in made:
            print(f"[图] {m}")
        if not made:
            print("没有可绘制的数据（需 fly1_* 扫描文件或 fly2_*/fly3_* 对照文件）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
