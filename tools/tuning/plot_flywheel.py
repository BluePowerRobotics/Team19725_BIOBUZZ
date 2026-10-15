#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞轮三阶段调试数据出图 / 对比（配合 OpModes/Flywheel 的四个 OpMode）。

读取 `<RC 外部存储>/flywheel/*.csv`（列：t,power,velocity,busV,motorV[,mode]），自动识别：
  fly1_*  = 阶段① 无控制器功率扫描（sweep）      → 画 速度-功率 曲线 + 极限点
  fly2_*  = 阶段② 单 PID ⇄ 开环对照（含 mode 列） → 画 时间序列（按 mode 分色）
  fly3_*  = 阶段③ 现有控制律 ⇄ 纯 PID 对照        → 同上

输出（默认写到 --out 目录）：
  flywheel_compare.png  所有对照文件的速度/占空比/电压时间序列叠加图（跨文件比较）
  flywheel_sweep.png    阶段① 的 速度-功率 与 电压-功率 扫描曲线（标出极限点）
  flywheel_bench.png    基准表（# bench 行）的 v / p / sd_v 对比柱状图 —— 三阶段横向对比
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


def plot_compare(plt, series_list, out_png):
    comp = [s for s in series_list if s.kind == "compare"]
    if not comp:
        return False
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    styles = ["-", "--", "-.", ":"]
    colors = {0.0: "#1f77b4", 1.0: "#d62728"}
    for k, s in enumerate(comp):
        for m, i0, i1 in segments(s):
            if i1 - i0 < 3:
                continue
            c = colors.get(m, "#7f7f7f")
            lab = f"{s.name[:22]} mode={int(m)}"
            axes[0].plot(s.t[i0:i1], s.vel[i0:i1], color=c, ls=styles[k % 4], lw=1.2, label=lab)
            axes[1].plot(s.t[i0:i1], s.power[i0:i1], color=c, ls=styles[k % 4], lw=1.2)
            axes[2].plot(s.t[i0:i1], s.bus[i0:i1], color=c, ls=styles[k % 4], lw=1.2)
    axes[0].set_ylabel("转速 (tick/s)")
    axes[0].set_title("飞轮对照：速度 / 占空比 / 母线电压（按 mode 分色）")
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=7, ncol=2)
    axes[1].set_ylabel("占空比 (power)")
    axes[1].grid(alpha=0.3)
    axes[2].set_ylabel("母线电压 (V)")
    axes[2].set_xlabel("时间 (s)")
    axes[2].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    return True


def plot_sweep(plt, series_list, out_png):
    sw = [s for s in series_list if s.kind == "sweep"]
    if not sw:
        return False
    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    for s in sw:
        # 每级取末段均值：按占空比分组
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
        axes[0].errorbar(ps, vs, yerr=sds, marker="o", ms=4, capsize=3, lw=1.2, label=s.name[:24])
        axes[1].plot(ps, buses, "o-", ms=4, lw=1.2, label=f"{s.name[:18]} 母线")
        axes[1].plot(ps, np.array(ps) * np.array(buses), "s--", ms=4, lw=1.0,
                     label=f"{s.name[:18]} 端电压≈busV×power")
        if ps:
            imax = int(np.argmax(vs))
            axes[0].annotate(f"极限 {vs[imax]:.0f} @ p={ps[imax]:.2f}",
                             xy=(ps[imax], vs[imax]), xytext=(ps[imax] * 0.55, vs[imax] * 0.95),
                             arrowprops=dict(arrowstyle="->", color="k", lw=1), fontsize=9)
    axes[0].set_xlabel("占空比 power")
    axes[0].set_ylabel("转速 (tick/s)")
    axes[0].set_title("阶段① 无控制器扫描：速度-功率（误差棒=各级 sd）")
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[1].set_xlabel("占空比 power")
    axes[1].set_ylabel("电压 (V)")
    axes[1].set_title("阶段① 电压-功率")
    axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    return True


def plot_bench(plt, series_list, out_png):
    rows = []
    for s in series_list:
        rows.extend(s.bench)
    if not rows:
        return False
    # 去掉重复 tag（后出现者覆盖），保持顺序
    seen = {}
    order = []
    for r in rows:
        if r[0] not in seen:
            order.append(r[0])
        seen[r[0]] = r
    rows = [seen[k] for k in order]

    tags = [r[0] for r in rows]
    v = [r[1] for r in rows]
    p = [r[2] for r in rows]
    sdv = [r[3] for r in rows]

    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    x = np.arange(len(tags))
    for ax, data, title, color in (
            (axes[0], v, "稳态转速均值 v (tick/s)", "#1f77b4"),
            (axes[1], sdv, "转速标准差 sd_v (tick/s)  —— 越小越稳", "#d62728"),
            (axes[2], p, "稳态占空比 p (0~1)  —— 控制代价", "#2ca02c")):
        bb = ax.bar(x, data, color=color, width=0.6)
        ax.set_title(title)
        ax.set_xticks(x)
        ax.set_xticklabels(tags, fontsize=8, rotation=20, ha="right")
        ax.grid(alpha=0.3, axis="y")
        for rect, val in zip(bb, data):
            ax.text(rect.get_x() + rect.get_width() / 2, val, f"{val:.0f}" if abs(val) > 10 else f"{val:.4f}",
                    ha="center", va="bottom", fontsize=7)
    fig.suptitle("三阶段基准表对比（阶段①极限 / ②PID vs 开环 / ③控制律 vs PID）")
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    return True


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
        if plot_compare(plt, series_list, os.path.join(out_dir, "flywheel_compare.png")):
            made.append("flywheel_compare.png")
        if plot_sweep(plt, series_list, os.path.join(out_dir, "flywheel_sweep.png")):
            made.append("flywheel_sweep.png")
        if plot_bench(plt, series_list, os.path.join(out_dir, "flywheel_bench.png")):
            made.append("flywheel_bench.png")
        for m in made:
            print(f"[图] {os.path.join(out_dir, m)}")
        if not made:
            print("没有可绘制的数据（需 fly1_* 扫描文件或 fly2_*/fly3_* 对照文件）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
