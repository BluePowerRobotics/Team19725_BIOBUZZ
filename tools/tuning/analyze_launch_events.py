#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞轮「发射掉速事件」分析：每一次掉速的深度、恢复时间、补偿是否生效。

配合 analyze_dashboard_log.py 使用（复用其读盘与时间轴重建）。专用于**带球的实测日志**：
把每一次速度跌落当成一次发射事件，逐个量化：

    跌幅 depth = ω* − min(ω)          恢复时间 = 从跌破判定线到连续回到误差带
    恢复期 duty 峰值 / 是否顶饱和      恢复期电流峰值
    补偿 boost 是否参与（触发时刻占比、boost term 峰值）
    恢复后是否过冲（回到带内之后的最大超调）

输出：
    <stem>_launch_overview.png   全程速度/占空比/boost，掉速区间着色
    <stem>_launch_zoom.png       典型 4 次事件放大（速度 + 占空比 + 分量）
    <stem>_launch_stats.png      跌幅-恢复时间散点、跌幅直方图、恢复时间直方图
    终端打印逐次事件表与统计汇总

用法：
    python analyze_launch_events.py <log.csv> [--target-band 0.02] [--min-target 500]
                                    [--out DIR] [--no-plot]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import analyze_dashboard_log as A
except ImportError:
    print("需要与本脚本同目录的 analyze_dashboard_log.py", file=sys.stderr)
    raise


def detect_events(t, tgt, vel, duty, it, pt, boost, dt, target_band, min_target, step):
    """在每一段稳态里检测掉速事件；返回事件列表。"""
    events = []
    for tv, i0, i1 in A.segments(tgt, t):
        if tv < min_target or (i1 - i0) * dt < 2.0:
            continue
        band = max(target_band * tv, 1.0 * step)
        # 跳过起始加速段：从第一次 |e| < 2*band 持续 10 帧之后开始
        start = i0
        run = 0
        for j in range(i0, i1):
            if abs(tgt[j] - vel[j]) <= 2 * band:
                run += 1
                if run >= 10:
                    start = j
                    break
            else:
                run = 0
        delta = max(100.0, 3.0 * target_band * tv)
        j = start
        while j < i1:
            if tgt[j] - vel[j] > delta:                      # 跌破判定线
                k = j + 1
                vmin = vel[j]
                while k < i1 and tgt[k] - vel[k] > band:     # 直到连续 10 帧回到带内
                    if vel[k] < vmin:
                        vmin = vel[k]
                    k += 1
                # 找连续 10 帧达标的恢复点
                end = k
                run = 0
                for q in range(j + 1, min(i1, j + 4000)):
                    if abs(tgt[q] - vel[q]) <= band:
                        run += 1
                        if run >= 10:
                            end = q - 9
                            break
                    else:
                        run = 0
                seg = slice(j, max(end + 1, j + 1))
                ov = slice(end, min(i1, end + int(2.0 / dt)))
                events.append(dict(
                    target=tv, t0=t[j], t1=t[min(end, i1 - 1)],
                    depth=tv - vmin, vmin=vmin,
                    recover=t[min(end, i1 - 1)] - t[j],
                    duty_max=float(np.nanmax(duty[seg])),
                    duty_mean=float(np.nanmean(duty[seg])),
                    i_min=float(np.nanmin(it[seg])),
                    i_max=float(np.nanmax(it[seg])),
                    p_max=float(np.nanmax(np.abs(pt[seg]))),
                    cur_max=float(np.nanmax(A_cur[seg])) if A_cur is not None else float("nan"),
                    boost_frac=float(np.mean(boost[seg] > 0.5)) if boost is not None else 0.0,
                    boost_max=float(np.nanmax(A_bt[seg])) if A_bt is not None else 0.0,
                    overshoot=float(np.nanmax(vel[ov]) - tv) if ov.stop > ov.start else 0.0,
                    j0=j, j1=min(end, i1 - 1)))
                j = max(end, j + 1)
            else:
                j += 1
    return events


A_cur = None
A_bt = None


def main(argv=None):
    ap = argparse.ArgumentParser(description="飞轮发射掉速事件分析")
    ap.add_argument("log")
    ap.add_argument("--out", default=None)
    ap.add_argument("--target-band", type=float, default=0.02, help="恢复判定带（占目标比例），默认 0.02")
    ap.add_argument("--min-target", type=float, default=500.0)
    ap.add_argument("--no-plot", action="store_true")
    cfg = ap.parse_args(argv)
    out = cfg.out or os.path.dirname(os.path.abspath(cfg.log))
    os.makedirs(out, exist_ok=True)

    global A_cur, A_bt
    rows, header, cols, labels, d = A.load(cfg.log)
    t, span, dt, back, bad = A.rebuild_time(labels)
    vel, tgt, duty, it, pt, ff = d["velocity"], d["target"], d["duty"], d["i"], d["p"], d["ff"]
    boost, A_cur, A_bt = d["boostflag"], d["current"], d["boostterm"]
    n = len(t)
    step = A.gcd_step(vel)
    stem = os.path.splitext(os.path.basename(cfg.log))[0].replace(" ", "_")

    L = []
    add = L.append
    add("=" * 92)
    add(f"发射掉速事件分析：{os.path.basename(cfg.log)}")
    add(f"样本 {n}    时长 {span:.2f} s    采样间隔 {dt * 1000:.1f} ms    量化台阶 {step:.0f} tick/s")
    add(f"事件判定：速度低于 ω*−max(100, 3%·ω*)；恢复判定：|e| ≤ max({cfg.target_band:.0%}·ω*, 1 个台阶) 连续 10 帧")
    add("=" * 92)

    ev = detect_events(t, tgt, vel, duty, it, pt, boost, dt, cfg.target_band, cfg.min_target, step)
    if not ev:
        add("未检测到掉速事件（确认这批日志里确实投球了）")
        print("\n".join(L))
        return 0

    add(f"\n共 {len(ev)} 次事件")
    add(f"{'#':>3}{'目标':>7}{'起始s':>8}{'跌幅':>7}{'跌%':>7}{'最低v':>8}{'恢复s':>8}"
        f"{'duty峰':>8}{'duty饱和':>9}{'I项min':>8}{'P项峰':>7}{'电流峰':>7}{'boost占比':>10}{'boost峰':>9}{'恢复后过冲':>11}")
    for idx, e in enumerate(ev, 1):
        add(f"{idx:>3}{e['target']:>7.0f}{e['t0']:>8.2f}{e['depth']:>7.0f}"
            f"{100.0 * e['depth'] / e['target']:>7.2f}{e['vmin']:>8.0f}{e['recover']:>8.3f}"
            f"{e['duty_max']:>8.3f}{('是' if e['duty_max'] >= 0.999 else '否'):>9}"
            f"{e['i_min']:>8.3f}{e['p_max']:>7.3f}{e['cur_max']:>7.2f}"
            f"{100.0 * e['boost_frac']:>9.0f}%{e['boost_max']:>9.3f}{e['overshoot']:>11.0f}")

    dep = np.array([e["depth"] for e in ev])
    rec = np.array([e["recover"] for e in ev])
    ov = np.array([e["overshoot"] for e in ev])
    bf = np.array([e["boost_frac"] for e in ev])
    dm = np.array([e["duty_max"] for e in ev])
    add("\n--- 统计汇总 ---")
    add(f"跌幅    ：中位 {np.median(dep):.0f} tick/s    最大 {dep.max():.0f}     最小 {dep.min():.0f}")
    add(f"跌幅占比：中位 {100 * np.median(dep) / np.median([e['target'] for e in ev]):.2f}%")
    add(f"恢复时间：中位 {np.median(rec):.3f} s    最长 {rec.max():.3f}    <0.3 s 的比例 {100 * np.mean(rec < 0.3):.0f}%")
    add(f"恢复后过冲：中位 {np.median(ov):.0f} tick/s    最大 {ov.max():.0f}（负值=未过冲）")
    add(f"boost 参与：{100 * np.mean(bf > 0.5):.0f}% 的事件里 boost 触发；boost term 峰值 {max(e['boost_max'] for e in ev):.3f}")
    add(f"恢复期占空比：{100 * np.mean(dm >= 0.999):.0f}% 的事件顶到饱和（≥0.999）")
    add(f"事件间隔：中位 {np.median(np.diff([e['t0'] for e in ev])):.2f} s" if len(ev) > 1 else "")

    # 诊断
    add("\n--- 诊断 ---")
    if np.mean(bf > 0.5) < 0.6:
        add("  [警告] 多数掉速事件里 boost 没有参与 → 检测阈值偏高或武装规则把它挡住了，恢复完全靠 PI。")
    if max(e["boost_max"] for e in ev) >= 0.399:
        add("  [警告] boost term 已顶到 boostMax 限幅（0.400）→ 补偿强度被限住，可考虑提高 boostMax 或 kBoost。")
    if np.mean(dm >= 0.999) > 0.5:
        add("  [警告] 过半事件恢复期占空比顶到 100% → 恢复速度受母线电压/电机能力限制，加大增益没有用。")
    if np.median(rec) > 0.4:
        add(f"  [警告] 恢复时间中位 {np.median(rec):.2f} s 偏长（比赛要求通常 <0.3~0.5 s）。")
    if np.median(ov) < -50:
        add(f"  [提示] 恢复后普遍欠冲（中位 {np.median(ov):.0f}）→ 恢复过慢，仍有提升空间。")

    text = "\n".join(L)
    print(text)
    with open(os.path.join(out, stem + "_launch_report.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")

    if not cfg.no_plot:
        try:
            plot(t, tgt, vel, duty, it, pt, boost, A_bt, ev, out, stem)
        except Exception as ex:
            print(f"[绘图失败] {type(ex).__name__}: {ex}", file=sys.stderr)
    return 0


def plot(t, tgt, vel, duty, it, pt, boost, bt, ev, out, stem):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    fig, ax = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    ax[0].plot(t, tgt, "r--", lw=1.0, label="目标 ω*")
    ax[0].plot(t, vel, "b-", lw=0.8, label="实测 ω")
    for e in ev:
        ax[0].axvspan(e["t0"], e["t1"], color="orange", alpha=0.35)
    ax[0].set_ylabel("转速 (tick/s)")
    ax[0].set_title(f"{stem}  发射掉速事件（橙色=事件区间，共 {len(ev)} 次）")
    ax[0].grid(alpha=0.3)
    ax[0].legend(fontsize=8)
    ax[1].plot(t, duty, color="#d97a26", lw=0.8, label="duty")
    ax[1].plot(t, it, color="#2ca02c", lw=0.7, label="I")
    ax[1].plot(t, pt, color="#9467bd", lw=0.7, label="P")
    ax[1].axhline(1.0, color="r", lw=1.0)
    ax[1].set_ylabel("占空比")
    ax[1].grid(alpha=0.3)
    ax[1].legend(fontsize=7, ncol=3)
    ax[2].plot(t, np.asarray(boost) * 100, color="g", lw=0.8, label="boost(on=%d)")
    if bt is not None:
        ax[2].plot(t, np.nan_to_num(bt) * 100, color="k", lw=0.7, label="boost term ×100")
    ax[2].set_ylabel("补偿")
    ax[2].set_xlabel("时间 (s)")
    ax[2].grid(alpha=0.3)
    ax[2].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(out, stem + "_launch_overview.png"), dpi=140)
    plt.close(fig)

    # 典型事件放大
    pick = sorted(range(len(ev)), key=lambda i: -ev[i]["depth"])[:4]
    fig, axes = plt.subplots(len(pick), 1, figsize=(12, 2.4 * len(pick)), sharex=False)
    if len(pick) == 1:
        axes = [axes]
    for axi, i in zip(axes, pick):
        e = ev[i]
        a, b = max(0, e["j0"] - int(0.3 / (t[1] - t[0]))), min(len(t), e["j1"] + int(1.0 / (t[1] - t[0])))
        tt = t[a:b] - t[e["j0"]]
        axi.plot(tt, vel[a:b], "b.-", ms=2.5, lw=0.8, label="ω")
        axi.axhline(e["target"], color="r", ls="--", lw=1.0)
        axi.axvspan(0, e["t1"] - e["t0"], color="orange", alpha=0.25)
        axi2 = axi.twinx()
        axi2.plot(tt, duty[a:b], color="#d97a26", lw=1.0, label="duty")
        axi2.plot(tt, np.nan_to_num(it[a:b]), color="#2ca02c", lw=0.9, label="I")
        axi2.set_ylim(-0.1, 1.05)
        axi.set_ylabel("ω (tick/s)")
        axi2.set_ylabel("duty / I")
        axi.set_title(f"事件#{i+1} 目标 {e['target']:.0f}  跌幅 {e['depth']:.0f} ({100*e['depth']/e['target']:.1f}%)"
                      f"  恢复 {e['recover']:.3f}s  duty峰 {e['duty_max']:.3f}  boost峰 {e['boost_max']:.3f}"
                      f"  过冲 {e['overshoot']:.0f}", fontsize=9)
        axi.grid(alpha=0.3)
    axes[-1].set_xlabel("相对事件起点的时间 (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, stem + "_launch_zoom.png"), dpi=140)
    plt.close(fig)

    # 统计图
    dep = np.array([e["depth"] for e in ev])
    rec = np.array([e["recover"] for e in ev])
    bf = np.array([e["boost_frac"] for e in ev])
    fig, ax = plt.subplots(1, 3, figsize=(13, 4))
    ax[0].scatter(dep, rec, c=bf * 100, cmap="viridis", s=30)
    ax[0].set_xlabel("跌幅 (tick/s)")
    ax[0].set_ylabel("恢复时间 (s)")
    ax[0].set_title("跌幅 vs 恢复时间（颜色=boost 占比%）")
    ax[0].grid(alpha=0.3)
    ax[1].hist(dep, bins=15, color="#1f77b4")
    ax[1].set_xlabel("跌幅 (tick/s)")
    ax[1].set_title("跌幅分布")
    ax[2].hist(rec, bins=15, color="#d62728")
    ax[2].set_xlabel("恢复时间 (s)")
    ax[2].set_title("恢复时间分布")
    fig.tight_layout()
    fig.savefig(os.path.join(out, stem + "_launch_stats.png"), dpi=140)
    plt.close(fig)
    print(f"[图] {os.path.join(out, stem)}_launch_overview.png / _launch_zoom.png / _launch_stats.png")


if __name__ == "__main__":
    sys.exit(main())
