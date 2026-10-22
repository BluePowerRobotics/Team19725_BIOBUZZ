#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞轮「发射掉速事件」分析：每一次掉速的深度、恢复时间、补偿是否生效。

配合 analyze_dashboard_log.py 使用（复用其读盘与时间轴重建）。专用于**带球的实测日志**：
把每一次速度跌落当成一次发射事件，逐个量化：

    跌幅 depth = ω* − min(ω)          恢复时间 = 从跌破判定线到连续回到误差带
    恢复期 duty 峰值 / 是否顶饱和      恢复期电流峰值
    补偿 boost 是否参与（触发时刻占比、boost term 峰值）
    恢复后是否过冲（回到带内之后的最大超调）

输出（**一图一文件**；`<stem>` = 日志文件名去扩展名，`<NNN>` = 事件序号三位补零）：
    <stem>_launch_velocity.png          全程转速 + 目标，掉速事件区间着色
    <stem>_launch_duty.png              全程占空比与 I / P 分量（看恢复期是否顶到饱和）
    <stem>_launch_boost.png             全程 boost 标志与 boost term（补偿是否参与）
    <stem>_launch_001.png … _NNN.png    每一次事件单独一张（转速 + 占空比/I，逐发可贴报告）
    <stem>_launch_stats_depth_vs_recover.png  跌幅-恢复时间散点（颜色 = boost 占比%）
    <stem>_launch_stats_depth_hist.png        跌幅直方图
    <stem>_launch_stats_recover_hist.png      恢复时间直方图
    <stem>_launch_events.csv            每次事件的总表（CSV，便于 Excel/评审）
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
    add(f"事件判定：速度低于 ω*−max(100, {3.0 * cfg.target_band:.0%}·ω*)"
        f"（delta = max(100, 3×band_frac×ω*)）；"
        f"恢复判定：|e| ≤ max({cfg.target_band:.0%}·ω*, 1 个台阶) 连续 10 帧")
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

    # 每次事件一行，写成总表 CSV（评审 / Excel 直接开）
    csv_path = os.path.join(out, stem + "_launch_events.csv")
    with open(csv_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("idx,target,t0_s,recover_s,depth_tick_s,depth_pct,vmin,"
                 "duty_max,is_saturated,i_min,p_max,current_max,boost_frac,boost_max,overshoot\n")
        for idx, e in enumerate(ev, 1):
            fh.write("%d,%.0f,%.4f,%.4f,%.1f,%.2f,%.1f,%.4f,%d,%.4f,%.4f,%.3f,%.4f,%.4f,%.1f\n" % (
                idx, e["target"], e["t0"], e["recover"], e["depth"],
                100.0 * e["depth"] / e["target"] if e["target"] else float("nan"), e["vmin"],
                e["duty_max"], 1 if e["duty_max"] >= 0.999 else 0, e["i_min"], e["p_max"],
                e["cur_max"], e["boost_frac"], e["boost_max"], e["overshoot"]))
    print("[表] " + csv_path)

    if not cfg.no_plot:
        try:
            plot(t, tgt, vel, duty, it, pt, boost, A_bt, ev, out, stem)
        except Exception as ex:
            print(f"[绘图失败] {type(ex).__name__}: {ex}", file=sys.stderr)
    return 0


def plot(t, tgt, vel, duty, it, pt, boost, bt, ev, out, stem):
    """**一图一文件**：全程 3 张（转速/占空比/boost）+ 每发事件 1 张 + 统计 3 张。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    made = []

    def save(fig, suffix):
        fp = os.path.join(out, f"{stem}_{suffix}.png")
        fig.savefig(fp, dpi=150, bbox_inches="tight")
        plt.close(fig)
        made.append(fp)
        return fp

    step = A.gcd_step(vel)
    dep = np.array([e["depth"] for e in ev])
    rec = np.array([e["recover"] for e in ev])
    bf = np.array([e["boost_frac"] for e in ev])
    dm = np.array([e["duty_max"] for e in ev])
    ov = np.array([e["overshoot"] for e in ev])

    # ---- ① 全程转速 ----
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, tgt, "r--", lw=1.0, label="目标 ω*")
    ax.plot(t, vel, "b-", lw=0.8, label="实测 ω")
    for e in ev:
        ax.axvspan(e["t0"], e["t1"], color="orange", alpha=0.35)
    ax.annotate(f"橙色 = 掉速事件区间（共 {len(ev)} 次）\n"
                f"跌幅中位 {np.median(dep):.0f} tick/s"
                f"（{100.0 * np.median(dep) / np.median([e['target'] for e in ev]):.2f}%）",
                xy=(0.01, 0.02), xycoords="axes fraction", fontsize=8, va="bottom",
                bbox=dict(boxstyle="round", fc="white", alpha=0.85))
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("转速 (tick/s)")
    ax.set_title(f"{stem} — 发射掉速事件：转速（目标 vs 实测）")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    save(fig, "launch_velocity")

    # ---- ② 全程占空比与分量 ----
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, duty, color="#d97a26", lw=0.8, label="duty")
    ax.plot(t, it, color="#2ca02c", lw=0.7, label="I 项")
    ax.plot(t, pt, color="#9467bd", lw=0.7, label="P 项")
    ax.axhline(1.0, color="r", lw=1.0)
    for e in ev:
        ax.axvspan(e["t0"], e["t1"], color="orange", alpha=0.2)
    ax.annotate(f"恢复期顶到饱和（duty≥0.999）的事件占 {100.0 * np.mean(dm >= 0.999):.0f}%\n"
                f"顶尖饱和 = 恢复速度受母线电压/电机能力限制，加大增益没有用",
                xy=(0.01, 0.02), xycoords="axes fraction", fontsize=8, va="bottom",
                bbox=dict(boxstyle="round", fc="white", alpha=0.85))
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("占空比 (0~1)")
    ax.set_title(f"{stem} — 发射掉速事件：占空比与 I / P 分量")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=3)
    save(fig, "launch_duty")

    # ---- ③ 全程 boost ----
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, np.asarray(boost) * 100, color="g", lw=0.8, label="boost 标志 ×100（>50 即触发）")
    if bt is not None:
        ax.plot(t, np.nan_to_num(bt) * 100, color="k", lw=0.7, label="boost term ×100")
        ax.axhline(39.9, color="r", ls="--", lw=1.0, label="boostMax 限幅 0.400 ×100")
    for e in ev:
        ax.axvspan(e["t0"], e["t1"], color="orange", alpha=0.2)
    ax.annotate(f"boost 参与事件比例 {100.0 * np.mean(bf > 0.5):.0f}%\n"
                f"boost term 峰值 {max(e['boost_max'] for e in ev):.3f}"
                f"（≥0.399 说明顶到 boostMax 限幅）",
                xy=(0.01, 0.98), xycoords="axes fraction", fontsize=8, va="top",
                bbox=dict(boxstyle="round", fc="white", alpha=0.85))
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("补偿（×100 显示）")
    ax.set_title(f"{stem} — 掉速补偿 boost 是否参与恢复")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    save(fig, "launch_boost")

    # ---- ④ 每次事件单独一张 ----
    dt = float(t[1] - t[0]) if len(t) > 1 else 0.02
    npre = max(1, int(round(0.3 / dt)))
    npost = max(1, int(round(1.0 / dt)))
    for idx, e in enumerate(ev, 1):
        a = max(0, e["j0"] - npre)
        b = min(len(t), e["j1"] + npost)
        tt = t[a:b] - t[e["j0"]]
        band = max(0.02 * e["target"], 1.0 * step)
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot(tt, vel[a:b], "b.-", ms=2.5, lw=0.9, label="实测 ω")
        ax.axhline(e["target"], color="r", ls="--", lw=1.0,
                   label=f"目标 {e['target']:.0f} tick/s")
        ax.axhline(e["target"] - band, color="orange", ls=":", lw=1.0,
                   label=f"恢复判定带 −{band:.0f}（{max(2.0, 100.0 * band / e['target']):.0f}%）")
        ax.axhline(e["vmin"], color="purple", ls=":", lw=1.0,
                   label=f"最低 {e['vmin']:.0f}（跌幅 {e['depth']:.0f}）")
        ax.axvspan(0, e["t1"] - e["t0"], color="orange", alpha=0.25)
        ax.annotate(f"跌幅 {e['depth']:.0f} tick/s（{100.0 * e['depth'] / e['target']:.1f}%）\n"
                    f"恢复 {e['recover']:.3f} s    duty 峰 {e['duty_max']:.3f}"
                    f"{'（饱和）' if e['duty_max'] >= 0.999 else ''}\n"
                    f"boost 占比 {100.0 * e['boost_frac']:.0f}%  峰值 {e['boost_max']:.3f}\n"
                    f"恢复后过冲 {e['overshoot']:+.0f} tick/s（负 = 未过冲）",
                    xy=(0.02, 0.02), xycoords="axes fraction", fontsize=8, va="bottom",
                    bbox=dict(boxstyle="round", fc="white", alpha=0.85))
        ax.set_xlabel("相对事件起点的时间 (s)")
        ax.set_ylabel("转速 (tick/s)")
        ax.set_title(f"{stem} — 第 {idx}/{len(ev)} 发：目标 {e['target']:.0f} tick/s")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="lower right")
        save(fig, f"launch_{idx:03d}")

    # ---- ⑤ 跌幅 vs 恢复时间 ----
    fig, ax = plt.subplots(figsize=(9, 4))
    sc = ax.scatter(dep, rec, c=bf * 100, cmap="viridis", s=30)
    fig.colorbar(sc, ax=ax, label="boost 占比 (%)")
    ax.axhline(0.3, color="r", ls="--", lw=1.0, label="0.3 s 目标线（比赛要求 <0.3~0.5 s）")
    ax.annotate(f"恢复时间中位 {np.median(rec):.3f} s / 最长 {rec.max():.3f} s\n"
                f"<0.3 s 的比例 {100.0 * np.mean(rec < 0.3):.0f}%",
                xy=(0.02, 0.98), xycoords="axes fraction", fontsize=8, va="top",
                bbox=dict(boxstyle="round", fc="white", alpha=0.85))
    ax.set_xlabel("跌幅 (tick/s)")
    ax.set_ylabel("恢复时间 (s)")
    ax.set_title(f"{stem} — 跌幅 vs 恢复时间（颜色 = boost 占比%）")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    save(fig, "launch_stats_depth_vs_recover")

    # ---- ⑥ 跌幅分布 ----
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.hist(dep, bins=15, color="#1f77b4")
    ax.axvline(np.median(dep), color="r", ls="--", lw=1.2,
               label=f"中位 {np.median(dep):.0f} tick/s")
    ax.set_xlabel("跌幅 (tick/s)")
    ax.set_ylabel("事件数")
    ax.set_title(f"{stem} — 跌幅分布（{len(ev)} 次事件）")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8)
    save(fig, "launch_stats_depth_hist")

    # ---- ⑦ 恢复时间分布 ----
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.hist(rec, bins=15, color="#d62728")
    ax.axvline(np.median(rec), color="k", ls="--", lw=1.2,
               label=f"中位 {np.median(rec):.3f} s")
    ax.axvline(0.3, color="r", ls=":", lw=1.2, label="0.3 s 目标线")
    ax.set_xlabel("恢复时间 (s)")
    ax.set_ylabel("事件数")
    ax.set_title(f"{stem} — 恢复时间分布")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8)
    save(fig, "launch_stats_recover_hist")

    print("[图] " + "  ".join(os.path.basename(p) for p in made))
    return made


if __name__ == "__main__":
    sys.exit(main())
