#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞轮阶跃响应与稳速误差分析（0 -> 目标 的越阶 + 稳态窗口）

与 analyze_dashboard_log.py 配套：那个脚本看"整段运行"，这个脚本专看**每一次目标阶跃**，
回答两个问题：
  1. 越阶（启动/换速）表现如何 —— 上升时间、超调、进入误差带的时间、饱和时长、电流峰值
  2. 到位后稳速误差如何 —— 误差均值/百分比、转速 sd（几个量化台阶）、占空比 sd 与极差、饱和

输入支持两种：
  - 原始 FTC Dashboard 导出 CSV（7 列标准遥测或带 FF/P/I 的详细遥测）
  - 由 export_clean_table.py 处理好的数据表（*_clean.csv）

**时间轴铁律**：原始日志的 time 列毫秒字段是坏的，一律按文件行序重建均匀时间轴
dt = 总时长/(n-1)，不按 time 排序。

用法：
    python analyze_step_response.py <日志或数据表> [--out DIR] [--tail 1.0]
                                   [--band-frac 0.02] [--band-steps 2] [--no-plot]
"""

import argparse
import csv
import os
import sys
from math import gcd

try:
    import numpy as np
except ImportError:
    print("需要 numpy：pip install numpy", file=sys.stderr)
    raise

# 原始 Dashboard 日志的列名候选（与 analyze_dashboard_log.py 保持一致）
CAND = {
    "ks": ["Shooter kS term", "kS term"],
    "ff": ["Shooter FF term", "Shooter FF", "FF term"],
    "i": ["Shooter I term", "Shooter I", "I term"],
    "p": ["Shooter P term", "Shooter P", "P term"],
    "boostflag": ["Shooter boost", "boost"],
    "boostterm": ["Shooter boost term", "boost term"],
    "current": ["Shooter current (A)", "current (A)"],
    "duty": ["Shooter power (duty)", "Shooter duty", "duty", "power"],
    "error": ["Shooter error (tick/s)", "error (tick/s)"],
    "target": ["Shooter target (tick/s)", "target (tick/s)"],
    "velocity": ["Shooter velocity (tick/s)", "velocity (tick/s)"],
    "voltage": ["Shooter voltage (V)", "voltage (V)"],
}

# 数据表（*_clean.csv）的列名
CLEAN = {"t": "t", "target": "target", "velocity": "velocity", "error": "error",
         "duty": "duty", "ks": "ks", "ff": "ff", "i": "i", "p": "p",
         "boostflag": "boost", "boostterm": "boost_term", "current": "current",
         "voltage": "voltage"}


def pick(header, names):
    for n in names:
        if n in header:
            return n
    return None


def parse_time(s):
    try:
        h, m, rest = s.split(":")
        return int(h) * 3600 + int(m) * 60 + float(rest)
    except Exception:
        return None


def load(path):
    """返回 dict：t/target/velocity/duty/current/voltage/... （缺列填 NaN）"""
    # 数据表（*_clean.csv）带 '#' 元数据行，先剔除再交给 DictReader
    with open(path, newline="", encoding="utf-8-sig", errors="replace") as fh:
        lines = [ln for ln in fh if not ln.lstrip().startswith("#")]
    rows = list(csv.DictReader(lines))
    if not rows:
        raise SystemExit("空文件")
    header = list(rows[0].keys())

    # --- 情况一：处理好的数据表（第一列是 t，且有 velocity/duty） ---
    is_clean = "t" in header and any(c in header for c in ("velocity", "target"))
    if is_clean:
        cols = {k: (v if v in header else None) for k, v in CLEAN.items()}
        t = np.array([float(r["t"]) for r in rows], float)
        n_back = 0
        raw_span = None
    else:
        cols = {k: pick(header, v) for k, v in CAND.items()}
        if cols["velocity"] is None or cols["duty"] is None:
            print("文件列名：", header, file=sys.stderr)
            raise SystemExit("未识别速度/占空比列，请确认是本项目的遥测日志")
        # 按行序重建均匀时间轴（time 列毫秒字段不可靠）
        labels = [parse_time(r.get("time", "")) for r in rows]
        good = [x for x in labels if x is not None]
        if len(good) >= 2:
            span = good[-1] - good[0]
            dt = span / (len(good) - 1)
            t = good[0] + dt * np.arange(len(rows))
            n_back = sum(1 for a, b in zip(good, good[1:]) if b < a)
            raw_span = span
        else:
            t = np.arange(len(rows), dtype=float) * 0.01
            n_back = 0
            raw_span = None

    def num(key, row):
        c = cols.get(key)
        if c is None:
            return float("nan")
        v = row.get(c, "")
        if v is None or v == "":
            return float("nan")
        if isinstance(v, str) and v.lower() in ("true", "false"):
            return 1.0 if v.lower() == "true" else 0.0
        try:
            return float(v)
        except ValueError:
            return float("nan")

    d = {k: np.array([num(k, r) for r in rows], float) for k in CAND}
    d["t"] = t
    if np.all(np.isnan(d["error"])):
        d["error"] = d["target"] - d["velocity"]
    return d, os.path.basename(path), n_back, raw_span


def gcd_step(values):
    """速度量化台阶 = 所有取值的最大公约数（先按 1000 倍取整避免浮点误差）"""
    ints = sorted({int(round(v * 1000)) for v in values if np.isfinite(v)})
    g = 0
    for v in ints:
        g = gcd(g, abs(v))
        if g == 1:
            return 1.0 / 1000.0
    return (g / 1000.0) if g else float("nan")


def segments(t, target, gap=0.05):
    """按目标变化切段，返回 [(i0, i1, target)]"""
    out, start = [], 0
    for i in range(1, len(t) + 1):
        boundary = (i == len(t)) or (target[i] != target[i - 1]) or ((t[i] - t[i - 1]) > gap)
        if not boundary:
            continue
        if i - start >= 3:
            out.append((start, i, float(target[start])))
        start = i
    return out


def settle_index(t, err, t0_idx, band, hold=10, limit_sec=20.0):
    """从 t0 起找第一个「连续 hold 帧 |err| <= band」的位置；找不到返回 None"""
    n = len(t)
    i = t0_idx
    while i < n and t[i] - t[t0_idx] <= limit_sec:
        if abs(err[i]) <= band:
            j = i
            while j + 1 < n and j - i < hold and abs(err[j + 1]) <= band:
                j += 1
            if j - i >= hold - 1:
                return i
            i = j + 1
        else:
            i += 1
    return None


def crossing_time(t, v, i0, i1, level):
    """[i0,i1) 内首次达到 level 的时刻"""
    for k in range(i0, i1):
        if v[k] >= level:
            return t[k]
    return None


def analyze(path, tail=1.0, band_frac=0.02, band_steps=2.0, out_dir=None, do_plot=True):
    d, name, n_back, raw_span = load(path)
    t, tgt, vel, duty, err = d["t"], d["target"], d["velocity"], d["duty"], d["error"]
    cur, vol = d["current"], d["voltage"]
    step = gcd_step(vel)

    L = []
    add = L.append
    add("=" * 96)
    add("飞轮阶跃响应与稳速误差分析：%s" % name)
    add("=" * 96)
    add("样本数 %d   时长 %.2f s   重建采样间隔 %.4f s (%.1f Hz)   time 列毫秒倒退 %d 处"
        % (len(t), t[-1] - t[0], (t[-1] - t[0]) / max(len(t) - 1, 1), (len(t) - 1) / max(t[-1] - t[0], 1e-9), n_back))
    add("速度量化台阶 = %.4g tick/s（台阶数是本报告的主要标尺：稳态误差 < 半个台阶即到极限）" % step)
    add("误差带判据 = max(%.0f%% × 目标, %.0f 个台阶) = max(%.0f%%, %.0f tick/s)"
        % (band_frac * 100, band_steps, band_frac * 100, band_steps * step))

    segs = segments(t, tgt)
    add("")
    add("--- 目标阶跃一览（只列向上阶跃与 0→目标 的启动） ---")
    rows_out = []
    for si in range(len(segs)):
        i0, i1, target = segs[si]
        if si == 0:
            prev = 0.0
            v_pre = float(vel[i0])
        else:
            pi0, pi1, prev = segs[si - 1]
            j0 = int(np.searchsorted(t[pi0:pi1], t[pi1 - 1] - 0.2)) + pi0
            v_pre = float(np.nanmean(vel[j0:pi1])) if pi1 - j0 > 0 else float("nan")
        if target <= prev:
            continue
        t0 = t[i0]
        amp = target - v_pre
        band = max(band_frac * target, band_steps * step)
        # 峰值（本段内）
        v_seg = vel[i0:i1]
        k_peak = int(np.nanargmax(v_seg)) if len(v_seg) else 0
        v_peak = float(v_seg[k_peak]) if len(v_seg) else float("nan")
        t_peak = t[i0 + k_peak]
        overshoot = 100.0 * (v_peak - target) / target if target else float("nan")
        t10 = crossing_time(t, vel, i0, i1, v_pre + 0.10 * amp)
        t90 = crossing_time(t, vel, i0, i1, v_pre + 0.90 * amp)
        t63 = crossing_time(t, vel, i0, i1, v_pre + 0.632 * amp)
        iset = settle_index(t, err, i0, band, limit_sec=min(20.0, t[i1 - 1] - t0))
        t_set = (t[iset] - t0) if iset is not None else float("nan")
        # 饱和：从 t0 到进入误差带
        iend = iset if iset is not None else i1
        sat = 100.0 * np.mean(duty[i0:iend + 1] >= 0.999) if iend > i0 else float("nan")
        imax = float(np.nanmax(cur[i0:i1])) if np.any(np.isfinite(cur[i0:i1])) else float("nan")

        # 稳态窗口：本段末尾 tail 秒
        st = t[i1 - 1]
        k0 = int(np.searchsorted(t[i0:i1], st - tail)) + i0
        has_steady = (i1 - k0) >= 5
        if has_steady:
            ve = float(np.nanmean(err[k0:i1]))
            vd = float(np.nanstd(vel[k0:i1], ddof=1))
            dm = float(np.nanmean(duty[k0:i1]))
            dd = float(np.nanstd(duty[k0:i1], ddof=1))
            dr = float(np.nanmax(duty[k0:i1]) - np.nanmin(duty[k0:i1]))
            sat_s = 100.0 * np.mean(duty[k0:i1] >= 0.999)
        else:
            ve = vd = dm = dd = dr = sat_s = float("nan")
        add("  %5.2f s  %5.0f → %-5.0f  幅 %+6.0f | t63.2%%=%.3f  t90%%=%.3f  峰值 %.0f(超调 %+.2f%%)  "
            "到位 %s  饱和 %.0f%%  电流峰 %.2f A"
            % (t0, prev, target, amp,
               (t63 - t0) if t63 else float("nan"), (t90 - t0) if t90 else float("nan"),
               v_peak, overshoot,
               ("%.3f s" % t_set) if np.isfinite(t_set) else "未进带", sat, imax))
        if has_steady:
            add("            稳态(末 %.1f s)：误差 %+.2f tick/s (%+.2f%%) = %.2f 个台阶 | 转速 sd %.2f (%.2f 台阶) | "
                "duty %.3f sd %.4f 极差 %.3f | 饱和 %.1f%%"
                % (tail, ve, 100.0 * ve / target if target else float("nan"), ve / step if step else float("nan"),
                   vd, vd / step if step else float("nan"), dm, dd, dr, sat_s))
        rows_out.append(dict(t0=t0, prev=prev, target=target, v_pre=v_pre, v_peak=v_peak,
                             overshoot=overshoot, t90=(t90 - t0) if t90 else float("nan"),
                             t63=(t63 - t0) if t63 else float("nan"), t_set=t_set, sat=sat,
                             imax=imax, err=ve, vsd=vd, duty=dm, dsd=dd, drange=dr, sat_s=sat_s,
                             i0=i0, i1=i1))

    # --- 汇总 ---
    fin = [r for r in rows_out if np.isfinite(r["err"])]
    if fin:
        add("")
        add("--- 汇总 ---")
        add("  向上阶跃 %d 次；稳态可用 %d 次" % (len(rows_out), len(fin)))
        errs = np.array([r["err"] for r in fin])
        add("  稳态误差：均值 %+.2f tick/s（%+.2f%%），绝对值中位 %.2f tick/s = %.2f 个台阶，最大 %.2f 个台阶"
            % (errs.mean(), 100.0 * errs.mean() / np.mean([r["target"] for r in fin]),
               np.median(np.abs(errs)), np.median(np.abs(errs)) / step, np.max(np.abs(errs)) / step))
        vsds = np.array([r["vsd"] for r in fin])
        add("  转速 sd ：中位 %.2f tick/s = %.2f 个台阶（越小越稳；<1 台阶为优秀）"
            % (np.median(vsds), np.median(vsds) / step))
        dsds = np.array([r["dsd"] for r in fin if np.isfinite(r["dsd"])])
        if len(dsds):
            add("  占空比 sd：中位 %.4f（>0.05 说明增益相对量化台阶过大，会抖）" % np.median(dsds))
        sats = np.array([r["sat"] for r in rows_out if np.isfinite(r["sat"])])
        if len(sats):
            add("  阶跃饱和：中位 %.0f%%（饱和段是「全油门加速」，属正常；但饱和时长直接决定到速时间）" % np.median(sats))
        over = np.array([r["overshoot"] for r in rows_out if np.isfinite(r["overshoot"])])
        if len(over):
            add("  超调    ：中位 %+.2f%%，最大 %+.2f%%" % (np.median(over), over.max()))

    # 前馈回归（无 FF 列时用稳态点 duty vs 转速）
    add("")
    add("--- 稳态前馈回归 d_ss(ω) = kS + kff·ω（用各段的稳态点） ---")
    pts = [(r["target"] + r["err"], r["duty"]) for r in fin if r["target"] > 0]
    xs = np.array([p[0] for p in pts]) if pts else np.array([])
    ys = np.array([p[1] for p in pts]) if pts else np.array([])
    if len(pts) >= 2:
        kff0 = float(np.sum(xs * ys) / np.sum(xs * xs))
        add("  工作点 " + "  ".join("(%.0f, %.3f)" % (x, y) for x, y in pts))
        add("  过原点 kff = %.8f" % kff0)
        if len(pts) >= 3 and (xs.max() - xs.min()) >= 300:
            kff, kS = np.polyfit(xs, ys, 1)
            add("  线性拟合 kS = %+.5f, kff = %.8f" % (kS, kff))
            if kS > 0.005:
                add("  截距为正 → 存在静摩擦/预紧项：本机建议 kS ≈ %.3f（当前 kS=0 时这部分全靠积分补，"
                    "表现为积分长期非零）" % kS)
        elif len(pts) >= 3:
            add("  （工作点转速跨度 <300 tick/s，kS/kff 无法分离，只用过原点 kff）")
        else:
            add("  （工作点 <3 个，kS/kff 无法分离，只用过原点 kff）")
    elif len(pts) == 1:
        add("  只有 1 个稳态点：过原点 kff = %.8f（无法分离 kS）" % (pts[0][1] / pts[0][0]))

    report = "\n".join(L)
    print(report)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        base = os.path.splitext(name)[0]
        rp = os.path.join(out_dir, base + "_step_report.txt")
        with open(rp, "w", encoding="utf-8") as fh:
            fh.write(report + "\n")
        print("[报告] " + rp)

    if do_plot and rows_out:
        try:
            make_plot(d, rows_out, step, band_frac, band_steps, tail, out_dir, name)
        except Exception as exc:  # 绘图失败不影响报告
            print("[绘图失败] %s: %s" % (type(exc).__name__, exc), file=sys.stderr)
    return rows_out


def make_plot(d, rows_out, step, band_frac, band_steps, tail, out_dir, name):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    t, tgt, vel, duty = d["t"], d["target"], d["velocity"], d["duty"]
    cur, vol = d["current"], d["voltage"]

    # 选一个「0 → 目标」的启动阶跃做主图（优先最大的目标）
    starts = [r for r in rows_out if r["prev"] <= 0.5]
    main = max(starts, key=lambda r: r["target"]) if starts else max(rows_out, key=lambda r: r["target"])
    i0, i1 = main["i0"], main["i1"]
    tw = 6.0
    k0 = int(np.searchsorted(t[i0:i1], t[i0] - 0.3)) + i0
    k1 = int(np.searchsorted(t[i0:i1], t[i0] + tw)) + i0
    k1 = min(k1, i1, len(t))
    tt = t[k0:k1] - t[i0]
    band = max(band_frac * main["target"], band_steps * step)

    fig = plt.figure(figsize=(13, 11))
    ax1 = fig.add_subplot(3, 1, 1)
    ax1.plot(tt, vel[k0:k1], "b.-", lw=1.2, ms=3, label="实测 ω")
    ax1.axhline(main["target"], color="r", ls="--", lw=1.2, label="目标 ω* = %.0f" % main["target"])
    ax1.axhspan(main["target"] - band, main["target"] + band, color="orange", alpha=0.15,
                label="误差带 ±%.0f (±%.0f%%)" % (band, max(band_frac * 100, 100 * band_steps * step / main["target"])))
    for dy in (-2 * step, -step, step, 2 * step):
        ax1.axhline(main["target"] + dy, color="0.7", lw=0.6, ls=":")
    ax1.axvline(main["t90"], color="green", ls="-.", lw=1.0)
    ax1.annotate("t90%% = %.3f s" % main["t90"], xy=(main["t90"], main["v_pre"] + 0.35 * (main["target"] - main["v_pre"])),
                 xytext=(main["t90"] + 0.15, main["v_pre"] + 0.30 * (main["target"] - main["v_pre"])),
                 color="green", fontsize=9)
    if np.isfinite(main["t_set"]):
        ax1.axvline(main["t_set"], color="purple", ls="-.", lw=1.0)
        ax1.annotate("进入误差带 %.3f s" % main["t_set"], xy=(main["t_set"], main["target"] * 0.55),
                     xytext=(main["t_set"] + 0.15, main["target"] * 0.5), color="purple", fontsize=9)
    ax1.set_title("① 阶跃 %.0f → %.0f tick/s（超调 %+.2f%%，峰值 %.0f）"
                  % (main["v_pre"], main["target"], main["overshoot"], main["v_peak"]))
    ax1.set_xlabel("相对阶跃时刻的时间 (s)"); ax1.set_ylabel("转速 (tick/s)")
    ax1.grid(alpha=0.3); ax1.legend(loc="lower right", fontsize=9)

    ax2 = fig.add_subplot(3, 1, 2)
    ax2.plot(tt, duty[k0:k1], color="darkorange", lw=1.2, label="duty")
    ax2.axhline(1.0, color="r", lw=0.8, ls="--")
    ax2.set_ylabel("占空比", color="darkorange"); ax2.set_ylim(-0.05, 1.08)
    ax2.grid(alpha=0.3)
    ax2b = ax2.twinx()
    ax2b.plot(tt, cur[k0:k1], color="purple", lw=1.0, alpha=0.8, label="电流 (A)")
    ax2b.set_ylabel("电流 (A)", color="purple")
    ax2.set_title("② 同一时刻：占空比（是否顶到 1.0 = 全油门）与电流 / 母线电压 %.2f V"
                  % float(np.nanmedian(vol[k0:k1])))
    lines = ax2.get_lines()[:2] + ax2b.get_lines()
    ax2.legend(lines, [l.get_label() for l in lines], loc="center right", fontsize=9)
    ax2.set_xlabel("相对阶跃时刻的时间 (s)")

    ax3 = fig.add_subplot(3, 1, 3)
    for idx, r in enumerate(rows_out):
        a, b = r["i0"], r["i1"]
        m0 = int(np.searchsorted(t[a:b], t[a] - 0.2)) + a
        m1 = min(int(np.searchsorted(t[a:b], t[a] + 6.0)) + a, b, len(t))
        ax3.plot(t[m0:m1] - t[a], vel[m0:m1], lw=1.0,
                 label="%.0f→%.0f @%.0fs" % (r["prev"], r["target"], r["t0"]))
    ax3.set_title("③ 全部向上阶跃对齐叠加（看一致性：起步形状是否一致、是否有迟滞）")
    ax3.set_xlabel("相对阶跃时刻的时间 (s)"); ax3.set_ylabel("转速 (tick/s)")
    ax3.grid(alpha=0.3); ax3.legend(fontsize=8, ncol=2, loc="lower right")

    fig.tight_layout()
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        fp = os.path.join(out_dir, os.path.splitext(name)[0] + "_step.png")
        fig.savefig(fp, dpi=150)
        plt.close(fig)
        print("[图] " + fp)


def main():
    ap = argparse.ArgumentParser(description="飞轮阶跃响应与稳速误差分析")
    ap.add_argument("log", help="原始 Dashboard 日志或 *_clean.csv 数据表")
    ap.add_argument("--out", default=None, help="输出目录")
    ap.add_argument("--tail", type=float, default=1.0, help="稳态窗口长度 (s)，默认 1.0")
    ap.add_argument("--band-frac", type=float, default=0.02, help="误差带比例，默认 2%%")
    ap.add_argument("--band-steps", type=float, default=2.0, help="误差带下限（量化台阶数），默认 2")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    analyze(args.log, args.tail, args.band_frac, args.band_steps, args.out, not args.no_plot)


if __name__ == "__main__":
    main()
