#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FTC Dashboard 遥测日志（Driver Station 导出 CSV）分析：出图 + 参数体检。

输入是 RC 上 Driver Station 导出的 telemetry CSV，第一列是 `time`（HH:MM:SS.mmm），
其余列是各 OpMode `telemetry.addData` 的条目，最后一列常是带引号的 `logs`。

本脚本按《交接说明》的既有教训处理数据：
  * 用 csv 模块解析（`logs` 列含多行引号文本，按行 split 会把行数数错）；
  * **time 列的毫秒字段是坏的**（同一秒内会从 ~900 跳回 ~100），按它排序会伪造锯齿暴跌；
    因此**按文件行序重建均匀时间轴** dt = span/(n-1)，并统计毫秒倒退处数；
  * 报出速度**量化台阶**（实测速度的最大公约数），任何稳态精度/阈值结论都要先看它。

输出：
  <stem>_overview.png  目标/实测、误差、占空比与 FF/P/I 分量、电流
  <stem>_zoom.png      稳态段放大：转速台阶 vs 占空比跳变（看 P 增益是否过大）
  <stem>_scatter.png   占空比-误差 散点（反演 kP）与 转速-占空比（看前馈/饱和）
并打印：分段统计、增益反演（kff/kp/ki）、饱和占比、掉速补偿触发情况、诊断结论。

用法：python analyze_dashboard_log.py <log.csv> [--out DIR] [--no-plot]
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

try:
    import numpy as np
except ImportError:
    print("需要 numpy：pip install numpy", file=sys.stderr)
    raise

CAND = {
    "ks": ["Shooter kS term", "kS term"],
    "ff": ["Shooter FF term", "Shooter FF", "FF term"],
    "i": ["Shooter I term", "Shooter I", "I term"],
    "p": ["Shooter P term", "Shooter P", "P term"],
    "boostflag": ["Shooter boost", "boost"],
    "boostterm": ["Shooter boost term", "boost term"],
    "current": ["Shooter current (A)", "current (A)"],
    "voltage": ["Shooter voltage (V)", "voltage (V)"],
    "duty": ["Shooter power (duty)", "Shooter duty", "duty", "power"],
    "error": ["Shooter error (tick/s)", "error (tick/s)"],
    "target": ["Shooter target (tick/s)", "target (tick/s)"],
    "velocity": ["Shooter velocity (tick/s)", "velocity (tick/s)"],
}


def pick(header, names):
    for n in names:
        if n in header:
            return n
    return None


def parse_time(s):
    try:
        h, m, rest = s.split(":")
        sec = float(rest)
        return int(h) * 3600 + int(m) * 60 + sec
    except Exception:
        return None


def load(path):
    with open(path, newline="", encoding="utf-8-sig", errors="replace") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit("空文件")
    header = list(rows[0].keys())
    cols = {k: pick(header, v) for k, v in CAND.items()}
    if cols["velocity"] is None or cols["duty"] is None:
        print("文件列名：", header, file=sys.stderr)
        raise SystemExit("未识别到 velocity/duty 列，请检查是否为本项目的遥测日志")

    def num(key, row):
        c = cols[key]
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

    labels = [parse_time(r.get("time", "")) for r in rows]
    data = {k: np.array([num(k, r) for r in rows], float) for k in CAND}
    return rows, header, cols, labels, data


def rebuild_time(labels):
    """按行序重建均匀时间轴；返回 (t, span, dt, n_backwards, n_bad)."""
    good = [x for x in labels if x is not None]
    n_bad = len(labels) - len(good)
    if len(good) < 2:
        n = len(labels)
        return np.arange(n) * 0.02, (n - 1) * 0.02, 0.02, 0, n_bad
    backwards = 0
    for a, b in zip(good, good[1:]):
        # 毫秒字段损坏：同一秒内出现倒退（b 比 a 小但差距远小于 1 s）
        if b < a and (a - b) < 0.9:
            backwards += 1
    span = good[-1] - good[0]
    n = len(labels)
    dt = span / (n - 1) if n > 1 and span > 0 else 0.02
    return np.arange(n) * dt, span, dt, backwards, n_bad


def gcd_step(values):
    g = 0
    for v in values:
        iv = int(round(abs(float(v))))
        if iv:
            g = math.gcd(g, iv)
    return float(g)


def segments(target, t):
    """按目标转速变化切段，返回 [(target, i0, i1)]。"""
    out = []
    start = 0
    for i in range(1, len(target)):
        if target[i] != target[i - 1]:
            out.append((float(target[i - 1]), start, i))
            start = i
    out.append((float(target[-1]), start, len(target)))
    return out


def zc_period(t, v):
    """过零法估主周期（用于极限环）。"""
    if len(v) < 20:
        return float("nan")
    m = np.nanmean(v)
    idx = []
    for i in range(1, len(v)):
        a, b = v[i - 1] - m, v[i] - m
        if (a < 0 <= b) or (a > 0 >= b):
            idx.append(t[i])
    if len(idx) < 5:
        return float("nan")
    return 2.0 * (idx[-1] - idx[0]) / (len(idx) - 1)


def _stats(t, vel, err, duty, cur, pt, i0, i1):
    v, e, du, c = vel[i0:i1], err[i0:i1], duty[i0:i1], cur[i0:i1]
    p = pt[i0:i1] if pt is not None else np.array([float("nan")])
    sat = 100.0 * np.mean(du >= 0.999) if np.any(np.isfinite(du)) else 0.0
    return dict(dur=t[i1 - 1] - t[i0], vmean=float(np.nanmean(v)),
                vsd=float(np.nanstd(v, ddof=1)), emean=float(np.nanmean(e)),
                esd=float(np.nanstd(e, ddof=1)), dmean=float(np.nanmean(du)),
                dsd=float(np.nanstd(du, ddof=1)),
                drange=float(np.nanmax(du) - np.nanmin(du)) if len(du) else 0.0,
                sat=sat, cmean=float(np.nanmean(c)),
                pmax=float(np.nanmax(np.abs(p))) if np.any(np.isfinite(p)) else 0.0)


def stats_of(t, vel, err, duty, cur, pt, target, i0, i1, tail_sec=1.0):
    """整段 + 末段稳态窗口的统计。"""
    dt = (t[1] - t[0]) if len(t) > 1 else 0.01
    j0 = max(i0, i1 - max(1, int(round(tail_sec / dt))))
    s = _stats(t, vel, err, duty, cur, pt, i0, i1)
    s["target"] = target
    s["i0"] = i0
    s["i1"] = i1
    s["tail"] = _stats(t, vel, err, duty, cur, pt, j0, i1)
    s["tail"]["i0"] = j0
    return s


def fmt_levels(x, top=4):
    """列出占空比/速度的主要取值档位及其占比，用于说明「只在几档之间跳」。"""
    vals, cnts = np.unique(np.round(np.asarray(x, float)[np.isfinite(x)], 3), return_counts=True)
    if len(vals) == 0:
        return "n/a"
    order = np.argsort(-cnts)[:top]
    tot = cnts.sum()
    return "  ".join(f"{vals[i]:.3f}×{100.0 * cnts[i] / tot:.0f}%" for i in order)


def analyze(path, out_dir, do_plot, tail=1.0):
    rows, header, cols, labels, d = load(path)
    t, span, dt, back, bad = rebuild_time(labels)
    n = len(t)
    stem = os.path.splitext(os.path.basename(path))[0].replace(" ", "_")

    L = []
    add = L.append
    add("=" * 84)
    add(f"文件：{os.path.basename(path)}")
    add(f"样本数：{n}    时长：{span:.2f} s    重建采样间隔：{dt * 1000:.1f} ms ({1 / dt:.1f} Hz)"
        f"    时间列毫秒倒退：{back} 处" + (f"    无法解析的时间：{bad} 行" if bad else ""))
    add("=" * 84)

    vel, tgt, err, duty = d["velocity"], d["target"], d["error"], d["duty"]
    ff, it, pt, boost = d["ff"], d["i"], d["p"], d["boostflag"]
    cur = d["current"]

    step = gcd_step(vel)
    # 剔除收尾异常帧：例如 stop() 之后速度读数归零、而目标仍是原值（会把 sd 和幅值撑大）
    last = n - 1
    while last > 0 and abs(tgt[last]) > 0 and abs(err[last]) > 0.5 * abs(tgt[last]):
        last -= 1
    cut = last + 1
    if cut < n:
        t, vel, tgt, err, duty = t[:cut], vel[:cut], tgt[:cut], err[:cut], duty[:cut]
        ff, it, pt, cur, boost = ff[:cut], it[:cut], pt[:cut], cur[:cut], boost[:cut]
        add(f"（已剔除末尾 {n - cut} 帧收尾异常帧：目标仍为原值但转速读数已丢失/归零）")
        n = cut
        step = gcd_step(vel)
    uniq = np.unique(np.round(vel).astype(int))
    add(f"速度量化台阶（gcd）= {step:.0f} tick/s    不同速度取值数 = {len(uniq)}"
        f"    范围 = [{np.nanmin(vel):.0f}, {np.nanmax(vel):.0f}]")

    # ---- 分段统计 ----
    add("\n--- 分段统计（按目标转速变化切段） ---")
    add(f"{'目标':>7}{'起(s)':>8}{'时长':>7}{'v均值':>8}{'v_sd':>7}{'误差均值':>9}{'误差sd':>7}"
        f"{'duty均值':>9}{'duty_sd':>8}{'饱和%':>7}{'电流均值':>9}")
    segs = []
    for tv, i0, i1 in segments(tgt, t):
        if i1 - i0 < 10:
            continue
        dur = t[i1 - 1] - t[i0]
        if dur < 0.4:
            continue
        segs.append(stats_of(t, vel, err, duty, cur, pt, tv, i0, i1, tail))
        s = segs[-1]
        add(f"{s['target']:>7.0f}{t[i0]:>8.2f}{s['dur']:>7.2f}{s['vmean']:>8.0f}{s['vsd']:>7.1f}"
            f"{s['emean']:>9.1f}{s['esd']:>7.1f}{s['dmean']:>9.3f}{s['dsd']:>8.3f}"
            f"{s['sat']:>7.1f}{s['cmean']:>9.2f}")

    add(f"\n--- 稳态窗口统计（每段末尾 {tail:.1f} s，排除启动/换速暂态）---")
    add(f"{'目标':>7}{'v均值':>8}{'v_sd':>7}{'台阶数':>7}{'误差均值':>9}{'误差%':>8}"
        f"{'duty均值':>9}{'duty_sd':>8}{'duty极差':>9}{'饱和%':>7}{'P项幅度':>8}")
    for s in segs:
        st = s["tail"]
        errp = 100.0 * abs(st["emean"]) / s["target"] if s["target"] else 0.0
        add(f"{s['target']:>7.0f}{st['vmean']:>8.0f}{st['vsd']:>7.1f}"
            f"{(st['vsd'] / step if step else float('nan')):>7.2f}"
            f"{st['emean']:>9.1f}{errp:>8.2f}{st['dmean']:>9.3f}{st['dsd']:>8.3f}"
            f"{st['drange']:>9.3f}{st['sat']:>7.1f}{st['pmax']:>8.3f}")

    # ---- 每段各分量均值 + 段内 ki 拟合 ----
    add("\n--- 每段各分量均值与 ki 段内拟合 ---")
    add("（控制器在 |e|≥iZone 或目标切换时会清零积分，所以 ki 只能在段内拟合）")
    ki_list = []
    for s in segs:
        i0, i1 = s["i0"], s["i1"]
        f_m = float(np.nanmean(ff[i0:i1]))
        i_m = float(np.nanmean(it[i0:i1]))
        p_m = float(np.nanmean(pt[i0:i1]))
        e = np.nan_to_num(err[i0:i1])
        acc = np.cumsum(e) * dt
        mm = np.isfinite(acc) & np.isfinite(it[i0:i1])
        ki_fit = float("nan")
        if np.count_nonzero(mm) > 10:
            A = np.vstack([acc[mm], np.ones(np.count_nonzero(mm))]).T
            coef, *_ = np.linalg.lstsq(A, it[i0:i1][mm], rcond=None)
            if coef[0] > 0:
                ki_fit = float(coef[0])
                ki_list.append(ki_fit)
        add(f"  目标 {s['target']:>5.0f}: FF均值={f_m:.4f}  I均值={i_m:.4f}  P均值={p_m:.4f}"
            f"  ki段内={'n/a' if not np.isfinite(ki_fit) else f'{ki_fit:.8f}'}")
    if ki_list:
        add(f"  → ki 段内拟合中位数 = {np.median(ki_list):.8f}")

    # ---- 用各稳态工作点回归前馈增益（等价于一次功率-转速标定）----
    pts = [(s["tail"]["vmean"], s["tail"]["dmean"]) for s in segs if s["target"] > 0 and s["tail"]["dmean"] > 1e-3]
    if len(pts) >= 3:
        w = np.array([p[0] for p in pts], float)
        dd = np.array([p[1] for p in pts], float)
        kff_fit = float(np.sum(w * dd) / np.sum(w * w))
        sl, ic = np.polyfit(w, dd, 1)
        m_ff = (tgt != 0) & np.isfinite(ff) & (ff != 0)
        kff_used = float(np.median(ff[m_ff] / tgt[m_ff])) if np.any(m_ff) else float("nan")
        add("\n--- 前馈增益回归（各稳态工作点：转速 vs 稳态占空比）---")
        add(f"  过原点 kff = Σ(ω·d)/Σ(ω²) = {kff_fit:.8f}"
            + (f"    当前使用 {kff_used:.8f}，偏差 {100.0 * (kff_used - kff_fit) / kff_fit:+.1f}%"
               if np.isfinite(kff_used) else ""))
        add(f"  线性拟合 d = {sl:.8e}·ω {ic:+.4f}"
            + ("    （截距 > 0 → 存在静摩擦，当前控制律无 kS 项）" if ic > 0.01 else ""))

    # ---- 掉速补偿触发分析 ----
    if np.any(np.isfinite(boost)):
        on = boost > 0.5
        onset = np.nonzero(on[1:] & ~on[:-1])[0] + 1
        if len(onset):
            dv = np.gradient(vel, dt)
            dv_at = dv[onset]
            e_at = err[onset]
            nb = ~on
            add("\n--- 掉速补偿触发分析 ---")
            add(f"  触发 {len(onset)} 次，占 {100.0 * np.mean(on):.1f}% 时间（稳态下频繁触发=阈值偏低）")
            add(f"  触发帧误差：中位 {np.median(e_at):.0f}，最小 {np.min(e_at):.0f} tick/s"
                f"   → dropThreshold 若走误差判据，应不小于 {np.max(np.abs(e_at)) * 3:.0f} 量级")
            add(f"  触发帧减速度：中位 {np.median(dv_at):.0f}，最快 {np.min(dv_at):.0f} tick/s²")
            if np.any(nb):
                add(f"  未触发帧的最大减速度：{np.min(dv[nb]):.0f} tick/s²"
                    f"   → dropRateThreshold 必须显著大于 {abs(np.min(dv[nb])):.0f}，否则量化抖动就会误触发")

    # ---- 增益反演 ----
    add("\n--- 增益反演（由遥测分量反推，与 Dashboard 里填的值对照）---")

    m = (tgt != 0) & np.isfinite(ff) & (ff != 0)
    if np.any(m):
        kff = float(np.median(ff[m] / tgt[m]))
        add(f"kff  ≈ median(FF term / target) = {kff:.8f} 占空比/(tick/s)"
            f"    （FF 在 target={tgt[m][0]:.0f} 时为 {ff[m][0]:.4f}）")
    else:
        kff = float("nan")
        add("kff  ：未检测到非零 FF term")

    m = np.isfinite(pt) & np.isfinite(err) & (np.abs(err) > 1e-9) & (np.abs(pt) > 1e-9)
    if np.any(m):
        ratios = pt[m] / err[m]
        kp = float(np.median(ratios))
        add(f"kp   ≈ median(P term / error) = {kp:.8f} 占空比/(tick/s)"
            f"    比值范围 [{ratios.min():.6f}, {ratios.max():.6f}]")
        add(f"      → 误差为 1 个量化台阶({step:.0f} tick/s) 时，P 项跳变 {abs(kp) * step:.4f} 占空比")
    else:
        kp = float("nan")
        add("kp   ：未检测到非零 P term / error")

    # ki：I term = ki * ∫e dt → 对累计积分做线性回归
    ok = np.isfinite(err) & np.isfinite(it)
    if np.count_nonzero(ok) > 20:
        e = np.nan_to_num(err)
        acc = np.cumsum(e) * dt
        A = np.vstack([acc[ok], np.ones(np.count_nonzero(ok))]).T
        coef, *_ = np.linalg.lstsq(A, it[ok], rcond=None)
        add(f"ki   ≈ 回归(I term ~ ∫e dt) = {coef[0]:.8f} 占空比/(tick/s·s)"
            f"    （截距 {coef[1]:.4f}；I term 均值 {np.nanmean(it[ok]):.4f}）")

    # ---- 掉速补偿 ----
    if np.any(np.isfinite(boost)):
        frac = 100.0 * np.mean(boost > 0.5)
        add(f"\n掉速补偿 boost：触发时间占比 = {frac:.1f}%"
            + ("（全程未触发）" if frac < 0.1 else ""))
        if cols["boostterm"]:
            bt = d["boostterm"]
            add(f"                boost term 非零占比 = {100.0 * np.mean(np.abs(bt) > 1e-9):.1f}%"
                f"   最大值 {np.nanmax(np.abs(bt)):.4f}")

    # ---- 占空比与饱和 ----
    if np.any(np.isfinite(duty)):
        add(f"\n占空比：均值 {np.nanmean(duty):.3f}   最小 {np.nanmin(duty):.3f}   最大 {np.nanmax(duty):.3f}"
            f"   饱和(≥0.999) 占比 {100.0 * np.mean(duty >= 0.999):.1f}%"
            f"   贴底(≤0.001) 占比 {100.0 * np.mean(duty <= 0.001):.1f}%")
        if np.any(np.isfinite(ff)) and np.any(np.isfinite(pt)):
            comp = np.nan_to_num(d["ks"]) + ff + pt + np.nan_to_num(it)
            over = comp > 1.0
            add(f"        kS+FF+P+I 之和超过 1.0（被限幅截掉）的时间占比 = {100.0 * np.mean(over):.1f}%")

    # ---- 极限环 ----
    if segs:
        last = segs[-1]
        st = last["tail"]
        j0, i1 = st["i0"], last["i1"]
        add(f"\n最后一段稳态窗口（目标 {last['target']:.0f}，取末尾 {tail:.1f} s，排除启动暂态）分析：")
        if last["target"] != 0:
            add(f"  稳态误差均值 {st['emean']:.2f} tick/s（{100.0 * abs(st['emean']) / last['target']:.2f}%），"
                f"sd {st['esd']:.2f} tick/s（= {st['esd'] / step if step else float('nan'):.2f} 个量化台阶）")
        add(f"  转速 sd {st['vsd']:.2f} tick/s = {st['vsd'] / step if step else float('nan'):.2f} 个台阶")
        add(f"  占空比 sd {st['dsd']:.4f}；占空比极差 {st['drange']:.3f}；|P 项| 最大 {st['pmax']:.3f}")
        add(f"  占空比档位占比：{fmt_levels(duty[j0:i1])}")
        add(f"  转速档位占比：  {fmt_levels(vel[j0:i1], 5)}")
        per = zc_period(t[j0:i1], vel[j0:i1])
        if np.isfinite(per):
            add(f"  速度过零周期 ≈ {per * 1000:.0f} ms  → 若这是不衰减的等幅振荡，说明 kP 已在临界附近")
        amp = (np.nanmax(vel[j0:i1]) - np.nanmin(vel[j0:i1])) / 2
        add(f"  速度极限环幅值 ≈ {amp:.1f} tick/s = {amp / step if step else float('nan'):.2f} 个台阶")

    # ---- 诊断 ----
    add("\n--- 诊断 ---")
    diag = []
    if np.isfinite(kp) and step > 0:
        swing = abs(kp) * step
        if swing >= 0.20:
            diag.append(f"[严重] kp 相对量化台阶过大：误差只要跳 1 个台阶({step:.0f} tick/s)，"
                        f"P 项就跳 {swing:.3f} 占空比。稳态必然在限幅之间来回撞，"
                        f"表现为转速 2000↔2020 反复、占空比 0.5↔1.0 横跳。")
        elif swing >= 0.05:
            diag.append(f"[警告] kp×台阶 = {swing:.3f} 占空比，稳态会有可见抖动。")
        else:
            diag.append(f"[正常] kp×台阶 = {swing:.3f} 占空比，量化噪声不会主导输出。")
    if segs and segs[-1]["target"] != 0:
        st = segs[-1]["tail"]
        err_pct = 100.0 * abs(st["emean"]) / segs[-1]["target"]
        if err_pct > 2:
            diag.append(f"[警告] 稳态误差 {err_pct:.1f}% 偏大（{st['emean']:.1f} tick/s）。")
        else:
            diag.append(f"[正常] 稳态误差 {err_pct:.2f}%（{st['emean']:.1f} tick/s），在 ±2% 带内。")
    if np.any(np.isfinite(boost)) and 100.0 * np.mean(boost > 0.5) < 0.1:
        diag.append("[提示] 全程未触发掉速补偿：本次没有掉速扰动（正常），"
                    "但也没有验证过 boost 是否有效——需要投球实测。")
    if np.any(np.isfinite(duty)) and 100.0 * np.mean(duty >= 0.999) > 5:
        diag.append(f"[警告] 占空比饱和占比 {100.0 * np.mean(duty >= 0.999):.1f}%：前馈+反馈已把输出顶到上限，"
                    f"此时 Kp 再大也只会加深饱和，抗饱和与积分上限要一起看。")
    for x in diag:
        add("  " + x)

    text = "\n".join(L)
    print(text)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, stem + "_report.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")

    if do_plot:
        try:
            make_plots(stem, out_dir, t, tgt, vel, err, duty, ff, it, pt, cur, boost, segs, step)
        except Exception as e:
            print(f"[绘图失败] {type(e).__name__}: {e}", file=sys.stderr)
    return 0


def make_plots(stem, out_dir, t, tgt, vel, err, duty, ff, it, pt, cur, boost, segs, step):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    # ---- 总览 ----
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    ax[0].plot(t, tgt, "r--", lw=1.2, label="目标 ω*")
    ax[0].plot(t, vel, "b-", lw=1.0, label="实测 ω")
    ax[0].set_ylabel("转速 (tick/s)")
    ax[0].set_title(f"{stem}  —  目标 vs 实测")
    ax[0].grid(alpha=0.3)
    ax[0].legend(fontsize=8)

    ax[1].plot(t, err, color="#7f7f7f", lw=1.0)
    ax[1].axhline(0, color="k", lw=0.8)
    if step:
        ax[1].axhline(step, color="r", ls=":", lw=1)
        ax[1].axhline(-step, color="r", ls=":", lw=1)
    ax[1].set_ylabel("误差 (tick/s)")
    ax[1].set_title(f"误差（红虚线 = ±1 个量化台阶 = ±{step:.0f} tick/s）")
    ax[1].grid(alpha=0.3)

    ax[2].plot(t, duty, color="#d97a26", lw=1.2, label="duty（限幅后）")
    ax[2].plot(t, ff, color="#1f77b4", lw=0.9, label="FF")
    ax[2].plot(t, it, color="#2ca02c", lw=0.9, label="I")
    ax[2].plot(t, pt, color="#9467bd", lw=0.9, label="P")
    ax[2].plot(t, ff + it + pt, "k:", lw=0.8, label="FF+P+I（限幅前）")
    ax[2].axhline(1.0, color="r", lw=1.0)
    ax[2].axhline(0.0, color="r", lw=1.0)
    ax[2].set_ylabel("占空比")
    ax[2].set_title("占空比与各分量（黑点线超过红线上限=被限幅）")
    ax[2].grid(alpha=0.3)
    ax[2].legend(fontsize=7, ncol=3)

    ax[3].plot(t, cur, color="#8c564b", lw=1.0)
    ax[3].set_ylabel("电流 (A)")
    ax[3].set_xlabel("时间 (s)")
    ax[3].set_title("电机电流")
    ax[3].grid(alpha=0.3)
    if np.any(np.isfinite(boost)):
        on = boost > 0.5
        for i in range(1, len(on)):
            if on[i] and not on[i - 1]:
                for a in ax:
                    a.axvline(t[i], color="g", ls="--", lw=1.0, alpha=0.7)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, stem + "_overview.png"), dpi=150)
    plt.close(fig)

    # ---- 稳态放大 ----
    if segs:
        s = segs[-1]
        i0, i1 = s["i0"], s["i1"]
        j0 = max(i0, i1 - int(2.0 / max(t[1] - t[0], 1e-6)))
        tt = t[j0:i1] - t[j0]
        fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        ax[0].plot(tt, vel[j0:i1], "b.-", ms=3, lw=1.0, label="实测 ω")
        ax[0].axhline(s["target"], color="r", ls="--", lw=1.0, label="目标")
        if step:
            for k in range(-3, 4):
                ax[0].axhline(s["target"] + k * step, color="gray", ls=":", lw=0.6)
        ax[0].set_ylabel("转速 (tick/s)")
        ax[0].set_title(f"稳态放大（最后 2 s）：转速台阶 {step:.0f} tick/s")
        ax[0].grid(alpha=0.3)
        ax[0].legend(fontsize=8)
        ax[1].plot(tt, duty[j0:i1], color="#d97a26", lw=1.2, label="duty")
        ax[1].plot(tt, pt[j0:i1], color="#9467bd", lw=1.0, label="P term")
        ax[1].axhline(1.0, color="r", lw=1.0)
        ax[1].set_ylabel("占空比")
        ax[1].set_xlabel("时间 (s，段内相对)")
        ax[1].set_title("同样的时刻：转速只跳一格，占空比却在大范围跳")
        ax[1].grid(alpha=0.3)
        ax[1].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, stem + "_zoom.png"), dpi=150)
        plt.close(fig)

    # ---- 散点 ----
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
    m = np.isfinite(pt) & np.isfinite(err)
    ax[0].plot(err[m], pt[m], ".", ms=3, alpha=0.5)
    ax[0].set_xlabel("误差 e (tick/s)")
    ax[0].set_ylabel("P term (占空比)")
    ax[0].set_title("P 项 vs 误差（斜率 = kp，台阶状=量化）")
    ax[0].grid(alpha=0.3)
    m2 = np.isfinite(duty) & np.isfinite(vel)
    ax[1].plot(duty[m2], vel[m2], ".", ms=3, alpha=0.5)
    ax[1].set_xlabel("占空比")
    ax[1].set_ylabel("转速 (tick/s)")
    ax[1].set_title("转速 vs 占空比（右侧下垂=饱和）")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, stem + "_scatter.png"), dpi=150)
    plt.close(fig)
    print(f"[图] {os.path.join(out_dir, stem)}_overview.png / _zoom.png / _scatter.png")


def main(argv=None):
    ap = argparse.ArgumentParser(description="FTC Dashboard 遥测日志分析")
    ap.add_argument("log", help="Driver Station 导出的 CSV")
    ap.add_argument("--out", default=None)
    ap.add_argument("--tail", type=float, default=1.0, help="稳态窗口长度 (s)，默认 1.0")
    ap.add_argument("--no-plot", action="store_true")
    cfg = ap.parse_args(argv)
    out = cfg.out or os.path.dirname(os.path.abspath(cfg.log))
    return analyze(cfg.log, out, not cfg.no_plot, cfg.tail)


if __name__ == "__main__":
    sys.exit(main())
