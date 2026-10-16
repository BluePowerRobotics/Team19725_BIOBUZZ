#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
飞轮标定数据离线分析 / 参数复算（Python 主用版）

读取 RC 上 `OpModes/Tuning` 五个标定 OpMode 导出的 CSV，自动判别方法、
出图，并用与机器人端**同一套公式**复算控制参数，便于：
  1. 复核机器人遥测里给出的建议值（在线 vs 离线应一致）；
  2. 用更好的拟合（最小二乘 τ、多点回归 Kff）提高精度；
  3. 存档每次实验的曲线图，供赛前对比与评审。

对应《飞轮系统控制理论研究报告》第 6 章：
    6.1 step01  StepResponseTuner     → Kd / τ / kff / kp / ki
    6.2 zn02    ZieglerNicholsTuner   → Ku / Tu / kp / ki
    6.3 relay03 RelayFeedbackTuner    → a / Tu / Ku / kp / ki
    6.4 drop04  DropDetectionTuner    → σ / 量化台阶 / maxDecel / 阈值 / kBoost
    6.5 ff05    FeedforwardTuner      → 多点 (ω, d_ss) 回归 Kff

用法：
    python analyze_tuning.py <csv 或目录> [更多输入...] [选项]

常用选项：
    --out DIR      图片与报告输出目录（默认与 CSV 同目录）
    --no-plot      只打印报告，不出图（无 matplotlib 环境时用）
    --show         出图后弹窗显示（默认只存 PNG）
    --window S     振荡类方法（zn02/relay03）分析的尾部窗口时长，默认 5.0 / 4.0 s
    --ku K         zn02 未锁定 Ku 时手工指定临界增益
    --kp K         drop04 需要 kBoost=ratio*kp 且 CSV 里没有 kBoost 时手工指定 kp
    --alpha A      drop04 求最大减速度前的速度 EMA 系数（默认 0.3，与端上一致）

CSV 格式（由 TuningLogger 生成）：
    # 若干注释行（含本次实验的参数与结论，形如 key=value）
    t,duty,velocity
    0.0010,0.1500,1450.00
    ...  时间单位 s，转速单位 tick/s（DcMotorEx#getVelocity()）

注意：step01 的 CSV 在阶跃处会插入 `# --- step applied here ... ---`，
其后 `t` 从 0 重新开始；本脚本按该标记分段处理。
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import glob
from dataclasses import dataclass, field

try:
    import numpy as np
except ImportError:  # pragma: no cover
    print("需要 numpy：pip install numpy", file=sys.stderr)
    raise

# ---------------------------------------------------------------------------
# CSV 读取
# ---------------------------------------------------------------------------

STEP_MARK = "step applied here"


@dataclass
class TuningData:
    path: str
    meta: list = field(default_factory=list)          # 所有 '#' 注释行（去掉 #）
    events: list = field(default_factory=list)        # [(当时的行号, 注释行)] 保留出现顺序
    header: list = field(default_factory=list)        # 表头列名
    t: "np.ndarray" = None
    duty: "np.ndarray" = None
    vel: "np.ndarray" = None

    @property
    def n(self) -> int:
        return 0 if self.t is None else len(self.t)

    @property
    def method(self) -> str:
        base = os.path.basename(self.path).lower()
        for key in ("step01", "zn02", "relay03", "drop04", "ff05"):
            if base.startswith(key):
                return key
        # 文件名没带前缀时，靠注释行判断
        blob = " ".join(self.meta).lower()
        if "step response" in blob:
            return "step01"
        if "critical-gain" in blob:
            return "zn02"
        if "relay feedback" in blob:
            return "relay03"
        if "drop-detection" in blob:
            return "drop04"
        if "feedforward" in blob:
            return "ff05"
        return "unknown"


def read_tuning_csv(path: str) -> TuningData:
    """读取 TuningLogger 生成的 CSV（兼容无表头、CRLF、BOM）。"""
    ts, duties, vels = [], [], []
    data = TuningData(path=path)
    with open(path, "r", encoding="utf-8-sig", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                text = line.lstrip("#").strip()
                data.meta.append(text)
                data.events.append((len(ts), text))
                continue
            parts = [p for p in line.split(",")]
            if len(parts) < 3:
                continue
            try:
                t, d, v = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                # 表头行
                data.header = [p.strip() for p in parts]
                continue
            ts.append(t)
            duties.append(d)
            vels.append(v)
    data.t = np.asarray(ts, dtype=float)
    data.duty = np.asarray(duties, dtype=float)
    data.vel = np.asarray(vels, dtype=float)
    return data


def meta_get(meta, key, default=None):
    """从注释行里取 `key=value`（value 为数字）。"""
    pat = re.compile(r"\b" + re.escape(key) + r"\s*=\s*(-?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)")
    for line in meta:
        m = pat.search(line)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                return default
    return default


# ---------------------------------------------------------------------------
# 通用数值工具（与机器人端保持同一口径）
# ---------------------------------------------------------------------------

def gcd_step(values) -> float:
    """速度量化台阶：所有取整速度值的最大公约数。"""
    g = 0
    for v in values:
        iv = int(round(abs(float(v))))
        if iv:
            g = math.gcd(g, iv)
    return float(g)


def ema_max_decel(t, v, alpha: float):
    """先 EMA 平滑再差分，取最大减速度（正值）。与端上 DropDetectionTuner 一致。"""
    if len(t) < 2:
        return 0.0
    ema = float(v[0])
    prev_ema = ema
    prev_t = float(t[0])
    max_decel = 0.0
    for i in range(1, len(t)):
        ema += alpha * (float(v[i]) - ema)
        dt = float(t[i]) - prev_t
        dv = ema - prev_ema
        if dt > 1e-4 and dv < 0:
            decel = -dv / dt
            if decel > max_decel:
                max_decel = decel
        prev_ema = ema
        prev_t = float(t[i])
    return max_decel


def osc_period(t, v, min_crossings: int = 4):
    """过零法估计振荡周期；返回 (Tu, amp, mean)，无法判定时 Tu=None。"""
    if len(t) < 10:
        return None, 0.0, 0.0
    mean = float(np.mean(v))
    amp = (float(np.max(v)) - float(np.min(v))) / 2.0
    first = last = None
    crossings = 0
    for i in range(1, len(v)):
        a = float(v[i - 1]) - mean
        b = float(v[i]) - mean
        if (a < 0 <= b) or (a > 0 >= b):
            if first is None:
                first = float(t[i])
            last = float(t[i])
            crossings += 1
    if crossings < min_crossings or first is None or last is None or last <= first:
        return None, amp, mean
    cycles = (crossings - 1) // 2
    if cycles < 1:
        return None, amp, mean
    return (last - first) / cycles, amp, mean


def first_order_fit(t, v, tau_lo=1e-3, tau_hi=20.0, iters=200):
    """最小二乘拟合 v(t)=v0+K*(1-exp(-t/tau))：对 τ 做黄金分割，其余参数线性求解。"""
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    if len(t) < 5:
        return None

    def sse(tau):
        x = 1.0 - np.exp(-t / tau)
        A = np.vstack([np.ones_like(x), x]).T
        try:
            coef, res, *_ = np.linalg.lstsq(A, v, rcond=None)
        except np.linalg.LinAlgError:
            return float("inf"), None
        r = v - A @ coef
        return float(r @ r), coef

    gr = (math.sqrt(5.0) - 1.0) / 2.0
    lo, hi = tau_lo, tau_hi
    c, d = hi - gr * (hi - lo), lo + gr * (hi - lo)
    for _ in range(iters):
        if sse(c)[0] < sse(d)[0]:
            hi = d
        else:
            lo = c
        c, d = hi - gr * (hi - lo), lo + gr * (hi - lo)
    tau = 0.5 * (lo + hi)
    err, coef = sse(tau)
    if coef is None:
        return None
    return dict(tau=tau, v0=float(coef[0]), K=float(coef[1]), sse=err)


# ---------------------------------------------------------------------------
# 各方法的分析
# ---------------------------------------------------------------------------

def analyze_step(d: TuningData, cfg) -> dict:
    """6.1 阶跃响应辨识：Kd / τ / kff / kp / ki。"""
    idx = None
    for row, text in d.events:
        if STEP_MARK in text:
            idx = row
            break
    if idx is None:
        idx = 0  # 没标记就整段当阶跃段，仍需 d0/dStep 元数据

    pre_t, pre_v, pre_d = d.t[:idx], d.vel[:idx], d.duty[:idx]
    post_t, post_v, post_d = d.t[idx:], d.vel[idx:], d.duty[idx:]
    if len(post_t) < 5:
        return dict(ok=False, msg="阶跃段采样不足（<5 点）")

    steady_win = meta_get(d.meta, "steadyWindowSec", 0.5)
    d0 = float(np.median(pre_d)) if len(pre_d) else meta_get(d.meta, "d0", 0.0)
    d_step = float(np.median(post_d)) if len(post_d) else meta_get(d.meta, "dStep", 1.0)

    def tail_mean(t, v, win):
        if len(t) == 0:
            return 0.0
        sel = t >= (t[-1] - win)
        return float(np.mean(v[sel])) if np.any(sel) else float(np.mean(v))

    omega0 = tail_mean(pre_t, pre_v, steady_win)
    omega_ss = tail_mean(post_t, post_v, steady_win)

    delta_d = d_step - d0
    delta_w = omega_ss - omega0
    if abs(delta_d) < 1e-9 or abs(delta_w) < 1e-9:
        return dict(ok=False, msg="Δd 或 Δω 过小：dStep 与 d0 太接近，或飞轮没真正转起来",
                    d0=d0, d_step=d_step, omega0=omega0, omega_ss=omega_ss)

    kd = delta_w / delta_d

    # τ：① 63.2% 判据（与端上一致） ② 最小二乘（离线更准）
    tau_632 = None
    target = omega0 + 0.632 * delta_w
    hit = np.nonzero(post_v >= target)[0]
    if len(hit):
        tau_632 = float(post_t[hit[0]])
    fit = first_order_fit(post_t, post_v)

    t_recover = meta_get(d.meta, "tRecover", 0.5)
    ratio = meta_get(d.meta, "ratio", 4.0)
    tau_c = t_recover / ratio

    def suggest(tau):
        if tau is None or kd <= 0:
            return None
        return dict(tau=tau, kff=1.0 / kd, kp=tau / (kd * tau_c), ki=1.0 / (kd * tau_c))

    return dict(ok=True, d0=d0, d_step=d_step, omega0=omega0, omega_ss=omega_ss,
                kd=kd, tau_632=tau_632, fit=fit, tau_c=tau_c,
                sug_632=suggest(tau_632), sug_ls=suggest(fit["tau"] if fit else None))


def analyze_zn(d: TuningData, cfg) -> dict:
    """6.2 ZN 临界比例度法：尾部等幅振荡 → Ku / Tu → kp / ki。"""
    win = cfg.window if cfg.window else 5.0
    sel = d.t >= (d.t[-1] - win)
    t, v = d.t[sel], d.vel[sel]
    tu, amp, mean = osc_period(t, v)
    ku = meta_get(d.meta, "Ku")
    if ku is None:
        ku = cfg.ku
    locked_tu = meta_get(d.meta, "Tu")

    out = dict(ok=True, amp=amp, mean=mean, tu=tu, tu_locked=locked_tu, ku=ku, window=win)
    if ku and tu:
        out["kp"] = 0.45 * ku
        out["ki"] = 0.54 * ku / tu
    if ku is None:
        out["msg"] = "CSV 里没有锁定的 Ku——请在 OpMode 里按 A 锁定后重跑，或用 --ku 指定"
    return out


def analyze_relay(d: TuningData, cfg) -> dict:
    """6.3 继电反馈：极限环 → a / Tu / Ku → kp / ki。"""
    measure = meta_get(d.meta, "measureSec")
    win = cfg.window if cfg.window else (measure if measure else 4.0)
    sel = d.t >= (d.t[-1] - win)
    t, v, duty = d.t[sel], d.vel[sel], d.duty[sel]
    tu, amp, mean = osc_period(t, v)
    d_high = float(np.max(duty)) if len(duty) else 0.0
    d_low = float(np.min(duty)) if len(duty) else 0.0
    delta_d = d_high - d_low
    ku = 4.0 * delta_d / (math.pi * amp) if amp > 1e-9 else None
    out = dict(ok=True, amp=amp, mean=mean, tu=tu, d_high=d_high, d_low=d_low,
               delta_d=delta_d, ku=ku, window=win)
    if ku and tu:
        out["kp"] = 0.45 * ku
        out["ki"] = 0.54 * ku / tu
    return out


def analyze_drop(d: TuningData, cfg) -> dict:
    """6.4 掉速检测参数：σ / 量化台阶 / maxDecel → 阈值与 kBoost。"""
    measure = meta_get(d.meta, "measureSec", 5.0)
    sel = d.t >= (d.t[-1] - measure)
    t, v = d.t[sel], d.vel[sel]
    if len(v) < 10:
        return dict(ok=False, msg="测量段采样不足（<10 点）")
    mean = float(np.mean(v))
    sd = float(np.std(v, ddof=1))
    step = gcd_step(v)
    max_decel = ema_max_decel(t, v, cfg.alpha)

    k_sigma = meta_get(d.meta, "kSigma", 5.0)
    k_rate = meta_get(d.meta, "kRate", 2.5)
    k_ratio = meta_get(d.meta, "kBoostRatio", 2.0)
    drop_th = k_sigma * sd
    drop_rate = k_rate * max_decel
    k_boost_onboard = meta_get(d.meta, "kBoost")
    kp = meta_get(d.meta, "kp", cfg.kp)
    k_boost = k_boost_onboard if k_boost_onboard is not None else (
        k_ratio * kp if kp is not None else None)

    return dict(ok=True, mean=mean, sd=sd, step=step, max_decel=max_decel, alpha=cfg.alpha,
                k_sigma=k_sigma, k_rate=k_rate, k_ratio=k_ratio,
                drop_threshold=drop_th, drop_rate_threshold=drop_rate,
                sigma_steps=(sd / step if step else None),
                threshold_steps=(drop_th / step if step else None),
                k_boost=k_boost,
                onboard=dict(mean=meta_get(d.meta, "meanVel"), sd=meta_get(d.meta, "sigma"),
                             step=meta_get(d.meta, "step"), max_decel=meta_get(d.meta, "maxDecel"),
                             drop_threshold=meta_get(d.meta, "dropThreshold"),
                             drop_rate_threshold=meta_get(d.meta, "dropRate"),
                             k_boost=k_boost_onboard))


def analyze_ff(d: TuningData, cfg) -> dict:
    """6.5 前馈：从 `# point omega=..., dss=...` 汇总 (ω, d_ss) 做回归。"""
    pts = []
    for text in d.meta:
        if not text.startswith("point"):
            continue
        w = meta_get([text], "omega")
        ds = meta_get([text], "dss")
        if w is not None and ds is not None:
            pts.append((w, ds))
    out = dict(ok=bool(pts), points=pts)
    if not pts:
        out["msg"] = "CSV 里没有 `# point ...` 记录——请在 OpMode 里按 A 逐点记录后重跑"
        return out
    w = np.array([p[0] for p in pts], dtype=float)
    ds = np.array([p[1] for p in pts], dtype=float)
    out["kff_origin"] = float(np.sum(w * ds) / np.sum(w * w))
    if len(pts) >= 2:
        slope, intercept = np.polyfit(w, ds, 1)
        out["slope"] = float(slope)
        out["intercept"] = float(intercept)
    return out


ANALYZERS = {"step01": analyze_step, "zn02": analyze_zn, "relay03": analyze_relay,
             "drop04": analyze_drop, "ff05": analyze_ff}


# ---------------------------------------------------------------------------
# 绘图
# ---------------------------------------------------------------------------

def make_plots(d: TuningData, res: dict, out_png: str, show: bool) -> None:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    m = d.method
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=False)

    if m == "step01" and res.get("ok"):
        idx = next((row for row, text in d.events if STEP_MARK in text), 0)
        pre_t, post_t = d.t[:idx], d.t[idx:]
        # 把两段拼到同一条时间轴上：阶跃点 = 0
        t_shift = pre_t - pre_t[-1] if len(pre_t) else pre_t
        ax = axes[0]
        ax.plot(t_shift, d.vel[:idx], "b-", lw=1.2, label="实测 ω (阶跃前)")
        ax.plot(post_t, d.vel[idx:], "b-", lw=1.3, label="实测 ω (阶跃后)")
        ax.axhline(res["omega0"], color="gray", ls=":", lw=1)
        ax.axhline(res["omega_ss"], color="gray", ls=":", lw=1)
        if res.get("fit"):
            tt = np.linspace(0, float(post_t[-1]), 400)
            vv = res["fit"]["v0"] + res["fit"]["K"] * (1.0 - np.exp(-tt / res["fit"]["tau"]))
            ax.plot(tt, vv, "r--", lw=1.4,
                    label=f"最小二乘拟合 τ={res['fit']['tau']:.3f}s")
        if res.get("tau_632"):
            ax.axvline(res["tau_632"], color="green", ls="--", lw=1,
                       label=f"t63.2% = {res['tau_632']:.3f}s")
        ax.axvline(0, color="k", lw=0.8)
        ax.set_xlabel("时间 (s)  （0 = 阶跃时刻）")
        ax.set_ylabel("转速 (tick/s)")
        ax.set_title(f"6.1 阶跃响应辨识  Kd={res['kd']:.4f} (tick/s)/duty")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="lower right")

        ax = axes[1]
        ax.plot(t_shift, d.duty[:idx], color="#d97a26", lw=1.2)
        ax.plot(post_t, d.duty[idx:], color="#d97a26", lw=1.3)
        ax.axvline(0, color="k", lw=0.8)
        ax.set_ylim(max(-0.02, float(d.duty.min()) - 0.05), min(1.12, float(d.duty.max()) + 0.05))
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("占空比")
        ax.set_title("占空比（开环阶跃）")
        ax.grid(alpha=0.3)
    else:
        ax = axes[0]
        ax.plot(d.t, d.vel, "b-", lw=1.2, label="转速 ω")
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("转速 (tick/s)")
        ax.set_title(f"{m} 实测转速")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)

        ax = axes[1]
        ax.plot(d.t, d.duty, color="#d97a26", lw=1.2, label="占空比")
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("占空比")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)

        if m in ("zn02", "relay03") and res.get("ok"):
            for a in axes:
                if res.get("mean") is not None:
                    a.axhline(res["mean"], color="gray", ls=":", lw=1)
                    a.axhline(res["mean"] + res["amp"], color="r", ls=":", lw=1)
                    a.axhline(res["mean"] - res["amp"], color="r", ls=":", lw=1)
            axes[0].set_title(
                f"{m}  幅值 a={res['amp']:.1f}  Tu=" +
                ("n/a" if not res.get("tu") else f"{res['tu']:.4f}s"))
            axes[0].legend([f"窗口 {res.get('window')}s 内的等幅振荡"], fontsize=8)

        if m == "ff05" and res.get("ok") and res.get("points"):
            w = np.array([p[0] for p in res["points"]])
            ds = np.array([p[1] for p in res["points"]])
            axes[1].clear()
            axes[1].plot(w, ds, "ko", ms=5, label="测量点")
            ww = np.linspace(0, float(w.max()) * 1.05, 100)
            axes[1].plot(ww, res["kff_origin"] * ww, "r-", lw=1.4,
                         label=f"过原点 Kff={res['kff_origin']:.3e}")
            if "slope" in res:
                axes[1].plot(ww, res["slope"] * ww + res["intercept"], "b--", lw=1.2,
                             label=f"线性拟合 {res['slope']:.3e}·ω+{res['intercept']:.4f}")
            axes[1].set_xlabel("转速 ω (tick/s)")
            axes[1].set_ylabel("稳态占空比 d_ss")
            axes[1].set_title("6.5 前馈增益标定")
            axes[1].grid(alpha=0.3)
            axes[1].legend(fontsize=8)

    fig.suptitle(os.path.basename(d.path), fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------

def build_report(d: TuningData, res: dict) -> str:
    def span_and_rate(t):
        """按分段累加得到真实总时长（step01 的 t 在阶跃处会重新从 0 开始）。"""
        if t is None or len(t) < 2:
            return 0.0, 0.0
        span, start = 0.0, float(t[0])
        for i in range(1, len(t)):
            if t[i] < t[i - 1]:
                span += float(t[i - 1]) - start
                start = float(t[i])
        span += float(t[-1]) - start
        return span, (len(t) / span if span > 0 else 0.0)

    L = []
    add = L.append
    span, rate = span_and_rate(d.t)
    add("=" * 78)
    add(f"文件：{d.path}")
    add(f"方法：{d.method}    采样点：{d.n}    时长：{span:.2f} s    行率：{rate:.1f} Hz")
    add("=" * 78)

    if d.method == "step01":
        if not res.get("ok"):
            add(f"[无效] {res.get('msg')}")
            return "\n".join(L)
        add(f"d0 = {res['d0']:.4f}    dStep = {res['d_step']:.4f}    Δd = {res['d_step'] - res['d0']:.4f}")
        add(f"ω0 = {res['omega0']:.1f}    ω(∞) = {res['omega_ss']:.1f}    Δω = {res['omega_ss'] - res['omega0']:.1f} tick/s")
        add(f"Kd = Δω/Δd = {res['kd']:.4f} (tick/s)/占空比")
        add(f"τ (t63.2%) = " + ("n/a" if res["tau_632"] is None else f"{res['tau_632']:.4f} s"))
        if res.get("fit"):
            add(f"τ (最小二乘) = {res['fit']['tau']:.4f} s   (拟合残差 SSE={res['fit']['sse']:.3g})")
        add(f"τc = tRecover/ratio = {res['tau_c']:.4f} s")
        for tag, sug in (("按 t63.2%", res.get("sug_632")), ("按最小二乘", res.get("sug_ls"))):
            if sug:
                add(f"  建议参数（{tag}）：kff = {sug['kff']:.8f}   kp = {sug['kp']:.8f}   ki = {sug['ki']:.8f}")
        add("  → 填入 PrototypeShooter：kff / kp / ki（单位：占空比/(tick/s)、占空比/(tick/s·s)）")
        add("  提示：kff 也可用方法 6.5 的实测稳态占空比修正；Kd 偏小时说明 Δd 太小，加大阶跃幅度重跑。")

    elif d.method == "zn02":
        add(f"分析窗口：尾部 {res['window']:.2f} s")
        add(f"等幅振荡估计：幅值 a = {res['amp']:.1f} tick/s    中心 = {res['mean']:.1f}    Tu = "
            + ("n/a（过零不足或幅值太小）" if not res.get("tu") else f"{res['tu']:.4f} s"))
        if res.get("ku") is None:
            add(f"[无效] {res.get('msg')}")
            return "\n".join(L)
        src = "lock (A 键锁定)" if meta_get(d.meta, "Ku") is not None else "命令行 --ku"
        add(f"Ku = {res['ku']:.8f}   （来源：{src}）")
        if res.get("tu_locked"):
            add(f"锁定时的 Tu = {res['tu_locked']:.4f} s")
        if "kp" in res:
            add(f"ZN 表：建议 kp = 0.45·Ku = {res['kp']:.8f}    ki = 0.54·Ku/Tu = {res['ki']:.8f}")
        add("  提示：需确认振荡近似等幅；幅值失控请立即停转。")

    elif d.method == "relay03":
        add(f"分析窗口：尾部 {res['window']:.2f} s")
        add(f"继电器输出：dHigh = {res['d_high']:.4f}   dLow = {res['d_low']:.4f}   Δd = {res['delta_d']:.4f}")
        add(f"极限环：幅值 a = {res['amp']:.1f} tick/s    Tu = "
            + ("n/a" if not res.get("tu") else f"{res['tu']:.4f} s"))
        if res.get("ku") is None:
            add("[无效] 幅值或周期无法判定：请加大 dHigh/dLow 差距后重跑")
            return "\n".join(L)
        add(f"Ku = 4·Δd/(π·a) = {res['ku']:.8f}")
        if "kp" in res:
            add(f"ZN 表：建议 kp = 0.45·Ku = {res['kp']:.8f}    ki = 0.54·Ku/Tu = {res['ki']:.8f}")

    elif d.method == "drop04":
        if not res.get("ok"):
            add(f"[无效] {res.get('msg')}")
            return "\n".join(L)
        add(f"稳态均值 = {res['mean']:.1f} tick/s")
        add(f"σ (样本标准差 ÷N-1) = {res['sd']:.2f} tick/s")
        add(f"速度量化台阶 = {res['step']:.1f} tick/s   →  σ = "
            + ("n/a" if res["sigma_steps"] is None else f"{res['sigma_steps']:.2f} 个台阶"))
        add(f"最大减速度 maxDecel(EMA α={res.get('alpha', 0.3):g}) = {res['max_decel']:.1f} tick/s²")
        add(f"Δω_th = k_σ·σ = {res['k_sigma']:g}×{res['sd']:.2f} = {res['drop_threshold']:.2f} tick/s"
            + ("" if res["threshold_steps"] is None else f"  (≈{res['threshold_steps']:.2f} 个台阶)"))
        add(f"k_th   = k_rate·maxDecel = {res['k_rate']:g}×{res['max_decel']:.1f} = {res['drop_rate_threshold']:.2f} tick/s²")
        if res.get("k_boost") is not None:
            add(f"kBoost = k_boostRatio·kp = {res['k_ratio']:g}×kp = {res['k_boost']:.8f}")
        else:
            add("kBoost：CSV 无 kBoost 且未给 --kp，请在命令行加 --kp <当前 kp>")
        ob = res.get("onboard") or {}
        if ob.get("sigma") is not None:
            add("--- 端上（OpMode）给出的值，用于一致性核对 ---")
            add(f"  sigma={ob['sigma']} step={ob['step']} maxDecel={ob['max_decel']} "
                f"dropThreshold={ob['drop_threshold']} dropRate={ob['drop_rate_threshold']} kBoost={ob['k_boost']}")
        if res["threshold_steps"] is not None and res["threshold_steps"] < 3:
            add("  ⚠ 阈值不足 3 个量化台阶：阈值会被量化噪声主导，请提高目标转速或保守取值。")

    elif d.method == "ff05":
        if not res.get("ok"):
            add(f"[无效] {res.get('msg')}")
            return "\n".join(L)
        add("测量点 (ω, d_ss)：")
        for w, ds in res["points"]:
            add(f"  ω = {w:8.1f} tick/s    d_ss = {ds:.4f}    Kff = {ds / w if w else float('nan'):.8e}")
        add(f"过原点斜率（推荐填入 kff）= {res['kff_origin']:.8f} 占空比/(tick/s)")
        if "slope" in res:
            add(f"线性拟合斜率 = {res['slope']:.8f}    截距 = {res['intercept']:.4f}")
            add("  截距明显非 0 → 存在静摩擦；当前控制律无 kS 项，可考虑加常数偏置或改用含 kS 的控制器。")

    else:
        add("未识别方法：仅输出基本统计")
        if d.n:
            add(f"转速 min/mean/max = {d.vel.min():.1f} / {d.vel.mean():.1f} / {d.vel.max():.1f}")
            add(f"占空比 min/mean/max = {d.duty.min():.4f} / {d.duty.mean():.4f} / {d.duty.max():.4f}")

    return "\n".join(L)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def collect_inputs(paths):
    files = []
    for p in paths:
        if os.path.isdir(p):
            files.extend(sorted(glob.glob(os.path.join(p, "*.csv"))))
        elif any(ch in p for ch in "*?"):
            files.extend(sorted(glob.glob(p)))
        else:
            files.append(p)
    out = []
    for f in files:
        if f not in out:
            out.append(f)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="飞轮标定 CSV 离线分析 / 参数复算")
    ap.add_argument("inputs", nargs="+", help="CSV 文件、目录或通配符")
    ap.add_argument("--out", default=None, help="输出目录（默认与 CSV 同目录）")
    ap.add_argument("--no-plot", action="store_true", help="不出图")
    ap.add_argument("--show", action="store_true", help="出图后弹窗显示")
    ap.add_argument("--window", type=float, default=None, help="振荡类方法分析窗口 (s)")
    ap.add_argument("--ku", type=float, default=None, help="zn02 手工指定 Ku")
    ap.add_argument("--kp", type=float, default=None, help="drop04 计算 kBoost 所需的 kp")
    ap.add_argument("--alpha", type=float, default=0.3, help="drop04 速度 EMA 系数")
    cfg = ap.parse_args(argv)

    files = collect_inputs(cfg.inputs)
    if not files:
        print("没有找到输入 CSV", file=sys.stderr)
        return 2

    reports = []
    for path in files:
        if not os.path.isfile(path):
            print(f"[跳过] 不存在：{path}", file=sys.stderr)
            continue
        d = read_tuning_csv(path)
        fn = ANALYZERS.get(d.method)
        res = fn(d, cfg) if fn else {"ok": False}
        text = build_report(d, res)
        print(text)
        print()
        reports.append(text)

        out_dir = cfg.out or os.path.dirname(os.path.abspath(path))
        os.makedirs(out_dir, exist_ok=True)
        stem = os.path.splitext(os.path.basename(path))[0]
        with open(os.path.join(out_dir, stem + "_report.txt"), "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
        if not cfg.no_plot:
            try:
                png = os.path.join(out_dir, stem + "_analysis.png")
                make_plots(d, res, png, cfg.show)
                print(f"[图] {png}")
            except ImportError as e:
                print(f"[跳过绘图] {e}（可加 --no-plot）", file=sys.stderr)
            except Exception as e:  # 绘图失败不应影响报告
                print(f"[绘图失败] {type(e).__name__}: {e}", file=sys.stderr)

    if cfg.out and len(reports) > 1:
        with open(os.path.join(cfg.out, "all_reports.txt"), "w", encoding="utf-8") as fh:
            fh.write("\n\n".join(reports))
    return 0


if __name__ == "__main__":
    sys.exit(main())
