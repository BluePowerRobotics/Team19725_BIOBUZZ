#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通用飞轮 CSV 出图器：**任意列 → 一图一表**。

只要 CSV 里有数就能画：本脚本不假设固定的 11 列格式，而是
①先把列名归一化（Dashboard 原始导出的大写列、`TuningLogger` 的自定义表头、
`FlywheelRun` / 电压扫描 / 多代对比的宽表全吃），
②按行序重建均匀时间轴（Dashboard 的 `time` 列会倒退，不能直接画），
③**每个数值列单独出一张 PNG**（一图一文件），
④出一份 `*_summary.txt` 汇总表（整段统计 + 分段统计），
⑤有分段列（`segment_id`）时自动在图上着色分段并给逐段表，
有分组列（`gen`）时把同一张图按组画多条线（多代对比）。

用法
----
    # 最常用：任意 CSV → 每列一张图 + 汇总表
    python tools/tuning/plot_csv.py tuning/FlywheelRun_xxx.csv --out ./out

    # 多代对比：同一张图里按 gen 分线（每列仍是一张图）
    python tools/tuning/plot_csv.py cmp.csv --group gen --out ./out

    # 电压扫描：按 segment_id 分段着色 + 逐段统计表
    python tools/tuning/plot_csv.py sweep.csv --segment segment_id --out ./out

    # 只画关心的几列 / 把几列叠在一张图里
    python tools/tuning/plot_csv.py log.csv --cols duty,ff,i,p
    python tools/tuning/plot_csv.py log.csv --overlay duty,ff,i,p --tag 分量

约定（与其余脚本一致）
----------------------
- 注释行以 `#` 开头（`TuningLogger` 会写 `# flywheel tuning data`），直接跳过；
- 时间单位秒、严格单调：不单调就按行序重建 `dt = 中位间隔`（并在汇总表里记一笔）；
- 速度单位 tick/s、占空比 0~1；
- 存图口径 `dpi=150, bbox_inches='tight'`，单图 `figsize=(9, 4)`。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# ---------------------------------------------------------------- 列名归一化

# 同义列名 → 规范名。规范名只有下面这些是「约定」的，
# 其余未知列一律保留 snake_case 原名照样出图（这就是「有数据就能画」）。
_ALIASES = {
    # 时间
    "t": "t", "time": "t", "times": "t", "timestamp": "t", "sec": "t",
    "secs": "t", "seconds": "t", "time_s": "t", "t_sec": "t", "runtime": "t",
    "time_ms": "t_ms", "t_ms": "t_ms", "ms": "t_ms", "millis": "t_ms",
    "elapsed": "t",
    # 目标 / 实测
    "target": "target", "target_velocity": "target", "velocity_target": "target",
    "targetvel": "target", "target_vel": "target", "setpoint": "target",
    "set_point": "target", "sp": "target", "ref": "target", "reference": "target",
    "velocity": "velocity", "vel": "velocity", "speed": "velocity",
    "actual": "velocity", "actual_velocity": "velocity",
    "current_velocity": "velocity", "motor_velocity": "velocity",
    # 输出
    "duty": "duty", "power": "duty", "out": "duty", "output": "duty",
    "cmd": "duty", "motor_power": "duty", "pwm": "duty",
    # 电学量
    "voltage": "voltage", "vbus": "voltage", "v_bus": "voltage",
    "busv": "voltage", "bus_voltage": "voltage", "battery": "voltage",
    "batt": "voltage", "vbat": "voltage", "v_battery": "voltage",
    "current": "current", "amps": "current", "amp": "current",
    "curr": "current", "motor_current": "current", "i_amps": "current",
    # 控制分量
    "error": "error", "err": "error", "e": "error",
    "ff": "ff", "ff_term": "ff", "feedforward": "ff",
    "p": "p", "p_term": "p", "pt": "p",
    "i": "i", "i_term": "i", "it": "i", "integral": "i",
    "d": "d", "d_term": "d", "dt": "d", "derivative": "d",
    "ks_term": "kS", "ksterm": "kS", "ks": "kS",
    "vscale": "vscale", "v_scale": "vscale",
    # 状态位
    "indrop": "inDrop", "in_drop": "inDrop", "drop": "inDrop",
    "iactive": "iActive", "integral_active": "iActive", "i_active": "iActive",
    "settled": "settled", "is_settled": "settled",
    # 分段 / 分组（第 4 代新增测试程序用）
    "segment": "segment_id", "segmentid": "segment_id",
    "segment_id": "segment_id", "seg": "segment_id",
    "gen": "gen", "generation": "gen", "genid": "gen", "gen_id": "gen",
    "vbustarget": "vbus_target", "vbus_target": "vbus_target",
    "vbustgt": "vbus_target",
    "vbusmeas": "vbus_meas", "vbus_meas": "vbus_meas",
    "vbusfilt": "vbus_meas", "vbus": "vbus_meas",
}

# 单位后缀（画图时写进 y 轴标签）
_UNITS = {
    "t": "s", "velocity": "tick/s", "target": "tick/s", "error": "tick/s",
    "duty": "", "ff": "", "p": "", "i": "", "d": "", "kS": "",
    "voltage": "V", "vbus_target": "V", "vbus_meas": "V",
    "current": "A", "vscale": "×", "inDrop": "bool", "iActive": "bool",
    "settled": "bool", "gen": "", "segment_id": "",
}

_CN = {
    "t": "时间 (s)", "velocity": "转速 (tick/s)", "target": "目标转速 (tick/s)",
    "error": "误差 e = ω* − ω (tick/s)", "duty": "占空比 (0~1)",
    "ff": "前馈项 (占空比)", "p": "P 项 (占空比)", "i": "I 项 (占空比)",
    "d": "D 项 (占空比)", "kS": "kS 项 (占空比)",
    "voltage": "母线电压 (V)", "vbus_target": "目标母线电压 (V)",
    "vbus_meas": "母线电压 (V)", "current": "电流 (A)", "vscale": "电压缩放 s (×)",
    "inDrop": "掉速态 (0/1)", "iActive": "积分活跃 (0/1)", "settled": "稳态标志 (0/1)",
}


def _snake(name: str) -> str:
    """列名 → snake_case（去掉单位括号、空格、连接符）。"""
    s = str(name).strip()
    for ch in " ()[]-./\\":
        s = s.replace(ch, "_")
    s = s.replace("%", "pct")
    while "__" in s:
        s = s.replace("__", "_")
    return s.strip("_").lower()


def normalize_columns(df: pd.DataFrame) -> pd.DataFrame:
    """把列名映射到规范名；未知列保留 snake_case 原名（照样会出图）。"""
    mapping = {}
    for col in df.columns:
        key = _snake(col)
        mapping[col] = _ALIASES.get(key, key)
    # 去重：同一规范名只保留第一次出现（后面加 _2/_3，避免静默丢列）
    seen: dict[str, int] = {}
    out = {}
    for col in df.columns:
        name = mapping[col]
        if name in seen:
            seen[name] += 1
            name = f"{name}_{seen[name]}"
        else:
            seen[name] = 0
        out[col] = name
    return df.rename(columns=out)


def load_csv(path: Path) -> pd.DataFrame:
    """读 CSV：跳过 `#` 注释行、列名归一化、丢掉全空列。"""
    df = pd.read_csv(path, comment="#", skipinitialspace=True)
    df = df.dropna(axis=1, how="all")
    if df.empty:
        raise SystemExit(f"[错误] {path} 没有数据行")
    return normalize_columns(df)


# ---------------------------------------------------------------- 时间轴

def ensure_time(df: pd.DataFrame, notes: list[str]) -> pd.DataFrame:
    """保证有严格单调的 `t`（秒）。没有时间列就按行序重建。"""
    if "t_ms" in df.columns and "t" not in df.columns:
        df = df.rename(columns={"t_ms": "t"})
        df["t"] = df["t"] / 1000.0
        notes.append("时间列按毫秒换算成秒")
    if "t" not in df.columns:
        df = df.copy()
        df.insert(0, "t", np.arange(len(df), dtype=float) * 0.01)
        notes.append("没有时间列 → 按行序以 10 ms 重建")
        return df

    t = pd.to_numeric(df["t"], errors="coerce").to_numpy(dtype=float)
    n_bad = int(np.count_nonzero(~np.isfinite(t)))
    if n_bad:
        notes.append(f"时间列有 {n_bad} 个非数值 → 按行序重建")
        t = np.arange(len(df), dtype=float) * 0.01
    else:
        d = np.diff(t)
        n_back = int(np.count_nonzero(d <= 0))
        if n_back > 0:
            notes.append(
                f"时间列有 {n_back} 处非单调（倒退/重复）→ 按行序用中位间隔重建"
            )
            pos = d[d > 0]
            dt = float(np.median(pos)) if pos.size else 0.01
            t = np.arange(len(df), dtype=float) * dt
        elif t.size > 1 and float(np.median(d)) > 1.0:
            notes.append("时间列中位间隔 > 1 → 视为毫秒，换算成秒")
            t = t / 1000.0
    df = df.copy()
    df["t"] = t
    return df


def infer_velocity_step(values: np.ndarray) -> float:
    """速度量化台阶：对「速度的全部不同整数取值」求 gcd。

    先试整数 tick/s（`DcMotorEx#getVelocity()` 就是整数）；含小数时退回
    「最小正差值」，并在调用方注明不可靠。
    """
    v = np.unique(values[np.isfinite(values)])
    if v.size < 2:
        return 0.0
    is_int = bool(np.all(np.abs(v - np.round(v)) < 1e-6))
    if not is_int:
        d = np.diff(np.sort(v))
        d = d[d > 1e-9]
        return float(d.min()) if d.size else 0.0
    g = 0
    base = int(round(v[0]))
    for val in v[1:]:
        g = math.gcd(g, int(round(val)) - base)
        if g == 1:
            break
    return float(abs(g))


# ---------------------------------------------------------------- 出图

def _label(col: str) -> str:
    return _CN.get(col, col)


def _segments(df: pd.DataFrame, seg_col: str) -> list[tuple[object, pd.DataFrame]]:
    out = []
    for key, sub in df.groupby(seg_col, sort=True):
        if len(sub) >= 2:
            out.append((key, sub))
    return out


def plot_one(ax, df: pd.DataFrame, col: str, *, xcol: str, group: str | None,
             seg_col: str | None) -> tuple[float, float]:
    """在给定 ax 上画一列；返回 (mean, sd)。"""
    y = pd.to_numeric(df[col], errors="coerce")

    if seg_col is not None:
        segs = _segments(df, seg_col)
        for i, (key, sub) in enumerate(segs):
            if i % 2 == 0:
                ax.axvspan(float(sub[xcol].iloc[0]), float(sub[xcol].iloc[-1]),
                           color="0.5", alpha=0.06, lw=0)

    if group is not None:
        for key, sub in df.groupby(group, sort=True):
            yy = pd.to_numeric(sub[col], errors="coerce")
            ax.plot(sub[xcol], yy, lw=1.1, label=f"{group}={key}")
        ax.legend(loc="best", fontsize=8)
    else:
        ax.plot(df[xcol], y, lw=1.1, color="#1f77b4")

    finite = y.to_numpy(dtype=float)
    finite = finite[np.isfinite(finite)]
    mean = float(np.mean(finite)) if finite.size else float("nan")
    sd = float(np.std(finite)) if finite.size else float("nan")
    if finite.size:
        ax.axhline(mean, color="0.35", ls=":", lw=0.9)
        ax.annotate(f"mean={mean:.4g}  sd={sd:.4g}",
                    xy=(0.01, 0.02), xycoords="axes fraction",
                    fontsize=8, color="0.25")
    ax.set_ylabel(_label(col))
    ax.grid(True, ls=":", alpha=0.5)
    return mean, sd


def make_figures(df: pd.DataFrame, *, stem: str, outdir: Path, cols: list[str] | None,
                 xcol: str, group: str | None, seg_col: str | None,
                 plt) -> list[str]:
    numeric = [c for c in df.columns
               if c != xcol and pd.api.types.is_numeric_dtype(df[c])]
    if cols:
        numeric = [c for c in numeric if c in cols]
        if not numeric:
            raise SystemExit(f"[错误] --cols 里没有可用列；可选：{sorted(set(df.columns))}")

    made = []
    for col in numeric:
        fig, ax = plt.subplots(figsize=(9, 4))
        plot_one(ax, df, col, xcol=xcol, group=group, seg_col=seg_col)
        # 转速图上叠目标（看得清跟踪偏差）
        if col == "velocity" and "target" in df.columns and group is None:
            ax.plot(df[xcol], pd.to_numeric(df["target"], errors="coerce"),
                    ls="--", lw=1.0, color="#d62728", alpha=0.85, label="target")
            ax.legend(loc="best", fontsize=8)
        if seg_col is not None:
            ax.set_title(f"{stem}: {col}（按 {seg_col} 分段着色）")
        else:
            ax.set_title(f"{stem}: {col}")
        ax.set_xlabel(_label(xcol) if xcol == "t" else xcol)
        fp = outdir / f"{stem}_{col}.png"
        fig.savefig(fp, dpi=150, bbox_inches="tight")
        plt.close(fig)
        made.append(fp.name)
    return made


def make_overlay(df: pd.DataFrame, *, stem: str, outdir: Path, cols: list[str],
                 xcol: str, tag: str, group: str | None, plt) -> str:
    """把多列叠在一张图里（分量对比用），仍是一图一文件。"""
    fig, ax = plt.subplots(figsize=(9, 4))
    for col in cols:
        y = pd.to_numeric(df[col], errors="coerce")
        if group is not None:
            for key, sub in df.groupby(group, sort=True):
                ax.plot(sub[xcol], pd.to_numeric(sub[col], errors="coerce"),
                        lw=1.0, label=f"{col} @{group}={key}")
        else:
            ax.plot(df[xcol], y, lw=1.1, label=col)
    ax.set_xlabel(_label(xcol) if xcol == "t" else xcol)
    ax.set_ylabel(" / ".join(_label(c) for c in cols) if len(cols) <= 2
                  else f"{len(cols)} 列叠加（见图例）")
    ax.set_title(f"{stem}: {tag}（{', '.join(cols)}）")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(loc="best", fontsize=8)
    fp = outdir / f"{stem}_{tag}.png"
    fig.savefig(fp, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return fp.name


# ---------------------------------------------------------------- 汇总表

_STAT_COLS = ["target", "velocity", "error", "duty", "voltage", "vbus_meas",
              "current", "vscale", "settled", "inDrop", "iActive"]


def write_summary(df: pd.DataFrame, *, path: Path, outdir: Path, stem: str,
                  notes: list[str]) -> Path:
    t = df["t"].to_numpy(dtype=float)
    dt = np.diff(t)
    lines = [
        f"# {stem} 汇总表",
        f"源文件   : {path}",
        f"行数     : {len(df)}",
        f"时长     : {t[-1] - t[0]:.3f} s" if len(t) > 1 else "时长     : -",
        f"dt 中位  : {float(np.median(dt)) * 1000:.3f} ms（{1.0 / float(np.median(dt)):.1f} Hz）"
        if dt.size else "dt 中位  : -",
        "",
    ]
    if notes:
        lines.append("## 读入修正")
        lines += [f"- {n}" for n in notes]
        lines.append("")

    vel = None
    for cand in ("velocity", "target"):
        if cand in df.columns:
            vel = pd.to_numeric(df[cand], errors="coerce").to_numpy(dtype=float)
            break
    step = infer_velocity_step(vel) if vel is not None else 0.0
    lines.append(f"## 量化台阶（速度列 gcd 推断）：{step:.3f} tick/s" if step > 0
                 else "## 量化台阶：无法推断（数据不足或非整数 tick/s）")
    lines.append("")

    lines.append("## 整段统计（按一图一表的顺序）")
    header = f"{'列':<14}{'n':>7}{'mean':>12}{'sd':>12}{'min':>12}{'max':>12}"
    lines.append(header)
    lines.append("-" * len(header))
    for col in df.columns:
        if col == "t" or not pd.api.types.is_numeric_dtype(df[col]):
            continue
        v = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
        v = v[np.isfinite(v)]
        if v.size == 0:
            lines.append(f"{col:<14}{0:>7}{'-':>12}{'-':>12}{'-':>12}{'-':>12}")
            continue
        lines.append(f"{col:<14}{v.size:>7}{np.mean(v):>12.4f}{np.std(v):>12.4f}"
                     f"{np.min(v):>12.4f}{np.max(v):>12.4f}")
    lines.append("")

    seg_col = "segment_id" if "segment_id" in df.columns else None
    if seg_col is not None:
        lines.append(f"## 逐段统计（{seg_col}）")
        cols = [c for c in _STAT_COLS if c in df.columns]
        head = f"{'seg':>5}{'n':>6}{'t0':>8}{'t1':>8}" + "".join(
            f"{c + '_mean':>16}" for c in cols)
        lines.append(head)
        lines.append("-" * len(head))
        for key, sub in df.groupby(seg_col, sort=True):
            row = f"{key:>5}{len(sub):>6}{sub['t'].iloc[0]:>8.2f}{sub['t'].iloc[-1]:>8.2f}"
            for c in cols:
                row += f"{pd.to_numeric(sub[c], errors='coerce').mean():>16.4f}"
            lines.append(row)
        lines.append("")

    if "gen" in df.columns:
        lines.append("## 逐代统计（gen）")
        head = f"{'gen':>5}{'n':>6}" + "".join(f"{c + '_mean':>16}" for c in _STAT_COLS
                                               if c in df.columns)
        lines.append(head)
        lines.append("-" * len(head))
        for key, sub in df.groupby("gen", sort=True):
            row = f"{key:>5}{len(sub):>6}"
            for c in _STAT_COLS:
                if c in df.columns:
                    row += f"{pd.to_numeric(sub[c], errors='coerce').mean():>16.4f}"
            lines.append(row)
        lines.append("")

    fp = outdir / f"{stem}_summary.txt"
    fp.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return fp


# ---------------------------------------------------------------- main

def _resolve(names: list[str]) -> list[str]:
    """把用户写的列名（大小写/空格随意，可写别名）解析成规范列名。"""
    out = []
    for n in names:
        key = _snake(n)
        out.append(_ALIASES.get(key, key))
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="通用飞轮 CSV 出图器：任意列 → 一图一表（每列一张 PNG + 汇总表）")
    ap.add_argument("files", nargs="+", help="一个或多个 CSV")
    ap.add_argument("--out", default=None, help="输出目录（默认与输入同级）")
    ap.add_argument("--x", default="t", help="横轴列名（默认 t）")
    ap.add_argument("--cols", default=None, help="只画这些列，逗号分隔")
    ap.add_argument("--overlay", default=None,
                    help="把这些列叠到同一张图，逗号分隔（另存一张）")
    ap.add_argument("--tag", default="overlay", help="--overlay 那张图的名字后缀")
    ap.add_argument("--group", default=None,
                    help="分组列（如 gen）：同一张图按组画多条线")
    ap.add_argument("--segment", default=None,
                    help="分段列（如 segment_id）：图上着色 + 逐段统计表")
    ap.add_argument("--no-plot", action="store_true", help="只出汇总表，不出图")
    args = ap.parse_args(argv)

    cols = _resolve([c.strip() for c in args.cols.split(",")]) if args.cols else None
    overlay = _resolve([c.strip() for c in args.overlay.split(",")]) if args.overlay else []

    plt = None
    if not args.no_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # noqa: F811

        # 中文字体（与其余脚本同口径），否则标题/轴标签全是方框
        plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False

    rc = 0
    for f in args.files:
        path = Path(f)
        if not path.is_file():
            print(f"[跳过] 不存在：{path}")
            rc = 1
            continue
        stem = path.stem
        outdir = Path(args.out) if args.out else path.parent
        outdir.mkdir(parents=True, exist_ok=True)

        notes: list[str] = []
        df = ensure_time(load_csv(path), notes)
        if args.x != "t" and args.x not in df.columns:
            raise SystemExit(f"[错误] 横轴列 {args.x} 不存在；可选：{sorted(df.columns)}")
        if args.x not in df.columns:
            args.x = "t"
        group = args.group if args.group in df.columns else None
        if args.group and group is None:
            print(f"[提示] {path.name} 没有 {args.group} 列 → 不分组")
        seg_col = args.segment if args.segment in df.columns else None
        if args.segment and seg_col is None and args.segment in ("segment_id",):
            print(f"[提示] {path.name} 没有分段列 → 不分段")

        print(f"\n=== {path.name} : {len(df)} 行, {df['t'].iloc[-1] - df['t'].iloc[0]:.2f} s")
        for n in notes:
            print(f"    · {n}")

        if not args.no_plot:
            made = make_figures(df, stem=stem, outdir=outdir, cols=cols,
                                xcol=args.x, group=group, seg_col=seg_col, plt=plt)
            print(f"    图 {len(made)} 张 → {outdir}")
            for m in made:
                print(f"      - {m}")
            if overlay:
                missing = [c for c in overlay if c not in df.columns]
                if missing:
                    print(f"    [提示] --overlay 忽略不存在的列：{missing}")
                ok = [c for c in overlay if c in df.columns]
                if ok:
                    name = make_overlay(df, stem=stem, outdir=outdir, cols=ok,
                                        xcol=args.x, tag=args.tag, group=group, plt=plt)
                    print(f"      - {name}")

        sp = write_summary(df, path=path, outdir=outdir, stem=stem, notes=notes)
        print(f"    汇总表 → {sp}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
