#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 FTC Dashboard 导出的原始遥测 CSV，处理成「可直接画图的数据表」。

处理内容（与《交接说明》的既有教训一致）：
  1. **按文件行序重建均匀时间轴**：原始 `time` 列（HH:MM:SS.mmm）的毫秒字段是坏的，
     同一秒内会倒退；按它排序会伪造锯齿暴跌。这里丢弃毫秒、用行序重建
     `dt = (末行秒数 − 首行秒数)/(n−1)`，得到严格单调的 `t`。
  2. 只保留画图/分析需要的列并重命名，数值全部转成浮点。
  3. `error` 统一重新计算为 `target − velocity`（不是日志里那一列，避免与速度不同帧）。
  4. 速度量化台阶（gcd）写进元数据。
  5. 剔除末尾"目标未变但速度读数已归零"的收尾异常帧。

输出：`<源文件名>_clean.csv`，格式：以 `#` 开头的元数据行 + 表头 + 数据行。
      两种读法都对：
        Python : pandas.read_csv(f, comment='#')
        MATLAB : readtable(f,'CommentStyle','#')   或见 plot_speed.m

用法：
    python export_clean_table.py <原始日志.csv 或 目录> [...] [--out DIR]
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import analyze_dashboard_log as A
except ImportError:
    print("需要与本脚本同目录的 analyze_dashboard_log.py", file=sys.stderr)
    raise

COLS = ["t", "target", "velocity", "error", "duty", "ff", "i", "p", "boost", "boost_term", "current"]


def clean_name(path):
    stem = os.path.splitext(os.path.basename(path))[0]
    return re.sub(r"[^0-9A-Za-z_\-]+", "_", stem).strip("_")


def export(path, out_dir):
    rows, header, cols, labels, d = A.load(path)
    t, span, dt, back, bad = A.rebuild_time(labels)
    vel, tgt, duty = d["velocity"], d["target"], d["duty"]
    ff, it, pt = d["ff"], d["i"], d["p"]
    boost, bt, cur = d["boostflag"], d["boostterm"], d["current"]
    n = len(t)

    # 剔除末尾收尾异常帧
    last = n - 1
    while last > 0 and abs(tgt[last]) > 0 and abs(tgt[last] - vel[last]) > 0.5 * abs(tgt[last]):
        last -= 1
    cut = last + 1
    if cut < n:
        t, vel, tgt, duty = t[:cut], vel[:cut], tgt[:cut], duty[:cut]
        ff, it, pt, boost, bt, cur = ff[:cut], it[:cut], pt[:cut], boost[:cut], bt[:cut], cur[:cut]
        n = cut

    err = tgt - vel                       # 统一重新计算，避免与日志列不同帧
    step = A.gcd_step(vel)
    name = clean_name(path)
    out = os.path.join(out_dir, name + "_clean.csv")

    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("# 飞轮遥测数据表（已处理，可直接画图）\n")
        fh.write(f"# 源文件: {os.path.basename(path)}\n")
        fh.write(f"# 样本数: {n}   时长(s): {span:.3f}   重建采样间隔(s): {dt:.6f}   采样率(Hz): {1.0 / dt:.1f}\n")
        fh.write(f"# 原始时间列毫秒倒退处数: {back}" + (f"   无法解析时间行: {bad}" if bad else "") + "\n")
        fh.write(f"# 速度量化台阶(gcd, tick/s): {step:.0f}   速度取值范围: [{np.nanmin(vel):.0f}, {np.nanmax(vel):.0f}]\n")
        fh.write("# 列: t(s), target(tick/s), velocity(tick/s), error(tick/s)=target-velocity,\n")
        fh.write("#     duty(0~1), ff(占空比), i(占空比), p(占空比), boost(0/1), boost_term(占空比), current(A)\n")
        fh.write(",".join(COLS) + "\n")
        for k in range(n):
            fh.write("%.4f,%.0f,%.0f,%.0f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.3f\n" % (
                t[k], tgt[k], vel[k], err[k], duty[k], ff[k], it[k], pt[k],
                1 if boost[k] > 0.5 else 0, bt[k], cur[k]))
    print(f"[表] {out}   （{n} 行，{span:.1f} s，台阶 {step:.0f} tick/s）")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="把原始遥测日志处理成可画图的数据表")
    ap.add_argument("inputs", nargs="+", help="原始 CSV、目录或通配符")
    ap.add_argument("--out", default=None, help="输出目录（默认 <第一个输入目录>/clean）")
    cfg = ap.parse_args(argv)

    files = []
    for p in cfg.inputs:
        if os.path.isdir(p):
            files.extend(sorted(glob.glob(os.path.join(p, "*.csv"))))
        elif any(c in p for c in "*?"):
            files.extend(sorted(glob.glob(p)))
        else:
            files.append(p)
    files = [f for f in files if os.path.isfile(f) and not f.endswith("_clean.csv")]
    if not files:
        print("没有找到输入", file=sys.stderr)
        return 2
    out_dir = cfg.out or os.path.join(os.path.dirname(os.path.abspath(files[0])), "clean")
    os.makedirs(out_dir, exist_ok=True)
    for f in files:
        try:
            export(f, out_dir)
        except SystemExit as e:
            print(f"[跳过] {f}: {e}", file=sys.stderr)
        except Exception as e:
            print(f"[失败] {f}: {type(e).__name__}: {e}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
