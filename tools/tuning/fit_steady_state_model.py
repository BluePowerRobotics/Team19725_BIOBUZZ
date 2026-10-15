#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从处理完的数据表（export_clean_table.py 产出的 *_clean.csv）拟合稳态模型

     d_ss(ω) = kS + kff·ω

其中
    kS  —— 静摩擦/预紧/齿槽转矩对应的那份占空比（不产生转速，只对抗摩擦），单位 占空比
    kff —— 稳态前馈增益，单位 (占空比)/(tick/s)

做法：把每段「目标恒定且持续 ≥ min_dwell 秒」的区间当作一个稳态工作点，
取该段末尾 window 秒的转速与占空比均值作为一个样本点，再对所有样本做一次线性回归。
（段内末尾取均值可避开换档暂态；这也是 why 短档位必须被 dwell 过滤掉——它们会把 kS 抬高。）

用法:
    python fit_steady_state_model.py out/clean/*.csv
    python fit_steady_state_model.py out/clean/*.csv --min-dwell 2.0 --window 1.0
"""

import argparse
import glob
import os
import sys

import numpy as np


def load_clean(path):
    """读取带 '#' 注释头的 CSV，返回 (meta 行列表, 结构化数组)

    注意：不要用 genfromtxt(comments='#', names=True)——在「# 注释头 + 表头」
    组合下 numpy 会把第一行注释当成字段名，随后整列报
    “got 11 columns instead of 1”。这里显式数出注释行数再 skip_header。
    """
    meta = []
    n_meta = 0
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                meta.append(line.strip())
                n_meta += 1
            else:
                break
    data = np.genfromtxt(path, delimiter=",", names=True, skip_header=n_meta,
                         encoding="utf-8")
    return meta, data


def steady_points(data, min_dwell=2.0, window=1.0, max_sd=25.0):
    """提取稳态工作点 (omega, duty)，按「目标恒定 + 持续够久」分段

    max_sd: 末尾窗口内 ω 的标准差上限（tick/s）。正常稳态约 10 tick/s
            （半个量化台阶），超过说明该段还在收敛（例如刚从高转速回落），
            这类点会把拟合带偏，必须剔除。
    """
    t, tgt, vel, duty = data["t"], data["target"], data["velocity"], data["duty"]
    pts = []
    segs = []
    dropped = []
    start = 0
    for i in range(1, len(t) + 1):
        boundary = (i == len(t)) or (tgt[i] != tgt[i - 1]) or ((t[i] - t[i - 1]) > 0.05)
        if not boundary:
            continue
        i0, i1 = start, i
        start = i
        if i1 - i0 < 3:
            continue
        seg_t = t[i1 - 1] - t[i0]
        target = tgt[i0]
        if target == 0 or seg_t < min_dwell:
            continue
        # 末尾 window 秒
        t_end = t[i1 - 1]
        j0 = int(np.searchsorted(t[i0:i1], t_end - window)) + i0
        if i1 - j0 < 3:
            continue
        w_v = float(np.mean(vel[j0:i1]))
        w_d = float(np.mean(duty[j0:i1]))
        w_sd = float(np.std(vel[j0:i1]))
        if w_sd > max_sd:
            dropped.append((target, seg_t, w_v, w_d, w_sd))
            continue
        pts.append((w_v, w_d))
        segs.append((target, seg_t, w_v, w_d, w_sd))
    return pts, segs, dropped


def main():
    ap = argparse.ArgumentParser(description="拟合稳态模型 d_ss(ω) = kS + kff·ω")
    ap.add_argument("files", nargs="+", help="处理完的数据表（*_clean.csv）")
    ap.add_argument("--min-dwell", type=float, default=2.0, help="段最短持续时间 (s)，默认 2.0")
    ap.add_argument("--window", type=float, default=1.0, help="末尾取均值的窗口 (s)，默认 1.0")
    ap.add_argument("--max-sd", type=float, default=25.0,
                    help="末尾窗口内 ω 标准差上限 (tick/s)，超过视为未稳态并剔除，默认 25")
    ap.add_argument("--out", default=None, help="把报告也写入该文件")
    args = ap.parse_args()

    paths = []
    for pat in args.files:
        hits = sorted(glob.glob(pat))
        paths.extend(hits if hits else [pat])

    lines = []

    def emit(s=""):
        print(s)
        lines.append(s)

    emit("=" * 78)
    emit("稳态模型拟合  d_ss(ω) = kS + kff·ω")
    emit("    min_dwell=%.1f s, window=%.1f s, max_sd=%.1f tick/s"
         % (args.min_dwell, args.window, args.max_sd))
    emit("=" * 78)

    all_pts = []
    for path in paths:
        if not os.path.isfile(path):
            emit("[跳过] 文件不存在: %s" % path)
            continue
        meta, data = load_clean(path)
        pts, segs, dropped = steady_points(data, args.min_dwell, args.window, args.max_sd)
        name = os.path.basename(path)
        emit("")
        emit("--- %s ---" % name)
        for m in meta:
            if "台阶" in m or "样本" in m or "时长" in m:
                emit("    " + m)
        if not segs:
            emit("    （无满足条件的稳态段）")
        else:
            emit("    工作点（目标 / 时长 s / ω 均值 / duty 均值 / ω sd）：")
            for target, seg_t, v, d, sd in segs:
                emit("      %6d / %6.2f / %9.1f / %.4f / %5.2f" % (target, seg_t, v, d, sd))
        for target, seg_t, v, d, sd in dropped:
            emit("      [剔除] 目标 %d / %6.2f s / ω %9.1f / duty %.4f / sd %5.2f (>%.0f，未稳态)"
                 % (target, seg_t, v, d, sd, args.max_sd))
        # 单文件拟合（至少 2 点才有意义）
        if len(pts) >= 2:
            xs = np.array([p[0] for p in pts])
            ys = np.array([p[1] for p in pts])
            kff_i, kS_i = np.polyfit(xs, ys, 1)
            emit("    单文件拟合: kS=%.5f  kff=%.8f" % (kS_i, kff_i))
        all_pts.extend(pts)

    if len(all_pts) < 2:
        emit("")
        emit("样本点不足（<2），无法拟合。")
        return

    xs = np.array([p[0] for p in all_pts])
    ys = np.array([p[1] for p in all_pts])
    kff, kS = np.polyfit(xs, ys, 1)
    pred = kS + kff * xs
    ss_res = float(np.sum((ys - pred) ** 2))
    ss_tot = float(np.sum((ys - np.mean(ys)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rms = float(np.sqrt(ss_res / len(xs)))

    # 过原点拟合（用于对比：不拆 kS 时「一个 kff 打天下」的偏差）
    kff0 = float(np.sum(xs * ys) / np.sum(xs * xs))
    resid0 = ys - kff0 * xs

    emit("")
    emit("=" * 78)
    emit("全部样本合并拟合（%d 个工作点）" % len(xs))
    emit("=" * 78)
    emit("    kS  = %.6f   (占空比)          ← 静摩擦/预紧前馈" % kS)
    emit("    kff = %.8f (占空比)/(tick/s)  ← 稳态前馈增益" % kff)
    emit("    R² = %.6f    RMS 残差 = %.5f 占空比" % (r2, rms))
    emit("")
    emit("    过原点拟合（不拆 kS）: kff0 = %.8f，残差 RMS = %.5f 占空比"
         % (kff0, float(np.sqrt(np.mean(resid0 ** 2)))))
    emit("    拆分收益: 残差 RMS %.5f → %.5f（%.0f%% 改善）"
         % (float(np.sqrt(np.mean(resid0 ** 2))), rms,
            100.0 * (1 - rms / float(np.sqrt(np.mean(resid0 ** 2))))))
    emit("")
    emit("    逐点残差（拟合 duty − 实测 duty，占空比）:")
    emit("      %8s %9s %9s %9s" % ("ω", "实测 duty", "拟合 duty", "残差"))
    for x, y, p in zip(xs, ys, pred):
        emit("      %8.1f %9.4f %9.4f %+9.4f" % (x, y, p, p - y))

    emit("")
    emit("    对每个工作点，新模型相对「当前 kff=0.00043203 无 kS」的稳态 duty 变化:")
    kff_now = 0.00043203
    for x, y in zip(xs, ys):
        new = kS + kff * x
        old = kff_now * x
        emit("      ω=%7.1f: 旧 %.4f → 新 %.4f  (%+.4f)" % (x, old, new, new - old))

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        print("\n[报告] %s" % args.out)


if __name__ == "__main__":
    sys.exit(main())
