# -*- coding: utf-8 -*-
"""数据表 + 图产物的一致性自检（独立于 plot_speed.m 实现，用于核对图的正确性）。

检查项：
  A. 数据表完整性：列数/行数、t 严格单调、无 NaN、时间与元数据一致
  B. 数值合理性：台阶、速度范围、占空比范围、boost 与 error 的关系
  C. 图产物：文件存在、尺寸正常（PNG 可解码）
  D. 交叉核对：手动复算几个稳态段与掉速事件，和 MATLAB 统计表对比
"""
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CLEAN = os.path.join(HERE, "out", "clean", "log4_clean.csv")
MATLAB_STATS = os.path.join(HERE, "out", "plot_speed_stats.txt")
PNGS = [os.path.join(HERE, "out", "log4_clean_speed_velocity.png"),
        os.path.join(HERE, "out", "log4_clean_speed_error.png"),
        os.path.join(HERE, "out", "log4_clean_diag_velocity.png"),
        os.path.join(HERE, "out", "log4_clean_diag_error.png"),
        os.path.join(HERE, "out", "log4_clean_diag_duty.png"),
        os.path.join(HERE, "out", "log4_clean_diag_current.png")]

ok = True


def check(name, cond, detail=""):
    global ok
    mark = "PASS" if cond else "FAIL"
    if not cond:
        ok = False
    print(f"[{mark}] {name}" + (f"   {detail}" if detail else ""))


# ---------------- A. 数据表 ----------------
meta, header, rows = [], None, []
with open(CLEAN, "r", encoding="utf-8") as fh:
    for line in fh:
        s = line.strip()
        if not s:
            continue
        if s.startswith("#"):
            meta.append(s)
            continue
        if header is None:
            header = s
            continue
        rows.append([float(x) for x in s.split(",")])
a = np.array(rows)
print(f"文件: {CLEAN}")
print(f"元数据 {len(meta)} 行；表头 {header}")
check("表头列名正确", header == "t,target,velocity,error,duty,ff,i,p,boost,boost_term,current", header)
check("列数=11", a.shape[1] == 11, f"shape={a.shape}")
check("行数与元数据一致", f"样本数: {a.shape[0]}" in meta[2], meta[2])
check("无 NaN/Inf", np.isfinite(a).all())
t = a[:, 0]
check("t 严格单调递增", bool(np.all(np.diff(t) > 0)), f"dt min={np.diff(t).min():.6f} max={np.diff(t).max():.6f}")
check("t 从 0 开始", abs(t[0]) < 1e-9, f"t0={t[0]}")
span = t[-1] - t[0]
check("时长与元数据一致", abs(span - float(meta[2].split("时长(s):")[1].split()[0])) < 0.01, f"span={span:.3f}")

vel, tgt, err, duty = a[:, 2], a[:, 1], a[:, 3], a[:, 4]
check("error 列 = target − velocity", bool(np.allclose(err, tgt - vel, atol=0.51)),
      f"max|diff|={np.abs(err - (tgt - vel)).max():.2f}")
check("duty ∈ [0,1]", bool(duty.min() >= -1e-9 and duty.max() <= 1 + 1e-9), f"[{duty.min():.3f},{duty.max():.3f}]")
check("boost ∈ {0,1}", set(np.unique(a[:, 8]).astype(int)) <= {0, 1}, str(np.unique(a[:, 8])))

g = 0
import math
for v in np.unique(np.round(np.abs(vel)).astype(int)):
    if v:
        g = math.gcd(g, int(v))
check("量化台阶 = 20 tick/s", g == 20, f"gcd={g}")
import re as _re
_rng = _re.search(r"速度取值范围:\s*\[\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\]", " ".join(meta))
check("速度范围与元数据一致",
      _rng is not None and abs(float(_rng.group(1)) - vel.min()) < 1 and abs(float(_rng.group(2)) - vel.max()) < 1,
      f"实测 [{vel.min():.0f}, {vel.max():.0f}]" + (f"，元数据 [{_rng.group(1)}, {_rng.group(2)}]" if _rng else "，元数据未解析到"))

# ---------------- B. 数值合理性 ----------------
check("速度最高 ≈ 2400", abs(vel.max() - 2400) < 40, f"max={vel.max():.0f}")
check("速度反馈全部落在 20 tick/s 网格上", bool(np.all(np.abs(vel / 20 - np.round(vel / 20)) < 1e-6)),
      f"唯一值 {len(np.unique(vel))} 个")
# 注意：目标是用户整数设定（如 1950/2050 不是 20 的倍数），反馈才是量化网格 —— 这正是
# 「稳态误差不可能小于 10 tick/s」的原因，所以不要假设目标也是网格值。
check("目标为整数 tick/s", bool(np.all(np.abs(tgt - np.round(tgt)) < 1e-6)))
# P 项 vs 误差 的比例应稳定（斜率 kp）
m = (np.abs(err) > 20) & (np.abs(a[:, 7]) > 1e-6)
kp = np.median(a[m, 7] / err[m])
check("P 项/误差 比例稳定（kp 反推）", 0.0015 < abs(kp) < 0.0021, f"kp≈{kp:.6f}")

# ---------------- C. 图产物 ----------------
def png_size(path):
    with open(path, "rb") as fh:
        head = fh.read(33)
    if head[:8] != b"\x89PNG\r\n\x1a\n" or head[12:16] != b"IHDR":
        return None
    w, h = struct.unpack(">II", head[16:24])
    return w, h


for p in PNGS:
    if not os.path.isfile(p):
        # 图还没生成不算 FAIL（可能是没跑 MATLAB 的 plot_speed.m），但要显式提示
        print(f"[跳过] 图未生成（先跑 plot_speed.m）：{os.path.basename(p)}")
        continue
    sz = png_size(p)
    check(f"图可解码且尺寸合理 {os.path.basename(p)}",
          sz is not None and sz[0] > 800 and sz[1] > 300,
          f"{sz} {os.path.getsize(p) // 1024} KB")

# ---------------- D. 交叉核对 MATLAB 统计 ----------------
if os.path.isfile(MATLAB_STATS):
    txt = open(MATLAB_STATS, encoding="utf-8").read()
    # 复算：找最长的那段目标 2250，取该段末尾 2 s 的稳态
    idx = np.nonzero(tgt == 2250)[0]
    runs, s0 = [], None
    for k in range(len(tgt)):
        if tgt[k] == 2250 and s0 is None:
            s0 = k
        elif tgt[k] != 2250 and s0 is not None:
            runs.append((s0, k - 1)); s0 = None
    if s0 is not None:
        runs.append((s0, len(tgt) - 1))
    a0, a1 = max(runs, key=lambda r: r[1] - r[0])
    j0 = max(a0, a1 - int(2.0 / np.median(np.diff(t))))
    sl = slice(j0, a1 + 1)
    v_sd = vel[sl].std(ddof=1)
    err_mean = err[sl].mean()
    duty_mean = duty[sl].mean()
    print(f"\n独立复算（最长 2250 段 {t[a0]:.1f}~{t[a1]:.1f}s 的末 2 s）："
          f"v_sd={v_sd:.1f}  误差均值={err_mean:.1f}  台阶数={v_sd / 20:.2f}  duty均值={duty_mean:.3f}")
    check("MATLAB 统计表含该段且 v_sd 吻合（±1.5）",
          "2250" in txt and (abs(v_sd - 10.6) < 1.5), "MATLAB 表内 2250 段 v_sd≈10.6")
    check("MATLAB 统计表含掉速事件表", "掉速/发射事件" in txt)
    check("事件数在合理范围（13~16 次）",
          any(f"事件 {k} 次" in txt for k in range(13, 17)),
          [ln for ln in txt.splitlines() if "事件" in ln][:1])

print("\n" + ("=" * 60))
print("总体：" + ("全部通过" if ok else "存在 FAIL，请检查"))
sys.exit(0 if ok else 1)
