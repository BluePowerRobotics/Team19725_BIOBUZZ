#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 5 组合成标定 CSV，格式与 TuningLogger 完全一致。

用途：没有机器人时自检 analyze_tuning.py / analyze_tuning.m 本身。
真值已知，复算值应当还原它们：

    step01  Kd=8000 (tick/s)/duty, tau=0.30 s, omega0=1450
    zn02    等幅振荡 Tu=0.42 s, a=120 tick/s, 锁定 Ku=9e-5
    relay03 极限环 Tu=0.55 s, a≈180 tick/s, dHigh/dLow=0.7/0.5
    drop04  稳态 3000 tick/s, sigma≈15 tick/s, 量化台阶 1 tick/s
    ff05    d_ss = 1.2e-4*w + 0.012（含静摩擦截距）

用法：python make_samples.py [输出目录]     # 默认 ./samples
"""
import math
import os
import random
import sys

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "samples")
os.makedirs(OUT, exist_ok=True)
random.seed(7)


def w(name, lines):
    with open(os.path.join(OUT, name), "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines) + "\n")


def q(v):  # 速度量化到 1 tick/s
    return round(v)


HEAD = ["# t(s),duty,velocity(tick/s)", "t,duty,velocity"]

# ---------------- 6.1 step01 ----------------
lines = (["# flywheel tuning data (step01)"] + HEAD +
         ["# step response, d0=0.15, dStep=0.25, settleSec=2.0, recordSec=3.0, "
          "steadyWindowSec=0.5, tRecover=0.5, ratio=4.0"])
t = 0.0
while t < 2.0:
    lines.append("%.4f,%.4f,%.2f" % (t, 0.15, q(1450 + random.gauss(0, 12))))
    t += 0.02
lines.append("# --- step applied here (t=0 of this segment) ---")
Kd, tau, w0 = 8000.0, 0.30, 1450.0
t = 0.0
while t < 3.0:
    v = w0 + Kd * 0.1 * (1 - math.exp(-t / tau)) + random.gauss(0, 12)
    lines.append("%.4f,%.4f,%.2f" % (t, 0.25, q(v)))
    t += 0.02
lines.append("# omega0=1450.00 omegaSS=2245.00 Kd=8000.000000 tau=0.3012 "
             "Kff=0.00012500 Kp=0.00006024 Ki=0.00166667")
w("step01_1700000000000.csv", lines)

# ---------------- 6.2 zn02 ----------------
lines = (["# flywheel tuning data (zn02)"] + HEAD +
         ["# ZN critical-gain test, target=3000, startKp=5.0E-5, warmupSec=3.0"])
t, Tu, amp, mean = 0.0, 0.42, 120.0, 3000.0
while t < 20.0:
    if t < 12.0:
        v = 3000 * (1 - math.exp(-t / 2.0)) + random.gauss(0, 10)
        d = 0.5
    else:
        v = mean + amp * math.sin(2 * math.pi * (t - 12.0) / Tu) + random.gauss(0, 10)
        d = 0.5 + 0.05 * (1 if math.cos(2 * math.pi * (t - 12.0) / Tu) > 0 else -1)
    lines.append("%.4f,%.4f,%.2f" % (t, d, q(v)))
    t += 0.02
lines.append("# Ku=0.00009000 Tu=0.4200 Kp=0.00004050 Ki=0.00011571")
w("zn02_1700000000001.csv", lines)

# ---------------- 6.3 relay03 ----------------
lines = (["# flywheel tuning data (relay03)"] + HEAD +
         ["# relay feedback, target=3000, dHigh=0.7, dLow=0.5, transientSec=2.0, "
          "measureSec=4.0, hysteresis=0.0"])
t, Tu, amp, target = 0.0, 0.55, 180.0, 3000.0
last = 0.5
while t < 6.0:
    if t < 2.0:
        v = 2600 + random.gauss(0, 10)
    else:
        ph = (t - 2.0) % Tu
        v = target + amp * (1 if ph < Tu / 2 else -1) * 0.8 + random.gauss(0, 8)
    err = target - v
    if err > 0:
        last = 0.7
    elif err < 0:
        last = 0.5
    lines.append("%.4f,%.4f,%.2f" % (t, last, q(v)))
    t += 0.02
lines.append("# a=180.00 Tu=0.5500 Ku=0.00141471 Kp=0.00063662 Ki=0.00138913")
w("relay03_1700000000002.csv", lines)

# ---------------- 6.4 drop04 ----------------
lines = (["# flywheel tuning data (drop04)"] + HEAD +
         ["# drop-detection calibration, target=3000, settleSec=2.0, measureSec=5.0, "
          "kSigma=5.0, kRate=2.5, kBoostRatio=2.0"])
t = 0.0
while t < 7.0:  # 稳定段 + 测量段共 7 s
    lines.append("%.4f,%.4f,%.2f" % (t, 0.62, q(3000 + random.gauss(0, 15))))
    t += 0.02
lines.append("# meanVel=3000.1 sigma=15.02 step=1.0 maxDecel=612.3 dropThreshold=75.10 "
             "dropRate=1530.75 kBoost=0.00010000 Tboost=0.30")
w("drop04_1700000000003.csv", lines)

# ---------------- 6.5 ff05 ----------------
lines = (["# flywheel tuning data (ff05)"] + HEAD +
         ["# feedforward calibration (kff forced 0), settleSec=2.0, windowSec=1.5, "
          "velocityStep=200.0"])
t = 0.0
while t < 8.0:
    lines.append("%.4f,%.4f,%.2f" % (t, 0.5 + 0.05 * math.sin(t), q(2000 + 100 * t + random.gauss(0, 10))))
    t += 0.02
true_kff, bias = 1.2e-4, 0.012  # d_ss = kff*w + bias（含静摩擦截距）
for wv in (1000, 1500, 2000, 2500, 3000):
    lines.append("# point omega=%.1f dss=%.4f kff=%.8f" % (wv, true_kff * wv + bias, true_kff + bias / wv))
w("ff05_1700000000004.csv", lines)

print("samples written to", OUT)
