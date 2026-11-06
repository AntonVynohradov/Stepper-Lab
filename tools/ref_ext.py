#!/usr/bin/env python3
################################################################################
# Stepper-Lab: Stepper motor model and control simulation
#
# Description: Python twin of the tests T08 (parameter uncertainty), T11 (rotor
#              resonance, detent torque) and T12 (dry friction); the Scilab tests
#              tests/test_T08_*.sce, test_T11_*.sce, test_T12_*.sce compute the same
#              values with the same algorithms.
#
# Author:      Anton Vynohradov
# Email:       avynohradov@systemfromscratch.com
# License:     Apache License 2.0
# Copyright:   (c) 2026 Anton Vynohradov
#
# Usage:       python tools/ref_ext.py
#
# SPDX-License-Identifier: Apache-2.0
#
################################################################################

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ref_model import MOTOR, check, simulate  # noqa: E402
from ref_ctrl import BOARD, CTRL, sim_closed_loop, step_metrics  # noqa: E402

# Parameter cases of T08: plant values relative to what the controller assumes
T08_CURRENT = [(0.9, 0.8), (0.9, 1.2), (1.3, 0.8), (1.3, 1.2)]   # (R, L) factors
T08_MOVE = [(1.0, 1.0, 1.0, 0.0), (1.3, 1.2, 1.0, 0.0), (1.0, 1.0, 5.0, 0.0),
            (1.3, 0.8, 5.0, 0.15)]                                # (R, L, J, TL)

T11_MID = (5, 8, 11)   # rev/s: below, at, beyond the voltage limit (shared leg, 24 V)
TD_T11 = 0.02    # N*m, detent torque used in T11 (assumption: about 3 % of 0.59 N*m)
TC_T12 = 0.01    # N*m, dry friction used in T12 (assumption)


def ref_hold(I, phi):
    return lambda t: (I, phi, 0.0, 0.0)


def ref_speed(I, rev_s, m, t_ramp, TL=0.0):
    """As model/ref_speed_ramp.sci (constant current, linear speed ramp)."""
    p = m["Ns"] / 4
    w_end = 2 * math.pi * rev_s

    def f(t):
        if t < t_ramp:
            w = w_end * t / t_ramp
            th = w_end * t * t / (2 * t_ramp)
        else:
            w = w_end
            th = w_end * t_ramp / 2 + w_end * (t - t_ramp)
        return I, p * th, w, TL
    return f


def ref_angle_step(I, phi1, t_step):
    """As model/ref_angle_step.sci: constant current, electrical angle 0 -> phi1."""
    return lambda t: (I, phi1 if t >= t_step else 0.0, 0.0, 0.0)


def load_angle(log, m):
    """Electrical angle between the current vector and the rotor [deg]."""
    return np.degrees(log[:, 11] - m["Ns"] / 4 * log[:, 4])


def t08(m, c):
    ok = True
    brd = dict(BOARD, topology="shared")
    for fr, fl in T08_CURRENT:
        plant = dict(m, R=m["R"] * fr, L=m["L"] * fl)
        log = sim_closed_loop(plant, brd, c, lambda t: ((2.0 if t >= 1e-3 else 0.0), 0.0, 0.0, 0.0),
                              6e-3, locked=True, m_ctrl=m)
        over, t_rise, t_set, e_ss = step_metrics(log, 1e-3, 2.0)
        ok &= check(f"T08 current step, plant R x{fr}, L x{fl}",
                    over < 0.10 and t_set < 2e-3 and e_ss < 1e-2,
                    f"overshoot {over * 100:.2f} %, settling (2 %) {t_set * 1e3:.3f} ms, "
                    f"steady error {e_ss * 100:.3f} %")
    for fr, fl, fj, TL in T08_MOVE:
        plant = dict(m, R=m["R"] * fr, L=m["L"] * fl, J=m["J"] * fj)
        log = sim_closed_loop(plant, brd, c, ref_speed(2.0, 4.0, m, 0.1, TL), 0.2, m_ctrl=m)
        ang = load_angle(log, m)
        sel = log[:, 0] >= 0.15
        ok &= check(f"T08 ramp to 4 rev/s, plant R x{fr}, L x{fl}, J x{fj}, TL {TL} N*m",
                    np.max(np.abs(ang)) < 90.0,
                    f"max load angle {np.max(np.abs(ang)):.1f} deg, final "
                    f"{np.mean(ang[sel]):.1f} deg")
    return ok


def ring_frequency(t, y):
    """Frequency from the mean period between rising zero crossings of y."""
    k = np.where((y[:-1] < 0) & (y[1:] >= 0))[0]
    tc = t[k] - y[k] * (t[k + 1] - t[k]) / (y[k + 1] - y[k])
    return 1.0 / np.mean(np.diff(tc)), len(tc)


def t11(m, c):
    ok = True
    brd = dict(BOARD, topology="shared")
    p = m["Ns"] / 4
    I = 2.0
    fn = math.sqrt(p * m["Kt"] * I / m["J"]) / (2 * math.pi)
    # T11a: natural frequency after a small angle step (current vector 0 -> 0.1 rad)
    phi1 = 0.1
    log = sim_closed_loop(m, brd, c, ref_angle_step(I, phi1, 5e-3), 0.03)
    sel = log[:, 0] >= 5e-3
    f, n = ring_frequency(log[sel, 0], p * log[sel, 4] - phi1)
    ok &= check("T11 rotor natural frequency sqrt(p Kt I / J) / 2 pi",
                abs(f / fn - 1) < 0.05 and n >= 4,
                f"{f:.1f} Hz from {n} periods (theory {fn:.1f} Hz)")
    # T11b: detent torque excites the rotor at 4 x electrical frequency; resonance when
    # 4 p n = fn
    md = dict(m, Td=TD_T11)
    n_res = fn / (4 * p)
    speeds = np.round(np.arange(0.8, 1.81, 0.1), 2)
    amp = []
    for v in speeds:
        log = sim_closed_loop(md, brd, c, ref_speed(I, v, m, 0.1), 0.3)
        sel = log[:, 0] >= 0.2
        ang = load_angle(log[sel], m)
        amp.append((np.max(ang) - np.min(ang)) / 2)
    amp = np.array(amp)
    v_peak = speeds[np.argmax(amp)]
    ok &= check(f"T11 detent {TD_T11} N*m: largest rotor oscillation at 4 p n = fn",
                abs(v_peak / n_res - 1) < 0.1 and np.max(amp) > 3 * np.min(amp),
                f"peak at {v_peak:.2f} rev/s (theory {n_res:.2f}), oscillation "
                + ", ".join(f"{v:.1f}: {a:.1f}" for v, a in zip(speeds, amp)) + " deg")
    # T11c: mid-band instability: once the voltage limit holds the current, the rotor
    # oscillation is no longer damped by the current loop (shared leg, 24 V)
    osc = []
    for v in T11_MID:
        log = sim_closed_loop(m, brd, c, ref_speed(I, v, m, 0.2), 0.5)
        sel = log[:, 0] >= 0.4
        ang = load_angle(log[sel], m)
        osc.append(((np.max(ang) - np.min(ang)) / 2, float(np.min(log[sel, 9]))))
    ok &= check("T11 mid-band instability: damped below the voltage limit, limit cycle above",
                osc[0][0] < 1.0 and osc[1][0] < 5.0 and osc[2][0] > 20.0,
                ", ".join(f"{v} rev/s: {o:.1f} deg (scale {k:.2f})"
                          for v, (o, k) in zip(T11_MID, osc)))
    return ok


def t12(m):
    """Dry friction: spin-down of an unpowered rotor without back-EMF (Kt = 0):
    w' = -(B w + Tc) / J  ->  stop after (J / B) ln(1 + B w0 / Tc)."""
    md = dict(m, Kt=0.0, Tc=TC_T12)
    w0 = 50.0
    w1 = 0.01 * w0
    a = md["Tc"] / md["B"]
    t_stop = md["J"] / md["B"] * math.log((w0 + a) / a)
    t_1 = md["J"] / md["B"] * math.log((w0 + a) / (w1 + a))
    t = np.linspace(0, 1.5 * t_stop, 6001)
    x = simulate(md, lambda tt, xx: (0.0, 0.0, 0.0), t, x0=(0.0, 0.0, w0, 0.0))
    k = np.argmax(x[2] < w1)
    t_meas = t[k - 1] + (x[2, k - 1] - w1) / (x[2, k - 1] - x[2, k]) * (t[k] - t[k - 1])
    ok = check("T12 dry friction: spin-down time (J/B) ln((w0 + Tc/B) / (w + Tc/B))",
               abs(t_meas / t_1 - 1) < 0.01 and abs(x[2, -1]) < 0.1,
               f"{t_meas * 1e3:.2f} ms to 1 % of {w0:.0f} rad/s (theory {t_1 * 1e3:.2f} ms; "
               f"stop {t_stop * 1e3:.2f} ms), final {x[2, -1]:.4f} rad/s")
    return ok


def main():
    m, c = MOTOR, CTRL
    ok = t08(m, c)
    ok &= t11(m, c)
    ok &= t12(m)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
