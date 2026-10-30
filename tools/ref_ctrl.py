#!/usr/bin/env python3
################################################################################
# sfs-stepper-sim: Stepper motor model and control simulation
#
# Description: Python twin of the discrete current control loop (model/bridge_avg.sci,
#              model/current_pi.sci, model/sim_closed_loop.sci): averaged power stage
#              for both topologies, PI current controllers with back-calculation
#              anti-windup, one-period computation delay, fixed-step RK4 plant.
#              Runs the tests B01, T03, T04 and writes tests/reference/x02_*.csv.
#
# Author:      Anton Vynohradov
# Email:       avynohradov@systemfromscratch.com
# License:     Apache License 2.0
# Copyright:   (c) 2026 Anton Vynohradov
#
# Usage:       python tools/ref_ctrl.py [--write]
#
# SPDX-License-Identifier: Apache-2.0
#
################################################################################

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ref_model import MOTOR, em, check, friction  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Same values as params/board_bg431b_esc1.sci
BOARD = dict(
    vbus=24.0,          # V, laboratory supply (the board accepts 3S..6S)
    topology="shared",  # "shared" (3 legs, B-G431B-ESC1) or "full" (2 H-bridges)
    fs=20000.0,         # Hz, fast loop = PWM frequency
)

# Same values as params/ctrl_default.sci
CTRL = dict(
    bw_hz=1000.0,  # current loop bandwidth
    delay=1,       # computation delay in fast loop periods (0 or 1)
    ff=1,          # back-EMF feed-forward from the reference speed (0 or 1)
    nsub=4,        # RK4 sub-steps per fast loop period
)


def bridge_avg(vref, vbus, topology):
    """Phase voltage request -> (applied phase voltages, leg duties, scale k).

    The voltage vector is scaled down (direction kept) when it cannot be produced.
    full:   4 legs, |vA| <= vbus and |vB| <= vbus
    shared: 3 legs (A between 0 and 2, B between 1 and 2), max(a,b,0) - min(a,b,0) <= 1
    """
    a = vref[0] / vbus
    b = vref[1] / vbus
    if topology == "full":
        m = max(abs(a), abs(b))
        k = 1.0 / m if m > 1.0 else 1.0
        a, b = a * k, b * k
        duty = [0.5 + a / 2, 0.5 - a / 2, 0.5 + b / 2, 0.5 - b / 2]
    else:
        hi = max(a, b, 0.0)
        lo = min(a, b, 0.0)
        span = hi - lo
        k = 1.0 / span if span > 1.0 else 1.0
        a, b, hi, lo = a * k, b * k, hi * k, lo * k
        w = 0.5 - (hi + lo) / 2
        duty = [a + w, b + w, w]
    return (a * vbus, b * vbus), duty, k


def ctrl_gains(m, c, fs):
    wc = 2 * math.pi * c["bw_hz"]
    Ts = 1.0 / fs
    return dict(Kp=m["L"] * wc, Ki=m["R"] * wc, Ts=Ts, Kb=Ts * m["R"] / m["L"])


def current_pi(iref, imeas, integ, ff, g):
    """One PI step per phase. Returns (unsaturated voltage, integrator after P part)."""
    e = [iref[0] - imeas[0], iref[1] - imeas[1]]
    vu = [g["Kp"] * e[0] + integ[0] + ff[0], g["Kp"] * e[1] + integ[1] + ff[1]]
    return vu, e


def pi_update(integ, e, vu, vsat, g):
    """Integrator with back-calculation: Ki Ts e + Ts/Tt (vsat - vu), Tt = L/R."""
    return [integ[j] + g["Ki"] * g["Ts"] * e[j] + g["Kb"] * (vsat[j] - vu[j]) for j in range(2)]


def plant_f(x, u, m, locked):
    Te, eA, eB = em(x, m)
    diA = (u[0] - m["R"] * x[0] - eA) / m["L"]
    diB = (u[1] - m["R"] * x[1] - eB) / m["L"]
    if locked:
        return np.array([diA, diB, 0.0, 0.0])
    return np.array([diA, diB, (Te - u[2] - friction(x[2], m)) / m["J"], x[2]])


def rk4(x, u, m, h, locked):
    k1 = plant_f(x, u, m, locked)
    k2 = plant_f(x + h / 2 * k1, u, m, locked)
    k3 = plant_f(x + h / 2 * k2, u, m, locked)
    k4 = plant_f(x + h * k3, u, m, locked)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def sim_closed_loop(m, brd, c, ref, T, locked=False, m_ctrl=None):
    """ref(t) -> (I amplitude [A], phi [rad electrical], w_ref [rad/s mech], TL [N*m]).
    ref may keep state (it is called once per period, in time order).
    log columns: t iA iB w theta iA_ref iB_ref vA vB k I phi
    m_ctrl: motor parameters known to the controller (gains, feed-forward); default m.

    Order in each period k (t = k Ts): sample currents, compute the references and the
    controller, apply the voltage computed in the previous period (delay = 1) or now
    (delay = 0) for one period, integrate the plant.
    """
    mc = m if m_ctrl is None else m_ctrl
    g = ctrl_gains(mc, c, brd["fs"])
    Ts = g["Ts"]
    N = int(round(T / Ts))
    x = np.zeros(4)
    integ = [0.0, 0.0]
    v_hold = (0.0, 0.0)
    log = np.zeros((N, 12))
    for k in range(N):
        t = k * Ts
        I, phi, w_ref, TL = ref(t)
        iref = (I * math.cos(phi), I * math.sin(phi))
        ff = (0.0, 0.0)
        if c["ff"]:
            ff = (-mc["Kt"] * w_ref * math.sin(phi), mc["Kt"] * w_ref * math.cos(phi))
        vu, e = current_pi(iref, (x[0], x[1]), integ, ff, g)
        vsat, duty, ksc = bridge_avg(vu, brd["vbus"], brd["topology"])
        integ = pi_update(integ, e, vu, vsat, g)
        v_apply = v_hold if c["delay"] else vsat
        v_hold = vsat
        log[k] = [t, x[0], x[1], x[2], x[3], iref[0], iref[1], v_apply[0], v_apply[1], ksc, I, phi]
        u = (v_apply[0], v_apply[1], TL)
        h = Ts / c["nsub"]
        for _ in range(c["nsub"]):
            x = rk4(x, u, m, h, locked)
    return log


# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------

def test_b01():
    ok = True
    rng = np.random.default_rng(2)
    for topo in ("full", "shared"):
        worst_dir, worst_exact, duty_ok = 0.0, 0.0, True
        for _ in range(20000):
            v = rng.uniform(-40, 40, 2)
            (va, vb), duty, k = bridge_avg(v, 24.0, topo)
            duty_ok &= all(-1e-12 <= d <= 1 + 1e-12 for d in duty)
            # direction kept
            cross = abs(va * v[1] - vb * v[0]) / (np.hypot(*v) * max(np.hypot(va, vb), 1e-9))
            worst_dir = max(worst_dir, cross)
            # applied voltages equal leg differences
            if topo == "full":
                da, db = (duty[0] - duty[1]) * 24, (duty[2] - duty[3]) * 24
            else:
                da, db = (duty[0] - duty[2]) * 24, (duty[1] - duty[2]) * 24
            worst_exact = max(worst_exact, abs(da - va), abs(db - vb))
            if k == 1.0:
                worst_exact = max(worst_exact, abs(va - v[0]), abs(vb - v[1]))
        ok &= check(f"B01 {topo}: duties in 0..1, direction kept, exact when feasible",
                    duty_ok and worst_dir < 1e-12 and worst_exact < 1e-12,
                    f"direction error {worst_dir:.1e}, voltage error {worst_exact:.1e} V")
    # largest sinusoidal amplitude: full -> vbus, shared -> vbus / sqrt(2)
    for topo, want in (("full", 24.0), ("shared", 24.0 / math.sqrt(2))):
        amp = min(np.hypot(*bridge_avg((100 * math.cos(p), 100 * math.sin(p)), 24.0, topo)[0])
                  for p in np.linspace(0, 2 * math.pi, 3601))
        ok &= check(f"B01 {topo}: largest circular voltage amplitude", abs(amp - want) < 1e-9,
                    f"{amp:.4f} V (expected {want:.4f} V)")
    return ok


def ref_t04(t):
    """Locked rotor, phi = 0 (phase A), current step 0 -> 2 A at 1 ms."""
    return (2.0 if t >= 1e-3 else 0.0), 0.0, 0.0, 0.0


def step_metrics(log, t_step, target, j=1):
    t = log[:, 0]
    i = log[:, j]
    after = t >= t_step
    over = (np.max(i[after]) - target) / target
    outside = np.where(after & (np.abs(i - target) > 0.02 * target))[0]
    t_settle = t[outside[-1]] - t_step + (t[1] - t[0]) if len(outside) else 0.0
    t_rise = t[np.argmax(after & (i >= 0.9 * target))] - t_step
    e_ss = abs(i[-1] - target) / target
    return over, t_rise, t_settle, e_ss


def test_t04(m, brd, c):
    ok = True
    for topo in ("full", "shared"):
        b = dict(brd, topology=topo)
        log = sim_closed_loop(m, b, c, ref_t04, 6e-3, locked=True)
        over, t_rise, t_set, e_ss = step_metrics(log, 1e-3, 2.0)
        ok &= check(f"T04 {topo}: current step 0 -> 2 A",
                    over < 0.05 and t_set < 1e-3 and e_ss < 1e-3,
                    f"overshoot {over * 100:.2f} %, rise (90 %) {t_rise * 1e3:.3f} ms, "
                    f"settling (2 %) {t_set * 1e3:.3f} ms, steady error {e_ss * 100:.3f} %")
    return ok


def ref_t03(t):
    """Slow microstepping: 2 A, phi ramps at 10 Hz electrical (0.2 rev/s), 0.3 s."""
    p = MOTOR["Ns"] / 4
    f_el = 10.0
    return 2.0, 2 * math.pi * f_el * t, 2 * math.pi * f_el / p, 0.0


def test_t03(m, brd, c):
    ok = True
    log = sim_closed_loop(m, brd, c, ref_t03, 0.3)
    sel = log[:, 0] >= 0.1                       # after the start transient
    t, iA, iB = log[sel, 0], log[sel, 1], log[sel, 2]
    w = 2 * math.pi * 10.0
    basis = np.column_stack([np.cos(w * t), np.sin(w * t)])
    cA = np.linalg.lstsq(basis, iA, rcond=None)[0]
    cB = np.linalg.lstsq(basis, iB, rcond=None)[0]
    ampA, ampB = np.hypot(*cA), np.hypot(*cB)
    phA, phB = math.atan2(-cA[1], cA[0]), math.atan2(-cB[1], cB[0])  # i = amp cos(wt + ph)
    shift = math.degrees((phA - phB + math.pi) % (2 * math.pi) - math.pi)
    err = np.max(np.hypot(log[sel, 1] - log[sel, 5], log[sel, 2] - log[sel, 6]))
    ok &= check("T03 microstep: A leads B by 90 deg, amplitude 2 A",
                abs(shift - 90.0) < 0.5 and abs(ampA - 2) < 0.01 and abs(ampB - 2) < 0.01,
                f"shift {shift:.3f} deg, amplitudes {ampA:.4f} / {ampB:.4f} A, "
                f"max tracking error {err * 1e3:.2f} mA")
    p = m["Ns"] / 4
    lag = math.degrees(w * log[-1, 0] - p * log[-1, 4])  # phi - p theta at the last sample
    ok &= check("T03 rotor follows the field", abs(lag) < 20.0,
                f"rotor {log[-1, 4]:.4f} rad, electrical lag {lag:.2f} deg")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()
    m, brd, c = MOTOR, BOARD, CTRL
    g = ctrl_gains(m, c, brd["fs"])
    print(f"gains: Kp = {g['Kp']:.4f} V/A, Ki = {g['Ki']:.2f} V/(A*s), Ts = {g['Ts'] * 1e6:.1f} us")
    ok = test_b01()
    ok &= test_t04(m, brd, c)
    ok &= test_t03(m, brd, c)
    if args.write:
        log = sim_closed_loop(m, brd, c, ref_t04, 3e-3, locked=True)
        path = os.path.join(ROOT, "tests", "reference", "x02_current_step_shared.csv")
        with open(path, "w", newline="\n") as f:
            f.write("# X02 reference: closed current loop, shared-leg bridge, 24 V, 20 kHz, "
                    "bw 1 kHz, delay 1, locked rotor, step 0 -> 2 A at 1 ms. "
                    "Generated by tools/ref_ctrl.py.\n")
            f.write("t_s,iA_A,iB_A,vA_V,vB_V\n")
            for r in log:
                f.write(f"{r[0]:.8f},{r[1]:.12e},{r[2]:.12e},{r[7]:.12e},{r[8]:.12e}\n")
        print(f"wrote {os.path.relpath(path, ROOT)}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
