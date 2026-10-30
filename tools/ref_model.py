#!/usr/bin/env python3
################################################################################
# sfs-stepper-sim: Stepper motor model and control simulation
#
# Description: Python twin of the Scilab motor model (model/*.sci). Same equations,
#              same parameters, independent solver (SciPy). It
#              - checks the model conventions and the analytic tests T0..T2, and
#              - writes tests/reference/*.csv, which the Scilab test X01 compares with.
#
# Author:      Anton Vynohradov
# Email:       avynohradov@systemfromscratch.com
# License:     Apache License 2.0
# Copyright:   (c) 2026 Anton Vynohradov
#
# Usage:       python tools/ref_model.py [--write]
#
# SPDX-License-Identifier: Apache-2.0
#
################################################################################

import argparse
import math
import os
import sys

import numpy as np
from scipy.integrate import solve_ivp

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Same values as params/motor_17hs19_2004s1.sci
MOTOR = dict(
    R=1.60,      # Ohm, datasheet 1.60 +/-10 % at 25 degC
    L=3.0e-3,    # H, datasheet 3.0 mH +/-20 % at 1 kHz
    I_rated=2.0, # A
    Kt=0.2086,   # N*m/A = 0.59 N*m / (sqrt(2) * 2 A), both phases at rated current
    Ns=200,      # full steps per revolution
    J=82e-7,     # kg*m^2, rotor 82 g*cm^2
    B=1e-5,      # N*m*s/rad, assumption
    Td=0.0,      # N*m, detent torque amplitude, unknown (T11 uses 0.02, assumption)
    Tc=0.0,      # N*m, dry (Coulomb) friction, unknown (T12 uses 0.01, assumption)
    w_c=0.05,    # rad/s, speed below which the dry friction is smoothed (tanh)
)


def friction(w, m):
    """Viscous + dry friction torque; the dry part is Tc tanh(w / w_c) (no stiction)."""
    return m["B"] * w + m.get("Tc", 0.0) * math.tanh(w / m.get("w_c", 0.05))


def em(x, m):
    """Back-EMF and torque. Returns (Te, eA, eB)."""
    iA, iB, w, th = x
    p = m["Ns"] / 4.0
    s, c = math.sin(p * th), math.cos(p * th)
    Te = m["Kt"] * (-iA * s + iB * c) - m["Td"] * math.sin(4.0 * p * th)
    eA = -m["Kt"] * w * s
    eB = m["Kt"] * w * c
    return Te, eA, eB


def rhs(t, x, m, src, locked):
    iA, iB, w, th = x
    vA, vB, TL = src(t, x)
    Te, eA, eB = em(x, m)
    diA = (vA - m["R"] * iA - eA) / m["L"]
    diB = (vB - m["R"] * iB - eB) / m["L"]
    if locked:
        dw, dth = 0.0, 0.0
    else:
        dw = (Te - TL - friction(w, m)) / m["J"]
        dth = w
    return [diA, diB, dw, dth]


def simulate(m, src, t, x0=(0.0, 0.0, 0.0, 0.0), locked=False):
    sol = solve_ivp(rhs, (t[0], t[-1]), list(x0), method="Radau", t_eval=t,
                    args=(m, src, locked), rtol=1e-10, atol=1e-12)
    assert sol.success, sol.message
    return sol.y


def src_dc(vA, vB, TL=0.0):
    return lambda t, x: (vA, vB, TL)


def src_open_loop(V, f_el, TL=0.0):
    """Voltage vector rotating at f_el (electrical Hz): vA = V cos, vB = V sin."""
    return lambda t, x: (V * math.cos(2 * math.pi * f_el * t),
                         V * math.sin(2 * math.pi * f_el * t), TL)


def check(name, ok, detail):
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    return ok


def run_tests():
    m = MOTOR
    ok = True

    # T0: power balance eA*iA + eB*iB = Te*w (detent torque excluded: it does no
    # electrical work), and stable equilibrium p*theta = phi for iA = cos, iB = sin
    rng = np.random.default_rng(1)
    worst = 0.0
    m0 = dict(m, Td=0.0)
    for _ in range(1000):
        x = rng.uniform([-3, -3, -500, -10], [3, 3, 500, 10])
        Te, eA, eB = em(x, m0)
        worst = max(worst, abs(eA * x[0] + eB * x[1] - Te * x[2]))
    ok &= check("T00 power balance", worst < 1e-9, f"max |Pe - Pm| = {worst:.2e} W")
    p = m["Ns"] / 4
    phi = 0.7
    th_eq = phi / p
    dth = 1e-6
    t_plus = em((math.cos(phi), math.sin(phi), 0, th_eq + dth), m0)[0]
    t_zero = em((math.cos(phi), math.sin(phi), 0, th_eq), m0)[0]
    ok &= check("T00 equilibrium", abs(t_zero) < 1e-12 and t_plus < 0,
                f"Te(eq) = {t_zero:.1e}, restoring torque {t_plus:.2e} N*m")

    # T1: steady state current V/R, rotor locked and free
    V = m["R"] * m["I_rated"]
    tau = m["L"] / m["R"]
    t = np.linspace(0, 20 * tau, 2001)
    x = simulate(m, src_dc(V, 0.0), t, locked=True)
    err = abs(x[0, -1] - V / m["R"]) / (V / m["R"])
    ok &= check("T01 steady state (locked)", err < 1e-6, f"i = {x[0, -1]:.6f} A, rel. error {err:.1e}")
    x = simulate(m, src_dc(V, 0.0), t, locked=False)
    ok &= check("T01 rotor stays at theta = 0 (free)", abs(x[3, -1]) < 1e-9,
                f"theta = {x[3, -1]:.1e} rad, i = {x[0, -1]:.6f} A")

    # T2: step response i(t) = V/R (1 - exp(-t/tau))
    t = np.linspace(0, 5 * tau, 5001)
    x = simulate(m, src_dc(V, 0.0), t, locked=True)
    ana = V / m["R"] * (1 - np.exp(-t / tau))
    e = np.max(np.abs(x[0] - ana)) / (V / m["R"])
    t63 = np.interp(1 - math.exp(-1), x[0] / (V / m["R"]), t)
    ok &= check("T02 time constant", abs(t63 - tau) / tau < 1e-3 and e < 1e-6,
                f"t63 = {t63 * 1e3:.4f} ms (L/R = {tau * 1e3:.4f} ms), max curve error {e:.1e}")
    return ok


def write_reference():
    """X01: open-loop rotating voltage at 5 Hz electrical from rest, 0.4 s."""
    m = MOTOR
    V = m["R"] * m["I_rated"]
    t = np.round(np.arange(0, 0.4 + 1e-12, 1e-3), 6)
    x = simulate(m, src_open_loop(V, 5.0), t)
    path = os.path.join(ROOT, "tests", "reference", "x01_open_loop_5hz.csv")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="\n") as f:
        f.write("# X01 reference: 17HS19-2004S1, vA = V cos(2 pi 5 t), vB = V sin(2 pi 5 t),"
                f" V = {V:.3f} V, from rest. Generated by tools/ref_model.py (SciPy Radau,"
                " rtol 1e-10).\n")
        f.write("t_s,iA_A,iB_A,w_rad_s,theta_rad\n")
        for k in range(len(t)):
            f.write(f"{t[k]:.6f},{x[0, k]:.9e},{x[1, k]:.9e},{x[2, k]:.9e},{x[3, k]:.9e}\n")
    p = m["Ns"] / 4
    lag = 2 * math.pi * 5.0 * t[-1] - p * x[3, -1]
    print(f"wrote {os.path.relpath(path, ROOT)}: final theta {x[3, -1]:.5f} rad, "
          f"load angle {math.degrees(lag):.2f} deg (electrical)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="write tests/reference/*.csv")
    args = ap.parse_args()
    ok = run_tests()
    if args.write:
        write_reference()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
