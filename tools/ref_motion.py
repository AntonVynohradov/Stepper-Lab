#!/usr/bin/env python3
################################################################################
# sfs-stepper-sim: Stepper motor model and control simulation
#
# Description: Python twin of the motion generator (model/ref_motion.sci): slow loop
#              (trapezoidal velocity profile, move / run / stop commands, hold
#              current) with the angle interpolated in every fast loop period.
#              Runs the tests T05, T06, T07, T09 and writes tests/reference/x03_*.csv.
#
# Author:      Anton Vynohradov
# Email:       avynohradov@systemfromscratch.com
# License:     Apache License 2.0
# Copyright:   (c) 2026 Anton Vynohradov
#
# Usage:       python tools/ref_motion.py [--write]
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
from ref_model import MOTOR, check  # noqa: E402
from ref_ctrl import BOARD, CTRL, sim_closed_loop  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Same values as params/motion_default.sci
MOTION = dict(
    M=16,               # microsteps per full step
    f_slow=1000.0,      # Hz, slow loop rate
    v_max=16000.0,      # microsteps/s (5 rev/s with M = 16)
    acc=160000.0,       # microsteps/s^2 (50 rev/s^2)
    dec=160000.0,       # microsteps/s^2
    i_run=2.0,          # A
    i_hold=1.0,         # A
    hold_delay=0.1,     # s
)

MOVE, RUN, STOP, LOAD = 1, 2, 3, 4


Q16 = 65536


def div_round(num, den):
    """C: round(num / den) half away from zero, den > 0 (sstep_ctrl.c div_round)."""
    return (num + den // 2) // den if num >= 0 else -((-num + den // 2) // den)


def c_div(num, den):
    """C integer division: truncation toward zero."""
    q = abs(num) // abs(den)
    return q if (num >= 0) == (den > 0) else -q


def isqrt(x):
    return math.isqrt(x)


class Motion:
    """Reference generator, bit-exact twin of the Stepper-C slow loop (sstep_motion.c,
    sstep_slow_loop) and of the angle interpolation of the fast loop. Called once per
    fast loop period, in time order.

    Integer units as in C: position Q16 microsteps, velocity Q16 microsteps per slow
    period, acceleration Q16 per period^2.
    cmds: rows (t, kind, value): MOVE target [usteps], RUN velocity [usteps/s],
    STOP (decelerate to 0), LOAD torque [N*m].
    """

    def __init__(self, m, brd, mp, cmds):
        self.m, self.mp, self.cmds = m, mp, cmds
        self.ratio = int(round(brd["fs"] / mp["f_slow"]))
        self.f = int(mp["f_slow"])
        self.dt = 1.0 / mp["f_slow"]
        self.shift = 14 - int(round(math.log2(mp["M"])))
        # sstep_ctrl.c set_limits()
        self.v_max = div_round(int(mp["v_max"]) * Q16, self.f)
        self.acc = div_round(int(mp["acc"]) * Q16, self.f * self.f)
        self.dec = div_round(int(mp["dec"]) * Q16, self.f * self.f)
        self.brake_vmax = (self.v_max * self.v_max) // (2 * self.dec)
        self.hold_ticks = (int(round(mp["hold_delay"] * 1000)) * self.f) // 1000
        self.n = 0
        self.ci = 0
        self.mode = 0          # 0 hold, MOVE, RUN
        self.target = 0
        self.v_run = 0
        self.pos = 0
        self.vel = 0
        self.still = 0
        self.hold_active = False
        self.angle0 = 0        # unwrapped electrical angle at the slow period start [2^-32 turn]
        self.dangle = 0
        self.k = 0
        self.v_s = 0           # mean velocity of the period [usteps/s]
        self.I = mp["i_run"]
        self.TL = 0.0
        self.hist = []         # (t, pos [usteps], v [usteps/s], vel Q16, mode) per slow tick

    def target_velocity(self):
        if self.mode == MOVE:
            rem = self.target - self.pos
            arem, av, d = abs(rem), abs(self.vel), self.dec
            if arem >= self.brake_vmax + (self.v_max + av) // 2:
                mag = self.v_max
            else:
                disc = d * d + 8 * d * arem - 4 * d * av
                root = isqrt(disc) if disc > 0 else 0
                mag = (root - d) // 2 if root > d else 0
            return mag if rem > 0 else (-mag if rem < 0 else 0)
        if self.mode == RUN:
            return max(-self.v_max, min(self.v_max, self.v_run))
        return 0

    def motion_step(self):
        """sstep_motion_step(): returns True when a move reached its target."""
        done = False
        vt = self.target_velocity()
        speeding_up = abs(vt) > abs(self.vel) and (self.vel == 0 or (vt > 0) == (self.vel > 0))
        a = self.acc if speeding_up else self.dec
        v_new = self.vel + max(-a, min(a, vt - self.vel))
        p_new = self.pos + (self.vel + v_new) // 2          # floor, as half_floor()
        if self.mode == MOVE:
            rem0 = self.target - self.pos
            rem1 = self.target - p_new
            crossed = (rem0 >= 0 and rem1 <= 0) or (rem0 <= 0 and rem1 >= 0)
            near = abs(rem1) < 64 and abs(v_new) <= self.dec
            if crossed or near:
                p_new, v_new, self.mode, done = self.target, 0, 0, True
        self.pos, self.vel = p_new, v_new
        return done

    def slow(self, t):
        mp = self.mp
        while self.ci < len(self.cmds) and self.cmds[self.ci][0] <= t + 1e-12:
            _, kind, val = self.cmds[self.ci]
            if kind == MOVE:
                self.mode, self.target = MOVE, int(val) * Q16
            elif kind == RUN:
                self.mode, self.v_run = RUN, div_round(int(val) * Q16, self.f)
            elif kind == STOP:
                if self.mode != 0:
                    self.mode, self.v_run = RUN, 0
            else:
                self.TL = float(val)
            self.ci += 1

        p0 = self.pos
        self.motion_step()
        if self.mode == RUN and self.v_run == 0 and self.vel == 0:
            self.mode, self.target = 0, self.pos           # stop completed
        dp = self.pos - p0
        if dp != 0:
            self.still, self.hold_active = 0, False
        else:
            self.still += 1
            if self.still >= self.hold_ticks and not self.hold_active:
                self.hold_active = True
        self.I = mp["i_hold"] if self.hold_active else mp["i_run"]
        self.angle0 = p0 << self.shift                     # unwrapped
        self.dangle = c_div(dp << self.shift, self.ratio)
        self.k = 0
        self.v_s = div_round(dp * self.f, Q16)
        self.hist.append((t, self.pos / Q16, self.vel * self.f / Q16, self.vel, self.mode))

    def __call__(self, t):
        if self.n % self.ratio == 0:
            self.slow(t)
        m, mp = self.m, self.mp
        angle = self.angle0 + self.k * self.dangle
        phi = angle * (2.0 * math.pi / 4294967296.0)
        w_ref = self.v_s * 2.0 * math.pi / (m["Ns"] * mp["M"])
        if self.k < self.ratio:
            self.k += 1
        self.n += 1
        return self.I, phi, w_ref, self.TL


def load_angle(log, m):
    """phi - p theta [deg electrical]; positive = rotor behind the field."""
    return np.degrees(log[:, 11] - m["Ns"] / 4 * log[:, 4])


def run(m, brd, c, mp, cmds, T):
    mo = Motion(m, brd, mp, cmds)
    log = sim_closed_loop(m, brd, c, mo, T)
    return log, mo


def test_t05(m, brd, c, mp):
    ok = True
    target = 6400.0  # 2 revolutions
    log, mo = run(m, brd, c, mp, [(0.0, MOVE, target)], 0.6)
    h = np.array(mo.hist)
    pos, vq, mode = h[:, 1], h[:, 3], h[:, 4]
    dv = np.abs(np.diff(np.concatenate(([0.0], vq))))
    i_done = int(np.argmax(mode == 0))          # period in which the move ended
    done = h[i_done, 0]
    # limits in the integer units of the library: |dv| <= acc per period, the last
    # period of the move may stop from below 2 dec (exact stop on the target)
    ok &= check("T05 trajectory: reaches target, limits kept",
                pos[-1] == target and np.max(np.abs(vq)) <= mo.v_max
                and np.max(dv[:i_done]) <= mo.acc and dv[i_done] <= 2 * mo.dec
                and np.all(np.diff(pos) >= 0),
                f"target reached at {done * 1e3:.0f} ms, v max {np.max(np.abs(h[:, 2])):.0f} usteps/s, "
                f"|dv| max {np.max(dv[:i_done]):.0f} (acc {mo.acc}) Q16/period, last {dv[i_done]:.0f}, monotonic")
    d = load_angle(log, m)
    rotor_usteps = m["Ns"] / 4 * log[-1, 4] / (math.pi / 2) * mp["M"]
    ok &= check("T05 rotor follows the move (2 rev, 5 rev/s, 50 rev/s^2)",
                np.max(np.abs(d)) < 45.0 and abs(rotor_usteps - target) < 0.5 * mp["M"],
                f"max load angle {np.max(np.abs(d)):.2f} deg, final rotor {rotor_usteps:.2f} usteps "
                f"(target {target:.0f}), final current {log[-1, 10]:.1f} A")
    return ok


def test_t06(m, brd, c, mp):
    ok = True
    v = 3200.0  # 1 rev/s
    for TL, expect_slip in ((0.25, False), (0.5, True)):
        log, _ = run(m, brd, c, mp, [(0.0, RUN, v), (0.15, LOAD, TL)], 0.3)
        d = load_angle(log, m)
        slip = np.max(np.abs(d)) > 180.0
        if not expect_slip:
            w = v * 2 * math.pi / (m["Ns"] * mp["M"])
            sel = log[:, 0] >= 0.25
            # angle of the measured current vector ahead of the rotor, and its amplitude
            p = m["Ns"] / 4
            di = np.degrees(np.arctan2(log[sel, 2], log[sel, 1]) - p * log[sel, 4])
            di = (di + 180.0) % 360.0 - 180.0
            amp = float(np.mean(np.hypot(log[sel, 1], log[sel, 2])))
            want = math.degrees(math.asin((TL + m["B"] * w) / (m["Kt"] * amp)))
            mean_i = float(np.mean(di))
            mean_r = float(np.mean(d[sel]))
            ok &= check(f"T06 load {TL} N*m (below Kt*I = {m['Kt'] * mp['i_run']:.3f}): synchronous",
                        (not slip) and abs(mean_i - want) < 1.0,
                        f"current-rotor angle {mean_i:.2f} deg (asin(T/(Kt I)) = {want:.2f}), "
                        f"reference-rotor {mean_r:.2f} deg, max {np.max(np.abs(d)):.1f} deg")
        else:
            t_slip = log[np.argmax(np.abs(d) > 180.0), 0]
            ok &= check(f"T06 load {TL} N*m (above Kt*I): loss of synchronism detected",
                        slip, f"|load angle| > 180 deg at {t_slip * 1e3:.1f} ms")
    return ok


def test_t07(m, brd, c, mp):
    v = 12800.0  # 4 rev/s
    mp2 = dict(mp, v_max=v)
    log, mo = run(m, brd, c, mp2, [(0.0, RUN, v), (0.15, RUN, -v)], 0.4)
    h = np.array(mo.hist)
    dv = np.max(np.abs(np.diff(h[:, 3])))
    d = load_angle(log, m)
    w_end = log[-1, 3] * 60 / (2 * math.pi)
    return check("T07 reversal +4 -> -4 rev/s: synchronous, limits kept",
                 np.max(np.abs(d)) < 90.0 and dv <= max(mo.acc, mo.dec)
                 and abs(w_end + 240.0) < 12.0,
                 f"max load angle {np.max(np.abs(d)):.2f} deg, |dv| max {dv:.0f} Q16/period "
                 f"(limit {mo.dec}), final speed {w_end:.1f} rpm (expected -240)")


def test_t09(m, brd, c, mp):
    ok = True
    v = 25600.0  # 8 rev/s
    mp2 = dict(mp, v_max=v, acc=320000.0, dec=320000.0)
    res = {}
    for topo in ("full", "shared"):
        b = dict(brd, topology=topo)
        log, _ = run(m, b, c, mp2, [(0.0, RUN, v)], 0.15)
        sel = log[:, 0] >= 0.1
        amp = float(np.mean(np.hypot(log[sel, 1], log[sel, 2])))
        kmin = float(np.min(log[sel, 9]))
        d = load_angle(log, m)
        res[topo] = (amp, kmin, np.max(np.abs(d)))
    a_f, k_f, d_f = res["full"]
    a_s, k_s, d_s = res["shared"]
    ok &= check("T09 8 rev/s: shared leg reaches the voltage limit earlier than full bridge",
                k_s < k_f and a_s < a_f - 0.2 and d_f < 90.0 and d_s < 90.0,
                f"full: {a_f:.3f} A, min scale {k_f:.3f}, max load angle {d_f:.1f} deg; "
                f"shared: {a_s:.3f} A, min scale {k_s:.3f}, max load angle {d_s:.1f} deg")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()
    m, brd, c, mp = MOTOR, BOARD, CTRL, MOTION
    ok = test_t05(m, brd, c, mp)
    ok &= test_t06(m, brd, c, mp)
    ok &= test_t07(m, brd, c, mp)
    ok &= test_t09(m, brd, c, mp)
    if args.write:
        log, _ = run(m, brd, c, mp, [(0.0, MOVE, 160.0)], 0.05)
        path = os.path.join(ROOT, "tests", "reference", "x03_move_160_usteps.csv")
        with open(path, "w", newline="\n") as f:
            f.write("# X03 reference: motion generator + current loop, shared leg, move to 160 "
                    "usteps (M = 16), motion_default. Generated by tools/ref_motion.py.\n")
            f.write("t_s,iA_A,iB_A,w_rad_s,theta_rad,phi_rad,I_A\n")
            for r in log[::10]:
                f.write(f"{r[0]:.8f},{r[1]:.12e},{r[2]:.12e},{r[3]:.12e},{r[4]:.12e},"
                        f"{r[11]:.12e},{r[10]:.6f}\n")
        print(f"wrote {os.path.relpath(path, ROOT)}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
