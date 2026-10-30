// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T03_microstep.sce
// Description: T03: microstepping at constant slow speed (2 A, 10 Hz electrical =
//              0.2 rev/s): phase A leads B by 90 deg, amplitude tracked, rotor follows.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

mot = motor_17hs19_2004s1();
brd = board_bg431b_esc1();
c = ctrl_default();
f_el = 10;
refp = struct("I", 2, "f_el", f_el, "Ns", mot.Ns, "TL", 0);
rec = sim_closed_loop(mot, brd, c, ref_const_speed, refp, 0.3);

sel = rec(:, 1) >= 0.1;                  // after the start transient
t = rec(sel, 1);
w = 2 * %pi * f_el;
basis = [cos(w * t), sin(w * t)];
cA = basis \ rec(sel, 2);                // i = c1 cos(wt) + c2 sin(wt)
cB = basis \ rec(sel, 3);
ampA = norm(cA);
ampB = norm(cB);
shift = (atan(-cA(2), cA(1)) - atan(-cB(2), cB(1))) * 180 / %pi;
shift = pmodulo(shift + 180, 360) - 180;
err = max(sqrt((rec(sel, 2) - rec(sel, 6)).^2 + (rec(sel, 3) - rec(sel, 7)).^2));

sim_failures = sim_failures + sim_check("T03 microstep: A leads B by 90 deg, amplitude 2 A", ..
    (abs(shift - 90) < 0.5) & (abs(ampA - 2) < 0.01) & (abs(ampB - 2) < 0.01), ..
    msprintf("shift %.3f deg, amplitudes %.4f / %.4f A, max tracking error %.2f mA", ..
             shift, ampA, ampB, err * 1e3));

p = mot.Ns / 4;
lag = (w * rec($, 1) - p * rec($, 5)) * 180 / %pi;
sim_failures = sim_failures + sim_check("T03 rotor follows the field", abs(lag) < 20, ..
    msprintf("rotor %.4f rad, electrical lag %.2f deg", rec($, 5), lag));
