// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T07_reversal.sce
// Description: T07: direction change +4 -> -4 rev/s with the deceleration limit:
//              synchronous, limits kept, final speed reached.
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
mp = motion_default();
v = 12800;                                      // 4 rev/s
mp.v_max = v;
st = motion_init(mot, brd, mp, [0, 2, v; 0.15, 2, -v]);
[rec, st] = sim_closed_loop(mot, brd, c, ref_motion, [], 0.4, %f, st);
dv = max(abs(diff(st.hist(:, 4))));             // Q16 per period
d = sim_load_angle(rec, mot);
w_end = rec($, 4) * 60 / (2 * %pi);
sim_failures = sim_failures + sim_check("T07 reversal +4 -> -4 rev/s: synchronous, limits kept", ..
    (max(abs(d)) < 90) & (dv <= max(st.acc, st.dec)) & (abs(w_end + 240) < 12), ..
    msprintf("max load angle %.2f deg, |dv| max %.0f Q16/period (limit %d), final speed %.1f rpm (expected -240)", ..
             max(abs(d)), dv, st.dec, w_end));
