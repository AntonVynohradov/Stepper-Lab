// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T05_speed_ramp.sce
// Description: T05: move of 2 revolutions with the trapezoidal profile (5 rev/s,
//              50 rev/s^2): limits kept, target reached, rotor follows.
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
target = 6400;                                  // 2 revolutions with M = 16
st = motion_init(mot, brd, mp, [0, 1, target]);
[rec, st] = sim_closed_loop(mot, brd, c, ref_motion, [], 0.6, %f, st);

hv = st.hist;                                   // [t pos v vel_q16 mode] per slow period
dv = abs(diff([0; hv(:, 4)]));
i_done = find(hv(:, 5) == 0, 1);                // period in which the move ended
t_done = hv(i_done, 1);
// limits in the integer units of Stepper-C: |dv| <= acc per period; the last period of
// the move may stop from below 2 dec (exact stop on the target)
sim_failures = sim_failures + sim_check("T05 trajectory: reaches target, limits kept", ..
    (hv($, 2) == target) & (max(abs(hv(:, 4))) <= st.v_max) & ..
    (max(dv(1:i_done - 1)) <= st.acc) & (dv(i_done) <= 2 * st.dec) & and(diff(hv(:, 2)) >= 0), ..
    msprintf("target reached at %.0f ms, v max %.0f usteps/s, |dv| max %.0f (acc %d) Q16/period, last %.0f", ..
             t_done * 1e3, max(abs(hv(:, 3))), max(dv(1:i_done - 1)), st.acc, dv(i_done)));

d = sim_load_angle(rec, mot);
rotor = mot.Ns / 4 * rec($, 5) / (%pi / 2) * mp.M;
sim_failures = sim_failures + sim_check("T05 rotor follows the move (2 rev, 5 rev/s, 50 rev/s^2)", ..
    (max(abs(d)) < 45) & (abs(rotor - target) < 0.5 * mp.M), ..
    msprintf("max load angle %.2f deg, final rotor %.2f usteps (target %d), final current %.1f A", ..
             max(abs(d)), rotor, target, rec($, 11)));
