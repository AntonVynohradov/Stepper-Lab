// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T09_voltage_limit.sce
// Description: T09: 8 rev/s with 24 V: the shared-leg stage (Vbus/sqrt(2)) reaches
//              the voltage limit earlier than two H-bridges; both stay synchronous.
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
v = 25600;                                      // 8 rev/s
mp.v_max = v;
mp.acc = 320000;
mp.dec = 320000;
topos = ["full", "shared"];
res = zeros(2, 3);
for n_t = 1:2
    brd.topology = topos(n_t);
    st = motion_init(mot, brd, mp, [0, 2, v]);
    rec = sim_closed_loop(mot, brd, c, ref_motion, [], 0.15, %f, st);
    sel = rec(:, 1) >= 0.1;
    d = sim_load_angle(rec, mot);
    res(n_t, :) = [mean(sqrt(rec(sel, 2).^2 + rec(sel, 3).^2)), min(rec(sel, 10)), max(abs(d))];
end
sim_failures = sim_failures + sim_check("T09 8 rev/s: shared leg reaches the voltage limit earlier than full bridge", ..
    (res(2, 2) < res(1, 2)) & (res(2, 1) < res(1, 1) - 0.2) & (res(1, 3) < 90) & (res(2, 3) < 90), ..
    msprintf("full: %.3f A, min scale %.3f, max load angle %.1f deg; shared: %.3f A, min scale %.3f, max load angle %.1f deg", ..
             res(1, 1), res(1, 2), res(1, 3), res(2, 1), res(2, 2), res(2, 3)));
