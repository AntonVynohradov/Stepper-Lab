// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T06_load_step.sce
// Description: T06: load torque step at 1 rev/s: below Kt*I the motor stays
//              synchronous with the static load angle; above it synchronism is lost.
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
v = 3200;                                       // 1 rev/s
p = mot.Ns / 4;

// 0.25 N*m < Kt*I: synchronous, current vector ahead of the rotor by asin(T / (Kt I))
TL = 0.25;
st = motion_init(mot, brd, mp, [0, 2, v; 0.15, 4, TL]);
rec = sim_closed_loop(mot, brd, c, ref_motion, [], 0.3, %f, st);
d = sim_load_angle(rec, mot);
sel = rec(:, 1) >= 0.25;
di = (atan(rec(sel, 3), rec(sel, 2)) - p * rec(sel, 5)) * 180 / %pi;
di = pmodulo(di + 180, 360) - 180;
amp = mean(sqrt(rec(sel, 2).^2 + rec(sel, 3).^2));
w = v * 2 * %pi / (mot.Ns * mp.M);
want = asin((TL + mot.B * w) / (mot.Kt * amp)) * 180 / %pi;
sim_failures = sim_failures + sim_check(msprintf("T06 load %.2f N*m (below Kt*I = %.3f): synchronous", TL, mot.Kt * mp.i_run), ..
    (max(abs(d)) <= 180) & (abs(mean(di) - want) < 1), ..
    msprintf("current-rotor angle %.2f deg (asin(T/(Kt I)) = %.2f), reference-rotor %.2f deg, max %.1f deg", ..
             mean(di), want, mean(d(sel)), max(abs(d))));

// 0.5 N*m > Kt*I: loss of synchronism (|load angle| > 180 deg)
TL = 0.5;
st = motion_init(mot, brd, mp, [0, 2, v; 0.15, 4, TL]);
rec = sim_closed_loop(mot, brd, c, ref_motion, [], 0.3, %f, st);
d = sim_load_angle(rec, mot);
idx = find(abs(d) > 180, 1);
if isempty(idx) then
    detail = "no slip";
else
    detail = msprintf("|load angle| > 180 deg at %.1f ms", rec(idx, 1) * 1e3);
end
sim_failures = sim_failures + sim_check(msprintf("T06 load %.2f N*m (above Kt*I): loss of synchronism detected", TL), ..
    ~isempty(idx), detail);
