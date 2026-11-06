// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T08_parameters.sce
// Description: T08: parameter uncertainty. The controller is tuned for the nominal
//              motor; the plant has other R, L, J and a load. The current loop
//              stays stable and the rotor synchronous (twin: tools/ref_ext.py).
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
c.mot = mot;                                    // controller knows the nominal motor

// current step 0 -> 2 A, locked rotor: plant R and L factors
cases_i = [0.9, 0.8; 0.9, 1.2; 1.3, 0.8; 1.3, 1.2];
refp = struct("I0", 0, "I1", 2, "t_step", 1e-3, "phi", 0);
for k = 1:size(cases_i, 1)
    pl = mot;
    pl.R = mot.R * cases_i(k, 1);
    pl.L = mot.L * cases_i(k, 2);
    rec = sim_closed_loop(pl, brd, c, ref_current_step, refp, 6e-3, %t);
    [over, t_rise, t_set, e_ss] = sim_step_metrics(rec, 1e-3, 2, 2);
    sim_failures = sim_failures + sim_check(msprintf("T08 current step, plant R x%g, L x%g", ..
        cases_i(k, 1), cases_i(k, 2)), ..
        (over < 0.10) & (t_set < 2e-3) & (e_ss < 1e-2), ..
        msprintf("overshoot %.2f %%, settling (2 %%) %.3f ms, steady error %.3f %%", ..
                 over * 100, t_set * 1e3, e_ss * 100));
end

// ramp to 4 rev/s in 0.1 s: plant R, L, J factors and load torque
cases_m = [1.0, 1.0, 1.0, 0; 1.3, 1.2, 1.0, 0; 1.0, 1.0, 5.0, 0; 1.3, 0.8, 5.0, 0.15];
for k = 1:size(cases_m, 1)
    pl = mot;
    pl.R = mot.R * cases_m(k, 1);
    pl.L = mot.L * cases_m(k, 2);
    pl.J = mot.J * cases_m(k, 3);
    sp = struct("I", 2, "rev_s", 4, "t_ramp", 0.1, "Ns", mot.Ns, "TL", cases_m(k, 4));
    rec = sim_closed_loop(pl, brd, c, ref_speed_ramp, sp, 0.2);
    d = sim_load_angle(rec, mot);
    sel = rec(:, 1) >= 0.15;
    sim_failures = sim_failures + sim_check(msprintf("T08 ramp to 4 rev/s, plant R x%g, L x%g, J x%g, TL %g N*m", ..
        cases_m(k, 1), cases_m(k, 2), cases_m(k, 3), cases_m(k, 4)), ..
        max(abs(d)) < 90, ..
        msprintf("max load angle %.1f deg, final %.1f deg", max(abs(d)), mean(d(sel))));
end
