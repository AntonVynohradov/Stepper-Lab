// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T04_current_step.sce
// Description: T04: current amplitude step 0 -> 2 A, locked rotor, both topologies:
//              overshoot, rise and settling time, steady state error.
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
refp = struct("I0", 0, "I1", 2, "t_step", 1e-3, "phi", 0);
topos = ["full", "shared"];
for n_t = 1:2
    brd.topology = topos(n_t);
    rec = sim_closed_loop(mot, brd, c, ref_current_step, refp, 6e-3, %t);
    [over, t_rise, t_set, e_ss] = sim_step_metrics(rec, 1e-3, 2, 2);
    sim_failures = sim_failures + sim_check("T04 " + topos(n_t) + ": current step 0 -> 2 A", ..
        (over < 0.05) & (t_set < 1e-3) & (e_ss < 1e-3), ..
        msprintf("overshoot %.2f %%, rise (90 %%) %.3f ms, settling (2 %%) %.3f ms, steady error %.3f %%", ..
                 over * 100, t_rise * 1e3, t_set * 1e3, e_ss * 100));
end
