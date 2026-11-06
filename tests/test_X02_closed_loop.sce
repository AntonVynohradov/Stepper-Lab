// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_X02_closed_loop.sce
// Description: X02: cross-check of the discrete current loop with the Python twin
//              (tools/ref_ctrl.py): shared-leg bridge, current step, locked rotor.
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
rec = sim_closed_loop(mot, brd, c, ref_current_step, refp, 3e-3, %t);

// Reference: comment line, header line, then t, iA, iB, vA, vB
csv_lines = mgetl(fullfile(sim_root, "tests", "reference", "x02_current_step_shared.csv"));
ref = csvTextScan(csv_lines(3:$), ",", ".", "double");

ok_len = size(ref, 1) == size(rec, 1);
if ok_len then
    di = max(abs(rec(:, 2:3) - ref(:, 2:3)));
    dv = max(abs(rec(:, 8:9) - ref(:, 4:5)));
else
    di = %inf;
    dv = %inf;
end
sim_failures = sim_failures + sim_check("X02 Scilab vs Python twin (closed loop)", ..
    ok_len & (di < 1e-7) & (dv < 1e-6), ..
    msprintf("%d samples, max diff i %.1e A, v %.1e V", size(rec, 1), di, dv));
