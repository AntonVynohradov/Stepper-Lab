// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_X03_motion.sce
// Description: X03: cross-check of motion generator + current loop with the Python
//              twin (tools/ref_motion.py): move to 160 microsteps.
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
st = motion_init(mot, brd, mp, [0, 1, 160]);
rec = sim_closed_loop(mot, brd, c, ref_motion, [], 0.05, %f, st);
rec = rec(1:10:$, :);                           // the reference keeps every 10th row

// Reference: comment line, header line, then t, iA, iB, w, theta, phi, I
csv_lines = mgetl(fullfile(sim_root, "tests", "reference", "x03_move_160_usteps.csv"));
ref = csvTextScan(csv_lines(3:$), ",", ".", "double");

ok_len = size(ref, 1) == size(rec, 1);
if ok_len then
    di = max(abs(rec(:, 2:3) - ref(:, 2:3)));
    dw = max(abs(rec(:, 4) - ref(:, 4)));
    dth = max(abs(rec(:, 5) - ref(:, 5)));
    dphi = max(abs(rec(:, 12) - ref(:, 6)));
    dI = max(abs(rec(:, 11) - ref(:, 7)));
else
    di = %inf; dw = %inf; dth = %inf; dphi = %inf; dI = %inf;
end
sim_failures = sim_failures + sim_check("X03 Scilab vs Python twin (motion)", ..
    ok_len & (di < 1e-7) & (dw < 1e-5) & (dth < 1e-8) & (dphi < 1e-9) & (dI < 1e-9), ..
    msprintf("%d samples, max diff i %.1e A, w %.1e rad/s, theta %.1e rad, phi %.1e rad", ..
             size(rec, 1), di, dw, dth, dphi));
