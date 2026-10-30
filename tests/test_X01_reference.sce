// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_X01_reference.sce
// Description: X01: cross-check with the Python twin (tools/ref_model.py):
//              open-loop rotating voltage, 5 Hz electrical, from rest.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

mot = motor_17hs19_2004s1();
V = mot.R * mot.I_rated;

// Reference: comment line, header line, then t, iA, iB, w, theta
csv_lines = mgetl(fullfile(sim_root, "tests", "reference", "x01_open_loop_5hz.csv"));
ref = csvTextScan(csv_lines(3:$), ",", ".", "double");
t = ref(:, 1)';
xref = ref(:, 2:5)';

srcp = struct("V", V, "f_el", 5, "TL", 0);
x = stepper_simulate(mot, src_open_loop, srcp, t, zeros(4, 1), %f);

d = max(abs(x - xref), "c");      // largest difference of each state
tol = [1e-4; 1e-4; 1e-3; 1e-5];   // A, A, rad/s, rad
sim_failures = sim_failures + sim_check("X01 Scilab vs Python twin", and(d <= tol), ..
    msprintf("max diff iA %.1e A, iB %.1e A, w %.1e rad/s, theta %.1e rad", ..
             d(1), d(2), d(3), d(4)));
