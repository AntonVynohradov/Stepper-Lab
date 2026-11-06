// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T01_resistance.sce
// Description: T01: constant voltage on phase A: the current settles at V/R.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

mot = motor_17hs19_2004s1();
V = mot.R * mot.I_rated;          // voltage for the rated current
tau = mot.L / mot.R;
t = linspace(0, 20 * tau, 2001);
srcp = struct("vA", V, "vB", 0, "TL", 0);

// Rotor locked
x = stepper_simulate(mot, src_dc, srcp, t, zeros(4, 1), %t);
err = abs(x(1, $) - V / mot.R) / (V / mot.R);
sim_failures = sim_failures + sim_check("T01 steady state (locked)", err < 1e-6, ..
    msprintf("i = %.6f A, rel. error %.1e", x(1, $), err));

// Rotor free: at theta = 0 phase A alone produces no torque, the rotor stays
x = stepper_simulate(mot, src_dc, srcp, t, zeros(4, 1), %f);
sim_failures = sim_failures + sim_check("T01 rotor stays at theta = 0 (free)", ..
    abs(x(4, $)) < 1e-9, msprintf("theta = %.1e rad, i = %.6f A", x(4, $), x(1, $)));
