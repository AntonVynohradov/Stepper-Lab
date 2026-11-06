// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T12_friction.sce
// Description: T12: dry friction. Unpowered rotor without back-EMF (Kt = 0) spins down:
//              w' = -(B w + Tc) / J, time to w: (J/B) ln((w0 + Tc/B) / (w + Tc/B)).
//              Twin: tools/ref_ext.py.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

mot = motor_17hs19_2004s1();
md = mot;
md.Kt = 0;
md.Tc = 0.01;                                   // N*m, assumption
w0 = 50;
w1 = 0.01 * w0;
a = md.Tc / md.B;
t_stop = md.J / md.B * log((w0 + a) / a);
t_1 = md.J / md.B * log((w0 + a) / (w1 + a));
t = linspace(0, 1.5 * t_stop, 6001);
srcp = struct("vA", 0, "vB", 0, "TL", 0);
x = stepper_simulate(md, src_dc, srcp, t, [0; 0; w0; 0]);
k = find(x(3, :) < w1, 1);
t_meas = t(k - 1) + (x(3, k - 1) - w1) / (x(3, k - 1) - x(3, k)) * (t(k) - t(k - 1));
sim_failures = sim_failures + sim_check("T12 dry friction: spin-down time (J/B) ln((w0 + Tc/B) / (w + Tc/B))", ..
    (abs(t_meas / t_1 - 1) < 0.01) & (abs(x(3, $)) < 0.1), ..
    msprintf("%.2f ms to 1 %% of %.0f rad/s (theory %.2f ms; stop %.2f ms), final %.4f rad/s", ..
             t_meas * 1e3, w0, t_1 * 1e3, t_stop * 1e3, x(3, $)));
