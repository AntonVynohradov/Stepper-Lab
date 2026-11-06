// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T02_time_constant.sce
// Description: T02: voltage step, locked rotor: i(t) = V/R (1 - exp(-t/tau)),
//              tau = L/R.
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
tau = mot.L / mot.R;
t = linspace(0, 5 * tau, 5001);
srcp = struct("vA", V, "vB", 0, "TL", 0);

x = stepper_simulate(mot, src_dc, srcp, t, zeros(4, 1), %t);
i_ss = V / mot.R;
i_ana = i_ss * (1 - exp(-t / tau));
e = max(abs(x(1, :) - i_ana)) / i_ss;
t63 = interp1(x(1, :) / i_ss, t, 1 - exp(-1));

sim_failures = sim_failures + sim_check("T02 time constant", ..
    (abs(t63 - tau) / tau < 1e-3) & (e < 1e-6), ..
    msprintf("t63 = %.4f ms (L/R = %.4f ms), max curve error %.1e", ..
             t63 * 1e3, tau * 1e3, e));
