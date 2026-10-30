// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/current_pi_gains.sci
// Description: Current controller gains from the motor parameters
//              (Stepper-C computes the same in sstep_init()).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function g = current_pi_gains(mot, c, fs)
    // Pole-zero cancellation of the R-L phase: Kp = L wc, Ki = R wc, so that the
    // closed loop is first order with bandwidth wc (without delay).
    // Anti-windup back-calculation gain Kb = Ts / Tt with Tt = Ti = L / R.
    wc = 2 * %pi * c.bw_hz;
    g = struct();
    g.Ts = 1 / fs;
    g.Kp = mot.L * wc;
    g.Ki = mot.R * wc;
    g.Kb = g.Ts * mot.R / mot.L;
endfunction
