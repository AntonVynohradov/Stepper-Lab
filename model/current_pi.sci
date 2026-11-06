// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/current_pi.sci
// Description: PI current controller of both phases (one fast loop period).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [vu, e] = current_pi(iref, imeas, integ, ff, g)
    // iref, imeas  reference and measured currents [iA; iB] [A]
    // integ        integrator states [V]
    // ff           feed-forward voltages [V]
    // g            gains (current_pi_gains)
    // vu           unlimited voltage request [V], e current error [A]
    e = iref - imeas;
    vu = g.Kp * e + integ + ff;
endfunction
