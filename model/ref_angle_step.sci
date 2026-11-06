// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/ref_angle_step.sci
// Description: Current reference: constant amplitude, electrical angle step 0 -> phi1
//              (rotor natural frequency, test T11).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [r, st] = ref_angle_step(t, p, st)
    // p.I       current amplitude [A]
    // p.phi1    electrical angle after the step [rad]
    // p.t_step  step time [s]
    // st        state (unused, returned unchanged)
    // r = [I; phi; w_ref; TL]
    if t >= p.t_step then
        phi = p.phi1;
    else
        phi = 0;
    end
    r = [p.I; phi; 0; 0];
endfunction
