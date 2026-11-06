// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/ref_current_step.sci
// Description: Reference: current amplitude step at a fixed electrical angle.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [r, st] = ref_current_step(t, p, st)
    // p.I0, p.I1  amplitude before / after the step [A]
    // p.t_step    step time [s]
    // p.phi       electrical angle [rad]
    // st          state (unused, returned unchanged)
    // r = [I; phi; w_ref; TL]
    if t >= p.t_step then
        I = p.I1;
    else
        I = p.I0;
    end
    r = [I; p.phi; 0; 0];
endfunction
