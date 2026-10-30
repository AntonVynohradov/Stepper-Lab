// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/ref_speed_ramp.sci
// Description: Current reference: constant amplitude, speed ramps linearly to a
//              constant value (twin of ref_speed in tools/ref_switching.py).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [r, st] = ref_speed_ramp(t, p, st)
    // p.I       current amplitude [A]
    // p.rev_s   final speed [rev/s]
    // p.t_ramp  ramp time [s]
    // p.Ns      full steps per revolution
    // p.TL      load torque [N*m] (optional, default 0)
    // st        state (unused, returned unchanged)
    // r = [I; phi; w_ref; TL]
    w_end = 2 * %pi * p.rev_s;
    if t < p.t_ramp then
        w = w_end * t / p.t_ramp;
        th = w_end * t * t / (2 * p.t_ramp);
    else
        w = w_end;
        th = w_end * p.t_ramp / 2 + w_end * (t - p.t_ramp);
    end
    TL = 0;
    if isfield(p, "TL") then
        TL = p.TL;
    end
    r = [p.I; p.Ns / 4 * th; w; TL];
endfunction
