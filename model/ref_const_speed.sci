// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/ref_const_speed.sci
// Description: Reference: constant current amplitude, electrical angle turning at a
//              constant electrical frequency (microstepping at constant speed).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [r, st] = ref_const_speed(t, p, st)
    // p.I     current amplitude [A]
    // p.f_el  electrical frequency [Hz] (one period = 4 full steps)
    // p.Ns    full steps per revolution (for the mechanical reference speed)
    // p.TL    load torque [N*m]
    // st      state (unused, returned unchanged)
    // r = [I; phi; w_ref; TL]
    w_el = 2 * %pi * p.f_el;
    r = [p.I; w_el * t; w_el / (p.Ns / 4); p.TL];
endfunction
