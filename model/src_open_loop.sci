// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/src_open_loop.sci
// Description: Source: voltage vector rotating at a constant electrical frequency
//              (open loop, no current control).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function u = src_open_loop(t, x, p)
    // p.V     voltage amplitude [V]
    // p.f_el  electrical frequency [Hz]; positive turns the rotor forward
    // p.TL    load torque [N*m]
    //
    // vA = V cos(2 pi f_el t), vB = V sin(2 pi f_el t)
    ph = 2 * %pi * p.f_el * t;
    u = [p.V * cos(ph); p.V * sin(ph); p.TL];
endfunction
