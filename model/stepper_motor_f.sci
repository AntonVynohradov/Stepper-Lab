// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stepper_motor_f.sci
// Description: State equations with constant inputs: dx/dt = f(x, u).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function dx = stepper_motor_f(x, u, mot, locked)
    // x       state [iA; iB; w; theta] (A, A, rad/s, rad)
    // u       inputs [vA; vB; TL] (V, V, N*m)
    // mot     motor parameters
    // locked  %t: rotor locked (w and theta stay constant)

    [Te, eA, eB] = stepper_motor_em(x, mot);

    diA = (u(1) - mot.R * x(1) - eA) / mot.L;
    diB = (u(2) - mot.R * x(2) - eB) / mot.L;

    if locked then
        dw  = 0;
        dth = 0;
    else
        // viscous friction and dry friction (smoothed around w = 0)
        Tf  = mot.B * x(3) + mot.Tc * tanh(x(3) / mot.w_c);
        dw  = (Te - u(3) - Tf) / mot.J;
        dth = x(3);
    end

    dx = [diA; diB; dw; dth];
endfunction
