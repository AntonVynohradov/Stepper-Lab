// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stepper_rk4.sci
// Description: One fixed-step Runge-Kutta 4 step of the motor with constant inputs
//              (used by the discrete closed-loop simulation).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function x = stepper_rk4(x, u, mot, h, locked)
    // x  state, u = [vA; vB; TL] held constant over the step, h step [s]
    k1 = stepper_motor_f(x, u, mot, locked);
    k2 = stepper_motor_f(x + h / 2 * k1, u, mot, locked);
    k3 = stepper_motor_f(x + h / 2 * k2, u, mot, locked);
    k4 = stepper_motor_f(x + h * k3, u, mot, locked);
    x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
endfunction
