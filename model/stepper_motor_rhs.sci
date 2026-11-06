// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stepper_motor_rhs.sci
// Description: State equations for ode(): dx/dt = f(t, x) with a source function.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function dx = stepper_motor_rhs(t, x, mot, src, srcp, locked)
    // t       time [s]
    // x       state [iA; iB; w; theta]
    // mot     motor parameters
    // src     source function: u = src(t, x, srcp), u = [vA; vB; TL]
    //         (phase voltages [V] and load torque [N*m])
    // srcp    parameters of the source function
    // locked  %t: rotor locked (w and theta stay constant)

    dx = stepper_motor_f(x, src(t, x, srcp), mot, locked);
endfunction
