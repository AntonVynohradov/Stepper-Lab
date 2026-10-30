// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stepper_simulate.sci
// Description: Integrates the motor model over a time vector.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function x = stepper_simulate(mot, src, srcp, t, x0, locked)
    // mot     motor parameters
    // src     source function u = src(t, x, srcp), see stepper_motor_rhs
    // srcp    source parameters
    // t       row vector of output times [s], t(1) is the initial time
    // x0      initial state [iA; iB; w; theta] (optional, default zeros)
    // locked  %t to lock the rotor (optional, default %f)
    //
    // x       4 x length(t) matrix of states at the times t
    //
    // Solver: lsoda (switches automatically between stiff and non-stiff methods),
    // rtol 1e-8, atol 1e-10. This is the averaged model: the phase voltages are
    // continuous, there is no PWM switching.

    [lhs, rhs] = argn(0);
    if rhs < 5 then
        x0 = zeros(4, 1);
    end
    if rhs < 6 then
        locked = %f;
    end

    rtol = 1e-8;
    atol = 1e-10;
    x = ode(x0(:), t(1), t, rtol, atol, list(stepper_motor_rhs, mot, src, srcp, locked));
endfunction
