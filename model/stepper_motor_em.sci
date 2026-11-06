// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stepper_motor_em.sci
// Description: Electromagnetic part of the two-phase hybrid stepper model:
//              back-EMF and torque.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [Te, eA, eB] = stepper_motor_em(x, mot)
    // x   state [iA; iB; w; theta] (A, A, rad/s, rad)
    // mot motor parameters (motor_17hs19_2004s1)
    //
    // Te  electromagnetic torque [N*m] including the detent torque
    // eA  back-EMF of phase A [V], eB back-EMF of phase B [V]
    //
    // Conventions (p = Ns/4 electrical periods per mechanical turn):
    //   Te = Kt (-iA sin(p theta) + iB cos(p theta)) - Td sin(4 p theta)
    //   eA = -Kt w sin(p theta),  eB = Kt w cos(p theta)
    // so that eA iA + eB iB = Te w (without detent). With the current vector
    // iA = I cos(phi), iB = I sin(phi) the stable rotor position is p theta = phi:
    // increasing phi turns the rotor in the positive direction.

    p = mot.Ns / 4;
    s = sin(p * x(4));
    c = cos(p * x(4));

    Te = mot.Kt * (-x(1) * s + x(2) * c) - mot.Td * sin(4 * p * x(4));
    eA = -mot.Kt * x(3) * s;
    eB = mot.Kt * x(3) * c;
endfunction
