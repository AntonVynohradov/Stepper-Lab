// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T00_conventions.sce
// Description: T00: sign conventions of the model: power balance and stable
//              rotor position for a given current vector.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

mot = motor_17hs19_2004s1();
mot.Td = 0;   // the detent torque does no electrical work

// Power balance: eA iA + eB iB = Te w for any state
rand("seed", 1);
worst = 0;
for n = 1:1000
    x = [6 * rand() - 3; 6 * rand() - 3; 1000 * rand() - 500; 20 * rand() - 10];
    [Te, eA, eB] = stepper_motor_em(x, mot);
    worst = max(worst, abs(eA * x(1) + eB * x(2) - Te * x(3)));
end
sim_failures = sim_failures + sim_check("T00 power balance", worst < 1e-9, ..
    msprintf("max |Pe - Pm| = %.2e W", worst));

// Current vector iA = cos(phi), iB = sin(phi): zero torque at p theta = phi and a
// restoring (negative) torque when the rotor is ahead
p = mot.Ns / 4;
phi = 0.7;
Te0 = stepper_motor_em([cos(phi); sin(phi); 0; phi / p], mot);
Te1 = stepper_motor_em([cos(phi); sin(phi); 0; phi / p + 1e-6], mot);
sim_failures = sim_failures + sim_check("T00 equilibrium", (abs(Te0) < 1e-12) & (Te1 < 0), ..
    msprintf("Te(eq) = %.1e, restoring torque %.2e N*m", Te0, Te1));
