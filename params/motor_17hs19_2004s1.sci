// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        params/motor_17hs19_2004s1.sci
// Description: Parameters of the StepperOnline 17HS19-2004S1 (NEMA 17, bipolar,
//              2 A, 1.8 deg). SI units.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function mot = motor_17hs19_2004s1()
    // Returns the motor parameter structure used by every model function.
    //
    // Source: StepperOnline full datasheet 17HS19-2004S1 (2025). Values marked
    // "assumption" or "derived" must be confirmed by measurement (see README).

    mot = struct();
    mot.name    = "17HS19-2004S1";

    // Electrical
    mot.R       = 1.60;    // Ohm, phase resistance at 25 degC, datasheet 1.60 +/-10 %
                           // (older listings give 1.4 Ohm: measure your motor)
    mot.L       = 3.0e-3;  // H, phase inductance at 1 kHz, datasheet 3.0 mH +/-20 %
    mot.I_rated = 2.0;     // A, rated phase current

    // Electromagnetic
    // derived: holding torque 0.59 N*m with both phases at rated current,
    // T = Kt * sqrt(2) * I  ->  Kt = 0.59 / (sqrt(2) * 2) = 0.2086 N*m/A (= V*s/rad).
    // If the datasheet value is for one phase, Kt = 0.295 N*m/A. Measure the back-EMF.
    mot.Kt      = 0.2086;  // N*m/A
    mot.Ns      = 200;     // full steps per revolution (1.8 deg)
    mot.Td      = 0;       // N*m, detent torque amplitude: not in the datasheet
                           // (T11 uses 0.02, about 3 % of the holding torque, assumption;
                           // measure: turn the unpowered shaft with a torque gauge)

    // Mechanical
    mot.J       = 82e-7;   // kg*m^2, rotor inertia 82 g*cm^2 (datasheet), no load
    mot.B       = 1e-5;    // N*m*s/rad, viscous friction: assumption
    mot.Tc      = 0;       // N*m, dry (Coulomb) friction: unknown (T12 uses 0.01)
    mot.w_c     = 0.05;    // rad/s, dry friction smoothed as Tc tanh(w / w_c) (no stiction)
endfunction
