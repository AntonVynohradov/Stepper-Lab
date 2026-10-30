// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        params/motion_default.sci
// Description: Default motion settings (Stepper-C: microsteps, slow loop rate, limits,
//              run and hold current).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function mp = motion_default()
    mp = struct();
    mp.M          = 16;       // microsteps per full step (3200 per revolution)
    mp.f_slow     = 1000;     // Hz, slow loop rate
    mp.v_max      = 16000;    // microsteps/s (5 rev/s)
    mp.acc        = 160000;   // microsteps/s^2 (50 rev/s^2)
    mp.dec        = 160000;   // microsteps/s^2
    mp.i_run      = 2.0;      // A, current amplitude while moving
    mp.i_hold     = 1.0;      // A, current amplitude at standstill
    mp.hold_delay = 0.1;      // s, standstill time before the hold current
endfunction
