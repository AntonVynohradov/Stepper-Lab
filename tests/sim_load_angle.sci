// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/sim_load_angle.sci
// Description: Test helper: load angle phi - p theta in electrical degrees
//              (positive = rotor behind the current reference).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function d = sim_load_angle(rec, mot)
    d = (rec(:, 12) - mot.Ns / 4 * rec(:, 5)) * 180 / %pi;
endfunction
