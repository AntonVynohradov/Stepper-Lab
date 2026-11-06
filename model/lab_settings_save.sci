// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/lab_settings_save.sci
// Description: Writes the settings of the window (lab_settings_load).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function lab_settings_save(s)
    // s  settings (lab_settings_load)
    mputl(toJSON(s), lab_settings_file());
endfunction
