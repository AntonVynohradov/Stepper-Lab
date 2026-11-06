// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/lab_settings_file.sci
// Description: File of the settings of the window (per user, outside the repository).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function p = lab_settings_file()
    // p  environment variable SSTEP_LAB_SETTINGS, else SCIHOME/sfs_stepper_lab.json
    p = getenv("SSTEP_LAB_SETTINGS", "");
    if p == "" then
        p = fullfile(SCIHOME, "sfs_stepper_lab.json");
    end
endfunction
