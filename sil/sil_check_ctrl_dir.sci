// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        sil/sil_check_ctrl_dir.sci
// Description: Checks that a folder holds the Stepper-C sources.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function msg = sil_check_ctrl_dir(d)
    // d    folder
    // msg  "" when valid, else the first missing file
    msg = "";
    files = [fullfile(d, "sstep.h"), fullfile(d, "inc", "sstep_ctrl.h"), fullfile(d, "src", "sstep_ctrl.c")];
    for k = 1:size(files, "*")
        if ~isfile(files(k)) then
            msg = "Missing " + files(k);
            return;
        end
    end
endfunction
