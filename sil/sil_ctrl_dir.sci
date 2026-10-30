// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        sil/sil_ctrl_dir.sci
// Description: Folder of the Stepper-C sources (sfs-stepper-ctrl).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function d = sil_ctrl_dir()
    // 1. environment variable SSTEP_CTRL_DIR
    // 2. git submodule ext/sfs-stepper-ctrl
    // 3. sibling folder ../sfs-stepper-ctrl
    global sim_root_g;
    d = getenv("SSTEP_CTRL_DIR", "");
    if d == "" then
        d = fullfile(sim_root_g, "ext", "sfs-stepper-ctrl");
        if ~isfile(fullfile(d, "sstep.h")) then
            d = fullfile(sim_root_g, "..", "sfs-stepper-ctrl");
        end
    end
    if ~isfile(fullfile(d, "sstep.h")) then
        error(msprintf("Stepper-C sources not found (looked for %s). Set SSTEP_CTRL_DIR.", ..
                       fullfile(d, "sstep.h")));
    end
endfunction
