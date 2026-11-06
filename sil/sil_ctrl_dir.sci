// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        sil/sil_ctrl_dir.sci
// Description: Folder of the Stepper-C sources.
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
    // 2. the folder saved by the window (start_lab.sce); not valid any more: a warning
    // 3. git submodule ext/Stepper-C, sibling folder ../Stepper-C (name of the public repository)
    // 4. the same places under the name sfs-stepper-ctrl
    global sim_root_g;
    d = getenv("SSTEP_CTRL_DIR", "");
    if d == "" then
        saved = lab_settings_load().ctrl_dir;
        if saved <> "" then
            msg = sil_check_ctrl_dir(saved);
            if msg == "" then
                d = saved;
            else
                mprintf("WARNING: the saved Stepper-C folder is not valid: %s\n", msg);
            end
        end
    end
    if d == "" then
        places = [fullfile(sim_root_g, "ext", "Stepper-C"); fullfile(sim_root_g, "..", "Stepper-C"); ..
                  fullfile(sim_root_g, "ext", "sfs-stepper-ctrl"); fullfile(sim_root_g, "..", "sfs-stepper-ctrl")];
        d = places($);
        for k = 1:size(places, "*")
            if isfile(fullfile(places(k), "sstep.h")) then
                d = places(k);
                break;
            end
        end
    end
    msg = sil_check_ctrl_dir(d);
    if msg <> "" then
        error(msprintf("Stepper-C sources not found: %s. Select the folder in the window (start_lab.sce) or set SSTEP_CTRL_DIR.", msg));
    end
endfunction
