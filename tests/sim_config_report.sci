// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/sim_config_report.sci
// Description: Test helper: prints what a test run uses (library folder, ngspice stage
//              file); a selected stage file that is not valid, or an old build without
//              its sources, counts as a failure, so that a result never comes from another
//              source without saying so.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function nfail = sim_config_report(quiet)
    // quiet  %t: only count (test C01), optional, default %f
    // nfail  number of problems: a selected stage file that is not valid; no Stepper-C
    //        folder with a built library, which the tests would use without its sources
    global sim_root_g;
    [lhs, rhs] = argn(0);
    if rhs < 1 then
        quiet = %f;
    end
    nfail = 0;
    txt = [];
    d = "not found";
    if execstr("d = sil_ctrl_dir();", "errcatch") <> 0 then
        d = "not found";
        built = isfile(fullfile(sim_root_g, "sil", "build", "fixed", "loader.sce")) | ..
                isfile(fullfile(sim_root_g, "sil", "build", "float", "loader.sce"));
        if built then
            txt = [txt; "FAIL  configuration: no Stepper-C folder, the tests would use an old build in sil/build"];
            nfail = 1;
        end
    end
    [path, msg] = stage_source();
    if path == "" then
        path = "default (params/stage_esc1_spice)";
    end
    if msg <> "" then
        txt = [txt; "FAIL  configuration: " + msg];
        nfail = nfail + 1;
    end
    if ~quiet then
        mprintf("Library: %s\nngspice stage: %s\n", d, path);
        for k = 1:size(txt, "*")
            mprintf("%s\n", txt(k));
        end
    end
endfunction
