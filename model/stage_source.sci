// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stage_source.sci
// Description: The active power stage file: environment variable SSTEP_STAGE_FILE, then
//              the file saved by the window, then the default. Keeps the parsed file while
//              its path, size and time do not change (every board call asks for it).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [path, msg, stg] = stage_source()
    // path  the active file, "" = default (stage_esc1_spice)
    // msg   "" or why the selected file is not used
    // stg   the stage of the file, [] for the default
    global sim_stage_cache;
    msg = "";
    stg = [];
    path = getenv("SSTEP_STAGE_FILE", "");
    if path == "" then
        path = lab_settings_load().stage_file;
    end
    if path == "" then
        return;
    end
    info = fileinfo(path);
    key = [];
    if ~isempty(info) then
        key = info([1, 6]);
    end
    hit = %f;
    if ~isempty(sim_stage_cache) & ~isempty(key) then
        hit = (sim_stage_cache.path == path) & and(sim_stage_cache.key == key);
    end
    if hit then
        stg = sim_stage_cache.stg;
    elseif execstr("stg = stage_load(path);", "errcatch") == 0 then
        sim_stage_cache = struct("path", path, "key", key, "stg", stg);
    else
        msg = lasterror();
        path = "";
        stg = [];
    end
endfunction
