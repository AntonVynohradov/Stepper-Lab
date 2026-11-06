// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/lab_settings_load.sci
// Description: Reads the settings of the window; a missing or damaged file or an invalid
//              value gives the default.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function s = lab_settings_load()
    // s  ctrl_dir, stage_file ("" = not selected), backend "FIXED" or "FLOAT",
    //    model "SIL" or "MIL", scenario (index in lab_scenarios)
    s = struct("ctrl_dir", "", "stage_file", "", "backend", "FIXED", "model", "SIL", "scenario", 1);
    p = lab_settings_file();
    if ~isfile(p) then
        return;
    end
    r = [];
    if execstr("r = fromJSON(strcat(mgetl(p), """"));", "errcatch") <> 0 then
        return;
    end
    if typeof(r) <> "st" then
        return;
    end
    if isfield(r, "ctrl_dir") then
        if type(r.ctrl_dir) == 10 then
            s.ctrl_dir = r.ctrl_dir;
        end
    end
    if isfield(r, "stage_file") then
        if type(r.stage_file) == 10 then
            s.stage_file = r.stage_file;
        end
    end
    if isfield(r, "backend") then
        if type(r.backend) == 10 then
            if or(r.backend == ["FIXED", "FLOAT"]) then
                s.backend = r.backend;
            end
        end
    end
    if isfield(r, "model") then
        if type(r.model) == 10 then
            if or(r.model == ["SIL", "MIL"]) then
                s.model = r.model;
            end
        end
    end
    if isfield(r, "scenario") then
        if type(r.scenario) == 1 then
            if r.scenario >= 1 then
                s.scenario = r.scenario;
            end
        end
    end
endfunction
