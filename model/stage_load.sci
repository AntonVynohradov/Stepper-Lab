// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stage_load.sci
// Description: Reads a power stage file exported by sfs-stepper-power-spice
//              (python tools/run_tests.py --export), the format of
//              params/stage_esc1_spice.json.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function stg = stage_load(path)
    // path  JSON file
    // stg   the structure of stage_esc1_spice(); an invalid file is an error that names
    //       the file and the field. Fields not used by the model and missing in the file
    //       are %nan.
    if ~isfile(path) then
        error(msprintf("Stage file not found: %s", path));
    end
    r = [];
    if execstr("r = fromJSON(strcat(mgetl(path), """"));", "errcatch") <> 0 then
        error(msprintf("Stage file %s: not valid JSON", path));
    end
    if typeof(r) <> "st" then
        error(msprintf("Stage file %s: not valid JSON", path));
    end
    for f = ["format", "vbus", "fs", "r_leg", "vd", "dead_timer_ns", "dead_eff_ns", "leg_error_table"]
        if ~isfield(r, f) then
            error(msprintf("Stage file %s: the field %s is missing", path, f));
        end
        if (type(r(f)) <> 1) | isempty(r(f)) then
            error(msprintf("Stage file %s: the field %s is not a number", path, f));
        end
    end
    // one finite number each: positive for vbus and fs, not negative for the others
    for f = ["format", "vbus", "fs", "r_leg", "vd", "dead_timer_ns", "dead_eff_ns"]
        v = r(f);
        if (size(v, "*") <> 1) | or(isnan(v)) | or(isinf(v)) then
            error(msprintf("Stage file %s: the field %s is not one finite number", path, f));
        end
        if ((f == "vbus") | (f == "fs")) & (v <= 0) then
            error(msprintf("Stage file %s: the field %s must be positive", path, f));
        end
        if (v < 0) then
            error(msprintf("Stage file %s: the field %s must not be negative", path, f));
        end
    end
    if r.format <> 1 then
        error(msprintf("Stage file %s: the field format is %d, this model reads 1", path, r.format));
    end
    t = r.leg_error_table;
    if (size(t, 2) <> 2) | (size(t, 1) < 1) | (size(t, 1) > 16) then
        error(msprintf("Stage file %s: the field leg_error_table needs 1 to 16 rows [i v]", path));
    end
    if or(isnan(t)) | or(isinf(t)) then
        error(msprintf("Stage file %s: the field leg_error_table holds a value that is not finite", path));
    end
    if or(diff(t(:, 1)) <= 0) then
        error(msprintf("Stage file %s: the field leg_error_table needs a rising current", path));
    end

    stg = struct();
    stg.rds_on = %nan;
    stg.r_leg = r.r_leg;
    stg.vd = r.vd;
    stg.dead_timer_ns = r.dead_timer_ns;
    stg.dead_eff_ns = r.dead_eff_ns;
    stg.shunt_settle_ns = %nan;
    stg.duty_max_sampling = %nan;
    stg.leg_error = t;
    stg.vbus = r.vbus;
    stg.fs = r.fs;
    for f = ["rds_on", "shunt_settle_ns", "duty_max_sampling"]
        if isfield(r, f) then
            stg(f) = r(f);
        end
    end
endfunction
