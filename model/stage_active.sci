// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stage_active.sci
// Description: The power stage in use: the selected ngspice stage file or the default
//              (stage_source). A selected file that is not valid gives the default and one
//              warning per message.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function stg = stage_active()
    // stg  the structure of stage_esc1_spice()
    global sim_stage_warned;
    [path, msg, stg] = stage_source();
    if path == "" then
        stg = stage_esc1_spice();
    end
    if msg == "" then
        sim_stage_warned = [];
    elseif isempty(sim_stage_warned) then
        sim_stage_warned = msg;
        mprintf("WARNING: default power stage in use. %s\n", msg);
    elseif sim_stage_warned <> msg then
        sim_stage_warned = msg;
        mprintf("WARNING: default power stage in use. %s\n", msg);
    end
endfunction
