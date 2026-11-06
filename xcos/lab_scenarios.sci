// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/lab_scenarios.sci
// Description: Scenarios of the stepper_lab window: context lines and source blocks.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function cs = lab_scenarios()
    // cs  list of struct:
    //     name     text of the list in the window
    //     T        simulated time [s]
    //     ctx      lines added to the end of the diagram context
    //     sources  list of list(source label, kind, parameters): kind "step" [t from to],
    //              "sine" [amplitude frequency_Hz], "ramp" [slope start_time]
    //     expect   "ok": no fault, "fault": SSTEP_FAULT_STALL in the first fault
    //     changes  %t: the applied motion command changes during the run
    function s = mk(name, T, ctx, sources, expect, changes)
        s = struct("name", name, "T", T, "ctx", [ctx(:); msprintf("T_end = %g;", T)], ..
                   "sources", sources, "expect", expect, "changes", changes);
    endfunction

    stall = "ctl.stall_time_ms = 50;";
    cs = list();
    cs($ + 1) = mk("Current step 0.5 -> 2 A, rotor locked", 0.03, ["lab_cmd = 0;"; "locked = 1;"], ..
                   list(list("lab_irun", "step", [0.01, 0.5, 2])), "ok", %f);
    cs($ + 1) = mk("Move 2 revolutions", 0.8, ["lab_mode = 2;"; "lab_cmd = 6400;"], list(), "ok", %f);
    cs($ + 1) = mk("Velocity step to 1 rev/s", 0.4, "lab_mode = 1;", ..
                   list(list("lab_cmd", "step", [0.05, 0, 3200])), "ok", %t);
    cs($ + 1) = mk("Velocity sine, 1 rev/s, 2 Hz", 0.6, "lab_mode = 1;", ..
                   list(list("lab_cmd", "sine", [3200, 2])), "ok", %t);
    cs($ + 1) = mk("Velocity ramp to 5 rev/s", 0.5, "lab_mode = 1;", ..
                   list(list("lab_cmd", "ramp", [40000, 0.05])), "ok", %t);
    cs($ + 1) = mk("Load step 0.25 N*m, below the pull-out torque", 0.4, ["lab_mode = 1;"; stall], ..
                   list(list("lab_load", "step", [0.2, 0, 0.25])), "ok", %f);
    cs($ + 1) = mk("Load step 0.5 N*m, above the pull-out torque", 0.4, ["lab_mode = 1;"; stall], ..
                   list(list("lab_load", "step", [0.2, 0, 0.5])), "fault", %f);
    cs($ + 1) = mk("Stall at the hold current, 0.12 N*m", 0.62, ..
                   ["lab_cmd = 0;"; stall; "mp.i_run = 1.5;"; "mp.i_hold = 0.5;"; "mp.hold_delay = 0.5;"; ..
                    "lab_irun = 1.5;"; "lab_load = 0.12;"], list(), "fault", %f);
endfunction
