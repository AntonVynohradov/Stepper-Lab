// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/sim_stall_flags.sci
// Description: Names of the fault flags of Stepper-C.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function txt = sim_stall_flags(flags)
    // flags  fault flags, txt  their names, separated by a blank
    names = ["OVERCURRENT", "OVERVOLTAGE", "UNDERVOLTAGE", "HAL", "EXTERNAL", "STALL"];
    txt = "";
    for b = 0:5
        if bitand(flags, 2^b) <> 0 then
            txt = txt + " " + names(b + 1);
        end
    end
    txt = stripblanks(txt);
endfunction
