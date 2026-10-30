// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/sil_check_rpar.sci
// Description: Checks a Stepper-C parameter vector with the library itself
//              (sstep_init in sstep_xcos_check); empty string when valid.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function msg = sil_check_rpar(rpar)
    msg = "";
    if ~c_link("sstep_xcos_check") then
        return;                         // library not built yet: checked at simulation
    end
    err = call("sstep_xcos_check", rpar(:), 1, "d", size(rpar, "*"), 2, "i", ..
               "out", [1, 1], 3, "i");
    select err
    case 0 then
        msg = "";
    case -1 then
        msg = "Stepper-C: a value is out of range (SSTEP_ERR_PARAM).";
    case -2 then
        msg = "Stepper-C: inconsistent configuration, e.g. run current above the trip level (SSTEP_ERR_CONFIG).";
    case -4 then
        msg = "Stepper-C: topology not compiled in (SSTEP_ERR_UNSUPPORTED).";
    case -100 then
        msg = "Stepper-C: wrong parameter vector, use sil_ctrl_rpar(...).";
    else
        msg = msprintf("Stepper-C: sstep_init returned %d.", err);
    end
endfunction
