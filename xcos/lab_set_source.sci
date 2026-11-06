// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/lab_set_source.sci
// Description: Replaces a source of the Inputs superblock of stepper_lab (lab_cmd, lab_irun,
//              lab_load) by another block with one output, keeping its link.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function scs_m = lab_set_source(scs_m, name, block)
    // name   label of the source: "lab_cmd", "lab_irun" or "lab_load"
    // block  the new block, for example GENSIN_f("define") with its exprs set
    a = [];
    for k = 1:size(scs_m.objs)
        if typeof(scs_m.objs(k)) == "Block" then
            if scs_m.objs(k).model.label == "Inputs" then
                a = k;
            end
        end
    end
    if isempty(a) then
        error("lab_set_source: no Inputs superblock");
    end
    inner = scs_m.objs(a).model.rpar;
    for k = 1:size(inner.objs)
        if typeof(inner.objs(k)) == "Block" then
            if inner.objs(k).model.label == name then
                block.graphics.orig = inner.objs(k).graphics.orig;
                block.graphics.pout = inner.objs(k).graphics.pout;
                block.model.label = name;
                inner.objs(k) = block;
                scs_m.objs(a).model.rpar = inner;
                return;
            end
        end
    end
    error("lab_set_source: no source " + name);
endfunction
