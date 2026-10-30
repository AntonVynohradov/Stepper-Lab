// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/blocks/STEPPER_CTRL.sci
// Description: Xcos block: the Stepper-C library (fast and slow loop), compiled
//              from sfs-stepper-ctrl. Computational function sstep_xcos_ctrl.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [x, y, typ] = STEPPER_CTRL(job, arg1, arg2)
    // Input:  ADC sample [i_a i_b vbus] (counts)
    // Output: 1 [compare(4) enabled], 2 monitor [phi position velocity state faults
    //         iA_ref iB_ref]
    // Event input: fast loop clock (fs)
    x = [];
    y = [];
    typ = [];
    select job
    case "set" then
        x = arg1;
        graphics = arg1.graphics;
        exprs = graphics.exprs;
        model = arg1.model;
        while %t do
            [ok, rpar, exprs] = scicos_getvalue("Stepper-C library (software in the loop)", ..
                "Configuration vector: sil_ctrl_rpar(mot, brd, ctl, mp, cmds)", ..
                list("vec", -1), exprs);
            if ~ok then
                break;
            end
            mess = sil_check_rpar(rpar);
            if mess <> "" then
                message(mess);
                ok = %f;
            end
            if ok then
                model.rpar = rpar(:);
                graphics.exprs = exprs;
                x.graphics = graphics;
                x.model = model;
                break;
            end
        end
    case "define" then
        model = scicos_model();
        model.sim = list("sstep_xcos_ctrl", 4);
        model.in = 3;
        model.in2 = 1;
        model.intyp = 1;
        model.out = [5; 7];
        model.out2 = [1; 1];
        model.outtyp = [1; 1];
        model.evtin = 1;
        model.rpar = [];
        model.blocktype = "d";
        model.dep_ut = [%t %f];
        exprs = "sil_ctrl_rpar(mot, brd, ctl, mp, cmds)";
        x = standard_define([6 4], model, exprs, []);
        x.graphics.style = "blockWithLabel;displayedLabel=Stepper-C";
    end
endfunction
