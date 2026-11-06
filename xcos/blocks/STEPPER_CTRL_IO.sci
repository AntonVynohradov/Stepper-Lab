// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/blocks/STEPPER_CTRL_IO.sci
// Description: Xcos block: the Stepper-C library with signal inputs for the motion command
//              and the run current. Computational function sstep_xcos_ctrl_io.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [x, y, typ] = STEPPER_CTRL_IO(job, arg1, arg2)
    // Inputs:  1 ADC sample [i_a i_b vbus] (counts), 2 motion command (microsteps/s in the
    //          velocity mode, microsteps in the position mode, ignored with cmds),
    //          3 run current (A)
    // Outputs: 1 [compare(4) enabled], 2 the 12 monitor values of STEPPER_CTRL, then applied
    //          command, applied run current (A), input limit flag
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
            [ok, rpar, mode, exprs] = scicos_getvalue("Stepper-C library with signal inputs", ..
                ["Configuration vector: sil_ctrl_rpar(mot, brd, ctl, mp, cmds)";
                 "Mode: 0 command table, 1 velocity input, 2 position input"], ..
                list("vec", -1, "vec", 1), exprs);
            if ~ok then
                break;
            end
            mess = sil_check_rpar(rpar);
            if mess == "" & and(mode <> [0, 1, 2]) then
                mess = "The mode is 0, 1 or 2.";
            end
            if mess <> "" then
                message(mess);
                ok = %f;
            end
            if ok then
                model.rpar = rpar(:);
                model.ipar = mode;
                graphics.exprs = exprs;
                x.graphics = graphics;
                x.model = model;
                break;
            end
        end
    case "define" then
        model = scicos_model();
        model.sim = list("sstep_xcos_ctrl_io", 4);
        model.in = [3; 1; 1];
        model.in2 = [1; 1; 1];
        model.intyp = [1; 1; 1];
        model.out = [5; 15];
        model.out2 = [1; 1];
        model.outtyp = [1; 1];
        model.evtin = 1;
        model.rpar = [];
        model.ipar = 1;
        model.blocktype = "d";
        model.dep_ut = [%t %f];
        exprs = ["sil_ctrl_rpar(mot, brd, ctl, mp, cmds)"; "lab_mode"];
        x = standard_define([6 4], model, exprs, []);
        x.graphics.style = "blockWithLabel;displayedLabel=Stepper-C";
    end
endfunction
