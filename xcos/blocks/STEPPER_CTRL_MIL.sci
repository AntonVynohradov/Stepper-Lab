// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/blocks/STEPPER_CTRL_MIL.sci
// Description: Xcos block: the Scilab prototype of the controller (model in the loop), with
//              the ports of STEPPER_CTRL_IO. Computational function ctrl_mil_block.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [x, y, typ] = STEPPER_CTRL_MIL(job, arg1, arg2)
    // Ports as STEPPER_CTRL_IO. The structures of the context reach the computational
    // function through a global variable (ctrl_mil_config): rpar holds only numbers.
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
            [ok, cfg, mode, exprs] = scicos_getvalue("Scilab prototype of the controller", ..
                ["Configuration: ctrl_mil_config(mot, brd, ctl, mp, cmds)";
                 "Mode: 0 command table, 1 velocity input, 2 position input"], ..
                list("vec", 1, "vec", 1), exprs);
            if ~ok then
                break;
            end
            mess = ctrl_mil_check();
            if mess == "" & and(mode <> [0, 1, 2]) then
                mess = "The mode is 0, 1 or 2.";
            end
            if mess <> "" then
                message(mess);
                ok = %f;
            end
            if ok then
                model.ipar = mode;
                graphics.exprs = exprs;
                x.graphics = graphics;
                x.model = model;
                break;
            end
        end
    case "define" then
        model = scicos_model();
        model.sim = list("ctrl_mil_block", 5);
        model.in = [3; 1; 1];
        model.in2 = [1; 1; 1];
        model.intyp = [1; 1; 1];
        model.out = [5; 15];
        model.out2 = [1; 1];
        model.outtyp = [1; 1];
        model.evtin = 1;
        model.ipar = 1;
        model.blocktype = "d";
        model.dep_ut = [%t %f];
        exprs = ["ctrl_mil_config(mot, brd, ctl, mp, cmds)"; "lab_mode"];
        x = standard_define([6 4], model, exprs, []);
        x.graphics.style = "blockWithLabel;displayedLabel=Scilab prototype";
    end
endfunction
