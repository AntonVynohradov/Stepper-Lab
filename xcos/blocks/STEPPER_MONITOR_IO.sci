// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/blocks/STEPPER_MONITOR_IO.sci
// Description: Xcos block: signals of the stepper_lab scopes and of the variable sil.
//              Computational function sstep_xcos_monitor_io.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [x, y, typ] = STEPPER_MONITOR_IO(job, arg1, arg2)
    // Inputs:  1 plant (9, STEPPER_PLANT_IO), 2 controller (15, STEPPER_CTRL_IO)
    // Outputs: 1 currents [iA iB iA_ref iB_ref] (A), 2 speed (rpm),
    //          3 positions [commanded rotor] (microsteps), 4 load angle (electrical deg),
    //          5 [state faults stall_level], 6 all values (23) for the variable sil
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
            [ok, rpar, exprs] = scicos_getvalue("Monitor", ..
                "[full steps per revolution, microsteps]", list("vec", 2), exprs);
            if ~ok then
                break;
            end
            model.rpar = rpar(:);
            graphics.exprs = exprs;
            x.graphics = graphics;
            x.model = model;
            break;
        end
    case "define" then
        model = scicos_model();
        model.sim = list("sstep_xcos_monitor_io", 4);
        model.in = [9; 15];
        model.in2 = [1; 1];
        model.intyp = [1; 1];
        model.out = [4; 1; 2; 1; 3; 23];
        model.out2 = [1; 1; 1; 1; 1; 1];
        model.outtyp = [1; 1; 1; 1; 1; 1];
        model.rpar = [200; 16];
        model.blocktype = "c";
        model.dep_ut = [%t %f];
        exprs = "[mot.Ns, mp.M]";
        x = standard_define([4 8], model, exprs, []);
        x.graphics.style = "blockWithLabel;displayedLabel=Monitor";
    end
endfunction
