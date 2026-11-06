// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/blocks/STEPPER_PLANT_IO.sci
// Description: Xcos block: power stage + motor + ADC with the load torque as a signal.
//              Computational function sstep_xcos_plant_io (model: sil/c/sstep_plant.c).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [x, y, typ] = STEPPER_PLANT_IO(job, arg1, arg2)
    // Inputs:  1 [compare(4) enabled], 2 load torque (N*m)
    // Outputs: 1 ADC sample [i_a i_b vbus] (counts), 2 the 8 signals of STEPPER_PLANT, then
    //          the load torque
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
            [ok, rpar, exprs] = scicos_getvalue("Power stage + stepper motor + ADC", ..
                "Parameter vector: sil_plant_rpar(mot, brd, [0, 0], locked, 4)", ..
                list("vec", -1), exprs);
            if ~ok then
                break;
            end
            if size(rpar, "*") < 27 then
                message("Use sil_plant_rpar(...) to build the parameter vector.");
                ok = %f;
            end
            if ok then
                model.rpar = rpar(:);
                model.dstate = zeros(13, 1);
                graphics.exprs = exprs;
                x.graphics = graphics;
                x.model = model;
                break;
            end
        end
    case "define" then
        model = scicos_model();
        model.sim = list("sstep_xcos_plant_io", 4);
        model.in = [5; 1];
        model.in2 = [1; 1];
        model.intyp = [1; 1];
        model.out = [3; 9];
        model.out2 = [1; 1];
        model.outtyp = [1; 1];
        model.evtin = 1;
        model.dstate = zeros(13, 1);
        model.rpar = [];
        model.blocktype = "d";
        model.dep_ut = [%f %f];
        exprs = "sil_plant_rpar(mot, brd, [0, 0], locked, 4)";
        x = standard_define([6 4], model, exprs, []);
        x.graphics.style = "blockWithLabel;displayedLabel=Power stage + motor + ADC";
    end
endfunction
