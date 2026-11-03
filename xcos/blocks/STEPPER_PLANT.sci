// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/blocks/STEPPER_PLANT.sci
// Description: Xcos block: averaged power stage, two-phase hybrid stepper motor
//              (RK4) and ADC. Computational function sstep_xcos_plant.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [x, y, typ] = STEPPER_PLANT(job, arg1, arg2)
    // Input:  [compare(4) enabled] from STEPPER_CTRL
    // Output: 1 ADC sample [i_a i_b vbus] (counts),
    //         2 [iA iB w theta vA vB |iA|max |iB|max] (SI, maxima over the last period)
    // Power stage: brd.stage = "avg" (averaged; brd.leg_error adds the ngspice leg
    // voltage error) or "switching" (dead time, diodes). Model: sil/c/sstep_plant.c.
    // Event input: fast loop clock (fs). The voltage computed in a period is applied in
    // the next one (computation delay of the microcontroller).
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
                "Parameter vector: sil_plant_rpar(mot, brd, loads, locked, 4)", ..
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
        model.sim = list("sstep_xcos_plant", 4);
        model.in = 5;
        model.in2 = 1;
        model.intyp = 1;
        model.out = [3; 8];
        model.out2 = [1; 1];
        model.outtyp = [1; 1];
        model.evtin = 1;
        model.dstate = zeros(13, 1);
        model.rpar = [];
        model.blocktype = "d";
        model.dep_ut = [%f %f];
        exprs = "sil_plant_rpar(mot, brd, loads, locked, 4)";
        x = standard_define([6 4], model, exprs, []);
        x.graphics.style = "blockWithLabel;displayedLabel=Power stage<br>+ motor + ADC";
    end
endfunction
