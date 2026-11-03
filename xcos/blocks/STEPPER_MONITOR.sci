// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/blocks/STEPPER_MONITOR.sci
// Description: Xcos block: signals for the scope and the workspace (currents,
//              speed, load angle, positions). Computational function sstep_xcos_monitor.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [x, y, typ] = STEPPER_MONITOR(job, arg1, arg2)
    // Inputs:  1 plant [iA iB w theta vA vB |iA|max |iB|max], 2 controller monitor (12)
    // Outputs: 1 currents [iA iB iA_ref iB_ref] (A), 2 speed (rpm),
    //          3 load angle (electrical deg), 4 positions [commanded rotor] (microsteps),
    //          5 all values + [vA vB |iA|max |iB|max] + [state faults emf_d emf_q
    //          rotor_velocity load_angle_est stall_level] (19) for the workspace
    //          (variable sil)
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
        model.sim = list("sstep_xcos_monitor", 4);
        model.in = [8; 12];
        model.in2 = [1; 1];
        model.intyp = [1; 1];
        model.out = [4; 1; 1; 2; 19];
        model.out2 = [1; 1; 1; 1; 1];
        model.outtyp = [1; 1; 1; 1; 1];
        model.rpar = [200; 16];
        model.blocktype = "c";
        model.dep_ut = [%t %f];
        exprs = "[mot.Ns, mp.M]";
        x = standard_define([4 6], model, exprs, []);
        x.graphics.style = "blockWithLabel;displayedLabel=Monitor";
    end
endfunction
