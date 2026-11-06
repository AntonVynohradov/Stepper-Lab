// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/lab_run.sci
// Description: The stepper_lab diagram of a scenario (lab_scenario_diagram) and its run
//              without windows.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function scs_m = lab_scenario_diagram(kind, k)
    // kind  "SIL" or "MIL", k  index in lab_scenarios()
    // scs_m the diagram with the sources and the context of the scenario
    global sil_linked_backend;
    if isempty(sil_linked_backend) then
        sil_build(lab_settings_load().backend);     // the plant is C in both kinds
    end
    sc = lab_scenarios()(k);
    scs_m = lab_diagram(kind);
    for j = 1:size(sc.sources)
        src = sc.sources(j);
        p = src(3);
        select src(2)
        case "step" then
            b = STEP_FUNCTION("define");
            b.model.rpar.objs(1).graphics.exprs = [string(p(1)); string(p(2)); string(p(3))];
        case "sine" then
            b = GENSIN_f("define");
            b.graphics.exprs = [string(p(1)); msprintf("2*%%pi*%g", p(2)); "0"];
        case "ramp" then
            b = RAMP("define");
            b.graphics.exprs = [string(p(1)); string(p(2)); "0"];
        end
        scs_m = lab_set_source(scs_m, src(1), b);
    end
    scs_m.props.context = [scs_m.props.context(:); sc.ctx(:)];
endfunction

function [res, scs_m] = lab_run(kind, k)
    // kind, k  as lab_scenario_diagram
    // res      results of the run (the variable sil of the diagram)
    // scs_m    the diagram, ready for xcos(scs_m)
    scs_m = lab_scenario_diagram(kind, k);
    scicos_simulate(scs_m, list(), struct(), "nw");
    res = sil;
    if kind == "MIL" then
        global lab_mil;
        if lab_mil.invalid <> "" then
            error("MIL: " + lab_mil.invalid);
        end
    end
endfunction
