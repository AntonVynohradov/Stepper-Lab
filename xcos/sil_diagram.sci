// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/sil_diagram.sci
// Description: Builds the software-in-the-loop Xcos diagram (Stepper-C + motor model)
//              and optionally saves it as .zcos.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function scs_m = sil_diagram(path)
    // path  file to write (xcos/stepper_sil.zcos); "" or omitted: only return scs_m
    //
    // Layout:   clock -> Stepper-C <-> power stage + motor + ADC -> monitor -> scope
    //                                                                       -> workspace (sil)
    // The scenario (commands, load, topology, ...) is the diagram context:
    // Simulation > Set Context.
    [lhs, rhs] = argn(0);
    if rhs < 1 then
        path = "";
    end
    loadXcosLibs();
    loadScicos();

    ctx = ["// ===== Stepper-C software in the loop: scenario =====";
           "// Edit here (Simulation > Set Context), then press Start.";
           "mot = motor_17hs19_2004s1();   // motor, params/motor_17hs19_2004s1.sci";
           "brd = board_bg431b_esc1();     // power stage + ADC, params/board_bg431b_esc1.sci";
           "// brd.topology = ""full"";      // two H-bridges instead of the ESC1 shared leg";
           "// brd.stage = ""switching"";    // PWM legs with dead time and diodes (slower)";
           "// mot.Td = 0.02; mot.Tc = 0.01;  // detent torque, dry friction (N*m)";
           "ctl = ctrl_default();          // current loop (bandwidth)";
           "mp  = motion_default();        // microsteps, speed, acceleration, currents";
           "";
           "// Commands [t kind value]: 1 MOVE to [microsteps], 2 RUN [microsteps/s], 3 STOP";
           "cmds = [0, 1, 6400];           // move 2 revolutions (M = 16: 3200 per revolution)";
           "// cmds = [0, 2, 12800; 0.15, 2, -12800];   // run 4 rev/s, then reverse";
           "";
           "loads  = [0, 0];               // load torque steps [t TL], N*m (e.g. [0 0; 0.15 0.25])";
           "locked = 0;                    // 1: rotor locked (current loop test)";
           "T_end  = 0.6;                  // simulated time, s";
           "";
           "// scope ranges";
           "scope_i = 3; scope_rpm = 400; scope_pos = 7000;";
           "";
           "// derived values (do not edit)";
           "Ts = 1 / brd.fs;";
           "N_log = round(T_end / Ts) + 10;"];

    objs = list();
    // 1 fast loop clock
    objs(1) = SampleCLK("define");
    objs(1).graphics.exprs = ["Ts"; "0"];
    objs(1).graphics.orig = [20, 300];
    objs(1).graphics.sz = [40, 40];
    // 2 event split
    objs(2) = CLKSPLIT_f("define");
    objs(2).graphics.orig = [120, 280];
    // 3 library
    objs(3) = STEPPER_CTRL("define");
    objs(3).graphics.orig = [160, 140];
    objs(3).graphics.sz = [120, 80];
    // 4 plant
    objs(4) = STEPPER_PLANT("define");
    objs(4).graphics.orig = [400, 140];
    objs(4).graphics.sz = [120, 80];
    // 5 monitor
    objs(5) = STEPPER_MONITOR("define");
    objs(5).graphics.orig = [620, 120];
    objs(5).graphics.sz = [80, 120];
    // 6 scope: currents, speed, load angle, positions
    objs(6) = CMSCOPE("define");
    objs(6).graphics.exprs = ["4 1 1 2";
                              "1 3 5 7 2 2 2 5";
                              "-1";
                              "[]";
                              "[]";
                              "[-scope_i, -scope_rpm, -180, 0]";
                              "[scope_i, scope_rpm, 180, scope_pos]";
                              "[T_end, T_end, T_end, T_end]";
                              "4000";
                              "1";
                              "Stepper-C SIL"];
    // four input ports before linking (the evaluation below completes the model)
    objs(6).model.in = [4; 1; 1; 2];
    objs(6).model.in2 = [1; 1; 1; 1];
    objs(6).model.intyp = [1; 1; 1; 1];
    objs(6).graphics.pin = [0; 0; 0; 0];
    objs(6).graphics.orig = [800, 120];
    objs(6).graphics.sz = [60, 120];
    // 7 results to the workspace: variable sil (time, values: 12 columns)
    objs(7) = TOWS_c("define");
    objs(7).graphics.exprs = ["N_log"; "sil"; "1"];
    objs(7).graphics.orig = [800, 20];
    objs(7).graphics.sz = [80, 40];
    // 8 end of simulation at T_end
    objs(8) = END_c("define");
    objs(8).graphics.exprs = "T_end";
    objs(8).graphics.orig = [20, 20];
    objs(8).graphics.sz = [40, 40];

    // event links
    objs(9)  = scicos_link(from = [1 1 0], to = [2 1 1], ct = [5 -1]);
    objs(10) = scicos_link(from = [2 1 0], to = [3 1 1], ct = [5 -1]);
    objs(11) = scicos_link(from = [2 2 0], to = [4 1 1], ct = [5 -1]);
    objs(12) = scicos_link(from = [8 1 0], to = [8 1 1], ct = [5 -1]);
    // signal links
    objs(13) = scicos_link(from = [4 1 0], to = [3 1 1], ct = [1 1]);   // ADC -> library
    objs(14) = scicos_link(from = [3 1 0], to = [4 1 1], ct = [1 1]);   // PWM -> power stage
    objs(15) = scicos_link(from = [4 2 0], to = [5 1 1], ct = [1 1]);
    objs(16) = scicos_link(from = [3 2 0], to = [5 2 1], ct = [1 1]);
    objs(17) = scicos_link(from = [5 1 0], to = [6 1 1], ct = [1 1]);
    objs(18) = scicos_link(from = [5 2 0], to = [6 2 1], ct = [1 1]);
    objs(19) = scicos_link(from = [5 3 0], to = [6 3 1], ct = [1 1]);
    objs(20) = scicos_link(from = [5 4 0], to = [6 4 1], ct = [1 1]);
    objs(21) = scicos_link(from = [5 5 0], to = [7 1 1], ct = [1 1]);

    props = scicos_params(tf = 1e5, context = ctx);
    props.title = "stepper_sil";
    scs_m = scicos_diagram(props = props, objs = objs);

    // evaluate the block parameters in the context (as Xcos does before a simulation),
    // so that the saved diagram is consistent (inherited scope events, parameters)
    [%scicos_context, ierr] = script2var(ctx, struct());
    if ierr <> 0 then
        error("sil_diagram: the context cannot be evaluated");
    end
    needcompile = 4;            // read by do_eval
    %cpr = list();
    [scs_m, %cpr, needcompile, ok] = do_eval(scs_m, %cpr, %scicos_context);
    if ~ok then
        error("sil_diagram: block parameters cannot be evaluated");
    end

    if path <> "" then
        xcosDiagramToScilab(path, scs_m);
        mprintf("Diagram written: %s\n", path);
    end
endfunction
