// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/lab_diagram.sci
// Description: Builds the stepper_lab diagram: four superblocks, Inputs (command, run
//              current, load torque), Controller, Plant, Outputs (labelled scopes and the
//              variable sil).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function scs_m = lab_diagram(kind, path)
    // kind  "SIL": the C library, "MIL": the Scilab prototype, same layout
    // path  file to write; "" or omitted: only return scs_m
    [lhs, rhs] = argn(0);
    if rhs < 2 then
        path = "";
    end
    if and(kind <> ["SIL", "MIL"]) then
        error("lab_diagram: unknown kind " + kind);
    end
    loadXcosLibs();
    loadScicos();

    ctx = ["// ===== Stepper-C lab: scenario =====";
           "// Edit here (Simulation > Set Context), then press Start.";
           "mot = motor_17hs19_2004s1();   // motor, params/motor_17hs19_2004s1.sci";
           "brd = board_bg431b_esc1();     // power stage + ADC, params/board_bg431b_esc1.sci";
           "// brd.topology = ""full"";      // two H-bridges instead of the ESC1 shared leg";
           "// brd.stage = ""switching"";    // PWM legs with dead time and diodes (slower)";
           "// brd.leg_error = stage_active().leg_error;  // averaged legs + ngspice voltage error";
           "// mot.Td = 0.02; mot.Tc = 0.01;  // detent torque, dry friction (N*m)";
           "ctl = ctrl_default();          // current loop (bandwidth), stall detection";
           "// ctl.stall_time_ms = 50;       // stall detection on";
           "mp  = motion_default();        // microsteps, speed, acceleration, currents";
           "";
           "// Which input drives the motion: 1 velocity (microsteps/s), 2 position (microsteps),";
           "// 0 the command table cmds [t kind value]: 1 MOVE, 2 RUN, 3 STOP";
           "lab_mode = 1;";
           "cmds = zeros(0, 3);";
           "";
           "// Values of the example sources in the Inputs superblock; replace the blocks";
           "// there for other signals (step, sine, ramp, ...)";
           "lab_cmd  = 3200;               // 1 rev/s at M = 16";
           "lab_irun = mp.i_run;           // run current, A";
           "lab_load = 0;                  // load torque, N*m";
           "locked = 0;                    // 1: rotor locked (current loop test)";
           "T_end  = 0.6;                  // simulated time, s";
           "";
           "// scope ranges";
           "scope_i = 3; scope_rpm = 400; scope_pos = 7000;";
           "";
           "// derived values (do not edit)";
           "Ts = 1 / brd.fs;"];

    function b = place(b, orig, sz)
        b.graphics.orig = orig;
        b.graphics.sz = sz;
    endfunction

    function b = port_in(id, n, orig)
        b = IN_f("define");
        b.graphics.exprs = string(id);
        b.model.ipar = id;
        b.model.out = n;
        b.model.out2 = 1;
        b.model.outtyp = 1;
        b = place(b, orig, [20, 20]);
    endfunction

    function b = port_out(id, n, orig)
        b = OUT_f("define");
        b.graphics.exprs = string(id);
        b.model.ipar = id;
        b.model.in = n;
        b.model.in2 = 1;
        b.model.intyp = 1;
        b = place(b, orig, [20, 20]);
    endfunction

    function b = port_event(orig)
        b = CLKINV_f("define");
        b.graphics.exprs = "1";
        b.model.ipar = 1;
        b = place(b, orig, [20, 20]);
    endfunction

    function b = scope(title, ports, colors, ymin, ymax, orig)
        b = CMSCOPE("define");
        b.graphics.exprs = [string(ports); colors; "-1"; "[]"; "[]"; ymin; ymax; "T_end"; "4000"; "1"; title];
        b.model.in = ports;
        b.model.in2 = 1;
        b.model.intyp = 1;
        b.graphics.pin = 0;
        b = place(b, orig, [80, 50]);
    endfunction

    function b = source(expr, label, orig)
        b = CONST_m("define");
        b.graphics.exprs = expr;
        b.model.label = label;
        b = place(b, orig, [60, 40]);
    endfunction

    function objs = with_links(objs, links)
        // links: rows of from block, from port, to block, to port (signals), then events
        n = size(objs);
        for k = 1:size(links, 1)
            r = links(k, :);
            objs(n + k) = scicos_link(from = [r(1), r(2), 0], to = [r(3), r(4), 1], ct = [r(5), r(6)]);
        end
    endfunction

    function b = superblock(label, objs, in, out, evt, orig, sz)
        // label name, objs inner diagram, in, out port sizes, evt: an event input
        props = scicos_params(tf = 1e5);
        props.title = label;
        b = SUPER_f("define");
        b.model.rpar = scicos_diagram(props = props, objs = objs);
        b.model.in = in(:);
        b.model.in2 = ones(in(:));
        b.model.intyp = ones(in(:));
        b.model.out = out(:);
        b.model.out2 = ones(out(:));
        b.model.outtyp = ones(out(:));
        b.model.label = label;
        b.graphics.pin = zeros(in(:));
        b.graphics.pout = zeros(out(:));
        b.graphics.in_implicit = emptystr(in(:)) + "E";
        b.graphics.out_implicit = emptystr(out(:)) + "E";
        b.graphics.style = "blockWithLabel;displayedLabel=" + label;
        if evt then
            b.model.evtin = 1;
            b.graphics.pein = 0;
        else
            b.model.evtin = [];
            b.graphics.pein = [];
        end
        b = place(b, orig, sz);
    endfunction

    // ---- Inputs: the sources, the user replaces them
    o = list();
    o(1) = source("lab_cmd", "lab_cmd", [40, 160]);
    o(2) = source("lab_irun", "lab_irun", [40, 100]);
    o(3) = source("lab_load", "lab_load", [40, 40]);
    o(4) = port_out(1, 1, [200, 170]);
    o(5) = port_out(2, 1, [200, 110]);
    o(6) = port_out(3, 1, [200, 50]);
    o = with_links(o, [1 1 4 1 1 1; 2 1 5 1 1 1; 3 1 6 1 1 1]);
    b_in = superblock("Inputs", o, [], [1; 1; 1], %f, [40, 120], [100, 90]);

    // ---- Controller: the Stepper-C library, or its Scilab prototype
    o = list();
    if kind == "SIL" then
        o(1) = place(STEPPER_CTRL_IO("define"), [180, 90], [120, 80]);
    else
        o(1) = place(STEPPER_CTRL_MIL("define"), [180, 90], [120, 80]);
    end
    o(2) = port_in(1, 3, [60, 150]);
    o(3) = port_in(2, 1, [60, 120]);
    o(4) = port_in(3, 1, [60, 90]);
    o(5) = port_out(1, 5, [380, 150]);
    o(6) = port_out(2, 15, [380, 100]);
    o(7) = port_event([230, 220]);
    o = with_links(o, [2 1 1 1 1 1; 3 1 1 2 1 1; 4 1 1 3 1 1; 1 1 5 1 1 1; 1 2 6 1 1 1; 7 1 1 1 5 -1]);
    b_ctrl = superblock("Controller", o, [3; 1; 1], [5; 15], %t, [220, 120], [120, 90]);

    // ---- Plant: power stage + motor + ADC, one C block
    o = list();
    o(1) = place(STEPPER_PLANT_IO("define"), [180, 90], [120, 80]);
    o(2) = port_in(1, 5, [60, 150]);
    o(3) = port_in(2, 1, [60, 100]);
    o(4) = port_out(1, 3, [380, 150]);
    o(5) = port_out(2, 9, [380, 100]);
    o(6) = port_event([230, 220]);
    o = with_links(o, [2 1 1 1 1 1; 3 1 1 2 1 1; 1 1 4 1 1 1; 1 2 5 1 1 1; 6 1 1 1 5 -1]);
    b_plant = superblock("Plant", o, [5; 1], [3; 9], %t, [420, 120], [120, 90]);

    // ---- Outputs: monitor, scopes, the variable sil
    o = list();
    o(1) = place(STEPPER_MONITOR_IO("define"), [160, 130], [80, 170]);
    o(2) = port_in(1, 9, [60, 250]);
    o(3) = port_in(2, 15, [60, 180]);
    o(4) = scope("Phase currents [A]", 4, "1 3 5 7", "-scope_i", "scope_i", [320, 360]);
    o(5) = scope("Speed [rpm]", 1, "1", "-scope_rpm", "scope_rpm", [320, 290]);
    o(6) = scope("Position [microsteps]", 2, "2 5", "0", "scope_pos", [320, 220]);
    o(7) = scope("Load angle [deg]", 1, "5", "-180", "180", [320, 150]);
    o(8) = scope("State, faults, stall level", 3, "1 3 5", "-1", "101", [320, 80]);
    o(9) = TOWS_c("define");
    o(9).graphics.exprs = ["round(T_end * brd.fs) + 10"; "sil"; "1"];
    o(9) = place(o(9), [320, 10], [80, 40]);
    o = with_links(o, [2 1 1 1 1 1; 3 1 1 2 1 1; 1 1 4 1 1 1; 1 2 5 1 1 1; 1 3 6 1 1 1; ..
                       1 4 7 1 1 1; 1 5 8 1 1 1; 1 6 9 1 1 1]);
    b_out = superblock("Outputs", o, [9; 15], [], %f, [620, 120], [100, 90]);

    // ---- top level
    objs = list();
    objs(1) = b_in;
    objs(2) = b_ctrl;
    objs(3) = b_plant;
    objs(4) = b_out;
    objs(5) = place(SampleCLK("define"), [240, 300], [40, 40]);
    objs(5).graphics.exprs = ["Ts"; "0"];
    objs(6) = CLKSPLIT_f("define");
    objs(6).graphics.orig = [340, 270];
    objs(7) = place(END_c("define"), [40, 20], [40, 40]);
    objs(7).graphics.exprs = "T_end";
    objs = with_links(objs, [1 1 2 2 1 1; 1 2 2 3 1 1; 1 3 3 2 1 1; ..   // command, current, load
                             3 1 2 1 1 1; 2 1 3 1 1 1; ..                // ADC, PWM
                             3 2 4 1 1 1; 2 2 4 2 1 1; ..                // to the outputs
                             5 1 6 1 5 -1; 6 1 2 1 5 -1; 6 2 3 1 5 -1; 7 1 7 1 5 -1]);

    props = scicos_params(tf = 1e5, context = ctx);
    props.title = "stepper_lab";
    scs_m = scicos_diagram(props = props, objs = objs);

    // evaluate the block parameters in the context, as in sil_diagram
    [%scicos_context, ierr] = script2var(ctx, struct());
    if ierr <> 0 then
        error("lab_diagram: the context cannot be evaluated");
    end
    needcompile = 4;            // read by do_eval
    %cpr = list();
    [scs_m, %cpr, needcompile, ok] = do_eval(scs_m, %cpr, %scicos_context);
    if ~ok then
        error("lab_diagram: block parameters cannot be evaluated");
    end

    if path <> "" then
        xcosDiagramToScilab(path, scs_m);
        mprintf("Diagram written: %s\n", path);
    end
endfunction
