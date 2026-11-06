// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_S07_lab.sce
// Description: S07..S11: the stepper_lab model. S07 signal inputs against the command
//              table, S08 input limits, S09 runs through a fault, S10 hierarchy, S11 report.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

sil_can_run = %t;
if isempty(sil_linked_backend) then
    if ~isfile(fullfile(sim_root, "sil", "build", "fixed", "loader.sce")) & ~haveacompiler() then
        mprintf("SKIP  S07: no C compiler and no built library (README: Software in the loop)\n");
        sil_can_run = %f;
    end
end

function [v, t] = s07_run(scs, ctx)
    // simulates without windows; v, t: sil.values, sil.time
    scicos_simulate(scs, list(), ctx, "nw");
    v = sil.values;
    t = sil.time;
endfunction

if sil_can_run then
    if ~exists("sil_test_backend") then
        sil_test_backend = "FIXED";
    end
    sil_build(sil_test_backend);
    lab = lab_diagram("SIL");

    // ---- S07 constant signal and the equivalent command table, free rotor
    cases = list(list("velocity", 1, 3200, [0, 2, 3200]), list("position", 2, 640, [0, 1, 640]));
    for k = 1:size(cases)
        c = cases(k);
        va = s07_run(lab, struct("lab_mode", c(2), "lab_cmd", c(3), "T_end", 0.15));
        vb = s07_run(lab, struct("lab_mode", 0, "cmds", c(4), "T_end", 0.15));
        n = min(size(va, 1), size(vb, 1));
        di = max(abs(va(1:n, 1:2) - vb(1:n, 1:2)));
        dp = max(abs(va(1:n, 7) - vb(1:n, 7)));
        sim_failures = sim_failures + sim_check(msprintf("S07 %s signal equals the command table", c(1)), ..
            (n >= 2999) & (size(va, 2) == 23) & (di < 1e-9) & (dp == 0) & and(va(1:n, 14) == vb(1:n, 14)) & ..
            (max(abs(va(1:n, 21) - motion_default().i_run)) == 0) & (max(va(1:n, 23)) == 0) & ..
            (va(n, 20) == c(3)) & (vb(n, 20) == c(3)), ..
            msprintf("%d samples, %d columns, max diff i %.1e A, commanded %.1e usteps", n, size(va, 2), di, dp));
    end

    // ---- S08 inputs out of range: limited, the run reaches T_end
    mp = motion_default();
    mot = motor_17hs19_2004s1();
    brd = board_bg431b_esc1();
    T = 0.05;
    Ts = 1 / brd.fs;
    i_lim = min(mot.I_rated, brd.i_trip);
    // mode, command, run current, expected column 20, expected column 21
    cs = [1,  1e9,  mp.i_run, mp.v_max,   mp.i_run;
          1, -1e9,  mp.i_run, -mp.v_max,  mp.i_run;
          1, %inf,  mp.i_run, mp.v_max,   mp.i_run;
          2, 1e12,  mp.i_run, 2147483647, mp.i_run;
          2, -1e12, mp.i_run, -2147483648, mp.i_run;
          1, %nan,  mp.i_run, 0,          mp.i_run;
          1, 3200,  0,        3200,       0.001;
          1, 3200,  -1,       3200,       0.001;
          1, 3200,  10,       3200,       i_lim;
          1, 3200,  %inf,     3200,       i_lim;
          1, 3200,  %nan,     3200,       mp.i_run];
    for k = 1:size(cs, 1)
        [v, t] = s07_run(lab, struct("lab_mode", cs(k, 1), "lab_cmd", cs(k, 2), "lab_irun", cs(k, 3), "T_end", T));
        ok = (t($) >= T - 2 * Ts) & (v($, 20) == cs(k, 4)) & (abs(v($, 21) - cs(k, 5)) < 1e-12) & (v($, 23) == 1) & (max(v(:, 14)) == 0);
        sim_failures = sim_failures + sim_check(msprintf("S08 mode %d, command %g, current %g A: limited", cs(k, 1:3)), ok, ..
            msprintf("applied %.0f, %.3f A, flag %d, faults %d, end %.3f s", v($, 20), v($, 21), v($, 23), max(v(:, 14)), t($)));
    end

    // ---- S08 a current below the hold current: the hold stays below the run current
    hold_ctx = struct("lab_mode", 1, "lab_cmd", 0, "lab_irun", 0.5, "T_end", mp.hold_delay + 0.1);
    [v, t] = s07_run(lab, hold_ctx);
    sim_failures = sim_failures + sim_check("S08 hold current follows a lower run current", ..
        (max(sqrt(v($ - 100:$, 1).^2 + v($ - 100:$, 2).^2)) < 0.53) & (v($, 21) == 0.5) & (v($, 23) == 0), ..
        msprintf("final amplitude %.3f A", sqrt(v($, 1)^2 + v($, 2)^2)));

    // ---- S08 a sine: a new value every slow period, no fault
    for c = list(list(1, 3200, "velocity"), list(2, 640, "position"))
        src = GENSIN_f("define");
        src.graphics.exprs = [string(c(2)); "2*%pi*5"; "0"];
        s = lab_set_source(lab, "lab_cmd", src);
        [v, t] = s07_run(s, struct("lab_mode", c(1), "T_end", 0.25));
        sel = 21:size(v, 1);
        err = max(abs(v(sel, 20) - c(2) * sin(2 * %pi * 5 * t(sel))));
        sim_failures = sim_failures + sim_check("S08 sine on the " + c(3) + " input", ..
            (t($) >= 0.25 - 2 * Ts) & (max(v(:, 14)) == 0) & (max(v(:, 23)) == 0) & (err < 0.05 * c(2)), ..
            msprintf("largest difference to the input %.1f, faults %d", err, max(v(:, 14))));
    end

    // ---- S09 a run through a fault: T06 load step, the command keeps changing
    ctl = ctrl_default();
    ctl.stall_time_ms = 50;
    load_step = STEP_FUNCTION("define");
    load_step.model.rpar.objs(1).graphics.exprs = ["0.15"; "0"; "0.5"];
    cmd_step = STEP_FUNCTION("define");
    cmd_step.model.rpar.objs(1).graphics.exprs = ["0.3"; "3200"; "1600"];
    s = lab_set_source(lab, "lab_load", load_step);
    s = lab_set_source(s, "lab_cmd", cmd_step);
    [v, t] = s07_run(s, struct("lab_mode", 1, "ctl", ctl, "T_end", 0.4));
    kf = find(v(:, 14) <> 0, 1);
    ok = %f;
    detail = "no fault";
    if ~isempty(kf) then
        ok = (t($) >= 0.4 - 2 * Ts) & (bitand(v(kf, 14), 32) <> 0) & and(v(kf:$, 13) == 3) & ..
             (max(abs(v(kf + 1:$, 7) - v(kf + 1, 7))) == 0) & (v($, 20) == v(kf, 20));
        detail = msprintf("faults %d at %.1f ms, end %.3f s, applied %.0f", v(kf, 14), t(kf) * 1e3, t($), v($, 20));
    end
    sim_failures = sim_failures + sim_check("S09 the run goes on after a fault", ok, detail);
    with_fault = struct("time", t, "values", v);
    t_first = t(kf);

    // ---- S10 hierarchy: four superblocks, the file is read back, equal to stepper_sil
    names = "";
    nblocks = 0;
    for k = 1:size(lab.objs)
        if typeof(lab.objs(k)) == "Block" then
            nblocks = nblocks + 1;
            if lab.objs(k).model.sim(1) == "super" then
                names = [names, lab.objs(k).model.label];
            end
        end
    end
    sim_failures = sim_failures + sim_check("S10 the top level has the four superblocks", ..
        (strcat(names, " ") == " Inputs Controller Plant Outputs") & (nblocks == 7), ..
        msprintf("%s, %d blocks", strcat(names, " "), nblocks));
    s10_file = fullfile(TMPDIR, "s10 lab.zcos");
    lab_diagram("SIL", s10_file);
    s10_read = %f;
    s10_saved = lab;
    if isfile(s10_file) then
        scs_m = [];
        s10_read = importXcosDiagram(s10_file);
        if s10_read then
            s10_saved = scs_m;
        end
    end
    [v, t] = s07_run(s10_saved, struct("lab_mode", 1, "lab_cmd", 3200, "T_end", 0.05));
    sim_failures = sim_failures + sim_check("S10 the file is read back and runs", ..
        s10_read & (t($) >= 0.05 - 2 * Ts) & (size(v, 2) == 23), msprintf("end %.3f s", t($)));
    sil_ref = sil_diagram();
    for stg = ["avg", "switching"]
        brd_s = brd;
        brd_s.stage = stg;
        c10 = [0, 2, 3200];
        va = s07_run(lab, struct("lab_mode", 0, "cmds", c10, "brd", brd_s, "T_end", 0.08));
        vb = s07_run(sil_ref, struct("cmds", c10, "brd", brd_s, "loads", [0, 0], "T_end", 0.08));
        n = min(size(va, 1), size(vb, 1));
        d19 = max(abs(va(1:n, 1:19) - vb(1:n, 1:19)));
        sim_failures = sim_failures + sim_check("S10 columns 1 to 19 equal stepper_sil, " + stg, ..
            (n >= 1599) & (d19 < 1e-9), msprintf("%d samples, largest difference %.1e", n, d19));
    end
    for T = [0.1, 0.4]
        [v, t] = s07_run(lab, struct("lab_mode", 1, "lab_cmd", 3200, "T_end", T));
        n = size(v, 1);
        sim_failures = sim_failures + sim_check(msprintf("S10 the buffer follows T_end = %.1f s", T), ..
            abs(n - T * brd.fs) <= 2, msprintf("%d samples", n));
    end

    // ---- S10 scenarios of the window: each runs to its end with the expected outcome
    scs = lab_scenarios();
    for k = 1:size(scs)
        sc = scs(k);
        rs = lab_run("SIL", k);
        faults = rs.values(:, 14);
        ok = (rs.time($) >= sc.T - 2 * Ts);
        if sc.expect == "fault" then
            ok = ok & (bitand(faults(find(faults <> 0, 1)), 32) <> 0);
        else
            ok = ok & (max(faults) == 0);
        end
        if sc.changes then
            ok = ok & (max(rs.values(:, 20)) - min(rs.values(:, 20)) > 100);
        end
        sim_failures = sim_failures + sim_check("S10 scenario: " + sc.name, ok, ..
            msprintf("end %.3f s, faults %d", rs.time($), max(faults)));
    end

    // ---- S11 report: labels of every panel and the first fault
    [v2, t2] = s07_run(lab, struct("lab_mode", 1, "lab_cmd", 3200, "T_end", 0.05));
    no_fault = struct("time", t2, "values", v2);
    d = sil_report_data(with_fault);
    d0 = sil_report_data(no_fault);
    ok = (size(d.panels) >= 7);
    for k = 1:size(d.panels)
        p = d.panels(k);
        ok = ok & (p.title <> "") & (strindex(p.ylabel, "[") <> []) & (size(p.legend, "*") == size(p.y, 2)) & (size(p.y, 1) == size(with_fault.time, 1));
    end
    sim_failures = sim_failures + sim_check("S11 every panel has a title, a unit and a legend", ok, msprintf("%d panels", size(d.panels)));
    // the runaway of the free rotor after a fault must not set the scale
    k_ang = 4;
    pa = d.panels(k_ang);
    kf = find(with_fault.values(:, 14) <> 0, 1);
    sim_failures = sim_failures + sim_check("S11 the scale of a panel comes from the run up to the fault", ..
        isfield(pa, "ylim") & (max(abs(pa.y($, :))) > 2 * max(abs(pa.y(kf, :)))) & (pa.ylim(2) < max(pa.y)), ..
        msprintf("load angle at the fault %.0f deg, at the end %.0f deg", pa.y(kf), pa.y($)));

    sim_failures = sim_failures + sim_check("S11 time and names of the first fault", ..
        (d.t_fault == t_first) & (strindex(d.fault_names, "STALL") <> []) & (d0.t_fault == -1) & (d0.fault_names == ""), ..
        msprintf("%s at %.1f ms", d.fault_names, d.t_fault * 1e3));
    if execstr("h = sil_report(with_fault, %f);", "errcatch") == 0 then
        na = size(h.children(h.children.type == "Axes"), "*");
        delete(h);
        sim_failures = sim_failures + sim_check("S11 the report draws one axis per panel", na == size(d.panels), msprintf("%d axes", na));
    else
        mprintf("SKIP  S11 drawing: %s\n", lasterror());
    end
end
