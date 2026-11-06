// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_M01_mil.sce
// Description: M01: the MIL controller (Scilab prototype) against the SIL controller (C
//              library) in the same diagram layout: currents within 5 mA, commanded
//              position within 1e-6 microsteps, same fault at the same time.
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
        mprintf("SKIP  M01: no C compiler and no built library (README: Software in the loop)\n");
        sil_can_run = %f;
    end
end

function [v, t] = m01_run(scs, ctx)
    scicos_simulate(scs, list(), ctx, "nw");
    v = sil.values;
    t = sil.time;
endfunction

if sil_can_run then
    if ~exists("sil_test_backend") then
        sil_test_backend = "FIXED";
    end
    sil_build(sil_test_backend);
    m_sil = lab_diagram("SIL");
    m_mil = lab_diagram("MIL");
    mp = motion_default();
    Ts = 1 / board_bg431b_esc1().fs;

    // ---- current step at enable, rotor locked, and a move, free rotor
    cases = list(list("current step, rotor locked", struct("lab_mode", 1, "lab_cmd", 0, "locked", 1, "T_end", 0.006)), ..
                 list("move 640 microsteps", struct("lab_mode", 2, "lab_cmd", 640, "T_end", 0.15)));
    for k = 1:size(cases)
        tic();
        vm = m01_run(m_mil, cases(k)(2));
        tm = toc();
        vs = m01_run(m_sil, cases(k)(2));
        n = min(size(vm, 1), size(vs, 1));
        di = max(abs(vm(1:n, 1:2) - vs(1:n, 1:2)));
        dp = max(abs(vm(1:n, 7) - vs(1:n, 7)));
        sim_failures = sim_failures + sim_check("M01 MIL equals SIL: " + cases(k)(1), ..
            (n >= cases(k)(2).T_end / Ts - 1) & (di < 5e-3) & (dp < 1e-6), ..
            msprintf("%d samples, max diff current %.2f mA, commanded %.1e usteps; MIL %.0f s per simulated s", ..
                     n, di * 1e3, dp, tm / cases(k)(2).T_end));
    end

    // ---- the load step above the pull-out torque: the same fault at the same time
    k_fault = 7;
    rs = lab_run("SIL", k_fault);
    rm = lab_run("MIL", k_fault);
    ds = sil_report_data(rs);
    dm = sil_report_data(rm);
    sim_failures = sim_failures + sim_check("M01 the same fault at the same time", ..
        (ds.fault_names == dm.fault_names) & (ds.t_fault >= 0) & (abs(ds.t_fault - dm.t_fault) <= 1e-3 + 1e-9), ..
        msprintf("SIL %s at %.1f ms, MIL %s at %.1f ms", ds.fault_names, ds.t_fault * 1e3, dm.fault_names, dm.t_fault * 1e3));

    // ---- the limits of the signal inputs
    v = m01_run(m_mil, struct("lab_mode", 1, "lab_cmd", 1e9, "lab_irun", 10, "T_end", 0.02));
    i_lim = min(motor_17hs19_2004s1().I_rated, board_bg431b_esc1().i_trip);
    sim_failures = sim_failures + sim_check("M01 input limits", ..
        (v($, 20) == mp.v_max) & (abs(v($, 21) - i_lim) < 1e-12) & (v($, 23) == 1), ..
        msprintf("applied %.0f, %.3f A, flag %d", v($, 20), v($, 21), v($, 23)));

    // ---- a changed run current reaches the stall window, as sstep_set_current in C
    ctl_s = ctrl_default();
    ctl_s.stall_time_ms = 50;
    m01_run(m_mil, struct("lab_mode", 1, "lab_cmd", 3200, "lab_irun", 1, "ctl", ctl_s, "T_end", 0.01));
    global lab_mil;
    sim_failures = sim_failures + sim_check("M01 the stall window follows the run current", ..
        lab_mil.cs.sd.ri == 1600, msprintf("R I = %d mV (1.6 Ohm, 1 A)", lab_mil.cs.sd.ri));

    // ---- what the library refuses, MIL refuses
    mp_bad = mp;
    mp_bad.i_run = 3;
    v1 = m01_run(m_mil, struct("mp", mp_bad, "lab_irun", 2, "T_end", 0.002));
    m1 = lab_mil.invalid;
    ctl_v = ctrl_default();
    ctl_v.drive = 1;
    v2 = m01_run(m_mil, struct("ctl", ctl_v, "T_end", 0.002));
    m2 = lab_mil.invalid;
    sim_failures = sim_failures + sim_check("M01 a configuration the library refuses, and voltage drive: outputs off, fault, reason", ..
        and(v1(:, 13) == 3) & and(v2(:, 13) == 3) & and(v1(:, 14) == 16) & and(v2(:, 14) == 16) & ..
        (max(abs(v1(:, 1:2))) == 0) & (max(abs(v2(:, 1:2))) == 0) & (strindex(m1, "SSTEP_ERR_CONFIG") <> []) & (strindex(m2, "voltage drive") <> []), ..
        m1 + "; " + m2);

    // ---- comparison of the window
    c = lab_compare_data(rs, rm);
    sim_failures = sim_failures + sim_check("M01 comparison data", ..
        (size(c.panels) == 3) & (strindex(c.panels(1).title, "mA") <> []), c.panels(1).title);
end
