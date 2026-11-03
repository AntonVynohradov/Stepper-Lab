// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/sim_stall_run.sci
// Description: Test helper: runs a stall detection scenario (sim_stall_cases) on the
//              Scilab prototype or on the Xcos diagram and compares the two.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function brd = sim_stall_board(s, stage)
    // board of scenario s: "avg" = averaged stage with the ngspice leg voltage error,
    // "switching" = switching stage
    brd = board_bg431b_esc1();
    brd.topology = s.topology;
    brd.stage = stage;
    if stage == "avg" then
        brd.leg_error = stage_esc1_spice().leg_error;
    end
endfunction

function [r, y] = sim_stall_proto(s, stage, c)
    // s      scenario (sim_stall_cases), stage "avg" or "switching"
    // c      controller with the stall configuration (ctrl_default + stall_*)
    // r      result (sim_stall_eval)
    // y      one row per period [t iA iB emf_d emf_q faults] (A, mV)
    brd = sim_stall_board(s, stage);
    c.mot = s.mot_c;
    st = motion_init(s.mot, brd, s.mp, s.cmds);
    [rec, st, ext, sd] = sim_closed_loop(s.mot, brd, c, ref_motion, [], s.T, s.locked <> 0, st);
    p = s.mot.Ns / 4;
    M = s.mp.M;
    d = (rec(:, 12) - p * rec(:, 5)) * 180 / %pi;
    faults = sd.faults * bool2s((sd.t_fault >= 0) & (rec(:, 1) >= sd.t_fault - 1e-9));
    r = sim_stall_eval(rec(:, 1), d, rec(:, 12) / (%pi / 2) * M, p * rec(:, 5) / (%pi / 2) * M, ..
                       faults, sd.rec(:, 3) * 100 / sd.N, M);
    y = [rec(:, 1:3), sd.rec(:, 1:2), faults];
endfunction

function [r, y, v, t] = sim_stall_sil(scs_m, s, stage, ctl)
    // scs_m  diagram (sil_diagram), s scenario, stage "avg" or "switching"
    // ctl    controller with the stall configuration
    // r, y   as sim_stall_proto; faults and the detector values come from the library
    //        (sstep_get_status through the monitor, variable sil)
    // v, t   sil.values, sil.time of the run
    brd = sim_stall_board(s, stage);
    ctl.mot = s.mot_c;
    loads = [0, 0];
    if ~isempty(s.cmds) then
        kl = find(s.cmds(:, 2) == 4);
        if ~isempty(kl) then
            loads = s.cmds(kl, [1, 3]);
        end
    end
    ctx = struct("cmds", s.cmds, "brd", brd, "ctl", ctl, "mp", s.mp, "mot", s.mot, ..
                 "loads", loads, "locked", s.locked, "T_end", s.T, ..
                 "N_log", round(s.T * brd.fs) + 10);
    scicos_simulate(scs_m, list(), ctx, "nw");
    v = sil.values;
    t = sil.time;
    r = sim_stall_eval(t, v(:, 6), v(:, 7), v(:, 8), v(:, 14), v(:, 19), s.mp.M);
    y = [t, v(:, 1:2), v(:, 15:16), v(:, 14)];
endfunction

function r = sim_stall_eval(t, d, pcmd, prot, faults, level, M)
    // t, d        time [s], load angle [electrical deg] per period
    // pcmd, prot  commanded and rotor position [microsteps]
    // faults      fault flags per period (32 SSTEP_FAULT_STALL, 1 OVERCURRENT)
    // level       stall detector level [%] per period
    // r           [t_stall t_90 delay lost peak d_max flags t_fault]:
    //             time of SSTEP_FAULT_STALL (-1 none), time the load angle passes 90 deg
    //             (pull-out, -1 none), delay = t_stall - t_90, full steps lost at the
    //             first fault (|commanded - rotor| / M), largest detector level [%],
    //             largest |load angle| before the first fault [deg], flags of the first
    //             fault, its time (-1 none), full steps lost at the end of the run
    kf = find(faults <> 0, 1);
    ks = find(bitand(faults, 32) <> 0, 1);
    k90 = find(abs(d) > 90, 1);
    t_fault = -1;
    flags = 0;
    lost = 0;
    sel = 1:size(t, 1);
    if ~isempty(kf) then
        t_fault = t(kf);
        flags = faults(kf);
        lost = abs(pcmd(kf) - prot(kf)) / M;
        sel = 1:kf - 1;
    end
    t_stall = -1;
    if ~isempty(ks) then
        t_stall = t(ks);
    end
    t90 = -1;
    if ~isempty(k90) then
        t90 = t(k90);
    end
    delay = -1;
    if (t_stall >= 0) & (t90 >= 0) then
        delay = t_stall - t90;
    end
    r = [t_stall, t90, delay, lost, max(level), max(abs(d(sel))), flags, t_fault, ..
         abs(pcmd($) - prot($)) / M];
endfunction

function txt = sim_stall_flags(flags)
    // names of the fault flags of Stepper-C
    names = ["OVERCURRENT", "OVERVOLTAGE", "UNDERVOLTAGE", "HAL", "EXTERNAL", "STALL"];
    txt = "";
    for b = 0:5
        if bitand(flags, 2^b) <> 0 then
            txt = txt + " " + names(b + 1);
        end
    end
    txt = stripblanks(txt);
endfunction

function [ok, detail] = sim_stall_verdict(s, r, c)
    // s  scenario, r  result (sim_stall_eval), c  controller (stall_time_ms)
    // must trip:     SSTEP_FAULT_STALL first, 0 < delay after the pull-out (90 deg) <
    //                stall_time_ms + 20 ms
    // must not trip: no fault, less than 2 full steps from the command at the end
    if s.trip then
        ok = (r(7) == 32) & (r(2) >= 0) & (r(3) > 0) & (r(3) < c.stall_time_ms / 1000 + 0.02);
        if r(8) < 0 then
            detail = msprintf("no fault, pull-out at %.1f ms, largest detector level %d %%", ..
                              r(2) * 1e3, r(5));
        elseif r(7) == 32 then
            detail = msprintf("SSTEP_FAULT_STALL at %.1f ms, %.1f ms after the pull-out (90 deg at %.1f ms), %.1f full steps lost", ..
                              r(1) * 1e3, r(3) * 1e3, r(2) * 1e3, r(4));
        else
            detail = msprintf("first fault %s at %.1f ms (%.1f ms after the pull-out), %.1f full steps lost, largest stall detector level %d %%", ..
                              sim_stall_flags(r(7)), r(8) * 1e3, (r(8) - r(2)) * 1e3, r(4), r(5));
        end
    else
        ok = (r(8) < 0) & (r(9) < 2);
        if r(8) < 0 then
            detail = msprintf("no fault, largest detector level %d %% (margin %d %%), max load angle %.1f deg, %.2f full steps from the command at the end", ..
                              r(5), 100 - r(5), r(6), r(9));
        else
            detail = msprintf("%s at %.1f ms, max load angle before %.1f deg", ..
                              sim_stall_flags(r(7)), r(8) * 1e3, r(6));
        end
    end
endfunction

function [di, de, dt] = sim_stall_compare(rp, yp, rs, ys)
    // rp, yp  prototype result and record, rs, ys  SIL (sim_stall_proto, sim_stall_sil)
    // di      largest current difference while the rotor is synchronous (before the
    //         first fault and before the load angle passes 90 deg in either) [A]
    // de      largest back-EMF estimate difference in the same samples [mV]
    // dt      difference of the times of the first fault [s] (0: neither has a fault,
    //         %inf: only one, or different fault flags)
    // After the pull-out the free slip amplifies small differences (ADC quantization,
    // fixed point); the fault time is compared on its own.
    n = min(size(yp, 1), size(ys, 1));
    te = [rp(8), rs(8), rp(2), rs(2)];
    te = te(te >= 0);
    sel = 1:n;
    if ~isempty(te) then
        sel = find(yp(1:n, 1) < min(te) - 1e-9);
    end
    di = max(abs(yp(sel, 2:3) - ys(sel, 2:3)));
    de = max(abs(yp(sel, 4:5) - ys(sel, 4:5)));
    if (rp(8) < 0) & (rs(8) < 0) then
        dt = 0;
    elseif (rp(8) < 0) | (rs(8) < 0) | (rp(7) <> rs(7)) then
        dt = %inf;
    else
        dt = abs(rp(8) - rs(8));
    end
endfunction
