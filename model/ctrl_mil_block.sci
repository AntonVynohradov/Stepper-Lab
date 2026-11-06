// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/ctrl_mil_block.sci
// Description: Computational function of STEPPER_CTRL_MIL: the prototype of the controller
//              (ctrl_period, motion generator) between ADC counts and compare values, with
//              the input limits of sstep_xcos_ctrl_io.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function n = ctrl_mil_config(mot, brd, ctl, mp, cmds)
    // keeps the structures of the context for the next run; n: the value of the parameter
    global lab_mil_cfg;
    lab_mil_cfg = struct("mot", mot, "brd", brd, "ctl", ctl, "mp", mp, "cmds", cmds);
    n = 1;
endfunction

function mess = ctrl_mil_check()
    // mess  "" or why the configuration of ctrl_mil_config cannot run. What sstep_init()
    //       refuses, MIL refuses. The prototype has the current drive only. The bus voltage
    //       trips cannot fire: one board gives the configuration and the plant, and
    //       sstep_init() checks it. An error in the block at the start would end Scilab, so
    //       the set job of the block asks here.
    global lab_mil_cfg;
    cfg = lab_mil_cfg;
    mess = sil_check_rpar(sil_ctrl_rpar(cfg.mot, cfg.brd, cfg.ctl, cfg.mp, cfg.cmds));
    if mess == "" & isfield(cfg.ctl, "drive") then
        if cfg.ctl.drive <> 0 then
            mess = "The Scilab prototype has no voltage drive (ctl.drive = 1): use SIL.";
        end
    end
endfunction

function [v, limited] = ctrl_mil_limit(x, lo, hi, last, limited)
    // as input_limit of sil/c/sstep_xcos.c: rounded, limited, a NaN gives the last value
    if isnan(x) then
        v = last;
        limited = %t;
    else
        v = round(x);
        if v < lo then
            v = lo;
            limited = %t;
        elseif v > hi then
            v = hi;
            limited = %t;
        end
    end
endfunction

function block = ctrl_mil_block(block, flag)
    global lab_mil_cfg;
    global lab_mil;
    if flag == 4 then
        // without windows the set job does not stop a run, and an error or set_blockerror in
        // this block ends Scilab: outputs off, state fault, external fault flag, and the
        // reason in lab_mil.invalid for lab_run
        mess = ctrl_mil_check();
        lab_mil = struct("invalid", mess);
        if mess <> "" then
            mprintf("MIL: %s\n", mess);
            return;
        end
        cfg = lab_mil_cfg;
        c = cfg.ctl;
        mot_c = cfg.mot;
        if isfield(c, "mot") then
            mot_c = c.mot;
        end
        c.protect = 1;
        st = motion_init(cfg.mot, cfg.brd, cfg.mp, cfg.cmds);
        lab_mil = struct("invalid", "", "cs", ctrl_period_init(mot_c, cfg.brd, c, st), "cfg", cfg, "n", 0, ..
                         "cmd", 0, "ma", round(cfg.mp.i_run * 1000), "limited", %f);
    elseif (flag == 1) & (block.nevprt > 0) then
        m = lab_mil;
        if m.invalid <> "" then
            block.outptr(1) = zeros(5, 1);
            block.outptr(2) = [zeros(3, 1); 3; 16; zeros(10, 1)];     // state 3, SSTEP_FAULT_EXTERNAL
            return;
        end
        cfg = m.cfg;
        brd = cfg.brd;
        mode = block.ipar(1);
        t = m.n / brd.fs;
        cs = m.cs;
        if modulo(cs.st.n, cs.st.ratio) == 0 then
            m.limited = %f;
            [ma, m.limited] = ctrl_mil_limit(block.inptr(3) * 1000, 1, ..
                round(min(cfg.mot.I_rated, brd.i_trip) * 1000), m.ma, m.limited);
            value = m.cmd;
            if mode == 1 then
                [value, m.limited] = ctrl_mil_limit(block.inptr(2), -cfg.mp.v_max, cfg.mp.v_max, m.cmd, m.limited);
            elseif mode == 2 then
                [value, m.limited] = ctrl_mil_limit(block.inptr(2), -2147483648, 2147483647, m.cmd, m.limited);
            end
            if ~cs.fault then
                if value <> m.cmd then
                    cs.st.cmds = [cs.st.cmds; t, 3 - mode, value];     // 2 RUN, 1 MOVE
                    m.cmd = value;
                end
                if ma <> m.ma then
                    cs.st.i_run = ma / 1000;
                    cs.st.i_hold = min(cfg.mp.i_hold, ma / 1000);
                    if cs.stall then
                        cs.sd.ri = floor(round(cs.mot_c.R * 1000) * ma / 1000);
                    end
                    m.ma = ma;
                end
            end
            if mode == 0 then
                done = cs.st.cmds(1:min(cs.st.ci, size(cs.st.cmds, 1)), :);
                done = done(done(:, 1) <= t + 1e-12 & (done(:, 2) == 1 | done(:, 2) == 2), :);
                if ~isempty(done) then
                    m.cmd = done($, 3);
                end
            end
        end
        iab = (block.inptr(1)(1:2) - brd.adc_zero) / brd.adc_span * brd.i_full;
        [cs, vsat, duty, k, r, iref] = ctrl_period(cs, iab, t, ref_motion, []);
        comp = zeros(5, 1);
        comp(1:size(duty, 1)) = round(duty * brd.period);
        comp(5) = bool2s(~cs.fault);
        block.outptr(1) = comp;

        state = 1;
        if cs.st.v_s <> 0 then
            state = 2;
        end
        faults = 0;
        est = zeros(1, 5);
        if cs.fault then
            state = 3;
            faults = cs.sd.faults;
        end
        if cs.stall then
            est = stall_status(cs.sd);
        end
        block.outptr(2) = [r(2); cs.st.pos / 65536; cs.st.v_s; state; faults; iref; est(:); ..
                           m.cmd; m.ma / 1000; bool2s(m.limited)];
        m.cs = cs;
        m.n = m.n + 1;
        lab_mil = m;
    end
endfunction
