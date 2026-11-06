// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/sil_ctrl_rpar.sci
// Description: Parameter vector of the Stepper-C Xcos block (STEPPER_CTRL) from the
//              Scilab parameter structures (SI units -> Stepper-C configuration).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function rpar = sil_ctrl_rpar(mot, brd, ctl, mp, cmds)
    // mot   motor (motor_17hs19_2004s1), brd board (board_bg431b_esc1)
    // ctl   current controller and stall detection (ctrl_default); ctl.mot (optional)
    //       is the motor the controller is configured for, default mot
    // mp    motion (motion_default)
    // cmds  commands [t kind value]: 1 MOVE to [usteps], 2 RUN [usteps/s], 3 STOP
    //       (kind 4, load torque, is a parameter of the plant block: loads)
    // rpar  layout of sil/c/sstep_xcos.c (C_NPAR = 34 values + 3 per command)
    if brd.topology == "shared" then
        topo = 1;
    elseif brd.topology == "full" then
        topo = 0;
    else
        error(msprintf("sil_ctrl_rpar: unknown topology ""%s""", brd.topology));
    end
    ncmd = 0;
    flat = [];
    if ~isempty(cmds) then             // Scilab: zeros(0, 3) is [] (0 x 0)
        if size(cmds, 2) <> 3 then
            error("sil_ctrl_rpar: cmds must have 3 columns [t kind value]");
        end
        cmds = cmds(cmds(:, 2) <> 4, :);   // load steps go to the plant
        ncmd = size(cmds, 1);
        flat = cmds';
        flat = flat(:);                     // [t1 kind1 value1 t2 ...]
    end
    drive = 0;
    if isfield(ctl, "drive") then
        drive = ctl.drive;              // 0 current control, 1 voltage drive
    end
    if isfield(ctl, "mot") then
        mot = ctl.mot;                  // motor the controller is tuned for (T08)
    end
    stall = zeros(4, 1);                // stall detection off
    if isfield(ctl, "stall_time_ms") then
        stall = [ctl.stall_time_ms; ctl.stall_filter_us; ctl.stall_speed; ctl.stall_sync_pct];
    end
    rpar = [round(mot.R * 1000); round(mot.L * 1e6); round(mot.I_rated * 1000); ..
            round(mot.Kt * 1e6); mot.Ns; ..
            topo; round(brd.vbus * 1000); round(brd.vbus_min * 1000); ..
            round(brd.vbus_max * 1000); round(brd.i_full * 1000); round(brd.i_trip * 1000); ..
            brd.period; brd.fs; brd.dead_ns; brd.adc_zero; brd.adc_span; ..
            round(brd.adc_vbus_full * 1000); brd.adc_full; ..
            drive; brd.fs; mp.f_slow; mp.M; ctl.bw_hz; round(mp.i_run * 1000); ..
            round(mp.i_hold * 1000); round(mp.hold_delay * 1000); mp.v_max; mp.acc; mp.dec; ..
            stall; ncmd; flat];
endfunction
