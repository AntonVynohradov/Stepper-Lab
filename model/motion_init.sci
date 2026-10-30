// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/motion_init.sci
// Description: Initial state of the motion generator (ref_motion). Integer units of
//              Stepper-C: position Q16 microsteps, velocity Q16 per slow period.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function st = motion_init(mot, brd, mp, cmds)
    // mot, brd  motor and board parameters
    // mp        motion settings (motion_default), engineering units
    // cmds      commands, one row [t kind value] in time order:
    //           kind 1 MOVE  value = target position [microsteps]
    //           kind 2 RUN   value = velocity [microsteps/s]
    //           kind 3 STOP  decelerate to standstill (value ignored)
    //           kind 4 LOAD  value = load torque [N*m] (simulation only)
    Q16 = 65536;
    st = struct();
    st.M = mp.M;
    st.Ns = mot.Ns;
    st.f = mp.f_slow;
    st.ratio = round(brd.fs / mp.f_slow);  // fast loop periods per slow period
    st.dt = 1 / mp.f_slow;
    st.shift = 14 - round(log2(mp.M));     // angle of one microstep = 2^30 / M
    // limits as in sstep_ctrl.c set_limits()
    st.v_max = sim_div_round(mp.v_max * Q16, st.f);
    st.acc = sim_div_round(mp.acc * Q16, st.f * st.f);
    st.dec = sim_div_round(mp.dec * Q16, st.f * st.f);
    st.brake_vmax = floor(st.v_max * st.v_max / (2 * st.dec));
    st.hold_ticks = floor(round(mp.hold_delay * 1000) * st.f / 1000);
    st.i_run = mp.i_run;
    st.i_hold = mp.i_hold;
    st.cmds = cmds;

    st.n = 0;            // fast loop period counter
    st.ci = 1;           // next command
    st.mode = 0;         // 0 hold, 1 move, 2 run
    st.target = 0;       // [Q16 microsteps]
    st.v_run = 0;        // [Q16 / period]
    st.pos = 0;          // [Q16 microsteps]
    st.vel = 0;          // [Q16 / period]
    st.still = 0;        // slow periods without motion
    st.hold_active = %f;
    st.angle0 = 0;       // electrical angle at the period start, unwrapped [2^-32 turn]
    st.dangle = 0;       // angle increment per fast period
    st.k = 0;            // increments applied in this period
    st.v_s = 0;          // mean velocity of the period [microsteps/s]
    st.I = mp.i_run;     // current amplitude [A]
    st.TL = 0;           // load torque [N*m]
    st.hist = zeros(0, 5);  // [t pos[usteps] v[usteps/s] vel[Q16/period] mode] per slow period
endfunction
