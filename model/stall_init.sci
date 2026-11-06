// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stall_init.sci
// Description: Stall detector constants and state (Stepper-C 1.1.0: check_stall(),
//              derive_stall(), sstep_stall_reset()).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function sd = stall_init(mot, brd, c, st)
    // mot  motor known to the controller: R, L, Kt in the integer units of
    //      sstep_motor_cfg (mOhm, uH, uV*s/rad), as sil_ctrl_rpar passes them
    // brd  board: fs (fast loop rate), vbus (V_base = vbus_nominal_mv)
    // c    controller: stall_time_ms (0 = off), stall_filter_us, stall_speed
    //      [microsteps/s], stall_sync_pct
    // st   motion generator state (motion_init): Ns, M, f (slow loop rate)
    // sd   detector: constants and state, used by stall_observe and stall_check
    if c.stall_time_ms == 0 then
        error("stall_init: stall detection is off (c.stall_time_ms = 0)");
    end
    if (c.stall_filter_us <= 0) | (c.stall_speed <= 0) | (c.stall_sync_pct <= 0) | ..
       (c.stall_sync_pct > 100) | (c.stall_filter_us * brd.fs < 1e6) then
        error("stall_init: invalid stall_* configuration (sstep_init: SSTEP_ERR_PARAM / CONFIG)");
    end
    r_mohm = round(mot.R * 1000);
    l_uh = round(mot.L * 1e6);
    ke_uv = round(mot.Kt * 1e6);
    nsm = st.Ns * st.M;

    sd = struct();
    sd.N = floor(c.stall_time_ms * st.f / 1000);          // slow periods to trip
    // back-EMF of stall_speed: Ke 2 pi v / (Ns M), 2 pi = 710 / 113 as in the library
    e_thr_uv = floor(c.stall_speed * ke_uv * 710 / (113 * nsm));
    sd.e_s = floor((e_thr_uv + 500) / 1000);              // [mV]
    if (sd.N == 0) | (sd.e_s == 0) | (sd.e_s >= round(brd.vbus * 1000)) then
        error("stall_init: stall_time_ms below a slow period or threshold out of range (SSTEP_ERR_CONFIG)");
    end
    sd.a = (l_uh * brd.fs + r_mohm * 500) / 1e6;          // L / Ts + R / 2 [Ohm]
    sd.b = (l_uh * brd.fs - r_mohm * 500) / 1e6;          // L / Ts - R / 2 [Ohm]
    sd.alpha = 1e6 / (brd.fs * c.stall_filter_us);        // Ts / tau
    sd.vbase = round(brd.vbus * 1000) / 1000;             // [V], saturation of 1 pu
    sd.kv = ke_uv * 1e-6 * 710 / (113 * nsm);             // back-EMF per microstep/s [V]
    sd.ke_uv = ke_uv;
    sd.nsm = nsm;
    sd.speed = c.stall_speed;
    sd.sync = c.stall_sync_pct;
    sd.ri = floor(r_mohm * round(st.i_run * 1000) / 1000);  // R I_run [mV]

    // state, as after sstep_enable() (sstep_stall_reset)
    sd.i_prev = zeros(2, 1);   // phase currents of the previous fast period [A]
    sd.v1 = zeros(2, 1);       // applied phase voltages computed 1 period ago [V]
    sd.v2 = zeros(2, 1);       // ... 2 periods ago [V]
    sd.emf = zeros(2, 1);      // filtered back-EMF [d; q] [V]
    sd.off = zeros(2, 1);      // mean deviation at rest [mV * 2^16]
    sd.vel = 0;                // commanded velocity of the slow period in use [usteps/s]
    sd.eref = 0;               // its back-EMF Ke w [mV]
    sd.count = 0;              // up/down counter
    sd.hold = 0;               // slow periods the condition stays true after a reversal
    sd.peak = 0;               // largest counter value
    sd.t_fault = -1;           // time of SSTEP_FAULT_STALL [s], -1 = none
    sd.hist = zeros(0, 8);     // per slow period: [t ed eq eref vel bad count |dev|]
endfunction
