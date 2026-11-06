// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        params/ctrl_default.sci
// Description: Default current controller and stall detection settings (Stepper-C:
//              struct sstep_ctrl_cfg).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function c = ctrl_default()
    c = struct();
    c.bw_hz = 1000;  // Hz, current loop bandwidth: Kp = L wc, Ki = R wc
    c.delay = 1;     // computation delay in fast loop periods (0 or 1); MCU: 1
    c.ff    = 1;     // back-EMF feed-forward from the reference speed (0 or 1)
    c.nsub  = 4;     // RK4 sub-steps of the plant per fast loop period
    c.protect = 0;   // 1: overcurrent trip of the library (brd.i_trip) in sim_closed_loop

    // Stall detection (Stepper-C 1.1.0, back-EMF observer). Off by default, as in the
    // library; tests T13 / S06 switch it on with stall_time_ms = 50. Time, filter and
    // speed are the example of the library documentation (17HS19, ESC1); the
    // synchronism threshold is 30 % instead of its example 50 %: back-EMF along the motion
    // is 35 % of Ke w at 11 rev/s, 24 V (shared leg, 2 A). The window on the d axis of the
    // library accepts that lag as well, so 50 % also runs there.
    c.stall_time_ms   = 0;     // ms, condition time before SSTEP_FAULT_STALL; 0 = off
    c.stall_filter_us = 1000;  // us, time constant of the back-EMF low-pass filter
    c.stall_speed     = 1600;  // microsteps/s (0.5 rev/s at M = 16): slip threshold,
                               // synchronism check from this commanded speed up
    c.stall_sync_pct  = 30;    // %, smallest back-EMF along the motion, of Ke w
endfunction
