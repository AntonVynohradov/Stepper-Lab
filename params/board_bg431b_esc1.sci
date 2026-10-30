// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        params/board_bg431b_esc1.sci
// Description: STMicroelectronics B-G431B-ESC1 used as a two-phase driver
//              (three half-bridges, phase A between legs U-W, phase B between V-W).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function brd = board_bg431b_esc1()
    brd = struct();
    brd.name     = "B-G431B-ESC1";
    brd.vbus     = 24;        // V, laboratory supply (the board accepts 3S..6S LiPo)
    brd.topology = "shared";  // 3 legs with a common leg
    brd.fs       = 20000;     // Hz, PWM frequency = fast loop rate

    // Used by the software-in-the-loop model (Stepper-C configuration). The ADC scaling
    // is a model of the measurement chain, to be replaced by the measured values of the
    // board (shunt, amplifier gain and offset).
    brd.vbus_min      = 10;      // V, undervoltage trip
    brd.vbus_max      = 30;      // V, overvoltage trip
    brd.i_full        = 4.0;     // A, phase current at ADC full scale
    brd.i_trip        = 3.5;     // A, overcurrent trip
    brd.period        = 4250;    // PWM compare value for 100 % (170 MHz, centre aligned)
    brd.adc_zero      = 2048;    // ADC counts at 0 A
    brd.adc_span      = 2047;    // ADC counts from 0 A to +i_full
    brd.adc_vbus_full = 34.32;   // V at the largest ADC reading
    brd.adc_full      = 4095;    // largest ADC reading (12 bits)

    // Power stage model: "avg" averaged (fast, default) or "switching" (PWM legs with
    // dead time and diodes, see model/bridge_switching.sci; about 5x slower).
    brd.stage    = "avg";
    brd.sw_nstep = 16;      // RK4 steps per PWM period (plus one at each edge)

    // Switching stage values from ngspice (sfs-stepper-power-spice, generated file
    // params/stage_esc1_spice.sci): the timer dead time is 500 ns (assumption), but
    // the driver and MOSFET delays shorten its effect on the phase voltage.
    stg = stage_esc1_spice();
    brd.dead_ns       = stg.dead_eff_ns;    // ns, effective dead time (model and
                                            // Stepper-C dead_time_ns, for compensation)
    brd.dead_timer_ns = stg.dead_timer_ns;  // ns, dead time set in the timer
    brd.rds_on        = stg.r_leg;          // Ohm, leg resistance (switch, shunt)
    brd.vd            = stg.vd;             // V, body diode at 2 A
endfunction
