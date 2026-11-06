// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/ctrl_period.sci
// Description: The controller of one fast loop period: references, stall detection,
//              overcurrent trip, current loop, voltage limit, without the plant. Used by
//              sim_closed_loop and by the MIL block of stepper_lab.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function cs = ctrl_period_init(mot_c, brd, c, st)
    // mot_c  motor known to the controller, brd board, c controller
    // st     state of the reference function (struct() when it has none)
    // cs     state of ctrl_period
    cs = struct();
    cs.g = current_pi_gains(mot_c, c, brd.fs);
    cs.mot_c = mot_c;
    cs.brd = brd;
    cs.c = c;
    cs.st = st;
    cs.integ = zeros(2, 1);
    if brd.topology == "full" then
        cs.d_hold = zeros(4, 1);
    else
        cs.d_hold = zeros(3, 1);
    end
    cs.stall = %f;
    if isfield(c, "stall_time_ms") then
        cs.stall = c.stall_time_ms > 0;
    end
    cs.protect = %f;
    if isfield(c, "protect") then
        cs.protect = c.protect == 1;
    end
    cs.sd = [];
    if cs.stall then
        if ~isfield(st, "ratio") then
            error("sim_closed_loop: stall detection needs the motion generator (ref_motion, motion_init)");
        end
        cs.sd = stall_init(mot_c, brd, c, st);
    elseif cs.protect then
        cs.sd = struct("t_fault", -1);
    end
    if cs.protect then
        // overcurrent trip of sstep_fast_loop() in ADC counts from zero
        cs.i_trip_counts = floor(round(brd.i_trip * 1000) * brd.adc_span / round(brd.i_full * 1000));
    end
    cs.fault = %f;
    if cs.stall | cs.protect then
        cs.sd.faults = 0;
    end
    cs.r_last = [];
endfunction

function [cs, vsat, duty, k, r, iref] = ctrl_period(cs, imeas, t, ref, refp)
    // cs     state (ctrl_period_init)
    // imeas  measured phase currents [iA; iB] [A]
    // t      time of the period [s]
    // ref    reference function [r, st] = ref(t, refp, st), r = [I; phi; w_ref; TL]
    // vsat   phase voltages after the limit of the power stage [V], duty leg duties,
    //        k voltage scale
    // r      references of the period, iref phase current references
    // A fault freezes the reference and switches the outputs off (cs.fault), the load in
    // r(4) goes on.
    g = cs.g;
    c = cs.c;
    brd = cs.brd;
    if cs.stall then
        slow = modulo(cs.st.n, cs.st.ratio) == 0;
    end
    [r, cs.st] = ref(t, refp, cs.st);
    if cs.stall & ~cs.fault & slow then
        [cs.sd, trip] = stall_check(cs.sd, t, cs.st.v_s);
        if trip then
            cs.fault = %t;              // shutdown(): outputs off, no fast loop
            cs.sd.t_fault = t;
            cs.sd.faults = 32;          // SSTEP_FAULT_STALL
        end
    end
    if cs.protect & ~cs.fault then
        if or(abs(round(imeas * brd.adc_span / brd.i_full)) > cs.i_trip_counts) then
            cs.fault = %t;
            cs.sd.t_fault = t;
            cs.sd.faults = 1;           // SSTEP_FAULT_OVERCURRENT
            if cs.stall then
                if cs.sd.count <> 0 then
                    cs.sd.faults = 33;  // ... with SSTEP_FAULT_STALL: the detector had started
                end
            end
        end
    end
    if cs.fault then
        r(1:3) = cs.r_last(1:3);        // reference frozen, the load goes on
    end
    cs.r_last = r;
    iref = r(1) * [cos(r(2)); sin(r(2))];
    if c.ff == 1 then
        ff = cs.mot_c.Kt * r(3) * [-sin(r(2)); cos(r(2))];
    else
        ff = zeros(2, 1);
    end

    if cs.fault then
        vsat = zeros(2, 1);
        duty = cs.d_hold;
        k = 1;
    else
        [vu, e] = current_pi(iref, imeas, cs.integ, ff, g);
        [vsat, duty, k] = bridge_avg(vu, brd.vbus, brd.topology);
        cs.integ = current_pi_update(cs.integ, e, vu, vsat, g);
        if cs.stall then
            cs.sd = stall_observe(cs.sd, imeas, vsat, r(2));
        end
    end
    cs.d_hold = duty;
endfunction
