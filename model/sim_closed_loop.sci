// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/sim_closed_loop.sci
// Description: Discrete closed-loop simulation: fast loop (current control and
//              power stage) at fs, motor integrated with RK4 between samples.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [rec, st, ext, sd] = sim_closed_loop(mot, brd, c, ref, refp, T, locked, st)
    // mot, brd, c  motor, board and controller parameters
    // ref          reference function [r, st] = ref(t, refp, st), r = [I; phi; w_ref; TL]
    //              (current amplitude [A], electrical angle [rad], mechanical
    //              reference speed [rad/s], load torque [N*m]); st is the state of the
    //              reference generator, called once per period in time order
    // T            simulated time [s]
    // locked       %t: rotor locked (optional, default %f)
    // st           initial state of the reference generator (optional, default struct())
    //
    // rec          one row per fast loop period:
    //              [t iA iB w theta iA_ref iB_ref vA vB k I phi]
    //              (states at the sampling instant t, voltage applied during the period,
    //              k = power stage scale (1 = not limited), I amplitude and phi electrical
    //              angle of the current reference)
    // st           final state of the reference generator
    // ext          one row per period [iAmin iAmax iBmin iBmax vA vB]: current extremes
    //              and mean phase voltages within the period (switching stage; with the
    //              averaged stage the end values and the applied voltage)
    //
    // c.mot        motor parameters known to the controller (gains, feed-forward),
    //              optional, default mot (T08: controller tuned for other values)
    //
    // brd.stage    "avg" (default): averaged power stage, c.nsub RK4 steps per period;
    //              "switching": PWM legs with dead time, Rds(on) and body diodes
    //              (stepper_period_sw), the leg duties of the period are applied
    // brd.leg_error  averaged stage: leg voltage error table [i v] (bridge_avg_error),
    //              optional, [] = ideal legs
    //
    // c.stall_time_ms > 0: stall detection of Stepper-C 1.1.0 (stall_init,
    //              stall_observe, stall_check); needs the motion generator (ref_motion
    //              with st from motion_init) for the commanded velocity
    // c.protect = 1: overcurrent trip of the library (brd.i_trip, brd.i_full,
    //              brd.adc_span: |current| above the trip level in ADC counts), optional,
    //              default off
    // sd           stall detector after the run (stall_init); sd.t_fault time of the
    //              fault (-1: none), sd.faults its flag (32 SSTEP_FAULT_STALL,
    //              1 SSTEP_FAULT_OVERCURRENT), sd.rec one row per period
    //              [emf_d emf_q (mV) counter]. After the fault the outputs are off
    //              (zero voltage, switching stage: all legs off) from the next period
    //              and the current reference is frozen, as in the library.
    //              Only t_fault and faults with c.protect alone, [] without both.
    //
    // Order in every period (as on the MCU): sample the currents at t, compute the
    // references and the controller, apply the voltage computed in the previous period
    // (c.delay = 1) or immediately (c.delay = 0) for one period, integrate the motor.
    // With stall detection the decision runs after the slow loop of the period
    // (motion_slow), before the fast loop, as in sstep_slow_loop().

    [lhs, rhs] = argn(0);
    if rhs < 7 then
        locked = %f;
    end
    if rhs < 8 then
        st = struct();
    end

    mot_c = mot;
    if isfield(c, "mot") then
        mot_c = c.mot;
    end
    g = current_pi_gains(mot_c, c, brd.fs);
    N = round(T / g.Ts);
    h = g.Ts / c.nsub;
    x = zeros(4, 1);
    v_hold = zeros(2, 1);
    rec = zeros(N, 12);
    ext = zeros(N, 6);
    switching = isfield(brd, "stage");
    if switching then
        switching = brd.stage == "switching";
    end
    if brd.topology == "full" then
        d_hold = zeros(4, 1);
    else
        d_hold = zeros(3, 1);
    end
    en_hold = %f;
    lerr = [];
    if isfield(brd, "leg_error") then
        lerr = brd.leg_error;
    end

    cs = ctrl_period_init(mot_c, brd, c, st);
    sd_rec = [];
    if cs.stall then
        sd_rec = zeros(N, 3);
    end

    for n = 0:N - 1
        t = n * g.Ts;
        [cs, vsat, duty, k, r, iref] = ctrl_period(cs, x(1:2), t, ref, refp);
        if cs.stall then
            sd_rec(n + 1, :) = [round(cs.sd.emf' * 1000), cs.sd.count];
        end

        if c.delay == 1 then
            v_apply = v_hold;
            d_apply = d_hold;
            en = en_hold;
        else
            v_apply = vsat;
            d_apply = duty;
            en = ~cs.fault;
        end
        v_hold = vsat;
        d_hold = duty;
        en_hold = ~cs.fault;

        rec(n + 1, :) = [t, x', iref', v_apply', k, r(1), r(2)];

        if switching then
            [x, e_sw] = stepper_period_sw(x, d_apply, en, r(4), mot, brd, g.Ts, locked);
            ext(n + 1, :) = e_sw;
        else
            u = [v_apply; r(4)];
            for s = 1:c.nsub
                if en & ~isempty(lerr) then
                    u(1:2) = v_apply + bridge_avg_error(lerr, x(1:2), brd.topology);
                end
                x = stepper_rk4(x, u, mot, h, locked);
            end
            ext(n + 1, :) = [x(1), x(1), x(2), x(2), v_apply'];
        end
    end
    st = cs.st;
    sd = cs.sd;
    if cs.stall then
        sd.rec = sd_rec;
    end
endfunction
