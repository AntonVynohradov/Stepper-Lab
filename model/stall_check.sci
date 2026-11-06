// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stall_check.sci
// Description: Stall decision of one slow loop period (Stepper-C 1.1.0:
//              sstep_stall_check() and the hand-over of the next command in
//              sstep_slow_loop()). Integer millivolts, as in both numeric backends.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [sd, trip] = stall_check(sd, t, vel_next)
    // sd        detector (stall_init, stall_observe)
    // t         time of the slow period [s] (history only)
    // vel_next  commanded velocity of the slow period that starts [microsteps/s]
    // trip      %t: SSTEP_FAULT_STALL (the condition persisted for stall_time_ms)
    //
    // Decision against the command of the period that just ended (sd.vel, sd.eref):
    // in motion (|v| >= stall_speed) sgn(v) eq < sync_pct / 100 |Ke w| with ed inside
    // [-(|Ke w| / 3 + R I / 10), |Ke w| / 3 + 2 R I / 5], a rotor that does not turn;
    // sgn(v) eq < -(|Ke w| / 2 + e_s), a reversed back-EMF, keeps the condition true for
    // N / 2 periods. At rest or slow |(ed, eq - Ke w) - o| > e_s with o the running mean
    // (time constant N periods). Up/down counter, trip at N.
    OFF_ONE = 65536;
    ed = round(sd.emf(1) * 1000);                  // [mV], rounded as real_to_units
    eq = round(sd.emf(2) * 1000);
    eref = sd.eref;
    vel = sd.vel;
    dmag = 0;
    if abs(vel) >= sd.speed then
        if vel > 0 then
            along = eq;
        else
            along = -eq;
        end
        er = abs(eref);
        bad = (along * 100 < sd.sync * er) & ..
              (ed <= floor(er / 3) + floor(sd.ri * 2 / 5)) & ..
              (ed >= -(floor(er / 3) + floor(sd.ri / 10)));
        if along * 2 < -(er + 2 * sd.e_s) then
            sd.hold = floor(sd.N / 2);
        end
    else
        dev = [ed; eq - eref];
        for k = 1:2
            sd.off(k) = sd.off(k) + sim_c_div(dev(k) * OFF_ONE - sd.off(k), sd.N);
            dev(k) = dev(k) - sim_c_div(sd.off(k), OFF_ONE);
        end
        dmag = sqrt(dev(1)^2 + dev(2)^2);
        bad = dev(1)^2 + dev(2)^2 > sd.e_s^2;
    end
    if sd.hold <> 0 then
        sd.hold = sd.hold - 1;
        bad = %t;
    end
    if bad then
        sd.count = sd.count + 1;
    elseif sd.count > 0 then
        sd.count = sd.count - 1;
    end
    sd.peak = max(sd.peak, sd.count);
    trip = sd.count >= sd.N;
    sd.hist($ + 1, :) = [t, ed, eq, eref, vel, bool2s(bad), sd.count, dmag];

    // command of the next period: velocity and its back-EMF (feed-forward value,
    // saturated at 1 pu, in mV)
    sd.vel = vel_next;
    sd.eref = round(min(max(sd.kv * vel_next, -sd.vbase), sd.vbase) * 1000);
endfunction
