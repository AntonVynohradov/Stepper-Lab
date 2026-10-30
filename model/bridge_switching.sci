// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/bridge_switching.sci
// Description: Switching power stage: centre-aligned PWM leg timing with dead time,
//              leg voltages with Rds(on) and body diodes (twin: tools/ref_switching.py).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function ev = bridge_leg_events(d, Ts, td)
    // d   leg duty cycle 0..1 (compare / period)
    // Ts  PWM period [s], td dead time [s]
    // ev  switching events of the leg in one period, rows [t_start state],
    //     state 0 = low side on, 1 = high side on, 2 = dead (both off, diode conducts)
    //
    // The period starts at the counter peak, where all low-side switches are on (the
    // current sampling instant). Centre-aligned PWM: high side on in
    // [Ts/2 (1 - d), Ts/2 (1 + d)]; the timer delays every turn-on by td, so a pulse
    // shorter than td disappears.
    if d >= 1 then
        ev = [0, 1];
    elseif d <= 0 then
        ev = [0, 0];
    else
        t1 = Ts / 2 * (1 - d);
        t2 = Ts / 2 * (1 + d);
        ev = [0, 0; t1, 2];
        if t1 + td < t2 then
            ev = [ev; t1 + td, 1];
        end
        ev = [ev; t2, 2];
        if t2 + td < Ts then
            ev = [ev; t2 + td, 0];
        end
    end
endfunction

function s = bridge_leg_state(ev, t)
    // state of a leg at time t (last event with t_start <= t)
    s = ev(1, 2);
    for k = 1:size(ev, 1)
        if ev(k, 1) <= t then
            s = ev(k, 2);
        end
    end
endfunction

function v = bridge_leg_voltage(state, i_leg, vbus, brd)
    // leg output voltage referred to the negative rail; i_leg > 0 flows out of the leg
    if state == 1 then
        v = vbus - i_leg * brd.rds_on;
    elseif state == 0 then
        v = -i_leg * brd.rds_on;
    elseif i_leg >= 0 then
        v = -brd.vd;              // dead: low-side diode carries the current
    else
        v = vbus + brd.vd;        // dead: high-side diode
    end
endfunction

function v = bridge_phase_voltages(states, x, vbus, brd)
    // phase voltages [vA; vB] for the leg states and the phase currents x(1:2)
    iA = x(1);
    iB = x(2);
    if brd.topology == "full" then
        il = [iA, -iA, iB, -iB];
        vl = zeros(1, 4);
        for k = 1:4
            vl(k) = bridge_leg_voltage(states(k), il(k), vbus, brd);
        end
        v = [vl(1) - vl(2); vl(3) - vl(4)];
    else
        il = [iA, iB, -(iA + iB)];
        vl = zeros(1, 3);
        for k = 1:3
            vl(k) = bridge_leg_voltage(states(k), il(k), vbus, brd);
        end
        v = [vl(1) - vl(3); vl(2) - vl(3)];
    end
endfunction

function [x, ext] = stepper_period_sw(x, duty, enabled, TL, mot, brd, Ts, locked)
    // One PWM period of the switching stage.
    // x        motor state [iA; iB; w; theta] at the start of the period
    // duty     leg duty cycles (4 legs "full", 3 legs "shared")
    // enabled  %f: all switches off (only the diodes conduct)
    // TL       load torque [N*m]
    // brd      board: vbus, topology, dead_ns, rds_on, vd, sw_nstep
    // ext      [iAmin iAmax iBmin iBmax vA vB] over the period (vA, vB = mean phase
    //          voltages)
    //
    // The motor is integrated with RK4 between the switching instants (at least
    // sw_nstep steps per period), so every edge falls on a step boundary.
    td = brd.dead_ns * 1e-9;
    nleg = size(duty, "*");
    evs = list();
    bps = [0; Ts];
    for k = 1:nleg
        if enabled then
            ev = bridge_leg_events(duty(k), Ts, td);
        else
            ev = [0, 2];
        end
        evs(k) = ev;
        inner = ev(ev(:, 1) > 0 & ev(:, 1) < Ts, 1);
        bps = [bps; inner];
    end
    bps = unique(bps);
    hmax = Ts / brd.sw_nstep;
    ext = [x(1), x(1), x(2), x(2), 0, 0];
    states = zeros(1, nleg);
    for j = 1:size(bps, "*") - 1
        ta = bps(j);
        tb = bps(j + 1);
        for k = 1:nleg
            states(k) = bridge_leg_state(evs(k), ta);
        end
        n = ceil((tb - ta) / hmax - 1e-9);
        h = (tb - ta) / n;
        for s = 1:n
            v = bridge_phase_voltages(states, x, brd.vbus, brd);
            x = stepper_rk4(x, [v; TL], mot, h, locked);
            ext(5) = ext(5) + v(1) * h;
            ext(6) = ext(6) + v(2) * h;
            ext(1) = min(ext(1), x(1));
            ext(2) = max(ext(2), x(1));
            ext(3) = min(ext(3), x(2));
            ext(4) = max(ext(4), x(2));
        end
    end
    ext(5) = ext(5) / Ts;
    ext(6) = ext(6) / Ts;
endfunction
