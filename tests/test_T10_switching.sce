// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T10_switching.sce
// Description: T10: switching power stage. Ideal switches give the averaged model
//              and the analytic ripple; the dead time error equals
//              -2 sign(i) (Vbus + 2 Vd) td / Ts; at 4 rev/s with the real stage the
//              rotor stays synchronous and the current peaks stay below the trip.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

mot = motor_17hs19_2004s1();
brd0 = board_bg431b_esc1();
c = ctrl_default();
Ts = 1 / brd0.fs;
hold06 = struct("I0", 2, "I1", 2, "t_step", 0, "phi", 0.6);
hold0 = struct("I0", 2, "I1", 2, "t_step", 0, "phi", 0);
topos = ["full", "shared"];

// T10a: ideal switches (no dead time, no Rds, no diode drop)
for n_t = 1:2
    brd = brd0;
    brd.topology = topos(n_t);
    brd.dead_ns = 0;
    brd.rds_on = 0;
    brd.vd = 0;
    brd.stage = "switching";
    rec_s = sim_closed_loop(mot, brd, c, ref_current_step, hold06, 0.02, %t);
    brd.stage = "avg";
    rec_a = sim_closed_loop(mot, brd, c, ref_current_step, hold06, 0.02, %t);
    di = max(abs(rec_s(:, 2:3) - rec_a(:, 2:3)));
    brd.stage = "switching";
    [rec_s, st_s, ext] = sim_closed_loop(mot, brd, c, ref_current_step, hold0, 0.02, %t);
    last = ext($ - 199:$, :);
    pp = mean(last(:, 2) - last(:, 1));
    va = mot.R * 2;
    ana = (brd.vbus - mot.R * 2) * (va / brd.vbus) * Ts / 2 / mot.L;
    sim_failures = sim_failures + sim_check("T10 " + topos(n_t) + ": ideal switching = averaged model, ripple as analytic", ..
        (di < 1e-3) & (abs(pp - ana) / ana < 0.05), ..
        msprintf("max sampled difference %.4f mA, ripple %.2f mA p-p (analytic %.2f mA)", ..
                 di * 1e3, pp * 1e3, ana * 1e3));
end

// T10b: dead time voltage error of a phase (two legs, full bridge)
brd = brd0;
brd.topology = "full";
brd.stage = "switching";
td = brd.dead_ns * 1e-9;
want = 2 * (brd.vbus + 2 * brd.vd) * td / Ts;
worst = 0;
for I = [2, -2]
    hp = struct("I0", I, "I1", I, "t_step", 0, "phi", 0);
    [rec_s, st_s, ext] = sim_closed_loop(mot, brd, c, ref_current_step, hp, 0.02, %t);
    sel = size(rec_s, 1) - 199:size(rec_s, 1);
    resid = ext(sel, 5) - rec_s(sel, 8) + sign(I) * want + 2 * brd.rds_on * rec_s(sel, 2);
    worst = max(worst, max(abs(resid)));
end
sim_failures = sim_failures + sim_check("T10 dead time: phase voltage error = -2 sign(i) (Vbus + 2 Vd) td / Ts", ..
    worst < 0.01 * want, ..
    msprintf("expected %.3f V at td = %.0f ns, max deviation %.2f mV", want, td * 1e9, worst * 1e3));

// T10c: 4 rev/s with the real stage (dead time, Rds, diodes)
sp = struct("I", 2, "rev_s", 4, "t_ramp", 0.05, "Ns", mot.Ns);
for n_t = 1:2
    brd = brd0;
    brd.topology = topos(n_t);
    brd.stage = "switching";
    [rec_s, st_s, ext] = sim_closed_loop(mot, brd, c, ref_speed_ramp, sp, 0.15);
    brd.stage = "avg";
    rec_a = sim_closed_loop(mot, brd, c, ref_speed_ramp, sp, 0.15);
    sel = rec_s(:, 1) >= 0.1;
    lag = max(abs(rec_s(sel, 12) - mot.Ns / 4 * rec_s(sel, 5))) * 180 / %pi;
    err_s = sqrt(mean((rec_s(sel, 2) - rec_s(sel, 6)).^2 + (rec_s(sel, 3) - rec_s(sel, 7)).^2));
    err_a = sqrt(mean((rec_a(sel, 2) - rec_a(sel, 6)).^2 + (rec_a(sel, 3) - rec_a(sel, 7)).^2));
    peak = max(abs(ext(:, 1:4)));
    sim_failures = sim_failures + sim_check("T10 " + topos(n_t) + " 4 rev/s with dead time: synchronous, peak below the trip", ..
        (lag < 90) & (peak < brd.i_trip), ..
        msprintf("peak %.3f A, max load angle %.1f deg, rms tracking error %.1f mA (averaged %.1f mA)", ..
                 peak, lag, err_s * 1e3, err_a * 1e3));
end
