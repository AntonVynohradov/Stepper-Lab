// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T11_resonance.sce
// Description: T11: rotor resonance. Natural frequency sqrt(p Kt I / J) / 2 pi; the
//              detent torque excites it at 4 p n = fn (low-speed resonance); beyond
//              the voltage limit the oscillation is no longer damped (mid-band
//              instability). Twin: tools/ref_ext.py.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

mot = motor_17hs19_2004s1();
brd = board_bg431b_esc1();
c = ctrl_default();
p = mot.Ns / 4;
I = 2;
fn = sqrt(p * mot.Kt * I / mot.J) / (2 * %pi);

// T11a natural frequency after a small angle step
phi1 = 0.1;
rp = struct("I", I, "phi1", phi1, "t_step", 5e-3);
rec = sim_closed_loop(mot, brd, c, ref_angle_step, rp, 0.03);
sel = rec(:, 1) >= 5e-3;
t = rec(sel, 1);
y = p * rec(sel, 5) - phi1;
k = find((y(1:$-1) < 0) & (y(2:$) >= 0));
tc = t(k) - y(k) .* (t(k + 1) - t(k)) ./ (y(k + 1) - y(k));
f = 1 / mean(diff(tc));
sim_failures = sim_failures + sim_check("T11 rotor natural frequency sqrt(p Kt I / J) / 2 pi", ..
    (abs(f / fn - 1) < 0.05) & (size(tc, "*") >= 4), ..
    msprintf("%.1f Hz from %d periods (theory %.1f Hz)", f, size(tc, "*"), fn));

// T11b detent torque: resonance when 4 p n = fn
md = mot;
md.Td = 0.02;                                   // N*m, assumption (about 3 % of 0.59 N*m)
n_res = fn / (4 * p);
speeds = 0.8:0.1:1.8;
amp = zeros(speeds);
for k = 1:size(speeds, "*")
    sp = struct("I", I, "rev_s", speeds(k), "t_ramp", 0.1, "Ns", mot.Ns);
    rec = sim_closed_loop(md, brd, c, ref_speed_ramp, sp, 0.3);
    d = sim_load_angle(rec(rec(:, 1) >= 0.2, :), mot);
    amp(k) = (max(d) - min(d)) / 2;
end
[a_max, k_max] = max(amp);
txt = strcat(msprintf("%.1f: %.1f\n", speeds', amp'), ", ");
sim_failures = sim_failures + sim_check("T11 detent 0.02 N*m: largest rotor oscillation at 4 p n = fn", ..
    (abs(speeds(k_max) / n_res - 1) < 0.1) & (a_max > 3 * min(amp)), ..
    msprintf("peak at %.2f rev/s (theory %.2f), oscillation %s deg", speeds(k_max), n_res, txt));

// T11c mid-band instability (shared leg, 24 V): 5, 8, 11 rev/s
vs = [5, 8, 11];
osc = zeros(vs);
scl = zeros(vs);
for k = 1:3
    sp = struct("I", I, "rev_s", vs(k), "t_ramp", 0.2, "Ns", mot.Ns);
    rec = sim_closed_loop(mot, brd, c, ref_speed_ramp, sp, 0.5);
    sel = rec(:, 1) >= 0.4;
    d = sim_load_angle(rec(sel, :), mot);
    osc(k) = (max(d) - min(d)) / 2;
    scl(k) = min(rec(sel, 10));
end
txt = strcat(msprintf("%d rev/s: %.1f deg (scale %.2f)\n", vs', osc', scl'), ", ");
sim_failures = sim_failures + sim_check("T11 mid-band instability: damped below the voltage limit, limit cycle above", ..
    (osc(1) < 1) & (osc(2) < 5) & (osc(3) > 20), txt);
