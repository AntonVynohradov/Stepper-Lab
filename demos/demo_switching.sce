// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        demos/demo_switching.sce
// Description: Demo: switching power stage (B-G431B-ESC1, 24 V, 20 kHz, dead time)
//              at 4 rev/s: sampled current with its ripple band, mean phase voltage
//              error caused by the dead time, comparison with the averaged model.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

exec(fullfile(get_absolute_file_path("demo_switching.sce"), "..", "loader.sce"), -1);

mot = motor_17hs19_2004s1();
brd = board_bg431b_esc1();
c = ctrl_default();
sp = struct("I", 2, "rev_s", 4, "t_ramp", 0.02, "Ns", mot.Ns);
T = 0.05;

brd.stage = "switching";
[rec, st, ext] = sim_closed_loop(mot, brd, c, ref_speed_ramp, sp, T);
brd.stage = "avg";
rec_a = sim_closed_loop(mot, brd, c, ref_speed_ramp, sp, T);

t = rec(:, 1) * 1e3;
scf(0); clf();
subplot(3, 1, 1);
plot(t, rec(:, 2), "b", t, ext(:, 1), "c", t, ext(:, 2), "c", t, rec_a(:, 2), "r--", t, rec(:, 6), "k:");
xtitle("Phase A current: switching (sampled, ripple band), averaged, reference", "t [ms]", "iA [A]");
legend(["switching (sampled)"; "ripple min"; "ripple max"; "averaged model"; "reference"]);
subplot(3, 1, 2);
plot(t, (ext(:, 2) - ext(:, 1)) * 1e3, "b", t, (ext(:, 4) - ext(:, 3)) * 1e3, "r");
xtitle("Current ripple within each PWM period", "t [ms]", "[mA p-p]");
legend(["phase A"; "phase B"]);
subplot(3, 1, 3);
plot(t, ext(:, 5) - rec(:, 8), "b", t, ext(:, 6) - rec(:, 9), "r");
xtitle(msprintf("Mean phase voltage error (effective dead time %.0f ns, diode %.2f V)", brd.dead_ns, brd.vd), ..
       "t [ms]", "[V]");
legend(["phase A"; "phase B"]);
