// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        demos/demo_open_loop.sce
// Description: Demo: rotating voltage vector (open loop) at 5 Hz electrical;
//              plots phase currents, speed and load angle.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

exec(fullfile(get_absolute_file_path("demo_open_loop.sce"), "..", "loader.sce"), -1);

mot = motor_17hs19_2004s1();
f_el = 5;                                  // electrical Hz (= 0.1 rev/s)
srcp = struct("V", mot.R * mot.I_rated, "f_el", f_el, "TL", 0);
t = 0:1e-4:0.4;
x = stepper_simulate(mot, src_open_loop, srcp, t);

p = mot.Ns / 4;
load_angle = 2 * %pi * f_el * t - p * x(4, :);   // electrical rad

scf(0); clf();
subplot(3, 1, 1);
plot(t, x(1, :), "b", t, x(2, :), "r");
xtitle("Phase currents", "t [s]", "i [A]");
legend(["iA"; "iB"]);
subplot(3, 1, 2);
plot(t, x(3, :) * 60 / (2 * %pi));
xtitle("Rotor speed", "t [s]", "n [rpm]");
subplot(3, 1, 3);
plot(t, load_angle * 180 / %pi);
xtitle("Load angle (electrical)", "t [s]", "[deg]");
