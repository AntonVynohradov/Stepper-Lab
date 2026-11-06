// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_B01_bridge.sce
// Description: B01: averaged power stage of both topologies: duty cycles in 0..1,
//              voltage direction kept, exact when feasible, largest circular amplitude.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

vbus = 24;
rand("seed", 2);
topos = ["full", "shared"];
for n_t = 1:2
    topo = topos(n_t);
    worst_dir = 0;
    worst_v = 0;
    duty_ok = %t;
    for n = 1:5000
        vr = 80 * rand(2, 1) - 40;
        [v, duty, k] = bridge_avg(vr, vbus, topo);
        duty_ok = duty_ok & and(duty >= -1e-12) & and(duty <= 1 + 1e-12);
        cr = abs(v(1) * vr(2) - v(2) * vr(1)) / (norm(vr) * max(norm(v), 1e-9));
        worst_dir = max(worst_dir, cr);
        if topo == "full" then
            dv = [duty(1) - duty(2); duty(3) - duty(4)] * vbus;
        else
            dv = [duty(1) - duty(3); duty(2) - duty(3)] * vbus;
        end
        worst_v = max(worst_v, max(abs(dv - v)));
        if k == 1 then
            worst_v = max(worst_v, max(abs(v - vr)));
        end
    end
    sim_failures = sim_failures + sim_check("B01 " + topo + ": duties in 0..1, direction kept, exact when feasible", ..
        duty_ok & (worst_dir < 1e-12) & (worst_v < 1e-12), ..
        msprintf("direction error %.1e, voltage error %.1e V", worst_dir, worst_v));
end

// Largest circular amplitude: full -> vbus, shared -> vbus / sqrt(2)
expected = [vbus, vbus / sqrt(2)];
for n_t = 1:2
    amp = %inf;
    for ph = linspace(0, 2 * %pi, 3601)
        v = bridge_avg(100 * [cos(ph); sin(ph)], vbus, topos(n_t));
        amp = min(amp, norm(v));
    end
    sim_failures = sim_failures + sim_check("B01 " + topos(n_t) + ": largest circular voltage amplitude", ..
        abs(amp - expected(n_t)) < 1e-9, ..
        msprintf("%.4f V (expected %.4f V)", amp, expected(n_t)));
end
