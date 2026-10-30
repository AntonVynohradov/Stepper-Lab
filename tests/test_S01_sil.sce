// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_S01_sil.sce
// Description: S01..S03: software in the loop. The Xcos diagram (Stepper-C C code +
//              motor model) is simulated without windows and compared with the
//              Scilab prototype of the same algorithm, sample by sample.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

sil_can_run = %t;
if isempty(sil_linked_backend) then
    if ~isfile(fullfile(sim_root, "sil", "build", "fixed", "loader.sce")) & ~haveacompiler() then
        mprintf("SKIP  S01..S04: no C compiler and no built library (README: Software in the loop)\n");
        sil_can_run = %f;
    end
end

if sil_can_run then
    sil_build("FIXED");
    scs_m = sil_diagram();
    mot = motor_17hs19_2004s1();
    brd = board_bg431b_esc1();
    c = ctrl_default();
    mp = motion_default();
    p = mot.Ns / 4;

    // ---- S01 current step 0 -> 2 A at enable, locked rotor
    ctx = struct("cmds", zeros(0, 3), "loads", [0, 0], "locked", 1, "T_end", 0.006);
    scicos_simulate(scs_m, list(), ctx, "nw");
    v = sil.values;
    refp = struct("I0", 2, "I1", 2, "t_step", 0, "phi", 0);
    rec = sim_closed_loop(mot, brd, c, ref_current_step, refp, 0.006, %t);
    n = min(size(v, 1), size(rec, 1));
    di = max(abs(v(1:n, 1:2) - rec(1:n, 2:3)));
    sim_failures = sim_failures + sim_check("S01 SIL current step equals the prototype", ..
        (n >= 119) & (di < 5e-3), ..
        msprintf("%d samples, max current difference %.2f mA (ADC step 1.95 mA)", n, di * 1e3));

    // ---- S02 short move (640 microsteps), free rotor
    ctx = struct("cmds", [0, 1, 640], "loads", [0, 0], "locked", 0, "T_end", 0.15);
    scicos_simulate(scs_m, list(), ctx, "nw");
    v = sil.values;
    st = motion_init(mot, brd, mp, [0, 1, 640]);
    rec = sim_closed_loop(mot, brd, c, ref_motion, [], 0.15, %f, st);
    n = min(size(v, 1), size(rec, 1));
    di = max(abs(v(1:n, 1:2) - rec(1:n, 2:3)));
    dphi = max(abs(v(1:n, 7) - rec(1:n, 12) / (%pi / 2) * mp.M));          // microsteps
    drot = max(abs(v(1:n, 8) - p * rec(1:n, 5) / (%pi / 2) * mp.M));       // microsteps
    sim_failures = sim_failures + sim_check("S02 SIL move equals the prototype", ..
        (n >= 2999) & (di < 5e-3) & (dphi < 1e-6) & (drot < 0.05), ..
        msprintf("%d samples, max diff: current %.2f mA, commanded %.1e usteps, rotor %.4f usteps", ..
                 n, di * 1e3, dphi, drot));

    // ---- S03 voltage limit at 8 rev/s: two H-bridges and shared leg
    amp = zeros(1, 2);
    topos = ["full", "shared"];
    for n_t = 1:2
        brd2 = brd;
        brd2.topology = topos(n_t);
        mp2 = mp;
        mp2.v_max = 25600;
        mp2.acc = 320000;
        mp2.dec = 320000;
        ctx = struct("cmds", [0, 2, 25600], "brd", brd2, "mp", mp2, "loads", [0, 0], ..
                     "locked", 0, "T_end", 0.15);
        scicos_simulate(scs_m, list(), ctx, "nw");
        sel = sil.time >= 0.1;
        amp(n_t) = mean(sqrt(sil.values(sel, 1).^2 + sil.values(sel, 2).^2));
    end
    sim_failures = sim_failures + sim_check("S03 SIL 8 rev/s current as the prototype (T09)", ..
        (abs(amp(1) - 1.862) < 0.03) & (abs(amp(2) - 1.557) < 0.03), ..
        msprintf("full %.3f A (prototype 1.862), shared %.3f A (prototype 1.557)", amp(1), amp(2)));

    // ---- S04 switching power stage (dead time, diodes): run at 4 rev/s
    brd4 = brd;
    brd4.stage = "switching";
    ctx = struct("cmds", [0, 2, 12800], "brd", brd4, "loads", [0, 0], "locked", 0, "T_end", 0.1);
    scicos_simulate(scs_m, list(), ctx, "nw");
    v = sil.values;
    st = motion_init(mot, brd4, mp, [0, 2, 12800]);
    [rec, st, ext] = sim_closed_loop(mot, brd4, c, ref_motion, [], 0.1, %f, st);
    n = min(size(v, 1), size(rec, 1));
    di = max(abs(v(1:n, 1:2) - rec(1:n, 2:3)));
    pk_sil = max(v(1:n, 11:12));
    pk_pro = max(abs(ext(1:n, 1:4)));
    sim_failures = sim_failures + sim_check("S04 SIL switching stage equals the prototype", ..
        (n >= 1999) & (di < 5e-3) & (abs(pk_sil - pk_pro) < 5e-3), ..
        msprintf("%d samples, max current difference %.2f mA, peak %.3f A (prototype %.3f A)", ..
                 n, di * 1e3, pk_sil, pk_pro));
end
