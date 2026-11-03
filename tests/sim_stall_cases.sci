// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/sim_stall_cases.sci
// Description: Test helper: scenarios of the stall detection tests (T13, S06),
//              shared by the Scilab prototype and the Xcos diagram.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function cs = sim_stall_cases()
    // cs  list of scenarios, each a structure:
    //     name     short name
    //     trip     %t: SSTEP_FAULT_STALL expected, %f: no fault expected
    //     cmds     commands [t kind value] (motion_init; kind 4 = load torque, the
    //              loads of the Xcos plant block)
    //     T        simulated time [s]
    //     locked   1: rotor locked
    //     mp       motion settings, mot plant motor, mot_c motor of the controller
    //     topology power stage topology
    //     proto_sw %t: also run the prototype on the switching stage (slow)
    //     exact    %f: an unstable operating point (mid-band limit cycle that grows
    //              from the ADC quantization): SIL and prototype are compared by
    //              their verdicts, not sample by sample
    mot = motor_17hs19_2004s1();
    mp = motion_default();
    cs = list();

    function s = mk(name, trip, cmds, T)
        s = struct("name", name, "trip", trip, "cmds", cmds, "T", T, "locked", 0, ..
                   "mp", mp, "mot", mot, "mot_c", mot, "topology", "shared", "proto_sw", %f, ..
                   "exact", %t);
    endfunction

    // ---- must trip
    // reported case (sfs-stepper-driver): 0.12 N*m held at 1.5 A, not at the 0.5 A hold
    // current after 500 ms
    s = mk("stall at hold 0.12 N*m, 1.5 -> 0.5 A", %t, [0, 4, 0.12], 0.62);
    s.mp.i_run = 1.5;
    s.mp.i_hold = 0.5;
    s.mp.hold_delay = 0.5;
    s.proto_sw = %t;
    cs($ + 1) = s;
    // T06: load step above the pull-out torque Kt I = 0.417 N*m at 1 rev/s
    s = mk("load step 0.5 N*m at 1 rev/s (T06)", %t, [0, 2, 3200; 0.15, 4, 0.5], 0.25);
    s.proto_sw = %t;
    cs($ + 1) = s;
    // overload at start: 0.5 N*m from the beginning, run 1 rev/s
    s = mk("overload at start 0.5 N*m, run 1 rev/s", %t, [0, 4, 0.5; 0, 2, 3200], 0.1);
    s.proto_sw = %t;
    cs($ + 1) = s;
    // blocked axis at start: rotor locked, run 1 rev/s
    s = mk("blocked rotor at start, run 1 rev/s", %t, [0, 2, 3200], 0.12);
    s.locked = 1;
    s.proto_sw = %t;
    cs($ + 1) = s;

    // ---- must not trip
    // enable and run -> hold current steps at rest, 2 -> 1 A, R +30 %, L -20 %
    s = mk("enable, run -> hold 2 -> 1 A, R +30 %, L -20 %", %f, zeros(0, 3), 0.3);
    s.mot.R = 1.3 * mot.R;
    s.mot.L = 0.8 * mot.L;
    cs($ + 1) = s;
    // reported currents with a load the hold current holds (Kt 0.5 A = 0.104 N*m)
    s = mk("enable, run -> hold 1.5 -> 0.5 A, 0.07 N*m", %f, [0, 4, 0.07], 0.7);
    s.mp.i_run = 1.5;
    s.mp.i_hold = 0.5;
    s.mp.hold_delay = 0.5;
    cs($ + 1) = s;
    // T05: move 2 rev at 5 rev/s, then hold
    cs($ + 1) = mk("move 6400 usteps, 5 rev/s, hold (T05)", %f, [0, 1, 6400], 0.8);
    // T07: reversal +4 -> -4 rev/s, stop
    s = mk("reversal +-4 rev/s, stop (T07)", %f, [0, 2, 12800; 0.15, 2, -12800; 0.3, 3, 0], 0.45);
    s.mp.v_max = 12800;
    s.proto_sw = %t;
    cs($ + 1) = s;
    // T09: 8 rev/s, voltage limit, both topologies
    for topo = ["full", "shared"]
        s = mk("8 rev/s, " + topo + " (T09)", %f, [0, 2, 25600], 0.2);
        s.mp.v_max = 25600;
        s.mp.acc = 320000;
        s.mp.dec = 320000;
        s.topology = topo;
        cs($ + 1) = s;
    end
    // T08: controller for the nominal motor, plant R +30 %, L -20 %, J x5, 0.15 N*m,
    // ramp to 4 rev/s in 0.1 s
    s = mk("4 rev/s, R +30 %, L -20 %, J x5, 0.15 N*m (T08)", %f, [0, 4, 0.15; 0, 2, 12800], 0.3);
    s.mot.R = 1.3 * mot.R;
    s.mot.L = 0.8 * mot.L;
    s.mot.J = 5 * mot.J;
    s.mp.v_max = 12800;
    s.mp.acc = 128000;
    cs($ + 1) = s;
    // T11b: detent torque 0.02 N*m, speeds around the resonance 4 p n = fn, 2 A,
    // ramp 0.1 s
    for n = 0.8:0.1:1.8
        v = round(n * mot.Ns * mp.M);
        s = mk(msprintf("detent 0.02 N*m, %.1f rev/s (T11)", n), %f, [0, 2, v], 0.3);
        s.mot.Td = 0.02;
        s.mp.acc = round(v / 0.1);
        cs($ + 1) = s;
    end
    // T11c: mid-band, 5, 8, 11 rev/s, 2 A, ramp 0.2 s
    for n = [5, 8, 11]
        v = round(n * mot.Ns * mp.M);
        s = mk(msprintf("mid-band %d rev/s (T11)", n), %f, [0, 2, v], 0.5);
        s.mp.v_max = v;
        s.mp.acc = round(v / 0.2);
        s.mp.dec = s.mp.acc;
        s.exact = n < 11;
        cs($ + 1) = s;
    end
endfunction

