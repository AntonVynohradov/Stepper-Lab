// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tools/docs/export_figures.sce
// Description: Writes the data of the result figures of the documentation to
//              docs/assets/data/*.csv: real runs, with the setups of the tests. Run it after
//              a change of the model, then python tools/docs/diagrams.py:
//                  WScilex-cli -quit -f tools/docs/export_figures.sce
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

exec("loader.sce", -1);
getd(fullfile(sim_root, "tests"));
ef_dir = fullfile(sim_root, "docs", "assets", "data");
sil_build("FIXED");

function ef_write(name, source, head, m, fmt)
    // name file, source comment line, head column names, m values, fmt format of one number
    lines = ["# " + source; strcat(head, ",")];
    row = strcat(emptystr(1, size(m, 2)) + fmt, ",");
    body = msprintf(row + "\n", m);
    mputl([lines; body], fullfile(ef_dir, name));
    mprintf("wrote %s: %d rows\n", name, size(m, 1));
endfunction

mot = motor_17hs19_2004s1();
brd = board_bg431b_esc1();
c = ctrl_default();
mp = motion_default();

// ---- lab scenario 1: current step, rotor locked, SIL and MIL
rs = lab_run("SIL", 1);
rm = lab_run("MIL", 1);
n = min(size(rs.values, 1), size(rm.values, 1));
ef_write("current_step.csv", "lab scenario 1: current step 0.5 to 2 A, rotor locked (lab_run)", ..
         ["t_s", "iA_ref_A", "iA_sil_A", "iA_mil_A"], ..
         [rs.time(1:n), rs.values(1:n, 3), rs.values(1:n, 1), rm.values(1:n, 1)], "%.6g");

// ---- lab scenario 8: stall at the hold current, SIL, around the fault
rs = lab_run("SIL", 8);
d = sil_report_data(rs);
mprintf("scenario 8: %s at %.1f ms\n", d.fault_names, d.t_fault * 1e3);
sel = (rs.time >= 0.45) & (rs.time <= d.t_fault + 0.02);
ef_write("stall_trip.csv", "lab scenario 8: 0.12 N*m, run 1.5 A, hold 0.5 A after 0.5 s, stall detection on (lab_run)", ..
         ["t_s", "iA_A", "iB_A", "angle_deg", "faults", "level_pct"], ..
         [rs.time(sel), rs.values(sel, [1, 2, 6, 14, 19])], "%.6g");

// ---- amplitude against speed: setup of T09, both topologies
speeds = [4, 6, 8, 10];
tops = ["full", "shared"];
amp = zeros(size(speeds, "*"), 2);
for j = 1:2
    b = brd;
    b.topology = tops(j);
    for k = 1:size(speeds, "*")
        v = speeds(k) * mot.Ns * mp.M;
        m = mp;
        m.v_max = v;
        m.acc = 320000;
        m.dec = 320000;
        st = motion_init(mot, b, m, [0, 2, v]);
        T_on = v / m.acc;
        rec = sim_closed_loop(mot, b, c, ref_motion, [], T_on + 0.1, %f, st);
        sel = rec(:, 1) >= T_on + 0.05;
        amp(k, j) = mean(sqrt(rec(sel, 2).^2 + rec(sel, 3).^2));
        mprintf("%s %d rev/s: %.3f A\n", tops(j), speeds(k), amp(k, j));
    end
end
ef_write("current_speed.csv", "setup of T09: 24 V, 2 A, bandwidth 1 kHz, unloaded rotor (sim_closed_loop)", ..
         ["speed_rev_s", "full_A", "shared_A"], [speeds', amp], "%.5g");

// ---- detent sweep: setup of T11
md = mot;
md.Td = 0.02;
sp_list = 0.8:0.1:1.8;
osc = zeros(sp_list);
for k = 1:size(sp_list, "*")
    sp = struct("I", 2, "rev_s", sp_list(k), "t_ramp", 0.1, "Ns", mot.Ns);
    rec = sim_closed_loop(md, brd, c, ref_speed_ramp, sp, 0.3);
    d = sim_load_angle(rec(rec(:, 1) >= 0.2, :), mot);
    osc(k) = (max(d) - min(d)) / 2;
end
ef_write("resonance.csv", "setup of T11: detent 0.02 N*m (assumption), 2 A, shared leg (sim_closed_loop)", ..
         ["speed_rev_s", "osc_deg"], [sp_list', osc'], "%.5g");
mprintf("done\n");
exit(0);
