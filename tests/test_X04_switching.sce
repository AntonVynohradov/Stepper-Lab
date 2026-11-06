// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_X04_switching.sce
// Description: X04: switching power stage, Scilab vs Python twin (tools/ref_switching.py).
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
brd.stage = "switching";
c = ctrl_default();
sp = struct("I", 2, "rev_s", 4, "t_ramp", 0.01, "Ns", mot.Ns);
[rec, st, ext] = sim_closed_loop(mot, brd, c, ref_speed_ramp, sp, 0.02);
rec = rec(1:10:$, :);
ext = ext(1:10:$, :);

// the stored reference comes from the default stage file: with another file the Python twin
// computes it from that file (SSTEP_PYTHON: the interpreter, it needs numpy and scipy)
x04_stage = stage_source();
x04_csv = fullfile(sim_root, "tests", "reference", "x04_switching_shared.csv");
x04_run = %t;
if x04_stage <> "" then
    x04_csv = fullfile(TMPDIR, "x04_stage.csv");
    x04_py = getenv("SSTEP_PYTHON", "python");
    x04_cmd = """" + x04_py + """ """ + fullfile(sim_root, "tools", "ref_switching.py") + ..
              """ --stage """ + x04_stage + """ --out """ + x04_csv + """";
    if host(x04_cmd) <> 0 then
        mprintf("SKIP  X04: the Python twin with numpy and scipy is needed to compute the reference of %s (SSTEP_PYTHON)\n", x04_stage);
        x04_run = %f;
    end
end

if x04_run then
csv_lines = mgetl(x04_csv);
ref = csvTextScan(csv_lines(3:$), ",", ".", "double");

ok_len = size(ref, 1) == size(rec, 1);
if ok_len then
    di = max(abs(rec(:, 2:3) - ref(:, 2:3)));
    dm = max(abs([rec(:, 4:5), ext(:, [2 4])] - ref(:, 4:7)));
    dv = max(abs(ext(:, 5:6) - ref(:, 8:9)));
else
    di = %inf;
    dm = %inf;
    dv = %inf;
end
sim_failures = sim_failures + sim_check("X04 Scilab vs Python twin (switching stage)", ..
    ok_len & (di < 1e-6) & (dm < 1e-5) & (dv < 1e-5), ..
    msprintf("%d samples, max diff i %.1e A, w/theta/peaks %.1e, mean v %.1e V", size(rec, 1), di, dm, dv));
end
