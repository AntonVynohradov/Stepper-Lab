// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/run_all.sce
// Description: Runs every test and prints a summary. Command line (CI):
//                scilab-cli -f tests/run_all.sce
//              exits with status 1 when a test fails (batch mode only).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

// Names used here start with rt_ so that the test scripts cannot overwrite them.
rt_dir = get_absolute_file_path("run_all.sce");
exec(fullfile(rt_dir, "..", "loader.sce"), -1);
exec(fullfile(rt_dir, "sim_check.sci"), -1);
exec(fullfile(rt_dir, "sim_step_metrics.sci"), -1);
exec(fullfile(rt_dir, "sim_load_angle.sci"), -1);

rt_tests = ["test_T00_conventions.sce"
            "test_T01_resistance.sce"
            "test_T02_time_constant.sce"
            "test_X01_reference.sce"
            "test_B01_bridge.sce"
            "test_T03_microstep.sce"
            "test_T04_current_step.sce"
            "test_X02_closed_loop.sce"
            "test_T05_speed_ramp.sce"
            "test_T06_load_step.sce"
            "test_T07_reversal.sce"
            "test_T08_parameters.sce"
            "test_T09_voltage_limit.sce"
            "test_X03_motion.sce"
            "test_T10_switching.sce"
            "test_X04_switching.sce"
            "test_T11_resonance.sce"
            "test_T12_friction.sce"
            "test_S01_sil.sce"];

rt_fail = 0;
for rt_k = 1:size(rt_tests, "*")
    mprintf("\n== %s\n", rt_tests(rt_k));
    sim_failures = 0;
    rt_err = exec(fullfile(rt_dir, rt_tests(rt_k)), "errcatch", -1);
    if rt_err <> 0 then
        mprintf("FAIL  %s: Scilab error %d: %s\n", rt_tests(rt_k), rt_err, lasterror());
        rt_fail = rt_fail + 1;
    else
        rt_fail = rt_fail + sim_failures;
    end
end

mprintf("\n%d failure(s)\n", rt_fail);

if getscilabmode() == "NWNI" then
    exit(bool2s(rt_fail > 0));
end
