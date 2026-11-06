// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
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
exec(fullfile(rt_dir, "sim_stall_cases.sci"), -1);
exec(fullfile(rt_dir, "sim_stall_run.sci"), -1);
exec(fullfile(rt_dir, "sim_config_report.sci"), -1);

rt_tests = ["test_C01_config.sce"
            "test_C02_stage.sce"
            "test_T00_conventions.sce"
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
            "test_T13_stall.sce"
            "test_S01_sil.sce"
            "test_S07_lab.sce"
            "test_C03_window.sce"
            "test_M01_mil.sce"
            "test_S06_stall.sce"];

// SSTEP_TEST_FILTER: regular expression over the file names (no delimiters), empty: all
rt_filter = getenv("SSTEP_TEST_FILTER", "");
if rt_filter <> "" then
    rt_tests = rt_tests(grep(rt_tests, "/" + rt_filter + "/", "r"));
end

rt_fail = sim_config_report();
// a test that stops with an error must not change the configuration of the next ones
rt_env_names = ["SSTEP_LAB_SETTINGS", "SSTEP_CTRL_DIR", "SSTEP_STAGE_FILE"];
rt_env_values = [getenv(rt_env_names(1), ""), getenv(rt_env_names(2), ""), getenv(rt_env_names(3), "")];
for rt_k = 1:size(rt_tests, "*")
    mprintf("\n== %s\n", rt_tests(rt_k));
    sim_failures = 0;
    rt_err = exec(fullfile(rt_dir, rt_tests(rt_k)), "errcatch", -1);
    for rt_j = 1:3
        setenv(rt_env_names(rt_j), rt_env_values(rt_j));
    end
    if rt_err <> 0 then
        mprintf("FAIL  %s: Scilab error %d: %s\n", rt_tests(rt_k), rt_err, lasterror());
        rt_fail = rt_fail + 1;
    else
        rt_fail = rt_fail + sim_failures;
    end
end

// the selection may have changed during a long run
rt_fail = rt_fail + sim_config_report();
mprintf("\n%d failure(s)\n", rt_fail);

if or(getscilabmode() == ["NWNI", "NW"]) then
    exit(bool2s(rt_fail > 0));
end
