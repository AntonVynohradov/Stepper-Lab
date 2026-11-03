// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/run_sil_backend.sce
// Description: Software-in-the-loop tests S01..S06 with the numeric backend of the
//              environment variable SSTEP_SIL_BACKEND ("FIXED" or "FLOAT", default
//              "FLOAT"). Started by test_S06_stall.sce in a second Scilab process (a
//              session links one backend only); SSTEP_STALL_PROTO names a .sod file
//              with the T13 results (stall_proto) for the comparison. Exits with the
//              number of failures.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

rb_dir = get_absolute_file_path("run_sil_backend.sce");
exec(fullfile(rb_dir, "..", "loader.sce"), -1);
exec(fullfile(rb_dir, "sim_check.sci"), -1);
exec(fullfile(rb_dir, "sim_stall_cases.sci"), -1);
exec(fullfile(rb_dir, "sim_stall_run.sci"), -1);

sil_child = %t;                 // test_S06_stall.sce does not start another process
sil_test_backend = convstr(getenv("SSTEP_SIL_BACKEND", "FLOAT"), "u");
rb_proto = getenv("SSTEP_STALL_PROTO", "");
if rb_proto <> "" then
    if isfile(rb_proto) then
        load(rb_proto);
    end
end

rb_fail = 0;
rb_tests = ["test_S01_sil.sce", "test_S06_stall.sce"];
for rb_k = 1:size(rb_tests, "*")
    mprintf("\n== %s (%s)\n", rb_tests(rb_k), sil_test_backend);
    sim_failures = 0;
    rb_err = exec(fullfile(rb_dir, rb_tests(rb_k)), "errcatch", -1);
    if rb_err <> 0 then
        mprintf("FAIL  [%s] %s: Scilab error %d: %s\n", sil_test_backend, rb_tests(rb_k), rb_err, lasterror());
        rb_fail = rb_fail + 1;
    else
        rb_fail = rb_fail + sim_failures;
    end
end
mprintf("\n%d failure(s) with the %s backend\n", rb_fail, sil_test_backend);
exit(min(rb_fail, 125));
