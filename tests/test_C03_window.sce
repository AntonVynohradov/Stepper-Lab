// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_C03_window.sce
// Description: C03: the window of stepper_lab, created invisible, its callbacks called
//              directly with the values a dialog would give (the look and the file
//              dialogs are checked by hand). The environment variables are restored.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

c03_env_set = getenv("SSTEP_LAB_SETTINGS", "");
c03_env_stage = getenv("SSTEP_STAGE_FILE", "");
c03_tmp = fullfile(TMPDIR, "c03");
mkdir(c03_tmp);
setenv("SSTEP_LAB_SETTINGS", fullfile(c03_tmp, "settings.json"));
setenv("SSTEP_STAGE_FILE", "");

// ---- pure functions: test groups and the log of a run
g = lab_test_groups();
sim_failures = sim_failures + sim_check("C03 test groups", ..
    (g.All == "") & (g.SIL == "^test_S") & (g.MIL == "^test_M") & (g.Model == "^test_(T|B|X)"), "");
r = lab_parse_log(["== test_T00.sce"; "PASS  a: x"; "FAIL  b: y"; "SKIP  c: z"; "PASS  d: x"; "1 failure(s)"]);
sim_failures = sim_failures + sim_check("C03 the log is parsed, FAIL first", ..
    (r.npass == 2) & (r.nfail == 1) & (r.nskip == 1) & r.done & (part(r.lines(1), 1:4) == "FAIL") & (part(r.lines(2), 1:4) == "SKIP"), ..
    msprintf("%d PASS, %d FAIL, %d SKIP", r.npass, r.nfail, r.nskip));
r = lab_parse_log("PASS  a");
sim_failures = sim_failures + sim_check("C03 a log without the summary is not finished", ~r.done, "");

c03_h = [];
if execstr("c03_h = lab_window(%f);", "errcatch") <> 0 then
    mprintf("SKIP  C03 window: %s\n", lasterror());
else
    // ---- library folder
    c03_lib = fullfile(c03_tmp, "my lib");
    mkdir(c03_lib);
    mkdir(fullfile(c03_lib, "inc"));
    mkdir(fullfile(c03_lib, "src"));
    mputl("//", fullfile(c03_lib, "sstep.h"));
    mputl("//", fullfile(c03_lib, "inc", "sstep_ctrl.h"));
    mputl("//", fullfile(c03_lib, "src", "sstep_ctrl.c"));
    lab_cb_library(c03_lib);
    t1 = lab_ui_text("lab_lib_status");
    s1 = lab_settings_load().ctrl_dir;
    lab_cb_library(fullfile(c03_tmp, "none"));
    t2 = lab_ui_text("lab_lib_status");
    s2 = lab_settings_load().ctrl_dir;
    sim_failures = sim_failures + sim_check("C03 a valid folder is saved, a wrong one refused", ..
        (s1 == c03_lib) & (strindex(t1, "OK") <> []) & (s2 == c03_lib) & (strindex(t2, "sstep.h") <> []), t2);
    st = lab_settings_load();
    st.ctrl_dir = "";
    lab_settings_save(st);

    // ---- ngspice stage file
    c03_base = fromJSON(strcat(mgetl(fullfile(sim_root, "params", "stage_esc1_spice.json")), ""));
    c03_stage = fullfile(c03_tmp, "stage a.json");
    mputl(toJSON(c03_base), c03_stage);
    lab_cb_stage(c03_stage);
    t1 = lab_ui_text("lab_stage_info");
    s1 = lab_settings_load().stage_file;
    lab_cb_stage(fullfile(c03_tmp, "none.json"));
    t2 = lab_ui_text("lab_lib_status") + lab_ui_text("lab_status");
    s2 = lab_settings_load().stage_file;
    sim_failures = sim_failures + sim_check("C03 a stage file shows its values, a wrong one is refused", ..
        (s1 == c03_stage) & (s2 == c03_stage) & (strindex(t1, "24 V") <> []) & (strindex(t1, "10 points") <> []) & (strindex(t2, "none.json") <> []), t1);
    s48 = c03_base;
    s48.vbus = 48;
    c03_48 = fullfile(c03_tmp, "stage 48.json");
    mputl(toJSON(s48), c03_48);
    lab_cb_stage(c03_48);
    t1 = lab_ui_text("lab_stage_info");
    sim_failures = sim_failures + sim_check("C03 another bus voltage than the board is shown", strindex(t1, "board") <> [], t1);
    lab_cb_stage_default();
    sim_failures = sim_failures + sim_check("C03 use default", ..
        (lab_settings_load().stage_file == "") & (strindex(lab_ui_text("lab_stage_file"), "(default)") <> []), lab_ui_text("lab_stage_file"));

    // ---- run, report, backend
    k = 1;
    lab_cb_scenario(k);
    c03_res = lab_cb_run();
    ok = (lab_settings_load().scenario == k) & (c03_res.time($) > 0.02) & (size(c03_res.values, 2) == 23);
    sim_failures = sim_failures + sim_check("C03 a scenario runs from the window", ok, lab_ui_text("lab_status"));
    c03_f = lab_cb_report(%f);
    sim_failures = sim_failures + sim_check("C03 the report of the last run", typeof(c03_f) == "handle", "");
    delete(c03_f);
    lab_cb_backend("FLOAT");
    t1 = lab_ui_text("lab_status");
    sim_failures = sim_failures + sim_check("C03 a change of backend after linking asks for a restart", ..
        (lab_settings_load().backend == "FLOAT") & (strindex(t1, "Restart") <> []), t1);
    lab_cb_backend("FIXED");

    // ---- MIL and the comparison
    lab_cb_model("MIL");
    c03_res = lab_cb_run();
    ok = (lab_settings_load().model == "MIL") & (size(c03_res.values, 2) == 23) & (strindex(lab_ui_text("lab_status"), "MIL") <> []);
    sim_failures = sim_failures + sim_check("C03 MIL runs from the window", ok, lab_ui_text("lab_status"));
    c03_f = lab_cb_compare(%f);
    sim_failures = sim_failures + sim_check("C03 the comparison of MIL and SIL", ..
        (typeof(c03_f) == "handle") & (strindex(lab_ui_text("lab_status"), "largest difference") <> []), lab_ui_text("lab_status"));
    delete(c03_f);
    lab_cb_model("SIL");

    // ---- tests started from the window, in a second process
    lab_cb_run_tests("All", "C01");
    c03_done = %f;
    for kw = 1:120
        c03_log = lab_cb_refresh();
        if c03_log.done then
            c03_done = %t;
            break;
        end
        sleep(1000);
    end
    sim_failures = sim_failures + sim_check("C03 tests run in a second process, the log is read", ..
        c03_done & (c03_log.npass > 0) & (c03_log.nfail == 0), msprintf("%d PASS, %d FAIL", c03_log.npass, c03_log.nfail));

    sim_failures = sim_failures + sim_check("C03 the test group list starts on a group", ..
        findobj("tag", "lab_tests_group").value == 1, "");

    delete(c03_h);
end

setenv("SSTEP_LAB_SETTINGS", c03_env_set);
setenv("SSTEP_STAGE_FILE", c03_env_stage);
