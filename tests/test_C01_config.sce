// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_C01_config.sce
// Description: C01: settings of the window (file missing, damaged, round trip) and the
//              search order of the Stepper-C folder. Runs in a temporary folder; the
//              environment variables are restored at the end.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

c01_env_set = getenv("SSTEP_LAB_SETTINGS", "");
c01_env_dir = getenv("SSTEP_CTRL_DIR", "");
c01_tmp = fullfile(TMPDIR, "c01");
mkdir(c01_tmp);
c01_file = fullfile(c01_tmp, "settings.json");
setenv("SSTEP_LAB_SETTINGS", c01_file);
setenv("SSTEP_CTRL_DIR", "");

function c01_make_lib(d)
    // the three files that make a Stepper-C folder
    mkdir(d);
    mkdir(fullfile(d, "inc"));
    mkdir(fullfile(d, "src"));
    mputl("//", fullfile(d, "sstep.h"));
    mputl("//", fullfile(d, "inc", "sstep_ctrl.h"));
    mputl("//", fullfile(d, "src", "sstep_ctrl.c"));
endfunction

// ---- defaults on a missing and on a damaged file
s = lab_settings_load();
sim_failures = sim_failures + sim_check("C01 missing settings file gives the defaults", ..
    (s.backend == "FIXED") & (s.ctrl_dir == "") & (s.stage_file == "") & (s.model == "SIL") & (s.scenario == 1), ..
    msprintf("backend %s, model %s, scenario %d", s.backend, s.model, s.scenario));
mputl("{broken", c01_file);
s = lab_settings_load();
sim_failures = sim_failures + sim_check("C01 damaged settings file gives the defaults", ..
    (s.backend == "FIXED") & (s.ctrl_dir == ""), "");

// ---- round trip, a path with a space and a non-ASCII character
c01_lib = fullfile(c01_tmp, "ścieżka test", "lib");
s = lab_settings_load();
s.ctrl_dir = c01_lib;
s.stage_file = fullfile(c01_tmp, "ścieżka test", "stage.json");
s.backend = "FLOAT";
s.model = "MIL";
s.scenario = 3;
lab_settings_save(s);
r = lab_settings_load();
sim_failures = sim_failures + sim_check("C01 settings round trip", ..
    (r.ctrl_dir == s.ctrl_dir) & (r.stage_file == s.stage_file) & (r.backend == "FLOAT") & ..
    (r.model == "MIL") & (r.scenario == 3), r.ctrl_dir);

// ---- a value of another type or out of the list gives its default
mputl("{""backend"": ""X"", ""model"": 3, ""scenario"": ""a"", ""ctrl_dir"": 5}", c01_file);
s = lab_settings_load();
sim_failures = sim_failures + sim_check("C01 invalid values give the defaults", ..
    (s.backend == "FIXED") & (s.model == "SIL") & (s.scenario == 1) & (s.ctrl_dir == ""), "");

// ---- the check of a folder names the missing file
c01_make_lib(c01_lib);
ok = sil_check_ctrl_dir(c01_lib) == "";
for f = ["sstep.h", "sstep_ctrl.h", "sstep_ctrl.c"]
    d = c01_lib + "_" + part(f, 1:$ - 2);
    c01_make_lib(d);
    if f == "sstep.h" then
        mdelete(fullfile(d, f));
    elseif f == "sstep_ctrl.h" then
        mdelete(fullfile(d, "inc", f));
    else
        mdelete(fullfile(d, "src", f));
    end
    ok = ok & (strindex(sil_check_ctrl_dir(d), f) <> []);
end
sim_failures = sim_failures + sim_check("C01 folder check names the missing file", ok, "");

// ---- search order
c01_env_lib = fullfile(c01_tmp, "env lib");
c01_make_lib(c01_env_lib);
s = lab_settings_load();
s.ctrl_dir = c01_lib;
lab_settings_save(s);
setenv("SSTEP_CTRL_DIR", c01_env_lib);
d1 = sil_ctrl_dir();
setenv("SSTEP_CTRL_DIR", "");
d2 = sil_ctrl_dir();
s.ctrl_dir = fullfile(c01_tmp, "deleted");
lab_settings_save(s);
d3 = "not found";
if execstr("d3 = sil_ctrl_dir();", "errcatch") <> 0 then
    d3 = "not found";
end
sim_failures = sim_failures + sim_check("C01 Stepper-C folder: environment, saved, then the next places", ..
    (d1 == c01_env_lib) & (d2 == c01_lib) & (d3 <> s.ctrl_dir), ..
    msprintf("after a deleted saved folder: %s", d3));

// ---- no folder anywhere: a clear error, and a prebuilt library does not pass unnoticed
global sim_root_g;
c01_root = sim_root_g;
sim_root_g = fullfile(c01_tmp, "root", "repo");
mkdir(fullfile(c01_tmp, "root"));
mkdir(sim_root_g);
c01_err = execstr("sil_ctrl_dir();", "errcatch");
c01_msg = lasterror();
c01_none = sim_config_report(%t);
mkdir(fullfile(sim_root_g, "sil"));
mkdir(fullfile(sim_root_g, "sil", "build"));
mkdir(fullfile(sim_root_g, "sil", "build", "fixed"));
mputl("//", fullfile(sim_root_g, "sil", "build", "fixed", "loader.sce"));
c01_old = sim_config_report(%t);
// a clone of the public repository is a folder named Stepper-C
c01_make_lib(fullfile(c01_tmp, "root", "Stepper-C"));
c01_pub = "";
execstr("c01_pub = sil_ctrl_dir();", "errcatch");
sim_root_g = c01_root;
sim_failures = sim_failures + sim_check("C01 a sibling clone named Stepper-C is found", ..
    strindex(c01_pub, "Stepper-C") <> [], c01_pub);
sim_failures = sim_failures + sim_check("C01 no Stepper-C folder: the error names the window", ..
    (c01_err <> 0) & (strindex(c01_msg, "start_lab.sce") <> []), c01_msg);
sim_failures = sim_failures + sim_check("C01 no folder, no build: not a failure; an old build alone: a failure", ..
    (c01_none == 0) & (c01_old == 1), msprintf("%d, %d", c01_none, c01_old));

setenv("SSTEP_LAB_SETTINGS", c01_env_set);
setenv("SSTEP_CTRL_DIR", c01_env_dir);
