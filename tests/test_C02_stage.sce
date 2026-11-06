// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_C02_stage.sce
// Description: C02: the ngspice stage file: stage_load (default file, changed values, each
//              invalid file), the order of stage_active and its effect on the board and
//              the plant parameters. The environment variables are restored at the end.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

c02_env_set = getenv("SSTEP_LAB_SETTINGS", "");
c02_env_stage = getenv("SSTEP_STAGE_FILE", "");
c02_tmp = fullfile(TMPDIR, "c02 ścieżka");
mkdir(c02_tmp);
setenv("SSTEP_LAB_SETTINGS", fullfile(c02_tmp, "settings.json"));
setenv("SSTEP_STAGE_FILE", "");

c02_default = fullfile(sim_root, "params", "stage_esc1_spice.json");
c02_base = fromJSON(strcat(mgetl(c02_default), ""));

function p = c02_write(name, s)
    p = fullfile(c02_tmp, name + ".json");
    mputl(toJSON(s), p);
endfunction

function msg = c02_error(p)
    // the message of stage_load, "" when it accepts the file
    msg = "";
    if execstr("stage_load(p);", "errcatch") <> 0 then
        msg = lasterror();
    end
endfunction

// ---- the default file equals the generated function
a = stage_load(c02_default);
b = stage_esc1_spice();
d = 0;
for f = fieldnames(b)'
    d = max(d, max(abs(a(f) - b(f))));
end
sim_failures = sim_failures + sim_check("C02 default JSON equals stage_esc1_spice", d < 1e-12, msprintf("largest difference %.1e", d));

// ---- a changed file reaches the board and the plant parameters
s = c02_base;
s.leg_error_table(:, 2) = 2 * s.leg_error_table(:, 2);
s.vd = 0.5;
s.dead_eff_ns = 200;
c02_changed = c02_write("changed", s);
setenv("SSTEP_STAGE_FILE", c02_changed);
stg = stage_active();
brd = board_bg431b_esc1();
sim_failures = sim_failures + sim_check("C02 a selected file reaches the board", ..
    (max(abs(stg.leg_error(:, 2) - 2 * b.leg_error(:, 2))) < 1e-12) & (brd.vd == 0.5) & (brd.dead_ns == 200), ..
    msprintf("vd %.3f V, dead time %.0f ns", brd.vd, brd.dead_ns));
brd.leg_error = stg.leg_error;
rp = sil_plant_rpar(motor_17hs19_2004s1(), brd, [0, 0], 0, 4);
sim_failures = sim_failures + sim_check("C02 a selected table reaches the plant parameters", ..
    or(abs(rp - stg.leg_error(1, 2)) < 1e-12) & ~or(abs(rp - b.leg_error(1, 2)) < 1e-12), "");

// ---- each invalid file is refused, naming the field
ok = %t;
detail = "";
for f = ["vbus", "fs", "r_leg", "vd", "dead_timer_ns", "dead_eff_ns", "leg_error_table", "format"]
    s = rmfield(c02_base, f);
    m = c02_error(c02_write("no_" + f, s));
    if strindex(m, f) == [] then
        ok = %f;
        detail = detail + " " + f;
    end
end
sim_failures = sim_failures + sim_check("C02 a missing field is named", ok, detail);
s = c02_base;
s.format = 2;
sim_failures = sim_failures + sim_check("C02 another format is refused", strindex(c02_error(c02_write("format", s)), "format") <> [], "");
s = c02_base;
s.leg_error_table = s.leg_error_table($:-1:1, :);
sim_failures = sim_failures + sim_check("C02 a falling current is refused", strindex(c02_error(c02_write("falling", s)), "leg_error_table") <> [], "");
s = c02_base;
s.leg_error_table = [(1:17)', zeros(17, 1)];
sim_failures = sim_failures + sim_check("C02 too many points are refused", strindex(c02_error(c02_write("many", s)), "leg_error_table") <> [], "");
// ---- values out of range or not a single finite number
ok = %t;
detail = "";
bad = list(list("vbus", 0), list("vbus", -24), list("vbus", %nan), list("fs", 0), list("r_leg", -1), ..
           list("vd", -1), list("dead_timer_ns", -1), list("dead_eff_ns", -1), list("vd", [1, 2]));
for k = 1:size(bad)
    s = c02_base;
    s(bad(k)(1)) = bad(k)(2);
    p = fullfile(c02_tmp, msprintf("bad%d.json", k));
    txt = toJSON(s);
    txt = strsubst(txt, "null", "NaN");
    mputl(txt, p);
    if strindex(c02_error(p), bad(k)(1)) == [] then
        ok = %f;
        detail = detail + " " + bad(k)(1);
    end
end
s = c02_base;
s.leg_error_table(2, 2) = %inf;
p = fullfile(c02_tmp, "bad_table.json");
mputl(strsubst(toJSON(s), "null", "1e999"), p);
ok_table = c02_error(p) <> "";      // JSON has no infinity: refused as a table or as JSON
sim_failures = sim_failures + sim_check("C02 a value out of range is refused, naming the field", ok & ok_table, detail);

c02_broken = fullfile(c02_tmp, "broken.json");
mputl("{broken", c02_broken);
sim_failures = sim_failures + sim_check("C02 a damaged file is refused", c02_error(c02_broken) <> "", "");
sim_failures = sim_failures + sim_check("C02 a missing file is refused", c02_error(fullfile(c02_tmp, "none.json")) <> "", "");

// ---- order: environment, saved, default
s = c02_base;
s.vd = 0.6;
c02_saved = c02_write("saved", s);
st = lab_settings_load();
st.stage_file = c02_saved;
lab_settings_save(st);
v_env = stage_active().vd;              // SSTEP_STAGE_FILE still set
setenv("SSTEP_STAGE_FILE", "");
v_saved = stage_active().vd;
[p_saved, m_saved] = stage_source();
st.stage_file = "";
lab_settings_save(st);
v_default = stage_active().vd;
[p_default, m_default] = stage_source();
sim_failures = sim_failures + sim_check("C02 order: environment, saved, default", ..
    (v_env == 0.5) & (v_saved == 0.6) & (v_default == b.vd) & (p_saved == c02_saved) & (p_default == "") & (m_saved == "") & (m_default == ""), ..
    msprintf("%.3f, %.3f, %.3f V", v_env, v_saved, v_default));

// ---- a changed file is read again
s.vd = 0.7;
c02_write("saved", s);
st.stage_file = c02_saved;
lab_settings_save(st);
// file times have a resolution of a second on some file systems: the size changes as well
s.source = "changed " + strcat(string(1:40));
c02_write("saved", s);
sim_failures = sim_failures + sim_check("C02 a changed file is read again", stage_active().vd == 0.7, "");

// ---- a saved file that became invalid: default with a message
mputl("{broken", c02_saved);
v_invalid = stage_active().vd;
[p_invalid, m_invalid] = stage_source();
sim_failures = sim_failures + sim_check("C02 an invalid saved file gives the default and a message", ..
    (v_invalid == b.vd) & (p_invalid == "") & (m_invalid <> ""), m_invalid);

setenv("SSTEP_LAB_SETTINGS", c02_env_set);
setenv("SSTEP_STAGE_FILE", c02_env_stage);
