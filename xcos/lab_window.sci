// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        xcos/lab_window.sci
// Description: The window of stepper_lab: library folder, ngspice stage file, numeric
//              backend, MIL or SIL, scenarios, tests. The lab_cb_* functions hold the logic
//              and take plain values, the lab_ui_* callbacks only ask for them.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

// ---- pure functions

function g = lab_test_groups()
    // g  regular expression over the test file names for each group of the window
    g = struct("All", "", "SIL", "^test_S", "MIL", "^test_M", "Model", "^test_(T|B|X)");
endfunction

function r = lab_parse_log(lines)
    // lines  log of tests/run_all.sce
    // r      npass, nfail, nskip, done (the summary line is there), lines: FAIL, SKIP, PASS
    lines = lines(:);
    p = lines(part(lines, 1:4) == "PASS");
    f = lines(part(lines, 1:4) == "FAIL");
    s = lines(part(lines, 1:4) == "SKIP");
    r = struct("npass", size(p, "*"), "nfail", size(f, "*"), "nskip", size(s, "*"), ..
               "done", grep(lines, "failure(s)") <> [], "lines", [f; s; p]);
endfunction

function ok = lab_mil_available()
    // ok  %t when this version has the MIL controller
    ok = isdef("STEPPER_CTRL_MIL");
endfunction

function s = lab_stage_text()
    // s  bus voltage, frequency and points of the active stage, and a mismatch with the board
    stg = stage_active();
    brd = board_bg431b_esc1();
    s = msprintf("%g V, %g kHz, %d points", stg.vbus, stg.fs / 1000, size(stg.leg_error, 1));
    if (stg.vbus <> brd.vbus) | (stg.fs <> brd.fs) then
        s = s + msprintf(" - the board runs %g V, %g kHz", brd.vbus, brd.fs / 1000);
    end
endfunction

// ---- user interface helpers

function t = lab_ui_text(tag)
    // t  text of the control tag, "" when the window is closed
    t = "";
    h = findobj("tag", tag);
    if ~isempty(h) then
        t = strcat(h.string, " ");
    end
endfunction

function lab_ui_set(tag, varargin)
    // sets property, value pairs of the control tag; nothing when the window is closed
    h = findobj("tag", tag);
    if ~isempty(h) then
        for k = 1:2:size(varargin)
            set(h, varargin(k), varargin(k + 1));
        end
    end
endfunction

function lab_ui_status(txt)
    lab_ui_set("lab_status", "string", txt);
endfunction

function lab_ui_refresh()
    // shows the settings in the controls
    s = lab_settings_load();
    d = "not found";
    if execstr("d = sil_ctrl_dir();", "errcatch") <> 0 then
        d = "not found";
    end
    lab_ui_set("lab_lib_file", "string", d);
    if d == "not found" then
        lab_ui_set("lab_lib_status", "string", "Select the folder of Stepper-C");
    else
        lab_ui_set("lab_lib_status", "string", "OK");
    end
    [path, msg] = stage_source();
    if path == "" then
        path = "(default) params/stage_esc1_spice";
    end
    lab_ui_set("lab_stage_file", "string", path);
    lab_ui_set("lab_stage_info", "string", lab_stage_text());
    lab_ui_set("lab_tests_stage", "string", "Tests use: " + path);
    lab_ui_set("lab_backend", "value", find(s.backend == ["FIXED", "FLOAT"]));
    lab_ui_set("lab_sil", "value", bool2s(s.model == "SIL"));
    lab_ui_set("lab_mil", "value", bool2s(s.model == "MIL"));
    if s.scenario <= size(lab_scenarios()) then
        lab_ui_set("lab_scenario", "value", s.scenario);
    end
    if msg <> "" then
        lab_ui_status(msg);
    end
endfunction

function msg = lab_restart_note()
    // msg  text when the linked library differs from the selection, else ""
    global sil_linked_backend;
    msg = "";
    if ~isempty(sil_linked_backend) then
        if sil_linked_backend <> lab_settings_load().backend then
            msg = msprintf("Restart Scilab to use %s: this session has linked %s.", ..
                           lab_settings_load().backend, sil_linked_backend);
        end
    end
endfunction

// ---- logic of the controls

function lab_cb_library(d)
    // d  folder of the Stepper-C sources; a wrong one keeps the selection
    global sil_linked_backend;
    msg = sil_check_ctrl_dir(d);
    if msg <> "" then
        lab_ui_set("lab_lib_status", "string", msg);
        return;
    end
    s = lab_settings_load();
    s.ctrl_dir = d;
    lab_settings_save(s);
    lab_ui_set("lab_lib_file", "string", d);
    lab_ui_set("lab_lib_status", "string", "OK");
    if ~isempty(sil_linked_backend) then
        lab_ui_status("Restart Scilab to link the library of the new folder.");
    end
endfunction

function lab_cb_stage(path)
    // path  stage file; a wrong one keeps the selection and shows why
    if execstr("stage_load(path);", "errcatch") <> 0 then
        lab_ui_status(lasterror());
        return;
    end
    s = lab_settings_load();
    s.stage_file = path;
    lab_settings_save(s);
    lab_ui_status("");
    lab_ui_refresh();
endfunction

function lab_cb_stage_default()
    s = lab_settings_load();
    s.stage_file = "";
    lab_settings_save(s);
    lab_ui_status("");
    lab_ui_refresh();
endfunction

function lab_cb_backend(name)
    // name  "FIXED" or "FLOAT"
    s = lab_settings_load();
    s.backend = name;
    lab_settings_save(s);
    lab_ui_status(lab_restart_note());
endfunction

function lab_cb_model(name)
    // name  "SIL" or "MIL"
    if (name == "MIL") & ~lab_mil_available() then
        lab_ui_status("MIL is not available in this version.");
        name = "SIL";
    end
    s = lab_settings_load();
    s.model = name;
    lab_settings_save(s);
    lab_ui_set("lab_sil", "value", bool2s(name == "SIL"));
    lab_ui_set("lab_mil", "value", bool2s(name == "MIL"));
endfunction

function lab_cb_scenario(k)
    // k  index in lab_scenarios()
    s = lab_settings_load();
    s.scenario = k;
    lab_settings_save(s);
    lab_ui_set("lab_scenario", "value", k);
endfunction

function res = lab_cb_run()
    // res  results of the scenario; kept for the report
    global lab_last_result;
    s = lab_settings_load();
    res = [];
    lab_ui_status(msprintf("Running %s ...", s.model));
    if execstr("res = lab_run(s.model, s.scenario);", "errcatch") <> 0 then
        lab_ui_status(lasterror());
        return;
    end
    lab_last_result = res;
    d = sil_report_data(res);
    txt = msprintf("%s finished at %.3f s, no fault.", s.model, res.time($));
    if d.t_fault >= 0 then
        txt = msprintf("%s finished at %.3f s, %s at %.1f ms.", s.model, res.time($), d.fault_names, d.t_fault * 1e3);
    end
    lab_ui_status(txt);
endfunction

function f = lab_cb_report(visible)
    // visible  optional, default %t; f  the figure, [] without a run
    global lab_last_result;
    [lhs, rhs] = argn(0);
    if rhs < 1 then
        visible = %t;
    end
    f = [];
    if isempty(lab_last_result) then
        lab_ui_status("Run a scenario first.");
        return;
    end
    f = sil_report(lab_last_result, visible);
endfunction

function c = lab_compare_data(rs, rm)
    // rs, rm  results of the same scenario on SIL and MIL
    // c       c.panels: list of struct title, ylabel, legend, t, y, with the largest
    //         differences while neither run has a fault
    n = min(size(rs.values, 1), size(rm.values, 1));
    t = rs.time(1:n);
    ks = find(rs.values(1:n, 14) <> 0, 1);
    km = find(rm.values(1:n, 14) <> 0, 1);
    last = min([n, ks, km]);
    function p = panel(title, ylabel, col, scale, unit)
        d = max(abs(rs.values(1:last, col) - rm.values(1:last, col))) * scale;
        p = struct("title", msprintf("%s: largest difference %.3g %s", title, d, unit), "ylabel", ylabel, ..
                   "legend", ["SIL", "MIL"], "t", t, "y", [rs.values(1:n, col), rm.values(1:n, col)]);
    endfunction
    c = struct();
    c.panels = list();
    c.panels($ + 1) = panel("Current iA", "Current [A]", 1, 1000, "mA");
    c.panels($ + 1) = panel("Speed", "Speed [rpm]", 5, 1, "rpm");
    c.panels($ + 1) = panel("Commanded position", "Position [microsteps]", 7, 1, "microsteps");
endfunction

function f = lab_cb_compare(visible)
    // runs the scenario on both models and overlays them; f  the figure
    // visible  optional, default %t
    [lhs, rhs] = argn(0);
    if rhs < 1 then
        visible = %t;
    end
    f = [];
    s = lab_settings_load();
    lab_ui_status("Running SIL and MIL ...");
    if execstr("rs = lab_run(""SIL"", s.scenario); rm = lab_run(""MIL"", s.scenario);", "errcatch") <> 0 then
        lab_ui_status(lasterror());
        return;
    end
    c = lab_compare_data(rs, rm);
    f = figure("visible", "off", "figure_name", "Stepper-Lab MIL / SIL", "background", -2);
    drawlater();
    for k = 1:size(c.panels)
        p = c.panels(k);
        subplot(size(c.panels), 1, k);
        plot(p.t, p.y);
        a = gca();
        a.grid = [color("gray"), color("gray")];
        title(p.title);
        ylabel(p.ylabel);
        xlabel("Time [s]");
        legend(p.legend, "lower_caption");
    end
    drawnow();
    if visible then
        f.visible = "on";
    end
    lab_ui_status(c.panels(1).title);
endfunction

function lab_cb_open()
    // the scenario in Xcos, from a temporary file with its context
    s = lab_settings_load();
    if execstr("scs_m = lab_scenario_diagram(s.model, s.scenario);", "errcatch") <> 0 then
        lab_ui_status(lasterror());
        return;
    end
    p = fullfile(TMPDIR, "stepper_lab_scenario.zcos");
    xcosDiagramToScilab(p, scs_m);
    xcos(p);
    lab_ui_status(msprintf("Opened in Xcos: %s", lab_scenarios()(s.scenario).name));
endfunction

function lab_cb_run_tests(group, filter)
    // group   name of lab_test_groups
    // filter  optional regular expression instead of the group
    global sim_root_g;
    [lhs, rhs] = argn(0);
    if rhs < 2 then
        filter = lab_test_groups()(group);
    end
    logf = fullfile(TMPDIR, "lab_tests.log");
    if isfile(logf) then
        mdelete(logf);
    end
    setenv("SSTEP_TEST_FILTER", filter);
    script = fullfile(sim_root_g, "tests", "run_all.sce");
    if getos() == "Windows" then
        exe = fullfile(SCI, "bin", "WScilex-cli.exe");
        host("start """" /b """ + exe + """ -quit -f """ + script + """ > """ + logf + """ 2>&1");
    else
        exe = fullfile(SCI, "bin", "scilab-cli");
        host("""" + exe + """ -quit -f """ + script + """ > """ + logf + """ 2>&1 &");
    end
    setenv("SSTEP_TEST_FILTER", "");
    lab_ui_status(msprintf("Tests started (%s). Press Refresh to read the results.", group));
endfunction

function r = lab_cb_refresh()
    // r  lab_parse_log of the running or last test run
    logf = fullfile(TMPDIR, "lab_tests.log");
    lines = "";
    if isfile(logf) then
        lines = mgetl(logf);
    end
    if isempty(lines) then
        lines = "";
    end
    r = lab_parse_log(lines);
    state = "running";
    if r.done then
        state = "finished";
    end
    lab_ui_set("lab_tests_summary", "string", msprintf("PASS %d   FAIL %d   SKIP %d   (%s)", r.npass, r.nfail, r.nskip, state));
    if ~isempty(r.lines) then
        lab_ui_set("lab_tests_list", "string", strcat(r.lines, "|"));
    end
endfunction

// ---- callbacks that ask the user

function lab_ui_browse_library()
    d = uigetdir(pwd(), "Select the folder of Stepper-C");
    if d <> "" then
        lab_cb_library(d);
    end
endfunction

function lab_ui_browse_stage()
    p = uigetfile("*.json", pwd(), "Select the stage file of sfs-stepper-power-spice");
    if p <> "" then
        lab_cb_stage(p);
    end
endfunction

function lab_ui_backend()
    lab_cb_backend(["FIXED", "FLOAT"](gcbo.value));
endfunction

function lab_ui_scenario()
    lab_cb_scenario(gcbo.value);
endfunction

function lab_ui_run_tests()
    g = ["All", "SIL", "MIL", "Model"];
    lab_cb_run_tests(g(findobj("tag", "lab_tests_group").value));
endfunction

// ---- the window

function h = lab_window(visible)
    // visible  %f: hidden, for the tests (optional, default %t)
    // h        figure handle
    [lhs, rhs] = argn(0);
    if rhs < 1 then
        visible = %t;
    end
    w = 760;
    H = 640;
    h = figure("visible", "off", "figure_name", "Stepper-Lab", "dockable", "off", ..
               "infobar_visible", "off", "toolbar_visible", "off", "menubar_visible", "off", ..
               "axes_size", [w, H], "background", -2, "tag", "lab_window", "resize", "off");

    function c = ctl(style, txt, pos, varargin)
        c = uicontrol(h, "style", style, "string", txt, "position", pos, ..
                      "horizontalalignment", "left", "fontsize", 12, "backgroundcolor", [1, 1, 1]);
        if style == "pushbutton" then
            set(c, "backgroundcolor", [0.86, 0.90, 0.96]);     // on white a plain button looks like text
            set(c, "horizontalalignment", "center");
        end
        for k = 1:2:size(varargin)
            set(c, varargin(k), varargin(k + 1));
        end
    endfunction

    // configuration
    ctl("text", "Configuration", [16, H - 34, 300, 22], "fontweight", "bold");
    ctl("text", "Stepper-C library", [16, H - 64, 130, 24]);
    ctl("edit", "", [150, H - 64, 400, 24], "tag", "lab_lib_file", "enable", "off");
    ctl("pushbutton", "Browse", [556, H - 64, 80, 24], "callback", "lab_ui_browse_library()");
    ctl("text", "", [644, H - 64, 110, 24], "tag", "lab_lib_status");
    ctl("text", "ngspice stage", [16, H - 94, 130, 24]);
    ctl("edit", "", [150, H - 94, 400, 24], "tag", "lab_stage_file", "enable", "off");
    ctl("pushbutton", "Browse", [556, H - 94, 80, 24], "callback", "lab_ui_browse_stage()");
    ctl("pushbutton", "Use default", [644, H - 94, 100, 24], "callback", "lab_cb_stage_default()");
    ctl("text", "", [150, H - 120, 590, 22], "tag", "lab_stage_info");
    ctl("text", "Backend", [16, H - 150, 130, 24]);
    ctl("popupmenu", "FIXED|FLOAT", [150, H - 150, 120, 24], "tag", "lab_backend", "callback", "lab_ui_backend()");

    // model
    ctl("text", "Model", [16, H - 190, 300, 22], "fontweight", "bold");
    ctl("radiobutton", "SIL: C code", [16, H - 220, 180, 24], "tag", "lab_sil", "callback", "lab_cb_model(""SIL"")");
    mil = ctl("radiobutton", "MIL: Scilab prototype", [200, H - 220, 220, 24], "tag", "lab_mil", "callback", "lab_cb_model(""MIL"")");
    if ~lab_mil_available() then
        mil.enable = "off";
    end
    ctl("text", "Scenario", [16, H - 252, 130, 24]);
    names = [];
    cs = lab_scenarios();
    for k = 1:size(cs)
        names = [names, cs(k).name];
    end
    ctl("popupmenu", strcat(names, "|"), [150, H - 252, 400, 24], "tag", "lab_scenario", "callback", "lab_ui_scenario()");
    ctl("pushbutton", "Open in Xcos", [16, H - 286, 130, 28], "callback", "lab_cb_open()");
    ctl("pushbutton", "Run", [152, H - 286, 90, 28], "callback", "sil = lab_cb_run();");
    ctl("pushbutton", "Report", [248, H - 286, 90, 28], "callback", "lab_cb_report();");
    cmp = ctl("pushbutton", "Compare MIL / SIL", [344, H - 286, 170, 28], "tag", "lab_compare", "callback", "lab_cb_compare();");
    if ~lab_mil_available() then
        cmp.enable = "off";
    end

    // tests
    ctl("text", "Tests", [16, H - 326, 300, 22], "fontweight", "bold");
    ctl("popupmenu", "All|SIL|MIL|Model", [16, H - 356, 120, 24], "tag", "lab_tests_group", "value", 1);
    ctl("pushbutton", "Run tests", [142, H - 356, 100, 24], "callback", "lab_ui_run_tests()");
    ctl("pushbutton", "Refresh", [248, H - 356, 90, 24], "callback", "lab_cb_refresh();");
    ctl("text", "", [348, H - 356, 396, 24], "tag", "lab_tests_summary");
    ctl("text", "", [16, H - 384, 728, 22], "tag", "lab_tests_stage");
    ctl("listbox", "", [16, 40, 728, H - 430], "tag", "lab_tests_list");
    ctl("text", "", [16, 8, 728, 26], "tag", "lab_status");

    lab_ui_refresh();
    if visible then
        h.visible = "on";
    end
endfunction
