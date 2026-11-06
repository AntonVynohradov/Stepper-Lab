// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_S06_stall.sce
// Description: S06: stall detection of the real Stepper-C C code in the Xcos diagram,
//              every scenario of sim_stall_cases on the averaged stage with the
//              ngspice leg voltage error and on the switching stage: the same
//              verdicts as T13 (must trip / must not trip), and the run equals the
//              prototype (T13, stall_proto) where it ran: currents within 5 mA (S01
//              to S04), the same fault at the same time (one slow period).
//              The library is linked once per Scilab session, so the other numeric
//              backend (FLOAT after FIXED) runs S01..S06 in a second Scilab process
//              (tests/run_sil_backend.sce).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

sil_can_run = %t;
if isempty(sil_linked_backend) then
    if ~isfile(fullfile(sim_root, "sil", "build", "fixed", "loader.sce")) & ~haveacompiler() then
        mprintf("SKIP  S06: no C compiler and no built library (README: Software in the loop)\n");
        sil_can_run = %f;
    end
end

if sil_can_run then
    if ~exists("sil_test_backend") then
        sil_test_backend = "FIXED";
    end
    sil_build(sil_test_backend);
    be = sil_linked_backend;
    scs_m = sil_diagram();
    ctl = ctrl_default();
    ctl.stall_time_ms = 50;
    cs = sim_stall_cases();
    have_proto = %f;
    if exists("stall_proto") then
        have_proto = size(stall_proto) == size(cs);
    end
    if ~have_proto then
        mprintf("SKIP  S06 %s comparison with the prototype: no T13 results\n", be);
    end
    for k = 1:size(cs)
        s = cs(k);
        for stg = ["avg", "switching"]
            [r, y] = sim_stall_sil(scs_m, s, stg, ctl);
            [ok, detail] = sim_stall_verdict(s, r, ctl);
            sim_failures = sim_failures + sim_check(msprintf("S06 %s %s [%s]", be, s.name, stg), ok, detail);
            if have_proto then
                pr = stall_proto(k)(stg);
                if ~isempty(pr) then
                    [di, de, dt] = sim_stall_compare(pr.r, pr.y, r, y);
                    if r(8) >= 0 then
                        ft = msprintf("%s at %.1f ms (prototype %s at %.1f ms)", sim_stall_flags(r(7)), ..
                                      r(8) * 1e3, sim_stall_flags(pr.r(7)), pr.r(8) * 1e3);
                    elseif pr.r(8) >= 0 then
                        ft = msprintf("no fault (prototype %s at %.1f ms)", sim_stall_flags(pr.r(7)), pr.r(8) * 1e3);
                    else
                        ft = "no fault in either";
                    end
                    if s.exact then
                        sim_failures = sim_failures + sim_check(msprintf("S06 %s %s [%s] equals the prototype", be, s.name, stg), ..
                            (di < 5e-3) & (dt <= 1e-3 + 1e-9), ..
                            msprintf("%s; synchronous part: current %.2f mA, back-EMF estimate %.0f mV", ft, di * 1e3, de));
                    else
                        // limit cycle: same verdict, the samples differ (printed)
                        sim_failures = sim_failures + sim_check(msprintf("S06 %s %s [%s] same verdict as the prototype", be, s.name, stg), ..
                            dt == 0, ..
                            msprintf("%s; unstable operating point, samples differ by up to %.0f mA (not compared)", ft, di * 1e3));
                    end
                end
            end
        end
    end

    // the other numeric backend in a second Scilab process
    if ~exists("sil_child") then
        other = "FLOAT";
        if be == "FLOAT" then
            other = "FIXED";
        end
        pfile = "";
        if have_proto then
            pfile = fullfile(TMPDIR, "stall_proto.sod");
            save(pfile, "stall_proto");
        end
        if getos() == "Windows" then
            exe = fullfile(SCI, "bin", "WScilex-cli.exe");
        else
            exe = fullfile(SCI, "bin", "scilab-cli");
        end
        setenv("SSTEP_SIL_BACKEND", other);
        setenv("SSTEP_STALL_PROTO", pfile);
        mprintf("Running S01..S06 with the %s backend in a second Scilab process\n", other);
        logf = fullfile(TMPDIR, "sil_backend.log");
        cmd = """" + exe + """ -quit -f """ + fullfile(sim_root, "tests", "run_sil_backend.sce") + ..
              """ > """ + logf + """ 2>&1";
        stat = host(cmd);
        rep = "";
        if isfile(logf) then
            rep = mgetl(logf);
        end
        lines = rep(grep(rep, ["PASS", "FAIL", "SKIP"]));
        for kl = 1:size(lines, "*")
            ln = lines(kl);
            if strindex(ln, " " + other + " ") == [] then
                ln = part(ln, 1:6) + "[" + other + "] " + part(ln, 7:length(ln));
            end
            mprintf("%s\n", ln);
        end
        nfail = size(grep(lines, "/^FAIL/", "r"), "*");
        if size(lines, "*") == 0 then
            mprintf("FAIL  S06 %s backend: the second Scilab process gave no result (status %d)\n", other, stat);
            mprintf("      %s\n", rep(max(1, size(rep, "*") - 5):$));
            nfail = 1;
        end
        sim_failures = sim_failures + nfail;
    end
end
