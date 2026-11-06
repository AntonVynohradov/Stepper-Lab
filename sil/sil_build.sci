// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        sil/sil_build.sci
// Description: Compiles the Stepper-C library and the Xcos blocks (sil/c) into a
//              dynamic library and links it into Scilab.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function sil_build(backend, force)
    // backend  "FIXED" (Q15, MCUs without FPU) or "FLOAT" (MCUs with FPU)
    // force    %t: rebuild even if the library exists
    //
    // The sources are copied into sil/build/<backend>/ and compiled with the compiler
    // Scilab uses for its own toolboxes (Visual Studio on Windows, gcc on Linux).
    // The build is done once and repeated when a source file has changed; later calls
    // only link the existing library.
    global sim_root_g;
    global sil_linked_backend;

    [lhs, rhs] = argn(0);
    if rhs < 1 then
        backend = "FIXED";
    end
    if rhs < 2 then
        force = %f;
    end
    backend = convstr(backend, "u");
    if and(backend <> ["FIXED", "FLOAT"]) then
        error("sil_build: backend must be ""FIXED"" or ""FLOAT""");
    end
    if ~isempty(sil_linked_backend) then
        if sil_linked_backend <> backend then
            error(msprintf("The %s library is already linked in this Scilab session. " + ..
                           "Restart Scilab to switch to %s.", sil_linked_backend, backend));
        elseif ~force then
            return;                               // already linked
        end
    end

    bdir = fullfile(sim_root_g, "sil", "build", convstr(backend, "l"));
    loader = fullfile(bdir, "loader.sce");

    // sources of the build; a source changed since the last build triggers a rebuild
    src = [];
    try
        cdir = sil_ctrl_dir();
        mprintf("Stepper-C sources: %s\n", cdir);
        src = [fullfile(cdir, "sstep.h");
               listfiles(fullfile(cdir, "inc", "*.h"));
               listfiles(fullfile(cdir, "src", "*.h"));
               listfiles(fullfile(cdir, "src", "*.c"));
               fullfile(sim_root_g, "sil", "c", "sstep_plant.h");
               fullfile(sim_root_g, "sil", "c", "sstep_plant.c");
               fullfile(sim_root_g, "sil", "c", "sstep_xcos.c")];
    catch
        if ~isfile(loader) then
            error(lasterror());
        end
    end
    stale = %f;
    if isfile(loader) then
        for k = 1:size(src, "*")
            [fdir, fname, fext] = fileparts(src(k));
            bcopy = fullfile(bdir, fname + fext);
            if ~isfile(bcopy) then
                stale = %t;
            else
                la = mgetl(src(k));
                lb = mgetl(bcopy);
                if size(la, "*") <> size(lb, "*") then
                    stale = %t;
                elseif or(la <> lb) then
                    stale = %t;
                end
            end
        end
        if stale then
            mprintf("Sources changed since the last build: rebuilding.\n");
        end
    end

    if force | stale | ~isfile(loader) then
        if ~haveacompiler() then
            error(["No C compiler found by Scilab."
                   "Windows: install Visual Studio (or Build Tools) with ""Desktop development with C++"","
                   "then restart Scilab. Linux: install gcc."]);
        end
        mkdir(bdir);
        for k = 1:size(src, "*")
            copyfile(src(k), bdir);
        end
        // the makefile does not follow the headers: a changed structure with an old object
        // file overwrites memory, so a rebuild compiles every file
        objs = [listfiles(fullfile(bdir, "Release", "*.obj")); listfiles(fullfile(bdir, "*.o"))];
        for k = 1:size(objs, "*")
            mdelete(objs(k));
        end
        cfiles = listfiles(fullfile(bdir, "*.c"));
        files = [];
        for k = 1:size(cfiles, "*")
            [fdir, fname, fext] = fileparts(cfiles(k));
            files = [files, fname + fext];
        end

        cflags = ilib_include_flag(bdir);
        cflags = cflags + " -I""" + SCI + "/modules/scicos_blocks/includes""";
        cflags = cflags + " -I""" + SCI + "/modules/scicos/includes""";
        cflags = cflags + " -DSSTEP_CFG_MATH_BACKEND=SSTEP_MATH_BACKEND_" + backend;
        ldflags = "";
        if getos() == "Windows" then
            if findmsvccompiler() <> "unknown" then
                ldflags = """" + SCI + "/bin/scicos.lib""";
                cflags = cflags + " /std:c11";      // Stepper-C needs C11 (_Static_assert)
            end
        else
            cflags = cflags + " -std=gnu11";
        end
        mprintf("Building Stepper-C (%s) and the Xcos blocks in %s\n", backend, bdir);
        tbx_build_src(["sstep_xcos_ctrl", "sstep_xcos_plant", "sstep_xcos_monitor", ..
                       "sstep_xcos_check", "sstep_xcos_ctrl_io", "sstep_xcos_plant_io", ..
                       "sstep_xcos_monitor_io"], ..
                      files, "c", bdir, "", ldflags, cflags, "", "", ..
                      "sstep_sil_" + convstr(backend, "l"));
    end

    exec(loader, -1);
    sil_linked_backend = backend;
    mprintf("Stepper-C (%s) linked for Xcos.\n", backend);
endfunction
