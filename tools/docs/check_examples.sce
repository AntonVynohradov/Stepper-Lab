// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tools/docs/check_examples.sce
// Description: Runs every Scilab example of the documentation (docs/*.md): the blocks of a
//              file run in order, in one workspace, after loader.sce. A block whose first
//              line is "// no-run" is skipped (it opens windows). Exit status 1 on an error.
//                  WScilex-cli -quit -f tools/docs/check_examples.sce
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

ce_root = getenv("SSTEP_DOCS_ROOT", "");
if ce_root == "" then
    ce_root = pwd();
end
chdir(ce_root);
exec("loader.sce", -1);

function [ok, msg, nrun, nskip] = ce_check_file(ce_path)
    // the examples of one file run in this function, so that files do not share variables
    ok = %t;
    msg = "";
    nrun = 0;
    nskip = 0;
    ce_lines = mgetl(ce_path);
    ce_k = 1;
    ce_block = 0;
    while ce_k <= size(ce_lines, "*")
        if stripblanks(ce_lines(ce_k)) == "```scilab" then
            ce_block = ce_block + 1;
            ce_first = ce_k + 1;
            ce_k = ce_first;
            while ce_k <= size(ce_lines, "*") & stripblanks(ce_lines(ce_k)) <> "```"
                ce_k = ce_k + 1;
            end
            ce_code = ce_lines(ce_first:ce_k - 1);
            if ce_code <> [] then
                if stripblanks(ce_code(1)) == "// no-run" then
                    nskip = nskip + 1;
                else
                    if execstr(ce_code, "errcatch", "n") <> 0 then
                        ok = %f;
                        msg = msprintf("block %d (line %d): %s", ce_block, ce_first, lasterror());
                        return;
                    end
                    nrun = nrun + 1;
                end
            end
        end
        ce_k = ce_k + 1;
    end
endfunction

ce_fail = 0;
ce_files = findfiles(fullfile(ce_root, "docs"), "*.md");
for ce_i = 1:size(ce_files, "*")
    [ce_ok, ce_msg, ce_n, ce_s] = ce_check_file(fullfile(ce_root, "docs", ce_files(ce_i)));
    if ce_ok then
        if ce_n + ce_s > 0 then
            mprintf("PASS  %s: %d run, %d skipped\n", ce_files(ce_i), ce_n, ce_s);
        end
    else
        mprintf("FAIL  %s: %s\n", ce_files(ce_i), ce_msg);
        ce_fail = ce_fail + 1;
    end
end
mprintf("%d file(s) failed\n", ce_fail);
exit(bool2s(ce_fail > 0));
