// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/sim_check.sci
// Description: Test helper: prints PASS/FAIL and returns the number of failures.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function n = sim_check(name, cond, detail)
    // name    test name
    // cond    boolean result
    // detail  measured values, printed in both cases
    // n       0 on success, 1 on failure (add it to sim_failures)
    if cond then
        mprintf("PASS  %s: %s\n", name, detail);
        n = 0;
    else
        mprintf("FAIL  %s: %s\n", name, detail);
        n = 1;
    end
endfunction
