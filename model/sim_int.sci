// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/sim_int.sci
// Description: Integer helpers with the C semantics of Stepper-C, on doubles (exact
//              for magnitudes below 2^53).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function q = sim_div_round(num, den)
    // round(num / den), half away from zero, den > 0 (sstep_ctrl.c div_round)
    if num >= 0 then
        q = floor((num + floor(den / 2)) / den);
    else
        q = -floor((-num + floor(den / 2)) / den);
    end
endfunction

function q = sim_c_div(num, den)
    // C integer division: truncation toward zero
    q = sign(num) * sign(den) * floor(abs(num) / abs(den));
endfunction

function r = sim_isqrt(x)
    // floor(sqrt(x)) for 0 <= x < 2^53 (corrects the rounding of sqrt)
    r = floor(sqrt(x));
    while r * r > x
        r = r - 1;
    end
    while (r + 1) * (r + 1) <= x
        r = r + 1;
    end
endfunction
