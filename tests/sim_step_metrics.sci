// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/sim_step_metrics.sci
// Description: Test helper: step response metrics.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [over, t_rise, t_set, e_ss] = sim_step_metrics(rec, t_step, target, col)
    // rec     simulation record (sim_closed_loop), column 1 = time
    // t_step  step time [s], target final value, col column of the signal
    // over    overshoot (fraction of target)
    // t_rise  time from the step to 90 % of target [s]
    // t_set   time from the step until the signal stays within +/-2 % [s]
    // e_ss    final error (fraction of target)
    t = rec(:, 1);
    y = rec(:, col);
    after = t >= t_step;
    over = (max(y(after)) - target) / target;
    outside = find(after & (abs(y - target) > 0.02 * target));
    if isempty(outside) then
        t_set = 0;
    else
        t_set = t(outside($)) - t_step + (t(2) - t(1));
    end
    idx = find(after & (y >= 0.9 * target), 1);
    t_rise = t(idx) - t_step;
    e_ss = abs(y($) - target) / target;
endfunction
