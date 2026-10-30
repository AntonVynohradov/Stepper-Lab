// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/motion_step.sci
// Description: One slow period of the motion generator: bit-exact twin of
//              sstep_motion_step() (Stepper-C, src/sstep_motion.c).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [st, done] = motion_step(st)
    // Trapezoidal profile in integer units. A move aims at the largest v_new for which
    // the distance of this period, (v + v_new) / 2, plus the braking distance from v_new,
    // v_new^2 / (2 dec), fits into the distance left:
    //   v_new = (-dec + sqrt(dec^2 + 8 dec |rem| - 4 dec |v|)) / 2
    // No overshoot; |dv| <= acc or dec per period, except the last period of a move
    // (stop on the target from v <= 2 dec).
    done = %f;

    if st.mode == 1 then
        rem = st.target - st.pos;
        arem = abs(rem);
        av = abs(st.vel);
        d = st.dec;
        if arem >= st.brake_vmax + floor((st.v_max + av) / 2) then
            mag = st.v_max;
        else
            disc = d * d + 8 * d * arem - 4 * d * av;
            if disc > 0 then
                root = sim_isqrt(disc);
            else
                root = 0;
            end
            if root > d then
                mag = floor((root - d) / 2);
            else
                mag = 0;
            end
        end
        vt = sign(rem) * mag;
    elseif st.mode == 2 then
        vt = max(-st.v_max, min(st.v_max, st.v_run));
    else
        vt = 0;
    end

    speeding_up = (abs(vt) > abs(st.vel)) & ((st.vel == 0) | ((vt > 0) == (st.vel > 0)));
    if speeding_up then
        a = st.acc;
    else
        a = st.dec;
    end
    v_new = st.vel + max(-a, min(a, vt - st.vel));
    p_new = st.pos + floor((st.vel + v_new) / 2);

    if st.mode == 1 then
        rem0 = st.target - st.pos;
        rem1 = st.target - p_new;
        crossed = ((rem0 >= 0) & (rem1 <= 0)) | ((rem0 <= 0) & (rem1 >= 0));
        near = (abs(rem1) < 64) & (abs(v_new) <= st.dec);
        if crossed | near then
            p_new = st.target;
            v_new = 0;
            st.mode = 0;
            done = %t;
        end
    end

    st.pos = p_new;
    st.vel = v_new;
endfunction
