// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/motion_slow.sci
// Description: Slow loop of the motion generator: commands, profile, hold current,
//              setpoint for the fast loop. Twin of sstep_slow_loop() (Stepper-C).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function st = motion_slow(t, st)
    Q16 = 65536;
    while (st.ci <= size(st.cmds, 1)) && (st.cmds(st.ci, 1) <= t + 1e-12)
        kind = st.cmds(st.ci, 2);
        val = st.cmds(st.ci, 3);
        select kind
        case 1 then
            st.mode = 1;
            st.target = val * Q16;
        case 2 then
            st.mode = 2;
            st.v_run = sim_div_round(val * Q16, st.f);
        case 3 then
            if st.mode <> 0 then
                st.mode = 2;
                st.v_run = 0;
            end
        else
            st.TL = val;
        end
        st.ci = st.ci + 1;
    end

    p0 = st.pos;
    st = motion_step(st);
    if (st.mode == 2) & (st.v_run == 0) & (st.vel == 0) then
        st.mode = 0;              // stop completed
        st.target = st.pos;
    end
    dp = st.pos - p0;

    // run current while moving, hold current after hold_ticks periods without motion
    if dp <> 0 then
        st.still = 0;
        st.hold_active = %f;
    else
        st.still = st.still + 1;
        if (st.still >= st.hold_ticks) & ~st.hold_active then
            st.hold_active = %t;
        end
    end
    if st.hold_active then
        st.I = st.i_hold;
    else
        st.I = st.i_run;
    end

    // setpoint: angle at the period start and increment per fast period
    st.angle0 = p0 * 2^st.shift;
    st.dangle = sim_c_div(dp * 2^st.shift, st.ratio);
    st.k = 0;
    st.v_s = sim_div_round(dp * st.f, Q16);
    st.hist($ + 1, :) = [t, st.pos / Q16, st.vel * st.f / Q16, st.vel, st.mode];
endfunction
