// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/ref_motion.sci
// Description: Reference from the motion generator: slow loop every fs/f_slow periods,
//              angle advanced by a constant increment every fast period (as Stepper-C).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [r, st] = ref_motion(t, p, st)
    // t   time [s], p unused, st state from motion_init
    // r   [I; phi; w_ref; TL], phi unwrapped [rad electrical]
    if modulo(st.n, st.ratio) == 0 then
        st = motion_slow(t, st);
    end
    phi = (st.angle0 + st.k * st.dangle) * (2 * %pi / 2^32);
    w_ref = st.v_s * 2 * %pi / (st.Ns * st.M);
    if st.k < st.ratio then
        st.k = st.k + 1;
    end
    st.n = st.n + 1;
    r = [st.I; phi; w_ref; st.TL];
endfunction
