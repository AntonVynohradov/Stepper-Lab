// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/src_dc.sci
// Description: Source: constant phase voltages and load torque.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function u = src_dc(t, x, p)
    // p.vA, p.vB  phase voltages [V]
    // p.TL        load torque [N*m]
    u = [p.vA; p.vB; p.TL];
endfunction
