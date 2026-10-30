// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        params/ctrl_default.sci
// Description: Default current controller settings (Stepper-C: struct sstep_ctrl_cfg).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function c = ctrl_default()
    c = struct();
    c.bw_hz = 1000;  // Hz, current loop bandwidth: Kp = L wc, Ki = R wc
    c.delay = 1;     // computation delay in fast loop periods (0 or 1); MCU: 1
    c.ff    = 1;     // back-EMF feed-forward from the reference speed (0 or 1)
    c.nsub  = 4;     // RK4 sub-steps of the plant per fast loop period
endfunction
