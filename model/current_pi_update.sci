// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/current_pi_update.sci
// Description: Integrator update with back-calculation anti-windup.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function integ = current_pi_update(integ, e, vu, vsat, g)
    // integ += Ki Ts e + Ts/Tt (vsat - vu): while the power stage limits the voltage,
    // the integrator is pulled back so that it does not wind up.
    integ = integ + g.Ki * g.Ts * e + g.Kb * (vsat - vu);
endfunction
