// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stall_observe.sci
// Description: Back-EMF observer of one fast loop period (Stepper-C 1.1.0:
//              sstep_stall_observe(), docs/motor-control.md, Stall detection).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function sd = stall_observe(sd, imeas, v, phi)
    // sd     detector (stall_init)
    // imeas  measured phase currents of this period [iA; iB] [A]
    // v      phase voltages computed in this period, after the power stage limit [V]
    // phi    commanded electrical angle of this period [rad]
    //
    // e = v(k-2) - (L/Ts + R/2) i(k) + (L/Ts - R/2) i(k-1): the voltage written two
    // calls ago is applied from sample k-1 to k (one period computation delay).
    // Saturated at 1 pu (V_base = vbus_nominal), rotated into the commanded frame
    // (d along the current, q 90 deg ahead), first-order low-pass with tau.
    vb = sd.vbase;
    e = min(max(sd.v2 - sd.a * imeas + sd.b * sd.i_prev, -vb), vb);
    sd.i_prev = imeas;
    sd.v2 = sd.v1;
    sd.v1 = v;
    cs = cos(phi);
    sn = sin(phi);
    edq = [e(1) * cs + e(2) * sn; e(2) * cs - e(1) * sn];
    edq = min(max(edq, -vb), vb);
    sd.emf = sd.emf + sd.alpha * (edq - sd.emf);
endfunction
