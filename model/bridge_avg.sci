// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/bridge_avg.sci
// Description: Averaged power stage: requested phase voltages -> applied phase
//              voltages and leg duty cycles, for both topologies.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function [v, duty, k] = bridge_avg(vref, vbus, topology)
    // vref      requested phase voltages [vA; vB] [V]
    // vbus      bus voltage [V]
    // topology  "full"   two H-bridges, legs [A+ A- B+ B-], |vA|, |vB| <= vbus
    //           "shared" three legs [U V W], A = U-W, B = V-W,
    //                    max(a, b, 0) - min(a, b, 0) <= 1 (a = vA/vbus, b = vB/vbus)
    //
    // v         applied phase voltages [vA; vB] [V]
    // duty      leg duty cycles 0..1 (column, 4 or 3 entries)
    // k         scale applied to the request (1 = not limited)
    //
    // When the request cannot be produced, the voltage vector is scaled down with its
    // direction kept (the current angle, and so the torque direction, is preserved).
    // Shared leg: the common leg is centred, w = 0.5 - (max + min) / 2, which gives the
    // largest circular voltage amplitude vbus / sqrt(2).

    a = vref(1) / vbus;
    b = vref(2) / vbus;

    if topology == "full" then
        m = max(abs(a), abs(b));
        if m > 1 then
            k = 1 / m;
        else
            k = 1;
        end
        a = a * k;
        b = b * k;
        duty = [0.5 + a / 2; 0.5 - a / 2; 0.5 + b / 2; 0.5 - b / 2];
    elseif topology == "shared" then
        hi = max(a, b, 0);
        lo = min(a, b, 0);
        span = hi - lo;
        if span > 1 then
            k = 1 / span;
        else
            k = 1;
        end
        a = a * k;
        b = b * k;
        w = 0.5 - (hi + lo) * k / 2;
        duty = [a + w; b + w; w];
    else
        error(msprintf("bridge_avg: unknown topology ""%s""", topology));
    end

    v = [a; b] * vbus;
endfunction
