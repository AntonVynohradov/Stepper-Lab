// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/bridge_leg_error.sci
// Description: Averaged power stage with the leg voltage error measured with ngspice
//              (brd.leg_error): table lookup and phase voltage error. Same algorithm
//              as sil/c/sstep_plant.c (sstep_plant_leg_error, period_avg).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function e = bridge_leg_error(tbl, i)
    // tbl  leg voltage error table [i v] (A, V; i rising), stage_esc1_spice().leg_error
    // i    leg current, positive out of the leg [A]
    // e    voltage error of the leg (actual - duty * Vbus) [V]: linear between the
    //      points, constant beyond them
    n = size(tbl, 1);
    if i <= tbl(1, 1) then
        e = tbl(1, 2);
        return;
    end
    for k = 2:n
        if i <= tbl(k, 1) then
            e = tbl(k - 1, 2) + (tbl(k, 2) - tbl(k - 1, 2)) * (i - tbl(k - 1, 1)) / ..
                (tbl(k, 1) - tbl(k - 1, 1));
            return;
        end
    end
    e = tbl(n, 2);
endfunction

function dv = bridge_avg_error(tbl, i, topology)
    // tbl       leg voltage error table (bridge_leg_error)
    // i         phase currents [iA; iB] [A]
    // topology  "full": A = legs 1-2, B = legs 3-4; "shared": A = U - W, B = V - W
    // dv        phase voltage error [dvA; dvB] [V], added to the averaged voltage
    if topology == "shared" then
        ew = bridge_leg_error(tbl, -(i(1) + i(2)));
        dv = [bridge_leg_error(tbl, i(1)) - ew; bridge_leg_error(tbl, i(2)) - ew];
    else
        dv = [bridge_leg_error(tbl, i(1)) - bridge_leg_error(tbl, -i(1));
              bridge_leg_error(tbl, i(2)) - bridge_leg_error(tbl, -i(2))];
    end
endfunction
