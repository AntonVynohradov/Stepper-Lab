// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/stall_status.sci
// Description: Observer fields of the status (Stepper-C 1.1.0: sstep_stall_status(),
//              struct sstep_status emf_d_mv .. stall_level_pct).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function s = stall_status(sd)
    // sd  detector (stall_init, stall_observe, stall_check)
    // s   [emf_d_mv emf_q_mv rotor_velocity load_angle_cdeg stall_level_pct]:
    //     filtered back-EMF [mV]; rotor speed sgn(eq) |e| Ns M / (2 pi Ke)
    //     [microsteps/s]; load angle atan2(sgn ed, sgn eq) [0.01 deg], sgn = sign of
    //     the command (at rest of eq); detector level 100 count / N [%]
    ed = round(sd.emf(1) * 1000);
    eq = round(sd.emf(2) * 1000);
    sg = 1;
    if (sd.vel < 0) | ((sd.vel == 0) & (eq < 0)) then
        sg = -1;
    end
    mag = sim_isqrt(ed^2 + eq^2);
    speed = floor(mag * 1000 * 113 * sd.nsm / (710 * sd.ke_uv));
    if eq < 0 then
        speed = -speed;
    end
    ang = fix(atan(sg * ed, sg * eq) * 18000 / %pi);
    s = [ed, eq, speed, ang, floor(sd.count * 100 / sd.N)];
endfunction
