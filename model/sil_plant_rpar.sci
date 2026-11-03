// =============================================================================
// sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/sil_plant_rpar.sci
// Description: Parameter vector of the plant Xcos block (STEPPER_PLANT): power stage
//              (averaged, optionally with the ngspice leg voltage error, or
//              switching), motor model and ADC. Layout: enum P_* of sil/c/sstep_xcos.c.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function rpar = sil_plant_rpar(mot, brd, loads, locked, nsub)
    // mot, brd  motor and board parameters
    // loads     load torque steps [t TL] (s, N*m); [] = no load
    // locked    1: rotor locked
    // nsub      RK4 sub-steps per fast loop period (4 as in the prototype)
    //
    // brd.leg_error (optional, averaged stage): leg voltage error table [i v]
    // (A, V; rising i), for example stage_esc1_spice().leg_error; [] = ideal legs.
    if brd.topology == "shared" then
        topo = 1;
    else
        topo = 0;
    end
    if isempty(loads) then
        loads = zeros(0, 2);
    end
    if size(loads, 2) <> 2 then
        error("sil_plant_rpar: loads must have 2 columns [t TL]");
    end
    if isfield(brd, "stage") then
        sw = bool2s(brd.stage == "switching");
    else
        sw = 0;
    end
    flat = loads';
    flat = flat(:);                     // [t1 TL1 t2 TL2 ...]
    lerr = [];
    if isfield(brd, "leg_error") then
        lerr = brd.leg_error;
    end
    if isempty(lerr) then
        lerr_i = [];                    // ideal legs (Scilab: zeros(0, 2) is [])
        lerr_v = [];
    elseif size(lerr, 2) <> 2 | size(lerr, 1) > 16 then
        error("sil_plant_rpar: brd.leg_error must be [i v] with at most 16 rows");
    else
        lerr_i = lerr(:, 1);
        lerr_v = lerr(:, 2);
    end
    rpar = [mot.R; mot.L; mot.Kt; mot.Ns; mot.J; mot.B; mot.Td; mot.Tc; mot.w_c; ..
            topo; brd.vbus; brd.period; brd.i_full; brd.adc_span; brd.adc_zero; ..
            brd.adc_vbus_full; brd.adc_full; brd.fs; nsub; bool2s(locked <> 0); ..
            sw; brd.dead_ns; brd.rds_on; brd.vd; brd.sw_nstep; ..
            size(lerr_i, "*"); size(loads, 1); flat; lerr_i; lerr_v];
endfunction
