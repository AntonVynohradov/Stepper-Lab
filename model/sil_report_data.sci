// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/sil_report_data.sci
// Description: Content of the report of a run: panels with labels and series, first fault.
//              No graphics: sil_report draws it.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function d = sil_report_data(sil)
    // sil  results of a run: sil.time, sil.values (19 columns of stepper_sil, 23 of
    //      stepper_lab)
    // d    d.panels: list of struct title, ylabel, legend, t, y
    //      d.t_fault: time of the first fault [s], -1 none; d.fault_names: its flags
    //      ylim of a panel: its range up to shortly after the first fault, because a load
    //      torque keeps accelerating the free rotor after it and would set the scale
    t = sil.time;
    v = sil.values;

    function p = panel(title, ylabel, legend, cols)
        p = struct("title", title, "ylabel", ylabel, "legend", legend, "t", t, "y", v(:, cols));
    endfunction

    d = struct();
    d.panels = list();
    d.panels($ + 1) = panel("Phase currents", "Current [A]", ["iA", "iB", "iA reference", "iB reference"], 1:4);
    d.panels($ + 1) = panel("Speed", "Speed [rpm]", "rotor", 5);
    d.panels($ + 1) = panel("Position", "Position [microsteps]", ["commanded", "rotor"], 7:8);
    d.panels($ + 1) = panel("Load angle", "Electrical angle [deg]", "field - rotor", 6);
    d.panels($ + 1) = panel("Phase voltages", "Voltage [V]", ["vA", "vB"], 9:10);
    if size(v, 2) >= 23 then
        d.panels($ + 1) = panel("Motion command", "Command [microsteps/s or microsteps]", "applied", 20);
        d.panels($ + 1) = panel("Run current, load, input limit", "[A], [N*m], flag", ["run current", "load torque", "input limited"], [21, 22, 23]);
    end
    d.panels($ + 1) = panel("State, faults, stall detector", "state, flags, level [%]", ["state (3 = fault)", "fault flags", "stall level"], [13, 14, 19]);

    d.t_fault = -1;
    d.fault_names = "";
    k = find(v(:, 14) <> 0, 1);
    last = size(v, 1);
    if ~isempty(k) then
        d.t_fault = t(k);
        d.fault_names = sim_stall_flags(v(k, 14));
        last = min(last, k + ceil(0.02 * last));
    end
    for j = 1:size(d.panels)
        p = d.panels(j);
        lo = min(p.y(1:last, :));
        hi = max(p.y(1:last, :));
        pad = 0.08 * (hi - lo);
        if pad == 0 then
            pad = max(1, abs(hi) * 0.1);
        end
        p.ylim = [lo - pad, hi + pad];
        d.panels(j) = p;
    end
endfunction
