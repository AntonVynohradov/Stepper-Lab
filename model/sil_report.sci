// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        model/sil_report.sci
// Description: Report of a run in one figure: a panel per group of signals with title,
//              unit, legend and grid, and a marker at the first fault.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

function h = sil_report(sil, visible)
    // sil      results of a run (sil_report_data)
    // visible  %f: hidden figure (optional, default %t)
    // h        figure handle
    [lhs, rhs] = argn(0);
    if rhs < 2 then
        visible = %t;
    end
    d = sil_report_data(sil);
    h = figure("visible", "off", "figure_name", "Stepper-Lab report", "background", -2, ..
               "figure_size", [1200, 950]);
    drawlater();
    n = size(d.panels);
    rows = ceil(n / 2);
    for k = 1:n
        p = d.panels(k);
        subplot(rows, 2, k);
        plot(p.t, p.y);
        a = gca();
        a.grid = [color("gray"), color("gray")];
        title(p.title);
        ylabel(p.ylabel);
        xlabel("Time [s]");
        a.data_bounds = [min(p.t), p.ylim(1); max(p.t), p.ylim(2)];
        a.tight_limits = "on";
        legend(p.legend, "in_upper_left");
        if d.t_fault >= 0 then
            yr = p.ylim(:);
            plot([d.t_fault, d.t_fault], yr, "r--");
            e = gce();
            e.children.thickness = 2;
            xstring(d.t_fault, yr(1) + 0.05 * (yr(2) - yr(1)), d.fault_names);
        end
    end
    drawnow();
    if visible then
        h.visible = "on";
    end
endfunction
