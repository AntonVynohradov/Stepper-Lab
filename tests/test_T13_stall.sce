// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        tests/test_T13_stall.sce
// Description: T13: stall detection of Stepper-C 1.1.0 in the Scilab prototype
//              (stall_init / stall_observe / stall_check in sim_closed_loop, with the
//              overcurrent trip of the library). Scenarios of sim_stall_cases: must
//              trip (SSTEP_FAULT_STALL in the first fault, within stall_time_ms + 20 ms
//              of the pull-out) and must not trip (no fault, no step lost). Averaged stage with the
//              ngspice leg voltage error for every scenario, switching stage for the
//              scenarios marked proto_sw (the switching prototype is slow; S06 runs
//              all of them on both stages in C). The results are kept in
//              stall_proto for S06.
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

c = ctrl_default();
c.stall_time_ms = 50;
c.protect = 1;
cs = sim_stall_cases();
stall_proto = list();
for k = 1:size(cs)
    s = cs(k);
    res = struct("avg", [], "switching", []);
    stages = "avg";
    if s.proto_sw then
        stages = ["avg", "switching"];
    end
    for stg = stages
        [r, y] = sim_stall_proto(s, stg, c);
        res(stg) = struct("r", r, "y", y);
        [ok, detail] = sim_stall_verdict(s, r, c);
        sim_failures = sim_failures + sim_check(msprintf("T13 %s [%s]", s.name, stg), ok, detail);
    end
    stall_proto(k) = res;
end
