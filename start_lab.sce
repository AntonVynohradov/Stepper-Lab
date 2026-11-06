// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        start_lab.sce
// Description: Opens the window of stepper_lab: select the Stepper-C folder and the ngspice
//              stage file, move between MIL and SIL, run scenarios and tests.
//
// How to use: open this file in Scilab and press F5 (or: exec("start_lab.sce", -1)).
// The first run compiles the library (needs a C compiler: Visual Studio on Windows).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

exec(fullfile(get_absolute_file_path("start_lab.sce"), "loader.sce"), -1);
lab_window();
