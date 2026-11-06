// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        start_sil.sce
// Description: Opens the software-in-the-loop model in Xcos: the Stepper-C library
//              (C code of Stepper-C) drives the stepper motor model.
//
// How to use (no Scilab knowledge needed):
//   1. Open this file in Scilab (double-click, or File > Open) and press F5
//      (or: exec("start_sil.sce", -1) in the console).
//   2. Xcos opens. Press Start (the triangle). The scope shows the currents, the
//      speed, the load angle and the positions.
//   3. Change the scenario in Simulation > Set Context (commands, load, topology),
//      then press Start again. The results are in the variable sil.
//
// The first start compiles the library (needs a C compiler: Visual Studio on Windows).
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

sil_backend = "FIXED";     // "FLOAT" for microcontrollers with an FPU (STM32G4: FLOAT)

sil_root = get_absolute_file_path("start_sil.sce");
exec(fullfile(sil_root, "loader.sce"), -1);
sil_build(sil_backend);

sil_zcos = fullfile(sil_root, "xcos", "stepper_sil.zcos");
sil_regenerate = %f;       // %t: rebuild the diagram from xcos/sil_diagram.sci
if sil_regenerate | ~isfile(sil_zcos) then
    sil_diagram(sil_zcos);
end
xcos(sil_zcos);
