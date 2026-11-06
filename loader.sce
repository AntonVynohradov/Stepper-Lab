// =============================================================================
// Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
//
// File:        loader.sce
// Description: Loads the parameter and model functions. Run once per session:
//              exec("loader.sce", -1);
//
// Author:      Anton Vynohradov
// Email:       avynohradov@systemfromscratch.com
// License:     Apache License 2.0
// Copyright:   (c) 2026 Anton Vynohradov
//
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

sim_root = get_absolute_file_path("loader.sce");
global sim_root_g;
sim_root_g = sim_root;
global sil_linked_backend;

getd(fullfile(sim_root, "params"));
getd(fullfile(sim_root, "model"));
getd(fullfile(sim_root, "sil"));
getd(fullfile(sim_root, "xcos"));
getd(fullfile(sim_root, "xcos", "blocks"));

// Xcos palettes and simulator (needed by the SIL diagram and its blocks)
loadXcosLibs();
loadScicos();

mprintf("Stepper-Lab loaded from %s\n", sim_root);
