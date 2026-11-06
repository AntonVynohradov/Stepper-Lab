# Reference

This chapter lists the columns of the results, the Xcos blocks, the main Scilab functions, the C module
of the plant and the folders of the repository.

## Results: `sil.values` { #results-silvalues }

A run writes `sil.time` (s) and `sil.values`, one row per fast period. The Lab diagrams have 23 columns;
`stepper_sil` has the first 19.

| Col. | Signal | Unit | Note |
| --- | --- | --- | --- |
| 1, 2 | iA, iB | A | Phase currents of the plant |
| 3, 4 | iA reference, iB reference | A | From the controller |
| 5 | Speed | rpm | Rotor |
| 6 | Load angle | Electrical deg | Commanded field minus rotor |
| 7 | Commanded position | Microsteps | |
| 8 | Rotor position | Microsteps | |
| 9, 10 | vA, vB | V | Commanded phase voltages: compare values × Vbus, 0 when the outputs are off. Dead time, diodes and the leg error table are not in them |
| 11, 12 | Largest \|iA\|, \|iB\| in the period | A | With the PWM ripple when `brd.stage = "switching"` |
| 13 | State | | 3 = fault |
| 14 | Fault flags | | 1 `SSTEP_FAULT_OVERCURRENT`, 16 `SSTEP_FAULT_EXTERNAL` (MIL with a refused configuration), 32 `SSTEP_FAULT_STALL` |
| 15, 16 | Back-EMF estimate d, q | mV | From `sstep_get_status()`, or `stall_status()` in MIL |
| 17 | Rotor velocity estimate | Microsteps/s | |
| 18 | Load angle estimate | deg | |
| 19 | Detector level | % | 100 % is the trip |
| 20 | Applied motion command | Microsteps/s or microsteps | After the limits |
| 21 | Applied run current | A | After the limits |
| 22 | Load torque | N·m | Input of the plant |
| 23 | Input limit flag | | 1: an input was limited, or `NaN`, in this slow period |

## Xcos blocks

All are in `xcos/blocks/`. Each runs at the fast loop rate.

| Block | Inputs | Outputs | Parameters |
| --- | --- | --- | --- |
| `STEPPER_CTRL_IO` | ADC counts (3), command, run current | Compare values with enable (5), controller signals (15) | `sil_ctrl_rpar(mot, brd, ctl, mp, cmds)`, `lab_mode` |
| `STEPPER_CTRL_MIL` | The same | The same | `ctrl_mil_config(mot, brd, ctl, mp, cmds)`, `lab_mode` |
| `STEPPER_PLANT_IO` | Compare values with enable (5), load torque | ADC counts (3): iA, iB, Vbus; plant signals (9) | `sil_plant_rpar(mot, brd, [0, 0], locked, 4)` |
| `STEPPER_MONITOR_IO` | Plant signals (9), controller signals (15) | Scope groups (4, 1, 2, 1, 3), all columns (23) | `[mot.Ns, mp.M]` |
| `STEPPER_CTRL`, `STEPPER_PLANT`, `STEPPER_MONITOR` | Used by `stepper_sil` | | Command table and load table in their parameters |

## Scilab functions

### Lab

| Function | Purpose |
| --- | --- |
| `lab_window(visible)` | Opens the window. `start_lab.sce` calls it |
| `lab_run(kind, k)` | Runs scenario `k` with `"SIL"` or `"MIL"` without windows. Returns the results, and the diagram as a second output |
| `lab_scenarios()` | The list of scenarios |
| `lab_scenario_diagram(kind, k)` | The diagram of a scenario |
| `lab_diagram(kind, path)` | Builds a Lab diagram, and writes it when `path` is given |
| `lab_set_source(scs_m, label, b)` | Replaces a source block of Inputs |
| `lab_compare_data(rs, rm)` | The comparison of two results, as data |
| `sil_report(sil, visible)` | Draws a result |
| `sil_report_data(sil)` | The panels of the report and the first fault, as data |
| `lab_settings_load()`, `lab_settings_save(s)` | The saved settings |

### Build and configuration

| Function | Purpose |
| --- | --- |
| `sil_build(backend, force)` | Compiles if a source changed, and links |
| `sil_ctrl_dir()` | The Stepper-C folder, or an error that names the window |
| `sil_check_ctrl_dir(d)` | `""`, or the first missing file |
| `sil_check_rpar(rpar)` | `""`, or what `sstep_init()` says about a configuration |
| `stage_load(path)` | Reads and checks a stage file |
| `stage_source()` | The active file, the reason a selected one is not used, and its content |
| `stage_active()` | The active stage |
| `motor_17hs19_2004s1()`, `board_bg431b_esc1()`, `ctrl_default()`, `motion_default()` | Parameters |

### Model

| Function | Purpose |
| --- | --- |
| `sim_closed_loop(mot, brd, c, ref, refp, T, locked, st)` | The prototype in closed loop |
| `ctrl_period_init()`, `ctrl_period()` | One period of the controller, shared by `sim_closed_loop()` and MIL |
| `stepper_simulate(mot, src, srcp, t, x0, locked)` | The motor alone, with a voltage source |
| `motion_init()`, `ref_motion()` | The motion generator |
| `ref_current_step()`, `ref_const_speed()`, `ref_speed_ramp()`, `ref_angle_step()` | Other references |
| `bridge_avg()`, `bridge_switching()`, `bridge_leg_error()` | Power stage models |
| `stall_init()`, `stall_observe()`, `stall_check()`, `stall_status()` | The stall detector |

## The plant in C

`sil/c/sstep_plant.[ch]` holds the power stage, the motor and the ADC of the plant block in portable C,
with no Scilab header. `sil/c/sstep_xcos.c` only wraps it. Other targets can compile the same file.

| Function | Purpose |
| --- | --- |
| `sstep_plant_init()` | Sets up the model: rotor at rest, no current, outputs off. Returns 0, or -1 for an invalid parameter |
| `sstep_plant_set_compare()` | Takes the compare values; they drive the legs in the next period |
| `sstep_plant_period()` | Advances one PWM period, with a load torque |
| `sstep_plant_adc_sample()` | ADC counts of the present state: iA, iB, Vbus |
| `sstep_plant_theta()` | Mechanical rotor angle, unwrapped |
| `sstep_plant_leg_error()` | Leg voltage error of the table at a leg current |

| Build option | Default | Meaning |
| --- | --- | --- |
| `SSTEP_PLANT_FLOAT` | 0 | 1: single precision, for an MCU with an FPU; 0: double, as in Xcos |
| `SSTEP_PLANT_SWITCHING` | 1 | 0: leaves the switching stage out, so no `qsort` is needed |

`stage.off` selects what "outputs off" means: `SSTEP_PLANT_OFF_ZERO_VOLTAGE`, as the prototype and Xcos
do, or `SSTEP_PLANT_OFF_OPEN`, open windings. The electrical angle is kept in [-π, π) with a separate turn
count, so a single-precision build does not lose accuracy over long runs.

## Folders

| Folder | Contents |
| --- | --- |
| `params/` | Motor, board, controller and motion parameters, and the default stage file |
| `model/` | Model functions: equations, controller prototype, stall detector, reports, settings |
| `sil/` | C blocks and plant (`sil/c/`), build of the library; compiled files in `sil/build/` |
| `xcos/` | Diagrams, their generators, the window and the scenarios; blocks in `xcos/blocks/` |
| `tests/` | Test scripts, runner and reference data |
| `tools/` | Python twin and export of the motor parameters; documentation tools in `tools/docs/` |
| `demos/` | Scripts with plots |
| `docs/` | This documentation |
