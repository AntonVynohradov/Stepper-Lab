# Configuration

This chapter lists what you can set: the variables of the diagram context, the parameter structures,
the search for the library and the stage file, and the environment variables.

## Context of a diagram

Edit it in Xcos under **Simulation → Set Context**. The Lab diagrams define:

| Variable | Default | Meaning |
| --- | --- | --- |
| `mot` | `motor_17hs19_2004s1()` | Motor |
| `brd` | `board_bg431b_esc1()` | Power stage and ADC |
| `ctl` | `ctrl_default()` | Current loop and stall detection |
| `mp` | `motion_default()` | Microsteps, speed, acceleration, currents |
| `lab_mode` | 1 | 1 velocity, 2 position, 0 the table `cmds` |
| `cmds` | Empty | Command table `[t kind value]`, for `lab_mode = 0` |
| `lab_cmd` | 3200 | Value of the command source: 1 rev/s at 16 microsteps |
| `lab_irun` | `mp.i_run` | Value of the run current source, A |
| `lab_load` | 0 | Value of the load torque source, N·m |
| `locked` | 0 | 1: rotor locked, for current loop tests |
| `T_end` | 0.6 | Simulated time, s |
| `scope_i`, `scope_rpm`, `scope_pos` | 3, 400, 7000 | Scope ranges |

## Motor: `mot`

`motor_17hs19_2004s1()` holds the example motor. Copy the file for another motor.

| Field | Value | Unit | Source |
| --- | --- | --- | --- |
| `R` | 1.60 | Ω | Datasheet, ±10 %; older listings give 1.4 Ω. Measure it |
| `L` | 3.0e-3 | H | Datasheet at 1 kHz, ±20 % |
| `I_rated` | 2.0 | A | Datasheet |
| `Kt` | 0.2086 | N·m/A | Derived: 0.59 N·m holding torque, both phases at 2 A. If the datasheet value is for one phase, it is 0.295. Measure the back-EMF |
| `Ns` | 200 | Steps/rev | 1.8° motor |
| `Td` | 0 | N·m | Detent torque, not in the datasheet. T11 uses 0.02 as an assumption |
| `J` | 82e-7 | kg·m² | Datasheet, rotor only |
| `B` | 1e-5 | N·m·s/rad | Viscous friction, assumption |
| `Tc` | 0 | N·m | Dry friction, unknown. T12 uses 0.01 |
| `w_c` | 0.05 | rad/s | Dry friction is smoothed as `Tc tanh(w / w_c)`, so there is no stiction |

To measure `Kt`, spin the shaft with a drill and measure the open-circuit phase voltage amplitude `E`
and the speed `w`. Then `Kt = E / w`, in V·s/rad, which equals N·m/A.

## Board: `brd`

`board_bg431b_esc1()` holds the example board.

| Field | Value | Unit | Meaning |
| --- | --- | --- | --- |
| `vbus` | 24 | V | Bus voltage |
| `topology` | `"shared"` | | `"shared"`: three legs; `"full"`: two H-bridges |
| `fs` | 20000 | Hz | PWM frequency, equal to the fast loop rate |
| `vbus_min`, `vbus_max` | 10, 30 | V | Undervoltage and overvoltage trips |
| `i_full` | 4.0 | A | Phase current at ADC full scale |
| `i_trip` | 3.5 | A | Overcurrent trip |
| `period` | 4250 | Counts | PWM compare value for 100 % |
| `adc_zero`, `adc_span` | 2048, 2047 | Counts | ADC reading at 0 A; counts from 0 A to `i_full` |
| `adc_vbus_full` | 34.32 | V | Bus voltage at the largest ADC reading |
| `adc_full` | 4095 | Counts | Largest ADC reading |
| `stage` | `"avg"` | | `"avg"` averaged, or `"switching"` with dead time and diodes |
| `sw_nstep` | 16 | | RK4 steps per PWM period in the switching stage, plus one at each edge |
| `dead_ns`, `dead_timer_ns` | From the stage file | ns | Effective dead time; dead time set in the timer |
| `rds_on`, `vd` | From the stage file | Ω, V | Leg resistance; body diode voltage |
| `leg_error` | `[]` | | Table of leg voltage error against current; `[]` means ideal legs. Set `brd.leg_error = stage_active().leg_error` to use it |

The ADC scaling is a model of the measurement chain. Replace it with the measured values of your
board.

## Controller: `ctl`

`ctrl_default()` gives the table below. Not every field reaches every controller: `delay` and `nsub`
apply only to `sim_closed_loop()`, the prototype outside the diagrams. `ff` applies to the prototype and
MIL; the library in SIL computes its own feed-forward from the back-EMF constant of the motor. `protect` is
always on in MIL and has no effect in SIL, where the library has its overcurrent trip in current drive
(`ctl.drive = 0`) and none in voltage drive.

| Field | Value | Unit | Meaning |
| --- | --- | --- | --- |
| `bw_hz` | 1000 | Hz | Current loop bandwidth: `Kp = L wc`, `Ki = R wc` |
| `delay` | 1 | Periods | Computation delay, 0 or 1; on the MCU it is 1 |
| `ff` | 1 | | Back-EMF feed-forward, 0 or 1 |
| `nsub` | 4 | | RK4 sub-steps of the plant per fast period, in the prototype |
| `protect` | 0 | | 1: the overcurrent trip of the library in `sim_closed_loop()` |
| `stall_time_ms` | 0 | ms | Condition time before the stall fault; 0 is off, as in the library. The tests use 50 |
| `stall_filter_us` | 1000 | µs | Time constant of the back-EMF filter |
| `stall_speed` | 1600 | Microsteps/s | 0.5 rev/s: slip threshold, and the speed from which the synchronism check runs |
| `stall_sync_pct` | 30 | % | Smallest back-EMF along the motion, as a percentage of `Ke w`. The library example is 50 |

Optional fields: `ctl.mot` tunes the controller for a motor other than the plant (test T08), and
`ctl.drive = 1` selects voltage drive, in SIL only.

## Motion: `mp`

`motion_default()` gives:

| Field | Value | Unit | Meaning |
| --- | --- | --- | --- |
| `M` | 16 | | Microsteps per full step: 3200 per revolution |
| `f_slow` | 1000 | Hz | Slow loop rate |
| `v_max` | 16000 | Microsteps/s | 5 rev/s |
| `acc`, `dec` | 160000 | Microsteps/s² | 50 rev/s² |
| `i_run` | 2.0 | A | Current while moving |
| `i_hold` | 1.0 | A | Current at standstill |
| `hold_delay` | 0.1 | s | Standstill time before the hold current |

## The Stepper-C folder

`sil_ctrl_dir()` takes the first of these:

1. The environment variable `SSTEP_CTRL_DIR`.
2. The folder saved by the window. If it is no longer valid, a warning is printed and the search goes on.
3. `ext/Stepper-C`, then `../Stepper-C`.
4. `ext/sfs-stepper-ctrl`, then `../sfs-stepper-ctrl`.

A valid folder holds `sstep.h`, `inc/sstep_ctrl.h` and `src/sstep_ctrl.c`. Without one, the error
message points to the window. `sil_build()` prints the folder it uses.

## The stage file

The stage file carries the results of the ngspice model of a power stage. This project does not start
ngspice. To create a file with
[sfs-stepper-power-spice](https://github.com/AntonVynohradov/sfs-stepper-power-spice):

1. Create an empty folder with a subfolder `params`, for example `my_stage/params`.
2. In `sfs-stepper-power-spice`, run `python tools/run_tests.py --export --sim <path to my_stage>`.
3. In the window, select `my_stage/params/stage_esc1_spice.json`.

Select that file, and not `export/stage_esc1.json`, which has no `format`, `vbus` or `fs` and which the
window refuses. Do not point `--sim` at this repository unless you mean to replace its default stage,
`params/stage_esc1_spice.*`.

The active file is the first of:

1. The environment variable `SSTEP_STAGE_FILE`.
2. The file saved by the window.
3. The default, `params/stage_esc1_spice`.

Simulations and tests use the active file, and `tests/run_all.sce` prints it at its start and end. A
selected file that is not valid counts as a test failure, so a result never comes from another source
without saying so.

| Field | Check | Meaning |
| --- | --- | --- |
| `format` | Must be 1 | Version of the file format |
| `vbus`, `fs` | One finite number, positive | Operating point of the table, V and Hz |
| `r_leg` | One finite number, not negative | Leg resistance: switch and shunt, Ω |
| `vd` | The same | Body diode voltage, V |
| `dead_timer_ns`, `dead_eff_ns` | The same | Dead time in the timer; effective dead time |
| `leg_error_table` | 1 to 16 rows `[i v]`, finite, current rising | Leg voltage error against leg current |

The table is valid at its operating point. The window warns when `vbus` or `fs` differs from the board.

The default file gives:

| Value | Result | Note |
| --- | --- | --- |
| Effective dead time | 133 ns | Timer 500 ns (assumption), minus the turn-off delay of the MOSFET; fitted for 1 to 3 A |
| Leg resistance | 3.4 mΩ | Switch 1.9 mΩ, with the low-side 3 mΩ shunt (assumption), averaged |
| Body diode | 0.73 V | At 2 A |

Below 1 A the real dead time error is smaller than the model, because the phase current charges the
switch node too slowly: 50 % of the model at 0.5 A, and 25 % at 0.25 A.

```scilab
stg = stage_active();
mprintf("%g V, %g kHz, %d table points\n", stg.vbus, stg.fs / 1000, size(stg.leg_error, 1));
```

## Motor parameters for C

`params/motor_*.json` holds the motor parameters for C users of the plant. The `.sci` file is the
source: `python tools/export_params.py` writes the JSON, and `--check` fails when it is older than
the `.sci` file. Both JSON kinds carry `"format": 1`, and readers check it.

## Settings file

The window keeps its choices in `SCIHOME/sfs_stepper_lab.json`: `ctrl_dir`, `stage_file`, `backend`,
`model` and `scenario`. A missing or damaged file, or a value of the wrong type, gives the default.
`SSTEP_LAB_SETTINGS` names another file.

## Environment variables

| Variable | Meaning |
| --- | --- |
| `SSTEP_CTRL_DIR` | Stepper-C folder; wins over the window |
| `SSTEP_STAGE_FILE` | Stage file; wins over the window |
| `SSTEP_LAB_SETTINGS` | Settings file |
| `SSTEP_TEST_FILTER` | Regular expression over the test file names |
| `SSTEP_PYTHON` | Python with numpy and scipy for test X04, when another stage file is active |
| `SSTEP_SIL_BACKEND`, `SSTEP_STALL_PROTO` | Used between `run_all.sce` and its second Scilab process |
