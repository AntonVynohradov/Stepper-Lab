# Changelog

All notable changes to this project are documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Stall detection of Stepper-C 1.1.0 in the prototype: back-EMF observer
  (`stall_observe`, fast loop), decision with the up/down counter and the learned
  offset at rest (`stall_check`, slow loop, integer millivolts), constants
  (`stall_init`) and status estimates (`stall_status`), with the equations of the
  library documentation. `sim_closed_loop` runs them with the motion generator when
  `c.stall_time_ms > 0` (fourth output `sd`: fault time, flags, per-period back-EMF
  estimates and counter); after a fault the outputs are off and the reference frozen,
  as in the library. Optional overcurrent trip of the library (`c.protect = 1`).
- `params/ctrl_default.sci`: `stall_time_ms` (0 = off), `stall_filter_us` 1000,
  `stall_speed` 1600, `stall_sync_pct` 30; `protect` 0.
- Averaged stage with the leg voltage error in the prototype (`model/bridge_leg_error.sci`,
  `brd.leg_error` in `sim_closed_loop`), the algorithm of `sstep_plant.c`, so that the
  prototype and the Xcos plant block can be compared on that stage.
- Tests T13 (prototype) and S06 (Xcos, FIXED and FLOAT backends; the other backend
  runs S01–S06 in a second Scilab process, `tests/run_sil_backend.sce`): must trip
  (stall at the hold current, load step above the pull-out torque, overload at start,
  blocked rotor), must not trip (enable and run → hold current steps, T05, T07, T09,
  T08 parameter uncertainty, T11 detent sweep and mid-band), on the averaged stage
  with the leg voltage error and on the switching stage; the Xcos run equals the
  prototype including the time of the fault. Scenarios in `tests/sim_stall_cases.sci`.
  Results in the README (Stall detection). Not solved by the detector as designed,
  reported as failing tests: the T06 load step (0.5 N·m at 1 rev/s) and the overload at
  start end in `SSTEP_FAULT_OVERCURRENT` 3.7 / 39 ms after the pull-out with the stall
  level at 4 / 30–38 % (a fast backward slip keeps `eq` above the threshold half of the
  time); `stall_sync_pct` 50 and 40 trip at 11 rev/s without lost steps.
- `sil/c/sstep_plant.[ch]`: the plant model (averaged and switching power stage,
  motor, ADC) in portable C, double or single precision (`SSTEP_PLANT_FLOAT`), the
  switching stage optional (`SSTEP_PLANT_SWITCHING`); outputs off as zero voltage or
  open windings. Used by the Xcos plant block and by the firmware simulation of
  sfs-stepper-driver.
- Averaged stage with the ngspice leg voltage error table (`brd.leg_error`,
  `stage_esc1_spice().leg_error`); test S05 against the switching stage.
- `tools/export_params.py`: `params/motor_*.json` from `params/motor_*.sci`
  (`--check` for CI); `params/motor_17hs19_2004s1.json`.

- Averaged two-phase hybrid stepper motor model (`model/`): state equations
  `[iA; iB; w; theta]`, back-EMF, torque with optional detent term, viscous friction,
  locked-rotor option; `stepper_simulate()` with lsoda.
- Parameters of the StepperOnline 17HS19-2004S1 (`params/motor_17hs19_2004s1.sci`).
- Sources `src_dc` and `src_open_loop` (phase voltages and load torque).
- Tests T00 (conventions), T01 (steady state V/R), T02 (time constant L/R) and X01
  (cross-check with the Python twin), runner `tests/run_all.sce`.
- Python twin `tools/ref_model.py` and reference data `tests/reference/`.
- Demo `demos/demo_open_loop.sce`.
- Discrete current loop (prototype of the Stepper-C fast loop): `current_pi_gains`,
  `current_pi`, `current_pi_update` (back-calculation anti-windup), one-period
  computation delay, back-EMF feed-forward, `sim_closed_loop` with an RK4 plant.
- Averaged power stage `bridge_avg` for two H-bridges and for three legs with a common
  leg (B-G431B-ESC1), voltage vector limited with its direction kept.
- Parameters `board_bg431b_esc1`, `ctrl_default`; references `ref_current_step`,
  `ref_const_speed`.
- Tests B01 (power stage), T03 (microstepping), T04 (current step), X02 (cross-check of
  the closed loop with `tools/ref_ctrl.py`).

- Motion generator (prototype of the Stepper-C slow loop): `motion_init`,
  `motion_slow` (trapezoidal profile with one-period look-ahead braking, MOVE / RUN /
  STOP commands, hold current), `ref_motion` (angle interpolated every fast period);
  parameters `motion_default`.
- Tests T05 (move with speed ramp), T06 (load step, loss of synchronism), T07 (reversal),
  T09 (voltage limit of both topologies), X03 (cross-check with `tools/ref_motion.py`).

### Changed

- `stepper_motor_rhs` uses the new `stepper_motor_f`, the single place of the state
  equations.
- Reference functions keep a state: `[r, st] = ref(t, p, st)`; `sim_closed_loop`
  returns the final state and records the reference angle `phi` (column 12).
- Motion generator replaced by the bit-exact twin of the Stepper-C integer algorithm
  (`motion_step`, `motion_slow`, `ref_motion`, helpers `sim_int.sci`): exact discrete
  braking, integer angle interpolation, hold current after periods without motion.
  `tools/ref_motion.py` matches `src/sstep_motion.c` over 3 052 885 periods.
- T05 and T07 check the limits in the integer units of the library.
- Software in the loop in Xcos: `start_sil.sce` compiles Stepper-C (`sil_build`, FIXED
  or FLOAT) and opens `xcos/stepper_sil.zcos` (generated by `sil_diagram`). Blocks
  `STEPPER_CTRL` (the C library), `STEPPER_PLANT` (averaged power stage, motor, ADC;
  one period computation delay), `STEPPER_MONITOR`; computational functions in
  `sil/c/sstep_xcos.c`; scenario in the diagram context; results in the variable `sil`.
- Board parameters for the SIL model (trip levels, PWM period, ADC scaling);
  `sil_ctrl_rpar`, `sil_plant_rpar`, `sil_check_rpar` (configuration checked by the
  library itself).
- Tests S01–S03: the Xcos diagram against the prototype.
- Switching power stage (`model/bridge_switching.sci`, `stepper_period_sw`): centre-aligned
  PWM legs, dead time inserted at every turn-on, body diode conduction during the dead
  time, Rds(on); the motor is integrated between the switching instants. Selected with
  `brd.stage = "switching"` in `sim_closed_loop` (third output: current extremes and
  mean phase voltages per period) and in the Xcos plant block.
- Board parameters `stage`, `rds_on`, `vd`, `sw_nstep`.
- Python twin `tools/ref_switching.py`; reference `tests/reference/x04_*.csv`.
- Tests T10 (switching stage: ideal switches equal the averaged model, analytic ripple,
  dead time voltage error, 4 rev/s with dead time), X04 (Scilab vs Python), S04 (Xcos
  plant with the switching stage vs prototype). Demo `demos/demo_switching.sce`.
- `ref_speed_ramp` (constant current, linear speed ramp).
- `params/stage_esc1_spice.sci` / `.json`: switching stage values generated by the
  ngspice model (`sfs-stepper-power-spice`); `board_bg431b_esc1` and
  `tools/ref_switching.py` use them (effective dead time 133 ns, leg resistance
  3.4 mΩ, body diode 0.73 V). New field `brd.dead_timer_ns`.

- Dry friction `Tc tanh(w / w_c)` in the mechanical equation (`stepper_motor_f`, Python
  twin, Xcos plant block); motor fields `Tc`, `w_c` (default 0, 0.05 rad/s).
- `sim_closed_loop`: optional `c.mot`, the motor the controller is tuned for (gains,
  feed-forward), different from the plant.
- `ref_angle_step`; `ref_speed_ramp` takes an optional load torque `p.TL`.
- Tests T08 (parameter uncertainty), T11 (natural frequency, detent resonance,
  mid-band instability), T12 (dry friction); Python twin `tools/ref_ext.py`.

### Changed

- Xcos plant block: 13 discrete states, output 2 adds the largest |iA|, |iB| of the
  last period; monitor output 5 (variable `sil`) has 12 columns (+ peaks).
- `sil_plant_rpar` adds the dry friction (P_TC, P_WC) and the power stage model
  parameters (P_STAGE..P_SW_NSTEP).
- Xcos plant block: thin wrapper of `sstep_plant.c`; the electrical angle is kept
  within one turn plus a turn count. `sil_plant_rpar` adds `P_NERR` and the leg error
  table after the load steps.
- `params/stage_esc1_spice.*` (generated by sfs-stepper-power-spice) carry the leg
  error table, its operating point and `"format": 1`.
- Controller block: the stall configuration in the parameter vector
  (`C_STALL_MS` .. `C_STALL_SYNC`, `C_NPAR` = 34), `sil_ctrl_rpar` takes it from
  `ctl` and the controller motor from `ctl.mot` (as `c.mot` in the prototype).
  Controller monitor output 12 values (+ back-EMF d/q, rotor velocity, load angle,
  detector level); monitor output 5 (variable `sil`) has 19 columns (+ state, fault
  flags, the five estimates).
- `test_S01_sil.sce` builds the backend of `sil_test_backend` (default FIXED).

### Fixed

- SIL with Stepper-C 1.1.0: `make_config()` left the new `stall_*` fields
  uninitialised, `sstep_init()` returned `SSTEP_ERR_PARAM` and S01–S05 failed; the
  configuration is now zeroed first.
- SIL build against Stepper-C 1.0.0: the monitor reads the setpoint through
  `sstep_internal.h` (the instance is opaque), MSVC compiles with `/std:c11`
  (`_Static_assert`), gcc with `-std=gnu11`.
- `sil_ctrl_rpar` / `sil_plant_rpar` with no command or load: Scilab's `zeros(0, n)`
  is `[]`, the column check failed.
