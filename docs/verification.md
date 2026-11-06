# Verification

This chapter describes how the model is checked and how to run the checks. The final full run gave
**433 PASS, 0 FAIL, 0 SKIP** with both backends, on Windows 10 with Scilab 2027.0.0, the default stage file
and Stepper-C at commit `3ef0f7e`.

## How the model is checked

| Check | Against | Tests |
| --- | --- | --- |
| Motor and control behavior | Theory: time constants, power balance, ripple, spin-down | T00 to T13, B01 |
| Scilab model | Python twin, written separately, with reference data in `tests/reference/` | X01 to X04 |
| C library in Xcos | Scilab prototype, sample by sample | S01 to S06 |
| Lab diagrams and report | The specified limits, labels and scenarios | S07 to S11 |
| MIL | SIL | M01 |
| Window and settings | The specified order and messages | C01 to C03 |

## Running the tests

```bash
WScilex-cli -quit -f tests/run_all.sce      # Windows
scilab-cli -quit -f tests/run_all.sce       # Linux
```

The exit status is 1 when a test fails. The log starts and ends with the library folder and the stage
file in use. A full run took 386 minutes.

| Task | How |
| --- | --- |
| Run a group from the window | Group All, SIL, MIL or Model, then **Run tests** and **Refresh** |
| Run selected tests | Set `SSTEP_TEST_FILTER`, a regular expression over the file names; see below |
| Run the second backend | `run_all.sce` starts it by itself, through `tests/run_sil_backend.sce` |

For example, to run C01 and T13:

```bash
SSTEP_TEST_FILTER="C01|T13" scilab-cli -quit -f tests/run_all.sce
```

In PowerShell, first set `$env:SSTEP_TEST_FILTER = "C01|T13"`. A filter that matches no file runs nothing and
still reports `0 failure(s)`, so check that the log lists the tests you expected.

## Testing another stage file

Select the file in the window, or set `SSTEP_STAGE_FILE`, and run the tests. Simulations and tests use it,
and the log names it.

- A file that is not valid counts as a failure, and the tests do not fall back to the default silently.
- Test X04 computes its reference with the Python twin for the active file, which needs `SSTEP_PYTHON` or a
  Python in `PATH` with numpy and scipy. Otherwise it reports SKIP.
- Tests that compare against numbers from the default stage can fail with a stage of other values. For
  example, S05 fails when dead time and diode voltage change while the leg error table stays the same,
  because the averaged and the switching model then describe different stages. Decide whether a failure
  comes from the new stage values before treating it as a defect.

Selected tests (`C0|X04|T10|S01`) ran on another stage file. A full run was made only on the default one.

## Python twin

`tools/` holds a separate implementation of parts of the model, in Python with numpy and scipy. It has no
stall detector, so the X tests run with detection off.

```bash
python tools/ref_model.py                # motor tests
python tools/ref_ctrl.py                 # current loop tests
python tools/ref_motion.py               # motion tests
python tools/ref_switching.py            # switching stage tests
python tools/ref_ext.py                  # T08, T11, T12
```

Add `--write` to the first four to regenerate `tests/reference/*.csv`. With another stage,
`ref_switching.py --stage <file> --out <file>` writes the reference elsewhere.

## Test list

| Test | Stimulus | Check |
| --- | --- | --- |
| T00 | Random states, current vector | Power balance, stable rotor position |
| T01 | Constant voltage on phase A | Steady-state current V/R, locked and free rotor |
| T02 | Voltage step, locked rotor | Current rises with τ = L/R |
| X01 | Rotating voltage, 5 Hz electrical | Scilab equals the Python twin |
| B01 | Random voltage requests, both topologies | Duties 0 to 1, direction kept, exact when feasible, largest amplitude |
| T03 | Microstepping 2 A, 10 Hz electrical | A leads B by 90°, amplitude, rotor follows the field |
| T04 | Current step 0 → 2 A, locked rotor, both topologies | Overshoot below 5 %, settling (2 %) below 1 ms, error below 0.1 % |
| X02 | Current step, shared leg | Discrete loop equals the Python twin |
| T05 | Move 2 rev, 5 rev/s, 50 rev/s² | Limits kept, target exact, load angle below 45° |
| T06 | Load step at 1 rev/s: 0.25 and 0.5 N·m | Synchronous at the expected angle below Kt·I, slip detected above |
| T07 | Reversal +4 → −4 rev/s | Synchronous, deceleration limit kept, final speed |
| T08 | Plant R ×0.9 to 1.3, L ×0.8 to 1.2, J ×5, load 0.15 N·m | Overshoot below 10 %, settling below 2 ms, error below 1 %; ramp to 4 rev/s synchronous |
| T09 | 8 rev/s, both topologies | Shared leg reaches the voltage limit first, both synchronous |
| X03 | Move 160 microsteps | Motion generator and loop equal the Python twin |
| T10 | Switching stage: hold 2 A, ±2 A with dead time, 4 rev/s | Ideal switches equal the averaged model, ripple and dead time error equal the formulas, synchronous, peak below the trip |
| X04 | Switching stage, shared leg, 4 rev/s | Scilab equals the Python twin |
| T11 | Angle step; speed sweep with detent; 5, 8, 11 rev/s | Natural frequency ±5 %, detent resonance ±10 %, oscillation below 1° and 5° below the voltage limit, above 20° beyond it |
| T12 | Unpowered rotor, Tc 0.01 N·m, 50 rad/s | Spin-down time from the formula, ±1 % |
| T13 | Stall scenarios in the prototype | Must trip: stall flag in the first fault, 0 to 70 ms after the pull-out. Must not trip: no fault, below 2 full steps from the command |
| S01 | SIL: current step, locked rotor | Library in Xcos equals the prototype, currents below 5 mA |
| S02 | SIL: move 640 microsteps | Currents below 5 mA, commanded angle identical, rotor below 0.05 microstep |
| S03 | SIL: 8 rev/s, both topologies | Current amplitude as the prototype |
| S04 | SIL: run 4 rev/s, switching stage | Currents below 5 mA, peak as the prototype |
| S05 | SIL: run 4 rev/s, averaged with leg error against switching | Commanded vA within 40 % of the ideal-leg difference, amplitude below 2 mA |
| S06 | SIL: the T13 scenarios on both stages, FIXED and FLOAT | Same verdicts; same fault within one slow period, currents below 5 mA while synchronous |
| S07–S09 | Lab: signal inputs equal the table; out of range, sines; through a fault | Currents below 1e-9 A, values and flag as specified, the run reaches `T_end` |
| S10 | Lab: four superblocks, file read back, columns 1–19 equal `stepper_sil`, scenarios | Below 1e-9, every scenario as expected |
| S11 | Report | Every panel has title, unit and legend; first fault named |
| M01 | MIL against SIL | Currents below 5 mA, commanded position below 1e-6 microsteps, same fault within 1 ms, input limits, refused configurations |
| C01–C03 | Settings, library folder, stage file, window and its callbacks | Order and messages as specified |

## Known gaps

- Everything above is simulation. The model has not been validated against a measured motor in this
  manual, and several motor parameters are derived or assumed.
- All results come from Windows. The Linux path of `sil_build()` was not run for this manual.
- M01 does not cover sine inputs or the table mode in MIL.
- The look of the window and the diagrams has no automatic test.
