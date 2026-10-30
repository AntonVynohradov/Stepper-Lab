# sfs-stepper-sim

Scilab/Xcos model of a two-phase hybrid stepper motor, used to develop and test the
control library **Stepper-C** (`sfs-stepper-ctrl`) before it runs on hardware.

> **Status:** averaged motor model, averaged and switching power stage (both
> topologies, dead time, diodes), discrete PI current loop and motion generator;
> software in the loop (SIL) in Xcos with the Stepper-C C library; detent torque and
> dry friction; tests T00–T12, B01, X01–X04, S01–S04.

## Software in the loop (Xcos) — start here

The Xcos diagram `xcos/stepper_sil.zcos` runs the **real C code of Stepper-C**
(`sfs-stepper-ctrl`) against the motor model. No Scilab knowledge is needed:

1. Install Scilab 2027 and, on Windows, Visual Studio (or the free *Build Tools*) with
   the workload *Desktop development with C++* (needed once, to compile the library).
2. Put `sfs-stepper-ctrl` next to this folder (or set `SSTEP_CTRL_DIR`).
3. In Scilab open **`start_sil.sce`** and press **F5**. The library is compiled the
   first time (a few seconds), then Xcos opens the diagram.
4. Press **Start** (▶). The scope shows: phase currents and their references, speed
   (rpm), load angle (electrical degrees; above 90° the motor is close to losing
   steps) and the commanded and real rotor positions (microsteps).
5. Change the scenario in **Simulation → Set Context** and press Start again:

   | Variable | Meaning | Example |
   | --- | --- | --- |
   | `cmds` | commands `[t kind value]`: 1 MOVE to (µsteps), 2 RUN (µsteps/s), 3 STOP | `[0 2 12800; 0.15 2 -12800]` |
   | `loads` | load torque steps `[t TL]` (N·m) | `[0 0; 0.15 0.25]` |
   | `brd.topology` | `"shared"` (B-G431B-ESC1) or `"full"` (two H-bridges) | `"full"` |
   | `brd.stage` | `"avg"` averaged power stage or `"switching"` (PWM, dead time, diodes) | `"switching"` |
   | `mp` | microsteps, speed and acceleration limits, run / hold current | `mp.v_max = 25600` |
   | `locked` | 1: rotor locked (current loop test) | `1` |
   | `T_end` | simulated time (s) | `1.0` |

   The results of the last run are in the variable **`sil`**: `sil.time` and
   `sil.values` with the columns iA, iB, iA_ref, iB_ref (A), speed (rpm), load angle
   (deg), commanded and rotor position (µsteps), vA, vB (V), largest |iA|, |iB| in the
   period (A, with the PWM ripple when `brd.stage = "switching"`).

```
 SampleCLK (fs) ──► Stepper-C (C library) ──PWM compare──► Power stage + motor + ADC
                         ▲                                         │
                         └──────────── ADC samples ◄───────────────┘
                     Monitor ──► scope, workspace variable "sil"
```

The blocks compute exactly what the microcontroller computes: ADC counts in, PWM
compare values out, one fast loop period of computation delay. The numeric backend is
chosen in `start_sil.sce` (`"FIXED"` or `"FLOAT"`). Tests S01–S04 run the diagram
without windows and compare it with the Scilab prototype sample by sample (currents
within the ADC step, commanded angle identical).

## Model

State `x = [iA; iB; w; theta]` (A, A, rad/s, rad), `p = Ns / 4`:

```
L diA/dt = vA - R iA - eA        eA = -Kt w sin(p theta)
L diB/dt = vB - R iB - eB        eB =  Kt w cos(p theta)
J dw/dt  = Te - TL - Tf          Te =  Kt (-iA sin(p theta) + iB cos(p theta)) - Td sin(4 p theta)
dtheta/dt = w                    Tf =  B w + Tc tanh(w / w_c)
```

Detent torque `Td` (one period per full step) and dry friction `Tc` are 0 by default
(not in the datasheet); set `mot.Td`, `mot.Tc` to study them (also in the Xcos context).
The dry friction is smoothed below `w_c` = 0.05 rad/s, so there is no stiction.

**Convention.** With the current vector `iA = I cos(phi)`, `iB = I sin(phi)` the rotor
settles at `p theta = phi`: increasing `phi` turns the rotor forward, and phase A leads
phase B by 90 degrees. Stepper-C uses the same convention. (The first project note used
`iA = I sin(phi)`, `iB = I cos(phi)`, which with these torque equations turns the rotor
backwards.)

The power balance `eA iA + eB iB = Te w` holds (test T00).

## Current loop (prototype of the Stepper-C fast loop)

Every fast loop period `Ts = 1 / fs` (as on the MCU):

1. sample `iA, iB`;
2. references `iA* = I cos(phi)`, `iB* = I sin(phi)`, back-EMF feed-forward
   `vff = Kt w* [-sin(phi); cos(phi)]`;
3. PI per phase: `v = Kp e + integ + vff`, `Kp = L wc`, `Ki = R wc` (pole-zero
   cancellation, closed loop of first order with bandwidth `wc`);
4. power stage limit (`bridge_avg`): the voltage vector is scaled down with its direction
   kept; full bridge `|vA|, |vB| <= Vbus`, shared leg `max(a,b,0) - min(a,b,0) <= 1`
   (circular amplitude `Vbus / sqrt(2)`, common leg centred);
5. anti-windup back-calculation `integ += Ki Ts e + Ts/Tt (vsat - v)`, `Tt = L/R`;
6. the voltage is applied in the **next** period (one-period computation delay);
7. the motor is integrated over `Ts` with RK4 (4 sub-steps).

## Motion generator (twin of the Stepper-C slow loop)

`motion_init` + `ref_motion` (`motion_slow`, `motion_step`) reproduce the integer algorithm
of Stepper-C **bit for bit** (checked against `src/sstep_motion.c` over 3 million periods):
positions in 1/65536 microstep, velocities in 1/65536 microstep per slow period.
Commands `[t kind value]`: 1 MOVE target, 2 RUN velocity, 3 STOP, 4 LOAD torque
(simulation only). Every slow period (1 kHz):

- velocity changes by at most `acc` (speeding up) or `dec` (slowing down) per period;
- a move aims at the largest `v_new` for which this period's distance `(v + v_new)/2`
  plus the braking distance `v_new^2 / (2 dec)` fits into the distance left (exact for
  the discrete profile: no overshoot, exact stop on the target; the last period stops
  from below `2 dec`);
- the fast loop advances the electrical angle by a constant increment
  `trunc(dp * 2^(14 - log2 M) / (fs / f_slow))` per period, re-synchronised every slow
  period (one electrical turn = 2^32, one microstep = 2^30 / M);
- run current while moving, hold current after `hold_delay` without motion.

**Rule:** the Scilab model always follows the algorithm of the library. A change in
Stepper-C is ported here (and to `tools/ref_motion.py`) in the same step.

## Power stage: averaged or switching

`brd.stage` selects the power stage model (in the Scilab prototype and in the Xcos plant
block):

- `"avg"` (default): each leg produces its mean voltage `duty * Vbus` — fast, used by
  most tests;
- `"switching"`: every PWM period is split at the switching instants of the
  centre-aligned legs (period from counter peak to counter peak, i.e. between two
  current samples). Every turn-on is delayed by the dead time `dead_ns`; during the
  dead time the body diode carries the current (leg at `-Vd` or `Vbus + Vd` depending
  on the current direction); conducting switches drop `Rds(on) * i`.

The switching model shows the current ripple, the peak current per period and the
voltage error of the dead time, `-2 sign(i) (Vbus + 2 Vd) td / Ts` per phase — the
current loop removes it in steady state, but it distorts the current near zero
crossings. With 3 mH the ripple is small: 23 mA p-p at 2 A on a locked phase
(analytic: `(Vbus - R I) (|vA| / Vbus) Ts / (2 L)`).

The stage values come from the ngspice model of the ESC1 leg
(`sfs-stepper-power-spice`, `python tools/run_tests.py --export` writes
`params/stage_esc1_spice.sci` and `.json`; do not edit them by hand):

| Value | ngspice result | Note |
| --- | --- | --- |
| effective dead time `dead_ns` | 133 ns | timer 500 ns (assumption) minus the turn-off delay of the MOSFET (slow, weak gate drive); fitted for 1–3 A |
| leg resistance `rds_on` | 3.4 mΩ | STL180N6F7 1.9 mΩ, low side + 3 mΩ shunt (assumption), averaged |
| body diode `vd` | 0.73 V | at 2 A |

Below 1 A the real dead time error is smaller than the model (the phase current
charges the switch node too slowly): 50 % of the model at 0.5 A, 25 % at 0.25 A.
`dead_ns` is also passed to Stepper-C (`dead_time_ns`) — the value a compensation
needs is the effective one.

Demo with plots: `exec("demos/demo_switching.sce", -1);`

## Resonance and robustness (Python twin = Scilab, 24 V, 2 A, shared leg)

- **Natural frequency** of the rotor on the current vector: `sqrt(p Kt I / J) / 2 pi`
  = 254 Hz at 2 A without load (the loop gives 248 Hz).
- **Low-speed resonance:** a detent torque of 0.02 N·m (assumption) excites it at
  `4 p n = fn`, i.e. about 1.2–1.3 rev/s: rotor oscillation ±13.6° there, ±3–5° away from
  it. Avoid running continuously around this speed or add load inertia / damping.
- **Mid-band instability:** below the voltage limit the current loop damps the rotor
  (0° oscillation at 5 rev/s); once the bridge limits the voltage the oscillation grows:
  ±1.3° at 8 rev/s, limit cycle ±48° at 11 rev/s (24 V, shared leg; two H-bridges: from
  about 12 rev/s). A higher bus voltage moves it up; damping in the library (e.g. from
  the back-EMF estimate) is a later topic.
- **Parameter uncertainty (T08):** with the gains for the nominal motor and the plant at
  R ×0.9…1.3, L ×0.8…1.2 the current step stays below 10 % overshoot, but the 2 %
  settling grows from 0.4 ms to 1.6 ms (R +30 %, L −20 %: hot winding). 5× load
  inertia and 0.15 N·m load at 4 rev/s stay synchronous (load angle 46°).

## Measured with the model (Python twin, 24 V, 2 A, bandwidth 1 kHz)

Current amplitude at constant speed, unloaded rotor:

| Speed | Two H-bridges | Shared leg (B-G431B-ESC1) |
| --- | --- | --- |
| 4 rev/s | 1.97 A | 1.97 A |
| 6 rev/s | 1.93 A | 1.90 A (voltage limit reached) |
| 8 rev/s | 1.86 A | 1.56 A |
| 10 rev/s | 1.46 A | 1.02 A |
| 12 rev/s | 1.04 A | 0.66 A |

Below the voltage limit the amplitude drops slightly with speed because the current
loop (1 kHz, one period delay) lags the rotating reference (about 4 deg at 1 rev/s).

## Parameters: 17HS19-2004S1

| Parameter | Value | Source |
| --- | --- | --- |
| R | 1.60 Ω | datasheet (±10 %); older listings: 1.4 Ω — **measure** |
| L | 3.0 mH | datasheet at 1 kHz (±20 %) |
| I rated | 2.0 A | datasheet |
| Kt | 0.2086 N·m/A | derived from 0.59 N·m holding torque, both phases at 2 A — **measure back-EMF** |
| Ns | 200 | 1.8° |
| J | 82 g·cm² | datasheet, rotor only |
| B | 1e-5 N·m·s/rad | assumption |
| Td | 0 | not in the datasheet |

Measuring Kt: spin the shaft (drill), measure the open-circuit phase voltage amplitude
`E` and the speed `w`: `Kt = E / w` (V·s/rad = N·m/A).

## Usage

```scilab
exec("loader.sce", -1);
mot = motor_17hs19_2004s1();
srcp = struct("V", 3.2, "f_el", 5, "TL", 0);
t = 0:1e-4:0.4;
x = stepper_simulate(mot, src_open_loop, srcp, t);
```

Demo with plots: `exec("demos/demo_open_loop.sce", -1);`

## Tests

```bash
scilab-cli -f tests/run_all.sce          # exits with 1 when a test fails
python tools/ref_model.py                # motor tests in the Python twin
python tools/ref_ctrl.py                 # current loop tests in the Python twin
python tools/ref_model.py --write        # regenerate tests/reference/x01_*.csv
python tools/ref_ctrl.py --write         # regenerate tests/reference/x02_*.csv
python tools/ref_motion.py [--write]     # motion tests, tests/reference/x03_*.csv
python tools/ref_switching.py [--write]  # switching stage tests (T10), x04_*.csv
python tools/ref_ext.py                  # T08, T11, T12 in the Python twin
```

| Test | Stimulus | Check |
| --- | --- | --- |
| T00 | random states, current vector | power balance, stable rotor position |
| T01 | constant voltage on phase A | steady state current V/R (locked and free rotor) |
| T02 | voltage step, locked rotor | i(t) = V/R (1 − e^(−t/τ)), τ = L/R |
| X01 | rotating voltage, 5 Hz electrical | Scilab result equals the Python twin |
| B01 | random voltage requests, both topologies | duties 0..1, direction kept, exact when feasible, max amplitude |
| T03 | microstepping 2 A, 10 Hz electrical | A leads B by 90°, amplitude, rotor follows the field |
| T04 | current step 0 → 2 A, locked rotor, both topologies | overshoot < 5 %, settling (2 %) < 1 ms, error < 0.1 % |
| X02 | current step, shared leg | discrete loop equals the Python twin |
| T05 | move 2 rev, 5 rev/s, 50 rev/s² | limits kept, target exact, rotor follows (load angle < 45°) |
| T06 | load step at 1 rev/s: 0.25 and 0.5 N·m | synchronous with angle asin(T/(Kt I)) below Kt·I, slip detected above |
| T07 | reversal +4 → −4 rev/s | synchronous, deceleration limit kept, final speed |
| T08 | controller for the nominal motor, plant R ×0.9–1.3, L ×0.8–1.2, J ×5, load 0.15 N·m | current step overshoot < 10 %, settling < 2 ms, error < 1 %; ramp to 4 rev/s synchronous |
| T09 | 8 rev/s, both topologies | shared leg reaches the voltage limit first, both synchronous |
| X03 | move 160 microsteps | motion generator + loop equal the Python twin |
| S01 | SIL: current step, locked rotor | C library in Xcos equals the prototype (currents < 5 mA) |
| S02 | SIL: move 640 microsteps | currents < 5 mA, commanded angle identical, rotor < 0.05 µstep |
| S03 | SIL: 8 rev/s, both topologies | current amplitude as the prototype (T09) |
| T10 | switching stage: hold 2 A, ±2 A with dead time, 4 rev/s | ideal switches = averaged model (< 1 mA), ripple = analytic (5 %), dead time error = formula (1 %), synchronous, peak < trip |
| X04 | switching stage, shared leg, 4 rev/s | Scilab equals the Python twin |
| T11 | angle step 0.1 rad; speed sweep 0.8–1.8 rev/s with detent 0.02 N·m; 5, 8, 11 rev/s | natural frequency ±5 %, detent resonance at `4 p n = fn` ±10 %, oscillation < 1° / < 5° below the voltage limit, > 20° beyond |
| T12 | unpowered rotor without back-EMF, Tc 0.01 N·m, 50 rad/s | spin-down time `(J/B) ln((w0 + Tc/B)/(w + Tc/B))` ±1 % |
| S04 | SIL: run 4 rev/s, switching stage | currents < 5 mA, peak current as the prototype |

## Structure

```
params/      motor, board, controller and motion parameters (functions returning structures)
model/       model functions (state equations, sources, simulation)
tests/       test scripts, runner, reference data
sil/         software in the loop: Xcos blocks in C (sil/c), build of the library
xcos/        Xcos diagram, its generator (sil_diagram) and block interfaces (xcos/blocks)
tools/       Python twin of the model
demos/       scripts with plots
```

## Related repositories

| Repository | Role |
| --- | --- |
| `sfs-stepper-ctrl` | Stepper-C control library |
| `sfs-stepper-sim` | this model |
| `sfs-stepper-power-spice` | ngspice models of the power stage |

## License

Apache License 2.0, see [LICENSE](LICENSE). SystemFromScratch® is a registered trademark
of Anton Vynohradov.
