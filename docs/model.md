# Motor and Control Model

This chapter gives the equations of the plant and of the prototype controller, in the order the code
computes them. The plant is implemented twice, in Scilab (`model/`) and in portable C
(`sil/c/sstep_plant.c`), and the controller prototype follows Stepper-C. Tests compare them.

**Rule:** a change of the model is made in `sstep_plant.c` and in the Scilab prototype together, and
a change in Stepper-C is ported to the prototype and to the Python twin in the same step.

## Motor

The state is $x = [i_A, i_B, \omega, \theta]$ in A, A, rad/s and rad, with $p = N_s / 4$ electrical
turns per mechanical turn.

$$
\begin{aligned}
L \frac{di_A}{dt} &= v_A - R\, i_A - e_A, & e_A &= -K_t\, \omega \sin(p\theta) \\
L \frac{di_B}{dt} &= v_B - R\, i_B - e_B, & e_B &= K_t\, \omega \cos(p\theta) \\
J \frac{d\omega}{dt} &= T_e - T_L - T_f \\
T_e &= K_t \left(-i_A \sin(p\theta) + i_B \cos(p\theta)\right) - T_d \sin(4 p\theta) \\
T_f &= B\, \omega + T_c \tanh(\omega / \omega_c) \\
\frac{d\theta}{dt} &= \omega
\end{aligned}
$$

$T_d$ is the detent torque, with one period per full step, and $T_c$ is the dry friction, smoothed
below $\omega_c$, so the model has no stiction. Both are 0 by default, because the datasheet does not
give them. The power balance $e_A i_A + e_B i_B = T_e\, \omega$ holds without detent torque (test
T00).

**Convention.** With $i_A = I \cos\varphi$ and $i_B = I \sin\varphi$ the rotor settles at
$p\theta = \varphi$. Increasing $\varphi$ turns the rotor forward, and phase A leads phase B by 90°.
Stepper-C uses the same convention.

## Current loop

Every fast period $T_s = 1 / f_s$, `ctrl_period()` does the following, as the fast loop of Stepper-C:

1. Sample $i_A$, $i_B$.
2. Compute the references and the back-EMF feed-forward at the commanded speed $\omega^*$:

    $$
    i_A^* = I \cos\varphi, \quad i_B^* = I \sin\varphi, \quad
    v_{ff} = K_t\, \omega^* \begin{bmatrix} -\sin\varphi \\ \cos\varphi \end{bmatrix}
    $$

3. Run one PI controller per phase, with pole-zero cancellation, so the closed loop is of first order
   with bandwidth $\omega_c = 2\pi \cdot$ `bw_hz`:

    $$
    v = K_p\, e + \textit{integ} + v_{ff}, \quad K_p = L\, \omega_c, \quad K_i = R\, \omega_c
    $$

4. Limit the voltage to what the power stage can give, scaling the vector and keeping its direction
   (`bridge_avg()`). With two H-bridges, $\lvert v_A \rvert, \lvert v_B \rvert \le V_{bus}$. With the
   shared leg, $\max(a, b, 0) - \min(a, b, 0) \le 1$ for the duty differences $a$ and $b$, which gives a circular amplitude
   of $V_{bus} / \sqrt{2}$.
5. Update the integrator with back-calculation, $T_t = L / R$:

    $$
    \textit{integ} \leftarrow \textit{integ} + K_i T_s\, e + \frac{T_s}{T_t} (v_{sat} - v)
    $$

6. The voltage is applied in the **next** period, the one-period computation delay of the MCU.
7. The prototype integrates the motor over $T_s$ with RK4, with `ctl.nsub` sub-steps.

## Motion generator

`motion_init()` and `ref_motion()` reproduce the integer algorithm of the slow loop of Stepper-C.
Positions are in 1/65536 microstep, and velocities in 1/65536 microstep per slow period. The commands
are `[t kind value]`: 1 MOVE target, 2 RUN velocity, 3 STOP, and in the prototype also 4 LOAD torque.

Every slow period:

- The velocity changes by at most `acc` when speeding up, or `dec` when slowing down.
- A move aims at the largest $v_{new}$ for which this period's distance $(v + v_{new})/2$ plus the
  braking distance $v_{new}^2 / (2\, dec)$ fits into the distance left:

    $$
    v_{new} = \frac{-dec + \sqrt{dec^2 + 8\, dec\, \lvert rem \rvert - 4\, dec\, \lvert v \rvert}}{2}
    $$

    This is exact for the discrete profile: no overshoot, and an exact stop on the target. The last
    period stops from a velocity at or below 2 `dec`.
- The fast loop advances the electrical angle by a constant increment per period, and it is set again
  every slow period. One electrical turn is $2^{32}$, and one microstep is $2^{30} / M$.
- The run current applies while moving, and the hold current after `hold_delay` without motion.

Test X03 compares this generator with the Python twin.

## Power stage

`brd.stage` selects the model, in the prototype and in the plant block.

### Averaged

`"avg"` is the default: each leg produces its mean voltage, duty × $V_{bus}$. It is fast and is used by
most tests.

With `brd.leg_error = stage_active().leg_error` every leg also gets the voltage error measured with
ngspice against the leg current, linear between the table points. This covers dead time, body diodes
and leg resistance, at the speed of the averaged model. The table is valid at its operating point
(`stg.vbus`, `stg.fs`). The default `[]` is ideal legs.

### Switching

`"switching"` splits every PWM period at the switching instants of the centre-aligned legs, from
counter peak to counter peak, which is between two current samples.

- Every turn-on is delayed by the dead time `dead_ns`.
- During the dead time the body diode carries the current, so the leg is at $-V_d$ or $V_{bus} + V_d$,
  depending on the current direction.
- Conducting switches drop $R_{ds(on)}\, i$.

It shows the current ripple, the peak current per period and the voltage error of the dead time:

$$
\Delta v = -2\, \mathrm{sign}(i)\, (V_{bus} + 2 V_d)\, \frac{t_d}{T_s}
$$

per phase. The current loop removes this in steady state, but it distorts the current near zero
crossings. The ripple on a locked phase is

$$
\Delta i_{pp} = (V_{bus} - R I)\, \frac{\lvert v_A \rvert}{V_{bus}}\, \frac{T_s}{2 L}
$$

The results are in [Measured Results](results.md#power-stage).

## Stall detection

`stall_init()`, `stall_observe()`, `stall_check()` and `stall_status()` are the detector of Stepper-C.
`ctl.stall_time_ms > 0` switches it on, and after the fault the outputs are off and the reference is
frozen, as in the library.

**Fast loop.** The back-EMF comes from the limited voltage. The voltage computed two calls ago is the
one applied from sample $k-1$ to $k$:

$$
e_k = v_{k-2} - \left(\frac{L}{T_s} + \frac{R}{2}\right) i_k + \left(\frac{L}{T_s} - \frac{R}{2}\right) i_{k-1}
$$

It is saturated at $\pm V_{bus}$, rotated into the commanded frame ($d$ along the current, $q$ 90° ahead),
and filtered by a first-order low-pass with the time constant `stall_filter_us`.

**Slow loop.** In integer millivolts, against the command of the period that just ended, with
$E = \lvert K_e\, \omega \rvert$, `sync` = `stall_sync_pct`, and $e_s$ the back-EMF at `stall_speed`:

| Case | The condition is true when |
| --- | --- |
| In motion, $\lvert v \rvert \ge$ `stall_speed` | $\mathrm{sgn}(v)\, e_q < \frac{sync}{100} E$, with $e_d$ between $-(E/3 + R I / 10)$ and $E/3 + 2 R I / 5$: a rotor that does not turn |
| Reversed back-EMF | $\mathrm{sgn}(v)\, e_q < -(E/2 + e_s)$ keeps the condition true for $N/2$ periods |
| At rest or slow | The deviation of $(e_d, e_q - K_e \omega)$ from its running mean, with a time constant of $N$ periods, is larger than $e_s$ |

An up/down counter rises while the condition is true and falls otherwise. The fault trips at
$N$ = `stall_time_ms` in slow periods. With `ctl.protect = 1`, an overcurrent trip while the counter
is above zero reports both `SSTEP_FAULT_OVERCURRENT` and `SSTEP_FAULT_STALL`.

The results and the limits of the detector are in
[Measured Results](results.md#stall-detection).

The Python twin has no stall detector; the X tests run with detection off.

## Parameters of the example

The values and their sources are in [Configuration](configuration.md#motor-mot). Those marked as
derived or assumed, in particular `Kt`, `B` and `Td`, should be measured on your motor before you trust
results near a limit.
