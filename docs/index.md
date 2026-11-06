---
title: Stepper-Lab — Stepper Motor Model in Scilab/Xcos
description: Run the C code of Stepper-C or a Scilab prototype against a model of a stepper motor and its power stage, with any input signal and labelled results.
template: home.html
hide:
  - navigation
  - toc
---

<span class="sfs-eyebrow">Quick start</span>

## A first simulation in five steps

1. Install Scilab 2027 and, on Windows, Visual Studio or the Build Tools with the workload
   *Desktop development with C++*.
2. Clone [Stepper-Lab](https://github.com/AntonVynohradov/Stepper-Lab) and
   [Stepper-C](https://github.com/AntonVynohradov/Stepper-C) next to each other.
3. In Scilab, open `start_lab.sce` and press **F5**.
4. Choose the scenario *Velocity step to 1 rev/s* and press **Run**. The first run compiles the library.
5. Press **Report** to see the currents, speed, position and load angle.

The same run from a script:

```scilab
exec("loader.sce", -1);         // from the root of the repository
res = lab_run("SIL", 3);
d = sil_report_data(res);
mprintf("%d samples, %d panels\n", size(res.values, 1), size(d.panels));
```

## Part of a family

Stepper-Lab tests [Stepper-C](https://antonvynohradov.github.io/Stepper-C/), the control library, and
takes its power stage values from
[sfs-stepper-power-spice](https://github.com/AntonVynohradov/sfs-stepper-power-spice), the ngspice models.

## Find your way

| If you want to… | Read |
| --- | --- |
| Understand what the model does and does not do | [Introduction](overview.md) |
| Install it and run it the first time | [Getting Started](guide.md) |
| Use the window, the diagrams and your own signals | [The Lab Window and Diagrams](lab.md) |
| Choose between the C library and the Scilab prototype | [MIL and SIL](mil-sil.md) |
| Set the motor, the board or the stage file | [Configuration](configuration.md) |
| See the equations | [Motor and Control Model](model.md) |
| See what the model shows | [Measured Results](results.md) |
| Look up a column, a block or a function | [Reference](reference.md) |
| Run the tests | [Verification](verification.md) |
| Read everything offline or print it | [PDF user manual](https://antonvynohradov.github.io/Stepper-Lab/downloads/Stepper-Lab_User_Manual.pdf) |
