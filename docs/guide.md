# Getting Started

This chapter takes you from an empty folder to the first simulation in the Lab window. Plan a few
minutes, plus the one compilation of the library.

## What you need

| Item | Note |
| --- | --- |
| Scilab 2027 | With Xcos. All results in this manual come from 2027.0.0 on Windows 10 |
| C compiler | The one Scilab uses for toolboxes. On Windows: Visual Studio or the free Build Tools, with the workload *Desktop development with C++*. `sil_build()` also has a gcc path for Linux, which was not run for this manual |
| Stepper-C | The sources of the library; only needed once to compile, and again when they change |
| Python 3 with numpy and scipy | Optional: the Python twin, and test X04 with another stage file |

## Get the sources

Clone both repositories next to each other. The model then finds the library by itself:

```bash
git clone https://github.com/AntonVynohradov/Stepper-Lab.git
git clone https://github.com/AntonVynohradov/Stepper-C.git
```

The library can also be in any other folder: select it in the window, or see the
[search order](configuration.md#the-stepper-c-folder).

## First run: the Lab window

1. Start Scilab, open **`start_lab.sce`** in its editor, and press **F5**.
2. Check the first line of the window, *Stepper-C library*. It shows the folder and `OK`. If it
   asks for the folder, press **Browse** and select it.
3. Leave *SIL: C code* selected. Choose the scenario *Velocity step to 1 rev/s*.
4. Press **Run**. The first run compiles the library. The status line at the bottom then says that
   the run finished.
5. Press **Report**. A figure shows currents, speed, position, load angle, voltages and the
   detector, each with a title, a unit and a legend.

To see the diagram itself, press **Open in Xcos**, then press Start in Xcos. [The Lab Window and
Diagrams](lab.md) explains each part.

## First run: from a script

Everything the window does is available as a function. Load the functions once per Scilab session, from
the root of the repository; `start_lab.sce` and `start_sil.sce` do it too. The examples in this manual assume
that this has been done.

```scilab
exec("loader.sce", -1);
res = lab_run("SIL", 3);        // scenario 3: velocity step to 1 rev/s
d = sil_report_data(res);       // the panels of the report, and the first fault
mprintf("%d samples, last speed %.0f rpm\n", size(res.values, 1), res.values($, 5));
```

Results are in `res.time` and `res.values`; [Reference](reference.md#results-silvalues) lists the
columns.

## The original SIL diagram

`start_sil.sce` compiles the library and opens `xcos/stepper_sil.zcos`, the diagram with the command
table as its only input. It is kept unchanged next to the Lab, and its scenario is set under
**Simulation → Set Context**. [MIL and SIL](mil-sil.md#the-stepper_sil-diagram) describes it.

## The model without a controller

The motor equations can also be used alone, with a voltage source:

```scilab
mot = motor_17hs19_2004s1();
srcp = struct("V", 3.2, "f_el", 5, "TL", 0);
t = 0:1e-4:0.04;
x = stepper_simulate(mot, src_open_loop, srcp, t);
```

`demos/demo_open_loop.sce` and `demos/demo_switching.sce` are scripts with plots.

## Tests

From the root of the repository, with the `bin` folder of Scilab in `PATH`:

```bash
WScilex-cli -quit -f tests/run_all.sce      # Windows
scilab-cli -quit -f tests/run_all.sce       # Linux
```

A full run takes hours, so use the groups of the window or `SSTEP_TEST_FILTER` for everyday work.
[Verification](verification.md) has the details.

## Documentation tooling

The website and this manual come from one set of Markdown files in `docs/`.

| Task | Command |
| --- | --- |
| Install the tools | `python -m pip install -r requirements-dev.txt` |
| Preview the website | `mkdocs serve` |
| Check the website | `mkdocs build --strict` |
| Build the PDF | `python tools/docs/build_manual.py` (needs the Pango library) |
| Redraw the figures | `python tools/docs/diagrams.py` |
| Export the figure data after a model change | `WScilex-cli -quit -f tools/docs/export_figures.sce` |
| Run the Scilab examples of the documentation | `WScilex-cli -quit -f tools/docs/check_examples.sce` |

On Linux, the command is `scilab-cli`. The version printed in the manual comes from the git tag `v<version>`. Without a tag it reads
`development`.
