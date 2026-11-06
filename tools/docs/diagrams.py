#******************************************************************************
# Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
#
# Description: Generates the SVG figures of the documentation
#              (docs/assets/images/*.svg). The structure figures follow
#              xcos/lab_diagram.sci, the result figures are drawn from the CSV files of
#              docs/assets/data (tools/docs/export_figures.sce). No Scilab needed:
#                  python tools/docs/diagrams.py
#
# Author:      Anton Vynohradov
# Email:       avynohradov@systemfromscratch.com
# License:     Apache License 2.0
# Copyright (c) 2026 Anton Vynohradov
#
# SPDX-License-Identifier: Apache-2.0
#*****************************************************************************

"""Documentation figures of Stepper-Lab, built with svgkit. Preview every figure (headless
Chrome --screenshot) and check its labels against the code after a change."""

from __future__ import annotations

import csv
import math
import os
import sys
from typing import Dict, List, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from svgkit import (BLUE, GREEN, GREY, HERO_DEFS, HERO_STYLE, Iso, LIGHT, LINE, MONO, NAVY,  # noqa: E402
                    ORANGE, RED, SANS, TEAL, WHITE, Svg, _write)

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
OUT = os.path.join(ROOT, "docs", "assets", "images")
DATA = os.path.join(ROOT, "docs", "assets", "data")

SOFT_TEAL = "#E6F4F5"
SOFT_ORANGE = "#FFF7EC"
SOFT_GREEN = "#EAF5EA"
SOFT_BLUE = "#E8F0FB"


def box(s: Svg, x, y, w, h, title, lines=(), fill=LIGHT, stroke=LINE, color=NAVY, size=15,
        mono_lines=False) -> None:
    """A rounded box with a bold title and smaller lines below it."""
    s.rect(x, y, w, h, fill=fill, stroke=stroke, rx=10)
    top = y + h / 2 - (len(lines) * 17) / 2 + (5 if not lines else -2)
    s.text(x + w / 2, top, title, size=size, weight="bold", fill=color)
    for i, value in enumerate(lines):
        s.text(x + w / 2, top + 20 + i * 17, value, size=12.5, fill=GREY, mono=mono_lines)


def arrow(s: Svg, x1, y1, x2, y2, color=GREY, label="", dy=-8, size=12, dash=False) -> None:
    kind = {TEAL: "teal", ORANGE: "orange", GREEN: "green"}.get(color, "end")
    s.line(x1, y1, x2, y2, color=color, arrow=kind, dash=dash)
    if label:
        s.text((x1 + x2) / 2, (y1 + y2) / 2 + dy, label, size=size, fill=color)


# ==============================================================================
# STRUCTURE
# ==============================================================================

def architecture() -> None:
    """The project and what it takes from the two related projects."""
    s = Svg(960, 400, "Architecture of Stepper-Lab")
    box(s, 30, 30, 240, 66, "Stepper-C", ["C sources, compiled by sil_build()"], fill=SOFT_TEAL, stroke=TEAL)
    box(s, 690, 30, 240, 66, "sfs-stepper-power-spice", ["stage file: ngspice results (JSON)"],
        fill=SOFT_ORANGE, stroke=ORANGE, size=14)

    s.rect(20, 130, 920, 250, fill=WHITE, stroke=LINE, rx=12, dash=True)
    s.text(40, 152, "Stepper-Lab", size=14, weight="bold", anchor="start", fill=BLUE)

    box(s, 40, 215, 150, 90, "Inputs", ["command", "run current", "load torque"])
    box(s, 260, 170, 200, 80, "Controller: SIL", ["the C library in a block"], fill=SOFT_TEAL, stroke=TEAL)
    box(s, 260, 268, 200, 80, "Controller: MIL", ["the Scilab prototype"], fill=SOFT_BLUE, stroke=BLUE)
    box(s, 590, 200, 150, 120, "Plant (C)", ["power stage", "motor", "ADC"], fill=SOFT_ORANGE, stroke=ORANGE)
    box(s, 790, 215, 130, 90, "Outputs", ["named scopes", "variable sil", "report"])

    arrow(s, 190, 235, 258, 215)
    arrow(s, 190, 285, 258, 305)
    s.line(460, 195, 480, 195)
    s.line(460, 325, 480, 325)
    s.line(480, 195, 480, 325)
    arrow(s, 480, 240, 588, 240, label="PWM compare", dy=-8)
    arrow(s, 588, 285, 480, 285, label="ADC counts", dy=18)
    arrow(s, 740, 260, 788, 260)

    arrow(s, 150, 96, 330, 168, color=TEAL)
    arrow(s, 810, 96, 680, 198, color=ORANGE)
    s.save(os.path.join(OUT, "architecture.svg"))


def lab_hierarchy() -> None:
    """The top level of xcos/lab_diagram.sci and what the superblocks hold."""
    s = Svg(960, 470, "The four superblocks of the stepper_lab diagram")
    tops = [(30, "Inputs"), (260, "Controller"), (490, "Plant"), (740, "Outputs")]
    widths = {"Inputs": 170, "Controller": 170, "Plant": 170, "Outputs": 190}
    for x, name in tops:
        fill, stroke = {"Controller": (SOFT_TEAL, TEAL), "Plant": (SOFT_ORANGE, ORANGE)}.get(name, (LIGHT, LINE))
        s.rect(x, 90, widths[name], 110, fill=fill, stroke=stroke, rx=10)
        s.text(x + widths[name] / 2, 118, name, size=16, weight="bold")

    # clock
    box(s, 345, 16, 200, 44, "Clock, Ts = 1 / brd.fs", size=13)
    s.path("M345,38 L345,38", color=GREY)
    arrow(s, 400, 60, 345, 88, dash=True)
    arrow(s, 490, 60, 575, 88, dash=True)

    # top-level signals, widths from the links of lab_diagram
    for k, (name, y) in enumerate((("command", 140), ("run current", 160), ("load torque", 180))):
        s.text(40, y, name, size=12, anchor="start", fill=GREY)
    arrow(s, 200, 140, 258, 140)
    arrow(s, 200, 160, 258, 160)
    s.path("M200,180 L232,180 L232,212 L470,212 L470,178 L488,178", color=GREY, arrow="end")
    arrow(s, 430, 130, 488, 130, label="PWM (5)", dy=-8)
    arrow(s, 488, 160, 430, 160, label="ADC (3)", dy=16)
    s.text(270, 145, "ADC", size=11, anchor="start", fill=GREY)
    s.text(270, 165, "command", size=11, anchor="start", fill=GREY)
    s.text(270, 185, "run current", size=11, anchor="start", fill=GREY)
    s.text(500, 145, "PWM", size=11, anchor="start", fill=GREY)
    s.text(500, 185, "load", size=11, anchor="start", fill=GREY)
    arrow(s, 660, 130, 738, 130, label="signals (9)", dy=-8)
    s.path("M345,200 L345,232 L720,232 L720,175 L738,175", color=GREY, arrow="end")
    s.text(600, 248, "controller signals (15)", size=12, fill=GREY)

    # inside
    s.line(115, 200, 60, 280, color=LINE, dash=True)
    s.line(115, 200, 200, 280, color=LINE, dash=True)
    s.rect(30, 280, 170, 165, fill=WHITE, stroke=LINE, rx=10, dash=True)
    s.text(115, 302, "replace with any source", size=12, weight="bold", fill=BLUE)
    for i, name in enumerate(("lab_cmd", "lab_irun", "lab_load")):
        box(s, 50, 315 + i * 42, 130, 34, name, size=13)

    s.line(345, 232, 280, 280, color=LINE, dash=True)
    s.rect(260, 280, 170, 165, fill=WHITE, stroke=LINE, rx=10, dash=True)
    s.text(345, 302, "one block", size=12, weight="bold", fill=BLUE)
    box(s, 275, 315, 140, 44, "STEPPER_CTRL_IO", ["SIL"], fill=SOFT_TEAL, stroke=TEAL, size=12)
    box(s, 275, 379, 140, 44, "STEPPER_CTRL_MIL", ["MIL"], fill=SOFT_BLUE, stroke=BLUE, size=12)

    s.rect(490, 280, 170, 165, fill=WHITE, stroke=LINE, rx=10, dash=True)
    s.text(575, 302, "one block", size=12, weight="bold", fill=BLUE)
    box(s, 505, 330, 140, 70, "STEPPER_PLANT_IO", ["sstep_plant.c"], fill=SOFT_ORANGE, stroke=ORANGE, size=12)

    s.line(835, 200, 835, 280, color=LINE, dash=True)
    s.rect(740, 280, 190, 165, fill=WHITE, stroke=LINE, rx=10, dash=True)
    s.text(835, 302, "STEPPER_MONITOR_IO", size=12, weight="bold", fill=BLUE)
    for i, name in enumerate(("Phase currents [A]", "Speed [rpm]", "Position [microsteps]", "Load angle [deg]",
                              "State, faults, stall level", "variable sil")):
        s.text(835, 325 + i * 20, name, size=12, fill=GREY if i < 5 else BLUE, weight="normal" if i < 5 else "bold")
    s.save(os.path.join(OUT, "lab-hierarchy.svg"))


def mil_sil() -> None:
    """What differs between the two controllers, and what they share."""
    s = Svg(960, 340, "MIL and SIL use one layout")
    box(s, 30, 130, 170, 90, "Inputs", ["same limits", "NaN keeps last value"])
    box(s, 760, 130, 170, 90, "Plant + Outputs", ["sstep_plant.c, in C", "columns of sil"], fill=SOFT_ORANGE, stroke=ORANGE)

    box(s, 270, 50, 400, 110, "SIL: STEPPER_CTRL_IO", ["sstep_xcos.c calls the Stepper-C library", "FIXED or FLOAT, one per Scilab session",
                                                        "compiled once by sil_build()"], fill=SOFT_TEAL, stroke=TEAL)
    box(s, 270, 190, 400, 110, "MIL: STEPPER_CTRL_MIL", ["ctrl_mil_block() calls ctrl_period()", "Scilab code of one controller period",
                                                          "much slower than SIL"], fill=SOFT_BLUE, stroke=BLUE)
    arrow(s, 200, 160, 268, 110)
    arrow(s, 200, 190, 268, 240)
    arrow(s, 670, 105, 758, 155, color=TEAL)
    arrow(s, 670, 245, 758, 195)
    s.text(845, 110, "PWM compare in,", size=12, fill=GREY)
    s.text(845, 126, "ADC counts back", size=12, fill=GREY)
    s.save(os.path.join(OUT, "mil-sil.svg"))


# ==============================================================================
# RESULTS: curves from docs/assets/data/*.csv
# ==============================================================================

def read_csv(name: str) -> Tuple[List[str], List[List[float]]]:
    """Columns from a CSV file with # comment lines and one header line."""
    with open(os.path.join(DATA, name), encoding="utf-8", newline="") as f:
        rows = [r for r in csv.reader(line for line in f if not line.startswith("#"))]
    head = rows[0]
    cols = [[float(r[k]) for r in rows[1:]] for k in range(len(head))]
    return head, cols


def nice_ticks(lo: float, hi: float, n: int = 5) -> List[float]:
    span = (hi - lo) or 1.0
    step = 10 ** math.floor(math.log10(span / n))
    for m in (1, 2, 2.5, 5, 10):
        if span / (m * step) <= n:
            step *= m
            break
    first = math.ceil(lo / step - 1e-9) * step
    out, v = [], first
    while v <= hi + step * 1e-6:
        out.append(round(v, 10))
        v += step
    return out


def fmt(v: float) -> str:
    return f"{v:g}"


def panel(s: Svg, x, y, w, h, xs: Sequence[float], series, xlabel: str, ylabel: str, title: str,
          ylim=None, xlim=None, marks=False, legend_at="right") -> Tuple:
    """One plot. series: list of (label, values, color, dashed)."""
    xlo, xhi = xlim or (min(xs), max(xs))
    if ylim:
        ylo, yhi = ylim
    else:
        allv = [v for _, ys, _, _ in series for v in ys]
        ylo, yhi = min(allv), max(allv)
        pad = (yhi - ylo) * 0.08 or 1.0
        ylo, yhi = ylo - pad, yhi + pad
    px = lambda v: x + (v - xlo) / (xhi - xlo) * w  # noqa: E731
    py = lambda v: y + h - (v - ylo) / (yhi - ylo) * h  # noqa: E731
    s.rect(x, y, w, h, fill=WHITE, stroke=LINE, rx=2, sw=1)
    s.text(x + w / 2, y - 12, title, size=14, weight="bold")
    for t in nice_ticks(xlo, xhi):
        s.line(px(t), y, px(t), y + h, color="#E7ECF3", width=1)
        s.text(px(t), y + h + 16, fmt(t), size=11, fill=GREY)
    for t in nice_ticks(ylo, yhi):
        s.line(x, py(t), x + w, py(t), color="#E7ECF3", width=1)
        s.text(x - 7, py(t) + 4, fmt(t), size=11, fill=GREY, anchor="end")
    s.text(x + w / 2, y + h + 34, xlabel, size=12, fill=GREY)
    s.text(x - 44, y + h / 2, ylabel, size=12, fill=GREY, rotate=-90)
    for label, ys, color, dashed in series:
        pts = [(px(a), py(min(max(b, ylo), yhi))) for a, b in zip(xs, ys) if xlo <= a <= xhi]
        step = max(1, len(pts) // 900)
        d = "M" + " L".join(f"{a:.1f},{b:.1f}" for a, b in pts[::step])
        extra = ' stroke-dasharray="5 4"' if dashed else ""
        s.parts.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="1.7"{extra}/>')
        if marks:
            for a, b in pts:
                s.parts.append(f'<circle cx="{a:.1f}" cy="{b:.1f}" r="3.2" fill="{color}"/>')
    lx = x + w - 10 if legend_at == "right" else x + 12
    for i, (label, _, color, dashed) in enumerate(series):
        ly = y + 16 + i * 17
        if legend_at == "right":
            s.text(lx, ly + 4, label, size=11.5, fill=color, anchor="end", weight="bold")
        else:
            s.text(lx, ly + 4, label, size=11.5, fill=color, anchor="start", weight="bold")
    return px, py


def current_step() -> None:
    head, c = read_csv("current_step.csv")
    t = [v * 1e3 for v in c[head.index("t_s")]]
    s = Svg(960, 340, "Current step of the phase A current, rotor locked")
    series = [("reference", c[head.index("iA_ref_A")], GREY, True),
              ("iA, Stepper-C in Xcos (SIL)", c[head.index("iA_sil_A")], TEAL, False),
              ("iA, Scilab prototype (MIL)", c[head.index("iA_mil_A")], ORANGE, True)]
    panel(s, 90, 40, 830, 240, t, series, "Time [ms]", "Current [A]", "Current step, rotor locked", ylim=(0, 3.0), legend_at="right")
    s.save(os.path.join(OUT, "current-step.svg"))


def current_speed() -> None:
    head, c = read_csv("current_speed.csv")
    x = c[head.index("speed_rev_s")]
    s = Svg(960, 340, "Current amplitude against speed")
    series = [("two H-bridges", c[head.index("full_A")], BLUE, False),
              ("shared leg (B-G431B-ESC1)", c[head.index("shared_A")], ORANGE, False)]
    panel(s, 90, 40, 830, 240, x, series, "Speed [rev/s]", "Current amplitude [A]",
          "Current amplitude at constant speed, 24 V, 2 A commanded", ylim=(0, 2.2), marks=True)
    s.save(os.path.join(OUT, "current-speed.svg"))


def stall_trip() -> None:
    head, c = read_csv("stall_trip.csv")
    t = [v * 1e3 for v in c[head.index("t_s")]]
    s = Svg(960, 640, "Stall at the hold current: the detector level rises to the fault")
    panel(s, 90, 40, 830, 150, t, [("iA", c[head.index("iA_A")], TEAL, False), ("iB", c[head.index("iB_A")], ORANGE, False)],
          "Time [ms]", "Current [A]", "Phase currents")
    panel(s, 90, 270, 830, 110, t, [("load angle", c[head.index("angle_deg")], BLUE, False)],
          "Time [ms]", "Angle [deg]", "Load angle")
    panel(s, 90, 450, 830, 130, t, [("fault flags", c[head.index("faults")], RED, False),
                                    ("stall detector level [%]", c[head.index("level_pct")], GREEN, False)],
          "Time [ms]", "Flags, level", "Detector", ylim=(-5, 118))
    s.save(os.path.join(OUT, "stall-trip.svg"))


def resonance() -> None:
    head, c = read_csv("resonance.csv")
    s = Svg(960, 340, "Rotor oscillation against speed, detent torque 0.02 N*m")
    panel(s, 90, 40, 830, 240, c[head.index("speed_rev_s")],
          [("rotor oscillation", c[head.index("osc_deg")], BLUE, False)],
          "Speed [rev/s]", "Oscillation [electrical deg]", "Low-speed resonance", marks=True)
    s.save(os.path.join(OUT, "resonance.svg"))


# ==============================================================================
# HERO
# ==============================================================================

def _wave(iso: Iso, x0, x1, y, z, amp, cycles, n=48):
    return [iso.p(x0 + (x1 - x0) * k / n, y, z + amp * math.sin(2 * math.pi * cycles * k / n)) for k in range(n + 1)]


def hero() -> None:
    """A block diagram standing on a board: signal in, the real C library, the motor model, a
    scope. The loop between the controller and the motor is what the project is about."""
    iso = Iso(scale=25.0)
    p = iso.p
    iso.box(0, 24, 0, 10, -0.4, 0, "url(#boardTop)", "#0a1628", "#071122", "#1e3a66", 1.2)
    # grid of the diagram editor
    for k in range(1, 24, 2):
        iso.path([p(k, 0.2), p(k, 9.8)], "#16345f", 0.7, ' opacity="0.65"')
    for k in range(1, 10, 2):
        iso.path([p(0.2, k), p(23.8, k)], "#16345f", 0.7, ' opacity="0.65"')

    # loop wires under the blocks: forward and feedback, drawn first
    forward = [p(9.0, 6.4, 0.05), p(9.0, 7.3, 0.05), p(14.4, 7.3, 0.05), p(14.4, 6.6, 0.05)]
    back = [p(16.6, 6.6, 0.05), p(16.6, 8.7, 0.05), p(7.0, 8.7, 0.05), p(7.0, 6.4, 0.05)]
    signal_in = [p(4.6, 4.2, 0.05), p(5.8, 4.2, 0.05)]
    out = [p(17.8, 4.2, 0.05), p(19.2, 4.2, 0.05)]
    for wire, cls, color in ((forward, "flow", "#7fd0ff"), (back, "flow-bus", "#3fcf8e"),
                             (signal_in, "flow", "#7fd0ff"), (out, "flow", "#7fd0ff")):
        iso.path(wire, "#0e2a4f", 6)
        iso.path(wire, "#2f8cff", 2.0, ' opacity="0.9"')
        iso.path(wire, color, 2.0, f' class="{cls}"')

    # inputs block: step, sine and ramp on its top
    iso.box(1.2, 4.6, 2.2, 6.4, 0, 0.5, "#1b2a40", "#131f31", "#0e1726", "#2a4870")
    y0, x0 = 2.9, 1.6
    iso.path([p(x0, y0, .52), p(x0 + 1.1, y0, .52), p(x0 + 1.1, y0 - .0, .52), p(x0 + 1.1, y0 + .55, .52), p(x0 + 2.6, y0 + .55, .52)], "#5ea6ff", 1.8)
    iso.path([p(x0 + 2.6 * k / 30, 4.3 + .3 * math.sin(k / 30 * 4 * math.pi), .52) for k in range(31)], "#5ea6ff", 1.8)
    iso.path([p(x0, 5.9, .52), p(x0 + 2.6, 5.2, .52)], "#5ea6ff", 1.8)

    # controller chip
    for i in range(7):
        c = 2.7 + i * 0.45
        iso.box(5.55, 5.8, c - 0.08, c + 0.08, 0, 0.12, "#8fa6c6", "#6b819f", "#5a6f8b")
        iso.box(10.0, 10.25, c - 0.08, c + 0.08, 0, 0.12, "#8fa6c6", "#6b819f", "#5a6f8b")
    iso.box(5.8, 10.0, 2.4, 6.4, 0, 0.8, "url(#chipTop)", "#0a1322", "#060d18", "#2f8cff", 1.0)
    iso.text(7.9, 4.3, 0.81, "STEPPER-C", 0.56, "#e6edf7", 800, 0.03)
    iso.text(7.9, 5.0, 0.81, "REAL C CODE IN THE LOOP", 0.21, "#5ea6ff", 600, 0.05)
    iso.led(6.3, 2.9, 0.81)

    # motor model: a square frame with a shaft
    iso.box(13.2, 17.8, 2.0, 6.6, 0, 1.0, "#1f3047", "#16233a", "#101b2d", "#2a4870")
    iso.box(13.6, 17.4, 2.4, 6.2, 1.0, 2.6, "#2a3d58", "#1d2d44", "#16233a", "#3a5a85")
    iso.box(13.2, 17.8, 2.0, 6.6, 2.6, 3.2, "#2f4464", "#1f3047", "#16233a", "#2a4870")
    iso.disc(15.5, 4.3, 3.2, 1.25, [(1.0, "#8aa0bf"), (0.84, "#c9d6ea"), (0.55, "#8aa0bf"), (0.4, "#c9d6ea")])
    cx, cy = p(15.5, 4.3, 3.2)
    iso.parts.append(f'<g transform="translate({cx:.1f} {cy:.1f}) scale(1 0.577)"><line x1="0" y1="0" x2="19" y2="0" '
                     f'stroke="#0a1628" stroke-width="3.2" stroke-linecap="round">'
                     f'<animateTransform attributeName="transform" type="rotate" from="0" to="360" dur="4s" '
                     f'repeatCount="indefinite"/></line></g>')
    iso.text(15.5, 6.6, 0.32, "MOTOR MODEL", 0.32, "#9aabc4", 700, face="y")

    # scope
    iso.box(19.2, 23.0, 2.2, 6.4, 0, 0.5, "#1b2a40", "#131f31", "#0e1726", "#2a4870")
    iso.box(19.5, 22.7, 2.6, 6.0, 0.5, 0.56, "#071a33", "#071a33", "#071a33", "#2f8cff", 0.8)
    n = 44
    ia = [p(19.7 + 2.8 * k / n, 3.45 + .55 * math.sin(k / n * 6 * math.pi), .58) for k in range(n + 1)]
    ib = [p(19.7 + 2.8 * k / n, 5.0 + .55 * math.cos(k / n * 6 * math.pi), .58) for k in range(n + 1)]
    iso.path(ia, "#3fcf8e", 1.8)
    iso.path(ib, "#ffb454", 1.8)

    label = "A block diagram on a board: signal sources drive the Stepper-C chip, which drives a motor model in a loop, and a scope shows the two phase currents."
    _write(os.path.join(OUT, "hero-3d.svg"), iso.svg(label, HERO_DEFS, HERO_STYLE, glow_at=(12, 5)))


def main() -> None:
    architecture()
    lab_hierarchy()
    mil_sil()
    hero()
    for name, fig in (("current_step.csv", current_step), ("current_speed.csv", current_speed),
                      ("stall_trip.csv", stall_trip), ("resonance.csv", resonance)):
        if os.path.isfile(os.path.join(DATA, name)):
            fig()
        else:
            print(f"skipped: no {name}")


if __name__ == "__main__":
    main()
