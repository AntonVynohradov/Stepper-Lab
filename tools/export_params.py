#!/usr/bin/env python3
################################################################################
# sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
#
# Description: Exports the motor parameters (params/motor_*.sci, the source of
#              truth) to params/motor_*.json for C users of the model
#              (sil/c/sstep_plant.c): the firmware simulation of
#              sfs-stepper-driver reads them at build time. The stage parameters
#              (params/stage_esc1_spice.json) come from sfs-stepper-power-spice.
#
# Author:      Anton Vynohradov
# Email:       avynohradov@systemfromscratch.com
# License:     Apache License 2.0
# Copyright:   (c) 2026 Anton Vynohradov
#
# Usage:       python tools/export_params.py           write params/motor_*.json
#              python tools/export_params.py --check   exit 1 if a JSON is stale
#
# SPDX-License-Identifier: Apache-2.0
#
################################################################################

import argparse
import glob
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Format of params/motor_*.json: raise it when a key changes meaning or
# disappears; adding a key keeps it.
FORMAT = 1

# Keys of the motor structure, SI units (sil/c/sstep_plant.h, struct sstep_plant_motor)
KEYS = {
    "R": "Ohm",
    "L": "H",
    "I_rated": "A",
    "Kt": "N*m/A",
    "Ns": "full steps per revolution",
    "Td": "N*m",
    "J": "kg*m^2",
    "B": "N*m*s/rad",
    "Tc": "N*m",
    "w_c": "rad/s",
}

ASSIGN = re.compile(r'^\s*mot\.(\w+)\s*=\s*("([^"]*)"|[-+0-9.eE]+)\s*;')


def parse(path):
    """Values assigned as `mot.<key> = <number or "text">;` in a motor .sci file."""
    values = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            m = ASSIGN.match(line)
            if m:
                values[m.group(1)] = m.group(3) if m.group(3) is not None else float(m.group(2))
    missing = [k for k in KEYS if k not in values]
    if missing:
        raise ValueError(f"{path}: missing {', '.join(missing)}")
    return values


def render(path):
    values = parse(path)
    doc = {
        "format": FORMAT,
        "source": f"sfs-stepper-sim params/{os.path.basename(path)} (tools/export_params.py)",
        "name": values.get("name", os.path.splitext(os.path.basename(path))[0]),
        "units": KEYS,
    }
    doc.update({k: values[k] for k in KEYS})
    return json.dumps(doc, indent=2) + "\n"


def main():
    ap = argparse.ArgumentParser(description="export params/motor_*.sci to JSON")
    ap.add_argument("--check", action="store_true", help="only check that the JSON files are current")
    args = ap.parse_args()
    stale = 0
    for sci in sorted(glob.glob(os.path.join(ROOT, "params", "motor_*.sci"))):
        out = os.path.splitext(sci)[0] + ".json"
        text = render(sci)
        current = open(out, encoding="utf-8").read() if os.path.exists(out) else None
        rel = os.path.relpath(out, ROOT)
        if args.check:
            if current != text:
                print(f"STALE {rel}: run python tools/export_params.py")
                stale += 1
            else:
                print(f"ok    {rel}")
        elif current != text:
            with open(out, "w", encoding="utf-8", newline="\n") as f:
                f.write(text)
            print(f"wrote {rel}")
        else:
            print(f"same  {rel}")
    return 1 if stale else 0


if __name__ == "__main__":
    sys.exit(main())
