#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 RFloEng
"""Packs the mod (mod/modelTire) into modelTire.zip, ready to drop into BeamNG's mods folder
(%LOCALAPPDATA%/BeamNG/BeamNG.drive/current/mods) or to attach to a release. Contains no RBR or tyre data."""
import os, zipfile
root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mod", "modelTire")
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "modelTire.zip")
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for d, _, files in os.walk(root):
        for f in files:
            p = os.path.join(d, f)
            z.write(p, os.path.relpath(p, root))
    z.write(os.path.join(root, "..", "..", "LICENSE"), "LICENSE")
print("wrote", os.path.abspath(out))
