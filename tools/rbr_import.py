#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 RFloEng
# Uses RBR 1.02 engine constants as documented by rerbr (https://github.com/JodusNodus/rerbr, GPL-3.0): Copyright (C) the ReRBR authors.
"""Reads the tyre and surface data from YOUR OWN Richard Burns Rally install and writes it as a local data file
for the modelTire BeamNG mod. The output is for your private use: do not share it, it holds RBR's data.

    python rbr_import.py --rbr "D:\\RSF"
    python rbr_import.py --physics-rbz "C:\\path\\to\\physics.rbz"

What it reads (vanilla RBR 1.02, no NGP data):
  * Physics/michelin/tyres.lsp      the 7 tyre types (tarmac dry ... snow)
  * Physics/physics.lsp             the "surface types" table (friction, soil, wetness of each surface)
RSF and NGP installs replace physics.lsp with placeholders (NGP supplies the surfaces from NGP.dll). When that is
found, the tool takes the retail physics.rbz out of RSF's rbr_install.7z.001 with the install's own 7z.exe.

The output goes to an extra unpacked mod (modelTire_rbrdata) in your BeamNG user folder, so the main mod stays
free of RBR data. Python 3.8+, standard library only.
"""
import argparse
import datetime
import io
import os
import re
import subprocess
import sys
import tempfile
import zipfile

# RBR 1.02 engine constants: not in any data file, they are fixed in the game's code (the CTyre::CDef
# constructor). Values as decoded by rerbr (crates/rbr-physics/src/tyre/layout/construct.rs, GPL-3.0).
ENGINE_102 = {
    "pressureAxis": [0.0, 150000.0, 300000.0, 500000.0],
    "contactPressureAxis": [50000.0, 80000.0, 100000.0, 130000.0, 150000.0, 200000.0],
    "contactConductance": [0.5, 0.8, 0.93, 1.05, 1.1, 1.2],
    "temperatureAxis": [273.0, 323.0, 353.0, 383.0, 403.0, 418.0, 453.0, 523.0],
    "loadAxis": [0.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 9000.0, 20000.0],
    "surfaceCapacity": [0.02, 2000.0],
    "treadCapacity": [1.5, 2000.0],
    "carcassCapacity": [9.0, 2000.0],
    "treadConductivity": 1.0,
    "carcassConductivity": 2.0,
    "initTemperature": 338.15,
    "referenceFriction": 1.0,
}

# BeamNG ground model -> RBR surface. Edit freely in the output file.
SURFACE_MAP = {
    "ASPHALT": "Tarmac_Fine_Clean_Dry",
    "ASPHALT_PREPPED": "Tarmac_Fine_Clean_Dry",
    "ASPHALT_OLD": "Tarmac_Medium_Clean_Dry",
    "ASPHALT_WET": "Tarmac_Fine_Clean_Wet",
    "CONCRETE": "Tarmac_Medium_Clean_Dry",
    "COBBLESTONE": "Tarmac_Coarse_Clean_Dry",
    "RUMBLE_STRIP": "Tarmac_Coarse_Clean_Dry",
    "ROCK": "Rough_Rough_Dry",
    "GRAVEL": "Gravel_Medium_Deep_Dry",
    "DIRT": "Gravel_Fine_Shallow_Dry",
    "DIRT_DUSTY": "Gravel_Fine_Deep_Dry",
    "GRASS": "Grass_Medium_Dry",
    "MUD": "Dirt_Soft_Wet",
    "SAND": "Gravel_Coarse_Deeper_Dry",
    "SNOW": "SnowOnGravel_Medium",
    "ICE": "SnowOnIce_Shallow",
    "METAL": "Tarmac_Fine_Clean_Wet",
    "WOOD": "Tarmac_Fine_Clean_Wet",
    "PLASTIC": "Tarmac_Fine_Clean_Wet",
    "KICKPLATE": "Tarmac_Fine_Clean_Wet",
}

TYRE_KEYS = {
    "CarcassWidth": "carcassWidth", "CarcassRadius": "carcassRadius", "TreadDepth": "treadDepth",
    "TreadWidth": "treadWidth", "CarcassThickness": "carcassThickness", "TreadCoverage": "treadCoverage",
    "SideWallHeight": "sidewallHeight", "PressureRatio": "pressureRatio", "TreadElasticity": "treadElasticity",
    "CarcassStiffnessLateral": "carcassStiffnessLateral", "CarcassStiffnessTorsional": "carcassStiffnessTorsional",
    "CarcassStiffnessVertical": "carcassStiffnessVertical", "WetnessDesign": "wetnessDesign",
    "WetnessWearFtr": "wetnessWearFtr", "RimRadius": "rimRadius", "HubOffset": "hubOffset",
}
TYRE_TABLES = {  # lsp prefix -> (lua name, entries)
    "SFric_T": ("sFric", 6), "WearFtr_T": ("wearFtr", 6), "WetSlope_T": ("wetSlope", 6),
    "StfFct_P": ("stfFct_P", 4), "RubMyFct_T": ("rubMyFct_T", 8), "RubWear_T": ("rubWear_T", 8),
    "CrnStf_L": ("crnStf_L", 8), "TrctStf_L": ("trctStf_L", 8), "CrnMyFct_L": ("crnMyFct_L", 8),
    "TrctMyFct_L": ("trctMyFct_L", 8), "SlpPkCrn_L": ("slpPkCrn_L", 8), "SlpPkTrct_L": ("slpPkTrct_L", 8),
}
SURFACE_KEYS = {
    "myStatic": "static", "myKinetic": "kinetic", "myStaticSoil": "staticSoil", "myKineticSoil": "kineticSoil",
    "SinkFactor": "sinkFactor", "SoilThickness": "soilThickness", "SoilDensity": "soilDensity",
    "WaterFactor": "water", "RollingResistance": "rollingResistance", "Type": "type",
}


# --- .lsp reading ---------------------------------------------------------------------------------------------

def decode_lsp(raw):
    """RBR reads a buffer that does not start with '((' as XOR-encoded; plain files are the norm."""
    text = raw.decode("latin1")
    if not text.lstrip().startswith("(("):
        raise ValueError("encoded .lsp files are not supported")
    return text


def tokenize(text):
    text = re.sub(r";[^\n]*", "", text)
    return re.findall(r'"[^"]*"|\(|\)|[^\s()"]+', text)


def parse(tokens):
    """Nested lists of tokens; strings keep their quotes."""
    stack = [[]]
    for t in tokens:
        if t == "(":
            stack.append([])
        elif t == ")":
            if len(stack) > 1:
                done = stack.pop()
                stack[-1].append(done)
        else:
            stack[-1].append(t)
    while len(stack) > 1:
        done = stack.pop()
        stack[-1].append(done)
    return stack[0]


def as_number(t):
    try:
        return float(t)
    except (TypeError, ValueError):
        return None


def record(lst):
    """A group ("name" key value key value ... key (child)) -> (name, fields, children)."""
    name, fields, children = None, {}, {}
    i = 0
    if lst and isinstance(lst[0], str) and lst[0].startswith('"'):
        name = lst[0].strip('"')
        i = 1
    key = None
    while i < len(lst):
        t = lst[i]
        if isinstance(t, list):
            if key is not None:
                children[key] = t
                key = None
        elif as_number(t) is not None and key is not None:
            fields[key] = as_number(t)
        elif t.startswith(("+", "-")) and as_number(t) is None:
            pass  # flag
        else:
            key = t
        i += 1
    return name, fields, children


def find_group(tree, key, name=None):
    """Depth-first search for `key (...)` whose child group has the given name."""
    if not isinstance(tree, list):
        return None
    for idx, t in enumerate(tree):
        if isinstance(t, str) and t == key and idx + 1 < len(tree) and isinstance(tree[idx + 1], list):
            grp = tree[idx + 1]
            if name is None or (grp and isinstance(grp[0], str) and grp[0].strip('"') == name):
                return grp
        if isinstance(t, list):
            found = find_group(t, key, name)
            if found is not None:
                return found
    return None


def read_tyres(text):
    top = parse(tokenize(text))
    while len(top) == 1 and isinstance(top[0], list):
        top = top[0]
    _, _, children = record(top)
    tyres = []
    for key in sorted(k for k in children if k.startswith("TyreType")):
        name, fields, _ = record(children[key])
        t = {"name": (name or key).strip("()")}
        for lsp, lua in TYRE_KEYS.items():
            if lsp in fields:
                t[lua] = fields[lsp]
        for prefix, (lua, n) in TYRE_TABLES.items():
            vals = [fields.get(f"{prefix}{i}") for i in range(n)]
            if any(v is None for v in vals):
                raise ValueError(f"{key}: missing {prefix}0..{n - 1}")
            t[lua] = vals
        tyres.append(t)
    if len(tyres) != 7:
        raise ValueError(f"expected 7 tyre types, found {len(tyres)}")
    return tyres


def read_surfaces(text):
    tree = parse(tokenize(text))
    grp = find_group(tree, "MaterialManager", "surface types")
    if grp is None:
        raise ValueError('no MaterialManager ("surface types") in physics.lsp')
    _, _, children = record(grp)
    surfaces = {}
    for key, lst in children.items():
        name, fields, _ = record(lst)
        if "myStatic" not in fields:
            continue
        s = {lua: fields[lsp] for lsp, lua in SURFACE_KEYS.items() if lsp in fields}
        s.setdefault("kinetic", s["static"])
        s.setdefault("staticSoil", s["static"])
        s.setdefault("kineticSoil", s["kinetic"])
        for k in ("sinkFactor", "soilThickness", "soilDensity", "water", "rollingResistance", "type"):
            s.setdefault(k, 0.0)
        s["class"] = int(s.pop("type")) + 1  # Type 0..5 picks the tyre's SFric_T0..T5
        surfaces[name or key] = s
    return surfaces


def surfaces_are_placeholders(surfaces):
    t = surfaces.get("Tarmac_Fine_Clean_Dry")
    return t is None or abs(t["static"] - 0.3) < 1e-6


# --- finding the files ------------------------------------------------------------------------------------------

def open_rbz(path):
    with open(path, "rb") as f:
        return zipfile.ZipFile(io.BytesIO(f.read()))


def member(zf, name):
    for info in zf.infolist():
        if info.filename.replace("\\", "/").lower() == name.lower():
            return zf.read(info)
    return None


def retail_rbz_from_rsf(rbr_dir, workdir):
    archive = os.path.join(rbr_dir, "rsf_installer_files", "rbr_install.7z.001")
    seven = os.path.join(rbr_dir, "7z.exe")
    if not (os.path.isfile(archive) and os.path.isfile(seven)):
        return None
    print("  physics.lsp has placeholders (RSF/NGP); taking the retail physics.rbz from rbr_install.7z.001 ...")
    subprocess.run([seven, "e", archive, f"-o{workdir}", "physics.rbz", "-r", "-y"], check=True,
                   stdout=subprocess.DEVNULL)
    out = os.path.join(workdir, "physics.rbz")
    return out if os.path.isfile(out) else None


# --- output -----------------------------------------------------------------------------------------------------

def lua_value(v, indent=""):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return f"{v:.9g}"
    if isinstance(v, str):
        return '"' + v.replace("\\", "\\\\").replace('"', '\\"') + '"'
    if isinstance(v, list):
        return "{" + ", ".join(lua_value(x) for x in v) + "}"
    if isinstance(v, dict):
        inner = indent + "  "
        items = []
        for k in sorted(v):
            key = k if re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", k) else f'["{k}"]'
            items.append(f"{inner}{key} = {lua_value(v[k], inner)}")
        return "{\n" + ",\n".join(items) + "\n" + indent + "}"
    raise TypeError(type(v))


def default_out():
    base = os.environ.get("LOCALAPPDATA", "")
    return os.path.join(base, "BeamNG", "BeamNG.drive", "current", "mods", "unpacked", "modelTire_rbrdata")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rbr", help="RBR or RSF install folder (has physics.rbz)")
    ap.add_argument("--physics-rbz", help="a retail physics.rbz to read instead")
    ap.add_argument("--out", default=default_out(), help="output mod folder (default: BeamNG unpacked mods)")
    a = ap.parse_args()
    if not a.rbr and not a.physics_rbz:
        ap.error("give --rbr or --physics-rbz")

    rbz_path = a.physics_rbz or os.path.join(a.rbr, "physics.rbz")
    print(f"reading {rbz_path}")
    zf = open_rbz(rbz_path)
    tyres_raw = member(zf, "Physics/michelin/tyres.lsp")
    physics_raw = member(zf, "Physics/physics.lsp")
    if tyres_raw is None or physics_raw is None:
        sys.exit("physics.rbz has no Physics/michelin/tyres.lsp or Physics/physics.lsp")
    tyres = read_tyres(decode_lsp(tyres_raw))
    surfaces = read_surfaces(decode_lsp(physics_raw))
    source = rbz_path

    if surfaces_are_placeholders(surfaces):
        with tempfile.TemporaryDirectory() as work:
            retail = retail_rbz_from_rsf(a.rbr, work) if a.rbr else None
            if not retail:
                sys.exit("physics.lsp only has placeholder surfaces (an NGP/RSF install). Point --physics-rbz at the "
                         "physics.rbz of a plain RBR 1.02 install.")
            zf2 = open_rbz(retail)
            surfaces = read_surfaces(decode_lsp(member(zf2, "Physics/physics.lsp")))
            source += " + retail physics.lsp from rbr_install.7z.001"
            if surfaces_are_placeholders(surfaces):
                sys.exit("the retail physics.lsp has no surface values either")

    missing = sorted({v for v in SURFACE_MAP.values() if v not in surfaces})
    if missing:
        print("  warning: surfaces in the BeamNG map not found: " + ", ".join(missing))

    data = {
        "source": source,
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "engine": ENGINE_102,
        "tyres": tyres,
        "surfaces": surfaces,
        "surfaceMap": SURFACE_MAP,
    }
    lua_dir = os.path.join(a.out, "lua", "vehicle")
    os.makedirs(lua_dir, exist_ok=True)
    out_file = os.path.join(lua_dir, "modelTire_rbrdata.lua")
    with open(out_file, "w", encoding="utf-8", newline="\n") as f:
        f.write("-- Generated by rbr_import.py from your own RBR install. PRIVATE: contains RBR's data, do not share.\n")
        f.write("return " + lua_value(data) + "\n")
    with open(os.path.join(a.out, "README.txt"), "w", encoding="utf-8") as f:
        f.write("RBR tyre and surface data for the modelTire mod, made by rbr_import.py from your own RBR install.\n"
                "It contains Richard Burns Rally's data: keep it private, do not upload or share it.\n")
    print(f"wrote {out_file}")
    print(f"  {len(tyres)} tyre types: " + ", ".join(t["name"] for t in tyres))
    print(f"  {len(surfaces)} surfaces")


if __name__ == "__main__":
    main()
