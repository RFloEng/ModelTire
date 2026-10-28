# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 RFloEng
"""Generates the ModelTire single-wheel rig (vehicle "modelTire_wheelrig") into mod/modelTire/vehicles/.

One test wheel on a steerable carrier. The carrier hangs on two pivots on a vertical line through the wheel centre (a kingpin with
no scrub and no caster), so a couple on the carrier turns the wheel (the slip angle) and the couple needed to hold it is the aligning
torque, as on a tyre testing machine's steering torque cell. The frame around it carries no wheel of its own: its weight, the wheel
load and the balance are set by forces from the controller (lua/vehicle/extensions/modelTire/wheelrig.lua).

Usage:  python gen_wheelrig.py [--model model.json]      (model.json: nodes and beams without the tyre, for the offline simulation)
Settings can be overridden with environment variables (BALLAST, K_CARR, ...).
"""
import os, sys, json, itertools, re

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "mod", "modelTire", "vehicles", "modelTire_wheelrig") + os.sep
os.makedirs(OUT, exist_ok=True)


def env(name, default):
    return float(os.environ.get(name, default))


# x: left positive, y: forward negative, z up. The wheel centre is the origin of x and y.
HUB_Z = 0.315
HUB_X = 0.125                        # the hub nodes (the axle) at +-HUB_X
FW, FL = env("FW", 0.5), env("FL", 0.8)         # frame half width and half length
FB_Z, FT_Z = env("FB_Z", 0.04), env("FT_Z", 0.95)  # frame bottom (the force nodes, at ground height) and top (the upper pivot)
CY = env("CY", 0.45)                 # the carrier's front and rear nodes
CZ_TOP, CZ_BOT = env("CZ_TOP", 0.75), env("CZ_BOT", 0.12)
BALLAST0 = env("BALLAST", 100)
K_CARR = env("K_CARR", 8e6)          # carrier beams (and the beams to the pivots)
K_MOUNT = env("K_MOUNT", 2e6)        # pivots to the frame
K_FRAME = env("K_FRAME", 2e6)
DT = 0.0005
WDT = env("WDT", 0.85)               # target omega * dt of every node: heavier nodes where the beams are stiffer

pos, minm, group, order = {}, {}, {}, []


def node(nid, p, m, g):
    pos[nid] = p; minm[nid] = m; group[nid] = g; order.append(nid)


corner = {}
for sx, sn in ((-1, "r"), (1, "l")):
    for sy, yn in ((-1, "f"), (1, "b")):
        corner["sv_%s%s" % (sn, yn)] = (FW * sx, FL * sy, FB_Z)      # bottom: the force nodes, carry the ballast
        corner["ft_%s%s" % (sn, yn)] = (FW * sx, FL * sy, FT_Z)
HZ = 0.5 * (FB_Z + FT_Z)
helpers = {"ref": (0, 0, HZ), "mB": (0, FL, HZ), "mL": (FW, 0, HZ), "mU": (0, 0, FT_Z)}
for i, p in corner.items(): node(i, p, None if i.startswith("sv_") else 5, "frame")
for i, p in helpers.items(): node(i, p, 5, "frame")
node("kt", (0, 0, FT_Z), 5, "kingpin"); node("kb", (0, 0, FB_Z), 5, "kingpin")
node("Wi", (HUB_X, 0, HUB_Z), 20, "wheelhub_W"); node("Wo", (-HUB_X, 0, HUB_Z), 20, "wheelhub_W")
carrier_extra = {"c1": (0, -CY, CZ_TOP), "c2": (0, CY, CZ_TOP), "c3": (0, -CY, CZ_BOT), "c4": (0, CY, CZ_BOT)}
for i, p in carrier_extra.items(): node(i, p, 10, "carrier")

beams = []
def beam(a, b, k, c=300): beams.append((a, b, k, c))
ids = list(corner)
for a, b in itertools.combinations(ids, 2): beam(a, b, K_FRAME)
for h in helpers:
    for c in ids: beam(h, c, K_FRAME)
for i in ("mB", "mL", "mU"): beam("ref", i, K_FRAME)
for p in ("kt", "kb"):
    for c in ids: beam(p, c, K_MOUNT, 250)
carrier = ["Wi", "Wo"] + list(carrier_extra)
for a, b in itertools.combinations(carrier, 2): beam(a, b, K_CARR)
for c in carrier:
    for p in ("kt", "kb"): beam(c, p, K_CARR)

ksum = {n: 0.0 for n in pos}
for a, b, k, c in beams: ksum[a] += k; ksum[b] += k
mass = {}
for n in pos:
    mass[n] = None if minm[n] is None else max(minm[n], ksum[n] * DT * DT / (WDT * WDT))


def fmt(p): return "%.3f, %.3f, %.3f" % tuple(p)


L = ['        {"collision":false},', '        {"selfCollision":false},', '        {"frictionCoef":0.5},', '        {"nodeMaterial":"|NM_METAL"},']
lastg, lastm = None, "unset"
for n in order:
    if group[n] != lastg:
        L.append('        {"group":"%s"},' % group[n]); lastg = group[n]
    m = "$=$ballast/4" if mass[n] is None else round(mass[n], 1)
    if m != lastm:
        L.append('        {"nodeWeight":%s},' % (('"%s"' % m) if isinstance(m, str) else m)); lastm = m
    L.append('        ["%s", %s],' % (n, fmt(pos[n])))
L.append('        {"group":""},')

B = ['        {"beamPrecompression":1, "beamType":"|NORMAL", "beamLongBound":1.0, "beamShortBound":1.0},',
     '        {"beamDeform":350000, "beamStrength":"FLT_MAX"},']
cur = None
for a, b, k, c in beams:
    if (k, c) != cur:
        B.append('        {"beamSpring":%d, "beamDamp":%d},' % (k, c)); cur = (k, c)
    B.append('        ["%s", "%s"],' % (a, b))

tire = open(os.path.join(HERE, "tire_block.txt")).read()
tire = re.sub(r'\n\s*\["RR".*\n\s*\["RL".*\n', '\n        ["W", "wheelhub_W", "tire_W", "Wo", "Wi", 9999, "c1", 1],\n', tire)
assert tire.count('"wheelhub_W"') == 1

jb = '''{
"modelTire_wheelrig": {
    "information":{
        "authors":"ModelTire",
        "name":"ModelTire single-wheel rig"
    },
    "slotType" : "main",
    "general": {
        "enableTracking": true
    },
    "sounds": {
        "wind":false
    },
    "refNodes":[
        ["ref:", "back:", "left:", "up:", "leftCorner:", "rightCorner:"],
        ["ref", "mB", "mL", "mU", "ft_lf", "ft_rf"]
    ],
    "cameraChase":{
        "distance":5,
        "distanceMin":2.5,
        "defaultRotation":{"x":0,"y":-12,"z":0},
        "offset":{"x":0, "y":0, "z":0.5},
        "fov":60
    },
    "cameraExternal":{
        "distance":5,
        "distanceMin":2.5,
        "offset":{"x":0, "y":0, "z":0.5},
        "fov":60
    },
    "variables": [
        ["name", "type", "unit", "category", "default", "min", "max", "title", "description"],
        ["$ballast", "range", "kg", "Rig", %d, 0, 600, "Ballast", "Mass of the frame's lower corners (the wheel load is set by the controller, not by this)", {"stepDis":25}],
        ["$tirepressure", "range", "psi", "Wheels", 30, 15, 50, "Tire Pressure", "Relative to atmospheric pressure"]
    ],
    "nodes": [
        ["id", "posX", "posY", "posZ"],
%s
    ],
    "beams": [
        ["id1:", "id2:"],
%s
    ],
%s
}
}
''' % (BALLAST0, "\n".join(L), "\n".join(B), tire)
open(OUT + "modelTire_wheelrig.jbeam", "w", encoding="utf-8", newline="\n").write(jb)
open(OUT + "info.json", "w", newline="\n").write('''{
    "Name":"ModelTire single-wheel rig",
    "Author":"ModelTire",
    "Type":"Prop",
    "Description":"One test wheel on a steerable carrier with a load set by a controller, like a tyre testing machine (see the ModelTire sweep window, test 'rig slip ramp').",
    "visualTestSkip": true
}
''')
open(OUT + "modelTire_wheelrig.pc", "w", newline="\n").write('''{
  "format":2,
  "mainPartName":"modelTire_wheelrig",
  "model":"modelTire_wheelrig",
  "parts":{}
}
''')
open(OUT + "info_modelTire_wheelrig.json", "w", newline="\n").write('''{
"Configuration":"Single-wheel rig",
"Description":"One test wheel, steerable carrier, controlled load",
"Value":0,
"Weight":600
}
''')

mm = {n: (BALLAST0 / 4 if mass[n] is None else mass[n]) for n in pos}
tot = sum(mm.values())
cogz = sum(mm[n] * pos[n][2] for n in pos) / tot
print("ballast %.0f kg: total %.0f kg, centre of gravity height %.3f m; nodes %d, beams %d" % (BALLAST0, tot, cogz, len(pos), len(beams)))
print("masses: hub %.0f, carrier %.0f, kingpin %.0f, frame helper %.0f, top corner %.0f kg" % (mm["Wi"], mm["c1"], mm["kt"], mm["ref"], mm["ft_rf"]))
if "--model" in sys.argv:
    path = sys.argv[sys.argv.index("--model") + 1]
    json.dump({"nodes": [{"name": n, "p": pos[n], "m": mm[n]} for n in order], "beams": [{"a": a, "b": b, "k": k, "c": c} for a, b, k, c in beams]},
              open(path, "w"))
