# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 RFloEng
"""Generates the ModelTire test sled (vehicle "modelTire_rig") into mod/modelTire/vehicles/. Usage: python gen_sled.py"""
import os, itertools, re

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "mod", "modelTire", "vehicles", "modelTire_rig") + os.sep
os.makedirs(out, exist_ok=True)

def env(name, default): return float(os.environ.get(name, default))

# x: left positive, y: forward negative, z up. Four identical wheels, each on its own A-arm with a spring and a damper.
HUB_Z = 0.325
WHEEL_Y = {"F": -0.9, "R": 0.9}
PIVOT_Y = {"F": -env("PIV_Y", 0.1), "R": env("PIV_Y", 0.1)}   # the arms reach nearly to the middle of the sled: a long base
X_IN, X_OUT = 0.60, 0.85
X_PIV_IN = env("PIV_X", 0.10)       # inner pivot near the centre line: a wide A-arm
TOWER_Z = env("TOWER_Z", 0.65)
FB_Z, FT_Z = env("FB_Z", 0.10), env("FT_Z", 0.40)   # frame bottom (carries the ballast) just over the ground, light top
PIV_Z = env("PIV_Z", 0.15)          # the hinge line of the arms: low, next to the frame
BALLAST0 = env("BALLAST", 600)
APEX_H = env("APEX_H", 0.6)
K_ARM = env("K_ARM", 25e6)          # arm beams (the stiffest part)
K_MOUNT = env("K_MOUNT", 3e6)       # pivots and towers to the frame
K_FRAME = env("K_FRAME", 3e6)
K_AXLE = env("K_AXLE", 25e6)
DT = 0.0005
WDT = env("WDT", 0.85)              # target omega * dt for every node: heavier nodes where the beams are stiffer

corner = {}
for sx, sn in ((-1, "r"), (1, "l")):
    for sy, yn in ((-1, "f"), (1, "b")):
        for z, zn in ((FB_Z, "fb"), (FT_Z, "ft")):
            corner["%s_%s%s" % (zn, sn, yn)] = (0.5 * sx, 0.9 * sy, z)
HZ = 0.5 * (FB_Z + FT_Z)
helpers = {"ref": (0, 0, HZ), "mB": (0, 0.9, HZ), "mL": (0.5, 0, HZ), "mU": (0, 0, FT_Z)}
servo = {"sv_rf": (-0.5, -0.9, 0.02), "sv_lf": (0.5, -0.9, 0.02), "sv_rb": (-0.5, 0.9, 0.02), "sv_lb": (0.5, 0.9, 0.02)}

wheels = {}
for e in ("F", "R"):
    for s, sx in (("R", -1), ("L", 1)):
        w = e + s
        wheels[w] = {"sx": sx, "end": e, "side": s.lower(),
                     "i": (sx * X_IN, WHEEL_Y[e], HUB_Z), "o": (sx * X_OUT, WHEEL_Y[e], HUB_Z),
                     "pi": (sx * X_PIV_IN, PIVOT_Y[e], PIV_Z), "po": (sx * X_OUT, PIVOT_Y[e], PIV_Z),
                     "ti": (sx * X_IN, WHEEL_Y[e], TOWER_Z), "to": (sx * X_OUT, WHEEL_Y[e], TOWER_Z),
                     "a": (sx * 0.25 * (X_IN + X_OUT + X_PIV_IN + X_OUT), 0.5 * (WHEEL_Y[e] + PIVOT_Y[e]), 0.5 * (HUB_Z + PIV_Z) + APEX_H)}

# ---- collect the nodes and the beams (a, b, spring, damper) first, so every node's mass can follow from its stiffness
pos = {}       # node -> position
minm = {}      # node -> minimum mass
group = {}     # node -> group name
order = []
def node(nid, p, m, g):
    pos[nid] = p; minm[nid] = m; group[nid] = g; order.append(nid)

for i, p in corner.items(): node(i, p, None if i.startswith("fb_") else 5, "frame")   # None: the ballast variable
for i, p in helpers.items(): node(i, p, 5, "frame")
for i, p in servo.items(): node(i, p, 5, "servo")
for w, d in wheels.items():
    node(w + "pi", d["pi"], 5, "suspension_" + w); node(w + "po", d["po"], 5, "suspension_" + w)
    node(w + "ti", d["ti"], 5, "suspension_" + w); node(w + "to", d["to"], 5, "suspension_" + w)
    node(w + "a", d["a"], 5, "suspension_" + w)
    node(w + "i", d["i"], 15, "wheelhub_" + w); node(w + "o", d["o"], 15, "wheelhub_" + w)

beams = []     # (a, b, k, c)
def beam(a, b, k, c=300): beams.append((a, b, k, c))
ids = list(corner)
for a, b in itertools.combinations(ids, 2): beam(a, b, K_FRAME)
for h in helpers:
    for c in ids: beam(h, c, K_FRAME)
for i in ("mB", "mL", "mU"): beam("ref", i, K_FRAME)
for s, p in servo.items():
    side = "r" if p[0] < 0 else "l"; end = "f" if p[1] < 0 else "b"
    for c in ("fb_rf", "fb_lf", "fb_rb", "fb_lb", "ft_%s%s" % (side, end)): beam(s, c, K_FRAME)
for w, d in wheels.items():
    for nn in ("pi", "po", "ti", "to"):
        for c in ids: beam(w + nn, c, K_MOUNT, 250)
    beam(w + "pi", w + "po", K_ARM); beam(w + "ti", w + "to", K_MOUNT)
    for a, b in (("pi", "ti"), ("po", "to"), ("pi", "to"), ("po", "ti")): beam(w + a, w + b, K_MOUNT)
    # the arm: pivots, hubs and an apex: a rigid pyramid that can only turn about the line through the two pivots
    for a, b in (("i", "pi"), ("o", "po"), ("i", "po"), ("o", "pi")): beam(w + a, w + b, K_ARM)
    beam(w + "i", w + "o", K_AXLE, 150)
    for nn in ("i", "o", "pi", "po"): beam(w + "a", w + nn, K_ARM)
springs = []
for w in wheels:
    springs.append((w + "ti", w + "i")); springs.append((w + "to", w + "o"))

# mass from stiffness: omega * dt <= WDT with omega = sqrt(sum k / m); the springs are too soft to count
ksum = {n: 0.0 for n in pos}
for a, b, k, c in beams: ksum[a] += k; ksum[b] += k
mass = {}
for n in pos:
    if minm[n] is None: mass[n] = None; continue
    mass[n] = max(minm[n], ksum[n] * DT * DT / (WDT * WDT))

def fmt(p): return "%.3f, %.3f, %.3f" % p
L = ['        {"collision":false},', '        {"selfCollision":false},', '        {"frictionCoef":0.5},', '        {"nodeMaterial":"|NM_METAL"},']
lastg = None; lastm = "unset"
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
B.append('        // springs and dampers (two per wheel: 70 kN/m, about 0.7 of critical damping for 400 kg per wheel)')
B.append('        {"beamSpring":35000, "beamDamp":3600},')
for a, b in springs: B.append('        ["%s", "%s"],' % (a, b))

tire = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "tire_block.txt")).read()
rows = []
for w, d in wheels.items():
    arm = "fb_%s%s" % (d["side"], "f" if d["end"] == "F" else "b")
    rows.append('        ["%s", "wheelhub_%s", "tire_%s", "%so", "%si", 9999, "%s", %d],' % (w, w, w, w, w, arm, 1 if d["sx"] < 0 else -1))
tire = re.sub(r'\n\s*\["RR".*\n\s*\["RL".*\n', "\n" + "\n".join(rows) + "\n", tire)
assert tire.count("wheelhub_FR") == 1 and "wheelhub_RL" in tire

jb = '''{
"modelTire_rig": {
    "information":{
        "authors":"ModelTire",
        "name":"ModelTire test sled"
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
        "distance":7,
        "distanceMin":3,
        "defaultRotation":{"x":0,"y":-12,"z":0},
        "offset":{"x":0, "y":0, "z":0.4},
        "fov":60
    },
    "cameraExternal":{
        "distance":7,
        "distanceMin":3,
        "offset":{"x":0, "y":0, "z":0.4},
        "fov":60
    },
    "variables": [
        ["name", "type", "unit", "category", "default", "min", "max", "title", "description"],
        ["$ballast", "range", "kg", "Rig", 600, 0, 2400, "Ballast", "Sprung mass carried by the four wheels: sets their load", {"stepDis":100}],
        ["$tirepressure", "range", "psi", "Wheels", 28, 15, 50, "Tire Pressure", "Relative to atmospheric pressure"]
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
''' % ("\n".join(L), "\n".join(B), tire)
open(out + "modelTire_rig.jbeam", "w", encoding="utf-8", newline="\n").write(jb)

open(out + "info.json", "w", newline="\n").write('''{
    "Name":"ModelTire test sled",
    "Author":"ModelTire",
    "Type":"Prop",
    "Description":"A box on four identical wheels, each with its own spring and damper, for repeatable tyre force measurements (see the ModelTire sweep window, test 'rig slip ramp').",
    "visualTestSkip": true
}
''')
open(out + "modelTire_rig.pc", "w", newline="\n").write('''{
  "format":2,
  "mainPartName":"modelTire_rig",
  "model":"modelTire_rig",
  "parts":{}
}
''')
open(out + "info_modelTire_rig.json", "w", newline="\n").write('''{
"Configuration":"Test sled",
"Description":"Four test wheels on springs, ballast, no steering",
"Value":0,
"Weight":1700
}
''')
tot = sum(m for n, m in mass.items() if m is not None)
mm = {n: (BALLAST0 / 4 if mass[n] is None else mass[n]) for n in pos}
cogz = sum(mm[n] * pos[n][2] for n in pos) / sum(mm.values())
print("ballast %.0f kg: total %.0f kg, centre of gravity height %.3f m (hub height %.3f)" % (BALLAST0, sum(mm.values()), cogz, HUB_Z))
print("ok nodes %d beams %d; non-ballast node mass %.0f kg; hub %.0f kg, pivot %.0f kg, apex %.0f kg, frame helper %.0f kg" % (
    len(pos), len(beams) + len(springs), tot, mass["FRi"], mass["FRpi"], mass["FRa"], mass["ref"]))
