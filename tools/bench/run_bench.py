#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 RFloEng
"""ModelTire offline bench: sweeps one wheel on a flat ground through every tyre mode, without the game.

    python tools/bench/run_bench.py [--svj tires.svj.json --key <tyre set key>] [--loads 2000,3500,5000] [--load 3500]
                                    [--speed 20] [--modes brush,rbr_light,rbr_full,svj_light,svj_full]
                                    [--ingame run1.csv run2.csv ...] [--out tools/bench/out] [--param name=value ...]

Needs `pip install lupa matplotlib`. Sweeps slip angle (-20..20 deg) and slip ratio (-0.5..0.5) at every load of --loads
and the load itself (1..7 kN at 6 deg). Writes bench.csv and PNG plots:
  bench_lat / bench_lon / bench_load   absolute forces of every mode (one line style per load)
  compare_lat / compare_lon            force / load, one panel per load: every bench mode, the theoretical Pacejka curve of
                                       the SVJ tyre at that load, and the in-game sweep runs (--ingame, CSV files saved
                                       by the in-game sweep tests, e.g. the BeamNG vanilla run) in the nominal-load panel
and prints peak force, slip at the peak and slope per mode and load. Without --svj the two Pacejka modes are skipped.
The bench measures what the wheel code puts on the car; BeamNG's own physics (suspension, soft tyre, collisions) are not in
it, and BeamNG's own tyre cannot run offline: it comes from an in-game run.
"""
import argparse
import csv
import json
import math
import os
import sys

import lupa.luajit21 as lj

HERE = os.path.dirname(os.path.abspath(__file__))
MOD = os.path.normpath(os.path.join(HERE, "..", "..", "mod", "modelTire")).replace("\\", "/")

THEORY_LUA = """function(modDir, T, ref, load, xs, lateral)
    local pac = dofile(modDir .. '/lua/vehicle/extensions/modelTire/pacejka.lua')
    pac.prepare(T)
    local fz0 = T.noFz0 and ref or T.Fz0
    local o, ys = {}, {}
    for i = 1, #xs do
      local kappa, alpha = 0, 0
      if lateral then alpha = xs[i] else kappa = xs[i] end
      pac._evaluate(T, fz0, kappa, alpha, 0, load, 0, 20, 1, 1, o)
      local f = lateral and o.Fy or o.Fx
      if lateral and T.flipLat then f = -f end
      ys[i] = math.abs(f) / load
    end
    return ys
end"""


def parse_value(s):
    for cast in (int, float):
        try:
            return cast(s)
        except ValueError:
            pass
    return {"true": True, "false": False}.get(s.lower(), s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--svj", help="an SVJ file with tyre sets")
    ap.add_argument("--key", help="tyre set key in that file (default: the first one)")
    ap.add_argument("--load", type=float, default=3500.0, help="nominal load: the Pacejka reference load and the load of the load sweep")
    ap.add_argument("--loads", default="2000,3500,5000", help="loads (N) of the slip sweeps")
    ap.add_argument("--speed", type=float, default=20.0)
    ap.add_argument("--radius", type=float, default=0.31)
    ap.add_argument("--material", default="ASPHALT", help="BeamNG ground material name for the light modes")
    ap.add_argument("--steps", type=int, default=1500)
    ap.add_argument("--modes", default="brush,rbr_light,rbr_full,svj_light,svj_full")
    ap.add_argument("--param", action="append", default=[], help="tyre.lua parameter, e.g. rbrGripScale=1.0")
    ap.add_argument("--ingame", nargs="*", default=[], help="CSV files saved by the in-game sweep tests (modelTire/sweeps/*.csv)")
    ap.add_argument("--debug", action="store_true", help="also write the load calibration rows")
    ap.add_argument("--out", default=os.path.join(HERE, "out"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    loads = [float(x) for x in a.loads.split(",")]

    L = lj.LuaRuntime()
    L.execute('package.path = "%s/lua/ge/extensions/?.lua;" .. package.path' % MOD)
    tyre = None
    if a.svj:
        doc = json.load(open(a.svj, encoding="utf-8"))
        sets = doc["tires"]["sets"]
        key = a.key or next(iter(sets))
        svj = L.eval('dofile("%s/lua/ge/extensions/modelTire/svj.lua")' % MOD)
        entry = svj.entriesOfDocument(L.table_from({"tires": {"sets": {key: sets[key]}}}, recursive=True), os.path.basename(a.svj))[1]
        tyre = svj.buildTyre(entry)
        print("Pacejka tyre:", key)
    params = L.table_from({p.split("=")[0]: parse_value(p.split("=", 1)[1]) for p in a.param})
    opt = L.table_from({"load": a.load, "loads": L.table_from(loads), "speed": a.speed, "radius": a.radius, "material": a.material,
                        "steps": a.steps, "modes": L.table_from(a.modes.split(",")), "params": params, "debug": a.debug})
    run = L.eval("function(path, a, b, c) local f = assert(loadfile(path)); return f(a, b, c) end")
    text = run(os.path.join(HERE, "bench.lua").replace("\\", "/"), MOD, tyre, opt)
    rows = [r.split(",") for r in text.split("\n") if r]
    with open(os.path.join(a.out, "bench.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["mode", "sweep", "x", "Fx", "Fy", "Fz"])
        w.writerows(rows)

    # the SVJ tyre IS the Pacejka model (SVJ is the file format): label it by the model
    names = {"svj_light": "pacejka_light", "svj_full": "pacejka_full"}
    rows = [[names.get(r[0], r[0])] + r[1:] for r in rows]
    # data[kind][load][mode] = [(x, Fx, Fy, Fz)], kind lat / lon / load
    data = {"lat": {}, "lon": {}, "load": {}}
    for mode, sweep, x, fx, fy, fz in rows:
        if "@" in sweep:
            kind, ld = sweep.split("@")
            ld = float(ld)
        elif sweep == "load":
            kind, ld = "load", a.load
        else:
            continue
        data[kind].setdefault(ld, {}).setdefault(mode, []).append((float(x), float(fx), float(fy), float(fz)))

    # summary
    for ld in sorted(data["lat"]):
        print("\nlateral, load %.0f N (slip angle sweep)" % ld)
        for mode, pts in data["lat"][ld].items():
            pos = [(x, -fy) for x, fx, fy, fz in pts if x > 0]
            peak = max(pos, key=lambda p: p[1])
            slope = [(-fy) / x for x, fx, fy, fz in pts if abs(x - 1) < 1e-6][0]
            fzm = sum(fz for *_, fz in pts) / len(pts)
            print("  %-10s peak %6.0f N at %4.1f deg (%.2f of the load), slope at 1 deg %5.0f N/deg" % (
                mode, peak[1], peak[0], peak[1] / fzm, slope))
    for ld in sorted(data["lon"]):
        print("longitudinal, load %.0f N (slip ratio sweep)" % ld)
        for mode, pts in data["lon"][ld].items():
            pos = [(x, fx) for x, fx, fy, fz in pts if x > 0]
            peak = max(pos, key=lambda p: p[1])
            print("  %-10s peak %6.0f N at slip %.2f" % (mode, peak[1], peak[0]))

    # pure theoretical Pacejka curve of the same tyre at every load (reference surface, no transient, no combined slip)
    theory = {"lat": {}, "lon": {}}
    if tyre is not None:
        ev = L.eval(THEORY_LUA)
        xl = [math.radians(d * 0.25) for d in range(0, 81)]
        xk = [k * 0.005 for k in range(0, 101)]
        for ld in loads:
            yl = ev(MOD, tyre, a.load, ld, L.table_from(xl), True)
            yk = ev(MOD, tyre, a.load, ld, L.table_from(xk), False)
            theory["lat"][ld] = ([math.degrees(x) for x in xl], [yl[i + 1] for i in range(len(xl))])
            theory["lon"][ld] = (xk, [yk[i + 1] for i in range(len(xk))])

    # in-game runs: acceleration in g against the mean slip of the wheels
    ingame = []
    for path in a.ingame:
        rowsg = {}
        with open(path, newline="") as f:
            for r in csv.DictReader(f):
                rowsg.setdefault(r["t"], []).append(r)
        lat, lon = [], []
        for tt, rs in rowsg.items():
            if float(rs[0]["speed"]) < 2:
                continue
            al = sum(abs(float(r["slip_angle_rad"])) for r in rs) / len(rs)
            ka = sum(abs(float(r["slip_ratio"])) for r in rs) / len(rs)
            lat.append((math.degrees(al), abs(float(rs[0]["ay_ms2"])) / 9.81))
            lon.append((ka, abs(float(rs[0]["ax_ms2"])) / 9.81))
        name = os.path.basename(path)[:-4]
        ingame.append((name, "steering" in name, lat, lon))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    modes_present = []
    for kind in data.values():
        for ld in kind.values():
            for m in ld:
                if m not in modes_present:
                    modes_present.append(m)
    colors = {m: "C%d" % i for i, m in enumerate(modes_present)}
    styles = ["-", "--", ":", "-."]

    # absolute forces, one line style per load
    for kind, title, xl_, col, sgn in (("lat", "lateral force (N)", "slip angle (deg)", 2, -1),
                                       ("lon", "longitudinal force (N)", "slip ratio", 1, 1)):
        if not data[kind]:
            continue
        plt.figure(figsize=(8, 5))
        for li, ld in enumerate(sorted(data[kind])):
            for mode, pts in data[kind][ld].items():
                plt.plot([p[0] for p in pts], [sgn * p[col] for p in pts], styles[li % 4], color=colors[mode], ms=3,
                         label="%s @ %.0f N" % (mode, ld))
        plt.xlabel(xl_)
        plt.ylabel(title)
        plt.grid(alpha=0.3)
        plt.legend(fontsize=6, ncol=2)
        plt.title("ModelTire bench: " + kind)
        plt.tight_layout()
        plt.savefig(os.path.join(a.out, "bench_%s.png" % kind), dpi=110)
        plt.close()
    if data["load"]:
        plt.figure(figsize=(7, 4.5))
        for mode, pts in next(iter(data["load"].values())).items():
            plt.plot([p[3] for p in pts], [-p[2] for p in pts], marker="o", ms=3, color=colors[mode], label=mode)
        plt.xlabel("vertical load (N)")
        plt.ylabel("lateral force at 6 deg (N)")
        plt.grid(alpha=0.3)
        plt.legend()
        plt.title("ModelTire bench: load")
        plt.tight_layout()
        plt.savefig(os.path.join(a.out, "bench_load.png"), dpi=110)
        plt.close()

    # comparison: one panel per load, force / load
    for kind, xl_ in (("lat", "slip angle (deg)"), ("lon", "slip ratio")):
        lds = sorted(data[kind])
        if not lds:
            continue
        fig, axes = plt.subplots(1, len(lds), figsize=(5.2 * len(lds), 4.8), sharey=True, squeeze=False)
        nominal = min(lds, key=lambda v: abs(v - a.load))
        for ax, ld in zip(axes[0], lds):
            for mode, pts in data[kind][ld].items():
                sel = [p for p in pts if p[0] >= 0]
                ax.plot([p[0] for p in sel], [abs(p[2] if kind == "lat" else p[1]) / max(p[3], 1) for p in sel],
                        marker="o", ms=3, color=colors[mode], label="bench: " + mode)
            if ld in theory[kind]:
                ax.plot(theory[kind][ld][0], theory[kind][ld][1], "k--", lw=1.6, label="theory: Pacejka pure slip")
            if ld == nominal:
                for name, is_steer, lat, lon in ingame:
                    pts = lat if kind == "lat" else lon
                    if pts and ((kind == "lat") == is_steer):
                        pts = sorted(pts)
                        ax.plot([p[0] for p in pts], [p[1] for p in pts], ".-", lw=1, ms=4, alpha=0.8, color="gray",
                                label="in game: " + name)
            ax.set_title("load %.0f N" % ld)
            ax.set_xlabel(xl_)
            ax.grid(alpha=0.3)
        axes[0][0].set_ylabel("force / load = acceleration (g)")
        axes[0][-1].legend(fontsize=6)
        fig.suptitle("ModelTire comparison: " + ("lateral" if kind == "lat" else "longitudinal"))
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, "compare_%s.png" % kind), dpi=115)
        plt.close(fig)
    print("\nwrote", os.path.join(a.out, "bench.csv"), ", bench_*.png and compare_*.png")


if __name__ == "__main__":
    sys.exit(main())
