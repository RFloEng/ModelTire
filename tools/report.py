#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 RFloEng
"""ModelTire: report of an automatic test.

    python tools/report.py <batch folder> [<more batch folders>] [--out report_dir] [--title "text"]

A batch folder is what the in-game "Automatic test + report" writes: modelTire/sweeps/batch_<date>/ in the BeamNG user folder
(.../BeamNG.drive/current/), one CSV per run plus manifest.json. Several folders are merged (run the test on other patches of the
map, or with another tyre, and pass all the folders). Needs matplotlib (pip install matplotlib).

Writes report.html (open it in a browser), summary.csv and img/*.png into the output folder (default: report_<first folder name>
next to the first folder).

What is measured: the test sled is moved at a constant speed while the direction of its travel turns away from its heading, so the
tyres see a slip angle that rises from 0 to the chosen maximum. The sled's controller measures the total sideways force (it has to
supply the opposite), so every tyre, BeamNG's own included, is measured the same way. Plotted: sideways force over load (g of lateral
acceleration the tyres give) against the mean slip angle of the wheels.
"""
import argparse, base64, csv, glob, html, io, json, math, os, sys

FAMILY_ORDER = ["vanilla", "brush", "adv_brush", "rbr_light", "svj_light", "rbr_full", "svj_full"]
FAMILY_NAMES = {
    "vanilla": "BeamNG vanilla tyre", "brush": "Brush test tyre", "adv_brush": "Advanced brush", "rbr_light": "RBR 1.02 on BeamNG's tyre",
    "svj_light": "SVJ Pacejka on BeamNG's tyre", "rbr_full": "FULL RBR tyre", "svj_full": "FULL SVJ Pacejka tyre",
}
FAMILY_COLORS = {
    "vanilla": "#2b7bd6", "brush": "#e07b1f", "adv_brush": "#8a5a2b", "rbr_light": "#3aa04a", "svj_light": "#d8479a", "rbr_full": "#b59a14", "svj_full": "#7d56d6",
}
FORCED = {"rbr_full", "svj_full"}      # families that can be given a surface; the others run on the ground under the sled


def moving_average(v, n):
    if n <= 1 or len(v) < 2:
        return list(v)
    h = n // 2
    out = []
    for i in range(len(v)):
        a, b = max(0, i - h), min(len(v), i + h + 1)
        out.append(sum(v[a:b]) / (b - a))
    return out


def read_run(path):
    """-> dict with slip (deg), g, and per-wheel load statistics; None when the file has no rig data."""
    rows = {}
    kap = {}
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        cols = rd.fieldnames or []
        if "rig_Fy" not in cols or "rig_load" not in cols:
            return None
        for r in rd:
            t = r["t"]
            e = rows.setdefault(t, {"fy": float(r["rig_Fy"]), "fx": float(r.get("rig_Fx", 0) or 0), "load": float(r["rig_load"]),
                                    "v": float(r["speed"]), "alpha": [], "fz": []})
            e["alpha"].append(abs(float(r["slip_angle_rad"])))
            e["fz"].append(float(r["Fz"]))
            kap.setdefault(r["wheel"], []).append(float(r["slip_ratio"]))
    ts = sorted(rows, key=float)
    slip, g, fx_g, fz = [], [], [], []
    for t in ts:
        e = rows[t]
        if e["load"] <= 100 or e["v"] < 2 or not e["alpha"]:
            continue
        slip.append(math.degrees(sum(e["alpha"]) / len(e["alpha"])))
        g.append(abs(e["fy"]) / e["load"])
        fx_g.append(e["fx"] / e["load"])
        fz.extend(e["fz"])
    if len(slip) < 10:
        return None
    sds = []
    for v in kap.values():
        m = sum(v) / len(v)
        sds.append(math.sqrt(sum((x - m) ** 2 for x in v) / len(v)))
    return {"slip": slip, "g": g, "fx_g": fx_g, "fz": fz, "kappa_sd": sum(sds) / len(sds) if sds else float("nan")}


def metrics(run):
    slip, g = run["slip"], run["g"]
    sm = moving_average(g, 7)
    n = len(sm)
    ip = max(range(n), key=lambda i: sm[i])
    peak, slip_peak = sm[ip], slip[ip]
    # cornering stiffness: g per degree, least squares through the origin over the first two degrees
    pts = [(s, v) for s, v in zip(slip, sm) if s <= 2.0]
    stiff = (sum(s * v for s, v in pts) / sum(s * s for s, v in pts)) if pts and sum(s * s for s, v in pts) > 0 else float("nan")

    def at(x):
        # value of the smoothed curve at slip x (nearest sample), nan when the run did not get that far
        if slip[-1] < x:
            return float("nan")
        i = min(range(n), key=lambda k: abs(slip[k] - x))
        return sm[i]
    tail = sm[max(0, int(n * 0.9)):]
    g_end = sum(tail) / len(tail)
    resid = [a - b for a, b in zip(g, moving_average(g, 7))]
    noise = math.sqrt(sum(r * r for r in resid) / len(resid)) / peak if peak > 0 else float("nan")
    fz = run["fz"]
    fzm = sum(fz) / len(fz) if fz else 0
    fz_cv = (math.sqrt(sum((x - fzm) ** 2 for x in fz) / len(fz)) / fzm) if fzm > 100 else float("nan")
    return {"peak_g": peak, "slip_peak": slip_peak, "stiffness": stiff, "g5": at(5), "g10": at(10), "g_end": g_end,
            "retention": g_end / peak if peak > 0 else float("nan"), "noise": noise, "fz_cv": fz_cv, "max_slip": slip[-1],
            "jitter": run.get("kappa_sd", float("nan"))}


def load_folders(folders):
    runs = []
    notes = []
    manifests = []
    for folder in folders:
        mpath = os.path.join(folder, "manifest.json")
        jobs = []
        if os.path.exists(mpath):
            with open(mpath) as f:
                man = json.load(f)
            manifests.append((folder, man))
            jobs = man.get("jobs", [])
            if isinstance(jobs, dict):
                jobs = [jobs[k] for k in sorted(jobs, key=lambda x: int(x))]
        else:
            notes.append("%s has no manifest.json: the runs are taken from the file names" % folder)
            for p in sorted(glob.glob(os.path.join(folder, "*.csv"))):
                tag = os.path.splitext(os.path.basename(p))[0]
                parts = tag.split("_", 2)
                fam = next((k for k in FAMILY_ORDER if tag[3:].startswith(k)), None)
                jobs.append({"idx": parts[0], "family": fam or "unknown", "tag": tag, "status": "ok", "surface": None})
        for j in jobs:
            rec = {"folder": folder, "job": j, "family": j.get("family") or "unknown", "surface": j.get("surface"),
                   "detected": j.get("detectedSurface"), "status": j.get("status", "?"), "reason": j.get("reason"), "tag": j.get("tag")}
            if rec["status"] == "ok" and rec["tag"]:
                p = os.path.join(folder, rec["tag"] + ".csv")
                if os.path.exists(p):
                    data = read_run(p)
                    if data is None:
                        rec["status"], rec["reason"] = "failed", "no rig data in the file (an old file, or too few samples)"
                    else:
                        rec["data"], rec["m"] = data, metrics(data)
                else:
                    rec["status"], rec["reason"] = "failed", "file missing: " + p
            runs.append(rec)
    return runs, notes, manifests


def label_of(r):
    """The column of the report a run belongs to: the forced surface, or the ground it ran on."""
    if r["family"] in FORCED:
        return r["surface"] or "?"
    return "ground: %s" % (r["detected"] or "unknown")


def main():
    ap = argparse.ArgumentParser(description="ModelTire automatic test report")
    ap.add_argument("folders", nargs="+", help="batch folder(s) written by the in-game automatic test")
    ap.add_argument("--out", help="output folder (default: report_<name of the first folder> beside it)")
    ap.add_argument("--title", default="ModelTire tyre model test")
    a = ap.parse_args()
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib is needed: pip install matplotlib")

    folders = [os.path.abspath(f) for f in a.folders]
    out = os.path.abspath(a.out) if a.out else os.path.join(os.path.dirname(folders[0]), "report_" + os.path.basename(folders[0]))
    os.makedirs(os.path.join(out, "img"), exist_ok=True)
    runs, notes, manifests = load_folders(folders)
    good = [r for r in runs if r["status"] == "ok" and "m" in r]
    if not good:
        sys.exit("no usable run in %s" % ", ".join(folders))

    images = []

    def save(fig, name, caption):
        p = os.path.join(out, "img", name + ".png")
        fig.savefig(p, dpi=110, bbox_inches="tight")
        plt.close(fig)
        images.append((name, caption))

    def curve(r):
        return r["data"]["slip"], moving_average(r["data"]["g"], 5)

    # reference runs: the families that cannot be given a surface
    refs = [r for r in good if r["family"] not in FORCED]
    forced = {f: [r for r in good if r["family"] == f] for f in FORCED}
    xmax = max(max(r["data"]["slip"]) for r in good)
    ymax = max(r["m"]["peak_g"] for r in good) * 1.15

    # 1. the reference families on the ground under the sled
    if refs:
        fig, ax = plt.subplots(figsize=(8, 4.6))
        for r in refs:
            s, y = curve(r)
            ax.plot(s, y, color=FAMILY_COLORS.get(r["family"]), lw=2, label="%s (%s)" % (FAMILY_NAMES.get(r["family"], r["family"]), label_of(r)))
        ax.set_xlabel("mean slip angle (deg)"); ax.set_ylabel("lateral force / load (g)"); ax.set_xlim(0, xmax); ax.set_ylim(0, ymax)
        ax.grid(alpha=0.3); ax.legend(fontsize=8); ax.set_title("Families that run on the ground under the sled")
        save(fig, "refs", "BeamNG's own tyre and the modes that keep BeamNG's tyre on the ground, on the ground under the sled.")

    # 2./3. every surface of the FULL modes, the references dashed behind
    for fam in ("rbr_full", "svj_full"):
        rs = forced[fam]
        if not rs:
            continue
        n = len(rs)
        cols = 4 if n > 4 else n
        rows_ = int(math.ceil(n / cols))
        fig, axes = plt.subplots(rows_, cols, figsize=(3.6 * cols, 2.9 * rows_), squeeze=False, sharex=True, sharey=True)
        for k, r in enumerate(rs):
            ax = axes[k // cols][k % cols]
            for q in refs:
                s, y = curve(q)
                ax.plot(s, y, color=FAMILY_COLORS.get(q["family"]), lw=1, ls="--", alpha=0.7)
            s, y = curve(r)
            ax.plot(s, y, color=FAMILY_COLORS[fam], lw=2.2)
            ax.set_title(r["surface"], fontsize=8); ax.grid(alpha=0.3); ax.set_xlim(0, xmax); ax.set_ylim(0, ymax)
        for k in range(n, rows_ * cols):
            axes[k // cols][k % cols].axis("off")
        fig.suptitle("%s on each surface (dashed: the reference families on the ground under the sled)" % FAMILY_NAMES[fam], fontsize=10)
        fig.supxlabel("mean slip angle (deg)", fontsize=9); fig.supylabel("lateral force / load (g)", fontsize=9)
        save(fig, "surfaces_" + fam, "%s, one panel per surface." % FAMILY_NAMES[fam])

    # 4. peak grip per surface with the references as lines
    for fam in ("rbr_full", "svj_full"):
        rs = forced[fam]
        if not rs:
            continue
        fig, ax = plt.subplots(figsize=(max(6, 0.55 * len(rs) + 2), 4.2))
        ax.bar(range(len(rs)), [r["m"]["peak_g"] for r in rs], color=FAMILY_COLORS[fam])
        for q in refs:
            ax.axhline(q["m"]["peak_g"], color=FAMILY_COLORS.get(q["family"]), ls="--", lw=1.2,
                       label="%s (%s)" % (FAMILY_NAMES.get(q["family"], q["family"]), label_of(q)))
        ax.set_xticks(range(len(rs))); ax.set_xticklabels([r["surface"] for r in rs], rotation=60, ha="right", fontsize=7)
        ax.set_ylabel("peak lateral force / load (g)"); ax.grid(axis="y", alpha=0.3); ax.legend(fontsize=7)
        ax.set_title("Peak grip, %s" % FAMILY_NAMES[fam])
        save(fig, "peak_" + fam, "Peak grip of %s on every surface; the dashed lines are the reference families on the ground under the sled." % FAMILY_NAMES[fam])

    # 5. all surfaces of a FULL family on one plot
    for fam in ("rbr_full", "svj_full"):
        rs = forced[fam]
        if len(rs) < 2:
            continue
        fig, ax = plt.subplots(figsize=(8, 4.6))
        cmap = plt.get_cmap("turbo")
        for k, r in enumerate(rs):
            s, y = curve(r)
            ax.plot(s, y, color=cmap(k / max(1, len(rs) - 1)), lw=1.6, label=r["surface"])
        ax.set_xlabel("mean slip angle (deg)"); ax.set_ylabel("lateral force / load (g)"); ax.set_xlim(0, xmax); ax.set_ylim(0, ymax)
        ax.grid(alpha=0.3); ax.legend(fontsize=6, ncol=2); ax.set_title("%s, all surfaces" % FAMILY_NAMES[fam])
        save(fig, "all_" + fam, "All surfaces of %s together." % FAMILY_NAMES[fam])

    # the table -----------------------------------------------------------------------------------------------------------
    def fmt(x, f="%.2f"):
        return "-" if x is None or (isinstance(x, float) and math.isnan(x)) else f % x
    order = {k: i for i, k in enumerate(FAMILY_ORDER)}
    good.sort(key=lambda r: (order.get(r["family"], 99), str(r["surface"] or ""), r["folder"]))
    cols_ = ["family", "surface", "peak_g", "slip_peak", "stiffness", "g5", "g10", "g_end", "retention", "noise", "fz_cv", "max_slip", "folder"]
    with open(os.path.join(out, "summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["family", "surface", "peak_g", "slip_at_peak_deg", "stiffness_g_per_deg", "g_at_5deg", "g_at_10deg", "g_end", "retention",
                    "noise", "load_cv", "wheel_speed_jitter", "max_slip_deg", "folder"])
        for r in good:
            m = r["m"]
            w.writerow([r["family"], label_of(r), "%.4f" % m["peak_g"], "%.2f" % m["slip_peak"], "%.4f" % m["stiffness"], "%.4f" % m["g5"],
                        "%.4f" % m["g10"], "%.4f" % m["g_end"], "%.3f" % m["retention"], "%.4f" % m["noise"], "%.3f" % m["fz_cv"], "%.4f" % m["jitter"],
                        "%.1f" % m["max_slip"], os.path.basename(r["folder"])])

    # html ------------------------------------------------------------------------------------------------------------------
    h = []
    h.append("<!doctype html><html><head><meta charset='utf-8'><title>%s</title><style>" % html.escape(a.title))
    h.append("body{font-family:system-ui,Segoe UI,Arial,sans-serif;max-width:1100px;margin:24px auto;padding:0 16px;color:#1b1f24}"
             "table{border-collapse:collapse;font-size:13px;margin:8px 0 18px}th,td{border:1px solid #cdd3da;padding:4px 8px;text-align:right}"
             "th{background:#eef1f4}td.l,th.l{text-align:left}img{max-width:100%;margin:4px 0}.bad{color:#b3441a}.small{color:#555;font-size:12px}"
             "h2{margin-top:30px;border-bottom:1px solid #cdd3da;padding-bottom:4px}</style></head><body>")
    h.append("<h1>%s</h1>" % html.escape(a.title))
    h.append("<p class='small'>Generated from %d batch folder(s). Lateral force over load (g) against the mean slip angle of the wheels, "
             "measured on the test sled.</p>" % len(folders))
    h.append("<h2>Setup</h2><table><tr><th class='l'>folder</th><th>created</th><th>speed</th><th>ramp</th><th>to slip</th><th>SVJ tyre</th><th>RBR tyre</th>"
             "<th>rbrGripScale</th><th>extra grip scale</th><th>runs ok / all</th></tr>")
    for folder, man in manifests:
        jobs = man.get("jobs", [])
        if isinstance(jobs, dict):
            jobs = list(jobs.values())
        nok = sum(1 for j in jobs if j.get("status") == "ok")
        h.append("<tr><td class='l'>%s</td><td>%s</td><td>%s km/h</td><td>%s s</td><td>%s deg</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%d / %d</td></tr>" % (
            html.escape(os.path.basename(folder)), html.escape(str(man.get("created", ""))), man.get("speedKmh", ""), man.get("rampSec", ""),
            man.get("maxSlip", ""), html.escape(str(man.get("svjTyre", ""))), man.get("rbrTyre", ""), man.get("rbrGripScale", ""),
            man.get("svjGripScale", ""), nok, len(jobs)))
    h.append("</table>")
    for n in notes:
        h.append("<p class='bad'>%s</p>" % html.escape(n))

    h.append("<h2>Summary</h2><table><tr><th class='l'>tyre model</th><th class='l'>surface</th><th>peak (g)</th><th>slip at peak (deg)</th>"
             "<th>stiffness (g/deg)</th><th>g at 5&deg;</th><th>g at 10&deg;</th><th>g at the end</th><th>kept after the peak</th><th>noise</th>"
             "<th>load variation</th><th>wheel speed jitter</th><th>reached (deg)</th></tr>")
    for r in good:
        m = r["m"]
        h.append("<tr><td class='l'>%s</td><td class='l'>%s</td><td><b>%s</b></td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>" % (
            html.escape(FAMILY_NAMES.get(r["family"], r["family"])), html.escape(label_of(r)), fmt(m["peak_g"]), fmt(m["slip_peak"], "%.1f"),
            fmt(m["stiffness"], "%.3f"), fmt(m["g5"]), fmt(m["g10"]), fmt(m["g_end"]), fmt(m["retention"] * 100, "%.0f%%"),
            fmt(m["noise"] * 100, "%.1f%%"), fmt(m["fz_cv"] * 100, "%.0f%%"), fmt(m["jitter"], "%.3f"), fmt(m["max_slip"], "%.1f")))
    h.append("</table><p class='small'>peak: highest value of the smoothed curve. stiffness: slope over the first 2 degrees. kept after the peak: "
             "the grip at the end of the ramp over the peak (low = the tyre lets go past the peak, 100% = it keeps its grip). noise: rms of "
             "the curve's jitter over the peak. load variation: spread of the load on a wheel over its mean (a high value means the wheel "
             "bounces; '-' for BeamNG's own tyre, which reports no load).</p>")

    h.append("<h2>Plots</h2>")
    for name, cap in images:
        h.append("<h3>%s</h3><img src='img/%s.png' alt='%s'>" % (html.escape(cap), name, html.escape(cap)))

    bad = [r for r in runs if r["status"] != "ok" or "m" not in r]
    h.append("<h2>Runs that did not give a result</h2>")
    if bad:
        h.append("<table><tr><th class='l'>run</th><th class='l'>family</th><th class='l'>surface</th><th class='l'>status</th><th class='l'>reason</th></tr>")
        for r in bad:
            h.append("<tr><td class='l'>%s</td><td class='l'>%s</td><td class='l'>%s</td><td class='l'>%s</td><td class='l'>%s</td></tr>" % (
                html.escape(str(r["tag"])), html.escape(FAMILY_NAMES.get(r["family"], r["family"])), html.escape(str(r["surface"] or "")),
                html.escape(str(r["status"])), html.escape(str(r["reason"] or ""))))
        h.append("</table>")
    else:
        h.append("<p>none: every run gave a result.</p>")

    h.append("<h2>How to read this, and its limits</h2><ul class='small'>"
             "<li>BeamNG's own tyre and the modes that keep BeamNG's tyre on the ground (brush, RBR on BeamNG's tyre, SVJ on BeamNG's tyre) "
             "run on the ground under the sled: the Lua tyre cannot be given a surface there, because the game decides the material under the "
             "wheel. Only the FULL modes were told the surface. To compare the others on other materials, move the sled to another patch and run again.</li>"
             "<li>The sled has no steering, no drive and no weight transfer: the load on every wheel is its static load, so these curves are the "
             "tyre alone. A car adds weight transfer and suspension on top.</li>"
             "<li>The grip value is lateral force over the sled's weight (it equals the load on the four wheels); the slip angle is the mean of the "
             "wheels' own measured slip angles.</li>"
             "<li>One run per combination: a small difference between two curves is not necessarily a real difference; run again to see the spread.</li></ul>")
    h.append("</body></html>")
    with open(os.path.join(out, "report.html"), "w", encoding="utf-8") as f:
        f.write("\n".join(h))
    print("report: %s" % os.path.join(out, "report.html"))
    print("%d runs used, %d without a result, %d plots, summary.csv written" % (len(good), len(bad), len(images)))


if __name__ == "__main__":
    main()
