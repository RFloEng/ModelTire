# ModelTire: alternative tyre models for BeamNG.drive

ModelTire is a BeamNG.drive mod that replaces the game's tyre grip with a pluggable tyre model computed in Lua, 2000 times
a second. Two models are included, more can be added (the tyre models are separate modules behind one wheel wrapper):

- the **Richard Burns Rally 1.02 tyre model** (a physical carcass-and-tread model with deformable ground), ported from
  [rerbr](https://github.com/JodusNodus/rerbr), vanilla 1.02 behaviour without the NGP changes;
- a **Pacejka Magic Formula tyre** (MF 5.2 / 6.1 / 6.2) that reads tyre sets from
  [SVJ](https://github.com/RFloEng/SVJ-standard-vehicle-json) files;
- a simple **brush tyre**, used to test the plumbing.

> **Status: experimental.** Tested on BeamNG.drive 0.40.3 with a few cars. Expect rough edges. The Lua tyre is optional and
> off by default; switching it off gives you BeamNG's own tyre back.

> **You must provide the tyre data yourself.** The repository contains no RBR data and no tyre library. RBR tyres are
> generated from your own copy of the game, Pacejka tyres come from SVJ files you supply. See
> [docs/DATA.md](docs/DATA.md).

## Modes

| # | Mode | What carries the load | Needs vehicle reload |
| --- | --- | --- | --- |
| 0 | Test tyre (brush) | BeamNG's tyre | no |
| 1 | RBR tyre on BeamNG's tyre | BeamNG's tyre | no |
| 2 | **Full RBR tyre** | the RBR contact model (BeamNG's ring does not touch the ground) | yes |
| 3 | **Full SVJ Pacejka tyre** | the Pacejka vertical spring | yes |
| 4 | SVJ Pacejka on BeamNG's tyre | BeamNG's tyre | no |

The full modes switch off collision on the tyre ring, so there is no double flex (BeamNG's soft tyre in series with the
model) and the RBR soil model (sinking, bulldozing) works. The rim keeps its collision as a safety stop.

Features: ground surface detection from the terrain, enveloping contact for steps and kerbs, force limits for wheel
safety, 3D force arrows, a browser for SVJ tyre libraries, an RBR tyre picker with your own tyre files, tooltips on every
setting, and floating per-wheel **force-versus-slip graphs** for the Pacejka tyre (curves for several loads and the
wheel's operating point, on the current ground).

## Install

1. Copy `mod/modelTire` to `%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\mods\unpacked\modelTire`
   (or run `python tools/build_zip.py` and put the zip in the `mods` folder).
2. Provide data ([docs/DATA.md](docs/DATA.md)):
   - Pacejka: copy one or more `*.svj.json` tyre files to `%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\modelTire\svj\`.
   - RBR: run `python tools/rbr_import.py --rbr "<your RBR folder>"` (needs your own RBR install). Optional: without it the
     mod uses placeholder tyres.
3. Start BeamNG, spawn a car, the window opens with the mod (the action "ModelTire: show/hide window" can be bound in the
   controls), tick **Lua tyre on the player's vehicle**, choose a mode. The full modes ask for
   "Reload vehicle".

Troubleshooting and tuning are in the [mod manual](mod/modelTire/README.md).

## Repository layout

| Path | Content |
| --- | --- |
| `mod/modelTire/` | the mod (Lua): `tyre.lua` (wheel wrapper), `rbr102.lua` (RBR port), `pacejka.lua`, `svj.lua`, `ui.lua`, `graphs.lua` |
| `tools/rbr_import.py` | builds the private RBR data mod from your RBR install |
| `tools/build_zip.py` | packs the mod |
| `tools/bench/` | offline bench: one wheel on a flat ground through every tyre mode (slip angle, slip ratio and load sweeps, CSV, plots and comparison graphs against the theoretical Pacejka curve and in-game runs); needs `pip install lupa matplotlib` |
| `docs/DATA.md` | which data you have to provide and how |
| `docs/FEASIBILITY.md` | how a Lua tyre fits into BeamNG's physics and what is possible |
| `docs/RBR_vs_NGP_tyre.md` | which parts of rerbr's tyre code are vanilla 1.02 and which NGP |
| `NOTICE.md` | third-party notices |

## Known limits

- One contact point per wheel with an enveloping step filter; it has only been validated offline on steps and kerbs.
- Surface detection reads the terrain's ground model; on maps whose road is a mesh you pick the surface by hand.
- The Pacejka tyre has no soil sinking, temperature or wear; the ground only scales grip and curve shape. Camber thrust
  is zero for fitted tyres that cancel it in MF 6.1.
- Low-speed damping and the load filters are not part of RBR 1.02 or Pacejka; they are there for BeamNG's physics step and
  can be turned off.
- Force feedback differs between modes and from BeamNG's own tyre (mode 1 feels much stronger); it is noted, not yet measured
  or matched. `mzScale` scales the aligning moment.
- Known game issue, not the mod: Vulkan swapchain crashes on some GPUs when resizing the window.
- Not validated against real RBR or a reference Magic Formula implementation (property tests only).

## Licence

Copyright (C) 2026 RFloEng. GPL-3.0-or-later, see [LICENSE](LICENSE): the RBR tyre code is a port of rerbr (GPL-3.0), which makes the whole project
GPL. SVJ is a separate Apache-2.0 project (same GitHub owner) and only its file format is used. Details in
[NOTICE.md](NOTICE.md). This project is not affiliated with Richard Burns Rally or BeamNG, and contains none of their assets
or data.

```bash
git clone https://github.com/RFloEng/ModelTire.git
```
