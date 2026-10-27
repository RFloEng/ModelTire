# ModelTire: alternative tyre models for BeamNG.drive

ModelTire is a BeamNG.drive mod that replaces the game's tyre grip with a pluggable tyre model computed in Lua, 2000 times a
second. The models are separate modules behind one wheel wrapper, so more can be added later. Included:

- the **Richard Burns Rally (RBR) 1.02 tyre model**: a physical carcass-and-tread model with temperature, wear and
  deformable ground (sinking, bulldozing), ported from [rerbr](https://github.com/JodusNodus/rerbr), vanilla 1.02 behaviour
  without the NGP changes;
- a **Pacejka Magic Formula tyre** (MF 5.2 / 6.1 / 6.2) that reads tyre sets from
  [SVJ](https://github.com/RFloEng/SVJ-standard-vehicle-json) files, optionally with a **hydroplaning layer** ("modded
  Pacejka");
- a simple **brush tyre** (Fiala model), used to test the plumbing.

It comes with a control window, live data, force arrows, per-wheel force-versus-slip graphs, in-game sweep tests that drive
the car by themselves, and an offline bench that compares the models without starting the game.

> **Status: experimental.** Developed and tested on BeamNG.drive 0.40.3. Expect rough edges, and read the
> [known limits](#known-limits). The Lua tyre is optional and off by default; switching it off gives you BeamNG's own tyre
> back.

> **You must provide the tyre data yourself.** The repository contains no RBR data and no tyre library. RBR tyres are
> generated from your own copy of the game; Pacejka tyres come from SVJ files you supply. See
> [docs/DATA.md](docs/DATA.md).

## Contents

1. [Why](#why)
2. [Modes](#modes)
3. [Install](#install)
4. [Quick start](#quick-start)
5. [The control window](#the-control-window)
6. [Settings that matter](#settings-that-matter)
7. [Data: RBR tyres, SVJ tyres, your own tyres](#data-rbr-tyres-svj-tyres-your-own-tyres)
8. [Ground surface detection](#ground-surface-detection)
9. [Graphs, sweep tests and the offline bench](#graphs-sweep-tests-and-the-offline-bench)
10. [How it works](#how-it-works)
11. [Safety limits](#safety-limits)
12. [Troubleshooting](#troubleshooting)
13. [Known limits](#known-limits)
14. [Repository layout](#repository-layout)
15. [Development](#development)
16. [Credits and licence](#credits-and-licence)

## Why

BeamNG's tyre is a soft-body ring of nodes whose grip is a friction coefficient per node, handled in the game's core. That
makes it hard to use a published tyre model with its own physics. ModelTire switches the native tread friction off (or
takes the tyre ring out of the ground contact altogether) and applies the forces of a Lua tyre model to the car instead.
The RBR tyre was the starting point because its deformable-ground model (a soil layer with sinking and bulldozing) is
something the native tyre has no equivalent of; the Pacejka tyre makes it possible to use tyre files from other sources.

## Modes

| # | Mode | What carries the load | Vehicle reload |
| --- | --- | --- | --- |
| 0 | Test tyre (brush) | BeamNG's tyre | no |
| 1 | RBR tyre on BeamNG's tyre | BeamNG's tyre | no |
| 2 | **Full RBR tyre** | the RBR contact model | yes |
| 3 | **Full SVJ Pacejka tyre** | the Pacejka vertical spring | yes |
| 4 | SVJ Pacejka tyre on BeamNG's tyre | BeamNG's tyre | no |

- **Light modes (0, 1, 4)** keep BeamNG's soft tyre touching the ground and carrying the load. Only the grip comes from the
  Lua model. BeamNG's tyre flexes in series with the model, so the car feels softer at the first moment of a turn ("double
  flex").
- **Full modes (2, 3)** take the tyre ring out of the ground contact when the vehicle is loaded (it is marked
  non-colliding while the JBeam compiles). The tyre model measures the hub's height over the ground and computes the load,
  the sinking into loose ground and the grip itself. The rim keeps its collision as a safety stop. Switching a full mode on
  or off needs "Reload vehicle".
- **Modded Pacejka:** tick `svjModded` in mode 3 or 4 to add a hydroplaning layer. Above a critical speed (about 95 km/h at
  2.2 bar, higher with deep grooves, lower for wide tyres) a wet road lifts part of the contact on a wedge of water and grip and
  cornering stiffness collapse with speed. Water comes from the ground model or from a depth you set.

In the offline bench the light and full modes give the same steady-state force for the same tyre. What you feel as the
difference between them comes from the dynamics around the tyre (BeamNG's tyre flex, how the load reaches the model, the
damping), not from the tyre model itself.

## Install

Requirements: BeamNG.drive (tested on 0.40.3) on Windows. Python 3 is only needed for the data importer and the offline bench.

1. Copy the folder `mod/modelTire` to
   `%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\mods\unpacked\modelTire`
   (or run `python tools/build_zip.py` and put the zip in the game's `mods` folder).
2. Provide data (see [below](#data-rbr-tyres-svj-tyres-your-own-tyres)):
   - Pacejka: copy one or more `*.svj.json` tyre files to `%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\modelTire\svj\`.
   - RBR: run `python tools/rbr_import.py --rbr "<your RBR folder>"`. This needs your own RBR install. Without it the mod
     uses placeholder tyres.
3. Start BeamNG. After editing any file, press **Ctrl+L** in the game to reload the Lua.

To remove it, delete `mods/unpacked/modelTire` (and `modelTire_rbrdata`, if you generated RBR data).

## Quick start

1. Load a map (the Gridmap is a good first test) and spawn a car.
2. Open the **ModelTire** window. It opens with the mod. If you don't see it, bind the action "ModelTire: show/hide window"
   in Options, Controls, or run `modelTire_ui.toggleWindow()` in the game's GE Lua console.
3. Tick **Lua tyre on the player's vehicle**.
4. Choose a mode under "1. Tyre mode". Modes 2 and 3 ask for **Reload vehicle**; the tyre turns itself back on afterwards.
5. For the Pacejka modes pick a tyre in "3. SVJ tyre". For RBR modes pick the RBR tyre in the Tuning section.
6. Drive. The live data table shows the load, forces, slip and surface per wheel; tick the force arrows to draw them in the
   3D view.

## The control window

| Section | What it does |
| --- | --- |
| master switch | turns the Lua tyre on or off for the player's vehicle; everything below is greyed out when it is off |
| 1. Tyre mode | the five modes, a Reload vehicle button, the rim safety-stop option |
| 2. Ground surface | automatic detection from terrain, meshes and decal roads, or a manual surface; the ground probe lines |
| 3. SVJ tyre | the SVJ library browser (filter, list, model choice) and a graphs switch |
| Tuning | the RBR tyre picker, grip, soil softness, aligning moment, Pacejka grip and load options |
| Damping and stability | load filter, vertical damping, fade-in, force limits, enveloping contact |
| Advanced / test tyre | rarely needed settings and the brush tyre parameters |
| Live data | per-wheel load (100 ms average), load noise, forces, slip, friction, patch nodes, surface, temperatures, wear |
| Sweep tests | opens the in-game sweep window (works with the Lua tyre off too, for the vanilla reference run) |

Every setting has a `(?)` tooltip. Settings that do not apply to the chosen mode are greyed out.

## Settings that matter

| Setting | Default | Applies to | What it does |
| --- | --- | --- | --- |
| `rbrGripScale` | 0.95 | RBR | multiplies all RBR surface grip; RBR tarmac grips more than most BeamNG tyres, so lower it if the car rolls over |
| `mzScale` | 1.0 | modes 1-4 | strength of the self-aligning moment (the force feedback) |
| `fullSoilSoftness` | 0.6 | full RBR | how soft loose ground is; lower = firmer, higher = sinks more |
| `svjFz0Scale` | 1.0 | Pacejka | scales the nominal load of tyres without one (higher = the tyre acts as built for a heavier car) |
| `svjSurfaceShape` | on | Pacejka | the ground changes the curve shape (lower peak at larger slip), not only the grip |
| `svjModded` + `hydro*` | off | Pacejka | hydroplaning layer: `hydroWaterMm`, `hydroTreadMm`, `hydroVoid`, `hydroMuFloor` |
| `fzFilterMs` | 15 | modes 0, 1, 4 | smooths BeamNG's flickering contact load; without it the tyres lose force whenever the contact blinks |
| `fullMaxLoadG` | 6 | full | cap on the vertical load, in multiples of the static wheel load |
| `fullFadeIn` | 0.3 s | full | the tyre forces fade in after switching on or a reset |
| `fullSlewKN` | 4 | full | most the tyre force may change per millisecond (kN), a spike filter |
| `fullEnvelope` | on | full | enveloping contact for steps and kerbs |

## Data: RBR tyres, SVJ tyres, your own tyres

The repository ships models, not data. Details and sources in [docs/DATA.md](docs/DATA.md).

- **RBR tyres** (modes 1 and 2): from your own legal copy of Richard Burns Rally. `tools/rbr_import.py` reads the 7 tyre
  types and the surface table and writes a private mod, `modelTire_rbrdata`, into your BeamNG user folder. Do not share
  that folder. Without it the mod uses four placeholder tyres with rough first-guess numbers, not RBR's.
- **Your own RBR tyres:** put `.json` files in `.../current/modelTire/rbrtyres/`. Each names a base tyre and the fields it
  changes: `{"name": "my gravel", "base": "gravel_dry", "set": {"carcassStiffnessLateral": 300000}}`. The picker lists them
  after the built-in ones.
- **Pacejka tyres** (modes 3 and 4): SVJ tyre files (the `tires` section with MF coefficient sets) in
  `.../current/modelTire/svj/`. The window lists every set in them. Missing nominal load, aligning or rolling
  coefficients are replaced by documented fallbacks, and tyres the reader finds suspect are marked.

## Ground surface detection

In the full modes the tyre needs to know the ground. In priority order:

1. **Decal and mesh roads:** a ray down from each wheel finds a mesh under it (bridges, mesh roads, objects) and reads the
   ground type of its materials. Decal roads (they have no collision geometry) are tested against their material's ground
   type; decals that are only paint (tyre marks, cracks, lines) are ignored.
2. **Terrain:** the terrain material under each wheel gives the ground model name.
3. **Manual:** if nothing is found, the surface you chose in the window is used.

Names the tables do not know (a map's own `DIRT_ROCKY`, `ASPHALT_ROUGH_WET`) are reduced to the nearest known ground by
keyword. The probe lines in the Ground surface section show what was found, so a map that does not work can be diagnosed
from the screen. The mesh and decal-road probes were written from the game's Lua and tested with stubs and a few maps;
more maps are the best test they can get.

## Graphs, sweep tests and the offline bench

**Force-versus-slip graphs.** In a Pacejka mode, tick "show force-versus-slip graphs". One floating window per wheel shows
sideways force against slip angle and longitudinal force against slip ratio, with a family of curves for several loads, the
curve for the load now, a grey reference curve when the ground is slipperier than the tyre's reference surface, and a dot
where the wheel is.

**In-game sweep tests.** The "Sweep tests (in-game) ..." button opens a window that drives the player's car by itself on a
flat area (a steering, braking or throttle ramp at a chosen speed) and records the tyre data, so the same manoeuvre can be
compared between modes. Run it once with the Lua tyre **off** (the BeamNG vanilla reference) and once per mode. The plots show
the car's measured acceleration in g against the mean slip of the wheels, plus a dashed theoretical Pacejka curve for the
selected tyre. A view selector switches between the whole car, per axle and per wheel (per wheel and per axle use each tyre's
own force; BeamNG's native tyre gives no per-wheel force, so a vanilla run is an estimate there). A steering run also records
the **steering rack force**, rebuilt from the steering hydros because the game computes its own force-feedback value only when
a force-feedback wheel is connected. Runs are saved as CSV in `.../current/modelTire/sweeps/`. A steering run stops when the
car spins (slip angle above 30 degrees).

**Offline bench.** `tools/bench` runs the real wheel code against a fake wheel and flat ground, without the game
(`pip install lupa matplotlib`):

```bash
python tools/bench/run_bench.py --svj path/to/tires.svj.json --key <tyre set> --loads 2000,3500,5000 \
    --ingame path/to/vanilla_run.csv path/to/other_run.csv
```

It sweeps slip angle (-20 to 20 degrees), slip ratio (-0.5 to 0.5) and load for every mode, prints peak force, slip at the
peak and stiffness, and writes `compare_lat.png`, `compare_lon.png` and absolute-force plots to `tools/bench/out/`. The
comparison graphs show the bench modes, the theoretical Pacejka curve at each load and any in-game runs you pass in.
First results: the bench's Pacejka curve lies exactly on the theoretical one (light and full), and the RBR and brush tyres
lose grip per newton as the load rises, each in its own way.

## How it works

- **Native friction off.** The tread nodes' friction is set to about zero (light modes), or the tyre ring is made
  non-colliding when the vehicle is compiled (full modes). The core then only handles vertical contact or none at all.
- **A wheel wrapper at 2000 Hz.** `tyre.lua` runs per physics step and per wheel. It finds the ground normal and height
  (a height query under the hub, with an enveloping contact over steps), the axle, the velocities and the wheel spin, and
  hands them to a model.
- **Models.** `rbr102.lua` (RBR 1.02: sticking and sliding models blended between 1 and 2 m/s, soil solve, heat and wear),
  `pacejka.lua` (Magic Formula with combined slip, aligning and overturning moments, rolling resistance, a transient
  relaxation of the contact deflections) and the brush tyre inside the wrapper.
- **Forces back on the car.** Light modes spread the friction force over the contact-patch tread nodes. Full modes apply one
  wrench (force and moments) to the axle nodes and the wheel-spin torque as tangential forces on rim nodes.
- **Game side.** `ui.lua` is the control window, `svj.lua` reads SVJ files, `graphs.lua` and `sweep.lua` draw the curves, and
  the game-side hook makes the tyre ring non-colliding when a full mode is on.
- **Not in the originals, added for BeamNG:** low-speed damping, load filtering, slew limiting and the enveloping contact.
  They exist because BeamNG's physics step and soft tyre are noisy; most can be turned off.

## Safety limits

A tyre force model applied to a soft-body car can do violent things with bad inputs, so the full modes are bounded: a load
cap (`fullMaxLoadG`), a fade-in after switching on or a reset, a slew limiter on the force, a friction-limit clamp on every
force, a torque cap on the wheel spin, a guard against non-finite or absurd values and a limit on the damping. The first time
a force is skipped the game log says which wheel and why.

## Troubleshooting

| Symptom | What to try |
| --- | --- |
| window not visible | Ctrl+L; bind the action "ModelTire: show/hide window"; or run `modelTire_ui.toggleWindow()` in the GE Lua console |
| a full mode says "press Reload vehicle" | the ring collision is decided when the vehicle loads; use the Reload vehicle button |
| the car creeps or slides at low speed | check `fzFilterMs` (15 ms default; raise it to 30-40); the load filter exists exactly for this |
| the wheels break, deflate or the car explodes on spawn | load cap, fade-in and slew limiter are on by default; if they were changed, restore them, and send the game log line |
| ice, gravel or a bridge reads as the wrong ground | read the "ground probe" lines in the Ground surface section and the game log lines starting `mesh probe:`; pick the surface by hand meanwhile |
| the force feedback is different from vanilla | expected, noted but not matched; `mzScale` scales the aligning moment |
| a crash with a swapchain size such as 1222x39 in `beamng.log` | a known game/driver problem when the window is resized, not the mod; play in fullscreen or borderless, try the launch option `-gfx dx11`, or another GPU driver |
| a sweep test does nothing | the test needs a flat open map and a vehicle with telemetry; check that the window says "accelerating" and that Stop gives the controls back |

## Known limits

- One contact point per wheel with an enveloping filter for steps and kerbs; it has been validated offline, not against real
  rally steps.
- The Pacejka tyre has no soil sinking, temperature or wear; the ground scales its grip and curve shape. Camber thrust is zero
  for fitted tyres that cancel it in MF 6.1. SVJ `brush` and `tmeasy` tyre models are read but not simulated.
- BeamNG's own tyre cannot run offline, so its curve only comes from the in-game sweep.
- The force feedback differs between modes and from vanilla (mode 1 felt much stronger); it is recorded, not yet matched.
- The ground probes (mesh, decal road) are tested on a few maps; mesh-only maps may need the manual surface.
- Not validated against real RBR or a reference Magic Formula implementation (property tests and the bench against the
  equations only).
- Tested on one game version and a handful of cars. Other vehicles with unusual wheels or steering (torsion-bar steering,
  dual wheels) may not work.

## Repository layout

| Path | Content |
| --- | --- |
| `mod/modelTire/` | the mod (Lua): `tyre.lua`, `rbr102.lua`, `pacejka.lua`, `hydro.lua`, `svj.lua`, `ui.lua`, `graphs.lua`, `sweep.lua` and the mod manual |
| `mod/modelTire/README.md` | the detailed manual and development log, feature by feature |
| `tools/rbr_import.py` | builds the private RBR data mod from your RBR install |
| `tools/build_zip.py` | packs the mod into `modelTire.zip` |
| `tools/bench/` | the offline bench (`run_bench.py`, `bench.lua`) |
| `docs/DATA.md` | which data you have to provide and how |
| `docs/FEASIBILITY.md` | how a Lua tyre fits into BeamNG's physics and what is possible |
| `docs/RBR_vs_NGP_tyre.md` | which parts of rerbr's tyre code are vanilla 1.02 and which NGP |
| `NOTICE.md` | licences and credits of the parts that are not ours |

## Development

- The game loads the mod from its `mods/unpacked` folder; copy the changed files there and press Ctrl+L. A full mode needs
  "Reload vehicle" because the collision flags are set when the vehicle compiles.
- The Lua is tested offline with a LuaJIT runtime (`lupa`) and stubs of the game's API, and with the bench; the in-game
  behaviour needs the game. The sweep tests are the in-game check.
- The tyre model modules take plain inputs (normal, axle, velocity, spin, load or hub height, dt) and return forces and
  moments, so a new tyre model is one more module and one more mode in the wrapper.
- Each file carries an SPDX licence identifier and the copyright line.

## Credits and licence

Copyright (C) 2026 RFloEng. GPL-3.0-or-later, see [LICENSE](LICENSE): the RBR tyre code is a port of
[rerbr](https://github.com/JodusNodus/rerbr) (GPL-3.0, "Copyright (C) the ReRBR authors"), which makes the whole project GPL.
The Pacejka tyre reads [SVJ](https://github.com/RFloEng/SVJ-standard-vehicle-json) files (Apache-2.0; a separate project,
same GitHub owner; only its file format is used) and implements the published Magic Formula equations (Besselink, Schmeitz,
Pacejka; Pacejka, *Tyre and Vehicle Dynamics*). Details in [NOTICE.md](NOTICE.md). This project is not affiliated with Richard
Burns Rally or BeamNG, and contains none of their assets or data.

```bash
git clone https://github.com/RFloEng/ModelTire.git
```
