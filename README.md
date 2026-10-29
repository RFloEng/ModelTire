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
| 5 | **Advanced brush** | BeamNG's tyre | no |
| 6 | **Full Advanced brush** | the brush's vertical spring | yes |

- **Light modes (0, 1, 4, 5)** keep BeamNG's soft tyre touching the ground and carrying the load. Only the grip comes from the
  Lua model. BeamNG's tyre flexes in series with the model, so the car feels softer at the first moment of a turn ("double
  flex").
- **Full modes (2, 3, 6)** take the tyre ring out of the ground contact when the vehicle is loaded (it is marked
  non-colliding while the JBeam compiles). The tyre model measures the hub's height over the ground and computes the load,
  the sinking into loose ground and the grip itself. The rim keeps its collision as a safety stop. Switching a full mode on
  or off needs "Reload vehicle".
- **Full Advanced brush (6)** is the Advanced brush on the full-tyre path: BeamNG's tyre does not touch the ground, the wheel height over
  the ground gives the load (a vertical spring, setting `fullBrushKz`, stiffer towards the rim) and the brush gives the forces, with the
  enveloping contact on steps and kerbs, the force on the hub and the wheel-spin torque on the rim like the other full modes. It has the
  same load, pressure, temperature and wear behaviour as mode 5 and the same brush settings; the surface grip comes from the ground
  model under the wheel (or the manual surface).
- **Contact points of the full modes.** With "debugDraw" on, the full modes (2, 3, 6) draw in the 3D view: a white cross and a grey line
  at the ground under the hub (what the wheel height is measured from), a yellow cross and a cyan line at the contact point the force
  acts at (it leaves the point under the hub on a step, a kerb or a slope: the enveloping contact), and an orange ring of the tyre's
  free radius about the hub. A ring that sinks into the ground, or a contact that is not where the tyre meets the ground, shows at once.
- **Advanced brush (5)** is the test tyre (mode 0, a brush model with relaxation lengths and a friction ellipse) with the things a
  real tyre does on top, all from published tyre physics: its stiffness per newton falls as the load rises (the same shape as the
  Magic Formula's stiffness-against-load term), the stiffness and the grip follow the tyre's own pressure (read from BeamNG's tyre),
  the grip depends on the tyre temperature (best at about 75 C, falling towards 70 % when cold or overheated) and the tyre heats up
  from sliding friction and from rolling, cools with the speed, and wears from the same friction (up to 25 % less grip when worn
  out, faster when overheated). The temperature and wear show in the live data table; "Reset tyre temperature and wear" starts
  again, and `advTimeScale` runs heating and wear faster to see them in a short test. At the nominal load, pressure and temperature
  it gives the brush's force, so it is the brush plus those effects, not a different tyre. The offline bench can run it
  (`--modes brush,adv_brush`, and `--thermal 300 --thermal-slip 6` for a heating test).
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
| 1. Tyre mode | the seven modes, a Reload vehicle button, the rim safety-stop option |
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

**Test sled (in-game rig).** The mod ships a minimal vehicle, "ModelTire test sled": a box on four identical wheels (the game's
205/55R16 tyre), each on its own spring and damper, with adjustable ballast and tyre pressure. Nothing else touches the ground, so
the load on the wheels is exactly its weight. Spawn it on a large flat area, sit in it, and pick the sweep test "rig slip ramp". A
controller (`rig.lua`) moves the sled at the chosen speed, holds its heading, and turns the direction of travel from 0 to the
chosen slip angle, so the slip angle is exactly what you set. It pushes at ground level, which cancels the roll and pitch couples,
and the force it has to supply to hold that velocity is the opposite of the tyres' total force: a ground-truth measurement that
works for any tyre, BeamNG's own included. The plots then compare the modes on the same load and slip with nothing else in the
way. The rig is new and has been run in the game only in an earlier two-wheel form: expect to adjust it.

**Single-wheel rig (the alternative to the sled).** "ModelTire single-wheel rig" is closer to a real tyre testing machine: one
wheel on a carrier that turns about a vertical line through the wheel centre (a kingpin without scrub or caster), inside a light
frame that has no wheel of its own. Everything that holds the rig up and moves it is a force from its controller (`wheelrig.lua`),
so every external force on the rig except the tyre's is known. The **load** is set exactly (slider "wheel load (N)", default 3500 N):
the frame's four lower corners are pushed up with the rig's weight minus that load, so the wheel carries the load whatever the rig
weighs and the tyre's vertical bounce is damped by the controller; virtual vertical springs on the same corners keep the frame level
(a single wheel cannot). The **slip angle** is made like on a machine: the frame goes straight at the set speed and the carrier is
turned against the direction of travel by a servo torque, so the slip angle is the wheel's heading. The torque the servo needs to
hold the wheel is the **aligning torque** of that one tyre (a steering torque cell), restoring positive; the forces Fx, Fy, Fz come
from Newton over each 0.1 s window (mass times acceleration minus the known servo forces), so they work for every tyre mode,
BeamNG's own included. It is used exactly like the sled: spawn it on a large flat area, sit in it, and run the sweep test "rig slip
ramp" or the automatic test, which recognises the rig by itself (both write the same CSVs and reports; the manifest says which rig
and load). Its controller loads itself when the vehicle is spawned, because a single wheel falls over without it. Compared with the
sled it has one tyre (no interaction, no unequal wheel loads), an exact load that can differ from the rig's weight, and a direct
steering torque; it has no suspension, so the tyre's own vertical dynamics are all there is. Offline, the real controller was run
on the real beam structure with a toy tyre (`tools/rig/`): the forces matched the tyre's true forces within a few newtons and the
aligning torque within about 1 N m of 62; the structure itself is untested in the game, so expect to adjust it.

**A battery at several loads.** With the single-wheel rig the window offers the loads 1500, 2500, 3500, 5000 and 6500 N (and one more of
your choice): the whole test is repeated for every ticked load, with the load set exactly by the rig. The report then adds tables of
the peak grip, the stiffness, the slip at the peak and the aligning torque against the wheel load for every tyre model (the load
sensitivity: grip over load falls as the load rises, and each model does it differently), followed by the plots of every load. The
sled has the load of its own weight and ignores the list.

**Automatic test and report (all in the game).** "Automatic test + report ..." (next to the sweep button) runs the sled's slip ramp for
every tyre model and surface you tick, one after the other, with no input from you. For every run it sets the mode and the surface,
reloads the vehicle (a run can capsize the sled, and the FULL modes need the reload anyway), waits for the tyre to be ready, puts the
sled back at its start pose, runs the ramp and keeps the result in `.../current/modelTire/sweeps/batch_<date>/` (one CSV per run plus
`manifest.json`). The FULL RBR tyre runs on its 19 surfaces, the FULL SVJ Pacejka tyre and the FULL Advanced brush on the 8 of the SVJ list; BeamNG's own tyre and the modes that
keep BeamNG's tyre on the ground run once on the ground under the sled (the game decides that material; to compare them on another one,
move the sled to another patch and run again). A full run of everything takes about 15 minutes. The window shows the progress and,
line by line, the result of every finished run (peak grip, slip at the peak, grip at the end, how much of the peak is kept); the
runs also appear as curves in the sweep window. When the test ends the game writes `report.html` (setup, summary table, plots of the
tyre models and of every surface, the runs that failed and why) and `summary.csv` into the batch folder, and "Open report" opens it in
your browser. Nothing outside the game is needed. `tools/report.py "<batch folder>"` (Python and matplotlib) makes a richer version of
the same report, and merges several folders.

**Steering feel (force feedback).** The sled's yaw hold holds its heading with a couple; whatever yaw moment the four tyres put on the
sled, that couple has to supply the opposite, so it measures the tyres' total yaw moment (the aligning torque, restoring = positive)
for every mode, BeamNG's own tyre included, and it captures what the game or a model actually produces, such as a force spread along
the contact patch. This is what a steering wheel feels before the car's steering geometry and the game's force feedback processing:
the slope at low slip is how heavy the wheel feels about the centre, the peak and its slip angle where it is heaviest, a drop after
the peak is the wheel going light as the tyre slides, a sign change a wheel that pulls the other way; trail = aligning torque over the
sideways force. The CSV has `rig_Mz`; the in-game table and `report.html` list the numbers and plot them (tyre models on the ground
under the sled, and every surface of the FULL modes); `tools/report.py` does the same and merges folders. It is not the force on the
steering wheel itself (that needs the steering geometry); for that the sweep's steering ramps on a car record the rack force.

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
| `tools/rig/` | generator of the single-wheel rig's vehicle files (`gen_wheelrig.py`, with the tyre definition it copies) |
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
