# ModelTire: mod manual and development notes

User manual and development log of the mod, in the order the features were built (the repository README has the
overview and the installation steps). Licence: GPL-3.0.

## Files

| File | Job |
| --- | --- |
| `lua/vehicle/extensions/modelTire/tyre.lua` | Vehicle Lua `modelTire_tyre`: the tyre model and force application |
| `lua/ge/extensions/modelTire/ui.lua` | Game Lua `modelTire_ui`: control window, parameters, live data |
| `scripts/modelTire/modScript.lua` | Loads the window when the mod is active |
| `lua/ge/extensions/core/input/actions/modelTire.json` | Bindable action "ModelTire: show/hide window" |

Install by copying this folder to `%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\mods\unpacked\modelTire` (delete
that folder to remove it). After editing the files, copy them again and press Ctrl+L in game to reload Lua.

## Test

Car: **Hirochi Sunburst** (any rally config). The code works on any car with pressure wheels.
Map: Gridmap V2 (flat tarmac, then the dirt areas).

1. Start the game, spawn the Sunburst. The "ModelTire" window shows. If not, open the console
   (`~`) and run `modelTire_ui.toggleWindow()`.
2. Tick **Lua tyre on player vehicle**. The table should fill with four wheels. Check the **Fz** column: about
   2,500-4,000 N per wheel at rest.
   - If Fz is 0: the core doesn't report load once friction is off. Tick `fzFallback` and note it.
3. Turn on debug view (vehicle debug, or Ctrl+K once) to see the lines: green = tyre force, red = sliding,
   blue = load.

| # | Check | What good looks like |
| --- | --- | --- |
| 1 | Stand still, 10 s | The car doesn't creep or shake; forces near 0 |
| 2 | Park on a slope, handbrake off, in neutral | Holds or creeps very slowly (the sticking spring) |
| 3 | Full-throttle launch | Wheelspin, then grip; no explosion or wobble |
| 4 | 80 km/h, hard braking | Stops straight; ABS works or wheels lock (red lines) |
| 5 | Steady circle at 50-60 km/h | Smooth Fy, sensible understeer/oversteer |
| 6 | Slalom, quick steering | No high-frequency buzz in the steering or the body |
| 7 | Handbrake turn on dirt | Rear slides, then grip comes back |
| 8 | Jump and land | Wheels stay stable after landing |
| 9 | Step cost in the window | Under ~40 µs for all wheels (about 8% of the 0.5 ms step) |
| 10 | Untick the box | Native tyre comes back and the car drives normally |

If it shakes or blows up, try in this order: lower `dampRatio` (0.2), raise `relaxLenY`/`relaxLenX`,
lower `cornerCoef`/`longCoef`, set `patchEverySteps` to 1, tick `slipFromTread`.

Please report: which checks passed, the Fz values, the step cost, and anything odd (a screenshot of the window
helps).

## RBR 1.02 tyre (Phase 1, in progress)

Tick `useRbr102` in the window to swap the brush tyre for the Lua port of RBR 1.02's tyre (vanilla, no NGP
parts). Pick the tyre with `rbrTyre`: 1 tarmac, 2 tarmac wet, 3 gravel, 4 snow. A second table then shows surface,
tread and carcass temperature, wear, how far the tyre sinks into loose ground, the sliding weight and the aligning
torque.

| File | Job |
| --- | --- |
| `lua/vehicle/extensions/modelTire/rbr102.lua` | The 1.02 tyre step, ported from rerbr's `tyre/` with NGP's changes undone |
| `lua/vehicle/extensions/modelTire/rbr102data.lua` | Our own tyre types and surfaces (not RBR's data) |

What is ported: contact frame, patch geometry, the soil solve (sink depth), slip, the sticking/sliding blend
(1-2 m/s), friction curve with 1.02 falloff and end slips, friction circle, carcass deflection and torsion, damping
factors, pneumatic trail, low-speed hold, rolling resistance, soil drag, temperature grip (with 1.02's quirk,
switch `rbrQuirk102`), heat, wear and cooling.

Differences from RBR, on purpose:
- The load comes from BeamNG's tyre; the 1.02 vertical solve is only used for the sink depth on loose ground.
- `rbrLowSpeedDamping` (default 2000 N·s/m) is not in 1.02. Without it the parked car wobbles ±8 mm, because the
  1.02 carcass is an undamped spring at standstill. Set it to 0 for pure 1.02.
- The cooling uses 1.02's plain heat-flow formulas with our own areas (rerbr only has NGP's versions).
- All tyre and surface numbers are first guesses, not RBR's data.

Offline rig results (single wheel, 3,500 N load, tarmac tyre): lateral peak ~3,250 N at 8°, longitudinal peak
~3,350 N at 10 % slip, locked-wheel stop, launch without wheelspin at 600 N·m, holds a side push at standstill,
20 s of sliding heats the surface to ~56 °C, about 1 µs per tyre step.

Tuning knobs: `rbrGripScale` (all surface friction), `rbrLatStiffScale` (carcass stiffness; lower it if the car
feels too stiff, since BeamNG's tyre already flexes).

## Using your own RBR data (not shipped)

The mod ships only our own tyre and surface values. Players who own Richard Burns Rally can make a private data
file from their install, and the mod then uses RBR's real values:

```
python tools/rbr_import.py --rbr "D:\RSF"
```

- Reads `Physics/michelin/tyres.lsp` (the 7 tyre types) and the retail `Physics/physics.lsp` (128 surfaces) from
  the install's `physics.rbz`. RSF/NGP installs replace `physics.lsp` with placeholders; the tool then takes the retail
  `physics.rbz` out of RSF's `rsf_installer_files/rbr_install.7z.001` with the install's own `7z.exe`.
  `--physics-rbz <file>` reads a given retail `physics.rbz` instead.
- Writes `mods/unpacked/modelTire_rbrdata/lua/vehicle/modelTire_rbrdata.lua` in the BeamNG user folder: a
  separate mod, so this one stays free of RBR data. Don't share that file.
- No NGP data is read (no `NGP.dll`, no NGP presets).
- The table steps (load, temperature, pressure, contact pressure) and heat constants are not in any data file; they
  are fixed in RBR's code. The tool carries them (`ENGINE_102`, from rerbr's GPL code). Decide before a public release
  whether to keep that, or read them from the player's exe.
- `surfaceMap` at the end of the generated file maps BeamNG ground models to RBR surfaces; edit it freely.

In the window: `rbrUseRbrData` (on by default) switches between your RBR data and our own. With RBR data, `rbrTyre`
is 1 tarmac dry, 2 tarmac intermediate, 3 tarmac wet, 4 gravel dry, 5 gravel intermediate, 6 gravel wet, 7 snow. The
surface column shows the mapping, for example `GRAVEL -> Gravel_Medium_Deep_Dry`.

Offline rig with the real data (4,000 N load, 20 m/s): tarmac dry on dry tarmac peaks at ~5,760 N lateral
(μ ≈ 1.44) at 10°; gravel dry on medium gravel builds to ~3,770 N (μ ≈ 0.94) by 30° and sinks ~12 mm; tyres start
at 65 °C.

## Full RBR tyre (no double flex)

The modes above push RBR's grip onto BeamNG's tyre, so BeamNG's soft tyre still carries the load and flexes (double
flex), and loose soil is only a friction change. The full mode removes the BeamNG tyre from the ground contact:

1. When a vehicle is (re)loaded, the mod's game-side hook (`onJbeamLoadingPhase2`, in `ui.lua`) sets
   `collision = false` on every tyre-ring node of the marked vehicle. The rim (hub) nodes still collide, as a
   safety stop. No per-car JBeam files are needed.
2. Every physics step, for each wheel, `tyre.lua` (`stepWheelFull`) finds the ground under the hub
   (`getSurfaceHeightBelow` plus a 3-point normal) and gives RBR the **hub height**. RBR's own contact solve
   (`rbr102.lua`, `computeLoad`) returns the vertical load from the penetration, with the tread, carcass and
   air pressure in series, and for loose ground the **sink depth**: the same soil model as 1.02, so a tyre on
   gravel or snow sinks 10-15 mm under a car's load and plows.
3. The result is applied to the rim as one force plus moments, so nothing of BeamNG's tyre is in series:
   half the force on each axle node, the perpendicular moment as a couple on the axle nodes, and the wheel-spin
   torque as a pure couple on four rim nodes. Checked offline: the applied force and moment equal the RBR
   contact force at the contact point to the digit.

How to use it (needs a vehicle reload; BeamNG compiles the tyre collision flags at load):
1. Spawn the car, tick the main box (`Lua tyre on player vehicle`) and `useRbr102`.
2. Tick `full RBR tyre on this vehicle`, then press `Reload vehicle`. The game log gets
   `full tyre: vehicle N, 4 wheels, 64 ring nodes without collision` and the window shows `Full tyre state: on`
   (the main box stays ticked and re-enables itself after the reload).
3. Pick the surface in the `surface (full mode)` list. There is no ground-material query without contact, so
   the surface is chosen by hand (tarmac, gravel depths, dirt, grass, snow ...). With your RBR data the list entries
   are real RBR surfaces; without it they map to our own.
4. Untick and reload to go back to the normal tyre. Turn it off before spawning traffic: only marked vehicles are
   changed.

Settings: `fullVertDamping` (RBR has no vertical tyre damping; its suspension damper does the work),
`fullGroundEvery` (ground query every N steps), `fullIgnoreCheck` (skip the check that the ring is free).

Known limits of the full mode:
- The tyre does not roll over kerbs or bumps; contact is at the point under the hub, like RBR.
- Tyre-to-tyre contact with other vehicles is gone (the rim still collides).
- Skid marks, tyre sounds, smoke and the game's ABS slip readouts depend on BeamNG's own contact and are quiet.
- The ring no longer deforms visually.
- Ground material is not detected: you choose it.
- Not yet driven in the game: expect to tune `fullVertDamping`, and to check the sign of the wheel spin and the
  rim-couple direction with a first roll and a first full-throttle launch.

## Known limits (on purpose for Phase 0)

- Tyre sounds, skid marks and smoke read slip from the core, which no longer slides, so they may go quiet.
- Surface grip is a rough table by ground-model name (`surfaceGrip` in `tyre.lua`).
- No heat, wear, pressure, camber or soil drag yet.

## Other cars: wheel size

`rbrScaleToWheel` (on by default) fits the RBR tyre to each BeamNG wheel: the radius always comes from the wheel; with
the option on, the width, the carcass stiffnesses, the load axis and the stiffness-by-load tables scale with the
tyre width (a tyre k times wider carries k times the load at the same pressure), the sidewall height comes from the
wheel (radius minus rim radius) and the heat masses follow the size. The log prints one line per wheel when the
tyre is enabled: `wheel FL: radius 0.242 m, width 0.145 m, tyre scale k = 0.71`. Offline check at 10 mm penetration:
a 145 mm tyre carries 3.2 kN (4.6 kN unscaled), a 300 mm truck tyre 8.7 kN (5.9 kN unscaled). Tyre pressure comes
from the car. Friction levels stay RBR's.

## Standstill noise and low-speed damping

RBR's low-speed mode is the sticking model (a spring that holds the tyre below 1-2 m/s) plus the yaw hold; both are
ported. 1.02 does not damp that spring (it runs at 7 ms steps), so in BeamNG's 0.5 ms soft-body world a parked tyre
rings. `rbrLowSpeedZeta` (default 0.25) adds damping as a fraction of the critical damping of the carcass spring
under the wheel's load; `fullLowSpeedVertZeta` (0.3) does the same for the vertical contact below ~4 m/s. Higher is
not better: damping turns velocity jitter into force noise (offline, 1500 N side push, 300 kg, +-0.02 m/s jitter:
zeta 0 gives +-515 N of noise, 0.2 gives +-67 N, 1.0 gives +-267 N). The wheel table shows 0.1 s averages of the
forces, and `Fz noise` is the peak-to-peak spread of the load in that window.

## SVJ / Pacejka tyres

The full mode can run a Pacejka Magic Formula tyre read from an SVJ file
([Standard Vehicle JSON](https://github.com/RFloEng/SVJ-standard-vehicle-json), Apache-2.0).

How to use:
1. Put `.svj.json` files (a whole-vehicle SVJ, or a tyre-only one such as the repo's
   `examples/tire_mf62_245_40r18.svj.json`) in `<BeamNG user folder>/modelTire/svj/`
   (`%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\modelTire\svj\`). The window creates the folder and shows its path.
2. Tick `full RBR tyre on this vehicle`, press `Reload vehicle` (the SVJ tyre runs inside the full mode).
3. Under `Tyre model inside the full mode` choose `SVJ Pacejka tyre`, pick the tyre set in the `SVJ tyre` list (every
   entry of `tires.sets` of every file is listed with its size and models), pick a surface preset and, if you like,
   the extra grip scale. `Rescan SVJ folder` re-reads the folder.

What is read (SVJ spec section 11): `pacejka` (MF52 / MF62 groups lateral, longitudinal, aligning, overturning,
rolling_resistance, combined; missing coefficients are 0), `reference` (Fz0, pressure, speed), `dimensions`
(overall diameter), `construction` (vertical stiffness and damping, rolling radius), `relaxation`, `pressure`.
`tmeasy` and `brush` sets are listed but greyed out: not simulated yet. External model files (`.tir`, FTire,
CDTire) are not read.

What the model does (`pacejka.lua`, MF 6.1 equations with the .tir/SVJ coefficient names): pure and combined slip
(Fx, Fy with camber, load, pressure), aligning moment Mz, overturning moment Mx, rolling resistance My; the force
builds up with the relaxation lengths (contact-point deflections, exact exponential step, so it also holds a car at
standstill like a spring); vertical load is the tyre's vertical stiffness times the deflection, plus the tyre's
vertical damping; low-speed damping `svjZeta` (0.25) stops the standstill ringing. Sign conventions: ISO axes inside
the model (x forward, y left, z up; alpha = atan(Vy/|Vx|), positive camber = top leans right), forces are those the
ground puts on the tyre (Fy < 0 for alpha > 0, so `pKy1 < 0`).

Surfaces: Pacejka coefficients belong to one surface (usually dry asphalt). The surface presets scale friction
(lambda-mu) and the stiffnesses by its square root; they are rough. Pacejka has no soil model: loose ground only
changes grip, no sinking. Use the RBR tyre for that.

Settings: `svjRadiusFromWheel` (deflection is measured from the BeamNG wheel radius, so the car keeps its ride height
whatever tyre you pick; off = from the SVJ tyre's own radius), `svjUseCarPressure`, `svjUseTyreDamping`, `svjZeta`.

Checked offline with the SVJ example tyre (245/40R18, Fz0 4500 N): peak lateral force 4,799 N (Dy 5,062 N minus the
+261 N ply-steer offset in the file), cornering stiffness 76.7 kN/rad as the formula gives, longitudinal stiffness
~120 kN, peak Fx 5,410 N, combined slip reduces Fy as Fx grows, mu falls with load, camber thrust points to the lean
side, aligning moment changes sign at large slip, rolling resistance 49 N at 25 m/s, and the force and moment applied
to the rim equal the contact force and moment. There is no independent reference implementation to compare with (the
SVJ repo has no tyre evaluator): the equations come from the published MF 6.1 set and have not been compared
with a commercial tool. The example's coefficients are marked "calculated, low confidence" by their author.

## Window layout, tuning and troubleshooting (current)

The window has one master switch (`Lua tyre on the player's vehicle`); when it is off everything below is greyed out.
Then, top to bottom:
1. **Tyre mode** (one choice): 0 test tyre (brush) | 1 RBR tyre on BeamNG's tyre | 2 FULL RBR tyre | 3 FULL SVJ Pacejka
   tyre. Modes 2 and 3 need `Reload vehicle` once (BeamNG decides tyre collision when the car is loaded).
2. **Ground surface**: in the full modes the surface is read from the map's terrain when the map has one (see below),
   else chosen by hand.
3. **SVJ tyre** (mode 3): the tyre list, model, files folder.
4. **Live data**: per-wheel load (0.1 s average), load noise, forces, slip, surface, temperatures.
5. **Settings**, grouped as *Tuning*, *Damping and stability*, *Advanced*, *Test tyre*. Hover the `(?)` next to a setting
   for what it does; settings that do not apply to the chosen mode are greyed.

### Ground surface in the full modes
Without tyre contact BeamNG gives no ground material, so the mod asks the game itself: on maps built on a terrain,
`core_terrain.getTerrain():getMaterialIdxWs(position)` gives the terrain material under each wheel and its ground
model name (ASPHALT, GRAVEL, DIRT, GRASS, SAND, MUD, SNOW, ICE ...), the same call BeamNG's rally editor uses. The
vehicle sends the ground point under each wheel to the game side about 10 times a second and gets the names back
(`modelTire_ui.probeGround` -> `modelTire_tyre.setGround`). RBR mode maps the name to an RBR surface
(`autoRbr` in `tyre.lua`), Pacejka mode to a friction scale (`autoGrip`). Both tables are rough: edit them.
Limits: maps without a terrain (the garage and showroom levels) have nothing to read, so the manual surface is used; a road that is a separate
mesh or a decal over the terrain reads as the terrain beneath it (untick `read the surface from the map's terrain` if
the `surface` column is wrong). A reading of the material of arbitrary mesh triangles does not exist in BeamNG's Lua
API (`Engine.castRay` returns point, normal and object only). The one full-fidelity way would be a light "feeler" node
per wheel whose collision events (`onNodeCollision`) report the material and normal; not built yet.

### Too much grip (car rolls over) and steering feel
- RBR tarmac grips at mu 1.4-1.5; most BeamNG cars have a higher centre of gravity than the RBR cars and roll over at
  that grip. `rbrGripScale` (default now 0.95, calibrated so the full mode matches the vanilla peak (about 1.2 g at 55 km/h) on dry tarmac; the light modes then sit about 10 % below) scales all RBR surface grip; for Pacejka use `extra grip scale` (default 1.25, same calibration).
- Force feedback is the self-aligning torque. `mzScale` (default 1) scales it in all RBR/SVJ modes. In the mode
  `RBR tyre on BeamNG's tyre` the force goes through BeamNG's tyre nodes at the contact patch and the low-speed `hold`
  torque of RBR (up to ~270 N*m) also reaches the steering, which is why that mode can feel much stronger than
  vanilla; lower `mzScale` there. The full modes apply the same torque directly to the wheel hub.

### Bouncing or jittering wheels
Changes made after a report of bouncing polygons in Pacejka mode: the wheel-spin torque is now spread over 8 rim nodes
(was 4: each light rim node got a huge force), the Pacejka vertical spring stiffens towards the rim (`svjProgressive`,
so landings cannot push the rim into the ground), and non-finite or absurd tyre forces are dropped (the log says
`non-finite tyre force`). If something still bounces, note which part (tyre, rim, body panels), when (standing, cornering,
landing) and the `Fz noise` column.

## The tyre library (tires.svj.json)

`svj_analyze --anonymize-tires` produces a tires-only SVJ document (`tires.svj.json`, 2,543 tyre sets, 2,526 with a
Pacejka MF 6.2 fit). Copy it to the SVJ folder (`%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\modelTire\svj\`), choose the
mode `FULL SVJ Pacejka tyre` and press `Load SVJ tyres`: no values to type.

In the window:
- **search** by size text (205/55, R16); **only tyres that fit this car** keeps tyres whose outer diameter is within 4 %
  of the car's wheels and whose width is within 15 %, closest first; **hide suspect fits** hides sets whose fit collapsed
  (no grip, or grip or stiffness at the fit's limits: about 20 of the 2,526); **sort by grip** orders by peak mu.
- every row shows `size  mu <peak grip>  C <cornering coefficient per unit of nominal load>  #<id>`; the selected tyre's
  details are listed under the list.

What the library lacks, and what the mod does about it:
- **Nominal load Fz0 is not stored** (the fit used each tyre's own, and the anonymisation removed it). The mod uses the car's
  static load on each wheel. `svjFz0Scale` (default 1) scales it: higher = the tyre acts as if built for a heavier car;
  `svjFz0` forces a value in N.
- **Sign convention:** `pKy1 > 0` (positive Fy for positive slip angle, allowed by the spec). The model flips the lateral
  force, aligning and overturning moments and the camber so the force still opposes the slip.
- **No aligning-moment coefficients:** a pneumatic trail of about a fifth of the contact length that fades to zero past
  the tyre's peak slip angle gives the self-aligning torque (steering feel). `mzScale` scales it.
- **No rolling-resistance coefficients:** qsy1 = 0.010, qsy3 = 0.0004 (a typical car tyre).
- **One relaxation length** (`relaxation_length_m`, about 0.08 m) for both directions; `svjRelaxScale` scales it. AC-style
  extras (`dcamber`, `speed_sensitivity`, `pressure_model`, `thermal`) are not used. Camber thrust is zero for these tyres
  (their `pKy6` is 0, and the MF 6.1 equations cancel `pVy3`).
- **Grip is high:** the median peak mu is 1.5 (p5 1.2, p95 1.8) at the nominal load, like Assetto Corsa's tyres.
  Use `extra grip scale` (about 0.6-0.8 for cars that roll over) if the car tips in corners.
- The rolling radius of the Pacejka tyre is the car's wheel radius (`svjRadiusFromWheel`), so BeamNG's wheel-speed sensors
  (speedometer, ABS) stay consistent.

Checked offline on all 2,526 sets: no NaN, the force opposes the slip on every healthy set; 205/55R16 at 4 kN: peak
force 5,015 N (mu 1.26) at 10 degrees, cornering stiffness 55 kN/rad, aligning moment peaking at ~94 N*m near 4 degrees,
rolling resistance 44 N at 25 m/s, standstill hold without ringing.

**Distribution:** the mod does not ship `tires.svj.json` (nor any RBR data). A player who wants the library copies the file
into `<BeamNG user folder>\modelTire\svj\`; the window shows that path and says what is missing when the folder is empty.
Any other `.svj.json` file with `tires.sets` in the same folder is listed too.

## Spawn safety (load cap and fade-in)

When a car is (re)loaded in a FULL mode its tyre ring does not collide, so it settles with the **rim** on the ground, far
lower than the tyre expects. An uncapped tyre spring turned that into hundreds of kN per wheel and blew the car apart
within 0.03 s (seen in the log as `fastest node 57 m/s` and `first broken ...`). Two guards now exist:
- `fullMaxLoadG` (default 6): the vertical load per wheel is capped at 6 times the car's static wheel load (both the RBR and
  the Pacejka tyre).
- `fullFadeIn` (default 0.3 s): the tyre forces fade in after the tyre is switched on or the car is reset.
If a tyre force is still NaN or absurd (> 1 MN) it is skipped and the log says so with the values
(`non-finite or absurd tyre force skipped (F ..., hub height ..., load ...)`).

## Steps and kerbs (enveloping contact), force limits, force arrows

**Why wheels broke or deflated (fixed).** The low-speed damping term of the Pacejka tyre was `2*zeta*sqrt(k*m)*slip speed`. Library
tyres have very stiff, short-relaxation longitudinal coefficients, so a speed jitter of a few tenths of a m/s gave forces of
1,000,000 N (the log showed `F -1.02e+06 ... load 2.07e+04`). That went into the rim nodes and broke the wheel beams
(`first broken ... wheeldata_F` about a second after each respawn). Now: the damping uses the car's share of mass and a capped
stiffness; every tyre force is limited to what friction allows (1.2 x the peak); the wheel-spin torque is limited to
2.5 x load x radius; and a slew limiter (`fullSlewKN`, default 4 kN per millisecond) stops spikes, so a 16 kN jump becomes a ramp of
about 4 ms.

**Step-up problem of one-point contact (enveloping contact).** A tyre that reads the ground under the hub only meets a step as a jump
of the ground height, and its ring (not colliding in the full modes) passes through the step face. Fix, as in rigid-ring tyre
models: with `fullEnvelope` on, ten ground samples are read ahead of and behind the hub along the wheel's heading (-30 to 80 degrees
of the wheel circle), an edge between two samples is located by three bisection rays, and the point of the ground closest to the hub
is the contact. The contact normal tilts toward that point, so the force pushes the wheel up and back and the car climbs the step
gradually. Offline test, 6 cm step: single-point contact jumps from 5 kN to the 20.8 kN cap in one step; the envelope starts when the
front of the wheel touches the corner and builds up over about 17 cm. Also `fullHeightTau` (1.5 ms) smooths the hub height.
Cost: about 10 raycasts per wheel per update (`fullEnvEvery` 2). The rim collision still catches walls taller than the hub.

**Force arrows in the game.** Tick `draw force arrows in the 3D view` (Live data): at each wheel's contact point, blue = load, green =
grip force (red when sliding), length from `m per kN` (default 0.25 m per kN). Drawn by the game side every frame from the last
telemetry, relative to the car, so they follow it smoothly while the values update 10 times a second. (The older vehicle-side lines
of the `debugDraw` setting need BeamNG's own vehicle-debug mode and may not show.) Works in all four modes.

## Pacejka on BeamNG's own tyre (mode 4), graphs, and the test tyre at low speed

**Mode "SVJ Pacejka tyre on BeamNG's tyre".** Like mode 1 but with the Pacejka tyre of the chosen SVJ file instead of
RBR's: BeamNG's soft tyre still touches the ground and carries the load, the Magic Formula gives the grip. No vehicle
reload is needed. The vertical load comes from BeamNG (smoothed, see below); slip, camber, pressure and the relaxation
lengths are handled as in the full mode, and the surface grip comes from BeamNG's own ground material. Because BeamNG's
tyre flexes in series, the car feels softer-than-needed in the first moments of a turn; use the full mode when you want
the Pacejka tyre alone.

**Force-versus-slip graphs.** In either Pacejka mode, tick *show force-versus-slip graphs* (SVJ tyre section). One
floating window per wheel opens (move, resize, close each one separately). Each has two plots:
- sideways force against slip angle, and longitudinal force against slip ratio;
- a family of curves for 0.5, 1, 1.5 and 2 times the wheel's static load (so you see how weight changes the peak and the
  stiffness), a white curve for the load the wheel carries right now, and a **red dot where the wheel is**.

The curves are the tyre's pure-slip Magic Formula with the wheel's own nominal load, surface grip, pressure and camber,
evaluated by the same `pacejka.lua` the vehicle runs. The dot shows the force the tyre actually gives; it sits below the
curve while the force is still building up (relaxation length) or when steering and braking/accelerating together
(combined slip). The dot is hollow and stuck to the frame when it is off the plot.

**Test tyre sliding at low speed (fixed).** Cause found by simulation: at low speed BeamNG's contact load flickers
between zero and a spike (the tyre nodes bounce). The brush tyre used that load raw, so its bristles lost their
deflection on every gap and the car crept (in the offline test a parked car on a 5 % slope crept 176 mm in 10 s with a
flickering load). The load is now low-passed (setting `fzFilterMs`, 15 ms by default; 0 = old behaviour), a gap in
contact fades over that time instead of vanishing, and the damper no longer eats into the stored bristle deflection.
The same filter feeds mode 4. If the car still creeps for you, raise `fzFilterMs` (30-40) and report what the live
table shows for *Fz noise*.

## Choosing the RBR tyre, custom tyres, soil softness

- **RBR tyre picker** (Tuning section, modes 1 and 2): a list of the tyres in your RBR data (or the four built-in ones),
  followed by your own.
- **Custom tyres:** drop `.json` files in the BeamNG user folder `modelTire/rbrtyres/` (the mod writes an example
  there on first use; press *Rescan tyre files*). A file is one object or a list of them:
  `{"name": "my gravel", "base": "gravel_dry", "set": {"carcassStiffnessLateral": 300000}}`. `base` names an existing
  tyre of the list, `set` replaces any of its fields (the names are those of `rbr102data.lua` / your generated data).
- **Mud** maps to RBR's `Dirt_Soft_Wet`, already the softest wet dirt RBR has (2 cm of soil, the highest sink factor of
  the dirt family). RBR's numbers are for a light rally car, so a heavy BeamNG car sinks further: the new setting
  **fullSoilSoftness** (default 0.6, full RBR mode) scales the soil's sink factor. Lower = firmer, higher = softer.

## Ground changes the curve shape (Pacejka)

The ground no longer only scales the Pacejka grip. Measurements on loose and slippery surfaces (TRB SR 115 and
Minnesota DOT friction studies; Pacejka's scaling-factor work finds friction and slip stiffness are the factors that
move most) show the peak friction is lower AND is reached at a larger slip: about 18 % slip on dry asphalt, 20 % wet,
about 30 % on snow and ice for road tyres (40-60 % for snow tyres). So the slip stiffness falls faster than the
friction. The mod uses, per ground model, a "peak slip factor" (asphalt 1, wet 1.15, gravel 1.5, dirt 1.5, grass 1.6,
mud/sand/snow 1.8, ice 1.7) and sets stiffness = friction scale / factor. Tyre differences (summer, winter, rally) come
from the tyre's own coefficients; the surface factor is the same for all. The setting `svjSurfaceShape` turns this off
(then stiffness follows sqrt of the friction as before). The graph windows draw a grey "ref" curve for the tyre's
reference surface next to the curve on the current ground, so the change of shape is visible.

## Modded Pacejka (hydroplaning)

Tick `svjModded` in either Pacejka mode (3 or 4) to add a water layer (`hydro.lua`) to the Pacejka tyre. It adds the one
effect the Magic Formula does not have: dynamic hydroplaning. The critical speed follows the NASA/Horne spin-down
relation (vp = 9 sqrt(p[psi]) knots, about 95 km/h at 2.2 bar), raised by groove drainage (`hydroVoid` x `hydroTreadMm`
against the water depth) and lowered for wide tyres. The floating part of the footprint starts at 55 % of vp and is
complete at 115 %, scaled by film/(film + 1 mm); it builds over about a metre of rolling. As it grows, the friction scale
falls to `hydroMuFloor` and the cornering stiffness to 20 %. Water comes from the ground model (wet asphalt 0.6 mm,
wet gravel 1 mm, mud 1.5 mm) or from `hydroWaterMm` on every ground. The wet-road baseline (lower, flatter peak) stays
with the ground model; this layer only adds the speed-dependent loss. Every constant is an estimate, not a measurement
of a tyre. The graph windows show the floating percentage, the critical speed and the water depth, and the curves already
include the loss. Offline sweep, 200 kPa, 205 mm road tyre: 0.6 mm film: grip x0.93 at 70, x0.72 at 90, x0.65 from about
110 km/h; 3 mm: x0.31 from about 110 km/h; a slick on 3 mm floats from about 70 km/h.

## Ground probe for meshes (experimental)

Terrain maps are read through the terrain block. For everything else (mesh roads, bridges, objects) the game has no
direct "ground model here" call, so the mod casts a ray down at each wheel (`Engine.castRay`, 10 times a second), takes
the hit object's material names (`getMaterialNames`), looks each material up in the scene and uses the first
`groundType` it can read. The result overrides the terrain value under that wheel. The Ground surface section lists what
the probe found ("ground probe" lines) and the same lines go to the game log (`mesh probe: ...`), so on a map where it
does not work you can see why: the call failed, the object has no material names, or the material has no readable
`groundType`. The calls were written from the game's Lua and checked offline with stubs only; they have not yet been
run against a real mesh map. If an object has several materials, the first one with a ground type wins.

The probe also tests the wheel position against **decal roads** (they have no collision geometry, so a ray only hits the
terrain below them): each road's middle line, half width and material ground type are cached once per level, and a wheel
inside a road with a readable ground type takes it (for example an ice road painted over grass terrain). The probe lines
say `on decal road <name> (material <m>): <ground>` when this applies.

Ground names the tables do not know (a map's own `DIRT_ROCKY`, `ASPHALT_ROUGH_WET` ...) are reduced to the nearest known
ground by keyword (ICE, SNOW, MUD, SAND, GRASS, GRAVEL, DIRT, ROCK, CONCRETE, METAL, WOOD, ASPHALT, with WET kept); only a
name with no keyword stays on the manual surface. Decal roads that are paint on an existing road (tyre marks, cracks, lines,
patches) are ignored by the decal-road check. If the collision ray finds only terrain, a second ray against render
geometry is tried (bridges and buildings may only have that).

## Offline bench (tools/bench)

`python tools/bench/run_bench.py --svj <file.svj.json> [--key <tyre set>]` runs the real wheel code against a fake
vehicle (one wheel, a flat ground, a hub moved on a prescribed path) for the brush, RBR (light and full) and Pacejka
(light and full) modes: sweeps of slip angle (-12..12 deg), slip ratio (-0.3..0.3) and load (1..7 kN), written to
`tools/bench/out/bench.csv` and `bench_lat/lon/load.png`, with the peak, the slip at the peak and the slope per mode
printed. `--param name=value` overrides a tyre setting, `--load`, `--speed`, `--radius`, `--modes` change the rig. It
measures the wheel code only: BeamNG's suspension, soft tyre and collisions are not in it. Its first result: at the same
load and slip, the light and the full mode give the same steady-state force (RBR 2692 N vs 2700 N peak lateral, Pacejka
3774 N vs 3763 N), so what you feel as a difference between them comes from the dynamics around the tyre (BeamNG's tyre
flex, the load path, the damping), not from the tyre model. The bench also found a real bug, fixed: the shared input table
kept the load of the light path, so a full Pacejka tyre could inherit it.

## In-game sweep tests

The main window has a button **Sweep tests (in-game)**. It opens a window that drives the player's car by itself on a
flat open area (Gridmap): it reaches the chosen speed with a throttle controller, then runs a slow **steering ramp**, a
**braking ramp** or a **throttle ramp** while recording, and gives the inputs back when it ends or when you press Stop.
The result is plotted as **acceleration in g** (the car's measured lateral or longitudinal acceleration, i.e. force over
weight) against the **mean slip of the wheels** (slip angle in degrees or slip ratio, from each wheel's hub velocity and
heading). That is the same quantity for every mode, so runs can be laid over each other:

1. Run with the Lua tyre **off**: the run is labelled "BeamNG vanilla tyre".
2. Switch the Lua tyre on, pick a mode, run again. Each run keeps its colour and a tick box in the list.
3. With a Pacejka tyre selected, the plots also show a dashed **theoretical curve** of that tyre (pure slip, reference
   surface, at the car's mean static wheel load). It ignores weight transfer and the car, so it is a guide, not a target.

Every run is also saved as CSV in the user folder (`modelTire/sweeps/`). The car's acceleration includes its body
dynamics, so use a slow ramp (10 s or more). The vehicle side only collects the extra data (`setNative`) while a sweep
runs.

The bench also draws the **comparison graphs** `compare_lat.png` and `compare_lon.png`: every bench mode as force / load,
the **theoretical Pacejka curve** of the chosen SVJ tyre (pure slip, same load, evaluated directly from the Magic
Formula), and, with `--ingame <csv> ...`, the CSV files of the in-game sweep tests (acceleration in g against mean slip), so
the **BeamNG vanilla tyre** run from the game sits on the same axes. BeamNG's own tyre cannot run offline, so it comes from
the in-game run. First result: the bench's Pacejka curve lies exactly on the theoretical one (light and full mode alike),
which confirms that the wheel wrapper adds nothing to the model; RBR and the brush tyre are shown with their own curves.

The bench sweeps slip angle and slip ratio at several loads (`--loads 2000,3500,5000`, N; `--load` is the nominal load that
is also the Pacejka tyre's reference load). The comparison graphs have one panel per load, each with the bench modes, the
theoretical Pacejka curve at that load and, in the nominal-load panel, the in-game runs. Result at 2 / 3.5 / 5 kN, peak
lateral force over load: Pacejka 1.07 / 1.02 / 0.96 (exactly on the theory curve at every load), RBR 0.82 / 0.80 / 0.76,
brush 1.09 / 1.03 / 0.98: all three lose grip per newton as the load rises, each in its own way.

Update: the slip-angle sweep now runs to 20 deg, the slip-ratio sweep to 0.5 with finer steps at low slip, and the two Pacejka modes are labelled `pacejka_light` / `pacejka_full` (the SVJ file is only the format; the model is the Pacejka Magic Formula). `compare_lon.png` is the longitudinal comparison.

## Brush tyre: smooth knee (Fiala / brush model)

The test tyre (mode 0) no longer clips the bristle force at the friction limit. It uses the standard brush model with a
parabolic pressure distribution: F = Fmax (3u - 3u^2 + u^3) with u = (bristle spring force) / (3 Fmax), constant beyond
u = 1, the same direction rule for combined slip (a friction ellipse), and the bristle deflection held just beyond the
saturation point so it unwinds at once. The force now rounds over instead of showing a corner. The stiffness defaults
changed (`cornerCoef` 28, `longCoef` 30; the force peaks at about 3 mu / coefficient), and the grip still falls with
slip speed (`muSlide`, `stribeckVel`), which gives the gentle decline after the peak. Bench result at 3.5 kN: peak 1.00 g
at 5 deg (was 1.03 g at 4 deg with a sharp corner), a parked car on a 5 % slope with the load filter on creeps 0.5 mm in
10 s.

## RBR on BeamNG's tyre: load smoothing (found with the in-game sweep)

The first in-game sweep showed mode 1 (RBR on BeamNG's tyre) giving far less lateral force than the same RBR tyre in the full
mode, with a noisy staircase curve. Cause: BeamNG's contact load flickers between zero and spikes, and the RBR step treats
every step without load as "no contact" (zero force). Offline, with a load that is zero 40 % of the time (3480 N average) the
mean lateral force was 205 N unfiltered, against 1511 N with a steady load and 1516 N with the 15 ms load filter. The filter
`fzFilterMs` (already used by the brush and the Pacejka-on-BeamNG tyres) now feeds mode 1 too.

## Per-axle and per-wheel curves in the sweep window

The sweep window has a selector **curves of: car (total) / per axle / per wheel**. In the axle and wheel views every panel
plots the group's own lateral (or longitudinal) force over its own load against that group's mean slip, with the
theoretical Pacejka curve at that group's static load. These are the tyre's own forces from the telemetry, exact for
every Lua-tyre run. BeamNG's native tyre reports no force per wheel (the game exposes slip, load and contact data only),
so a vanilla run in these views is dashed and is the car's acceleration in g, assuming every wheel uses the same share
of its grip; use the total view for a measured vanilla curve.

## Force feedback differs between modes (observed, not yet measured)

Observation from play: the steering force feedback is not the same in every mode. Mode 1 (RBR on BeamNG's tyre) felt much
stronger than BeamNG's own tyre and than the full modes, and the full modes and vanilla differ from each other too. The
bench and the sweep tests compare lateral and longitudinal force only; they do not measure what the wheel feels.

Why it is expected to differ (hypotheses, not verified):
- **Where the aligning moment is applied.** The tyre models give a self-aligning moment Mz (RBR: the trail terms and the hold
  torque; Pacejka: the aligning coefficients, or the fallback pneumatic trail for library tyres that have none). In the full
  modes it joins the wrench on the hub. In mode 1 it is applied as a couple along the contact patch of BeamNG's own tyre,
  which flexes and passes it on differently, and the lateral force acts at the patch, a trail behind the hub, so the moment
  arm differs too.
- **BeamNG's own tyre adds its own aligning effect** in the light modes (its carcass and contact geometry are still there).
- **Different trail laws:** RBR's trail falls with the sink depth and the slip, Pacejka's fallback fades with the slip angle,
  vanilla BeamNG has whatever the tyre geometry produces.
- **Slip and load filtering** (`fzFilterMs`, the slew limiter and the height filter) change how fast the force and its moment
  build up.

Tuning: `mzScale` multiplies the aligning moment in modes 1-4 (1 = as computed); lower it if the wheel is too heavy or
shakes. To compare modes properly the sweep should also record the steering force the game sends to the wheel (the
steering rack force). That is not recorded yet.

## Steering rack force in the sweep tests

While a sweep runs, the vehicle side (`setNative`) also reports the **steering rack force**: the game's steering hydros give
way under the load from the tyres, and the force on the rack is that give times the beam spring, summed over the steering
hydros (N). The force the game sends to a force-feedback wheel is built from the same quantity, but the game only computes
it when such a wheel is connected, so it is rebuilt here for every setup. It is a measure for comparing modes, not the
torque at the wheel (no wheel-ratio, smoothing or compression). The sweep window draws it against slip angle for steering
runs (a third plot, in every view) and writes it to the CSV (`rack_N`). Use it to compare the feel of the modes: mode 1,
the full modes and vanilla on the same ramp.

## Test sled (in-game rig)

Vehicle `modelTire_rig` (`vehicles/modelTire_rig/`, generated by a script, not edited by hand): a box on four identical wheels
(the game's 205/55R16 standard tyre, no steering, no drive, `brakeTorque` 2500). Each wheel has its own trailing arm (a rigid
pyramid of two pivot nodes (one near the centre line), two hub nodes and an apex node 0.5 m above that can only turn about the pivot line), two spring beams (35 kN/m each) and
two damping beams (3600 N s/m each): about 2 Hz and 0.7 of critical damping for 400 kg per wheel. Nothing but the four tyres
touches the ground (no skids, no collision on the frame), so the load on the wheels is exactly the sled's weight. Frame: eight
corner nodes carrying the ballast (`$ballast`, default 1400 kg, 400 to 3000) at heights 0.20 and 0.40 m, so the centre of
gravity is at hub height (0.3 m); `$tirepressure` sets the pressure. No meshes, so only the wheels are visible; the wheels are
named `FL FR RL RR` (the sweep window's per-axle view works). Node masses and beam stiffnesses were chosen so that
sqrt(k_total / m) * dt stays below 1 at the 2000 Hz physics step (an earlier version with 1 kg nodes on 22 MN/m beams exploded
within 0.03 s).

`rig.lua` (vehicle extension `modelTire_rig`, loaded by the sweep) runs a PI velocity servo plus a yaw hold on four force nodes
at ground level (`sv_*`, no collision). The servo force acts at the height of the contact patches, so it cancels the tyres' force
without any roll or pitch couple: the wheels keep equal loads whatever the height of the centre of gravity (the first, two-wheel
version had its force at the centre of gravity: the outer wheel carried 7.0 kN and the inner one 1.7 kN). Phases: settle (2 m/s^2
up to the speed), hold 1.5 s, ramp (slip 0 to the maximum, the velocity vector turns while the heading stays), done. The tyre
force on the sled is `m * a - servo force` with `a` the measured acceleration of the centre of mass over the report window (the
mean velocity of the structural nodes, so the frame's bounce does not disturb it), reported in the sled's axes through the telemetry (`data.rig`: phase, slip, Fx,
Fy, load) and written to the CSV (`rig_Fx`, `rig_Fy`, `rig_load`). For rig runs the sweep's total curve is the servo's
`|Fy| / (m g)` for every mode (vanilla included); per wheel and per axle views use the Lua tyre's own force. The controller needs
`enablePhysicsStepHook()` (the game does not call an extension's `onPhysicsStep` otherwise) and logs its phases and a status line
each second to `beamng.log` (tag `modelTire_rig`). An offline check with a point-mass sled and a magic-formula tyre gave the servo
force within 0.1 % of the true tyre force.

First runs in the game with the two-wheel version: the force-versus-slip curves of the BeamNG tyre and of RBR on it were clean, the
full Pacejka mode bounced (load between 0 and 23 kN: nothing damped the wheel's vertical motion) and three runs recorded zero force
on both wheels (cause not found yet: Fz = 0 for the whole run, so the Lua tyre produced nothing; check that the Lua tyre is switched
on again after every respawn of the sled).
Not yet run in the game in this four-wheel form.

Second run in the game (four wheels): the wheels cambered and splayed, and the servo oscillated between -30 and +30 m/s^2. Causes
found: (1) a flat arm (rectangle plus both diagonals) is a singular structure: a rigidity check gives 5 free motions with the pivots
fixed, so the wheel could camber and toe on the soft springs (35 kN/m at 0.25 m only resists 2 kN m/rad, a lateral force of 3.6 kN at
0.315 m gives half a radian); with an apex node the arm has exactly 1 free motion, the hinge. (2) The velocity loop (gain 20) was
fast against the sprung mass's 2 Hz bounce and used a single node's velocity: now gain 8, integral 10, and the centre of mass
velocity from all structural nodes.

Third run in the game: still flexing (a wheel visibly cambered). Checked with a rigidity analysis of the whole structure (the
rigidity matrix of all stiff beams, free motions expected: 6 rigid body + 4 hinges) and a static load case with the real beam
stiffnesses (4 kN of lateral tyre force at ground level on one wheel). Found: (1) the force nodes hung on two collinear beams and a
third: each could slide in y (14 free motions instead of 10); now each is tied to four bottom corners. (2) Pivots and towers were tied
to four far corners of the frame, nearly collinear: soft; now tied to all eight corners and to each other. (3) The arm was a long,
narrow plan (0.25 m wide, 0.6 m long): the hub moved 21 mm sideways and cambered 7 degrees at 4 kN. Now an A-arm: the inner pivot
sits near the centre line (x = 0.10, inside the frame), the apex is 0.5 m above the arm and the arm beams are 12 MN/m: 3.7 mm and
1.1 degrees at 4 kN. Tools: the checks are scripts that read the generated jbeam (not in the repository yet).
Node masses keep sqrt(k_total / m) * dt below 0.9 at the 2000 Hz step.

Fourth version (stiffness and height). Request: arms as stiff as possible, a structure with a wide base, the centre of gravity as low
as possible. Done: the frame bottom (with the ballast, `$ballast`, now 0 to 2400 kg, default 600) is 0.10 m over the ground
(about 0.04 m once the springs have settled), the top corners are light; the arms' hinge line is at 0.15 m, the apex 0.6 m over the
arm, the spring towers at 0.65 m; the inner pivots are near the centre line, the arms 0.8 m long; arm beams 25 MN/m, mounts and
frame 3 MN/m. Each node's mass follows from the stiffness of its beams (sqrt(k_total / m) * dt <= 0.85 at the 2000 Hz step), so
stiffer beams make a heavier sled: the structure is about 1040 kg, centre of gravity 0.29 m with 400 kg of ballast and lower with
more. Static test, 4 kN of lateral tyre force at ground level on one wheel: 1.3 mm of hub shift and 0.35 degrees of camber. The
structure has exactly the 10 free motions of 4 hinges and a rigid body. The sled is generated by a script (not in the repository).

## Automatic test and report

`batch.lua` (extension `modelTire_batch`, window "ModelTire automatic test", opened from the control window) builds a list of runs:
the chosen families (vanilla = Lua tyre off, brush, RBR on BeamNG's tyre, SVJ on BeamNG's tyre, FULL RBR, FULL SVJ) and, for the two
FULL ones, the chosen surfaces (the 19 RBR surfaces, the 8 SVJ ones). The state machine per run: `setup` (ui.lua `testSetup`: mode,
surface, `autoSurface` off, `forceReload`), `ready` (the telemetry says the tyre is in the mode asked for, for 1.5 s; vanilla has no
telemetry, so a fixed 5 s after the reload), `teleport` (the sled goes to the pose it had when the test started: the reload leaves it
where the last run ended), `settle` (still for 2 s), `run` (`modelTire_sweep.runRig`), `wait` (until the sweep is idle, with a
timeout). The user's request: reload the rig between runs, because some runs capsize it. The manifest is rewritten after every run, so
an interrupted test keeps what it has. Failures (tyre mode not active within 90 s, sled not at rest, run ended early, timeout) are
recorded with the reason and the test goes on.

`tools/report.py` reads the folder(s): per run it takes the sled's measured total sideways force over its weight against the mean
slip angle of the wheels, smooths it, and computes peak, slip at the peak, stiffness over the first 2 degrees, grip at 5 and 10
degrees and at the end, the fraction of the peak kept at the end, curve noise (rms jitter over the peak) and load variation (spread of
a wheel's load over its mean: bouncing). It was tested offline with synthetic data (6 families, 19 + 8 surfaces, one failed run); the
state machine was tested with stand-ins for the game (reloads, teleports, a failing run). Not yet run in the game.
First clean rig runs in the game (stiff sled): vanilla 0.95 g peak, RBR on BeamNG's tyre 1.15, FULL RBR 1.2, SVJ on BeamNG's tyre 1.2
with the steepest rise; brush rises very slowly and reaches 0.6 g at 20 degrees (its cornering stiffness is far too low). The earlier
calibration of `rbrGripScale` and the Pacejka grip scale used the car's own tyres (vanilla 1.19 g); the sled's 205/55R16 gives 0.95 g,
so the Lua tyres are 20 to 25 % above vanilla on the sled.

Report in the game (user request: the test and its report must run in the game, automatically). `batch.lua` analyses every finished
run itself (`analyse`: smoothed curve, peak, slip at the peak, stiffness over the first 2 degrees, grip at 5 and 10 degrees and at the
end, kept fraction, noise) from the curve the sweep hands over (`lastResult.curve`), shows a table of them in its window, and at the
end writes `report.html` (inline SVG plots, no libraries) and `summary.csv` to the batch folder; "Open report" calls `openWebBrowser`
and `Engine.Platform.exploreFolder` (both inside pcall: if the game does not offer them the path is shown in the window).
tools/report.py stays as the richer, offline version. Tested offline with stand-ins (SVG well-formed, files written).

## Advanced brush (mode 5)

Request: add a more complete brush tyre, built on the ideas of a report on another game's tyre model, but named "Advanced brush".
That report (a reverse-engineering dump of a proprietary executable) contained no numbers or code, only generic descriptions (a
friction coefficient that decays, a spring and damper, a Pacejka-like formula written wrongly, a heat equation, pressure scaling),
so nothing was taken from it and nothing from any binary: the mode is built from published tyre physics only.

Where: `tyre.lua`, in `stepWheel` (the brush) guarded by `P.advBrush`; `ui.lua` mode 5 (`modeOrder`, names and help, `currentParams`:
`useRbr102 = false`, `advBrush = true`; the brush settings are also enabled for mode 5, plus 17 `adv*` settings); `batch.lua` (a family
"Advanced brush"), `report.py` and the bench (`adv_brush`).

What it adds to the brush:
- Stiffness against load: `C(Fz) = coef * Fz0 * sin(2 atan(Fz / (shape * Fz0))) / sin(2 atan(1 / shape))`, so the stiffness per N is
  1 at the nominal load, 1.25 times that at zero load and 0.8 times at twice the load (shape 2). Offline: slope at 1 degree 999 N/deg
  at 2 kN, 1532 at 3.5 kN and 1842 at 5 kN (the plain brush: 846, 1469, 2080).
- Pressure: `(p / p_nom) ^ k` on the stiffness (k 0.6) and `1 - 0.4 (p / p_nom - 1)^2` on the grip, with `p` from
  `obj:getGroupPressure` of BeamNG's tyre (gauge, kPa; nominal 200).
- Temperature: one node, heat in = 0.15 x sliding friction power (force x slip speed, counted as the force saturates) + 0.012 W per
  N per m/s of rolling; out = (8 + 1.2 v) W/K x (T - 20); capacity 6000 J/K (a time constant of about 3 minutes). Grip factor
  `0.7 + 0.3 exp(-0.5 ((T - 75) / 35)^2)`. A light drive settles at about 45 C (grip 0.91), hard cornering at about 100 C.
- Wear: the sliding energy over `advWearJ` (1.2e7 J, about 30 minutes of limit cornering), times (1 + ((T - 110) / 25)^2) when overheated;
  grip falls by up to 25 % at wear 1.
- Telemetry: temperature and wear go to the live data table (`surfT`, `treadT`, `wear`).

Offline results (bench): at the nominal load the Advanced brush gives 0.97 of the brush's peak (the pressure of the bench's fake tyre
is 220 kPa against a nominal 200); a 6 degree steady-slip test holds about 77 C and wears 0.0088 per 5 s with the old default of
4e6 J; the 12 degree test heats to 88 C in two minutes. The first version started the tyre at the air temperature (20 C), which gave
59 % of the grip: the tyre now starts at `advTstart` (75 C).

The sweep CSV now also stores the slip velocities, the friction coefficient and the tyre temperature and wear per wheel
(`slip_vx`, `slip_vy`, `mu`, `treadT`, `wear`). Reason: on the test sled the plain brush gives only 0.35 of the load on average over
the ramp (peak 0.68 at 20 degrees; per wheel |Fy| 1350 N on 3900 N) while the offline bench gives 1.0 at 5 degrees: the model is right
and something in the game's coupling is not. The next brush and Advanced brush runs will show whether the slip velocity, the friction
coefficient or the load is at fault.

## Why the brush was weak on the sled, and the fix (brushRimTorque)

Advanced brush on the sled gave the same slow, near-linear curve as the plain brush (0.55 g at 20 degrees; 0.04 g per degree where RBR and
BeamNG's own tyre give 0.21). The per-wheel columns added to the CSV showed that the inputs were right (lateral slip speed 2.66 m/s
against 2.73 expected at 10 degrees, load a steady 3.7 kN, friction coefficient 0.88) and that the longitudinal slip speed was not: it
swung between +3 and -5 m/s on a free-rolling wheel. The spread of the slip ratio over the run (a new column in the report, "wheel
speed jitter"): BeamNG's own tyre 0.004, RBR on BeamNG's tyre 0.006, FULL RBR 0.000, brush 0.202 and Advanced brush 0.202 (75 % of the
samples above 0.1). With the wheel's speed ringing, the friction ellipse (the force follows the direction of the combined slip) was
mostly used by the longitudinal part and the lateral force was starved.

Cause (not proved in the game, but it fits everything): the brush pushed its longitudinal force into the tread nodes, which reach the
rim only through BeamNG's soft tyre carcass, and a stiff brush spring (about 760 kN/m at 3.8 kN) around that soft link rings at a few
hertz. The FULL modes put the force on the hub and the wheel-spin torque directly on the rim nodes, and are rock steady. The fix does
the same for the brush modes: `brushRimTorque` (default on; modes 0 and 5): the longitudinal force acts at the hub nodes, its torque
`-F * hub height` about `a = normal x forward` is applied as a pure couple on the rim nodes (`spinCouple`, the same construction as in
the FULL modes), and only the lateral force stays on the tread nodes. Off = the old path.

Offline (the bench with a free-spinning wheel, `--spin-inertia`, torque taken from all applied forces about the axle): both paths give
the same curves and a slip ratio of +0.002 at 10 degrees, down to 0.3 kg m^2 of wheel inertia, and the rim path is active (8 rim nodes
found). The offline plant has no soft carcass, so it cannot show the ringing: only the sled can. Next run: the "wheel speed jitter"
of the brush and the Advanced brush should fall from 0.20 towards 0.01, and the stiffness towards 0.2 g per degree.

## Aligning torque (steering feel) measured on the sled

`rig.lua` reports `Mz`: the yaw moment of the four tyres on the sled about its centre of mass, counter-clockwise seen from above
positive. The slip ramp turns the direction of travel counter-clockwise, so a restoring moment is positive. Newton for the rotation:
`Mz = Iz * (change of yaw rate / time) - servo moment`, over each 0.1 s report window. The servo moment is the yaw hold's couple plus the
servo force's offset from the centre of mass (the force acts at the centroid of the four force nodes). The yaw rate comes from the
angular momentum of the structural nodes about their centre of mass (differentiating the heading was too noisy at these torque levels);
`Iz` is the inertia of all the nodes. The result includes the small difference of the front and rear side forces, which is near zero on
this symmetric sled.

Checked offline with a rigid-body toy sled (13 nodes, a magic-formula tyre and a trail that falls from 35 mm to zero at 12 degrees,
once with the centre of mass 6 cm off the middle): the measured moment follows the true one within 2.3 N m over a 278 N m signal.

Plumbing: the sweep adds `rig_Mz` to the CSV and `lastResult.mz` (slip, Mz, |Fy|); `batch.lua` `analyseMz` gives peak, slip at the
peak, slope over the first 2 degrees (N m per degree), trail at 0.5 to 3 degrees (mm), value at 10 degrees and at the end, and the slip
angle where it changes sign; the report has a table and signed plots (`svgPlot` got a `ymin`), `summary.csv` seven more columns, the
in-game table two (peak and slope); `tools/report.py` the same. The values are the four tyres together, so a single tyre is a quarter
of them. A real car's steering feel also depends on the kingpin geometry and the game's force feedback settings, which the sled does
not have.
