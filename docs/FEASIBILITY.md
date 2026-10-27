# RBR (NGP) tyre model in BeamNG.drive — feasibility study

Date: 2026-10-25. Sources: the `JodusNodus/rerbr` repo (cloned, read), a BeamNG.drive install (buildbot 290897, vehicle Lua read directly), BeamNG docs and forums.

## Verdict

**Feasible, as a vehicle-Lua mod, without touching the game's C++ core.** The core's tyre friction can't be
replaced, but it can be switched off on the tread nodes and replaced by forces we compute ourselves every physics
step (2000 Hz). The RBR tyre is a good fit for that: one function per wheel per step, with clear inputs (contact
point, normal, axle, contact velocity, wheel spin, dt) and outputs (force, torque, reaction torque).

It won't be bit-exact RBR. BeamNG's soft-body tyre, suspension, ground mesh and 0.5 ms step all differ from RBR's
rigid-body car and 7 ms step. The goal is "RBR/NGP tyre feel inside BeamNG", not a clone of RBR's handling.

Main risks: stability at the handover between the sticking and sliding models, the tyre flex being counted twice
(BeamNG's soft body plus RBR's own carcass model), and where the tyre data comes from (licence).

## 1. What BeamNG lets a mod do (checked in the game's own Lua)

| Need | BeamNG mechanism | Evidence |
| --- | --- | --- |
| Run code every physics step | `onPhysicsStep(dtPhys)` → `extensionsHook("onPhysicsStep", dt)`, 2000 Hz; turned on by `enablePhysicsStepHook()` | `lua/vehicle/main.lua:61-84` |
| Turn off native grip on the tread | `obj:setNodeFrictionSlidingCoefs(cid, static, sliding)` at runtime; or set `frictionCoef`≈0 in a jbeam tyre part | `beamstate.lua:562` (used when a tyre bursts) |
| Apply our forces | `obj:applyForceVector(node, vec)`, `applyForce(n1, n2, f)`, `applyTorqueAxisCouple(...)` | `playerController.lua:163`, `combustionEngine.lua:804` |
| Wheel spin and drive torque | `wheels.wheelRotators[i].angularVelocity`, updated in the physics step before the powertrain | `main.lua:74-77` |
| Normal load, slip, contact material | `wheelObj:getSlipVelEnergyDownPeakForceDepthMats()` → slip, side slip, energy, down force, peak force, depth, material IDs | `wheels.lua:374` |
| Node positions, velocities, forces | `obj:getNodePosition`, `getNodeVelocityVector`, `getNodeForceVector`, `isNodeColliding` | many uses |
| Ground info | `obj:getSurfaceHeightBelow`, `castRayStatic`, ground-model material IDs | `wheels.lua`, `ai.lua` |

The native tyre (for reference) is per-node friction computed in the closed C++ core: each tread node has
static/sliding coefficients, a Stribeck curve (`stribeckExponent`, `stribeckVelMult`), load sensitivity
(`noLoadCoef`, `fullLoadCoef`, `loadSensitivitySlope`), `softnessCoef`, `treadCoef`, and thermal friction
(`setFrictionThermalSensitivity`), all set once at spawn (`jbeam/stage2.lua:213, 349`). Combined slip and the
friction circle are not formulas: they come out of the node friction and the tyre's structure.

Others have done this: the **Grip Dynamics** mod (v0.4.0, Dec 2025) computes Pacejka forces per wheel and writes the
result into tread-node friction, so this mod route works in practice.

## 2. The RBR/NGP tyre model (as ported in rerbr)

- Code: `crates/rbr-physics/src/tyre/` (≈2,300 lines of Rust, plus NGP heat and inclination in `src/ngp/`). It is
  a function-by-function port of RBR 1.02's `CTyre`/`CScratch::Step` with NGP 7.4.778's changes. Licence GPL-3.0.
- Per wheel, per step (`tyre/step.rs`):
  1. Air pressure → carcass stiffness factor (table).
  2. Contact frame and contact patch (`contact_frame`, `contact_patch`).
  3. Slip: `v_long = R·ω + s1`, spin slip, lateral/longitudinal velocities (`slip.rs`).
  4. **Two models blended:** a *sticking* model (carcass deflection and torsion, holds the car at rest) and a
     *sliding* model, weighted across 1-2 m/s of slip speed (`slip::weigh_models`, `sliding`, `hold`).
  5. Cornering/traction stiffness and friction from **tables by load**; NGP scales them by camber/inclination.
  6. **Soil drag** on loose surfaces: the tyre ploughs a furrow by sink depth, width and soil density (`soil.rs`).
     This is a big part of RBR's gravel/snow feel.
  7. Rubber friction from **tables by temperature** (surface 0.6, tread 0.4), wear by sector, water factor.
  8. Friction circle / sticking limits (`grip.rs`), forces and rolling-resistance torque (`forces.rs`).
  9. Heat step: tread, carcass, convection (`heat.rs`, `ngp::heat`).
- Surfaces: 50+ RBR materials (`Gravel_Fine_Deep_Dry`, `Tarmac_Fine_Sprinkled_Wet`, ...) with per-surface friction
  and soil tables.
- Physics step 7 ms (`MaxTimeStep 0.007`); BeamNG's is 0.5 ms, 14× more steps.
- The rerbr code is written to be bit-exact (x87 rounding emulation, memory-layout code). That part is useless in
  BeamNG and gets dropped. What stays is the physics.

## 3. Ways to do it

### A. Friction modulation (low effort, low fidelity)
Run the RBR model in Lua, then convert its force into one friction coefficient per wheel and write it to the tread
nodes (the Grip Dynamics method).
- Upside: BeamNG still does contact, flex and direction, so it's stable and quick to try.
- Downside: one number can't give separate lateral and longitudinal forces, the sticking/sliding blend, soil drag or
  aligning torque. It's BeamNG's tyre with RBR-flavoured grip levels, not the RBR tyre.

### B. Force injection (recommended)
1. A jbeam tyre part (unique name, added as a slot option, no vanilla files overwritten) or a runtime call sets the
   tread nodes' friction to ~0. The core still handles vertical contact, so bumps, impacts, punctures and the
   soft-body look still work.
2. A vehicle Lua extension runs at 2000 Hz, per wheel:
   - **inputs:** hub nodes `node1`/`node2` give the axle; ground normal from the contact or a ray;
     contact velocity = hub velocity + ω × r; ω from `wheelRotators`; Fz and material from
     `getSlipVelEnergyDownPeakForceDepthMats`.
   - **model:** a LuaJIT port of the rerbr tyre step, with its state (deflection, torsion, temperatures, wear) kept
     per wheel.
   - **outputs:** Fx/Fy spread over the tread nodes in contact (`applyForceVector`), so drive/brake torque reacts
     through the tyre on its own. Aligning and rolling torque go in as force pairs.
3. Ground models: a table maps BeamNG materials (ASPHALT, DIRT, GRAVEL, SAND, SNOW, ICE, MUD, ...) plus wetness to
   RBR surface classes and their soil parameters.
- Upside: the real RBR force model (stiffness by load, sticking/sliding, soil drag, heat, wear).
- Downside: the stability tuning and the side effects in section 4.

### C. Native code (not viable)
BeamNG.drive has no SDK for physics plugins. BeamNG.tech's co-simulation ("tight coupling") is a paid research
product and not available to `.zip` mods. Loading your own DLL through LuaJIT FFI isn't documented for mods, and
the mod repository wouldn't accept it. **Stay in pure Lua.**

## 4. Technical risks (B)

| Risk | Why | Mitigation |
| --- | --- | --- |
| Tyre flex counted twice | BeamNG's soft-body tyre already flexes sideways; RBR adds carcass deflection and relaxation on top | Stiffen the tread/sidewall beams in our tyre part, or scale down RBR's lateral carcass stiffness; tune against a steady-state skidpad test |
| Stability at 2000 Hz | Explicit sticking-model state (deflection, torsion) against a stiff soft-body structure; low-speed sticking can oscillate | Smaller dt actually helps; add damping on the deflection states; check with BeamNG's node-stability debug view |
| Getting Fz each step | The game reads `getSlipVel...` in `updateGFX`; it needs checking that it updates every physics step | Fallback: sum the normal component of `getNodeForceVector` on contact nodes, or estimate from contact depth |
| Native side effects | Tyre sounds, skid marks, particles, ABS/TC and AI read slip and friction from the core; at friction ~0 the core sees "no grip" | Feed our own slip signals: `setSlipTireMarkThreshold`, electrics values; re-check ABS/ESC and AI's `getStaticFrictionCoef` |
| CPU cost | 4 wheels × 2000 Hz = 8,000 tyre steps per second per car | LuaJIT, no allocation in the step (BeamNG's powertrain already runs at this rate in Lua); estimate 1-2% of a core per car |
| Surface coverage | RBR has ~50 tuned surfaces; BeamNG maps use about 15 ground models | Map table, extended per map when needed |

## 5. Data and licence

- **Code:** a Lua port of rerbr's tyre code is a derivative of GPL-3.0 code, so the mod must be GPL-3.0. Lua ships
  as source anyway, so that's easy. Credit rerbr.
- **Tyre data:** the parameter tables (stiffness/friction by load, temperature curves, surface tables, seven NGP
  tyre presets) come from RBR's game files and the player's `NGP.dll`. rerbr reads them from the player's own
  install and never ships them. A BeamNG mod can't ship them either. Options:
  1. **Your own parameter sets** (tarmac/gravel/snow compounds) using the RBR model's structure. Safest, and needed
     anyway because BeamNG cars aren't RBR cars.
  2. An optional **converter** that reads the player's own RBR/RSF install and writes tables into their mod folder.
- **Grey area:** rerbr comes from reverse-engineering RBR and NGP, and NGP's author doesn't share source. Releasing
  a port of NGP's behaviour on the BeamNG repo could cause friction with the RBR/RSF community. Contact the rerbr
  author and the RSF/NGP side before a public release.
- BeamNG modding rules (`NOTE_FOR_MODDERS.md`): don't edit the game folder, use unique names, don't overwrite
  vanilla parts. Both approaches above comply.

## 6. Suggested plan

| Phase | Content | Rough effort |
| --- | --- | --- |
| 0. Spike | One car: tread friction ~0, simple brush/linear tyre forces from Lua at 2000 Hz. Show it drives, stops, parks, and stays stable | 1-2 weeks |
| 1. Core port | rerbr slip, contact frame, load tables, sticking/sliding blend, friction circle, forces (no heat/soil) | 3-5 weeks |
| 2. RBR extras | Soil drag on loose surfaces, BeamNG ground-model mapping, heat, wear, pressure | 3-4 weeks |
| 3. Tuning and data | Your own compound tables, fix the doubled flex, ABS/TC/sound/skid marks, UI app with live tyre data, optional converter | ongoing |

Phase 0 answers the open questions (stability, Fz each step, side effects) cheaply before porting 2,300 lines.

## Sources
- rerbr: https://github.com/JodusNodus/rerbr (`crates/rbr-physics/src/tyre/`, `notes/physics.md`, `notes/ngp.md`, `data/ngp-lsp-keys.md`)
- BeamNG game Lua (local install): `lua/vehicle/main.lua`, `wheels.lua`, `beamstate.lua`, `jbeam/stage2.lua`
- Pressure wheels docs: https://documentation.beamng.com/modding/vehicle/sections/wheels/
- Tyre development blog: https://beamng.com/game/news/blog/a-look-at-tire-development-in-beamng-part-2
- Grip Dynamics mod: https://beamng.com/resources/grip-dynamics-mod.36559
- Physics rate: https://documentation.beamng.com/beamng_tech/architecture/
