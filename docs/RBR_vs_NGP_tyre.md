# rerbr tyre code: what is vanilla RBR 1.02 and what is NGP

Source: `rerbr/crates/rbr-physics` (`ports.csv`, `src/tyre/*.rs`, `src/ngp/*.rs`, `notes/ngp.md`), read 2026-10-25.

## Short answer

The **structure of the whole tyre model is vanilla RBR 1.02.** Every tyre function in `ports.csv` sits at an address
in the RBR 1.02 executable (`0x528980`-`0x5360c0`), not in `NGP.dll` (`0x10xxxxxx`). NGP is a plugin that patches the
1.02 exe in memory: it keeps the model and changes constants, a few formulas, the heat model and the tyre data.

rerbr ports each patched function **with NGP's behaviour only**, but its comments name the 1.02 value in most places.
So a "vanilla" version means rerbr's code with those NGP changes undone. The separate `rbr-recomp` repository (the
1.02 reconstruction) isn't public (github returns 404), so the comments are the only 1.02 reference.

## Vanilla RBR 1.02 (keep as is)

| Part | rerbr file |
| --- | --- |
| Step order (frame, patch, slip, soil, sticking/sliding, forces, heat, outputs) | `tyre/step.rs` |
| Contact frame and contact patch | `contact_frame.rs`, `contact_patch.rs` |
| Slip velocities, slip directions, sticking/sliding weights (1-2 m/s) | `slip.rs` |
| Low-speed hold torque | `hold.rs` |
| Soil drag (furrow on loose ground) | `soil.rs` |
| Sliding solver, tread/carcass solvers | `sliding.rs`, `solve.rs` |
| Friction circle and sticking limits, carcass deflection clamp | `grip.rs` |
| Friction curve and table lookups | `friction_curve.rs`, `locate.rs` |
| Surface friction blend | `surface.rs` |
| Rolling torque, force solve | `forces.rs` |
| Stiffness/friction tables by load, pressure, temperature (structure) | `step.rs`, `layout/` |

## NGP changes (undo or replace)

| NGP change | 1.02 value | Where |
| --- | --- | --- |
| Output scale 0.025 | 0.1 | `step.rs` `OUTPUT_SCALE` |
| Temperature grip scales static and **kinetic** friction | scales the static friction twice (a 1.02 quirk) | `step.rs` |
| Inclination (camber) scaling of grip and heat | none | `step.rs` calls `ngp::inclination` |
| Friction-curve falloff 0.95, 0.875, 0.775, 0.65 | 0.99, 0.96, 0.87, 0.7 | `patch_forces.rs` `FALLOFF` |
| Friction limits: end slip scale 1, lateral ×4.25 | peak slip × 3, capped at 1 | `friction_limits.rs`, `ngp/outputs.rs` |
| Hot-tread threshold 40 °C | 60 °C | `heat.rs` |
| Heat flows to air, carcass to inner air, sidewall convection | 1.02's `DamperStep1/2` kernels | `heat.rs`, `relax.rs`, `ngp/heat.rs` |
| Temperature relaxation rate per car | 100 per metre | `relax.rs` |
| Two-material tag blend weights | different weights | `ngp/tag_blend.rs` (1.02: `wheel/layout/tag_blend`) |
| Material sheet lookup 16×16 | slot's own size, first coordinate only | `ngp/material.rs` |
| x87 precision 53 bits | 24 bits | `fpu.rs` (irrelevant in Lua) |
| Tyre definition keys `*_NGP`, 7 tyre presets and temperatures read from `NGP.dll` | stock `CTyre::CDef` keys and the game's own tyre types | `data/ngp-lsp-keys.md`, `rbr-formats/src/ngp` |

## What this means for the mod

- Port the vanilla column with the 1.02 constants from the table, and leave out `ngp/` entirely: no NGP heat model,
  no inclination scaling, no NGP presets, no reading `NGP.dll`.
- Heat: use 1.02's simpler kernels (`spring_damper` shared kernels, `heat_flow`/`conduction`), or write a new
  simple model.
- Data: tyre tables still can't be copied from RBR's files into a public mod. Hand-tuned values with the same
  structure are fine.
- Some 1.02 details are only in comments ("1.02 scales the static one twice"). Where a comment is missing, the 1.02
  value can't be confirmed from rerbr alone.
- Legal note: vanilla avoids the NGP author's work, but RBR 1.02 itself is still copyrighted by its
  publisher. rerbr is a reverse-engineered port. A model "inspired by" RBR's structure with your own
  data and code is the safest route for a public release.
