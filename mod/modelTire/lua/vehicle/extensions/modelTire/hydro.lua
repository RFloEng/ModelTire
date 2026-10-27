-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- modelTire_hydro: the water layer of the "modded Pacejka" tyre. Licence: GPL-3.0-or-later.
--
-- The Magic Formula describes a tyre on the surface it was measured on. This adds one effect it does not have:
-- dynamic hydroplaning, where a wedge of water lifts part of the contact patch and grip and cornering stiffness
-- collapse with speed. The model is deliberately simple and every number is an engineering estimate, not a
-- measurement of a specific tyre:
--
--   * critical speed: the NASA/Horne spin-down relation, vp = 9 sqrt(p) knots with p the inflation pressure in psi
--     (about 26 m/s at 2.2 bar), raised by the drainage of the tread (groove volume x tread depth against the water
--     depth) and lowered for wide tyres;
--   * the part of the footprint on water starts at 55 % of vp and is complete at 115 % (smoothstep), scaled by the
--     water availability film / (film + 1 mm);
--   * the film builds and clears over a rolled distance (about 1 m), not a time, so the effect follows the car's speed;
--   * the friction scale falls from 1 to muFloor and the slip stiffness to 20 % as the footprint floats.
--
-- The baseline loss on a wet road (lower peak friction, longer slip at the peak) stays with the ground model; this layer
-- only adds the speed-dependent part. It does not touch the Pacejka coefficients, only the friction and stiffness scale
-- passed to the evaluation.

local M = {}

local max, min, sqrt, exp = math.max, math.min, math.sqrt, math.exp

local function clamp(x, a, b) if x < a then return a elseif x > b then return b end return x end
local function smooth(x) x = clamp(x, 0, 1); return x * x * (3 - 2 * x) end

-- water film (mm) a ground model stands for when the player has not set a depth
local groundFilm = {
  ASPHALT_WET = 0.6, GRAVEL_WET = 1.0, MUD = 1.5, WATER = 4.0,
}

function M.filmForGround(name) return groundFilm[name] or 0 end

function M.newState() return {f = 0, vp = 0, film = 0} end

-- i: speed (m/s along the ground), pressure (Pa, gauge), width (m), film (mm), tread (mm), void (0..0.6), muFloor
-- Returns the friction factor, the stiffness factor and the floating fraction f (0..1).
function M.step(s, i, dt)
  local film = max(i.film or 0, 0)
  s.film = film
  local psi = max((i.pressure or 200000) / 6894.76, 5)
  local vp0 = 9 * sqrt(psi) * 0.5144
  local capacity = clamp(i.void or 0.3, 0, 0.6) * max(i.tread or 7, 0)
  local drain = capacity / (capacity + film + 0.1)
  local width = clamp(i.width or 0.2, 0.1, 0.5)
  local vp = vp0 * (0.75 + 0.5 * drain) * (0.2 / width) ^ 0.25
  s.vp = vp
  local avail = film / (film + 1.0)
  local target = avail * smooth((max(i.speed, 0) / vp - 0.55) / 0.6)
  -- the film builds and clears over about one metre of rolling
  local tau = clamp(1.0 / max(i.speed, 1), 0.03, 0.5)
  s.f = s.f + (target - s.f) * (1 - exp(-dt / tau))
  if target == 0 and s.f < 1e-4 then s.f = 0 end
  local floor = clamp(i.muFloor or 0.08, 0.01, 0.5)
  return 1 - s.f * (1 - floor), 1 - 0.8 * s.f, s.f
end

return M
