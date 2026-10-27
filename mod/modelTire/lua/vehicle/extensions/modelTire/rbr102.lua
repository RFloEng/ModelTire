-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- Port of the Richard Burns Rally tyre model as published by rerbr: Copyright (C) the ReRBR authors (https://github.com/JodusNodus/rerbr).
-- modelTire_rbr102: Lua port of the Richard Burns Rally 1.02 tyre model (vanilla, no NGP parts).
-- Licence: GPL-3.0-or-later. Ported from rerbr (https://github.com/JodusNodus/rerbr), crates/rbr-physics/src/tyre/,
-- with NGP's changes undone using the 1.02 values rerbr's comments give (see RBR_vs_NGP_tyre.md).
--
-- Differences from the original, on purpose:
--  * The normal load comes from BeamNG's soft-body tyre (the 1.02 vertical solve is only used for the soil:
--    how far the tyre sinks into loose ground).
--  * No x87 rounding emulation; plain Lua doubles.
--  * The table axes, tyre data and surface data are our own (RBR's data files are not shipped).
--  * The tyre cooling ("relax") uses 1.02's plain heat-flow kernels with our own areas: the 1.02 versions
--    of those flows are not in rerbr (NGP replaces them).
--
-- Sign conventions are RBR's: the contact frame is rolling = normal x axle, lateral = rolling x normal, and
-- the spin is about the axle (right hand), so a wheel rolling along +rolling has spin = -v / R.

local M = {}

local abs, sqrt, min, max, cos, floor, pi = math.abs, math.sqrt, math.min, math.max, math.cos, math.floor, math.pi

-- 1.02 constants -------------------------------------------------------------------------------------------
-- friction curve falloff after the peak (1.02; NGP: 1, 0.95, 0.875, 0.775, 0.65, ...)
local FALLOFF = {[0] = 1.0, 0.99, 0.96, 0.87, 0.7, 0.5, 0.35, 0.2, 0.1, 0.05, 0.0, 0.0}
local END_SLIP_SCALE = 3.0      -- end slip = min(peak slip * 3, 1) (NGP: 1, lateral * 4.25)
local HOT_ABOVE = 60.0          -- degC (NGP: 40)
local ROAD_TEMP = 303.0         -- K, the heat step's cold side
local AMBIENT = 298.15          -- K, the relaxation's surroundings
local RELAX_RATE = 100.0        -- 1/m, carcass-to-tread gradient (NGP: per car)
local OUTPUT_SCALE = 0.1        -- sticking load output (NGP: 0.025); kept for telemetry
local INNER_AIR_CAPACITY = 742.85718

-- helpers ---------------------------------------------------------------------------------------------------
local function clamp(x, a, b) if x < a then return a elseif x > b then return b end return x end
local function sign(x) if x > 0 then return 1 elseif x < 0 then return -1 end return 0 end

-- table search: returns lo, hi (1-based) and the fraction, clamped at the ends
local function locate(axis, x)
  local n = #axis
  if x <= axis[1] then return 1, 2, 0 end
  if x >= axis[n] then return n - 1, n, 1 end
  for i = 1, n - 1 do
    if x < axis[i + 1] then
      return i, i + 1, (x - axis[i]) / (axis[i + 1] - axis[i])
    end
  end
  return n - 1, n, 1
end
local function lerp(tbl, lo, hi, f) return (1 - f) * tbl[lo] + tbl[hi] * f end

-- grip.rs: clamp_deflection
local function clampDeflection(a, b, c)
  local t = -b * c
  if a > 0 then
    if a < t then return (b > 0.001) and (-a / b) or 0
    elseif t < 0 then return 0 end
    return c
  else
    if a > t then return (b > 0.001) and (-a / b) or 0
    elseif t > 0 then return 0 end
    return c
  end
end

-- slip.rs: slip_directions
local function slipDirections(spin, lat, long, R)
  local c1 = spin * R + long
  local e = sqrt(c1 * c1 + lat * lat)
  if e > 0.001 then
    local a, b = abs(lat / e), abs(c1 / e)
    return a, b
  end
  return 1, 1
end

-- friction_curve.rs: table_lerp over FALLOFF with n = 10 segments
local function falloffLerp(x)
  local p = 10 * x
  local k = floor(p)
  if k < 0 then k = 0 elseif k > 10 then k = 10 end
  local f = p - k
  return (1 - f) * FALLOFF[k] + f * FALLOFF[k + 1]
end

-- friction_curve.rs: friction_curve -> friction coefficient, trail fraction, phase
local function frictionCurve(slip, peakSlip, endSlip, peakF, slideF, stiffness, load, trailLin, trailSlide)
  local linearSlip = peakF * load / stiffness * 1.2
  local knee
  if linearSlip > peakSlip then knee = linearSlip
  else
    local t = linearSlip * 3
    knee = (peakSlip > t) and t or peakSlip
  end
  if knee > slip then
    local q = slip / knee
    local r = (knee / (peakF * load)) * stiffness
    local p = (3 - r * 2) + (r - 2) * q
    return (p * q * q + q * r) * peakF, (1 - q) * trailLin + q * trailSlide, q
  elseif endSlip > slip then
    local s = (slip - knee) / (endSlip - knee)
    local f = falloffLerp(s)
    return (peakF - slideF) * f + slideF, f * trailSlide, s + 1
  end
  return slideF, 0, 2
end

local function ratio(a, c, c3)
  if c > a then return a / c
  elseif not (c3 > a) then return 2 end
  return (a - c) / (c3 - c) + 1
end

-- patch_forces.rs + friction_limits.rs (1.02 limits)
local function patchForces(s, T, x)
  local latShare, longShare = slipDirections(x.spin, x.vLat, x.vLong, x.R)
  local lo, hi, w = s.loadLo, s.loadHi, s.loadFrac
  local cornering = lerp(T.crnMyFct_L, lo, hi, w)
  local traction = lerp(T.trctMyFct_L, lo, hi, w)
  local crnSlip = lerp(T.slpPkCrn_L, lo, hi, w)
  local trctSlip = lerp(T.slpPkTrct_L, lo, hi, w)
  local staticScale = x.staticF * T.referenceFriction
  local kineticScale = x.kineticF * T.referenceFriction
  local kinetic = cornering * kineticScale -- both kinetic limits use the cornering factor (as the original)
  local latPeakLim = cornering * staticScale
  local latKin = kinetic * latShare
  local longPeakLim = traction * staticScale
  local longKin = kinetic * longShare
  local latPeakSlip, latEndSlip = crnSlip, min(crnSlip * END_SLIP_SCALE, 1)
  local longPeakSlip, longEndSlip = trctSlip, min(trctSlip * END_SLIP_SCALE, 1)

  -- lateral slip: v_lat / speed less the inclination's share, clamped to [-1, 1]
  local latSlip = clamp(x.vLat / x.speed - x.incSin * 0.1, -1, 1)
  local latNeg = latSlip < 0
  local latAbs = abs(latSlip)
  -- longitudinal slip ratio
  local y = 0
  if x.vLong > 0.001 or x.vLong < -0.001 then y = (-x.R * x.spin) / x.vLong - 1 end
  local longNeg = y < 0
  local longAbs = abs(y)
  local longSignV = (x.vLong < 0) and -1 or 1

  local latRatio = ratio(latAbs, latPeakSlip, latEndSlip)
  local longEnd = longNeg and 1 or longEndSlip
  local longRatio = ratio(longAbs, longPeakSlip, longEnd)
  local norm = sqrt(longRatio * longRatio + latRatio * latRatio)
  local latPkSlipNow, longPkSlipNow = latPeakSlip, longPeakSlip
  local latPeak, longPeak = latPeakLim, longPeakLim
  if norm > 0.01 then
    local longPart, latPart = longRatio / norm, latRatio / norm
    latPkSlipNow = latEndSlip - (latEndSlip - latPeakSlip) * latPart
    longPkSlipNow = longEndSlip - (longEndSlip - longPeakSlip) * longPart
    latPeak = max(latKin, latPart * latPeakLim)
    longPeak = max(longKin, longPart * longPeakLim)
  end

  local sinkW = clamp(x.sinkDepth * 33.333336, 0, 1)
  local trailLin = sinkW * -0.083333343 + 0.33333334
  local trailSlide = sinkW * -0.15 + 0.05

  local latFric, latTrail, latPhase = frictionCurve(latAbs, latPkSlipNow, latEndSlip, latPeak, latKin,
    x.corneringStiffness, x.load, trailLin, trailSlide)
  if latNeg then latFric = -latFric end
  local longFric, _, longPhase = frictionCurve(longAbs, longPkSlipNow, longEnd, longPeak, longKin,
    x.tractionStiffness, x.load, trailLin, trailSlide)
  if longNeg then longFric = -longFric end

  return -latFric * x.load, longFric * longSignV * x.load, x.patchLength * latTrail * 0.5, latPhase, longPhase
end

-- solve.rs: the soil contact solves -------------------------------------------------------------------------
local function quadRoot(t, q, y)
  return t * -0.5 - sqrt(max(0, t * t * 0.25 - (-1 / q) * y))
end

local solveCase
local function solveTreadOnly(c, it)
  local k = c.k
  local negTread = -k.treadStiffness
  local t = (k.linear - negTread) / k.quadratic
  local load = (c.penetration - c.soilThickness) * k.treadStiffness
  c.sink = c.soilThickness
  local d = quadRoot(t, k.quadratic, load)
  c.deflection = d
  local treadForce = negTread * d + load
  c.compression = (c.carcassRadius - c.hubHeight) - d
  c.treadForce = treadForce
  local tl, cc = treadForce > -1e-5, c.compression > 1e-5
  if (tl and not cc) or it > 3 then
    c.compression = 0; c.carcassForce = treadForce; c.treadEngaged, c.carcassEngaged = 1, 0
  else solveCase(c, tl, cc, it) end
end
local function solveBoth(c, it)
  local k = c.k
  local negTread, negGroove = -k.treadStiffness, -k.grooveSoil
  local t = ((k.linear - negTread) - negGroove) / k.quadratic
  local squash = c.carcassRadius - c.hubHeight
  local load = (c.penetration - c.soilThickness) * k.treadStiffness
  c.sink = c.soilThickness
  local d = quadRoot(t, k.quadratic, squash * k.grooveSoil + load)
  c.deflection = d
  c.compression = squash - d
  local treadForce = d * negTread + load
  c.treadForce = treadForce
  local tl, cc = treadForce > -1e-5, c.compression > -1e-5
  if (tl and cc) or it > 3 then
    c.carcassForce = (k.quadratic * d + k.linear) * d; c.treadEngaged, c.carcassEngaged = 1, 1
  else solveCase(c, tl, cc, it) end
end
local function solveNeither(c, it)
  local k = c.k
  local series = 1 / (1 / k.treadSoil + 1 / k.treadStiffness)
  local load = c.penetration * series
  local t = (k.linear + series) / k.quadratic
  local d = quadRoot(t, k.quadratic, load)
  c.deflection = d
  local treadForce = -series * d + load
  c.compression = (c.carcassRadius - c.hubHeight) - d
  c.treadForce = treadForce
  c.sink = c.penetration - d - treadForce / k.treadStiffness
  local tl, cc = c.sink - c.soilThickness > 1e-5, c.compression > 1e-5
  if (not tl and not cc) or it >= 4 then
    c.compression = 0; c.carcassForce = treadForce; c.treadEngaged, c.carcassEngaged = 0, 0
  else solveCase(c, tl, cc, it) end
end
local function solveCarcassOnly(c, it)
  local k = c.k
  local series = 1 / (1 / k.treadSoil + 1 / k.treadStiffness)
  local load = c.penetration * series
  local t = ((k.linear + series) + k.grooveSoil) / k.quadratic
  local squash = c.carcassRadius - c.hubHeight
  local d = quadRoot(t, k.quadratic, squash * k.grooveSoil + load)
  c.deflection = d
  c.compression = squash - d
  local treadForce = d * -series + load
  c.treadForce = treadForce
  c.sink = c.penetration - d - treadForce / k.treadStiffness
  local tl, cc = c.sink - c.soilThickness > 1e-5, c.compression > -1e-5
  if (not tl and cc) or it >= 4 then
    c.carcassForce = (k.quadratic * d + k.linear) * d; c.treadEngaged, c.carcassEngaged = 0, 1
  else solveCase(c, tl, cc, it) end
end
solveCase = function(c, treadLoaded, carcassCompressed, depth)
  local it = depth + 1
  if treadLoaded and carcassCompressed then solveBoth(c, it)
  elseif treadLoaded then solveTreadOnly(c, it)
  elseif carcassCompressed then solveCarcassOnly(c, it)
  else solveNeither(c, it) end
end

-- hold.rs: low-speed yaw hold. Directions are tables {x, y, z}.
local function hold(s, stickingW, speed, load, kineticF, L, Rl, N, dt)
  local d = s.holdDir
  if stickingW == 0 or s.active == 0 then
    d[1], d[2], d[3] = Rl[1], Rl[2], Rl[3]
    return 0
  end
  local dot = N[1] * d[1] + N[2] * d[2] + N[3] * d[3]
  local dx, dy, dz = d[1], d[2], d[3]
  if abs(dot) > 0.005 then
    dx, dy, dz = dx - N[1] * dot, dy - N[2] * dot, dz - N[3] * dot
    local len = sqrt(dx * dx + dy * dy + dz * dz)
    if not (len > 0.1) then
      d[1], d[2], d[3] = Rl[1], Rl[2], Rl[3]
      return 0
    end
    dx, dy, dz = dx / len, dy / len, dz / len
  end
  local localv = L[1] * dx + L[2] * dy + L[3] * dz
  local grip = clamp(load * kineticF * 2.2222222e-5, 0, 0.3) * stickingW
  local step = (clamp(abs(speed), 0.3, 100) - 0.3) * dt
  local moved = false
  if step > 0 and localv > 0 then
    moved = true
    localv = (step > localv) and 0 or (localv - step)
  end
  local rotate = moved
  if abs(localv) > grip then
    localv = sign(localv) * grip
    rotate = true
  end
  if rotate then
    local root = sqrt(max(0, 1 - localv * localv))
    d[1], d[2], d[3] = L[1] * localv + root * Rl[1], L[2] * localv + root * Rl[2], L[3] * localv + root * Rl[3]
  else
    d[1], d[2], d[3] = dx, dy, dz
  end
  return localv * -900
end

-- 1.02 heat kernels (spring_damper): heat_flow and conduction
local function heatFlow(hot, cold, coef, area) return (hot - cold) * coef * area end
local function conduction(gradient, k, area) return -(gradient * k * area) end

-- public ----------------------------------------------------------------------------------------------------

-- A new per-wheel state. radius: the BeamNG tyre's outer radius, m.
function M.newState(T, radius)
  local t0 = T.initTemperature or 303
  local s = {
    carcassRadius = radius - T.treadDepth,
    effRadius = radius,
    latDef = 0, torsion = 0, active = 0, gripClamped = 0,
    holdDir = {1, 0, 0},
    surfaceTemp = {t0, t0, t0, t0, t0, t0, t0, t0},
    treadTemp = t0, carcassTemp = t0, innerAirTemp = t0, innerAirTemp0 = t0,
    wear = {0, 0, 0, 0, 0, 0, 0, 0},
    treadEngaged = 1, carcassEngaged = 0,
    angle = 0, sector = 1, relaxSector = 1,
    loadLo = 1, loadHi = 2, loadFrac = 0,
    smoothedDrag = 0, smoothedDragWeight = 0, smoothedFurrow = 0,
    contact = {k = {}},
    -- outputs and telemetry
    Flat = 0, Flong = 0, Mz = 0, rollingTorque = 0, load = 0, slidingW = 0, slipPhase = 0,
    staticF = 0, kineticF = 0, tempGrip = 1, sinkDepth = 0, soilDrag = 0, trail = 0,
    vLat = 0, vLong = 0, spinSlip = 0,
  }
  return s
end

-- Cooling and heat exchange, one sector per call (1.02 steps a sector once in eight). Call every step.
local function relax(s, T, dt, speed)
  local R = s.carcassRadius
  local segArea = T.treadWidth * 2 * pi * R / 8
  local i = s.relaxSector
  s.relaxSector = (i % 8) + 1
  -- carcass <-> tread band, carcass -> inner air, sidewalls -> surroundings
  local bandArea = T.treadWidth * R * 6.2831855
  local band = conduction((s.carcassTemp - s.treadTemp) * RELAX_RATE, T.carcassConductivity, bandArea)
  local innerArea = 2 * pi * (R - T.sidewallHeight) * T.carcassWidth
  local toAir = heatFlow(s.carcassTemp, s.innerAirTemp, 30, innerArea)
  local sideArea = 2 * pi * (R * R - (R - T.sidewallHeight) ^ 2)
  local conv = (min(speed * 0.1, 1) * 0.5 + 1) * 20
  local toAmbient = heatFlow(s.carcassTemp, AMBIENT, conv, sideArea)
  local bandE, airE = band * dt, toAir * dt
  s.treadTemp = s.treadTemp + (-bandE) / (T.treadCapacity[1] * T.treadCapacity[2])
  s.carcassTemp = s.carcassTemp + (bandE - airE - toAmbient * dt) / (T.carcassCapacity[1] * T.carcassCapacity[2])
  s.innerAirTemp = s.innerAirTemp + airE / INNER_AIR_CAPACITY
  -- one sector's surface: loss to air, conduction to the tread under it, over 8 steps of time
  local elapsed = dt * 8
  local ts = s.surfaceTemp[i]
  local coef = (min(speed * 0.1, 1) * 0.5 + 1) * 30
  local loss = heatFlow(ts, AMBIENT, coef, segArea)
  local cond = conduction((s.treadTemp - ts) * 999.99994, T.treadConductivity, segArea)
  s.surfaceTemp[i] = ts + (-(cond + loss) * elapsed) / (T.surfaceCapacity[1] * T.surfaceCapacity[2])
  s.treadTemp = s.treadTemp + (cond * elapsed) / (T.treadCapacity[1] * T.treadCapacity[2])
end

local function noContact(s, airPressure)
  s.latDef, s.torsion, s.active = 0, 0, 0
  s.Flat, s.Flong, s.Mz, s.rollingTorque, s.load, s.normalForce = 0, 0, 0, 0, 0, 0
  s.slidingW, s.slipPhase = 0, 0
  s.inContact = false
end

-- One tyre step.
-- i: {N = {x,y,z} ground normal, A = {x,y,z} axle unit vector, V = {x,y,z} hub velocity (world),
--     spin (rad/s, RBR sign), hubHeight (m), Fz (N, from BeamNG), dt, airPressure (Pa gauge),
--     effRadius (m), contactAngle (rad, wheel rotation)}
-- T: tyre type table, mat: surface table. Results are left in s (Flat, Flong along s.L / s.Rl, Mz about s.N,
-- rollingTorque about the axle).
function M.step(s, T, mat, i)
  local dt = i.dt
  local N, A, V = i.N, i.A, i.V
  s.effRadius = i.effRadius or s.effRadius
  local R = s.effRadius
  local speedHub = sqrt(V[1] * V[1] + V[2] * V[2] + V[3] * V[3])
  relax(s, T, dt, speedHub)

  -- stiffness factor at the air pressure
  local pressure = i.airPressure
  if T.heatAffectsPressure then pressure = pressure * s.innerAirTemp / s.innerAirTemp0 end
  local plo, phi, pf = locate(T.pressureAxis, pressure)
  local sf = lerp(T.stfFct_P, plo, phi, pf)
  local latK = T.carcassStiffnessLateral * sf
  local torK = T.carcassStiffnessTorsional * sf

  -- contact frame
  local rx, ry, rz = N[2] * A[3] - N[3] * A[2], N[3] * A[1] - N[1] * A[3], N[1] * A[2] - N[2] * A[1]
  local rlen = sqrt(rx * rx + ry * ry + rz * rz)
  if not (rlen > 0.1) or (not i.computeLoad and i.Fz <= 0) then noContact(s); return end
  rx, ry, rz = rx / rlen, ry / rlen, rz / rlen
  local lx, ly, lz = ry * N[3] - rz * N[2], rz * N[1] - rx * N[3], rx * N[2] - ry * N[1]
  s.Rl = s.Rl or {0, 0, 0}; s.L = s.L or {0, 0, 0}; s.N = s.N or {0, 0, 1}
  local Rl, L, Nn = s.Rl, s.L, s.N
  Rl[1], Rl[2], Rl[3] = rx, ry, rz
  L[1], L[2], L[3] = lx, ly, lz
  Nn[1], Nn[2], Nn[3] = N[1], N[2], N[3]
  local incSin = -(N[1] * A[1] + N[2] * A[2] + N[3] * A[3])
  local incCos = lx * A[1] + ly * A[2] + lz * A[3]

  -- contact patch: sector, tread left, geometry
  local sector = (floor((i.contactAngle + 20 * pi) * 1.2732394933700562) % 8) + 1
  s.sector = sector
  local wear = s.wear[sector]
  local treadRem = (1 - wear) * T.treadDepth
  local defSq, sideSq = s.latDef * s.latDef, T.sidewallHeight * T.sidewallHeight
  local defRatio = (sideSq > defSq) and sqrt(defSq / sideSq) or 1
  local hub = i.hubHeight
  local camberRadius = (abs(incSin) > 1e-4) and (hub / incSin) or 1e7
  local outerR = treadRem + s.carcassRadius
  if i.computeLoad and outerR - hub < 1e-4 then noContact(s); return end -- 1.02: no contact under 0.1 mm
  -- computeLoad: the load comes from the penetration (the full tyre). Otherwise BeamNG says we are in
  -- contact, so keep a minimum patch.
  local pen = i.computeLoad and (outerR - hub) or max(outerR - hub, 0.002)
  local halfLen = sqrt(max(0, (2 * outerR - pen) * pen))
  local patchLength = halfLen * 2
  local contactArea = halfLen * T.carcassWidth * 2
  local treadArea = T.treadCoverage * contactArea
  local grooveArea = contactArea - treadArea

  -- surface friction: the material's coefficients times the tyre's class factor (BlendTable)
  local cls = mat.class
  local classF = (T.sFric[cls] + T.wetSlope[cls] * abs(T.wetnessDesign - mat.water)) * ((T.wearFtr[cls] - 1) * wear + 1)
  local staticF, kineticF = mat.static * classF, mat.kinetic * classF

  -- soil: how far the tyre sinks in (1.02 solve), blending soil and hard-ground friction
  local sink, compression = 0, 0
  local Fn = i.Fz -- the normal force the tyre carries (before the soil drag)
  local solveSoil = mat.sinkFactor > 1e-10 and mat.soilThickness > 0 and contactArea / mat.sinkFactor < 5e6
  if solveSoil or i.computeLoad then
    local pStiff = T.pressureRatio * pressure
    local chordSlope = pen / halfLen
    local chordLever = chordSlope * hub
    local quadShape = (chordLever - halfLen) / (pen * pen)
    local linShape = chordLever - quadShape * pen * 2
    local c = s.contact
    local k = c.k
    k.linear = linShape * pStiff * T.carcassWidth * 2 + T.carcassStiffnessVertical
    k.quadratic = quadShape * pStiff * T.carcassWidth * 2
    k.treadStiffness = T.treadElasticity * treadArea / (T.carcassThickness + treadRem)
    k.grooveSoil = grooveArea / max(mat.sinkFactor, 1e-12)
    k.treadSoil = treadArea / max(mat.sinkFactor, 1e-12)
    c.penetration, c.carcassRadius, c.hubHeight, c.soilThickness = pen, s.carcassRadius, hub, mat.soilThickness
    local quadOk = k.quadratic ~= 0 and k.quadratic == k.quadratic
    if solveSoil and quadOk then
      solveCase(c, s.treadEngaged ~= 0, s.carcassEngaged ~= 0, 0)
      s.treadEngaged, s.carcassEngaged = c.treadEngaged or 1, c.carcassEngaged or 0
      sink = clamp(c.sink or 0, 0, mat.soilThickness)
      compression = c.compression or 0
      if sink ~= sink then sink = 0 end
      if compression ~= compression then compression = 0 end
      if i.computeLoad then Fn = c.carcassForce end
    elseif i.computeLoad and quadOk then
      -- hard ground: the carcass and the tread in series (CContactPatch, no soil)
      local d = quadRoot((k.linear + k.treadStiffness) / k.quadratic, k.quadratic, k.treadStiffness * pen)
      Fn = (k.quadratic * d + k.linear) * d
      s.treadEngaged, s.carcassEngaged = 1, 0
    end
    if solveSoil then
      local t = sink / mat.soilThickness
      staticF = mat.staticSoil * classF * (1 - t) + staticF * t
      kineticF = mat.kineticSoil * classF * (1 - t) + kineticF * t
    end
  end
  s.sinkDepth = sink
  if i.computeLoad then
    if not (Fn > 0) or Fn ~= Fn then noContact(s); return end
    if i.maxLoad and Fn > i.maxLoad then Fn = i.maxLoad end
    s.normalForce = Fn
  end

  -- slip
  local vLat = V[1] * lx + V[2] * ly + V[3] * lz
  local vLong = V[1] * rx + V[2] * ry + V[3] * rz
  local speed = sqrt(vLat * vLat + vLong * vLong)
  local spin = i.spin
  local longSlipVel = R * spin + vLong
  local spinSlip = -vLong / R - spin
  s.vLat, s.vLong, s.spinSlip, s.longSlipVel = vLat, vLong, spinSlip, longSlipVel
  torK = torK * (clamp((speed - 10) * 0.1, 0, 1) * 0.9 + 0.1)

  -- soil drag (soil.rs); the normal load used for friction gains the drag, as in the original
  local Fz = Fn
  local latSoil, drag = 0, 0
  if mat.soilDensity > 0 and sink > 0 then
    local sideArea = patchLength * sink
    local lateralMassFlow = mat.soilDensity * abs(vLat) * sideArea
    local push = -(vLat * lateralMassFlow)
    local furrow = 1 - sideArea / (R * 2)
    local latShare = (1 - defRatio) * incCos
    local dragWeight = 1 - latShare
    latSoil = (1 - dragWeight) * push
    drag = abs(push) * dragWeight -- the longitudinal term is multiplied by 0 in the original
    s.smoothedDrag = s.smoothedDrag * 0.9 + drag * 0.1
    s.smoothedDragWeight = dragWeight * 0.1 + s.smoothedDragWeight * 0.9
    s.smoothedFurrow = s.smoothedFurrow * 0.9 + furrow * 0.1
  end
  s.soilDrag = drag
  local load = Fz + drag
  s.load = load
  local contactPressure = load / max(treadArea, 1e-6)
  s.cpLo, s.cpHi, s.cpFrac = locate(T.contactPressureAxis, contactPressure)
  s.loadLo, s.loadHi, s.loadFrac = locate(T.loadAxis, load)
  local corneringStiffness = lerp(T.crnStf_L, s.loadLo, s.loadHi, s.loadFrac) * sf
  local tractionStiffness = lerp(T.trctStf_L, s.loadLo, s.loadHi, s.loadFrac) * sf

  -- carcass deflection grows with the velocity along the axle, torsion with the spin slip
  local camberAngle = (vLong / camberRadius) * dt
  local neg = -((1 - cos(camberAngle)) * camberRadius)
  local axleVel = A[1] * V[1] + A[2] * V[2] + A[3] * V[3]
  s.latDef = s.latDef + axleVel * dt + (1 - incCos) * neg
  s.torsion = s.torsion - spinSlip * dt

  -- temperature grip: surface 0.6, tread 0.4; wet wear
  local surfT = s.surfaceTemp[sector]
  local alo, ahi, af = locate(T.temperatureAxis, surfT)
  local blo, bhi, bf = locate(T.temperatureAxis, s.treadTemp)
  local tempGrip = lerp(T.rubMyFct_T, blo, bhi, bf) * 0.4 + lerp(T.rubMyFct_T, alo, ahi, af) * 0.6
  if mat.water > 0 then
    tempGrip = tempGrip * ((((T.wetnessWearFtr - 1) * mat.water + 1) - 1) * wear + 1)
  end
  if T.quirk102 then
    staticF = staticF * tempGrip * tempGrip -- 1.02 scales the static friction twice and the kinetic not at all
  else
    staticF = staticF * tempGrip
    kineticF = kineticF * tempGrip
  end
  s.tempGrip, s.staticF, s.kineticF = tempGrip, staticF, kineticF

  -- weights of the sticking and the sliding model (1 to 2 m/s)
  local slidingW, stickingW
  if speed >= 2 then slidingW, stickingW = 1, 0
  elseif speed >= 1 then slidingW = speed - 1; stickingW = 1 - slidingW
  else slidingW, stickingW = 0, 1 end
  s.slidingW = slidingW

  -- sliding model
  local slideDef, slideTor, trail, latPhase, longPhase = 0, 0, 0, 0, 0
  if slidingW ~= 0 then
    local latF, longF
    latF, longF, trail, latPhase, longPhase = patchForces(s, T, {
      spin = spin, vLat = vLat, vLong = vLong, R = R, speed = speed, incSin = incSin,
      staticF = staticF, kineticF = kineticF, load = load,
      corneringStiffness = corneringStiffness, tractionStiffness = tractionStiffness,
      patchLength = patchLength, sinkDepth = sink,
    })
    slideDef = clampDeflection(latSoil + latF, incCos * latK, s.latDef)
    slideTor = clampDeflection(R * longF, torK, s.torsion)
  end

  -- sticking model: friction circle on the carcass forces
  local stickDef, stickTor, stickPhase = 0, 0, 0
  if stickingW > 0 then
    local limit = load * ((s.gripClamped ~= 0) and staticF or kineticF)
    local u = -s.latDef * latK
    local t = -1 / R * s.torsion * torK
    local fsq = t * t + u * u
    if not (fsq > limit * limit) then
      stickDef, stickTor, s.gripClamped = s.latDef, s.torsion, 0
    else
      local f = 0
      if fsq > 1e-5 and load > 0.001 then f = min(load * kineticF / sqrt(fsq), 1) end
      stickDef, stickTor, s.gripClamped, stickPhase = s.latDef * f, s.torsion * f, 1, 2
    end
  else
    s.gripClamped = 0
  end
  s.latDef = stickDef * stickingW + slideDef * slidingW
  s.torsion = stickTor * stickingW + slideTor * slidingW
  s.slipPhase = slidingW * max(latPhase, longPhase) + stickingW * stickPhase

  -- forces (forces.rs)
  local latForce = -s.latDef * latK * incCos
  local torsionTorque = -s.torsion * torK
  local longForce = torsionTorque / R
  if stickingW ~= 0 then
    if sign(latForce) == sign(vLat) then
      local a = abs(spinSlip)
      local v = clamp(a * a * -5 + 1, 0.6, 1)
      latForce = latForce * (v * stickingW + slidingW)
    end
    if sign(torsionTorque) ~= sign(spinSlip) then
      local a = abs(spinSlip)
      local y = 1 - (1 / (-1 - a * 10) + 1)
      local z = y * stickingW + slidingW
      longForce = longForce * z
      torsionTorque = torsionTorque * z
    end
    trail = trail * slidingW
    -- NOT 1.02: optional carcass damping while sticking (T.lowSpeedDamping, N*s/m; 0 = vanilla).
    -- The 1.02 sticking model is an undamped spring; this removes the standstill wobble.
    -- The damping is a fraction (T.lowSpeedZeta) of the critical damping of the carcass spring carrying
    -- this wheel's share of the car (mass = load / g), plus an optional fixed part (T.lowSpeedDamping).
    local zeta = T.lowSpeedZeta or 0
    local cFix = T.lowSpeedDamping or 0
    if zeta > 0 or cFix > 0 then
      local m = i.dampMass or (Fz / 9.81)
      local cLat = cFix + 2 * zeta * sqrt(max(latK * incCos, 0) * m)
      local cLong = cFix + 2 * zeta * sqrt(max(torK / (R * R), 0) * m)
      local axleVelNow = A[1] * V[1] + A[2] * V[2] + A[3] * V[3]
      latForce = latForce - cLat * axleVelNow * incCos * stickingW
      longForce = longForce + cLong * spinSlip * R * stickingW
      -- never more than the friction allows
      local fmax = 1.25 * max(staticF, kineticF) * load
      latForce, longForce = clamp(latForce, -fmax, fmax), clamp(longForce, -fmax, fmax)
    end
  end
  s.trail = trail
  local trailTorque = sign(vLong) * trail * latForce - slidingW * s.latDef * longForce
  local holdTorque = hold(s, stickingW, speed, load, kineticF, L, Rl, Nn, dt)
  s.Flat, s.Flong = latForce, longForce
  s.Mz = holdTorque + trailTorque
  s.torsionTorque = torsionTorque

  -- rolling resistance torque about the axle (only while moving: scaled by the sliding weight)
  s.rollingTorque = -(patchLength * mat.rollingResistance * 0.5 * load * slidingW * sign(spin))
  s.stickingLoad = Fz * OUTPUT_SCALE * stickingW

  -- heat step (1.02): frictional power into surface, tread, carcass; wear
  local water = mat.water
  local wetBoost = water * 3
  local wetTerm = wetBoost + 2
  local power = abs(s.rollingTorque * spin) * 0.5 + abs(longForce * longSlipVel) + abs(vLat * latForce)
  local heat = power * dt
  local near = heat * 0.7
  local deeper = heat - near
  local passedDown = near * 0.1
  local surfaceKept = near - passedDown
  local returned = passedDown * 0.2
  local carcassIn = deeper * 0.4
  local treadIn = (passedDown - returned) + deeper * 0.6
  local hot = clamp((((s.treadTemp + surfT) * 0.5) - 273.15 - HOT_ABOVE) * 0.0125, 0, 1)
  local wetDamping = 1 / (wetTerm / T.treadConductivity + 1)
  local conductance = lerp(T.contactConductance, s.cpLo, s.cpHi, s.cpFrac)
  local coefficient = conductance * (hot * 4500 + 1500) * (wetBoost + 1)
  local lossE = heatFlow(surfT, ROAD_TEMP, coefficient, treadArea) * dt
  local surfaceIn = ((hot * -0.6 + 1) * wetDamping * surfaceKept + returned) - lossE
  s.surfaceTemp[sector] = surfT + surfaceIn / (T.surfaceCapacity[1] * T.surfaceCapacity[2])
  s.treadTemp = s.treadTemp + treadIn / (T.treadCapacity[1] * T.treadCapacity[2])
  s.carcassTemp = s.carcassTemp + carcassIn / (T.carcassCapacity[1] * T.carcassCapacity[2])
  local slipSq = vLat * vLat + longSlipVel * longSlipVel
  if slipSq > 0.4 then
    local slipLen = sqrt(slipSq)
    local beyond = clamp(s.slipPhase - 1, 0, 1)
    local wearRate = lerp(T.rubWear_T, alo, ahi, af)
    local abrasion = contactPressure * beyond * wearRate
    s.wear[sector] = min(abrasion * slipLen * dt * 1.8e-8 * (T.wearScale or 1) + wear, 1)
  end

  s.active = 1
  s.inContact = true
end

M.locate = locate
M.frictionCurve = frictionCurve
M.clampDeflection = clampDeflection

return M
