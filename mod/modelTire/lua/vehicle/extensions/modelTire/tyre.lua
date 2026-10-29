-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- modelTire: Phase 0 test of a Lua tyre model in BeamNG.drive.
-- Licence: GPL-3.0-or-later (it contains a port of rerbr, which is GPL-3.0).
--
-- How it works: the tread nodes' native friction is set to ~0, so the core only
-- handles vertical contact. Every physics step (2000 Hz) each wheel runs a simple
-- tyre model and the result is applied as forces on the tread nodes in contact.
--
-- The model is a bristle/brush tyre with relaxation lengths:
--   dd/dt = s - (V / L) d        (d: carcass deflection, s: slip velocity)
--   F     = -(k d + c dd/dt)     (k = C / L, C = stiffness coefficient * Fz)
-- limited by a friction ellipse whose mu falls from a peak to a sliding value as
-- the slip speed rises (like RBR's sticking/sliding blend). At standstill it acts
-- as a damped spring, so the car can park on a slope without native friction.
--
-- Vehicle Lua name: modelTire_tyre. Controlled from the modelTire_ui window.

local M = {}

local abs, sqrt, exp, min, max = math.abs, math.sqrt, math.exp, math.min, math.max

-- Tunable parameters. The UI sends a full table with setParams().
local P = {
  mu = 1.10,             -- peak friction coefficient (dry tarmac)
  muSlide = 0.85,        -- friction coefficient when sliding fast
  stribeckVel = 2.5,     -- slip speed (m/s) over which mu falls from peak to sliding
  muLongScale = 1.05,    -- longitudinal mu / lateral mu (ellipse)
  loadSens = 0.12,       -- mu drop per unit of Fz / fzNominal above 1
  fzNominal = 3500,      -- N
  cornerCoef = 28,       -- cornering stiffness per N of load (N/rad per N): the force peaks at about 3 mu / cornerCoef rad
  longCoef = 30,         -- longitudinal slip stiffness per N of load: the force peaks at about 3 mu / longCoef
  -- Advanced brush (mode 5): the test tyre above plus load-dependent stiffness, pressure, temperature and wear. Built from
  -- published tyre physics only (see mod/modelTire/README.md); every number is a plain default to adjust.
  advBrush = false,
  advLoadShape = 2.0,    -- stiffness against load: C = coef * Fz0 * sin(2 atan(Fz / (shape Fz0))) / sin(2 atan(1 / shape)); a larger shape = closer to linear
  advPressureNom = 200,  -- kPa (gauge) where the stiffness and grip numbers apply; the pressure is read from BeamNG's tyre
  advPressureK = 0.6,    -- stiffness follows (p / p_nom) ^ k
  advPressureMu = 0.4,   -- grip lost per (p / p_nom - 1) ^ 2
  advTopt = 75,          -- deg C of best grip
  advTwidth = 35,        -- deg C: the gaussian width of the grip bump (grip is 61 % of the way to the floor this far from the optimum)
  advTfloor = 0.7,       -- grip factor far from the optimum (cold or overheated)
  advTamb = 20,          -- deg C: the air temperature
  advTstart = 75,        -- deg C: the temperature a tyre starts at (and returns to with Reset)
  advHeatCap = 6000,     -- J/K of the tread and carcass that heat up
  advHeatShare = 0.15,   -- fraction of the sliding friction power that heats the tyre (the rest goes into the road and the air)
  advRollHeat = 0.012,   -- rolling heat: W per N of load per m/s of wheel speed (about 45 C on a light drive)
  advCoolBase = 8,       -- W/K of cooling at rest
  advCoolSpeed = 1.2,    -- W/K of extra cooling per m/s of speed
  advTimeScale = 1,      -- heating and wear run this many times faster (to see them in a short test)
  advWearJ = 1.2e7,      -- J of sliding friction that wear the tread out (wear 1): about 30 minutes of limit cornering
  advWearLoss = 0.25,    -- grip lost at wear 1
  relaxLenY = 0.35,      -- lateral relaxation length, m
  relaxLenX = 0.15,      -- longitudinal relaxation length, m
  dampRatio = 0.5,       -- carcass damping, fraction of critical on a quarter car
  trail = 0.035,         -- pneumatic trail at zero lateral force, m
  crr = 0.012,           -- rolling resistance coefficient
  nativeFriction = 0.0,  -- native tread friction kept (0 = off, 1 = unchanged)
  slipFromTread = false, -- slip from tread node velocities instead of hub + wheel speed
  fzFallback = false,    -- use a fixed static load if the core reports no load
  patchBand = 0.012,     -- tread nodes within this height of the lowest are in the patch, m
  maxPatchNodes = 6,
  patchEverySteps = 4,   -- recompute patch nodes and ground normal every N physics steps
  minFz = 30,            -- N, below this the wheel counts as airborne
  debugDraw = true,
  brushRimTorque = true, -- brush modes: the longitudinal force goes to the hub and its wheel-spin torque to the rim nodes (as in the FULL modes), not into the tread
  fzFilterMs = 15,       -- ms: low-pass on the load BeamNG reports (its contact load is noisy; the grip limit must not follow it)
  lightModel = "rbr",    -- tyre on BeamNG's own tyre: "rbr" (RBR 1.02) or "svj" (Pacejka from an SVJ file)
  -- RBR 1.02 tyre port (rbr102.lua). When on, the brush parameters above are not used.
  useRbr102 = false,
  rbrTyre = 1,           -- 1 tarmac, 2 tarmac wet, 3 gravel, 4 snow (rbr102data.lua)
  rbrQuirk102 = true,    -- keep 1.02's temperature-grip quirk (static scaled twice, kinetic not)
  rbrGripScale = 0.95,   -- multiplies the surface friction (quick global tuning)
  rbrLatStiffScale = 1.0, -- multiplies the carcass lateral and torsional stiffness
  rbrLowSpeedDamping = 0,    -- N*s/m fixed carcass damping while sticking; NOT in 1.02 (0 = vanilla)
  rbrLowSpeedZeta = 0.25,    -- damping ratio of the sticking spring; NOT in 1.02 (0 = vanilla). Stops the standstill ringing
  rbrUseRbrData = true,  -- use your own RBR data (modelTire_rbrdata) when it is installed
  -- FULL RBR tyre: BeamNG's tyre ring does not touch the ground; RBR's own contact gives the load.
  fullTyre = false,
  fullSurface = "Tarmac_Fine_Clean_Dry", -- RBR surface name (own-data fallback maps it)
  fullVertDamping = 600,  -- N*s/m on the vertical contact (RBR has none; its suspension damper does the job)
  fullGroundEvery = 1,    -- ground height query every N physics steps
  fullLowSpeedVertZeta = 0.3, -- extra vertical damping ratio below ~4 m/s (0 = off)
  fullUnsprungMass = 30,  -- kg, mass used to size that damping
  fullSoilSoftness = 0.6, -- multiplies the soil's sink factor in the full RBR mode (higher = softer ground, sinks more)
  fullIgnoreCheck = false, -- skip the check that the tyre ring has no collision
  -- SVJ / Pacejka tyre (runs inside the full mode)
  fullModel = "rbr",          -- "rbr" (RBR 1.02 port), "svj" (Pacejka from an SVJ file) or "brush" (the Advanced brush, brushfull.lua)
  fullBrushKz = 200000,       -- FULL Advanced brush: vertical stiffness of the tyre, N/m
  fullBrushGrip = 1.0,        -- FULL Advanced brush: surface friction scale when the ground is not read from the map
  fullEnvelope = true,        -- enveloping contact: several ground samples along the wheel, so steps and kerbs are met smoothly
  fullEnvEvery = 2,           -- update the enveloping samples every N physics steps
  fullHeightTau = 1.5,        -- ms: smoothing of the hub height over the ground (0 = off)
  fullSlewKN = 4,             -- most the tyre force may change per millisecond, in kN (0 = off)
  fullMaxLoadG = 6,           -- the vertical load per wheel is capped at this many times the car's static wheel load
  fullFadeIn = 0.3,           -- seconds over which the tyre forces fade in after the tyre is switched on or the car reset
  svjFz0 = 0,                 -- nominal load of the Pacejka coefficients in N; 0 = from the file, else the car's static wheel load
  svjFz0Scale = 1.0,          -- multiplies that automatic nominal load (higher = the tyre acts as built for a heavier car)
  svjRelaxScale = 1.0,        -- multiplies the tyre's relaxation lengths (how fast the force builds up)
  svjModded = false,          -- 'modded Pacejka': adds the hydroplaning layer (hydro.lua) to the Pacejka tyre
  hydroWaterMm = 0,           -- water film in mm; 0 = from the ground model (wet asphalt 0.6 mm)
  hydroTreadMm = 7,           -- tread depth in mm
  hydroVoid = 0.3,            -- fraction of the tread that is open grooves
  hydroMuFloor = 0.08,        -- friction scale left when the contact is fully on water
  svjSurfaceShape = true,     -- the surface also changes the curve shape (stiffness, slip at the peak), not only the grip
  svjGripScale = 1.25,        -- extra grip scale applied on top of the detected surface
  svjGrip = 1.0,              -- surface friction scale (1 = the surface the coefficients were measured on)
  svjRadiusFromWheel = true,  -- measure the tyre deflection from the BeamNG wheel radius, not the SVJ tyre's
  svjUseCarPressure = false,  -- use the car's tyre pressure for the pressure terms (else the tyre's own)
  svjUseTyreDamping = true,   -- use the tyre's vertical damping from the file
  svjZeta = 0.25,             -- low-speed damping ratio of the contact spring (0 = none)
  mzScale = 1.0,              -- aligning torque (steering-wheel feedback) scale, all RBR/SVJ modes
  fullAutoSurface = true,     -- in full mode, take the surface from the terrain under the wheel when the map has one
  svjProgressive = true,      -- Pacejka vertical spring stiffens near the bump stop (else it stays linear)
  svjBumpDef = 0.04,          -- deflection (m) at which the progressive term equals the linear one
  rbrScaleToWheel = true,  -- fit the RBR tyre's width, stiffness and load tables to each BeamNG wheel
}

local rbr = require("extensions/modelTire/rbr102")
local rbrData = require("extensions/modelTire/rbr102data")
local pac = require("extensions/modelTire/pacejka")
local hydro = require("extensions/modelTire/hydro")
local brushfull = require("extensions/modelTire/brushfull")
local svjTyre = nil -- the SVJ tyre the player picked (made by the game-side reader), see setSvjTyre
local rbrTyreNames = {"tarmac", "tarmacWet", "gravel", "snow"}
local rbrTyreCache = {}
local tmpN, tmpA, tmpV = {0, 0, 1}, {0, 1, 0}, {0, 0, 0}
local rbrInput = {N = tmpN, A = tmpA, V = tmpV}

-- Grip multiplier by BeamNG ground model name. Rough first guesses.
local surfaceGrip = {
  ASPHALT = 1.0, ASPHALT_WET = 0.72, ASPHALT_PREPPED = 1.05, ASPHALT_OLD = 0.95,
  CONCRETE = 0.98, COBBLESTONE = 0.85, RUMBLE_STRIP = 0.95, ROCK = 0.85,
  DIRT = 0.72, DIRT_DUSTY = 0.68, GRAVEL = 0.68, GRASS = 0.55, MUD = 0.45, SAND = 0.55,
  SNOW = 0.42, ICE = 0.15, SLIPPERY = 0.08, FRICTIONLESS = 0.03, SNOWBANK = 0.25, GRAVEL_WET = 0.5, DIRT_DUSTY_LOOSE = 0.5, METAL = 0.6, WOOD = 0.7, PLASTIC = 0.6, KICKPLATE = 0.6,
}
local defaultSurfaceGrip = 0.8
-- How a surface changes the SHAPE of the Pacejka curves, not only their height. Measurements on loose and slippery
-- surfaces (TRB SR 115, Minnesota DOT friction studies, Pacejka's scaling-factor work) agree on this: the peak
-- friction is lower AND it is reached at a larger slip (about 18 % slip on dry asphalt, 20 % wet, about 30 % on snow
-- and ice for road tyres), i.e. the slip stiffness falls faster than the friction does. The value here is the factor
-- by which the slip at the peak is larger than on the surface the coefficients were measured on; the stiffness
-- scale is then lambda_mu / factor.
local surfacePeakSlip = {
  ASPHALT = 1.0, ASPHALT_PREPPED = 0.95, ASPHALT_OLD = 1.05, ASPHALT_WET = 1.15, CONCRETE = 1.0, COBBLESTONE = 1.15,
  RUMBLE_STRIP = 1.1, ROCK = 1.2, GRAVEL = 1.5, GRAVEL_WET = 1.6, DIRT = 1.5, DIRT_DUSTY = 1.6, DIRT_DUSTY_LOOSE = 1.7,
  GRASS = 1.6, MUD = 1.8, SAND = 1.8, SNOW = 1.8, SNOWBANK = 2.0, ICE = 1.7, SLIPPERY = 1.7, FRICTIONLESS = 1.7,
  METAL = 1.2, METAL_TREAD = 1.1, WOOD = 1.15, PLASTIC = 1.2, KICKPLATE = 1.3,
}
-- BeamNG maps and mods use many ground model names (DIRT_ROCKY, ASPHALT_ROUGH, GRAVEL_LOOSE ...). A name the tables do not
-- know is reduced to the nearest known one by keyword, so an unlisted ground never falls back to a default silently.
local knownGround = {}
for k in pairs(surfaceGrip) do knownGround[k] = true end
for k in pairs(surfacePeakSlip) do knownGround[k] = true end
local groundKeywords = {
  {"ICE", "ICE"}, {"SNOW", "SNOW"}, {"MUD", "MUD"}, {"SAND", "SAND"}, {"GRASS", "GRASS"}, {"LEAV", "GRASS"},
  {"GRAVEL", "GRAVEL"}, {"DIRT", "DIRT"}, {"ROCK", "ROCK"}, {"STONE", "COBBLESTONE"}, {"CONCRETE", "CONCRETE"},
  {"METAL", "METAL"}, {"WOOD", "WOOD"}, {"ASPHALT", "ASPHALT"}, {"TARMAC", "ASPHALT"}, {"ROAD", "ASPHALT"},
}
local canonCache = {}
local function canonGround(name)
  if not name then return nil end
  if knownGround[name] then return name end
  local c = canonCache[name]
  if c ~= nil then return c or nil end
  local up = string.upper(name)
  c = false
  for _, kw in ipairs(groundKeywords) do
    if up:find(kw[1], 1, true) then c = kw[2]; break end
  end
  if c == "ASPHALT" and up:find("WET", 1, true) then c = "ASPHALT_WET" end
  canonCache[name] = c
  return c or nil
end

-- peak-slip factor for a surface that has no entry: a smooth fit through the table (mu 1 -> 1, mu 0.3 -> 1.5, mu 0.1 -> 2)
local function clampPS(lam)
  local f = math.max(lam, 0.05) ^ -0.35
  return f < 1 and 1 or (f > 2.1 and 2.1 or f)
end

-- the stiffness scale to use with a friction scale lam on the surface called name (nil = the old sqrt rule)
local function gripStiffness(name, lam)
  if not P.svjSurfaceShape then return nil end
  local ps = name and surfacePeakSlip[canonGround(name) or name]
  if not ps then ps = clampPS(lam) end
  return lam / ps
end
local RUBBER_ID = 4

local enabled = false
local fullIsOn = false   -- the full tyre is running (needs fullTyre and a ring without ground collision)
local fullState = "off"  -- "off", "needs reload" or "on"
local refreshFull        -- set below
local initDone = false
local W = {}          -- per wheel state
local materialNames = {}
local stepCounter = 0
local timer = nil
local timeAccum, timeSteps = 0, 0
local telemetryTimer = 0
local lastStepUs = 0
local staticFz = 3000
local g = 9.81

local function vlen(x, y, z) return sqrt(x * x + y * y + z * z) end
local function clamp(x, a, b) if x < a then return a elseif x > b then return b end return x end

local function buildMaterialNames()
  materialNames = {}
  if particles and particles.getMaterialsParticlesTable then
    local mats = particles.getMaterialsParticlesTable()
    for id, m in pairs(mats or {}) do
      if type(m) == "table" and m.name then materialNames[id] = m.name end
    end
  end
end

local function surfaceForMaterials(m1, m2)
  local id = m1
  if id == RUBBER_ID or id == nil or id < 0 then id = m2 end
  local name = materialNames[id]
  local grip = (name and (surfaceGrip[name] or surfaceGrip[canonGround(name) or ""])) or defaultSurfaceGrip
  return grip, name or tostring(id)
end

local function initWheels()
  W = {}
  buildMaterialNames()
  local count = 0
  for _, wd in pairs(wheels.wheelRotators) do
    if wd.rotatorType == "wheel" and wd.hasTire and wd.treadNodes and #wd.treadNodes > 0 and wd.obj then
      local s = {
        wd = wd, name = wd.name, n1 = wd.node1, n2 = wd.node2,
        tread = {}, origFriction = {},
        dx = 0, dy = 0,                 -- carcass deflection (m), contact frame
        patch = {}, patchX = {}, patchCount = 0, sumX2 = 0,
        nx = 0, ny = 0, nz = 1,         -- ground normal
        fx_ = 1, fy_ = 0, fz_ = 0,      -- forward axis
        lx_ = 0, ly_ = 1, lz_ = 0,      -- lateral axis
        cx = 0, cy = 0, cz = 0,         -- patch centre (relative to vehicle ref)
        -- telemetry
        Fz = 0, Fx = 0, Fy = 0, sx = 0, sy = 0, muEff = 0, sliding = false, surface = "", grip = 1,
        -- RBR model
        radius = wd.radius or 0.3, hubHeight = wd.radius or 0.3, topNode = nil, rbrState = nil, rbrTyreKey = nil,
        tireWidth = wd.tireWidth, hubRadius = wd.hubRadius,
        pressureGroupId = (wd.pressureGroup and v.data.pressureGroups) and v.data.pressureGroups[wd.pressureGroup] or nil,
        wheelAngle = 0,
      }
      for i, cid in ipairs(wd.treadNodes) do
        s.tread[i] = cid
        local nd = v.data.nodes[cid]
        local fc = (nd and type(nd.frictionCoef) == "number") and nd.frictionCoef or 1
        local sfc = (nd and type(nd.slidingFrictionCoef) == "number") and nd.slidingFrictionCoef or fc
        s.origFriction[cid] = {fc, sfc}
      end
      count = count + 1
      W[count] = s
    end
  end
  -- static load per wheel for the fallback: total mass * g / wheels
  local mass = 0
  for _, nd in pairs(v.data.nodes) do mass = mass + (obj:getNodeMass(nd.cid) or 0) end
  staticFz = (count > 0) and (mass * g / count) or 3000
  initDone = true
  return count
end

local function setNativeFriction(scale)
  for _, s in ipairs(W) do
    for _, cid in ipairs(s.tread) do
      local o = s.origFriction[cid]
      obj:setNodeFrictionSlidingCoefs(cid, o[1] * scale, o[2] * scale)
    end
  end
end

-- Find the tread nodes in the contact patch and the ground normal.
local function updatePatch(s, refPos)
  local p1 = obj:getNodePosition(s.n1)
  local p2 = obj:getNodePosition(s.n2)
  local hx, hy, hz = (p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5

  -- ground normal below the wheel (world frame); fall back to the car's up vector
  local nx, ny, nz
  if mapmgr and mapmgr.surfaceNormalBelow then
    local n = mapmgr.surfaceNormalBelow(vec3(hx + refPos.x, hy + refPos.y, hz + refPos.z), 0.3)
    if n and n.z == n.z and (n.x * n.x + n.y * n.y + n.z * n.z) > 0.5 then nx, ny, nz = n.x, n.y, n.z end
  end
  if not nx then
    local up = obj:getDirectionVectorUp()
    nx, ny, nz = up.x, up.y, up.z
  end
  s.nx, s.ny, s.nz = nx, ny, nz

  -- contact frame: lateral = axle projected on the ground, forward = lateral x normal
  local ax, ay, az = p2.x - p1.x, p2.y - p1.y, p2.z - p1.z
  local an = ax * nx + ay * ny + az * nz
  local lx, ly, lz = ax - an * nx, ay - an * ny, az - an * nz
  local ll = vlen(lx, ly, lz)
  if ll < 1e-6 then return end
  lx, ly, lz = lx / ll, ly / ll, lz / ll
  local fx, fy, fz = ly * nz - lz * ny, lz * nx - lx * nz, lx * ny - ly * nx
  local vf = obj:getDirectionVector()
  if fx * vf.x + fy * vf.y + fz * vf.z < 0 then fx, fy, fz = -fx, -fy, -fz end
  -- re-derive lateral so (f, l, n) is right-handed: l = n x f
  lx, ly, lz = ny * fz - nz * fy, nz * fx - nx * fz, nx * fy - ny * fx
  s.fx_, s.fy_, s.fz_ = fx, fy, fz
  s.lx_, s.ly_, s.lz_ = lx, ly, lz

  -- lowest tread nodes along the normal
  local tread = s.tread
  local nT = #tread
  local heights = s.heights or {}
  local positions = s.positions or {}
  s.heights, s.positions = heights, positions
  local hmin = math.huge
  for i = 1, nT do
    local p = obj:getNodePosition(tread[i])
    positions[i] = p
    local h = (p.x - hx) * nx + (p.y - hy) * ny + (p.z - hz) * nz
    heights[i] = h
    if h < hmin then hmin = h end
  end
  local patch, patchX = s.patch, s.patchX
  local count = 0
  local limit = hmin + P.patchBand
  local cx, cy, cz = 0, 0, 0
  for i = 1, nT do
    if heights[i] <= limit and count < P.maxPatchNodes then
      count = count + 1
      patch[count] = tread[i]
      local p = positions[i]
      patchX[count] = p.x * fx + p.y * fy + p.z * fz -- position along forward, made relative below
      cx, cy, cz = cx + p.x, cy + p.y, cz + p.z
    end
  end
  s.patchCount = count
  s.hubHeight = -hmin
  if count > 0 then
    local nd = v.data.nodes[patch[1]]
    s.topNode = nd and nd.oppositeTreadNodeCid or nil
    cx, cy, cz = cx / count, cy / count, cz / count
    s.cx, s.cy, s.cz = cx, cy, cz
    local cf = cx * fx + cy * fy + cz * fz
    local sumX2 = 0
    for i = 1, count do
      local x = patchX[i] - cf
      patchX[i] = x
      sumX2 = sumX2 + x * x
    end
    s.sumX2 = sumX2
  end
end

-- RBR 1.02 tyre on one wheel --------------------------------------------------------------------------------

local function usingRbrData() return P.rbrUseRbrData and rbrData.rbr ~= nil end

-- Fits an RBR tyre type to a BeamNG wheel: the width, stiffness and load tables scale with the tyre width
-- (a tyre k times wider carries k times the load at the same pressure, so it behaves like the RBR tyre at
-- 1/k of the load), the sidewall height and the heat masses follow the wheel's size.
local scaledCache = {}
local function scaledForWheel(T, key, s)
  local w = s and s.tireWidth
  if not (P.rbrScaleToWheel and w and w > 0.05) then return T, 1 end
  local ck = key .. "|" .. tostring(T) .. "|" .. string.format("%.4f|%.4f|%.4f", w, s.radius, s.hubRadius or 0)
  local S = scaledCache[ck]
  if S then return S, S.scaleK end
  local k = w / T.carcassWidth
  local kr = s.radius / ((T.carcassRadius or 0.305) + T.treadDepth)
  S = {}
  for name, val in pairs(T) do S[name] = val end
  S.scaleK = k
  S.carcassWidth = w
  S.treadWidth = T.treadWidth * k
  for _, name in ipairs({"carcassStiffnessLateral", "carcassStiffnessTorsional", "carcassStiffnessVertical"}) do
    S[name] = T[name] * k
  end
  for _, name in ipairs({"loadAxis", "crnStf_L", "trctStf_L"}) do
    S[name] = {}
    for i, val in ipairs(T[name]) do S[name][i] = val * k end
  end
  if s.hubRadius and s.hubRadius > 0.05 and s.radius - s.hubRadius > 0.02 then
    S.sidewallHeight = s.radius - s.hubRadius
  end
  for _, name in ipairs({"surfaceCapacity", "treadCapacity", "carcassCapacity"}) do
    S[name] = {T[name][1] * k * kr, T[name][2]}
  end
  scaledCache[ck] = S
  return S, k
end

-- Tyres the player made in rbrtyres/*.json: {name, base = name of the tyre it starts from, set = {field = value}}
local customTyres = {}

local function baseTyreNames()
  if usingRbrData() then return rbrData.rbr.names end
  return rbrTyreNames
end

local function baseTyre(name)
  if usingRbrData() then
    for i, n in ipairs(rbrData.rbr.names) do
      if n == name then return rbrData.rbr.tyres[i], "rbr:" .. n end
    end
    return rbrData.rbr.tyres[1], "rbr:" .. rbrData.rbr.names[1]
  end
  local b = rbrData.tyres[name] and name or rbrTyreNames[1]
  return rbrData.tyres[b], b
end

-- every tyre the player can pick: the RBR (or built-in) tyres, then the custom ones
local function tyreChoices()
  local list = {}
  for _, n in ipairs(baseTyreNames()) do list[#list + 1] = n end
  for _, c in ipairs(customTyres) do list[#list + 1] = c.name .. " (custom)" end
  return list
end

local function rbrTyreType(s)
  local key, base
  local nBase = #baseTyreNames()
  local idx = math.max(1, math.min(P.rbrTyre, nBase + #customTyres))
  if idx > nBase then
    local c = customTyres[idx - nBase]
    local b, bkey = baseTyre(c.base)
    base = {}
    for k, val in pairs(b) do base[k] = val end
    for k, val in pairs(c.set) do base[k] = val end
    key = "custom:" .. c.name .. ":" .. bkey
  elseif usingRbrData() then
    key, base = "rbr:" .. rbrData.rbr.names[idx], rbrData.rbr.tyres[idx]
  else
    key = rbrTyreNames[idx]
    base = rbrData.tyres[key]
  end
  local cacheKey = key .. (P.rbrQuirk102 and "q" or "n") .. P.rbrLatStiffScale .. "_" .. P.rbrLowSpeedDamping .. "_" .. P.rbrLowSpeedZeta
  local T = rbrTyreCache[cacheKey]
  if not T then
    T = {}
    for k, val in pairs(base) do T[k] = val end
    T.quirk102 = P.rbrQuirk102
    T.carcassStiffnessLateral = base.carcassStiffnessLateral * P.rbrLatStiffScale
    T.carcassStiffnessTorsional = base.carcassStiffnessTorsional * P.rbrLatStiffScale
    T.lowSpeedDamping = P.rbrLowSpeedDamping
    T.lowSpeedZeta = P.rbrLowSpeedZeta
    rbrTyreCache[cacheKey] = T
  end
  local scaled = scaledForWheel(T, key, s)
  return scaled, key
end

local surfaceCache = {}
local function rbrSurface(m1, m2)
  local id = m1
  if id == RUBBER_ID or id == nil or id < 0 then id = m2 end
  local name = materialNames[id] or tostring(id)
  local useRbr = usingRbrData()
  local key = (useRbr and "rbr:" or "") .. name .. P.rbrGripScale
  local mat = surfaceCache[key]
  if not mat then
    local base
    if useRbr then
      local set = rbrData.rbr
      local rbrName = set.surfaceMap[name]
      base = (rbrName and set.surfaces[rbrName]) or set.defaultSurface
      surfaceCache[name .. ":label"] = name .. " -> " .. (rbrName or "Tarmac_Fine_Clean_Dry")
    else
      base = rbrData.surfaces[name] or rbrData.defaultSurface
    end
    mat = {}
    for k, val in pairs(base) do mat[k] = val end
    for _, k in ipairs({"static", "kinetic", "staticSoil", "kineticSoil"}) do mat[k] = base[k] * P.rbrGripScale end
    surfaceCache[key] = mat
  end
  return mat, (useRbr and surfaceCache[name .. ":label"]) or name
end

-- BeamNG's contact load is noisy, and at low speed the tyre nodes bounce, so the reported load flickers between zero
-- and a spike. A bristle model must not follow that: a friction limit that dips clips the bristles, and a step with no
-- load drops the deflection (the car then slides). The load is low-passed, and a gap in contact is part of the same
-- low-pass (the grip fades over the filter time instead of vanishing). Returns the smoothed load, or nil when the
-- wheel is really off the ground.
local function smoothLoad(s, raw, dt)
  if P.fzFilterMs <= 0 then
    if raw < P.minFz then s.fzF = nil; return nil end
    s.fzF = raw
    return raw
  end
  local f = s.fzF
  if raw < P.minFz then
    if not f then return nil end
    raw = 0
  elseif not f then
    s.fzF = raw
    return raw
  end
  f = f + (raw - f) * (1 - exp(-dt / (P.fzFilterMs * 0.001)))
  if f < P.minFz then s.fzF = nil; return nil end
  s.fzF = f
  return f
end

-- Modded Pacejka: the water layer scales the friction and the stiffness the Pacejka step is given. Called with the
-- wheel's inputs filled in (grip, gripK, airPressure, V, N); name is the ground model under the wheel (or nil).
local function applyHydro(s, inp, dt, name)
  if not P.svjModded then s.hydroF = nil; return end
  local st = s.hydro
  if not st then st = hydro.newState(); s.hydro = st end
  local V, N = inp.V, inp.N
  local vn = V[1] * N[1] + V[2] * N[2] + V[3] * N[3]
  local vx, vy, vz = V[1] - vn * N[1], V[2] - vn * N[2], V[3] - vn * N[3]
  local film = P.hydroWaterMm > 0 and P.hydroWaterMm or hydro.filmForGround(name)
  local kBase = inp.gripK or sqrt(inp.grip)
  local mf, kf, f = hydro.step(st, {
    speed = sqrt(vx * vx + vy * vy + vz * vz), pressure = inp.airPressure, width = s.tireWidth,
    film = film, tread = P.hydroTreadMm, void = P.hydroVoid, muFloor = P.hydroMuFloor,
  }, dt)
  inp.grip = inp.grip * mf
  inp.gripK = kBase * kf
  s.hydroF, s.hydroVp, s.hydroFilm = f, st.vp, film
end

-- Inputs of the Pacejka step that do not depend on the mode (surface grip is set by the caller)
local function fillPacInput(s, inp, pressure)
  inp.freeRadius = P.svjRadiusFromWheel and s.radius or nil
  inp.pressure = P.svjUseCarPressure and pressure or nil
  inp.zeta = P.svjZeta
  local fz0 = nil
  if P.svjFz0 > 0 then fz0 = P.svjFz0
  elseif svjTyre.noFz0 then fz0 = (s.staticLoad or staticFz) * P.svjFz0Scale
  elseif P.svjFz0Scale ~= 1 then fz0 = svjTyre.Fz0 * P.svjFz0Scale end
  inp.Fz0, inp.relaxScale = fz0, P.svjRelaxScale
  inp.Re = P.svjRadiusFromWheel and s.radius or nil
  inp.dampMass = (s.staticLoad or staticFz) / 9.81
  s.fz0used = fz0 or svjTyre.Fz0
  inp.progressive, inp.bumpDef = P.svjProgressive, P.svjBumpDef
end

local function stepWheelRbr(s, dt, refPos, doPatch)
  local wd = s.wd
  if wd.isBroken or wd.isTireDeflated then
    s.Fz, s.Fx, s.Fy = 0, 0, 0
    return
  end
  if doPatch or s.patchCount == 0 then updatePatch(s, refPos) end

  local usePac = P.lightModel == "svj" and svjTyre ~= nil
  local T, key, st
  if usePac then
    if not s.pacState or s.pacTyre ~= svjTyre then
      s.pacState = pac.newState(svjTyre, s.radius)
      s.pacTyre = svjTyre
    end
    st = s.pacState
  else
    T, key = rbrTyreType(s)
    if not s.rbrState or s.rbrTyreKey ~= key then
      s.rbrState = rbr.newState(T, s.radius)
      s.rbrTyreKey = key
    end
    st = s.rbrState
  end

  local _, _, _, downForce, _, _, m1, m2 = wd.obj:getSlipVelEnergyDownPeakForceDepthMats()
  local Fz = downForce or 0
  if P.fzFallback and Fz <= 0 and obj:isNodeColliding(s.patch[1] or s.tread[1]) then Fz = staticFz end
  if Fz < P.minFz or s.patchCount == 0 then Fz = 0 end
  s.Fz = Fz
  local mat, surfaceName
  if usePac then
    local g
    g, surfaceName = surfaceForMaterials(m1, m2)
    s.pacGrip = g * P.svjGripScale
  else
    mat, surfaceName = rbrSurface(m1, m2)
  end
  s.surface = surfaceName
  -- BeamNG's contact load flickers (the tyre nodes bounce): both tyres run on the same smoothed load as the brush tyre. An
  -- unsmoothed load switched the RBR tyre off in every step without contact and cost it a large part of its average force.
  Fz = smoothLoad(s, Fz, dt) or 0

  -- axle, flipped so that RBR's rolling axis (normal x axle) points forward
  local p1 = obj:getNodePosition(s.n1)
  local p2 = obj:getNodePosition(s.n2)
  local ax, ay, az = p2.x - p1.x, p2.y - p1.y, p2.z - p1.z
  local al = vlen(ax, ay, az)
  if al < 1e-6 then return end
  ax, ay, az = ax / al, ay / al, az / al
  local nx, ny, nz = s.nx, s.ny, s.nz
  local rx, ry, rz = ny * az - nz * ay, nz * ax - nx * az, nx * ay - ny * ax
  if rx * s.fx_ + ry * s.fy_ + rz * s.fz_ < 0 then ax, ay, az = -ax, -ay, -az end
  tmpN[1], tmpN[2], tmpN[3] = nx, ny, nz
  tmpA[1], tmpA[2], tmpA[3] = ax, ay, az
  local v1 = obj:getNodeVelocityVector(s.n1)
  local v2 = obj:getNodeVelocityVector(s.n2)
  tmpV[1], tmpV[2], tmpV[3] = (v1.x + v2.x) * 0.5, (v1.y + v2.y) * 0.5, (v1.z + v2.z) * 0.5

  -- spin in RBR's sign: rolling forward is negative spin about the forward-flipped axle
  local spin = -((wd.angularVelocity or 0) * (wd.wheelDir or 1))
  s.wheelAngle = (s.wheelAngle + spin * dt) % (2 * math.pi)
  local pressure = 200000
  if s.pressureGroupId then
    local p = obj:getGroupPressure(s.pressureGroupId)
    if p and p > 0 then pressure = p - 101325 end
  end

  local inp = rbrInput
  inp.computeLoad = false
  inp.spin, inp.hubHeight, inp.Fz, inp.dt = spin, s.hubHeight, Fz, dt
  inp.airPressure, inp.effRadius, inp.contactAngle = pressure, wd.dynamicRadius or s.radius, s.wheelAngle
  if usePac then
    inp.computeLoad = false
    inp.Fz = Fz > 0 and Fz or nil
    inp.hubHeight = s.hubHeight
    fillPacInput(s, inp, pressure)
    inp.grip = s.pacGrip
    inp.gripK = gripStiffness(surfaceName, s.pacGrip / math.max(P.svjGripScale, 1e-3))
    if inp.gripK then inp.gripK = inp.gripK * P.svjGripScale end
    inp.maxLoad = nil
    inp.progressive = false
    if Fz <= 0 then st.inContact = false; st.Flat, st.Flong, st.Mz, st.rollingTorque = 0, 0, 0, 0 else
      applyHydro(s, inp, dt, surfaceName)
      pac.step(st, svjTyre, nil, inp)
    end
  else
    rbr.step(st, T, mat, inp)
  end

  if not st.inContact then
    s.Fx, s.Fy, s.sliding, s.muEff = 0, 0, false, 0
    return
  end

  -- telemetry in the wrapper's frame (forward f = RBR rolling, lateral l = -RBR lateral)
  s.Fx, s.Fy = st.Flong, -st.Flat
  s.sx, s.sy = st.longSlipVel or 0, -(st.vLat or 0)
  s.muEff, s.sliding = st.staticF, (st.slidingW or 0) > 0.5

  -- apply: the friction force spread over the patch nodes; the aligning torque as a couple along the patch;
  -- the rolling resistance torque as a couple between the patch and the opposite tread node
  local L, Rl = st.L, st.Rl
  local count = s.patchCount
  local inv = 1 / count
  local Fxw = (L[1] * st.Flat + Rl[1] * st.Flong) * inv
  local Fyw = (L[2] * st.Flat + Rl[2] * st.Flong) * inv
  local Fzw = (L[3] * st.Flat + Rl[3] * st.Flong) * inv
  -- the couple's direction: torque of a force along L at +x along Rl is x (Rl x L)
  local dn = (Rl[2] * L[3] - Rl[3] * L[2]) * nx + (Rl[3] * L[1] - Rl[1] * L[3]) * ny + (Rl[1] * L[2] - Rl[2] * L[1]) * nz
  local mzk = 0
  if s.sumX2 > 1e-5 then mzk = st.Mz * P.mzScale * dn / s.sumX2 end
  local rrTop, rrBottom = 0, 0
  if s.topNode and st.rollingTorque ~= 0 then
    -- force at the top along -rolling and at the bottom along +rolling make a torque about the axle
    local F = st.rollingTorque / (2 * s.radius)
    rrTop, rrBottom = -F, F * inv
  end
  for i = 1, count do
    local cm = mzk * s.patchX[i]
    obj:applyForceVector(s.patch[i], vec3(Fxw + L[1] * cm + Rl[1] * rrBottom, Fyw + L[2] * cm + Rl[2] * rrBottom,
      Fzw + L[3] * cm + Rl[3] * rrBottom))
  end
  if rrTop ~= 0 then
    obj:applyForceVector(s.topNode, vec3(Rl[1] * rrTop, Rl[2] * rrTop, Rl[3] * rrTop))
  end
end

-- FULL RBR TYRE ----------------------------------------------------------------------------------------------
-- BeamNG's tyre ring no longer touches the ground (the GE hook in ui.lua turned its collision off when the
-- vehicle was loaded). The ground is found with a height query under the hub; RBR's own contact solve gives
-- the vertical load from the hub height, and the sinking into loose soil. The result is applied to the rim as
-- one force plus moments, so no BeamNG tyre flex sits in series with RBR's carcass.

local qv = vec3()
local fullSurfaceCache = {}

-- RBR surface name -> own-data surface (when the player's RBR data is not installed)
local function ownSurfaceFor(name)
  if name:find("^Tarmac") then return name:find("Wet") and "ASPHALT_WET" or "ASPHALT" end
  if name:find("^Gravel") then return "GRAVEL" end
  if name:find("^Dirt") or name:find("^Rough") then return "DIRT" end
  if name:find("^Snow") then return name:find("Ice") and "ICE" or "SNOW" end
  if name:find("^Grass") then return "GRASS" end
  return "ASPHALT"
end

-- BeamNG ground model (what the terrain under the wheel says) -> RBR surface, and -> friction scale for Pacejka.
local autoRbr = {
  ASPHALT = "Tarmac_Fine_Clean_Dry", ASPHALT_PREPPED = "Tarmac_Fine_Clean_Dry", ASPHALT_OLD = "Tarmac_Medium_Clean_Dry",
  ASPHALT_WET = "Tarmac_Fine_Clean_Wet", CONCRETE = "Tarmac_Medium_Clean_Dry", COBBLESTONE = "Tarmac_Coarse_Clean_Dry",
  RUMBLE_STRIP = "Tarmac_Coarse_Clean_Dry", ROCK = "Rough_Rough_Dry", GRAVEL = "Gravel_Medium_Deep_Dry",
  DIRT = "Gravel_Fine_Shallow_Dry", DIRT_DUSTY = "Gravel_Fine_Deep_Dry", GRASS = "Grass_Medium_Dry", MUD = "Dirt_Soft_Wet",
  SAND = "Gravel_Coarse_Deeper_Dry", SNOW = "SnowOnGravel_Medium", ICE = "SnowOnIce_Shallow",
  -- the rest of BeamNG's ground models
  DIRT_DUSTY_LOOSE = "Gravel_Fine_Deeper_Dry", GRAVEL_WET = "Gravel_Medium_Deep_Wet", SNOWBANK = "SnowOnGravel_Deep",
  SLIPPERY = "SnowOnIce_Shallow", FRICTIONLESS = "SnowOnIce_Deep", METAL = "Tarmac_Fine_Clean_Wet",
  METAL_TREAD = "Tarmac_Coarse_Clean_Dry", WOOD = "Tarmac_Medium_Clean_Dry", PLASTIC = "Tarmac_Fine_Clean_Wet",
  SPIKE_STRIP = "Tarmac_Coarse_Clean_Dry", KICKPLATE = "Tarmac_Fine_Clean_Wet", BRANCHES_STRONG = "Grass_Soft_Dry",
  LEAVES_STRONG = "Dirt_Soft_Damp", LEAVES_THIN = "Dirt_Medium_Damp", SOFT_COLLISION_GENERAL = "Grass_Soft_Dry",
}
local autoGrip = {
  ASPHALT = 1.0, ASPHALT_PREPPED = 1.05, ASPHALT_OLD = 0.95, ASPHALT_WET = 0.70, CONCRETE = 0.98, COBBLESTONE = 0.85,
  RUMBLE_STRIP = 0.95, ROCK = 0.85, GRAVEL = 0.60, DIRT = 0.55, DIRT_DUSTY = 0.50, GRASS = 0.50, MUD = 0.40, SAND = 0.50,
  SNOW = 0.30, ICE = 0.12,
  DIRT_DUSTY_LOOSE = 0.45, GRAVEL_WET = 0.50, SNOWBANK = 0.25, SLIPPERY = 0.08, FRICTIONLESS = 0.03, METAL = 0.60,
  METAL_TREAD = 0.75, WOOD = 0.70, PLASTIC = 0.60, SPIKE_STRIP = 0.60, KICKPLATE = 0.50, BRANCHES_STRONG = 0.45,
  LEAVES_STRONG = 0.50, LEAVES_THIN = 0.55, SOFT_COLLISION_GENERAL = 0.50,
}
local groundKind = "unknown"   -- "terrain" (read from the terrain under the wheel), "none" (map without terrain), "unknown"

local function detectedGround(s)
  if P.fullAutoSurface and (groundKind == "terrain" or groundKind == "mesh") and s.groundName and s.groundName ~= "" then return s.groundName end
  return nil
end

local function fullSurface(s)
  local gm = detectedGround(s)
  local mapped = gm and (autoRbr[gm] or autoRbr[canonGround(gm) or ""])
  local name = mapped or P.fullSurface
  if mapped then s.surfaceLabel = gm .. " -> " .. name
  elseif gm then s.surfaceLabel = gm .. " (unknown, manual: " .. name .. ")"
  else s.surfaceLabel = name .. " (manual)" end
  local useRbr = usingRbrData()
  local key = (useRbr and "rbr:" or "own:") .. name .. P.rbrGripScale .. "_" .. P.fullSoilSoftness
  local mat = fullSurfaceCache[key]
  if not mat then
    local base
    if useRbr then
      base = rbrData.rbr.surfaces[name] or rbrData.rbr.defaultSurface
    else
      base = rbrData.surfaces[ownSurfaceFor(name)] or rbrData.defaultSurface
    end
    mat = {}
    for k, val in pairs(base) do mat[k] = val end
    for _, k in ipairs({"static", "kinetic", "staticSoil", "kineticSoil"}) do mat[k] = base[k] * P.rbrGripScale end
    if mat.sinkFactor then mat.sinkFactor = mat.sinkFactor * P.fullSoilSoftness end
    fullSurfaceCache[key] = mat
  end
  return mat
end

-- Chooses up to 8 rim nodes spread evenly around the rim. They carry the wheel-spin torque as tangential forces
-- (many nodes, so no single light node gets a huge force).
local RIM_NODES = 8
local function setupRimPairs(s)
  s.rimNodes = nil
  local wd = s.wd
  local cids = wd.nodes
  if not cids or #cids < 4 then cids = s.tread end
  local p1, p2 = obj:getNodePosition(s.n1), obj:getNodePosition(s.n2)
  local hx, hy, hz = (p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5
  local ax, ay, az = p2.x - p1.x, p2.y - p1.y, p2.z - p1.z
  local al = vlen(ax, ay, az)
  if al < 1e-6 then return end
  ax, ay, az = ax / al, ay / al, az / al
  -- two axes perpendicular to the axle, to measure the angle of each node
  local ux, uy, uz = ay, -ax, 0
  if abs(ux) + abs(uy) < 1e-3 then ux, uy, uz = 0, az, -ay end
  local ul = vlen(ux, uy, uz)
  ux, uy, uz = ux / ul, uy / ul, uz / ul
  local wx, wy, wz = ay * uz - az * uy, az * ux - ax * uz, ax * uy - ay * ux
  local cand, rmax = {}, 0
  for _, cid in ipairs(cids) do
    local p = obj:getNodePosition(cid)
    local rx, ry, rz = p.x - hx, p.y - hy, p.z - hz
    local d = rx * ax + ry * ay + rz * az
    rx, ry, rz = rx - d * ax, ry - d * ay, rz - d * az
    local r = vlen(rx, ry, rz)
    cand[#cand + 1] = {cid = cid, r = r, ang = math.atan2(rx * wx + ry * wy + rz * wz, rx * ux + ry * uy + rz * uz)}
    if r > rmax then rmax = r end
  end
  local ring = {}
  for _, c in ipairs(cand) do
    if c.r > 0.8 * rmax then ring[#ring + 1] = c end
  end
  if #ring < 4 then return end
  local n = min(RIM_NODES, #ring)
  local chosen, used = {}, {}
  for k = 0, n - 1 do
    local target = -math.pi + (k + 0.5) * 2 * math.pi / n
    local best, bd = nil, 1e9
    for idx, c in ipairs(ring) do
      if not used[idx] then
        local d = abs(c.ang - target)
        d = min(d, 2 * math.pi - d)
        if d < bd then best, bd = idx, d end
      end
    end
    if best then used[best] = true; chosen[#chosen + 1] = ring[best].cid end
  end
  s.rimNodes = chosen
end

local function ringIsFree(s)
  local nd = v.data.nodes[s.tread[1]]
  return v.data.rbrFullTyre == true or (nd ~= nil and nd.collision == false)
end

-- Enveloping contact. A tyre that senses the ground at one point only meets a step as a sudden jump of the ground
-- height. A rigid wheel of radius R meets it with its front instead: the contact point moves to the edge of the step
-- and the normal tilts forward, so the force pushes the wheel up and back and the car climbs it gradually. Several
-- points of the ground profile ahead of and behind the hub are sampled; the closest to the hub is the contact.
local ENV_DEG = {-30, -15, 10, 20, 30, 40, 50, 60, 70, 80}
local ENV_SIN, ENV_COS = {}, {}
for k, d in ipairs(ENV_DEG) do ENV_SIN[k], ENV_COS[k] = math.sin(math.rad(d)), math.cos(math.rad(d)) end

local envX, envH = {}, {}
local function groundAt(wx, wy, wz, fhx, fhy, dx)
  qv:set(wx + fhx * dx, wy + fhy * dx, wz)
  local h = obj:getSurfaceHeightBelow(qv)
  if h and h == h then return h end
  return nil
end

local function envelope(s, wx, wy, wz, fhx, fhy, planarRho, gz)
  local R = s.radius
  local best, bdx, bdz = planarRho, nil, nil
  local function consider(dx, h)
    if h < wz then                              -- inside a wall taller than the hub: skip (the rim is the safety stop)
      local dz = wz - h
      local rho = sqrt(dx * dx + dz * dz)
      if rho < best - 0.0005 then best, bdx, bdz = rho, dx, dz end
    end
  end
  -- ground heights at the sample positions, in increasing distance along the heading (the point under the hub is gz)
  local n = 0
  for k = 1, #ENV_DEG do
    local dx = R * ENV_SIN[k]
    if ENV_DEG[k] > 0 and n == 0 then n = n + 1; envX[n], envH[n] = 0, gz end -- keep order: insert the hub point first
    n = n + 1
    envX[n], envH[n] = dx, groundAt(wx, wy, wz, fhx, fhy, dx)
  end
  for k = 1, n do
    local h = envH[k]
    if h then
      consider(envX[k], h)
      -- an edge between this sample and the previous one: find it by bisection and use its corner as a contact point
      local hp = k > 1 and envH[k - 1]
      if hp and abs(h - hp) > 0.015 then
        local lo, hi = envX[k - 1], envX[k]
        local hlo = hp
        for _ = 1, 3 do
          local mid = (lo + hi) * 0.5
          local hm = groundAt(wx, wy, wz, fhx, fhy, mid)
          if not hm then break end
          if abs(hm - hlo) < abs(h - hm) then lo = mid else hi = mid end
        end
        -- the corner: the edge position, at the height of the higher side
        local hh = (h > hp) and h or hp
        consider((lo + hi) * 0.5, hh)
      end
    end
  end
  if bdx then
    -- normal from the contact point to the hub, in the vertical plane of the heading
    local inv = 1 / best
    s.envN = s.envN or {0, 0, 1}
    s.envN[1], s.envN[2], s.envN[3] = -bdx * fhx * inv, -bdx * fhy * inv, bdz * inv
    s.envRho = best
    s.envDeg = math.deg(math.atan2(bdx, bdz))
  else
    s.envRho, s.envDeg = nil, 0
  end
end

local function stepWheelFull(s, dt, refPos, doFrame)
  local wd = s.wd
  if wd.isBroken then
    s.Fz, s.Fx, s.Fy, s.patchCount = 0, 0, 0, 0
    return
  end
  local p1 = obj:getNodePosition(s.n1)
  local p2 = obj:getNodePosition(s.n2)
  local hx, hy, hz = (p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5
  local wx, wy, wz = hx + refPos.x, hy + refPos.y, hz + refPos.z

  -- ground normal (a few times per ms) and the height under the hub
  if doFrame or not s.haveNormal then
    qv:set(wx, wy, wz)
    local n = mapmgr and mapmgr.surfaceNormalBelow and mapmgr.surfaceNormalBelow(qv, 0.3)
    if n and n.z == n.z and (n.x * n.x + n.y * n.y + n.z * n.z) > 0.5 then
      s.nx, s.ny, s.nz = n.x, n.y, n.z
    else
      local up = obj:getDirectionVectorUp()
      s.nx, s.ny, s.nz = up.x, up.y, up.z
    end
    s.haveNormal = true
  end
  local nx, ny, nz = s.nx, s.ny, s.nz
  if doFrame or not s.gz or (stepCounter % P.fullGroundEvery) == 0 then
    qv:set(wx, wy, wz)
    s.gz = obj:getSurfaceHeightBelow(qv)
  end
  local gz = s.gz
  if not gz or gz ~= gz or gz < wz - 5 then
    s.Fz, s.Fx, s.Fy, s.patchCount = 0, 0, 0, 0
    return
  end
  local hubH = (wz - gz) * max(nz, 0.3)
  s.pwx, s.pwy, s.pgz = wx, wy, gz

  -- axle pointing so that normal x axle is the forward direction
  local ax, ay, az = p2.x - p1.x, p2.y - p1.y, p2.z - p1.z
  local al = vlen(ax, ay, az)
  if al < 1e-6 then return end
  ax, ay, az = ax / al, ay / al, az / al
  local vf = obj:getDirectionVector()
  if (ny * az - nz * ay) * vf.x + (nz * ax - nx * az) * vf.y + (nx * ay - ny * ax) * vf.z < 0 then
    ax, ay, az = -ax, -ay, -az
  end

  -- enveloping contact (steps, kerbs): replaces the planar hub height and normal when the wheel's front or back
  -- meets the ground closer than the point straight below
  if P.fullEnvelope then
    if (stepCounter % P.fullEnvEvery) == 0 or s.envRho == nil and not s.envTried then
      s.envTried = true
      local rx, ry = ny * az - nz * ay, nz * ax - nx * az -- horizontal part of normal x axle = heading
      local rl = sqrt(rx * rx + ry * ry)
      if rl > 0.3 then envelope(s, wx, wy, wz, rx / rl, ry / rl, hubH, gz) end
    end
    if s.envRho and s.envRho < hubH then
      hubH = s.envRho
      nx, ny, nz = s.envN[1], s.envN[2], s.envN[3]
    end
  else
    s.envRho, s.envDeg = nil, 0
  end
  -- a short low-pass on the hub height takes the edge off single-step jumps
  if P.fullHeightTau > 0 then
    local prev = s.hubHf
    if prev and abs(hubH - prev) < 0.08 then
      hubH = prev + (hubH - prev) * (1 - math.exp(-dt / (P.fullHeightTau * 0.001)))
    end
    s.hubHf = hubH
  else
    s.hubHf = nil
  end

  local useBrush = P.fullModel == "brush"
  local useSvj = P.fullModel == "svj" and svjTyre ~= nil
  local T, key, st
  if useBrush then
    if not s.brushState then s.brushState = brushfull.newState() end
    st = s.brushState
  elseif useSvj then
    if not s.pacState or s.pacTyre ~= svjTyre then
      s.pacState = pac.newState(svjTyre, s.radius)
      s.pacTyre = svjTyre
    end
    st = s.pacState
  else
    T, key = rbrTyreType(s)
    if not s.rbrState or s.rbrTyreKey ~= key then
      s.rbrState = rbr.newState(T, s.radius)
      s.rbrTyreKey = key
    end
    st = s.rbrState
  end

  local v1 = obj:getNodeVelocityVector(s.n1)
  local v2 = obj:getNodeVelocityVector(s.n2)
  local vx, vy, vz = (v1.x + v2.x) * 0.5, (v1.y + v2.y) * 0.5, (v1.z + v2.z) * 0.5
  tmpN[1], tmpN[2], tmpN[3] = nx, ny, nz
  tmpA[1], tmpA[2], tmpA[3] = ax, ay, az
  tmpV[1], tmpV[2], tmpV[3] = vx, vy, vz

  local spin = -((wd.angularVelocity or 0) * (wd.wheelDir or 1))
  s.wheelAngle = (s.wheelAngle + spin * dt) % (2 * math.pi)
  local pressure = 200000
  if s.pressureGroupId then
    local p = obj:getGroupPressure(s.pressureGroupId)
    if p and p > 0 then pressure = p - 101325 end
  end

  local inp = rbrInput
  inp.computeLoad, inp.spin, inp.hubHeight, inp.dt = true, spin, hubH, dt
  inp.Fz = nil -- the shared input table: a load left by the light path must not leak into the full path
  inp.maxLoad = P.fullMaxLoadG * (s.staticLoad or staticFz)
  inp.dampMass = (s.staticLoad or staticFz) / 9.81
  inp.airPressure, inp.effRadius, inp.contactAngle = pressure, s.radius, s.wheelAngle
  if useBrush then
    local gmB = detectedGround(s)
    inp.grip = gmB and (surfaceGrip[gmB] or surfaceGrip[canonGround(gmB) or ""] or defaultSurfaceGrip) or P.fullBrushGrip
    s.surfaceLabel = gmB and (gmB .. " (x" .. string.format("%.2f", inp.grip) .. ")") or ("manual x" .. string.format("%.2f", P.fullBrushGrip))
    inp.freeRadius, inp.Re, inp.kz = s.radius, s.radius, P.fullBrushKz
    inp.progressive, inp.bumpDef = P.svjProgressive, P.svjBumpDef
    brushfull.step(st, P, nil, inp)
    s.advT, s.advWear = st.advT, st.advWear
  elseif useSvj then
    inp.freeRadius = P.svjRadiusFromWheel and s.radius or nil
    local gmS = detectedGround(s)
    inp.grip = gmS and ((autoGrip[gmS] or autoGrip[canonGround(gmS) or ""] or 0.8) * P.svjGripScale) or P.svjGrip
    inp.gripK = gripStiffness(gmS, inp.grip / math.max(P.svjGripScale, 1e-3))
    if inp.gripK then inp.gripK = inp.gripK * P.svjGripScale end
    s.surfaceLabel = gmS and (gmS .. " (x" .. string.format("%.2f", inp.grip) .. ")") or ("manual x" .. string.format("%.2f", P.svjGrip))
    inp.pressure = P.svjUseCarPressure and pressure or nil
    inp.zeta = P.svjZeta
    -- nominal load Fz0 of the coefficients: manual, else the file's, else the static load carried by this wheel
    local fz0 = nil
    if P.svjFz0 > 0 then fz0 = P.svjFz0
    elseif svjTyre.noFz0 then fz0 = (s.staticLoad or staticFz) * P.svjFz0Scale
    elseif P.svjFz0Scale ~= 1 then fz0 = svjTyre.Fz0 * P.svjFz0Scale end
    inp.Fz0, inp.relaxScale = fz0, P.svjRelaxScale
    inp.Re = P.svjRadiusFromWheel and s.radius or nil
    inp.dampMass = (s.staticLoad or staticFz) / 9.81
    s.fz0used = fz0 or svjTyre.Fz0
    inp.progressive, inp.bumpDef = P.svjProgressive, P.svjBumpDef
    applyHydro(s, inp, dt, gmS)
    pac.step(st, svjTyre, nil, inp)
  else
    rbr.step(st, T, fullSurface(s), inp)
  end

  s.hubH = hubH
  s.hubx, s.huby, s.hubz, s.axx, s.axy, s.axz = hx, hy, hz, ax, ay, az       -- (for the debug drawing)
  if not st.inContact then
    s.Fz, s.Fx, s.Fy, s.sliding, s.muEff, s.patchCount = 0, 0, 0, false, 0, 0
    return
  end

  local L, Rl = st.L, st.Rl
  local vn = vx * nx + vy * ny + vz * nz
  -- vertical damping: a fixed part, plus a part that fades in below ~4 m/s so a parked car does not ring
  local cV = (useSvj and P.svjUseTyreDamping and svjTyre.cz) or P.fullVertDamping
  if P.fullLowSpeedVertZeta > 0 then
    local spd = sqrt(vx * vx + vy * vy + vz * vz)
    local w = 1 - spd * 0.25
    if w > 0 then
      local kv = 1.5 * st.normalForce / max(s.radius - hubH, 1e-3) -- tyre vertical stiffness at this load
      cV = cV + w * 2 * P.fullLowSpeedVertZeta * sqrt(kv * P.fullUnsprungMass)
    end
  end
  local Fn = max(0, st.normalForce - cV * vn)

  -- force on the wheel, and its moment about the hub (contact point is hubH below the hub along the normal)
  local Fx_ = L[1] * st.Flat + Rl[1] * st.Flong + nx * Fn
  local Fy_ = L[2] * st.Flat + Rl[2] * st.Flong + ny * Fn
  local Fz_ = L[3] * st.Flat + Rl[3] * st.Flong + nz * Fn
  local rx, ry, rz = -nx * hubH, -ny * hubH, -nz * hubH
  local mf_ = st.Mfwd or 0 -- overturning moment about the forward axis (Pacejka only)
  local mzS = st.Mz * P.mzScale
  local Mx = ry * Fz_ - rz * Fy_ + mzS * nx + mf_ * Rl[1]
  local My = rz * Fx_ - rx * Fz_ + mzS * ny + mf_ * Rl[2]
  local Mz = rx * Fy_ - ry * Fx_ + mzS * nz + mf_ * Rl[3]
  if not (Fx_ == Fx_ and Fy_ == Fy_ and Fz_ == Fz_ and Mx == Mx and My == My and Mz == Mz) or
     abs(Fx_) + abs(Fy_) + abs(Fz_) > 12 * P.fullMaxLoadG * (s.staticLoad or staticFz) + 2e4 then
    if not s.warnedNaN then
      s.warnedNaN = true
      log("E", "modelTire", string.format("wheel %s: non-finite or absurd tyre force skipped (F %.3g %.3g %.3g, hub height %.3f m, load %.3g N)",
        tostring(s.name), Fx_, Fy_, Fz_, hubH, Fn))
    end
    s.Fz, s.Fx, s.Fy, s.patchCount = 0, 0, 0, 0
    return
  end
  -- fade-in after switching on / reset, so the first steps cannot kick the car
  s.age = (s.age or 0) + dt
  if P.fullFadeIn > 0 and s.age < P.fullFadeIn then
    local f = s.age / P.fullFadeIn
    Fx_, Fy_, Fz_, Mx, My, Mz = Fx_ * f, Fy_ * f, Fz_ * f, Mx * f, My * f, Mz * f
    st.rollingTorque = st.rollingTorque * f
  end
  -- slew limiter: the tyre force may not jump by more than fullSlewKN per millisecond (a spike filter)
  local slew = P.fullSlewKN
  if slew > 0 then
    local pf = s.prevW
    if pf and s.age > dt * 2 then
      local d = slew * 1000 * (dt / 0.001)
      local dm = d * 0.35
      Fx_ = pf[1] + clamp(Fx_ - pf[1], -d, d)
      Fy_ = pf[2] + clamp(Fy_ - pf[2], -d, d)
      Fz_ = pf[3] + clamp(Fz_ - pf[3], -d, d)
      Mx = pf[4] + clamp(Mx - pf[4], -dm, dm)
      My = pf[5] + clamp(My - pf[5], -dm, dm)
      Mz = pf[6] + clamp(Mz - pf[6], -dm, dm)
    end
    pf = pf or {}
    pf[1], pf[2], pf[3], pf[4], pf[5], pf[6] = Fx_, Fy_, Fz_, Mx, My, Mz
    s.prevW = pf
  else
    s.prevW = nil
  end
  local Maxle = Mx * ax + My * ay + Mz * az
  local tauSpin = Maxle + st.rollingTorque
  local tauMax = 2.5 * Fn * s.radius + 30 -- more than friction can transmit is a fault
  tauSpin = clamp(tauSpin, -tauMax, tauMax)
  local Px, Py, Pz = Mx - Maxle * ax, My - Maxle * ay, Mz - Maxle * az

  -- hub nodes: half of the force each, plus the couple that makes the perpendicular moment
  local dx, dy, dz = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
  local inv = 1 / (dx * dx + dy * dy + dz * dz)
  local gx, gy, gz2 = (Py * dz - Pz * dy) * inv, (Pz * dx - Px * dz) * inv, (Px * dy - Py * dx) * inv
  obj:applyForceVector(s.n1, vec3(Fx_ * 0.5 + gx, Fy_ * 0.5 + gy, Fz_ * 0.5 + gz2))
  obj:applyForceVector(s.n2, vec3(Fx_ * 0.5 - gx, Fy_ * 0.5 - gy, Fz_ * 0.5 - gz2))

  -- wheel-spin torque: tangential forces on the rim nodes, proportional to their radius, so that they add up to a
  -- pure couple about the axle (force k * (A x q), torque = k * sum |q|^2 = tauSpin)
  if s.rimNodes == nil and not s.rimTried then s.rimTried = true; setupRimPairs(s) end
  local rn = s.rimNodes
  if rn then
    local qs = s.qbuf or {}
    s.qbuf = qs
    local sumQ2 = 0
    for k = 1, #rn do
      local p = obj:getNodePosition(rn[k])
      local qx, qy, qz = p.x - hx, p.y - hy, p.z - hz
      local d = qx * ax + qy * ay + qz * az
      qx, qy, qz = qx - d * ax, qy - d * ay, qz - d * az
      local e = qs[k] or {}
      qs[k] = e
      e[1], e[2], e[3] = qx, qy, qz
      sumQ2 = sumQ2 + qx * qx + qy * qy + qz * qz
    end
    if sumQ2 > 1e-6 then
      local k = tauSpin / sumQ2
      for j = 1, #rn do
        local e = qs[j]
        obj:applyForceVector(rn[j], vec3((ay * e[3] - az * e[2]) * k, (az * e[1] - ax * e[3]) * k, (ax * e[2] - ay * e[1]) * k))
      end
    end
  end

  -- telemetry and debug-draw values (lateral axis here is minus RBR's, as in the other models)
  s.Fz, s.Fx, s.Fy = Fn, st.Flong, -st.Flat
  s.sx, s.sy = st.longSlipVel or 0, -(st.vLat or 0)
  s.muEff, s.sliding = st.staticF, st.slidingW > 0.5
  s.patchCount = 1
  s.cx, s.cy, s.cz = hx - nx * hubH, hy - ny * hubH, hz - nz * hubH
  s.fx_, s.fy_, s.fz_ = Rl[1], Rl[2], Rl[3]
  s.lx_, s.ly_, s.lz_ = -L[1], -L[2], -L[3]
  s.sumX2 = 0
end

-- Advanced brush helpers ------------------------------------------------------------------------------------------
-- Stiffness per N falls as the load rises (a tyre's cornering stiffness grows less than in proportion to the load): this is
-- 1 at the nominal load and the same shape as the Magic Formula's stiffness-against-load term.
local function advStiffLoad(Fz)
  local c = P.advLoadShape
  return math.sin(2 * math.atan(Fz / (c * P.fzNominal))) / math.sin(2 * math.atan(1 / c)) * P.fzNominal / max(Fz, 1)
end

-- grip against temperature: a gaussian bump from the floor up to 1 at the optimum
local function advTempFactor(T)
  local z = (T - P.advTopt) / P.advTwidth
  return P.advTfloor + (1 - P.advTfloor) * exp(-0.5 * z * z)
end

-- the pressure of BeamNG's tyre over the nominal one (1 when the tyre has no pressure group)
local function advPressureRatio(s)
  if s.pressureGroupId then
    local p = obj:getGroupPressure(s.pressureGroupId)
    if p and p > 0 then return max(0.3, min(2.5, (p - 101325) / (P.advPressureNom * 1000))) end
  end
  return 1
end

-- The wheel-spin torque as a pure couple on the rim nodes: tangential forces k * (a x q) on every rim node (q: its position
-- about the axle), whose torque about the axis a (unit vector) is k * sum |q|^2 = tau. Returns false when the wheel has no rim
-- nodes to push (then the caller applies the force the old way).
local function spinCouple(s, ax, ay, az, tau)
  local rn = s.rimNodes
  if not rn then return false end
  local p1, p2 = obj:getNodePosition(s.n1), obj:getNodePosition(s.n2)
  local hx, hy, hz = (p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5
  local qs = s.qbuf or {}
  s.qbuf = qs
  local sumQ2 = 0
  for k = 1, #rn do
    local p = obj:getNodePosition(rn[k])
    local qx, qy, qz = p.x - hx, p.y - hy, p.z - hz
    local d = qx * ax + qy * ay + qz * az
    qx, qy, qz = qx - d * ax, qy - d * ay, qz - d * az
    local e = qs[k] or {}
    qs[k] = e
    e[1], e[2], e[3] = qx, qy, qz
    sumQ2 = sumQ2 + qx * qx + qy * qy + qz * qz
  end
  if sumQ2 < 1e-6 then return false end
  local k = tau / sumQ2
  for j = 1, #rn do
    local e = qs[j]
    obj:applyForceVector(rn[j], vec3((ay * e[3] - az * e[2]) * k, (az * e[1] - ax * e[3]) * k, (ax * e[2] - ay * e[1]) * k))
  end
  return true
end

local function stepWheel(s, dt, refPos, doPatch)
  local wd = s.wd
  if wd.isBroken or wd.isTireDeflated then
    s.Fz, s.Fx, s.Fy = 0, 0, 0
    return
  end
  if doPatch or s.patchCount == 0 then updatePatch(s, refPos) end

  -- normal load and surface from the core
  local _, _, _, downForce, _, _, m1, m2 = wd.obj:getSlipVelEnergyDownPeakForceDepthMats()
  local Fz = downForce or 0
  if P.fzFallback and Fz <= 0 and obj:isNodeColliding(s.patch[1] or s.tread[1]) then Fz = staticFz end
  s.Fz = Fz
  Fz = smoothLoad(s, Fz, dt)
  if not Fz or s.patchCount == 0 then
    -- airborne: the carcass relaxes back
    s.dx, s.dy = s.dx * 0.95, s.dy * 0.95
    s.Fx, s.Fy, s.sliding = 0, 0, false
    return
  end
  local grip, surfaceName = surfaceForMaterials(m1, m2)
  s.grip, s.surface = grip, surfaceName

  local fx, fy, fz = s.fx_, s.fy_, s.fz_
  local lx, ly, lz = s.lx_, s.ly_, s.lz_

  -- slip velocity in the contact frame
  local vx, vy, sx, sy
  local wheelSpeed = wd.wheelSpeed or 0
  if P.slipFromTread then
    local ax, ay, az = 0, 0, 0
    for i = 1, s.patchCount do
      local vel = obj:getNodeVelocityVector(s.patch[i])
      ax, ay, az = ax + vel.x, ay + vel.y, az + vel.z
    end
    local inv = 1 / s.patchCount
    ax, ay, az = ax * inv, ay * inv, az * inv
    sx = ax * fx + ay * fy + az * fz
    sy = ax * lx + ay * ly + az * lz
    local v1 = obj:getNodeVelocityVector(s.n1)
    vx = v1.x * fx + v1.y * fy + v1.z * fz
    vy = v1.x * lx + v1.y * ly + v1.z * lz
  else
    local v1 = obj:getNodeVelocityVector(s.n1)
    local v2 = obj:getNodeVelocityVector(s.n2)
    local hx, hy, hz = (v1.x + v2.x) * 0.5, (v1.y + v2.y) * 0.5, (v1.z + v2.z) * 0.5
    vx = hx * fx + hy * fy + hz * fz
    vy = hx * lx + hy * ly + hz * lz
    sx = vx - wheelSpeed
    sy = vy
  end
  s.sx, s.sy = sx, sy

  -- stiffness and damping at this load
  local Cy = P.cornerCoef * Fz
  local Cx = P.longCoef * Fz
  local advMu = 1
  if P.advBrush then
    -- load: stiffness per N falls as the load rises; pressure: stiffness and grip follow the tyre's own pressure;
    -- temperature and wear: grip (the state of the last step)
    local kl = advStiffLoad(Fz)
    local pr = advPressureRatio(s)
    local kp = pr ^ P.advPressureK
    Cy, Cx = P.cornerCoef * Fz * kl * kp, P.longCoef * Fz * kl * kp
    advMu = advTempFactor(s.advT or P.advTstart) * (1 - P.advWearLoss * min(1, s.advWear or 0)) * max(0.6, 1 - P.advPressureMu * (pr - 1) ^ 2)
  end
  local ky = Cy / P.relaxLenY
  local kx = Cx / P.relaxLenX
  local mq = Fz / g -- quarter car mass carried by this wheel
  local cyD = 2 * P.dampRatio * sqrt(ky * mq)
  local cxD = 2 * P.dampRatio * sqrt(kx * mq)

  -- deflection update, exact exponential relaxation: dd/dt = s - a d, a = V / L
  local V = max(abs(vx), abs(wheelSpeed))
  local ay_ = V / P.relaxLenY
  local ax_ = V / P.relaxLenX
  local dx0, dy0 = s.dx, s.dy
  local dx, dy
  if ax_ * dt > 1e-6 then
    local e = exp(-ax_ * dt)
    dx = dx0 * e + sx * (1 - e) / ax_
  else
    dx = dx0 + sx * dt
  end
  if ay_ * dt > 1e-6 then
    local e = exp(-ay_ * dt)
    dy = dy0 * e + sy * (1 - e) / ay_
  else
    dy = dy0 + sy * dt
  end
  local ddx, ddy = (dx - dx0) / dt, (dy - dy0) / dt
  -- the spring force is limited by friction (and that is what sets the deflection when sliding); the damper acts on top
  -- and only the total is clamped, so velocity noise in the damper can never eat into the stored deflection
  local Fx = -(kx * dx)
  local Fy = -(ky * dy)

  -- friction limit: mu falls with slip speed, drops with load, scaled by the surface
  local slipSpeed = sqrt(sx * sx + sy * sy)
  local q = slipSpeed / P.stribeckVel
  local mu = P.muSlide + (P.mu - P.muSlide) * exp(-q * q)
  mu = mu * max(0.4, 1 - P.loadSens * (Fz / P.fzNominal - 1)) * grip * advMu
  local FmaxY = mu * Fz
  local FmaxX = mu * P.muLongScale * Fz
  -- Brush model with a parabolic pressure distribution (Pacejka, chapter 3; Fiala): the spring force Fs of the bristles is
  -- the force an unsaturated patch would give, and the real force rolls over smoothly as the rear of the patch starts to
  -- slide: F = Fmax (3u - 3u^2 + u^3), u = Fs / (3 Fmax), constant Fmax beyond u = 1. Combined slip: u is the length of
  -- the vector (Fsx / FmaxX, Fsy / FmaxY) / 3 and the force follows the same direction (a friction ellipse).
  local ux, uy = Fx / (3 * FmaxX), Fy / (3 * FmaxY)
  local ur = sqrt(ux * ux + uy * uy)
  local sliding = ur >= 1
  if ur > 1e-6 then
    local g = (ur < 1) and (3 * ur - 3 * ur * ur + ur * ur * ur) or 1
    local f = g / ur
    Fx, Fy = FmaxX * ux * f, FmaxY * uy * f
  end
  -- far past the limit the bristles are only wound up further for nothing: hold the deflection a little beyond the
  -- point where the force saturates, so unwinding starts at once when the slip falls again
  if ur > 1.15 then
    local k = 1.15 / ur
    dx, dy = dx * k, dy * k
  end
  s.dx, s.dy = dx, dy
  Fx, Fy = Fx - cxD * ddx, Fy - cyD * ddy
  local rt = sqrt((Fx / FmaxX) ^ 2 + (Fy / FmaxY) ^ 2)
  if rt > 1 then Fx, Fy = Fx / rt, Fy / rt end

  -- rolling resistance, smoothed through zero speed
  local rr = -P.crr * Fz * math.tanh(wheelSpeed / 0.5)
  local FxTotal = Fx + rr

  s.Fx, s.Fy, s.muEff, s.sliding = FxTotal, Fy, mu, sliding

  if P.advBrush then
    -- one-node thermal model: sliding friction power (force x slip speed, counted as it saturates) and rolling heat in,
    -- convection out; wear grows with the same power, faster when overheated
    local T = s.advT or P.advTstart
    local slideShare = max(0, min(1, (ur - 0.3) / 0.7))
    local Psl = (abs(Fx * sx) + abs(Fy * sy)) * slideShare
    local speed = max(abs(vx), abs(wheelSpeed))
    local heat = P.advHeatShare * Psl + P.advRollHeat * Fz * abs(wheelSpeed)
    local cool = (P.advCoolBase + P.advCoolSpeed * speed) * (T - P.advTamb)
    local k = dt * P.advTimeScale
    T = T + (heat - cool) / P.advHeatCap * k
    s.advT = max(P.advTamb - 40, min(220, T))
    local hot = max(0, (T - 110) / 25)
    s.advWear = (s.advWear or 0) + Psl * (1 + hot * hot) / P.advWearJ * k
    s.advP = Psl
  end

  -- apply: longitudinal shared evenly, lateral weighted so it acts a trail
  -- behind the patch centre (gives aligning torque), trail fades as Fy saturates
  local count = s.patchCount
  local inv = 1 / count
  -- Longitudinal force: pushed into the tread nodes it reaches the rim only through BeamNG's soft tyre carcass, and with a stiff
  -- brush spring that loop rings (a free-rolling wheel swung by +-0.3 slip ratio on the test sled and the lateral force starved).
  -- So, as in the FULL modes, it acts at the hub (where it reaches the car) and its torque spins the rim directly.
  local rimOk = false
  if P.brushRimTorque then
    if s.rimNodes == nil and not s.rimTried then s.rimTried = true; setupRimPairs(s) end
    rimOk = s.rimNodes ~= nil
  end
  local t = 0
  if s.sumX2 > 1e-5 then
    t = P.trail * max(0, 1 - abs(Fy) / FmaxY)
    -- keep every weight positive
    local maxShift = 0
    for i = 1, count do maxShift = max(maxShift, abs(s.patchX[i])) end
    local tMax = 0.9 * s.sumX2 / (count * maxShift + 1e-9)
    if t > tMax then t = tMax end
  end
  local tk = t / (s.sumX2 + 1e-12)
  for i = 1, count do
    local wy = inv - tk * s.patchX[i]
    local px = rimOk and 0 or FxTotal * inv
    local py = Fy * wy
    obj:applyForceVector(s.patch[i], vec3(fx * px + lx * py, fy * px + ly * py, fz * px + lz * py))
  end
  if rimOk then
    local half = FxTotal * 0.5
    obj:applyForceVector(s.n1, vec3(fx * half, fy * half, fz * half))
    obj:applyForceVector(s.n2, vec3(fx * half, fy * half, fz * half))
    -- torque of the contact force about the axle: -F * (hub height), about a = normal x forward (forward rolling is positive)
    local nx, ny, nz = s.nx, s.ny, s.nz
    local tau = -FxTotal * max(s.hubHeight or 0, 0.05)
    spinCouple(s, ny * fz - nz * fy, nz * fx - nx * fz, nx * fy - ny * fx, tau)
  end
end

-- Model-independent kinematics for the in-game sweep tests (sweep.lua): the slip angle and slip ratio of each wheel from
-- the hub velocity and the wheel's own heading, and the car's accelerations, valid for every mode and for BeamNG's own tyre.
local nativeOn = false
local function wheelKinematics(s)
  local p1, p2 = obj:getNodePosition(s.n1), obj:getNodePosition(s.n2)
  local ax, ay, az = p2.x - p1.x, p2.y - p1.y, p2.z - p1.z
  local up = obj:getDirectionVectorUp()
  -- the wheel's rolling direction: up x axle, turned to point the way the car points
  local fx, fy, fz = up.y * az - up.z * ay, up.z * ax - up.x * az, up.x * ay - up.y * ax
  local fl = vlen(fx, fy, fz)
  if fl < 1e-6 then return nil end
  fx, fy, fz = fx / fl, fy / fl, fz / fl
  local c = obj:getDirectionVector()
  if fx * c.x + fy * c.y + fz * c.z < 0 then fx, fy, fz = -fx, -fy, -fz end
  local v1, v2 = obj:getNodeVelocityVector(s.n1), obj:getNodeVelocityVector(s.n2)
  local vx, vy, vz = (v1.x + v2.x) * 0.5, (v1.y + v2.y) * 0.5, (v1.z + v2.z) * 0.5
  local vf = vx * fx + vy * fy + vz * fz
  -- lateral: the horizontal direction across the wheel plane
  local lx, ly, lz = up.y * fz - up.z * fy, up.z * fx - up.x * fz, up.x * fy - up.y * fx
  local vl = vx * lx + vy * ly + vz * lz
  local ref = math.max(abs(vf), 0.5)
  local wsp = (s.wd.wheelSpeed or 0)
  return math.atan(vl / ref), (abs(wsp) * ((vf >= 0) and 1 or -1) - vf) / ref
end

local rackAcc, rackN = 0, 0
local function steerRack()
  if not (hydros and hydros.hydros) then return nil end
  local ok, rack = pcall(function()
    local sum, n = 0, 0
    for _, h in ipairs(hydros.hydros) do
      if h.inputSource == "steering_input" and h.bcid and h.fgetDisplacement == obj.getBeamLength then
        local beam = v.data.beams[h.bcid]
        if beam and beam.beamSpring then
          sum = sum + (obj:getBeamLength(h.bcid) - obj:getBeamRefLength(h.bcid) * (h.state or 1)) * beam.beamSpring
          n = n + 1
        end
      end
    end
    return n > 0 and sum or nil
  end)
  return ok and rack or nil
end

local function sendTelemetry()
  if nativeOn and not initDone then initWheels() end
  local wl = {}
  for i, s in ipairs(W) do
    if s.accN and s.accN > 0 then
      s.avgFz, s.avgFx, s.avgFy = s.accFz / s.accN, s.accFx / s.accN, s.accFy / s.accN
    end
    wl[i] = {
      name = s.name, Fz = s.avgFz or s.Fz, Fx = s.avgFx or s.Fx, Fy = s.avgFy or s.Fy, sx = s.sx, sy = s.sy,
      FzSpread = (s.accN and s.accN > 0) and ((s.maxFz or 0) - (s.minFz or 0)) or 0,
      fz0 = s.fz0used, env = s.envDeg or 0, staticLoad = s.staticLoad or staticFz,
      mu = s.muEff, sliding = s.sliding, nodes = s.patchCount, surface = (fullIsOn and s.surfaceLabel) or s.surface,
      dx = s.dx, dy = s.dy,
    }
    s.accN, s.accFz, s.accFx, s.accFy, s.minFz, s.maxFz = 0, 0, 0, 0, nil, nil
    -- force vectors for the 3D arrows: contact point (relative to the vehicle), friction force, normal force (N)
    if s.patchCount and s.patchCount > 0 and (s.Fz or 0) > P.minFz and s.cx and s.fx_ and s.nx then
      local fx, fy = s.Fx or 0, s.Fy or 0
      wl[i].vec = {
        math.floor(s.cx * 1000) / 1000, math.floor(s.cy * 1000) / 1000, math.floor(s.cz * 1000) / 1000,
        math.floor(s.fx_ * fx + s.lx_ * fy), math.floor(s.fy_ * fx + s.ly_ * fy), math.floor(s.fz_ * fx + s.lz_ * fy),
        math.floor(s.nx * s.Fz), math.floor(s.ny * s.Fz), math.floor(s.nz * s.Fz),
      }
    end
    if nativeOn then
      local a, k = wheelKinematics(s)
      if a then wl[i].kin = {a, k} end
    end
    local ps = s.pacState
    if ps and ps.inContact and (fullIsOn and P.fullModel == "svj" or (not fullIsOn and P.useRbr102 and P.lightModel == "svj")) then
      wl[i].pac = {
        kappa = ps.kappa, alpha = ps.alpha, gamma = ps.gamma, dpi = ps.dpi, lam = ps.lam, lamK = ps.lamK, Vx = ps.Vx, hydro = s.hydroF, vp = s.hydroVp, film = s.hydroFilm,
        Fz = ps.normalForce, Fx = ps.Fx, Fy = ps.Fy, fz0 = ps.fz0,
      }
    end
    if (P.advBrush or (fullIsOn and P.fullModel == "brush")) and s.advT then
      wl[i].surfT, wl[i].treadT, wl[i].wear = s.advT, s.advT, s.advWear or 0   -- one node: the same temperature everywhere
    end
    local st = P.useRbr102 and s.rbrState
    if st then
      local surfAvg = 0
      for k = 1, 8 do surfAvg = surfAvg + st.surfaceTemp[k] end
      local wearAvg = 0
      for k = 1, 8 do wearAvg = wearAvg + st.wear[k] end
      wl[i].surfT = surfAvg / 8 - 273.15
      wl[i].treadT = st.treadTemp - 273.15
      wl[i].carcassT = st.carcassTemp - 273.15
      wl[i].wear = wearAvg / 8
      wl[i].sink = st.sinkDepth
      wl[i].slideW = st.slidingW
      wl[i].Mz = st.Mz
    end
  end
  local data = {enabled = enabled, wheels = wl, stepUs = lastStepUs, vid = obj:getId(),
    model = "brush", rbrDataAvailable = rbrData.rbr ~= nil, fullState = fullState}
  data.svjLoaded = svjTyre ~= nil and svjTyre.name or false
  if nativeOn and sensors then data.acc = {sensors.gx2 or 0, sensors.gy2 or 0} end
  -- steering rack force: the tyres push on the steering linkage and the game's steering hydros give way by an amount that
  -- is the force on the rack over its stiffness. The force the game would send to a force feedback wheel is built from
  -- the same quantity, but it is only computed when such a wheel is connected, so it is rebuilt here for every setup:
  -- sum over the steering hydros of (beam length - commanded length) x beam spring, in N (a measure, not the wheel torque)
  -- the mean over the graphics frames since the last report (a single sample of this signal is mostly noise)
  if nativeOn and rackN > 0 then data.rack = rackAcc / rackN end
  rackAcc, rackN = 0, 0
  -- the test sled's controller (rig.lua) reports the total tyre force it measures when a rig run is active
  if modelTire_rig and modelTire_rig.report then data.rig = modelTire_rig.report() end
  if modelTire_wheelrig and modelTire_wheelrig.report then data.rig = modelTire_wheelrig.report() end     -- (the single-wheel rig)
  data.rbrTyreList = tyreChoices()
  if fullIsOn and P.fullModel == "svj" then
    data.model = svjTyre and ((P.svjModded and "FULL Modded SVJ Pacejka " or "FULL SVJ Pacejka ") .. tostring(svjTyre.modelName) .. ": " .. tostring(svjTyre.name))
      or "SVJ Pacejka selected but no tyre sent: running the RBR tyre"
  elseif fullIsOn and P.fullModel == "brush" then
    data.model = "FULL Advanced brush"
  elseif fullIsOn then
    local _, key = rbrTyreType()
    data.model = "FULL RBR tyre, " .. (usingRbrData() and "RBR data, " or "own data, ") .. key:gsub("^rbr:", "") ..
      ", " .. tostring(P.fullSurface)
  elseif P.useRbr102 and P.lightModel == "svj" and svjTyre then
    data.model = (P.svjModded and "Modded " or "") .. "SVJ Pacejka on BeamNG's tyre " .. tostring(svjTyre.modelName) .. ": " .. tostring(svjTyre.name)
  elseif P.useRbr102 then
    local _, key = rbrTyreType()
    data.model = "RBR 1.02, " .. (usingRbrData() and "RBR data, " or "own data, ") .. key:gsub("^rbr:", "")
  end
  if P.advBrush and not P.useRbr102 and not fullIsOn then data.model = "Advanced brush" end
  data.groundKind = groundKind
  if W[1] then data.wheelInfo = {radius = W[1].radius, width = W[1].tireWidth or 0, hubRadius = W[1].hubRadius or 0} end
  obj:queueGameEngineLua("if modelTire_ui then modelTire_ui.onTelemetry(" .. serialize(data) .. ") end")
  -- ask the game side which ground model lies under each wheel (it can read the terrain; we cannot)
  if fullIsOn and P.fullAutoSurface then
    local pts = {}
    for _, s in ipairs(W) do
      if s.pwx then pts[#pts + 1] = {s.pwx, s.pwy, s.pgz} end
    end
    if #pts > 0 then
      obj:queueGameEngineLua("if modelTire_ui then modelTire_ui.probeGround(" .. obj:getId() .. "," .. serialize(pts) .. ") end")
    end
  end
end

-- public -------------------------------------------------------------------

local function enable()
  if not initDone then initWheels() end
  if #W == 0 then
    log("E", "modelTire", "No pressure wheels with tread nodes found on this vehicle")
    return
  end
  for _, s in ipairs(W) do s.dx, s.dy, s.patchCount, s.rbrState, s.gz, s.haveNormal, s.age, s.pacState, s.prevW, s.hubHf, s.envRho, s.envTried = 0, 0, 0, nil, nil, false, 0, nil, nil, nil, nil, false; s.brushState = nil end
  setNativeFriction(P.nativeFriction)
  enabled = true
  refreshFull()
  enablePhysicsStepHook()
  log("I", "modelTire", "enabled on " .. #W .. " wheels, full tyre: " .. fullState)
  if P.useRbr102 then
    for _, s in ipairs(W) do
      local T0, _ = rbrTyreType(s)
      log("I", "modelTire", string.format("wheel %s: radius %.3f m, width %.3f m, tyre scale k = %.2f", tostring(s.name), s.radius, s.tireWidth or 0, T0.scaleK or 1))
    end
  end
  sendTelemetry()
end

-- The full tyre only runs when the tyre ring has no ground collision (the vehicle was reloaded with the GE
-- hook active); otherwise the ring and RBR would both carry the load.
local function fullReady()
  if not P.fullTyre then return false end
  if P.fullIgnoreCheck then return true end
  for _, s in ipairs(W) do
    if not ringIsFree(s) then return false end
  end
  return #W > 0
end

refreshFull = function()
  local ready = initDone and fullReady()
  fullIsOn = enabled and ready
  if not P.fullTyre then fullState = "off"
  elseif ready then fullState = "on"
  else fullState = "needs reload" end
  if P.fullTyre and not ready and initDone then
    log("W", "modelTire", "full tyre requested but the tyre ring still collides with the ground: reload the vehicle")
  end
end

local function disable()
  if initDone then setNativeFriction(1) end
  enabled = false
  fullIsOn = false
  log("I", "modelTire", "disabled, native friction restored")
  sendTelemetry()
end

local function setParams(t)
  if type(t) ~= "table" then return end
  local oldNative = P.nativeFriction
  for k, val in pairs(t) do
    if P[k] ~= nil and type(P[k]) == type(val) then P[k] = val end
  end
  if enabled and P.nativeFriction ~= oldNative then setNativeFriction(P.nativeFriction) end
  if enabled then refreshFull() end
end

local function getParams() return P end

-- hooks ----------------------------------------------------------------------

local function onPhysicsStep(dt)
  if not enabled then return end
  if timer then timer:stopAndReset() end
  stepCounter = stepCounter + 1
  local doPatch = (stepCounter % P.patchEverySteps) == 0
  local refPos = obj:getPosition()
  local stepFn = P.useRbr102 and stepWheelRbr or stepWheel
  if fullIsOn then stepFn = stepWheelFull end
  for i = 1, #W do
    local s = W[i]
    stepFn(s, dt, refPos, doPatch)
    -- 100 ms averages for the window: single samples of a 2 kHz signal look noisier than they are
    s.accN = (s.accN or 0) + 1
    s.accFz = (s.accFz or 0) + (s.Fz or 0)
    s.accFx = (s.accFx or 0) + (s.Fx or 0)
    s.accFy = (s.accFy or 0) + (s.Fy or 0)
    local fz = s.Fz or 0
    if not s.minFz or fz < s.minFz then s.minFz = fz end
    if not s.maxFz or fz > s.maxFz then s.maxFz = fz end
  end
  if timer then
    timeAccum = timeAccum + timer:stopAndReset()
    timeSteps = timeSteps + 1
  end
end

local function updateGFX(dt)
  if nativeOn then
    local r = steerRack()
    if r then rackAcc, rackN = rackAcc + r, rackN + 1 end
  end
  telemetryTimer = telemetryTimer + dt
  if telemetryTimer >= 0.1 then
    telemetryTimer = 0
    if timeSteps > 0 then lastStepUs = timeAccum / timeSteps * 1000 end -- ms -> us
    timeAccum, timeSteps = 0, 0
    sendTelemetry()
  end
end

-- the FULL modes: where the tyre thinks the ground and its contact are. White cross and grey line: the ground under the hub (what the
-- height over the ground is measured from); yellow cross and cyan line: the contact point the force acts at (it differs from the point
-- under the hub on a step, a kerb or a slope: the enveloping contact); orange ring: the tyre's free radius about the hub.
local function drawFullContacts(refPos)
  local dd = obj.debugDrawProxy
  local white, grey, yellow, cyan, orange = color(255, 255, 255, 255), color(150, 150, 150, 255), color(255, 220, 0, 255), color(0, 230, 230, 255), color(255, 140, 0, 255)
  local function cross(p, r, col)
    dd:drawLine(p - vec3(r, 0, 0), p + vec3(r, 0, 0), col)
    dd:drawLine(p - vec3(0, r, 0), p + vec3(0, r, 0), col)
    dd:drawLine(p - vec3(0, 0, r), p + vec3(0, 0, r), col)
  end
  for _, s in ipairs(W) do
    if s.hubx and s.pgz and s.pwx then
      local hub = vec3(s.hubx + refPos.x, s.huby + refPos.y, s.hubz + refPos.z)
      local gp = vec3(s.pwx, s.pwy, s.pgz)
      cross(gp, 0.05, white)
      dd:drawLine(hub, gp, grey)
      if s.axx then
        -- two axes in the plane of the wheel
        local ux, uy, uz = s.axy, -s.axx, 0
        local ul = sqrt(ux * ux + uy * uy)
        if ul < 1e-3 then ux, uy, uz, ul = 0, s.axz, -s.axy, sqrt(s.axz * s.axz + s.axy * s.axy) end
        ux, uy, uz = ux / ul, uy / ul, uz / ul
        local wx, wy, wz = s.axy * uz - s.axz * uy, s.axz * ux - s.axx * uz, s.axx * uy - s.axy * ux
        local R, prev = s.radius, nil
        for k = 0, 24 do
          local a = k * 2 * math.pi / 24
          local c, sn = math.cos(a) * R, math.sin(a) * R
          local pt = hub + vec3(ux * c + wx * sn, uy * c + wy * sn, uz * c + wz * sn)
          if prev then dd:drawLine(prev, pt, orange) end
          prev = pt
        end
      end
      if s.patchCount > 0 and s.cx then
        local cp = vec3(s.cx + refPos.x, s.cy + refPos.y, s.cz + refPos.z)
        cross(cp, 0.07, yellow)
        dd:drawLine(hub, cp, cyan)
      end
    end
  end
end

local function onDebugDraw(focusPos)
  if not enabled or not P.debugDraw then return end
  local refPos = obj:getPosition()
  if fullIsOn then drawFullContacts(refPos) end
  for _, s in ipairs(W) do
    if s.patchCount > 0 and s.Fz > P.minFz then
      local base = vec3(s.cx + refPos.x, s.cy + refPos.y, s.cz + refPos.z)
      local scale = 1 / 4000 -- 1 m per 4 kN
      local fvec = vec3((s.fx_ * s.Fx + s.lx_ * s.Fy) * scale, (s.fy_ * s.Fx + s.ly_ * s.Fy) * scale, (s.fz_ * s.Fx + s.lz_ * s.Fy) * scale)
      local col = s.sliding and color(255, 60, 40, 255) or color(40, 220, 60, 255)
      obj.debugDrawProxy:drawLine(base, base + fvec, col)
      local nvec = vec3(s.nx, s.ny, s.nz) * (s.Fz * scale)
      obj.debugDrawProxy:drawLine(base, base + nvec, color(60, 120, 255, 255))
    end
  end
end

local function onReset()
  for _, s in ipairs(W) do s.dx, s.dy, s.patchCount, s.rbrState, s.age, s.pacState = 0, 0, 0, nil, 0, nil; s.advT, s.advWear, s.brushState = nil, nil, nil end
  -- a reset reapplies the jbeam friction, so turn ours back on
  if enabled then setNativeFriction(P.nativeFriction) end
end

local function onExtensionLoaded()
  timer = HighPerfTimer and HighPerfTimer() or nil
  return true
end

local function onExtensionUnloaded()
  if enabled and initDone then setNativeFriction(1) end
  enabled = false
end

M.enable = enable
M.disable = disable
M.setParams = setParams
M.getParams = getParams
M.isEnabled = function() return enabled end
-- Called from the game side with the ground models under the wheels: {kind = "terrain"|"none", names = {...}}
M.setGround = function(info)
  if type(info) ~= "table" then return end
  groundKind = info.kind or "unknown"
  for i, s in ipairs(W) do
    s.groundName = info.names and info.names[i] or nil
  end
end
-- Called from the game side with the tyre the player picked (a table built by svj.lua).
M.setSvjTyre = function(tyre)
  if type(tyre) ~= "table" or type(tyre.lateral) ~= "table" or type(tyre.longitudinal) ~= "table" then
    log("E", "modelTire", "setSvjTyre: the tyre has no lateral/longitudinal Pacejka group")
    return
  end
  svjTyre = pac.prepare(tyre)
  for _, s in ipairs(W) do s.pacState = nil end
  log("I", "modelTire", "SVJ tyre: " .. tostring(tyre.name) .. " (" .. tostring(tyre.modelName) .. ")")
end
-- Called from the game side with the player's own tyres (rbrtyres/*.json), a list of {name, base, set}.
M.setRbrCustom = function(list)
  customTyres = {}
  if type(list) == "table" then
    for _, c in ipairs(list) do
      if type(c) == "table" and type(c.name) == "string" and type(c.set) == "table" then
        customTyres[#customTyres + 1] = {name = c.name, base = c.base or "", set = c.set}
      end
    end
  end
  rbrTyreCache, scaledCache = {}, {}
end
M.setNative = function(on) nativeOn = on and true or false end
-- the advanced brush's tyres back to the air temperature and new
M.resetAdv = function() for _, s in ipairs(W) do s.advT, s.advWear, s.brushState = nil, nil, nil end end
M._wheels = function() return W end

M.onPhysicsStep = onPhysicsStep
M.updateGFX = updateGFX
M.onDebugDraw = onDebugDraw
M.onReset = onReset
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded

return M
