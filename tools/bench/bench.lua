-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- ModelTire offline bench: one wheel and a flat ground, no game. Runs the real wheel code (tyre.lua) against a fake
-- BeamNG vehicle whose hub is moved on a prescribed path, and measures the force the tyre puts on the car.
-- Called by run_bench.py: loadfile(bench.lua)(modDir, svjTyreOrNil, optionsTable) returns CSV text.
--
-- Sweeps (one row per point: mode, sweep, x, Fx, Fy, Fz in N, on the car, x forward, y left, z up):
--   lat   slip angle -12..12 deg at the nominal load, no slip ratio
--   lon   slip ratio -0.3..0.3 at zero slip angle
--   load  vertical load 1..7 kN at 6 deg slip angle (lateral grip against load)
local modDir, SVJT, opt = ...
opt = opt or {}
package.path = modDir .. "/lua/vehicle/?.lua;" .. package.path
local out = {}

local V3 = {}; V3.__index = V3
function V3:set(x, y, z) self.x, self.y, self.z = x, y, z end
function vec3(x, y, z) return setmetatable({x = x or 0, y = y or 0, z = z or 0}, V3) end
V3.__add = function(a, b) return vec3(a.x + b.x, a.y + b.y, a.z + b.z) end
V3.__sub = function(a, b) return vec3(a.x - b.x, a.y - b.y, a.z - b.z) end
V3.__mul = function(a, b) if type(a) == "number" then a, b = b, a end return vec3(a.x * b, a.y * b, a.z * b) end
function color() return 0 end
function log() end
function serialize() return "" end
function enablePhysicsStepHook() end

local R = opt.radius or 0.31
local SPEED = opt.speed or 20
local NOMINAL = opt.load or 3500
local nodes, pos = {}, {}
local H = {x = 0, y = 0, z = 0.30}
pos[0] = vec3(0, 0.1, H.z); pos[1] = vec3(0, -0.1, H.z)
nodes[0] = {cid = 0}; nodes[1] = {cid = 1}
local tread, rim = {}, {}
for i = 0, 15 do
  local a = i / 16 * 2 * math.pi
  local cid = 10 + i
  pos[cid] = vec3(R * math.sin(a), (i % 2 == 0) and 0.07 or -0.07, H.z - R * math.cos(a))
  nodes[cid] = {cid = cid, collision = false, frictionCoef = 1.2, slidingFrictionCoef = 1.0, oppositeTreadNodeCid = 10 + (i + 8) % 16}
  tread[#tread + 1] = cid
  local rc = 40 + i
  pos[rc] = vec3(0.2 * math.sin(a), (i % 2 == 0) and 0.09 or -0.09, H.z - 0.2 * math.cos(a))
  nodes[rc] = {cid = rc}
  rim[#rim + 1] = rc
end
v = {data = {nodes = nodes, pressureGroups = {pg = 7}}}

local applied = {}
local vx, vy = SPEED, 0
GROUND_Z = 0
FZ = function() return NOMINAL end
local wheelObj = {getSlipVelEnergyDownPeakForceDepthMats = function() return 0, 0, 0, FZ(), 0, 0, 4, 12 end}
obj = {
  getNodePosition = function(_, cid) local p = pos[cid]; return vec3(p.x, p.y, p.z) end,
  getNodeVelocityVector = function() return vec3(vx, vy, 0) end,
  getDirectionVector = function() return vec3(1, 0, 0) end,
  getDirectionVectorUp = function() return vec3(0, 0, 1) end,
  getPosition = function() return vec3(0, 0, 0) end,
  applyForceVector = function(_, cid, f) applied[#applied + 1] = {cid = cid, f = f} end,
  setNodeFrictionSlidingCoefs = function() end,
  getNodeMass = function() return 20 end,
  getGroupPressure = function() return 101325 + 220000 end,
  isNodeColliding = function() return true end,
  getSurfaceHeightBelow = function() return GROUND_Z end,
  getId = function() return 1 end,
  queueGameEngineLua = function() end,
  debugDrawProxy = {drawLine = function() end},
}
mapmgr = {surfaceNormalBelow = function() return vec3(0, 0, 1) end}
particles = {getMaterialsParticlesTable = function() return {[4] = {name = "RUBBER"}, [12] = {name = opt.material or "ASPHALT"}} end}
local wd = {rotatorType = "wheel", hasTire = true, treadNodes = tread, nodes = rim, obj = wheelObj, name = "FL",
  node1 = 0, node2 = 1, radius = R, dynamicRadius = R, wheelDir = 1, pressureGroup = "pg", angularVelocity = 0, wheelSpeed = 0}
wheels = {wheelRotators = {[0] = wd}}

local MODES = {
  brush = {useRbr102 = false},
  rbr_light = {useRbr102 = true, lightModel = "rbr", rbrUseRbrData = false},
  rbr_full = {useRbr102 = true, fullTyre = true, fullModel = "rbr", rbrUseRbrData = false, fullAutoSurface = false,
    fullSurface = "Tarmac_Fine_Clean_Dry"},
  svj_light = {useRbr102 = true, lightModel = "svj", svjGrip = 1.0, svjGripScale = 1.0},
  svj_full = {useRbr102 = true, fullTyre = true, fullModel = "svj", svjGrip = 1.0, fullAutoSurface = false},
}
local isFull = {rbr_full = true, svj_full = true}

local M = dofile(modDir .. "/lua/vehicle/extensions/modelTire/tyre.lua")
M.onExtensionLoaded()
if SVJT then M.setSvjTyre(SVJT) end

-- one step of the rig; returns the summed force on the car
local function step()
  applied = {}
  M.onPhysicsStep(0.0005)
  local fx, fy, fz = 0, 0, 0
  for _, a in ipairs(applied) do fx, fy, fz = fx + a.f.x, fy + a.f.y, fz + a.f.z end
  return fx, fy, fz
end

local function setKinematics(alpha, kappa)
  vx, vy = SPEED * math.cos(alpha), SPEED * math.sin(alpha)
  wd.angularVelocity = (1 + kappa) * vx / R
  wd.wheelSpeed = wd.angularVelocity * R
end

-- hub height (over the ground) that gives a vertical load, from a calibration at zero slip (full modes only)
local calib
local function calibrate(mode)
  calib = {}
  setKinematics(0, 0)
  local pen = 0.0001
  while pen < 0.06 do
    GROUND_Z = H.z - (R - pen)
    local sum, n = 0, 0
    for k = 1, 700 do
      local _, _, fz = step()
      if k > 600 then sum, n = sum + fz, n + 1 end
    end
    calib[#calib + 1] = {h = R - pen, fz = sum / n}
    if sum / n > 9000 then break end
    pen = pen * 1.35
  end
end
local function groundFor(load)
  for i = 1, #calib - 1 do
    local a, b = calib[i], calib[i + 1]
    if load <= b.fz or i == #calib - 1 then
      local u = (load - a.fz) / math.max(b.fz - a.fz, 1e-6)
      return H.z - (a.h + (b.h - a.h) * math.max(0, math.min(1, u)))
    end
  end
end

local STEPS, AVG = opt.steps or 1500, 200
local function measure(mode, alpha, kappa, load)
  if isFull[mode] then GROUND_Z = groundFor(load) else FZ = function() return load end end
  setKinematics(alpha, kappa)
  local sx, sy, sz = 0, 0, 0
  for k = 1, STEPS do
    local fx, fy, fz = step()
    if k > STEPS - AVG then sx, sy, sz = sx + fx, sy + fy, sz + fz end
  end
  if opt.debug then local ps = M._wheels()[1].pacState; if ps then out[#out + 1] = string.format('%s,dbg,alpha %.4f kappa %.4f lam %s lamK %s fz0 %s Fz %.0f Dy %.0f Kya %.0f', mode, ps.alpha or 0, ps.kappa or 0, tostring(ps.lam), tostring(ps.lamK), tostring(ps.fz0), ps.normalForce or 0, ps.out.Dy or 0, ps.out.Kya or 0) end end
  -- light modes: BeamNG's own tyre carries the load, so no vertical force comes from the model
  if not isFull[mode] then sz = load * AVG end
  return sx / AVG, sy / AVG, sz / AVG
end

local function row(mode, sweep, x, fx, fy, fz)
  out[#out + 1] = string.format("%s,%s,%.5f,%.1f,%.1f,%.1f", mode, sweep, x, fx, fy, fz)
end

local list = opt.modes or {"brush", "rbr_light", "rbr_full", "svj_light", "svj_full"}
for _, mode in ipairs(list) do
  local cfg = MODES[mode]
  local skip = (mode:find("svj") and not SVJT)
  if cfg and not skip then
    -- setParams merges: put the keys the modes differ in back to their defaults first
    local P = {useRbr102 = false, fullTyre = false, lightModel = "rbr", fullModel = "rbr", rbrUseRbrData = false,
      fullAutoSurface = true, svjGrip = 1.0, svjGripScale = 1.0}
    for k, val in pairs(cfg) do P[k] = val end
    for k, val in pairs(opt.params or {}) do P[k] = val end
    P.fullFadeIn = 0
    -- the rig's wheel has no meaningful static load: take the nominal load as the Pacejka tyre's reference load, so the
    -- bench is comparable with the theoretical curve evaluated at that load
    if mode:find("svj") and not P.svjFz0 then P.svjFz0 = NOMINAL end
    M.disable()
    M.setParams(P)
    M.enable()
    if isFull[mode] then calibrate(mode) end
    if isFull[mode] and opt.debug then for _, c in ipairs(calib) do row(mode, "calib", c.h, 0, 0, c.fz) end end
    -- slip sweeps at every load of the list (the Pacejka reference load stays the nominal one, as on a real tyre)
    for _, load in ipairs(opt.loads or {NOMINAL}) do
      for deg = -20, 20, 1 do
        local fx, fy, fz = measure(mode, math.rad(deg), 0, load)
        row(mode, string.format("lat@%d", load), deg, fx, fy, fz)
      end
      -- finer steps at low slip, where the force builds up, out to 0.5
      local kmag = {0, 0.01, 0.02, 0.03, 0.045, 0.06, 0.08, 0.1, 0.13, 0.17, 0.22, 0.3, 0.4, 0.5}
      local ks = {}
      for i = #kmag, 2, -1 do ks[#ks + 1] = -kmag[i] end
      for i = 1, #kmag do ks[#ks + 1] = kmag[i] end
      for _, kk in ipairs(ks) do
        local fx, fy, fz = measure(mode, 0, kk, load)
        row(mode, string.format("lon@%d", load), kk, fx, fy, fz)
      end
    end
    for load = 1000, 7000, 1000 do
      local fx, fy, fz = measure(mode, math.rad(6), 0, load)
      row(mode, "load", load, fx, fy, fz)
    end
  end
end
return table.concat(out, "\n")
