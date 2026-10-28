-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- ModelTire: controller of the test sled (vehicle "modelTire_rig"). Extension "modelTire_rig".
-- Licence: GPL-3.0-or-later.
--
-- The sled is a box on four identical wheels, each with its own trailing arm, spring and damper. This controller moves it at a
-- set speed along a set direction and holds its heading, so the angle between the wheels and the direction of travel is the
-- slip angle, chosen by the script: a ramp from 0 to a maximum. It does that with forces on four force nodes at ground level (a
-- velocity servo, PI, plus a yaw hold that is a pure couple). The servo force acts at the height of the tyres' contact patches,
-- so the sled gets no roll or pitch couple from it and the wheels keep equal loads whatever the height of the centre of gravity.
-- Whatever force the tyres put on the sled, the servo has to supply the opposite, so the servo force is a measurement of the
-- total tyre force that works for any tyre, BeamNG's own included:
--     tyre force = m * a - servo force      (a is the measured acceleration of the centre of mass over the report window)
-- Body frame, forces reported on the sled: Fx along the wheels' heading, Fy sideways (+ to the left).
--
-- Phases: settle (speed up to the target, slip 0), hold (1.5 s, the load settles), ramp (slip 0 -> max), done.

local M = {}

local g = 9.81
local KP, KI = 8, 10            -- velocity servo: acceleration (m/s^2) per m/s of error, and per m of travelled error
                                -- (slow on purpose: the sprung mass bounces at about 2 Hz and a faster loop would excite it)
local A_MAX = 30                -- servo acceleration limit (m/s^2), above any tyre
local KPY, KDY = 60, 14         -- yaw hold (rad/s^2 per rad, per rad/s)
local ACCEL = 2                 -- acceleration of the settle phase (m/s^2)
local HOLD = 1.5

local phase = "off"
local servoNodes = nil          -- the cids of the force nodes ("sv_*" in the jbeam)
local heavyNodes, heavyMass = nil, 0   -- {cid, m} of the structural nodes: their mean velocity is the sled's centre of mass velocity
local totalMass, yawInertia = 0, 1
local cmd = {speed = 15, maxSlip = 20, ramp = 20}
local tPhase = 0
local hx, hy, psi0 = 1, 0, 0     -- travel heading at the start (unit vector in the ground plane) and its angle
local iex, iey = 0, 0            -- integral of the velocity error
local prevPsi = nil
local omega = 0
local slipNow = 0                -- commanded slip angle (rad)
local vcmd = 0
-- accumulators for the report
local aSx, aSy, aN, aT = 0, 0, 0, 0   -- servo force (world) sum, steps, window time
local repVx, repVy = 0, 0              -- centre of mass velocity at the last report
local curVx, curVy, curPsi = 0, 0, 0
local lastFx, lastFy = 0, 0
local lastSpeed = 0
local logT = 0

local function collect()
  servoNodes, totalMass = {}, 0
  heavyNodes, heavyMass = {}, 0
  local all, cx, cy = {}, 0, 0
  for _, nd in pairs(v.data.nodes) do
    local m = obj:getNodeMass(nd.cid) or 0
    totalMass = totalMass + m
    all[#all + 1] = {cid = nd.cid, m = m}
    if tostring(nd.name or ""):match("^sv_") then servoNodes[#servoNodes + 1] = nd.cid end
    if m >= 5 then heavyNodes[#heavyNodes + 1] = {cid = nd.cid, m = m}; heavyMass = heavyMass + m end
  end
  local named = #servoNodes
  if named == 0 then
    -- the node names are not available: push every node with the same acceleration instead (exact, but at the wrong height)
    for _, a in ipairs(all) do servoNodes[#servoNodes + 1] = a.cid end
  end
  -- yaw inertia about the centre of mass, for the yaw hold's couple
  local comx, comy = 0, 0
  for _, a in ipairs(all) do
    local p = obj:getNodePosition(a.cid)
    a.x, a.y = p.x, p.y
    comx, comy = comx + p.x * a.m, comy + p.y * a.m
  end
  comx, comy = comx / math.max(totalMass, 1e-6), comy / math.max(totalMass, 1e-6)
  yawInertia = 0
  for _, a in ipairs(all) do yawInertia = yawInertia + a.m * ((a.x - comx) ^ 2 + (a.y - comy) ^ 2) end
  log("I", "modelTire_rig", string.format("%d nodes, mass %.1f kg, yaw inertia %.0f kg m^2, %d force nodes (%s)", #all, totalMass, yawInertia,
    #servoNodes, named > 0 and "by name" or "NAMES NOT FOUND: all nodes"))
end

local function wrap(a)
  while a > math.pi do a = a - 2 * math.pi end
  while a < -math.pi do a = a + 2 * math.pi end
  return a
end

local function headingNow()
  local d = obj:getDirectionVector()
  return math.atan2(d.y, d.x)
end

local function setPhase(p)
  if p ~= phase then log("I", "modelTire_rig", string.format("phase %s -> %s (speed %.2f m/s)", phase, p, lastSpeed)) end
  phase = p
end

local function stop()
  setPhase("off")
  iex, iey = 0, 0
end

local function run(speedKmh, maxSlipDeg, rampSec)
  collect()
  cmd.speed = (speedKmh or 55) / 3.6
  cmd.maxSlip = maxSlipDeg or 20
  cmd.ramp = math.max(rampSec or 20, 2)
  local d = obj:getDirectionVector()
  local l = math.sqrt(d.x * d.x + d.y * d.y)
  hx, hy = d.x / l, d.y / l
  psi0 = math.atan2(hy, hx)
  prevPsi, omega = nil, 0
  iex, iey = 0, 0
  slipNow, vcmd = 0, 0
  aSx, aSy, aN, aT = 0, 0, 0, 0
  repVx, repVy = nil, nil
  tPhase = 0
  log("I", "modelTire_rig", string.format("run: %.1f m/s, slip to %.0f deg over %.0f s, heading (%.2f, %.2f)", cmd.speed, cmd.maxSlip, cmd.ramp, hx, hy))
  -- the game only calls an extension's onPhysicsStep after this has been asked for
  if enablePhysicsStepHook then enablePhysicsStepHook() end
  setPhase("settle")
end

-- every physics step
local function onPhysicsStep(dt)
  if phase == "off" or not servoNodes or dt <= 0 then return end
  tPhase = tPhase + dt
  -- velocity of the centre of mass from the structural nodes (the frame bounces on its springs, a single node would show that)
  local vx, vy = 0, 0
  for i = 1, #heavyNodes do
    local nv = obj:getNodeVelocityVector(heavyNodes[i].cid)
    vx, vy = vx + nv.x * heavyNodes[i].m, vy + nv.y * heavyNodes[i].m
  end
  vx, vy = vx / heavyMass, vy / heavyMass
  local speedNow = math.sqrt(vx * vx + vy * vy)
  local lx, ly = -hy, hx                       -- the left of the travel direction
  local alphaDot = 0
  if phase == "settle" then
    vcmd = math.min(cmd.speed, vcmd + ACCEL * dt)
    if vcmd >= cmd.speed and speedNow > cmd.speed - 0.3 then setPhase("hold"); tPhase = 0 end
  elseif phase == "hold" then
    vcmd = cmd.speed
    if tPhase >= HOLD then setPhase("ramp"); tPhase = 0 end
  elseif phase == "ramp" then
    vcmd = cmd.speed
    local u = math.min(tPhase / cmd.ramp, 1)
    slipNow = math.rad(cmd.maxSlip) * u
    alphaDot = (u < 1) and math.rad(cmd.maxSlip) / cmd.ramp or 0
    if tPhase >= cmd.ramp + 0.5 then setPhase("done") end
  end

  -- target velocity and its derivative: speed along the travel direction rotated by the slip angle
  local ca, sa = math.cos(slipNow), math.sin(slipNow)
  local tvx, tvy = vcmd * (hx * ca + lx * sa), vcmd * (hy * ca + ly * sa)
  local tax, tay = vcmd * alphaDot * (-hx * sa + lx * ca), vcmd * alphaDot * (-hy * sa + ly * ca)
  if phase == "settle" then tax, tay = hx * ACCEL, hy * ACCEL end

  local ex, ey = tvx - vx, tvy - vy
  iex = math.max(-A_MAX / KI, math.min(A_MAX / KI, iex + ex * dt))
  iey = math.max(-A_MAX / KI, math.min(A_MAX / KI, iey + ey * dt))
  local ax = math.max(-A_MAX, math.min(A_MAX, KP * ex + KI * iex + tax))
  local ay = math.max(-A_MAX, math.min(A_MAX, KP * ey + KI * iey + tay))

  -- yaw hold: a couple on the force nodes (equal and opposite forces about their centroid, no net force)
  local psi = headingNow()
  if prevPsi then omega = 0.9 * omega + 0.1 * (wrap(psi - prevPsi) / dt) end
  prevPsi = psi
  local torque = -yawInertia * (KPY * wrap(psi - psi0) + KDY * omega)

  local n = #servoNodes
  local cx, cy = 0, 0
  local pos = {}
  for i = 1, n do
    local p = obj:getNodePosition(servoNodes[i])
    pos[i] = p
    cx, cy = cx + p.x, cy + p.y
  end
  cx, cy = cx / n, cy / n
  local r2 = 0
  for i = 1, n do r2 = r2 + (pos[i].x - cx) ^ 2 + (pos[i].y - cy) ^ 2 end
  local kT = (r2 > 1e-6) and torque / r2 or 0
  local fx, fy = totalMass * ax / n, totalMass * ay / n
  for i = 1, n do
    local rx, ry = pos[i].x - cx, pos[i].y - cy
    obj:applyForceVector(servoNodes[i], vec3(fx - kT * ry, fy + kT * rx, 0))
  end

  -- the tyres' force on the sled: m * a_target - servo force (world), then the sled's own axes (wheels' heading)
  aSx, aSy, aN, aT = aSx + totalMass * ax, aSy + totalMass * ay, aN + 1, aT + dt
  curVx, curVy, curPsi = vx, vy, psi
  if not repVx then repVx, repVy = vx, vy end
  lastSpeed = speedNow
  logT = logT + dt
  if logT >= 1 then
    logT = 0
    log("I", "modelTire_rig", string.format("%s: speed %.2f m/s (target %.2f), slip %.1f deg, servo a (%.2f, %.2f) m/s^2", phase, lastSpeed, vcmd, math.deg(slipNow), ax, ay))
  end
end

-- the mean forces since the last report
local function report()
  if aN > 0 and aT > 0 then
    -- Newton over the window: m * (change of velocity / time) = servo force + tyre force
    local Fwx = totalMass * (curVx - repVx) / aT - aSx / aN
    local Fwy = totalMass * (curVy - repVy) / aT - aSy / aN
    local hdx, hdy = math.cos(curPsi), math.sin(curPsi)
    lastFx, lastFy = Fwx * hdx + Fwy * hdy, -Fwx * hdy + Fwy * hdx
    repVx, repVy = curVx, curVy
  end
  aSx, aSy, aN, aT = 0, 0, 0, 0
  return {phase = phase, slip = math.deg(slipNow), speed = lastSpeed, Fx = lastFx, Fy = lastFy, load = totalMass * g,
    maxSlip = cmd.maxSlip, ramp = cmd.ramp}
end

local function onReset()
  stop()
  servoNodes = nil
end

M.run = run
M.stop = stop
M.report = report
M.onPhysicsStep = onPhysicsStep
M.onReset = onReset

return M
