-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- ModelTire: controller of the single-wheel rig (vehicle "modelTire_wheelrig"). Extension "modelTire_wheelrig".
-- Licence: GPL-3.0-or-later.
--
-- One test wheel on a carrier that turns about a vertical line through the wheel centre (a kingpin without scrub or caster), inside
-- a light frame that has no wheel of its own. Everything that holds the rig up and moves it is a force from this controller, so
-- every external force on the rig except the tyre's is known, and the tyre's follows from Newton:
--   * Load. The frame's four lower corners (the force nodes "sv_*", at ground height, no collision) get a vertical force that is the
--     rig's weight minus the wheel load that was asked for (plus damping for the bounce on the tyre's stiffness). The wheel carries
--     exactly that load, whatever the rig weighs and however the ground moves.
--   * Balance. The same four nodes get virtual vertical springs about their mean height: pure couples that keep the frame level
--     (a single wheel cannot).
--   * Speed. A velocity servo (PI) on the centre of mass drives the rig straight at the set speed; a yaw hold keeps its heading.
--   * Slip angle. The carrier is turned against the direction of travel by a PID torque on its four outer nodes (reaction on the
--     frame). The torque the servo needs to hold the wheel is the tyre's aligning moment: Mz = Jc * (carrier angular acceleration) -
--     applied torque, restoring (towards the direction of travel) positive, as on a tyre machine's steering torque cell.
--   * Forces. Fx, Fy (in the wheel's axes, + to the left) and Fz: tyre force = m * a - the known servo forces, over each report window.
--
-- Phases: stand (holds itself up and still, from the moment the extension loads), settle (speed up), hold (1.5 s), ramp (slip angle
-- 0 -> max), done (slows down). stop() goes back to stand: the rig must never be left without its controller, it would fall over.

local M = {}

local g = 9.81
local C = {KP = 8, KI = 10, A_MAX = 30, KPY = 60, KDY = 14, ACCEL = 2, DECEL = 3, HOLD = 1.5, LOAD_RATE = 2500, TYRE_K = 2.0e5, VZ_ZETA = 0.7, TILT_HZ = 4, TILT_ZETA = 0.7, STEER_HZ = 4, STEER_ZETA = 0.8, STEER_KI = 2, TAU_MAX = 600, WHEEL_YAW_INERTIA = 1.0}
local CARRIER = {Wi = true, Wo = true, c1 = true, c2 = true, c3 = true, c4 = true}

-- the module's state lives in one table: a Lua function may capture at most 60 upvalues
local S = {phase = "off", heavyMass = 0, totalMass = 1, yawInertia = 1, heavyInertia = 1, Jc = 1, steerKp = 1, steerKd = 1, steerKi = 1, tiltK = 0, tiltC = 0,
  cmd = {speed = 15, maxSlip = 20, ramp = 20}, loadSet = 3500, tPhase = 0, hx = 1, hy = 0, psi0 = 0, iex = 0, iey = 0, ieS = 0, omega = 0, slipNow = 0, vcmd = 0,
  aSx = 0, aSy = 0, aSz = 0, aN = 0, aT = 0, aTau = 0, curVx = 0, curVy = 0, curVz = 0, curPsiC = 0, curWc = 0,
  lastFx = 0, lastFy = 0, lastFz = 0, lastMz = 0, lastSpeed = 0, lastTau = 0, lastSteer = 0, logT = 0}


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
  if p ~= S.phase then log("I", "modelTire_wheelrig", string.format("phase %s -> %s (speed %.2f m/s)", S.phase, p, S.lastSpeed)) end
  S.phase = p
end

-- the controller's constants from the rig's nodes (after the vehicle exists)
local function collect()
  S.servo, S.carrier, S.heavy = {}, {}, {}
  S.heavyMass, S.totalMass = 0, 0
  local all = {}
  for _, nd in pairs(v.data.nodes) do
    local m = obj:getNodeMass(nd.cid) or 0
    local name = tostring(nd.name or "")
    S.totalMass = S.totalMass + m
    all[#all + 1] = {cid = nd.cid, m = m}
    if name:match("^sv_") then S.servo[#S.servo + 1] = nd.cid end
    if CARRIER[name] then S.carrier[#S.carrier + 1] = {cid = nd.cid, m = m} end
    if name == "c1" then S.cidC1 = nd.cid elseif name == "c2" then S.cidC2 = nd.cid elseif name == "kb" then S.cidKB = nd.cid end
    if m >= 5 then S.heavy[#S.heavy + 1] = {cid = nd.cid, m = m}; S.heavyMass = S.heavyMass + m end
  end
  if #S.servo ~= 4 or #S.carrier ~= 6 or not (S.cidC1 and S.cidC2 and S.cidKB) then
    log("E", "modelTire_wheelrig", string.format("the rig's nodes are not as expected (%d force nodes, %d carrier nodes, c1 %s, c2 %s, kb %s)", #S.servo, #S.carrier,
      tostring(S.cidC1), tostring(S.cidC2), tostring(S.cidKB)))
    S.servo = nil
    S.phase = "off"
    return false
  end
  -- yaw inertia about the centre of mass (all nodes; for the yaw hold) and of the structural nodes (for the angular rate)
  local comx, comy = 0, 0
  for _, a in ipairs(all) do
    local p = obj:getNodePosition(a.cid)
    a.x, a.y, a.z = p.x, p.y, p.z
    comx, comy = comx + p.x * a.m, comy + p.y * a.m
  end
  comx, comy = comx / math.max(S.totalMass, 1e-6), comy / math.max(S.totalMass, 1e-6)
  S.yawInertia = 0
  for _, a in ipairs(all) do S.yawInertia = S.yawInertia + a.m * ((a.x - comx) ^ 2 + (a.y - comy) ^ 2) end
  S.heavyInertia = 0
  local hmx, hmy = 0, 0
  for _, h in ipairs(S.heavy) do
    local p = obj:getNodePosition(h.cid)
    h.x, h.y = p.x, p.y
    hmx, hmy = hmx + p.x * h.m, hmy + p.y * h.m
  end
  hmx, hmy = hmx / math.max(S.heavyMass, 1e-6), hmy / math.max(S.heavyMass, 1e-6)
  for _, h in ipairs(S.heavy) do S.heavyInertia = S.heavyInertia + h.m * ((h.x - hmx) ^ 2 + (h.y - hmy) ^ 2) end
  -- the S.carrier's inertia about the steering axis (the line through the kingpin) and the steering S.servo's gains
  local pkb = obj:getNodePosition(S.cidKB)
  S.Jc = C.WHEEL_YAW_INERTIA
  for _, c in ipairs(S.carrier) do
    local p = obj:getNodePosition(c.cid)
    S.Jc = S.Jc + c.m * ((p.x - pkb.x) ^ 2 + (p.y - pkb.y) ^ 2)
  end
  local ws = 2 * math.pi * C.STEER_HZ
  S.steerKp, S.steerKd, S.steerKi = S.Jc * ws * ws, 2 * C.STEER_ZETA * S.Jc * ws, S.Jc * ws * ws * C.STEER_KI
  -- the level hold: virtual vertical springs on the four force nodes, stiff enough for C.TILT_HZ about the wheel's contact point
  local Jt, sr2 = 0, 0
  local cx, cy = 0, 0
  for i = 1, #S.servo do
    local p = obj:getNodePosition(S.servo[i])
    cx, cy = cx + p.x / #S.servo, cy + p.y / #S.servo
  end
  for i = 1, #S.servo do
    local p = obj:getNodePosition(S.servo[i])
    sr2 = sr2 + (p.x - cx) ^ 2 + (p.y - cy) ^ 2
  end
  sr2 = 0.5 * sr2                                  -- sum of rx^2 (= sum of ry^2 for a rectangle), the torque stiffness is K * sr2
  local Jx, Jy = 0, 0
  for _, a in ipairs(all) do
    Jx = Jx + a.m * ((a.y - pkb.y) ^ 2 + (a.z - pkb.z) ^ 2)      -- (positions are relative to the reference node: heights from the lower pivot,
    Jy = Jy + a.m * ((a.x - pkb.x) ^ 2 + (a.z - pkb.z) ^ 2)      -- which is at ground height)
  end
  Jt = math.max(Jx, Jy)
  local wt = 2 * math.pi * C.TILT_HZ
  S.tiltK = Jt * wt * wt / math.max(sr2, 1e-6)
  S.tiltC = 2 * C.TILT_ZETA * Jt * wt / math.max(sr2, 1e-6)
  log("I", "modelTire_wheelrig", string.format("%d nodes, mass %.1f kg, yaw inertia %.0f, carrier inertia %.1f kg m^2 (steering Kp %.0f, Kd %.0f N m/rad), level hold %.0f N/m, %.0f N s/m",
    #all, S.totalMass, S.yawInertia, S.Jc, S.steerKp, S.steerKd, S.tiltK, S.tiltC))
  return true
end

local function captureHeading()
  local d = obj:getDirectionVector()
  local l = math.sqrt(d.x * d.x + d.y * d.y)
  S.hx, S.hy = d.x / l, d.y / l
  S.psi0 = math.atan2(S.hy, S.hx)
  S.prevPsi, S.omega = nil, 0
end

local function resetState()
  S.iex, S.iey, S.ieS = 0, 0, 0
  S.slipNow, S.vcmd = 0, 0
  S.aSx, S.aSy, S.aSz, S.aN, S.aT, S.aTau = 0, 0, 0, 0, 0, 0
  S.repVx, S.repVy, S.repVz, S.repWc = nil, nil, nil, nil
  S.tPhase = 0
  S.loadNow = nil
end

local function stand()
  resetState()
  if S.servo then captureHeading() end
  if enablePhysicsStepHook then enablePhysicsStepHook() end     -- the game only calls onPhysicsStep after this has been asked for
  setPhase("stand")
end

local function stop()
  -- not "off": without the controller the rig falls over. Slow down, straighten the wheel and stand.
  if S.phase == "off" then return stand() end
  setPhase("stand")
  S.slipNow = 0
end

local function run(speedKmh, maxSlipDeg, rampSec, loadN)
  if not S.servo then collect() end
  if not S.servo then return end
  S.cmd.speed = (speedKmh or 55) / 3.6
  S.cmd.maxSlip = maxSlipDeg or 20
  S.cmd.ramp = math.max(rampSec or 20, 2)
  if loadN and loadN > 0 then S.loadSet = loadN end
  captureHeading()
  S.iex, S.iey, S.ieS = 0, 0, 0
  S.slipNow, S.vcmd = 0, 0
  S.aSx, S.aSy, S.aSz, S.aN, S.aT, S.aTau = 0, 0, 0, 0, 0, 0
  S.repVx, S.repVy, S.repVz, S.repWc = nil, nil, nil, nil
  S.tPhase = 0
  log("I", "modelTire_wheelrig", string.format("run: %.1f m/s, slip to %.0f deg over %.0f s, load %.0f N, heading (%.2f, %.2f)", S.cmd.speed, S.cmd.maxSlip, S.cmd.ramp, S.loadSet, S.hx, S.hy))
  if enablePhysicsStepHook then enablePhysicsStepHook() end
  setPhase("settle")
end

local function setLoad(n)
  if n and n > 0 then S.loadSet = n end
end

local function onPhysicsStep(dt)
  if S.phase == "off" or dt <= 0 then return end
  if not S.servo then
    if not collect() then return end
    captureHeading()
  end
  S.tPhase = S.tPhase + dt

  -- state of the structural nodes: centre of mass position and velocity
  local vx, vy, vz, px, py, pz = 0, 0, 0, 0, 0, 0
  for i = 1, #S.heavy do
    local h = S.heavy[i]
    local nv = obj:getNodeVelocityVector(h.cid)
    local p = obj:getNodePosition(h.cid)
    h.vx, h.vy, h.px, h.py = nv.x, nv.y, p.x, p.y
    vx, vy, vz = vx + nv.x * h.m, vy + nv.y * h.m, vz + nv.z * h.m
    px, py = px + p.x * h.m, py + p.y * h.m
  end
  vx, vy, vz = vx / S.heavyMass, vy / S.heavyMass, vz / S.heavyMass
  local comx, comy = px / S.heavyMass, py / S.heavyMass
  local speedNow = math.sqrt(vx * vx + vy * vy)
  if not S.loadNow then S.loadNow = S.totalMass * g end

  -- phases
  local lx, ly = -S.hy, S.hx
  local alphaDot = 0
  if S.phase == "stand" then
    S.vcmd = math.max(0, S.vcmd - C.DECEL * dt)
  elseif S.phase == "settle" then
    S.vcmd = math.min(S.cmd.speed, S.vcmd + C.ACCEL * dt)
    if S.vcmd >= S.cmd.speed and speedNow > S.cmd.speed - 0.3 then setPhase("hold"); S.tPhase = 0 end
  elseif S.phase == "hold" then
    S.vcmd = S.cmd.speed
    if S.tPhase >= C.HOLD then setPhase("ramp"); S.tPhase = 0 end
  elseif S.phase == "ramp" then
    S.vcmd = S.cmd.speed
    local u = math.min(S.tPhase / S.cmd.ramp, 1)
    S.slipNow = math.rad(S.cmd.maxSlip) * u
    alphaDot = (u < 1) and math.rad(S.cmd.maxSlip) / S.cmd.ramp or 0
    if S.tPhase >= S.cmd.ramp + 0.5 then setPhase("done") end
  elseif S.phase == "done" then
    S.vcmd = math.max(0, S.vcmd - C.DECEL * dt)
  end

  -- horizontal: the rig goes straight at S.vcmd along its heading (the slip angle is made by turning the wheel, not the rig)
  local tvx, tvy = S.vcmd * S.hx, S.vcmd * S.hy
  local tax, tay = 0, 0
  if S.phase == "settle" then tax, tay = S.hx * C.ACCEL, S.hy * C.ACCEL end
  local ex, ey = tvx - vx, tvy - vy
  S.iex = math.max(-C.A_MAX / C.KI, math.min(C.A_MAX / C.KI, S.iex + ex * dt))
  S.iey = math.max(-C.A_MAX / C.KI, math.min(C.A_MAX / C.KI, S.iey + ey * dt))
  local ax = math.max(-C.A_MAX, math.min(C.A_MAX, C.KP * ex + C.KI * S.iex + tax))
  local ay = math.max(-C.A_MAX, math.min(C.A_MAX, C.KP * ey + C.KI * S.iey + tay))

  -- frame yaw hold
  local psi = headingNow()
  if S.prevPsi then S.omega = 0.9 * S.omega + 0.1 * (wrap(psi - S.prevPsi) / dt) end
  S.prevPsi = psi
  local yawTorque = -S.yawInertia * (C.KPY * wrap(psi - S.psi0) + C.KDY * S.omega)

  -- the S.carrier: its heading from the front and rear nodes, its angular rate about the kingpin from the nodes' momentum
  local pc1, pc2 = obj:getNodePosition(S.cidC1), obj:getNodePosition(S.cidC2)
  local psiC = math.atan2(pc1.y - pc2.y, pc1.x - pc2.x)
  local pkb, vkb = obj:getNodePosition(S.cidKB), obj:getNodeVelocityVector(S.cidKB)
  local Lc, I2 = 0, 0
  for i = 1, #S.carrier do
    local c = S.carrier[i]
    local p, nv = obj:getNodePosition(c.cid), obj:getNodeVelocityVector(c.cid)
    local rx, ry = p.x - pkb.x, p.y - pkb.y
    Lc = Lc + c.m * (rx * (nv.y - vkb.y) - ry * (nv.x - vkb.x))
    I2 = I2 + c.m * (rx * rx + ry * ry)
    c.px, c.py = p.x, p.y
  end
  local wC = Lc / math.max(I2, 1e-6)
  -- steering PID: the wheel points S.slipNow to the right of the direction of travel
  local target = S.psi0 - S.slipNow
  local eS = wrap(psiC - target)
  S.ieS = math.max(-C.TAU_MAX / S.steerKi, math.min(C.TAU_MAX / S.steerKi, S.ieS + eS * dt))
  local tau = -(S.steerKp * eS + S.steerKd * (wC + alphaDot) + S.steerKi * S.ieS)
  tau = math.max(-C.TAU_MAX, math.min(C.TAU_MAX, tau))
  -- the torque on the S.carrier's four outer nodes (a couple: they are placed symmetrically about the kingpin)...
  local r2c = 0
  for i = 1, #S.carrier do
    local c = S.carrier[i]
    if c.m > 0 and not (c.px == nil) then
      c.rx, c.ry = c.px - pkb.x, c.py - pkb.y
      r2c = r2c + c.rx * c.rx + c.ry * c.ry
    end
  end
  for i = 1, #S.carrier do
    local c = S.carrier[i]
    obj:applyForceVector(c.cid, vec3(-tau * c.ry / r2c, tau * c.rx / r2c, 0))
  end

  -- the force nodes: speed S.servo, yaw hold (with the steering torque's reaction), load and level
  local n = #S.servo
  local pos, vel = {}, {}
  local cx, cy, zbar, vbar = 0, 0, 0, 0
  for i = 1, n do
    pos[i], vel[i] = obj:getNodePosition(S.servo[i]), obj:getNodeVelocityVector(S.servo[i])
    cx, cy = cx + pos[i].x, cy + pos[i].y
    zbar, vbar = zbar + pos[i].z, vbar + vel[i].z
  end
  cx, cy, zbar, vbar = cx / n, cy / n, zbar / n, vbar / n
  local r2 = 0
  for i = 1, n do r2 = r2 + (pos[i].x - cx) ^ 2 + (pos[i].y - cy) ^ 2 end
  local kT = (r2 > 1e-6) and (yawTorque - tau) / r2 or 0
  -- vertical: the rig's weight minus the wheel load, damped
  S.loadNow = S.loadNow + math.max(-C.LOAD_RATE * dt, math.min(C.LOAD_RATE * dt, S.loadSet - S.loadNow))
  local Cz = 2 * C.VZ_ZETA * math.sqrt(C.TYRE_K * S.totalMass)
  local Sz = S.totalMass * g - S.loadNow - Cz * vz
  local fx, fy, fz = S.totalMass * ax / n, S.totalMass * ay / n, Sz / n
  for i = 1, n do
    local rx, ry = pos[i].x - cx, pos[i].y - cy
    local level = -S.tiltK * (pos[i].z - zbar) - S.tiltC * (vel[i].z - vbar)
    obj:applyForceVector(S.servo[i], vec3(fx - kT * ry, fy + kT * rx, fz + level))
  end

  -- window sums for the report
  S.aSx, S.aSy, S.aSz, S.aN, S.aT, S.aTau = S.aSx + S.totalMass * ax, S.aSy + S.totalMass * ay, S.aSz + Sz, S.aN + 1, S.aT + dt, S.aTau + tau
  S.curVx, S.curVy, S.curVz, S.curPsiC, S.curWc = vx, vy, vz, psiC, wC
  if not S.repVx then S.repVx, S.repVy, S.repVz, S.repWc = vx, vy, vz, wC end
  S.lastSpeed, S.lastTau, S.lastSteer = speedNow, tau, math.deg(wrap(S.psi0 - psiC))
  S.logT = S.logT + dt
  if S.logT >= 1 then
    S.logT = 0
    log("I", "modelTire_wheelrig", string.format("%s: speed %.2f m/s (target %.2f), slip %.1f deg (wheel %.1f), steer torque %.1f N m, load target %.0f N",
      S.phase, S.lastSpeed, S.vcmd, math.deg(S.slipNow), S.lastSteer, S.lastTau, S.loadNow))
  end
end

-- the mean forces since the last report
local function report()
  if S.aN > 0 and S.aT > 0 and S.repVx then
    local Fwx = S.totalMass * (S.curVx - S.repVx) / S.aT - S.aSx / S.aN
    local Fwy = S.totalMass * (S.curVy - S.repVy) / S.aT - S.aSy / S.aN
    local hdx, hdy = math.cos(S.curPsiC), math.sin(S.curPsiC)
    S.lastFx, S.lastFy = Fwx * hdx + Fwy * hdy, -Fwx * hdy + Fwy * hdx
    S.lastFz = S.totalMass * (S.curVz - S.repVz) / S.aT + S.totalMass * g - S.aSz / S.aN
    S.lastMz = S.Jc * (S.curWc - S.repWc) / S.aT - S.aTau / S.aN
    S.repVx, S.repVy, S.repVz, S.repWc = S.curVx, S.curVy, S.curVz, S.curWc
  end
  S.aSx, S.aSy, S.aSz, S.aN, S.aT, S.aTau = 0, 0, 0, 0, 0, 0
  return {phase = S.phase, Mz = S.lastMz, slip = math.deg(S.slipNow), speed = S.lastSpeed, Fx = S.lastFx, Fy = S.lastFy, Fz = S.lastFz,
    load = (S.lastFz > 100) and S.lastFz or S.loadSet, maxSlip = S.cmd.maxSlip, ramp = S.cmd.ramp, steer = S.lastSteer, rigKind = "wheel"}
end

local function onReset()
  S.servo = nil
  S.loadNow = nil
  resetState()
  S.phase = "stand"
end

local function onExtensionLoaded()
  stand()
end

M.run = run
M.stop = stop
M.setLoad = setLoad
M.report = report
M.onPhysicsStep = onPhysicsStep
M.onReset = onReset
M.onExtensionLoaded = onExtensionLoaded

return M
