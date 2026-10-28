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
local KP, KI = 8, 10                -- velocity servo (as the sled's)
local A_MAX = 30                    -- servo acceleration limit (m/s^2)
local KPY, KDY = 60, 14             -- yaw hold of the frame (rad/s^2 per rad, per rad/s)
local ACCEL, DECEL = 2, 3           -- m/s^2
local HOLD = 1.5
local LOAD_RATE = 2500              -- N/s: how fast the load goes from the rig's weight to the set load
local TYRE_K = 2.0e5                -- N/m: assumed vertical stiffness of the tyre, for the damping of the load loop only
local VZ_ZETA = 0.7
local TILT_HZ, TILT_ZETA = 4, 0.7   -- the level hold
local STEER_HZ, STEER_ZETA, STEER_KI = 4, 0.8, 2   -- steering servo: bandwidth, damping, integral gain (per second)
local TAU_MAX = 600                 -- N m
local WHEEL_YAW_INERTIA = 1.0       -- kg m^2: the wheel's own inertia about the steering axis (the carrier's nodes do not include it)

local CARRIER = {Wi = true, Wo = true, c1 = true, c2 = true, c3 = true, c4 = true}

local phase = "off"
local servo, carrier, heavy = nil, nil, nil     -- node lists (cids; carrier and heavy with their masses)
local cidC1, cidC2, cidKB = nil, nil, nil
local heavyMass, totalMass, yawInertia, heavyInertia = 0, 1, 1, 1
local Jc, steerKp, steerKd, steerKi = 1, 1, 1, 1
local tiltK, tiltC = 0, 0
local cmd = {speed = 15, maxSlip = 20, ramp = 20}
local loadSet, loadNow = 3500, nil
local tPhase = 0
local hx, hy, psi0 = 1, 0, 0        -- travel heading and its angle
local iex, iey, ieS = 0, 0, 0
local prevPsi, omega = nil, 0
local slipNow, vcmd = 0, 0
-- accumulators of the report window
local aSx, aSy, aSz, aN, aT, aTau = 0, 0, 0, 0, 0, 0
local repVx, repVy, repVz, repWc = nil, nil, nil, nil
local curVx, curVy, curVz, curPsiC, curWc = 0, 0, 0, 0, 0
local lastFx, lastFy, lastFz, lastMz = 0, 0, 0, 0
local lastSpeed, lastTau, lastSteer = 0, 0, 0
local logT = 0

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
  if p ~= phase then log("I", "modelTire_wheelrig", string.format("phase %s -> %s (speed %.2f m/s)", phase, p, lastSpeed)) end
  phase = p
end

-- the controller's constants from the rig's nodes (after the vehicle exists)
local function collect()
  servo, carrier, heavy = {}, {}, {}
  heavyMass, totalMass = 0, 0
  local all = {}
  for _, nd in pairs(v.data.nodes) do
    local m = obj:getNodeMass(nd.cid) or 0
    local name = tostring(nd.name or "")
    totalMass = totalMass + m
    all[#all + 1] = {cid = nd.cid, m = m}
    if name:match("^sv_") then servo[#servo + 1] = nd.cid end
    if CARRIER[name] then carrier[#carrier + 1] = {cid = nd.cid, m = m} end
    if name == "c1" then cidC1 = nd.cid elseif name == "c2" then cidC2 = nd.cid elseif name == "kb" then cidKB = nd.cid end
    if m >= 5 then heavy[#heavy + 1] = {cid = nd.cid, m = m}; heavyMass = heavyMass + m end
  end
  if #servo ~= 4 or #carrier ~= 6 or not (cidC1 and cidC2 and cidKB) then
    log("E", "modelTire_wheelrig", string.format("the rig's nodes are not as expected (%d force nodes, %d carrier nodes, c1 %s, c2 %s, kb %s)", #servo, #carrier,
      tostring(cidC1), tostring(cidC2), tostring(cidKB)))
    servo = nil
    phase = "off"
    return false
  end
  -- yaw inertia about the centre of mass (all nodes; for the yaw hold) and of the structural nodes (for the angular rate)
  local comx, comy = 0, 0
  for _, a in ipairs(all) do
    local p = obj:getNodePosition(a.cid)
    a.x, a.y, a.z = p.x, p.y, p.z
    comx, comy = comx + p.x * a.m, comy + p.y * a.m
  end
  comx, comy = comx / math.max(totalMass, 1e-6), comy / math.max(totalMass, 1e-6)
  yawInertia = 0
  for _, a in ipairs(all) do yawInertia = yawInertia + a.m * ((a.x - comx) ^ 2 + (a.y - comy) ^ 2) end
  heavyInertia = 0
  local hmx, hmy = 0, 0
  for _, h in ipairs(heavy) do
    local p = obj:getNodePosition(h.cid)
    h.x, h.y = p.x, p.y
    hmx, hmy = hmx + p.x * h.m, hmy + p.y * h.m
  end
  hmx, hmy = hmx / math.max(heavyMass, 1e-6), hmy / math.max(heavyMass, 1e-6)
  for _, h in ipairs(heavy) do heavyInertia = heavyInertia + h.m * ((h.x - hmx) ^ 2 + (h.y - hmy) ^ 2) end
  -- the carrier's inertia about the steering axis (the line through the kingpin) and the steering servo's gains
  local pkb = obj:getNodePosition(cidKB)
  Jc = WHEEL_YAW_INERTIA
  for _, c in ipairs(carrier) do
    local p = obj:getNodePosition(c.cid)
    Jc = Jc + c.m * ((p.x - pkb.x) ^ 2 + (p.y - pkb.y) ^ 2)
  end
  local ws = 2 * math.pi * STEER_HZ
  steerKp, steerKd, steerKi = Jc * ws * ws, 2 * STEER_ZETA * Jc * ws, Jc * ws * ws * STEER_KI
  -- the level hold: virtual vertical springs on the four force nodes, stiff enough for TILT_HZ about the wheel's contact point
  local Jt, sr2 = 0, 0
  local cx, cy = 0, 0
  for i = 1, #servo do
    local p = obj:getNodePosition(servo[i])
    cx, cy = cx + p.x / #servo, cy + p.y / #servo
  end
  for i = 1, #servo do
    local p = obj:getNodePosition(servo[i])
    sr2 = sr2 + (p.x - cx) ^ 2 + (p.y - cy) ^ 2
  end
  sr2 = 0.5 * sr2                                  -- sum of rx^2 (= sum of ry^2 for a rectangle), the torque stiffness is K * sr2
  local Jx, Jy = 0, 0
  for _, a in ipairs(all) do
    Jx = Jx + a.m * ((a.y - pkb.y) ^ 2 + (a.z - pkb.z) ^ 2)      -- (positions are relative to the reference node: heights from the lower pivot,
    Jy = Jy + a.m * ((a.x - pkb.x) ^ 2 + (a.z - pkb.z) ^ 2)      -- which is at ground height)
  end
  Jt = math.max(Jx, Jy)
  local wt = 2 * math.pi * TILT_HZ
  tiltK = Jt * wt * wt / math.max(sr2, 1e-6)
  tiltC = 2 * TILT_ZETA * Jt * wt / math.max(sr2, 1e-6)
  log("I", "modelTire_wheelrig", string.format("%d nodes, mass %.1f kg, yaw inertia %.0f, carrier inertia %.1f kg m^2 (steering Kp %.0f, Kd %.0f N m/rad), level hold %.0f N/m, %.0f N s/m",
    #all, totalMass, yawInertia, Jc, steerKp, steerKd, tiltK, tiltC))
  return true
end

local function captureHeading()
  local d = obj:getDirectionVector()
  local l = math.sqrt(d.x * d.x + d.y * d.y)
  hx, hy = d.x / l, d.y / l
  psi0 = math.atan2(hy, hx)
  prevPsi, omega = nil, 0
end

local function resetState()
  iex, iey, ieS = 0, 0, 0
  slipNow, vcmd = 0, 0
  aSx, aSy, aSz, aN, aT, aTau = 0, 0, 0, 0, 0, 0
  repVx, repVy, repVz, repWc = nil, nil, nil, nil
  tPhase = 0
  loadNow = nil
end

local function stand()
  resetState()
  if servo then captureHeading() end
  if enablePhysicsStepHook then enablePhysicsStepHook() end     -- the game only calls onPhysicsStep after this has been asked for
  setPhase("stand")
end

local function stop()
  -- not "off": without the controller the rig falls over. Slow down, straighten the wheel and stand.
  if phase == "off" then return stand() end
  setPhase("stand")
  slipNow = 0
end

local function run(speedKmh, maxSlipDeg, rampSec, loadN)
  if not servo then collect() end
  if not servo then return end
  cmd.speed = (speedKmh or 55) / 3.6
  cmd.maxSlip = maxSlipDeg or 20
  cmd.ramp = math.max(rampSec or 20, 2)
  if loadN and loadN > 0 then loadSet = loadN end
  captureHeading()
  iex, iey, ieS = 0, 0, 0
  slipNow, vcmd = 0, 0
  aSx, aSy, aSz, aN, aT, aTau = 0, 0, 0, 0, 0, 0
  repVx, repVy, repVz, repWc = nil, nil, nil, nil
  tPhase = 0
  log("I", "modelTire_wheelrig", string.format("run: %.1f m/s, slip to %.0f deg over %.0f s, load %.0f N, heading (%.2f, %.2f)", cmd.speed, cmd.maxSlip, cmd.ramp, loadSet, hx, hy))
  if enablePhysicsStepHook then enablePhysicsStepHook() end
  setPhase("settle")
end

local function setLoad(n)
  if n and n > 0 then loadSet = n end
end

local function onPhysicsStep(dt)
  if phase == "off" or dt <= 0 then return end
  if not servo then
    if not collect() then return end
    captureHeading()
  end
  tPhase = tPhase + dt

  -- state of the structural nodes: centre of mass position and velocity
  local vx, vy, vz, px, py, pz = 0, 0, 0, 0, 0, 0
  for i = 1, #heavy do
    local h = heavy[i]
    local nv = obj:getNodeVelocityVector(h.cid)
    local p = obj:getNodePosition(h.cid)
    h.vx, h.vy, h.px, h.py = nv.x, nv.y, p.x, p.y
    vx, vy, vz = vx + nv.x * h.m, vy + nv.y * h.m, vz + nv.z * h.m
    px, py = px + p.x * h.m, py + p.y * h.m
  end
  vx, vy, vz = vx / heavyMass, vy / heavyMass, vz / heavyMass
  local comx, comy = px / heavyMass, py / heavyMass
  local speedNow = math.sqrt(vx * vx + vy * vy)
  if not loadNow then loadNow = totalMass * g end

  -- phases
  local lx, ly = -hy, hx
  local alphaDot = 0
  if phase == "stand" then
    vcmd = math.max(0, vcmd - DECEL * dt)
  elseif phase == "settle" then
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
  elseif phase == "done" then
    vcmd = math.max(0, vcmd - DECEL * dt)
  end

  -- horizontal: the rig goes straight at vcmd along its heading (the slip angle is made by turning the wheel, not the rig)
  local tvx, tvy = vcmd * hx, vcmd * hy
  local tax, tay = 0, 0
  if phase == "settle" then tax, tay = hx * ACCEL, hy * ACCEL end
  local ex, ey = tvx - vx, tvy - vy
  iex = math.max(-A_MAX / KI, math.min(A_MAX / KI, iex + ex * dt))
  iey = math.max(-A_MAX / KI, math.min(A_MAX / KI, iey + ey * dt))
  local ax = math.max(-A_MAX, math.min(A_MAX, KP * ex + KI * iex + tax))
  local ay = math.max(-A_MAX, math.min(A_MAX, KP * ey + KI * iey + tay))

  -- frame yaw hold
  local psi = headingNow()
  if prevPsi then omega = 0.9 * omega + 0.1 * (wrap(psi - prevPsi) / dt) end
  prevPsi = psi
  local yawTorque = -yawInertia * (KPY * wrap(psi - psi0) + KDY * omega)

  -- the carrier: its heading from the front and rear nodes, its angular rate about the kingpin from the nodes' momentum
  local pc1, pc2 = obj:getNodePosition(cidC1), obj:getNodePosition(cidC2)
  local psiC = math.atan2(pc1.y - pc2.y, pc1.x - pc2.x)
  local pkb, vkb = obj:getNodePosition(cidKB), obj:getNodeVelocityVector(cidKB)
  local Lc, I2 = 0, 0
  for i = 1, #carrier do
    local c = carrier[i]
    local p, nv = obj:getNodePosition(c.cid), obj:getNodeVelocityVector(c.cid)
    local rx, ry = p.x - pkb.x, p.y - pkb.y
    Lc = Lc + c.m * (rx * (nv.y - vkb.y) - ry * (nv.x - vkb.x))
    I2 = I2 + c.m * (rx * rx + ry * ry)
    c.px, c.py = p.x, p.y
  end
  local wC = Lc / math.max(I2, 1e-6)
  -- steering PID: the wheel points slipNow to the right of the direction of travel
  local target = psi0 - slipNow
  local eS = wrap(psiC - target)
  ieS = math.max(-TAU_MAX / steerKi, math.min(TAU_MAX / steerKi, ieS + eS * dt))
  local tau = -(steerKp * eS + steerKd * (wC + alphaDot) + steerKi * ieS)
  tau = math.max(-TAU_MAX, math.min(TAU_MAX, tau))
  -- the torque on the carrier's four outer nodes (a couple: they are placed symmetrically about the kingpin)...
  local r2c = 0
  for i = 1, #carrier do
    local c = carrier[i]
    if c.m > 0 and not (c.px == nil) then
      c.rx, c.ry = c.px - pkb.x, c.py - pkb.y
      r2c = r2c + c.rx * c.rx + c.ry * c.ry
    end
  end
  for i = 1, #carrier do
    local c = carrier[i]
    obj:applyForceVector(c.cid, vec3(-tau * c.ry / r2c, tau * c.rx / r2c, 0))
  end

  -- the force nodes: speed servo, yaw hold (with the steering torque's reaction), load and level
  local n = #servo
  local pos, vel = {}, {}
  local cx, cy, zbar, vbar = 0, 0, 0, 0
  for i = 1, n do
    pos[i], vel[i] = obj:getNodePosition(servo[i]), obj:getNodeVelocityVector(servo[i])
    cx, cy = cx + pos[i].x, cy + pos[i].y
    zbar, vbar = zbar + pos[i].z, vbar + vel[i].z
  end
  cx, cy, zbar, vbar = cx / n, cy / n, zbar / n, vbar / n
  local r2 = 0
  for i = 1, n do r2 = r2 + (pos[i].x - cx) ^ 2 + (pos[i].y - cy) ^ 2 end
  local kT = (r2 > 1e-6) and (yawTorque - tau) / r2 or 0
  -- vertical: the rig's weight minus the wheel load, damped
  loadNow = loadNow + math.max(-LOAD_RATE * dt, math.min(LOAD_RATE * dt, loadSet - loadNow))
  local Cz = 2 * VZ_ZETA * math.sqrt(TYRE_K * totalMass)
  local Sz = totalMass * g - loadNow - Cz * vz
  local fx, fy, fz = totalMass * ax / n, totalMass * ay / n, Sz / n
  for i = 1, n do
    local rx, ry = pos[i].x - cx, pos[i].y - cy
    local level = -tiltK * (pos[i].z - zbar) - tiltC * (vel[i].z - vbar)
    obj:applyForceVector(servo[i], vec3(fx - kT * ry, fy + kT * rx, fz + level))
  end

  -- window sums for the report
  aSx, aSy, aSz, aN, aT, aTau = aSx + totalMass * ax, aSy + totalMass * ay, aSz + Sz, aN + 1, aT + dt, aTau + tau
  curVx, curVy, curVz, curPsiC, curWc = vx, vy, vz, psiC, wC
  if not repVx then repVx, repVy, repVz, repWc = vx, vy, vz, wC end
  lastSpeed, lastTau, lastSteer = speedNow, tau, math.deg(wrap(psi0 - psiC))
  logT = logT + dt
  if logT >= 1 then
    logT = 0
    log("I", "modelTire_wheelrig", string.format("%s: speed %.2f m/s (target %.2f), slip %.1f deg (wheel %.1f), steer torque %.1f N m, load target %.0f N",
      phase, lastSpeed, vcmd, math.deg(slipNow), lastSteer, lastTau, loadNow))
  end
end

-- the mean forces since the last report
local function report()
  if aN > 0 and aT > 0 and repVx then
    local Fwx = totalMass * (curVx - repVx) / aT - aSx / aN
    local Fwy = totalMass * (curVy - repVy) / aT - aSy / aN
    local hdx, hdy = math.cos(curPsiC), math.sin(curPsiC)
    lastFx, lastFy = Fwx * hdx + Fwy * hdy, -Fwx * hdy + Fwy * hdx
    lastFz = totalMass * (curVz - repVz) / aT + totalMass * g - aSz / aN
    lastMz = Jc * (curWc - repWc) / aT - aTau / aN
    repVx, repVy, repVz, repWc = curVx, curVy, curVz, curWc
  end
  aSx, aSy, aSz, aN, aT, aTau = 0, 0, 0, 0, 0, 0
  return {phase = phase, Mz = lastMz, slip = math.deg(slipNow), speed = lastSpeed, Fx = lastFx, Fy = lastFy, Fz = lastFz,
    load = (lastFz > 100) and lastFz or loadSet, maxSlip = cmd.maxSlip, ramp = cmd.ramp, steer = lastSteer, rigKind = "wheel"}
end

local function onReset()
  servo = nil
  loadNow = nil
  resetState()
  phase = "stand"
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
