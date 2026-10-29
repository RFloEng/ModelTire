-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- modelTire_brushfull: the Advanced brush as a FULL tyre. BeamNG's tyre ring does not touch the ground; the wheel height over the
-- ground gives the load (a vertical spring), and this brush gives the forces: the same brush as the Advanced brush mode (stiffness per
-- N falls with the load, pressure, temperature and wear, friction ellipse, Stribeck friction) with the interface of pacejka.lua, so
-- tyre.lua's FULL path (envelope contact, force on the hub, wheel-spin torque on the rim) runs it like the RBR and SVJ tyres.
--   newState(), step(s, P, mat, i): P is tyre.lua's parameter table. Results in s: inContact, L (lateral, RBR sign = right), Rl,
--   N, Flat (along L), Flong, Mz (about N), Mfwd, rollingTorque, normalForce, vLat, longSlipVel, staticF, slidingW, advT, advWear.
-- Licence: GPL-3.0-or-later.

local M = {}

local abs, sqrt, exp, min, max, sin, atan, tanh = math.abs, math.sqrt, math.exp, math.min, math.max, math.sin, math.atan, math.tanh
local G = 9.81

function M.newState()
  return {
    dx = 0, dy = 0, inContact = false,
    Flat = 0, Flong = 0, Mz = 0, Mfwd = 0, rollingTorque = 0, normalForce = 0, load = 0,
    vLat = 0, longSlipVel = 0, staticF = 0, slidingW = 0,
    L = {0, 0, 0}, Rl = {1, 0, 0}, N = {0, 0, 1},
    advT = nil, advWear = 0, advP = 0,
  }
end

local function noContact(s)
  s.inContact = false
  s.Flat, s.Flong, s.Mz, s.Mfwd, s.rollingTorque, s.normalForce, s.load = 0, 0, 0, 0, 0, 0, 0
  s.slidingW = 0
  s.dx, s.dy = s.dx * 0.9, s.dy * 0.9          -- the carcass relaxes while airborne
end

-- the same shapes as tyre.lua's Advanced brush
local function stiffLoad(P, Fz)
  local c = P.advLoadShape
  return sin(2 * atan(Fz / (c * P.fzNominal))) / sin(2 * atan(1 / c)) * P.fzNominal / max(Fz, 1)
end

local function tempFactor(P, T)
  local z = (T - P.advTopt) / P.advTwidth
  return P.advTfloor + (1 - P.advTfloor) * exp(-0.5 * z * z)
end

-- i: N {x,y,z} ground normal, A {x,y,z} axle (N x A is the forward direction), V {x,y,z} hub velocity (world), spin (RBR sign:
--    negative when rolling forward), hubHeight (m, perpendicular to the ground), dt, freeRadius (m), Re (rolling radius, m),
--    kz (N/m, vertical stiffness), progressive/bumpDef (stiffening towards the rim), maxLoad (N), airPressure (Pa gauge), grip (surface scale)
function M.step(s, P, mat, i)
  local dt = i.dt
  local N, A, V = i.N, i.A, i.V

  -- contact frame (as pacejka.lua): Rl = N x A forward, left = N x Rl
  local rx, ry, rz = N[2] * A[3] - N[3] * A[2], N[3] * A[1] - N[1] * A[3], N[1] * A[2] - N[2] * A[1]
  local rlen = sqrt(rx * rx + ry * ry + rz * rz)
  local def = i.freeRadius - i.hubHeight
  if not (rlen > 0.1) or def < 1e-4 then noContact(s); return end
  rx, ry, rz = rx / rlen, ry / rlen, rz / rlen
  local lx, ly, lz = N[2] * rz - N[3] * ry, N[3] * rx - N[1] * rz, N[1] * ry - N[2] * rx
  local Rl, L, Nn = s.Rl, s.L, s.N
  Rl[1], Rl[2], Rl[3] = rx, ry, rz
  L[1], L[2], L[3] = -lx, -ly, -lz              -- RBR lateral = right
  Nn[1], Nn[2], Nn[3] = N[1], N[2], N[3]

  -- vertical load: a spring on the deflection, stiffer towards the rim, capped
  local Fz = (i.kz or 2e5) * def
  if i.progressive then
    local r = def / (i.bumpDef or 0.04)
    Fz = Fz * (1 + r * r)
  end
  if not (Fz > 0) then noContact(s); return end
  if i.maxLoad and Fz > i.maxLoad then Fz = i.maxLoad end
  s.normalForce, s.load = Fz, Fz

  -- slip velocities
  local vx = V[1] * rx + V[2] * ry + V[3] * rz
  local vy = V[1] * lx + V[2] * ly + V[3] * lz  -- left
  local Re = i.Re or i.freeRadius
  local wheelSpeed = -i.spin * Re
  local sx, sy = vx - wheelSpeed, vy
  s.vLat, s.longSlipVel = -vy, wheelSpeed - vx

  -- stiffness at this load, pressure and temperature, wear
  local pr = max(0.3, min(2.5, (i.airPressure or 2e5) / (P.advPressureNom * 1000)))
  local kl = stiffLoad(P, Fz)
  local kp = pr ^ P.advPressureK
  local Cy, Cx = P.cornerCoef * Fz * kl * kp, P.longCoef * Fz * kl * kp
  local T = s.advT or P.advTstart
  local advMu = tempFactor(P, T) * (1 - P.advWearLoss * min(1, s.advWear or 0)) * max(0.6, 1 - P.advPressureMu * (pr - 1) ^ 2)
  local ky, kx = Cy / P.relaxLenY, Cx / P.relaxLenX
  local mq = Fz / G
  local cyD = 2 * P.dampRatio * sqrt(ky * mq)
  local cxD = 2 * P.dampRatio * sqrt(kx * mq)

  -- deflection update, exact exponential relaxation
  local Vr = max(abs(vx), abs(wheelSpeed))
  local ay_, ax_ = Vr / P.relaxLenY, Vr / P.relaxLenX
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
  local Fx, Fy = -(kx * dx), -(ky * dy)

  -- friction limit: Stribeck, load, surface, temperature, wear, pressure
  local slipSpeed = sqrt(sx * sx + sy * sy)
  local q = slipSpeed / P.stribeckVel
  local mu = P.muSlide + (P.mu - P.muSlide) * exp(-q * q)
  mu = mu * max(0.4, 1 - P.loadSens * (Fz / P.fzNominal - 1)) * (i.grip or 1) * advMu
  local FmaxY = mu * Fz
  local FmaxX = mu * P.muLongScale * Fz
  -- parabolic pressure distribution (Pacejka ch. 3): the force rolls over smoothly as the rear of the patch slides; friction ellipse
  local ux, uy = Fx / (3 * FmaxX), Fy / (3 * FmaxY)
  local ur = sqrt(ux * ux + uy * uy)
  local sliding = ur >= 1
  if ur > 1e-6 then
    local gg = (ur < 1) and (3 * ur - 3 * ur * ur + ur * ur * ur) or 1
    local f = gg / ur
    Fx, Fy = FmaxX * ux * f, FmaxY * uy * f
  end
  if ur > 1.15 then
    local k = 1.15 / ur
    dx, dy = dx * k, dy * k
  end
  s.dx, s.dy = dx, dy
  Fx, Fy = Fx - cxD * ddx, Fy - cyD * ddy
  local rt = sqrt((Fx / FmaxX) ^ 2 + (Fy / FmaxY) ^ 2)
  if rt > 1 then Fx, Fy = Fx / rt, Fy / rt end
  if not (Fx == Fx and Fy == Fy) then
    s.dx, s.dy = 0, 0
    Fx, Fy = 0, 0
  end

  -- thermal model and wear (as the Advanced brush): sliding power and rolling heat in, convection out
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

  -- results. Fy is along the left axis, Flat along the RBR lateral (right). The aligning moment about N (ISO): the lateral force
  -- acts a pneumatic trail behind the centre, which fades as the force saturates. Rolling resistance as a torque, as pacejka.lua.
  local trail = P.trail * max(0, 1 - abs(Fy) / FmaxY)
  s.Flong = Fx
  s.Flat = -Fy
  s.Mz = -trail * Fy
  s.Mfwd = 0
  s.rollingTorque = P.crr * Fz * Re * tanh(wheelSpeed / 0.5)
  s.staticF = mu
  s.slidingW = sliding and 1 or 0
  s.inContact = true
end

return M
