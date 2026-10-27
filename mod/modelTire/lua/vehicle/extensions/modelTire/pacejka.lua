-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- modelTire_pacejka: Pacejka Magic Formula tyre (MF 5.2 / 6.1 / 6.2 style steady-state model with combined
-- slip, aligning moment, overturning moment and rolling resistance), plus first-order force build-up from the
-- relaxation lengths and a linear vertical spring. Fed by SVJ tyre sets (svj.lua). Licence: GPL-3.0-or-later.
--
-- Equations follow the Magic Formula 6.1 set (Besselink, Schmeitz, Pacejka: "An improved Magic Formula/Swift
-- tyre model...", 2010) with the coefficient names of the .tir/SVJ files. Scaling factors (lambda) are 1 except the
-- friction scale from the surface. Missing coefficients are 0, as the SVJ spec asks.
--
-- Axes (ISO): x forward, y left, z up. Slip angle alpha = atan(Vy / |Vx|) (positive when moving left),
-- slip ratio kappa = (Re * Omega - Vx) / |Vx| (positive when driving), camber gamma positive when the top of the
-- wheel leans right. Forces are those the ground puts on the tyre: Fy < 0 for alpha > 0 (pKy1 < 0).
--
-- Interface (same as rbr102.lua so the wrapper in tyre.lua can run either):
--   newState(T, radius), step(s, T, mat, i). Results in s: inContact, L (RBR lateral = right), Rl, N,
--   Flat (along L), Flong, Mz (about N), Mfwd (about forward axis), rollingTorque (about A, the RBR axle),
--   normalForce, vLat, longSlipVel, staticF, slidingW.

local M = {}

local abs, sqrt, exp, min, max, atan, sin, cos, tan, pi = math.abs, math.sqrt, math.exp, math.min, math.max,
  math.atan, math.sin, math.cos, math.tan, math.pi
local EPS = 1e-6
local G = 9.81

local function sign(x) if x > 0 then return 1 elseif x < 0 then return -1 end return 0 end
local function clamp(x, a, b) if x < a then return a elseif x > b then return b end return x end
local function sgnE(x) return (x >= 0) and 1 or -1 end -- sign with sgn(0) = 1, as in the MF equations

-- Magic Formula core: y = D sin(C atan(B x - E (B x - atan(B x))))
local function mf(B, C, D, E, x)
  local bx = B * x
  return D * sin(C * atan(bx - E * (bx - atan(bx))))
end

-- cos(C atan(B x - E (B x - atan(B x)))) used by the combined-slip weights
local function gcos(B, C, E, x)
  local bx = B * x
  return cos(C * atan(bx - E * (bx - atan(bx))))
end

-- The pure-slip and combined forces and moments. c: coefficient groups (numbers; missing keys read as 0).
-- inputs: kappa, alpha (rad), gamma (rad), Fz, dpi, Vx (signed), lam (friction scale), lamK (stiffness scale)
local function evaluate(T, Fz0, kappa, alpha, gamma, Fz, dpi, Vx, lam, lamK, out)
  local lat, lon, ali, ovt, rol, com = T.lateral, T.longitudinal, T.aligning, T.overturning, T.rolling, T.combined
  local R0 = T.R0
  local dfz = (Fz - Fz0) / Fz0
  local gs = sin(gamma)           -- gamma*
  local as = tan(alpha)           -- alpha*

  -- pure longitudinal
  local Cx = lon.pCx1
  local mux = max((lon.pDx1 + lon.pDx2 * dfz), 0.25 * abs(lon.pDx1)) * (1 + lon.pPx3 * dpi + lon.pPx4 * dpi * dpi) * (1 - lon.pDx3 * gamma * gamma) * lam
  local Dx = mux * Fz
  local Kxk = Fz * (lon.pKx1 + lon.pKx2 * dfz) * exp(lon.pKx3 * dfz) * (1 + lon.pPx1 * dpi + lon.pPx2 * dpi * dpi) * lamK
  local Bx = Kxk / (Cx * Dx + EPS)
  local SHx = lon.pHx1 + lon.pHx2 * dfz
  local SVx = Fz * (lon.pVx1 + lon.pVx2 * dfz) * lam
  local kx = kappa + SHx
  local Ex = min((lon.pEx1 + lon.pEx2 * dfz + lon.pEx3 * dfz * dfz) * (1 - lon.pEx4 * sgnE(kx)), 1)
  local Fx0 = mf(Bx, Cx, Dx, Ex, kx) + SVx

  -- pure lateral
  local Cy = lat.pCy1
  local muy = max((lat.pDy1 + lat.pDy2 * dfz), 0.25 * abs(lat.pDy1)) * (1 + lat.pPy3 * dpi + lat.pPy4 * dpi * dpi) * (1 - lat.pDy3 * gs * gs) * lam
  local Dy = muy * Fz
  local Kya = lat.pKy1 * Fz0 * (1 + lat.pPy1 * dpi) * (1 - lat.pKy3 * abs(gs)) *
    sin(lat.pKy4 * atan((Fz / Fz0) / ((lat.pKy2 + lat.pKy5 * gs * gs) * (1 + lat.pPy2 * dpi) + EPS))) * lamK
  local Kyap = Kya + ((Kya >= 0) and EPS or -EPS)
  local SVyg = Fz * (lat.pVy3 + lat.pVy4 * dfz) * gs * lam
  local SVy = Fz * (lat.pVy1 + lat.pVy2 * dfz) * lam + SVyg
  local Kyg0 = Fz * (lat.pKy6 + lat.pKy7 * dfz) * (1 + lat.pPy5 * dpi)
  local SHy = (lat.pHy1 + lat.pHy2 * dfz) + (Kyg0 * gs - SVyg) / Kyap
  local By = Kya / (Cy * Dy + EPS)
  local ay = as + SHy
  local Ey = min((lat.pEy1 + lat.pEy2 * dfz) * (1 + lat.pEy5 * gs * gs - (lat.pEy3 + lat.pEy4 * gs) * sgnE(ay)), 1)
  local Fy0 = mf(By, Cy, Dy, Ey, ay) + SVy

  -- combined slip
  local Cxa = com.rCx1
  local Exa = min(com.rEx1 + com.rEx2 * dfz, 1)
  local SHxa = com.rHx1
  local Bxa = (com.rBx1 + com.rBx3 * gs * gs) * cos(atan(com.rBx2 * kappa))
  local Gxa = 1
  if Cxa ~= 0 and Bxa ~= 0 then
    Gxa = gcos(Bxa, Cxa, Exa, as + SHxa) / gcos(Bxa, Cxa, Exa, SHxa)
  end
  local Fx = Gxa * Fx0

  local Cyk = com.rCy1
  local Eyk = min(com.rEy1 + com.rEy2 * dfz, 1)
  local SHyk = com.rHy1 + com.rHy2 * dfz
  local Byk = (com.rBy1 + com.rBy4 * gs * gs) * cos(atan(com.rBy2 * (as - com.rBy3)))
  local Gyk = 1
  if Cyk ~= 0 and Byk ~= 0 then
    Gyk = gcos(Byk, Cyk, Eyk, kappa + SHyk) / gcos(Byk, Cyk, Eyk, SHyk)
  end
  local DVyk = muy * Fz * (com.rVy1 + com.rVy2 * dfz + com.rVy3 * gs) * cos(atan(com.rVy4 * as))
  local SVyk = DVyk * sin(com.rVy5 * atan(com.rVy6 * kappa))
  local Fy = Gyk * Fy0 + SVyk

  -- aligning moment
  local Mz = 0
  if ali and T.hasAligning then
    local SHt = ali.qHz1 + ali.qHz2 * dfz + (ali.qHz3 + ali.qHz4 * dfz) * gs
    local SHf = SHy + SVy / Kyap
    local at, ar = as + SHt, as + SHf
    local ratio = Kxk / Kyap
    local ate = sqrt(at * at + ratio * ratio * kappa * kappa) * sgnE(at)
    local are = sqrt(ar * ar + ratio * ratio * kappa * kappa) * sgnE(ar)
    local Bt = (ali.qBz1 + ali.qBz2 * dfz + ali.qBz3 * dfz * dfz) * (1 + ali.qBz4 * gs + ali.qBz5 * abs(gs)) * lamK / max(lam, 0.05)
    local Ct = ali.qCz1
    local Dt = Fz * (R0 / Fz0) * (ali.qDz1 + ali.qDz2 * dfz) * (1 + ali.qPz1 * dpi) * (1 + ali.qDz3 * gs + ali.qDz4 * gs * gs)
    local Et = min((ali.qEz1 + ali.qEz2 * dfz + ali.qEz3 * dfz * dfz) * (1 + (ali.qEz4 + ali.qEz5 * gs) * (2 / pi) * atan(Bt * Ct * ate)), 1)
    local bt = Bt * ate
    local t = Dt * cos(Ct * atan(bt - Et * (bt - atan(bt)))) * cos(alpha)
    local Fyp = Fy - SVyk
    local Br = ali.qBz9 * lamK / max(lam, 0.05) + ali.qBz10 * By * Cy
    local Dr = Fz * R0 * ((ali.qDz6 + ali.qDz7 * dfz) + ((ali.qDz8 + ali.qDz9 * dfz) * (1 + ali.qPz2 * dpi) +
      (ali.qDz10 + ali.qDz11 * dfz) * abs(gs)) * gs) * lam
    local Mzr = Dr * cos(atan(Br * are)) * cos(alpha)
    local s = R0 * (ali.ssz1 + ali.ssz2 * (Fy / Fz0) + (ali.ssz3 + ali.ssz4 * dfz) * gs)
    Mz = -t * Fyp + Mzr + s * Fx
    out.t = t
  end

  -- overturning moment
  local Mx = 0
  if ovt and T.hasOverturning then
    local fyn = Fy / Fz0
    local fzn = Fz / Fz0
    local a6 = atan(ovt.qsx6 * fzn)
    Mx = R0 * Fz * (ovt.qsx1 - ovt.qsx2 * gamma * (1 + ovt.qPx1 * dpi) + ovt.qsx3 * fyn +
      ovt.qsx4 * cos(ovt.qsx5 * a6 * a6) * sin(ovt.qsx7 * gamma + ovt.qsx8 * atan(ovt.qsx9 * fyn)) +
      ovt.qsx10 * atan(ovt.qsx11 * fzn) * gamma)
  end

  -- rolling resistance moment (negative about +y for forward rolling)
  local My = 0
  if rol and T.hasRolling then
    local v = Vx / T.V0
    My = -Fz * R0 * (rol.qsy1 + rol.qsy2 * Fx / Fz0 + rol.qsy3 * abs(v) + rol.qsy4 * v ^ 4 +
      (rol.qsy5 + rol.qsy6 * Fz / Fz0) * gamma * gamma) * (Fz / Fz0) ^ rol.qsy7 * ((dpi + 1) ^ rol.qsy8)
  end

  out.Fx, out.Fy, out.Mz, out.Mx, out.My = Fx, Fy, Mz, Mx, My
  out.Dx, out.Dy, out.Kxk, out.Kya, out.Fy0, out.Fx0 = Dx, Dy, Kxk, Kya, Fy0, Fx0
end

-- Fills missing coefficient groups with zeros (an index metatable) so the equations can read any name.
local zeroMeta = {__index = function() return 0 end}
M._evaluate = evaluate
function M.prepare(T)
  if T.prepared then return T end
  for _, g in ipairs({"lateral", "longitudinal", "aligning", "overturning", "rolling", "combined"}) do
    T[g] = setmetatable(T[g] or {}, zeroMeta)
  end
  T.hasAligning = next(T.aligning) ~= nil
  T.hasOverturning = next(T.overturning) ~= nil
  if next(T.rolling) == nil then
    -- no rolling-resistance coefficients in the file: a typical car tyre
    T.rolling.qsy1, T.rolling.qsy3 = 0.010, 0.0004
  end
  T.hasRolling = true
  -- sign convention of the file (spec 11.13: the signs follow the coefficients). A positive pKy1 means a positive Fy
  -- for a positive slip angle, i.e. the mirror image of the ISO sign: the lateral force, aligning and overturning
  -- moments are flipped, and so is the camber.
  T.flipLat = rawget(T.lateral, "pKy1") ~= nil and T.lateral.pKy1 > 0
  T.Fz0 = (T.Fz0 and T.Fz0 > 0) and T.Fz0 or 4000
  T.p0 = (T.p0 and T.p0 > 0) and T.p0 or 220000
  T.V0 = (T.V0 and T.V0 > 0) and T.V0 or 16.67
  T.R0 = (T.R0 and T.R0 > 0) and T.R0 or 0.3
  T.Re = (T.Re and T.Re > 0) and T.Re or T.R0 * 0.97
  T.kz = (T.kz and T.kz > 0) and T.kz or 200000
  T.cz = T.cz or 0
  T.sigmaX = (T.sigmaX and T.sigmaX > 0.01) and T.sigmaX or 0.1
  T.sigmaY = (T.sigmaY and T.sigmaY > 0.01) and T.sigmaY or 0.3
  T.prepared = true
  return T
end

function M.newState(T, radius)
  M.prepare(T)
  return {
    u = 0, v = 0, inContact = false,
    Flat = 0, Flong = 0, Mz = 0, Mfwd = 0, rollingTorque = 0, normalForce = 0, load = 0,
    vLat = 0, longSlipVel = 0, staticF = 0, slidingW = 0, sinkDepth = 0,
    kappa = 0, alpha = 0, gamma = 0, Fx = 0, Fy = 0,
    L = {0, 0, 0}, Rl = {1, 0, 0}, N = {0, 0, 1},
    out = {},
  }
end

local function noContact(s)
  s.inContact = false
  s.Flat, s.Flong, s.Mz, s.Mfwd, s.rollingTorque, s.normalForce, s.load = 0, 0, 0, 0, 0, 0, 0
  s.slidingW = 0
  -- the force build-up relaxes while airborne
  s.u, s.v = s.u * 0.9, s.v * 0.9
end

-- One physics step.
-- i: N {x,y,z} ground normal, A {x,y,z} axle (N x A must be the forward direction), V {x,y,z} hub velocity (world),
--    spin (RBR sign: negative when rolling forward), hubHeight (m, perpendicular to the ground), dt,
--    freeRadius (m, tyre radius to measure the deflection from; default T.R0),
--    grip (surface friction scale, default 1), pressure (Pa gauge, default the tyre's own),
--    zeta (low-speed damping ratio, default 0)
function M.step(s, T, mat, i)
  local dt = i.dt
  local N, A, V = i.N, i.A, i.V
  local R0free = i.freeRadius or T.R0

  -- contact frame (as in rbr102.lua): Rl = N x A, left = N x Rl
  local rx, ry, rz = N[2] * A[3] - N[3] * A[2], N[3] * A[1] - N[1] * A[3], N[1] * A[2] - N[2] * A[1]
  local rlen = sqrt(rx * rx + ry * ry + rz * rz)
  local def = R0free - i.hubHeight
  if i.Fz then def = max(def, 0.01) end          -- the load is given (BeamNG's own tyre carries it): no deflection needed
  if not (rlen > 0.1) or def < 1e-4 then noContact(s); return end
  rx, ry, rz = rx / rlen, ry / rlen, rz / rlen
  local lx, ly, lz = N[2] * rz - N[3] * ry, N[3] * rx - N[1] * rz, N[1] * ry - N[2] * rx -- left = N x Rl
  local Rl, L, Nn = s.Rl, s.L, s.N
  Rl[1], Rl[2], Rl[3] = rx, ry, rz
  L[1], L[2], L[3] = -lx, -ly, -lz              -- RBR lateral = right
  Nn[1], Nn[2], Nn[3] = N[1], N[2], N[3]

  -- vertical load
  local Fz = i.Fz or (T.kz * def)
  if i.progressive and not i.Fz then
    -- a real tyre stiffens strongly towards the rim; a linear spring would let the rim hit the ground on big landings
    local r = def / (i.bumpDef or 0.04)
    Fz = Fz * (1 + r * r)
  end
  if not (Fz > 0) then noContact(s); return end
  -- the load is capped (i.maxLoad): a freshly loaded car sits with its rim on the ground, far below the tyre's own
  -- height, and an uncapped spring would throw it away with hundreds of kN
  if i.maxLoad and Fz > i.maxLoad then Fz = i.maxLoad end
  s.normalForce = Fz
  s.load = Fz

  -- kinematics
  local Vx = V[1] * rx + V[2] * ry + V[3] * rz
  local Vy = V[1] * lx + V[2] * ly + V[3] * lz  -- left
  local Omega = -i.spin                         -- forward rolling positive
  -- rolling radius: the car's wheel radius when given, so BeamNG's own wheel-speed sensors agree with the tyre
  local Re = i.Re or T.Re
  local sx = Re * Omega - Vx                    -- longitudinal slip speed (driving positive)
  local incSin = -(N[1] * A[1] + N[2] * A[2] + N[3] * A[3])
  local gamma = math.asin(clamp(incSin, -1, 1)) -- positive when the top leans right
  s.vLat, s.longSlipVel = -Vy, sx
  local Vxa = abs(Vx)
  local speed = sqrt(Vx * Vx + Vy * Vy)

  -- force build-up: contact point deflections relax with the distance rolled
  local rs = i.relaxScale or 1
  local sigX, sigY = max(T.sigmaX * rs, 0.01), max(T.sigmaY * rs, 0.01)
  local ax_, ay_ = Vxa / sigX, Vxa / sigY
  if ax_ * dt > 1e-6 then
    local e = exp(-ax_ * dt)
    s.u = s.u * e + sx * (1 - e) / ax_
  else
    s.u = s.u + sx * dt
  end
  if ay_ * dt > 1e-6 then
    local e = exp(-ay_ * dt)
    s.v = s.v * e + Vy * (1 - e) / ay_
  else
    s.v = s.v + Vy * dt
  end
  local kappa = clamp(s.u / sigX, -3, 3)
  local alpha = atan(s.v / sigY)

  -- pressure and surface
  local p = i.pressure or T.pressure or T.p0
  local dpi = (p - T.p0) / T.p0
  local lam = i.grip or 1
  local lamK = i.gripK or sqrt(lam)
  local o = s.out
  local fz0 = (i.Fz0 and i.Fz0 > 100) and i.Fz0 or T.Fz0
  evaluate(T, fz0, kappa, alpha, T.flipLat and -gamma or gamma, Fz, dpi, Vx, lam, lamK, o)
  if T.flipLat then o.Fy, o.Mz, o.Mx, o.Fy0 = -o.Fy, -o.Mz, -o.Mx, -o.Fy0 end

  local Fx, Fy = o.Fx, o.Fy
  if not T.hasAligning then
    -- no aligning-moment coefficients (e.g. the fitted library): a pneumatic trail that starts at about a fifth of the
    -- contact length and fades to zero past the peak slip angle, as a real tyre's does; Mz = -t * Fy (ISO signs)
    local contactLen = 2 * sqrt(max(2 * R0free * def, 0))
    local t0 = (T.trail0 or 0.2) * contactLen
    local az = 1.4 * (T.slipPeak or 0.14)
    o.Mz = -t0 * max(0, 1 - abs(alpha) / az) * Fy
  end
  -- guard: a non-finite or absurd result (bad coefficients) must never reach the physics
  local lim = 4 * Fz
  if not (Fx == Fx and Fy == Fy and o.Mz == o.Mz and o.Mx == o.Mx and o.My == o.My) or abs(Fx) > 1e7 or abs(Fy) > 1e7 then
    s.faulted = true
    Fx, Fy, o.Mz, o.Mx, o.My = 0, 0, 0, 0, 0
  else
    Fx, Fy = clamp(Fx, -lim, lim), clamp(Fy, -lim, lim)
    o.Mz = clamp(o.Mz, -0.5 * Fz * T.R0, 0.5 * Fz * T.R0)
  end

  -- low-speed damping (not part of the Magic Formula): a fraction of the critical damping of the contact spring
  local zeta = i.zeta or 0
  if zeta > 0 then
    local w = max(0, 1 - speed / 3)
    if w > 0 then
      -- mass: the car's share on this wheel (not the capped load), stiffness capped: fitted libraries have very stiff,
      -- short-relaxation tyres, and an unbounded k would turn speed noise into hundreds of kN
      local m = i.dampMass or (Fz / G)
      local kyy = min(abs(o.Kya) / sigY, 1.5e6)
      local kxx = min(abs(o.Kxk) / sigX, 1.5e6)
      Fy = Fy - w * 2 * zeta * sqrt(kyy * m) * Vy
      Fx = Fx + w * 2 * zeta * sqrt(kxx * m) * sx
    end
  end
  -- whatever the damping and the build-up did, a tyre cannot push harder than its friction allows
  local fxMax = max(1.2 * abs(o.Dx or 0), 0.3 * Fz)
  local fyMax = max(1.2 * abs(o.Dy or 0), 0.3 * Fz)
  Fx, Fy = clamp(Fx, -fxMax, fxMax), clamp(Fy, -fyMax, fyMax)

  s.kappa, s.alpha, s.gamma, s.Fx, s.Fy = kappa, alpha, gamma, Fx, Fy
  s.dpi, s.lam, s.lamK, s.Vx, s.fz0 = dpi, lam, lamK, Vx, fz0
  s.Flong = Fx
  s.Flat = -Fy                                   -- along the RBR lateral axis (right)
  s.Mz = o.Mz
  s.Mfwd = o.Mx
  -- rolling resistance: the moment about +y (left) opposes forward rolling; about the RBR axle (right) it is -My.
  -- Faded to zero with the wheel speed so a parked wheel does not creep.
  s.rollingTorque = -o.My * math.tanh(Omega * Re / 0.5)
  s.staticF = (o.Dy or 0) / max(Fz, 1)
  local sat = (abs(Fx) / max(abs(o.Dx), 1) > 0.97) or (abs(o.Fy0) / max(abs(o.Dy), 1) > 0.97)
  s.slidingW = sat and 1 or 0
  s.inContact = true
end

return M
