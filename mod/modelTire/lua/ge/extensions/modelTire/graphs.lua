-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- modelTire: floating force-versus-slip graphs for the Pacejka (SVJ) tyre, one window per wheel.
-- Extension "modelTire_graphs", drawn from the control window's update (ui.lua).
-- Licence: GPL-3.0-or-later.
--
-- Each window shows two plots for that wheel:
--   * sideways force against slip angle, for several loads (a family of curves) and for the load the wheel carries now,
--   * driving/braking force against slip ratio, the same way,
-- and a marker for where the wheel is right now. The curves are the tyre's pure-slip Magic Formula, evaluated here with
-- the same module the vehicle runs (pacejka.lua), with the wheel's own nominal load, pressure, surface grip and camber.
-- The marker shows the force the tyre actually gives: when it sits under the curve the tyre is in combined slip
-- (steering and braking/accelerating at once) or the force is still building up.

local M = {}
M.dependencies = {"ui_imgui"}

local im = ui_imgui

local show = im.BoolPtr(false)       -- master switch (checkbox in the control window)
local winOpen = {}                   -- per-window open flags (the close button of each window)
local tyre = nil                     -- the prepared tyre (pacejka.prepare) used for the curves
local tyreKey = nil
local pacMod, pacErr = nil, nil
local cache = {}                     -- curve cache

local FAMILY = {0.5, 1.0, 1.5, 2.0}  -- loads of the family, in multiples of the wheel's static (nominal) load

-- the Pacejka evaluator ------------------------------------------------------------------------------------------
local function loadPac()
  if pacMod or pacErr then return pacMod end
  for _, name in ipairs({"lua/vehicle/extensions/modelTire/pacejka", "vehicle/extensions/modelTire/pacejka"}) do
    local ok, m = pcall(require, name)
    if ok and type(m) == "table" and m._evaluate then pacMod = m; return pacMod end
  end
  local ok, src = pcall(readFile, "/lua/vehicle/extensions/modelTire/pacejka.lua")
  if ok and type(src) == "string" then
    local f, err = (loadstring or load)(src, "=modelTire pacejka")
    if f then
      local ok2, m = pcall(f)
      if ok2 and type(m) == "table" and m._evaluate then pacMod = m; return pacMod end
      err = tostring(m)
    end
    pacErr = "could not load pacejka.lua: " .. tostring(err)
  else
    pacErr = "pacejka.lua not found for the graphs"
  end
  return nil
end

-- called by ui.lua with the tyre table built by svj.lua whenever the selection changes
function M.setTyre(builtTyre, key)
  tyreKey = key
  cache = {}
  tyre = nil
  if not builtTyre then return end
  local p = loadPac()
  if p then tyre = p.prepare(builtTyre) end
end

function M.isShown() return show[0] end
function M.setShown(v) show[0] = v and true or false; for i = 1, 8 do winOpen[i] = v and true or false end end
function M.showPtr() return show end

-- curves ------------------------------------------------------------------------------------------------------------
local out = {}

local function evalForce(kind, x, p, Fz, fz0, ref)
  local kappa, alpha = 0, 0
  if kind == "lat" then alpha = x else kappa = x end
  local gamma = p.gamma or 0
  local lam = ref and 1 or (p.lam or 1)
  local lamK = ref and 1 or (p.lamK or math.sqrt(lam))
  pacMod._evaluate(tyre, fz0, kappa, alpha, tyre.flipLat and -gamma or gamma, Fz, p.dpi or 0, math.max(p.Vx or 10, 1), lam, lamK, out)
  if kind == "lat" then
    local fy = out.Fy
    if tyre.flipLat then fy = -fy end
    return -fy                     -- positive for a positive slip angle: the force that resists the slip
  end
  return out.Fx
end

local N_POINTS = 61

-- cached curve: returns {xs = {...}, ys = {...}}
local function curve(wi, kind, Fz, p, fz0, xr, ref)
  local key = string.format("%d%s|%d|%d|%.2f|%.2f|%.3f|%.2f|%d|%s", wi, kind, math.floor(Fz / 20), math.floor(fz0), p.lam or 1, p.lamK or 0, p.gamma or 0, p.dpi or 0, math.floor(xr * 1000), ref and "r" or "")
  local c = cache[key]
  if c then return c end
  c = {xs = {}, ys = {}}
  for k = 1, N_POINTS do
    local x = -xr + 2 * xr * (k - 1) / (N_POINTS - 1)
    c.xs[k] = x
    c.ys[k] = evalForce(kind, x, p, Fz, fz0, ref)
  end
  -- keep the cache small: drop everything when it grows
  local n = 0
  for _ in pairs(cache) do n = n + 1 end
  if n > 400 then cache = {} end
  cache[key] = c
  return c
end

-- drawing -----------------------------------------------------------------------------------------------------------
-- Pure theoretical curve of the selected tyre for the sweep tests: force / load against slip at one wheel load, on the
-- reference surface (no grip scale, no transient). kind "lat": x in rad of slip angle, y = sideways force / load;
-- kind "lon": x slip ratio, y = driving force / load. Returns nil when no tyre or evaluator is available.
function M.theory(kind, Fz, fz0, xs)
  if not (tyre and pacMod) then return nil end
  local p = {gamma = 0, dpi = 0, Vx = 20, lam = 1, lamK = 1}
  local ys = {}
  for i, x in ipairs(xs) do ys[i] = evalForce(kind, x, p, Fz, fz0 or (tyre.noFz0 and Fz or tyre.Fz0), true) / Fz end
  return ys, tyre.name
end

local function col(r, g, b, a) return im.GetColorU322(im.ImVec4(r, g, b, a or 1)) end

-- One plot. curves = list of {c = curve, color = u32, label = string, thick = number}; marker = {x, y, color}
local function plot(id, w, h, xr, curves, marker, xLabel, yLabel, xScale, xUnit)
  local dl = im.GetWindowDrawList()
  local pos = im.GetCursorScreenPos()
  im.InvisibleButton(id, im.ImVec2(w, h))
  local L, T, R, B = pos.x + 46, pos.y + 6, pos.x + w - 8, pos.y + h - 20
  local W, H = R - L, B - T

  local ym = 1
  for _, cv in ipairs(curves) do
    for k = 1, N_POINTS do local a = math.abs(cv.c.ys[k]); if a > ym then ym = a end end
  end
  if marker and math.abs(marker.y) > ym then ym = math.abs(marker.y) end
  ym = ym * 1.08

  local function sx(x) return L + (x + xr) / (2 * xr) * W end
  local function sy(y) return T + (1 - (y + ym) / (2 * ym)) * H end

  im.ImDrawList_AddRectFilled(dl, im.ImVec2(L, T), im.ImVec2(R, B), col(0.08, 0.09, 0.11, 0.9))
  -- grid and axes
  local grid, axis, txt = col(1, 1, 1, 0.08), col(1, 1, 1, 0.35), col(0.8, 0.82, 0.86, 1)
  for k = -2, 2 do
    local gx = sx(k * xr / 2)
    im.ImDrawList_AddLine(dl, im.ImVec2(gx, T), im.ImVec2(gx, B), (k == 0) and axis or grid, 1)
    local gy = sy(k * ym / 2)
    im.ImDrawList_AddLine(dl, im.ImVec2(L, gy), im.ImVec2(R, gy), (k == 0) and axis or grid, 1)
  end
  im.ImDrawList_AddRect(dl, im.ImVec2(L, T), im.ImVec2(R, B), col(1, 1, 1, 0.25))
  -- tick labels
  im.ImDrawList_AddText1(dl, im.ImVec2(L - 44, T - 2), txt, string.format("%.0f", ym), nil)
  im.ImDrawList_AddText1(dl, im.ImVec2(L - 44, B - 12), txt, string.format("%.0f", -ym), nil)
  im.ImDrawList_AddText1(dl, im.ImVec2(L - 44, sy(0) - 6), txt, "0", nil)
  im.ImDrawList_AddText1(dl, im.ImVec2(L, B + 3), txt, string.format(xScale > 50 and "%.0f" or "%.1f", -xr * xScale), nil)
  im.ImDrawList_AddText1(dl, im.ImVec2(R - 24, B + 3), txt, string.format(xScale > 50 and "%.0f" or "%.1f", xr * xScale), nil)
  im.ImDrawList_AddText1(dl, im.ImVec2(L + W * 0.5 - 40, B + 3), txt, xLabel .. " (" .. xUnit .. ")", nil)
  im.ImDrawList_AddText1(dl, im.ImVec2(L + 4, T + 2), txt, yLabel .. " (N)", nil)

  -- curves
  for ci, cv in ipairs(curves) do
    local c = cv.c
    for k = 1, N_POINTS - 1 do
      im.ImDrawList_AddLine(dl, im.ImVec2(sx(c.xs[k]), sy(c.ys[k])), im.ImVec2(sx(c.xs[k + 1]), sy(c.ys[k + 1])), cv.color, cv.thick or 1.5)
    end
    -- label at the right end, above the last point
    im.ImDrawList_AddText1(dl, im.ImVec2(R - 54, math.max(T, math.min(B - 12, sy(c.ys[N_POINTS]) - 13))), cv.color, cv.label, nil)
  end

  -- the wheel now
  if marker then
    local mx, my = sx(marker.x), sy(marker.y)
    local clamped = false
    if mx < L then mx, clamped = L, true elseif mx > R then mx, clamped = R, true end
    if my < T then my, clamped = T, true elseif my > B then my, clamped = B, true end
    -- guide lines to the axes
    im.ImDrawList_AddLine(dl, im.ImVec2(mx, B), im.ImVec2(mx, my), col(1, 1, 1, 0.25), 1)
    im.ImDrawList_AddLine(dl, im.ImVec2(L, my), im.ImVec2(mx, my), col(1, 1, 1, 0.25), 1)
    if clamped then
      im.ImDrawList_AddCircle(dl, im.ImVec2(mx, my), 6, marker.color, 12, 2)
    else
      im.ImDrawList_AddCircleFilled(dl, im.ImVec2(mx, my), 6, marker.color, 12)
      im.ImDrawList_AddCircle(dl, im.ImVec2(mx, my), 7, col(0, 0, 0, 0.8), 12, 1.5)
    end
  end
end

local famColors = {
  col(0.45, 0.62, 0.95, 0.75), col(0.35, 0.8, 0.85, 0.75), col(0.6, 0.85, 0.5, 0.75), col(0.95, 0.8, 0.4, 0.75),
}

local function drawWheel(wi, w)
  local p = w.pac
  local name = tostring(w.name or wi)
  im.Text(string.format("Fz %.0f N   slip angle %.1f deg   slip ratio %.3f", p.Fz or 0, math.deg(p.alpha or 0), p.kappa or 0))
  im.Text(string.format("Fy %.0f N   Fx %.0f N   nominal load %.0f N", -(p.Fy or 0), p.Fx or 0, p.fz0 or 0))
  if p.hydro then
    im.Text(string.format("hydroplaning %.0f %%   (critical speed %.0f km/h, water %.1f mm)", p.hydro * 100, (p.vp or 0) * 3.6, p.film or 0))
  end
  local fz0 = p.fz0 or tyre.Fz0
  local sp = tyre.slipPeak or 0.12
  local xrLat = math.min(0.6, math.max(0.2, 3 * sp))
  local xrLon = 0.4
  local wPlot, hPlot = 340, 200

  local function family(kind, xr)
    local list = {}
    for k, f in ipairs(FAMILY) do
      local Fz = f * (w.staticLoad or fz0)
      list[#list + 1] = {c = curve(wi, kind, Fz, p, fz0, xr), color = famColors[k], label = string.format("%.0f", Fz), thick = 1.2}
    end
    -- the same load on the surface the coefficients were measured on, to show what the ground changes
    if (p.lam or 1) < 0.97 then
      list[#list + 1] = {c = curve(wi, kind, math.max(p.Fz or fz0, 50), p, fz0, xr, true), color = col(0.7, 0.7, 0.7, 0.7),
        label = "ref", thick = 1.2}
    end
    -- the load now
    list[#list + 1] = {c = curve(wi, kind, math.max(p.Fz or fz0, 50), p, fz0, xr), color = col(1, 1, 1, 1),
      label = "now", thick = 2.5}
    return list
  end

  local latMarker = {x = p.alpha or 0, y = -(p.Fy or 0), color = col(1, 0.35, 0.25, 1)}
  plot("##lat" .. wi, wPlot, hPlot, xrLat, family("lat", xrLat), latMarker, "slip angle", "lateral force", 180 / math.pi, "deg")
  local lonMarker = {x = p.kappa or 0, y = p.Fx or 0, color = col(1, 0.35, 0.25, 1)}
  plot("##lon" .. wi, wPlot, hPlot, xrLon, family("lon", xrLon), lonMarker, "slip ratio", "longitudinal force", 100, "%")
  im.TextDisabled("curves: loads in N (static load x 0.5, 1, 1.5, 2); white = load now; red dot = the wheel; grey = the same load on the tyre's reference surface (when this ground is slipperier)")
end

-- Draws the windows. telemetry: the latest vehicle telemetry; alive: telemetry is fresh and the tyre is on.
function M.draw(telemetry, alive)
  if not show[0] then return end
  if not loadPac() then
    -- one small window with the reason
    if im.Begin("Tyre graphs", show) then im.TextWrapped(tostring(pacErr)) end
    im.End()
    return
  end
  local wheels = telemetry and telemetry.wheels
  for wi = 1, 8 do
    if winOpen[wi] == nil then winOpen[wi] = true end
  end
  local n = wheels and #wheels or 0
  for wi = 1, n do
    local w = wheels[wi]
    if winOpen[wi] then
      im.SetNextWindowSize(im.ImVec2(370, 520), im.Cond_FirstUseEver)
      im.SetNextWindowPos(im.ImVec2(420 + (wi - 1) * 40, 60 + (wi - 1) * 30), im.Cond_FirstUseEver)
      local ptr = im.BoolPtr(true)
      local visible = im.Begin("Tyre graphs: " .. tostring(w.name or wi) .. "###rbrTyreGraph" .. wi, ptr)
      if visible then
        if not tyre then
          im.TextWrapped("Choose an SVJ tyre in the 'SVJ tyre' section first.")
        elseif not (alive and w.pac) then
          im.TextWrapped("Waiting for data: the wheel is in the air or the Pacejka tyre is not running.")
        else
          local ok, err = pcall(drawWheel, wi, w)
          if not ok then im.TextWrapped("graph error: " .. tostring(err)) end
        end
      end
      im.End()
      if not ptr[0] then winOpen[wi] = false end
    end
  end
end

return M
