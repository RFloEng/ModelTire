-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- ModelTire: scripted in-game sweeps. Extension "modelTire_sweep", drawn and driven from ui.lua's update.
-- Licence: GPL-3.0-or-later.
--
-- The script drives the player's car by itself (speed held with a throttle controller, then a slow steering, braking or
-- throttle ramp) while recording the tyre telemetry, so the same manoeuvre can be repeated with each tyre mode and the
-- results laid over each other in one plot. Run it on a large flat area (the Gridmap). It controls the car's inputs:
-- Stop gives them back.
--
-- Plots (all wheels together, so cars and modes compare):
--   lateral:        sideways force / total load against the mean slip angle (steering ramp)
--   longitudinal:   braking or driving force / total load against the mean slip ratio (brake or throttle ramp)
-- The slip is derived from the telemetry's slip velocities over the car's speed; signs differ between the models, so
-- magnitudes are plotted. Runs are kept in memory and written to modelTire/sweeps/<mode>_<kind>.csv.

local M = {}
M.dependencies = {"ui_imgui"}

local im = ui_imgui

local KINDS = {"steering ramp", "braking ramp", "throttle ramp"}
local kindIdx = 1
local VIEWS = {"car (total)", "per axle", "per wheel"}
local viewIdx = 1
local targetKmh = im.FloatPtr(55)
local duration = im.FloatPtr(12)
local steerMax = im.FloatPtr(0.45)
local winOpen = im.BoolPtr(false)

local state = "idle"          -- idle | accelerate | run
local t, runT = 0, 0
local status = "idle"
local current = nil            -- the run being recorded
local runs = {}                -- finished runs, each {label, kind, samples = {...}, color, visible}
local newSample = false
local lastTelemetry = nil
local latestSpeed = 0

local COLORS = {
  {0.35, 0.7, 1, 1}, {1, 0.55, 0.25, 1}, {0.45, 0.9, 0.5, 1}, {1, 0.4, 0.7, 1}, {0.8, 0.8, 0.3, 1}, {0.7, 0.55, 1, 1},
}

local function playerVehicle()
  return getPlayerVehicle and getPlayerVehicle(0) or be:getPlayerVehicle(0)
end

local function sendInput(steer, throttle, brake)
  local veh = playerVehicle()
  if not veh then return end
  veh:queueLuaCommand(string.format(
    "input.event('steering', %.4f, FILTER_DIRECT); input.event('throttle', %.4f, FILTER_DIRECT); input.event('brake', %.4f, FILTER_DIRECT)",
    steer, throttle, brake))
end

local function setNative(on)
  local veh = playerVehicle()
  if veh then
    veh:queueLuaCommand("extensions.load('modelTire_tyre'); if modelTire_tyre then modelTire_tyre.setNative(" .. tostring(on) .. ") end")
  end
end

local function releaseInputs()
  sendInput(0, 0, 0)
  setNative(false)
end

local function modeLabel(tel)
  if tel and not tel.enabled then return "BeamNG vanilla tyre" end
  local m = tel and tel.model or "?"
  -- shorter label for the legend
  m = tostring(m):gsub(", own data", ""):gsub("RBR data, ", ""):gsub(": .*$", "")
  return m
end

local function finish(reason)
  releaseInputs()
  if current and #current.samples > 3 then
    current.color = COLORS[(#runs % #COLORS) + 1]
    current.visible = true
    runs[#runs + 1] = current
    -- keep the CSV next to the other mod data
    local lines = {"t,speed,steer,throttle,brake,ay_ms2,ax_ms2,rack_N,wheel,slip_angle_rad,slip_ratio,Fz,Fx,Fy"}
    for _, s in ipairs(current.samples) do
      for _, w in ipairs(s.w) do
        lines[#lines + 1] = string.format("%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%s,%.4f,%.4f,%.0f,%.0f,%.0f", s.t, s.v, s.steer, s.throttle,
          s.brake, s.acc[1], s.acc[2], s.rack or 0, tostring(w.name), w.alpha or 0, w.kappa or 0, w.Fz or 0, w.Fx or 0, w.Fy or 0)
      end
    end
    local name = string.format("%s_%s_%d", (current.label:gsub("[^%w]+", "_")), (current.kind:gsub("%s", "_")), #runs)
    local path = "/modelTire/sweeps/" .. name .. ".csv"
    local ok = pcall(function()
      if FS and FS.directoryCreate then FS:directoryCreate("modelTire/sweeps", true) end
      writeFile(path, table.concat(lines, "\n"))
    end)
    status = string.format("%s: %d samples recorded%s", reason, #current.samples, ok and (" (saved " .. path .. ")") or " (kept in memory only)")
  else
    status = reason .. " (nothing recorded)"
  end
  current = nil
  state = "idle"
end

local function start()
  local veh = playerVehicle()
  if not veh then status = "no player vehicle"; return end
  -- the Lua tyre may be on (a tyre mode) or off (BeamNG's own tyre: the "vanilla" reference run)
  setNative(true)
  current = {label = modeLabel(lastTelemetry), kind = KINDS[kindIdx], samples = {}, staticLoad = nil,
    native = not (lastTelemetry and lastTelemetry.enabled), wheelStatic = {},
    note = string.format("%.0f km/h, %.0f s%s", targetKmh[0], duration[0], KINDS[kindIdx] == "steering ramp" and string.format(", lock %.2f", steerMax[0]) or "")}
  state = "accelerate"
  runT, t = 0, 0
  status = "accelerating to " .. string.format("%.0f", targetKmh[0]) .. " km/h"
end

-- driving: called every frame with the real time step
function M.update(dt)
  if state == "idle" then return end
  local veh = playerVehicle()
  if not veh then finish("vehicle lost"); return end
  local v = veh:getVelocity():length()
  latestSpeed = v
  local target = targetKmh[0] / 3.6
  t = t + dt
  -- speed hold: proportional throttle, no brake unless well over
  local function hold()
    local e = target - v
    return math.max(0, math.min(1, 0.12 + 0.35 * e)), (e < -3) and math.min(1, (-e - 3) * 0.2) or 0
  end
  local kind = KINDS[kindIdx]
  if state == "accelerate" then
    local thr, brk = hold()
    sendInput(0, thr, brk)
    if v >= target - 0.5 and t > 1 then
      state = "run"; runT = 0; status = "running: " .. kind
    elseif t > 40 then finish("could not reach the speed (open area needed)") end
    return
  end
  -- state == "run"
  runT = runT + dt
  local u = math.min(runT / duration[0], 1)
  local steer, throttle, brake = 0, 0, 0
  if kind == "steering ramp" then
    local thr, brk = hold()
    steer, throttle, brake = u * steerMax[0], thr, brk
  elseif kind == "braking ramp" then
    steer, throttle, brake = 0, 0, u
  else -- throttle ramp: start from a low speed in the same gear: the car holds target/3 then floors it gradually
    steer, throttle, brake = 0, u, 0
  end
  sendInput(steer, throttle, brake)
  current.cmd = {steer = steer, throttle = throttle, brake = brake}
  if newSample and lastTelemetry and lastTelemetry.wheels and lastTelemetry.acc then
    newSample = false
    local s = {t = runT, v = v, steer = steer, throttle = throttle, brake = brake, acc = lastTelemetry.acc, rack = lastTelemetry.rack, w = {}}
    local ok = true
    local static, n = 0, 0
    for i, w in ipairs(lastTelemetry.wheels) do
      if not w.kin then ok = false; break end
      s.w[i] = {name = w.name, Fz = w.Fz, Fx = w.Fx, Fy = w.Fy, alpha = w.kin[1], kappa = w.kin[2]}
      static, n = static + (w.staticLoad or 0), n + 1
      current.wheelStatic[i] = w.staticLoad
    end
    if ok then
      current.samples[#current.samples + 1] = s
      if n > 0 then current.staticLoad = static / n end
      current.label = modeLabel(lastTelemetry)
      current.native = not lastTelemetry.enabled
      current.fz0 = lastTelemetry.wheels[1] and lastTelemetry.wheels[1].fz0
    end
  end
  if runT >= duration[0] then finish("finished")
  elseif kind == "braking ramp" and v < 1.5 then finish("stopped")
  elseif kind == "steering ramp" and v < 0.5 * target then finish("speed lost (stopped early)")
  elseif kind == "steering ramp" and current.samples[#current.samples] then
    -- the car has spun: past 30 degrees of mean slip the rest of the ramp is not a tyre curve any more
    local last, sum, n = current.samples[#current.samples], 0, 0
    for _, w in ipairs(last.w) do sum, n = sum + math.abs(w.alpha or 0), n + 1 end
    if n > 0 and math.deg(sum / n) > 30 then finish("slip angle above 30 degrees: the car is spinning, stopped") end
  end
end

function M.onTelemetry(data)
  lastTelemetry = data
  newSample = true
end

-- derived curves ----------------------------------------------------------------------------------------------------
-- points {x = mean slip of the wheels, y = force / weight}: the force on the car is its mass times the measured
-- acceleration, and the load is its weight, so the quotient is acceleration in g: the same quantity for every mode and
-- for BeamNG's own tyre. Slip comes from the wheels' own kinematics (hub velocity against the wheel's heading).
-- group: nil = the whole car, else a list of wheel indices (one wheel or an axle). For a Lua tyre run the force of the group
-- is the tyre's own force over its own load. BeamNG's native tyre gives no force per wheel, so a vanilla run is drawn from
-- the car's acceleration in g (an estimate that assumes every wheel uses the same share of its grip).
local function curve(run, lateral, group)
  local pts = {}
  for _, s in ipairs(run.samples) do
    local slip, n, f, fz = 0, 0, 0, 0
    for idx, w in ipairs(s.w) do
      local use = true
      if group then
        use = false
        for _, g in ipairs(group) do if g == idx then use = true end end
      end
      if use then
        slip = slip + (lateral and math.deg(math.abs(w.alpha or 0)) or math.abs(w.kappa or 0))
        f = f + (lateral and math.abs(w.Fy or 0) or (w.Fx or 0))
        fz = fz + (w.Fz or 0)
        n = n + 1
      end
    end
    if n > 0 and s.v > 2 then
      local y
      if group and not run.native then
        if fz > 100 then y = math.abs(f) / fz end
      else
        y = math.abs(lateral and s.acc[1] or s.acc[2]) / 9.81
      end
      if y then pts[#pts + 1] = {slip / n, y} end
    end
  end
  return pts
end

-- steering rack force (N, magnitude) against the mean slip angle of the wheels, for steering runs
local function rackCurve(run)
  local pts = {}
  for _, s in ipairs(run.samples) do
    local slip, n = 0, 0
    for _, w in ipairs(s.w) do slip, n = slip + math.deg(math.abs(w.alpha or 0)), n + 1 end
    if n > 0 and s.v > 2 and s.rack then pts[#pts + 1] = {slip / n, math.abs(s.rack)} end
  end
  return pts
end

-- the selected Pacejka tyre's pure-slip curve at the car's mean static wheel load (reference surface)
local function theoryCurve(lateral, run, group)
  if not modelTire_graphs or not modelTire_graphs.theory then return nil end
  local Fz = (run and run.staticLoad) or 3500
  if group and run and run.wheelStatic then
    local sum, n = 0, 0
    for _, g in ipairs(group) do if run.wheelStatic[g] then sum, n = sum + run.wheelStatic[g], n + 1 end end
    if n > 0 then Fz = sum / n end
  end
  local xs, disp = {}, {}
  local xmax = lateral and 20 or 0.5
  for k = 0, 40 do
    local x = xmax * k / 40
    disp[k + 1] = x
    xs[k + 1] = lateral and math.rad(x) or x
  end
  local ys, name = modelTire_graphs.theory(lateral and "lat" or "lon", Fz, run and run.fz0, xs)
  if not ys then return nil end
  local pts = {}
  for k = 1, #ys do pts[k] = {disp[k], math.abs(ys[k])} end
  return pts, name
end

local function col(r, g, b, a) return im.GetColorU322(im.ImVec4(r, g, b, a or 1)) end

local function plot(id, w, h, lateral, xmax, xlabel, ylabel, group, title, quantity)
  local dl = im.GetWindowDrawList()
  local pos = im.GetCursorScreenPos()
  im.InvisibleButton(id, im.ImVec2(w, h))
  local L, T, R, B = pos.x + 44, pos.y + 6, pos.x + w - 8, pos.y + h - 20
  im.ImDrawList_AddRectFilled(dl, im.ImVec2(L, T), im.ImVec2(R, B), col(0.08, 0.09, 0.11, 0.9))
  local curves, ymax = {}, (quantity == "rack") and 100 or 0.5
  for _, r in ipairs(runs) do
    if r.visible and ((lateral and r.kind == "steering ramp") or (not lateral and r.kind ~= "steering ramp")) then
      local c = (quantity == "rack") and rackCurve(r) or curve(r, lateral, group)
      curves[#curves + 1] = {c = c, color = r.color, label = r.label, dotted = group and r.native}
      for _, p in ipairs(c) do if math.abs(p[2]) > ymax then ymax = math.abs(p[2]) end end
    end
  end
  -- theoretical curve of the selected tyre, from the first visible run's static load
  local ref
  for _, r in ipairs(runs) do
    if r.visible and ((lateral and r.kind == "steering ramp") or (not lateral and r.kind ~= "steering ramp")) then ref = r; break end
  end
  local theory, theoryName
  if ref and quantity ~= "rack" then theory, theoryName = theoryCurve(lateral, ref, group) end
  if theory then for _, p in ipairs(theory) do if p[2] > ymax then ymax = p[2] end end end
  ymax = ymax * 1.1
  local function sx(x) return L + math.min(x / xmax, 1) * (R - L) end
  local function sy(y) return B - (y / ymax) * (B - T) end
  local txt = col(0.8, 0.82, 0.86, 1)
  for k = 0, 4 do
    local gy = B - k / 4 * (B - T)
    im.ImDrawList_AddLine(dl, im.ImVec2(L, gy), im.ImVec2(R, gy), col(1, 1, 1, 0.08), 1)
    im.ImDrawList_AddText1(dl, im.ImVec2(L - 40, gy - 6), txt, string.format("%.2f", ymax * k / 4), nil)
    local gx = L + k / 4 * (R - L)
    im.ImDrawList_AddText1(dl, im.ImVec2(gx - 10, B + 3), txt, string.format(xmax > 2 and "%.0f" or "%.2f", xmax * k / 4), nil)
  end
  im.ImDrawList_AddRect(dl, im.ImVec2(L, T), im.ImVec2(R, B), col(1, 1, 1, 0.25))
  im.ImDrawList_AddText1(dl, im.ImVec2(L + 4, T + 2), txt, title or (ylabel .. " vs " .. xlabel), nil)
  if theory then
    local rgba = col(0.85, 0.85, 0.85, 0.9)
    for k = 1, #theory - 1, 2 do   -- dashed
      im.ImDrawList_AddLine(dl, im.ImVec2(sx(theory[k][1]), sy(theory[k][2])), im.ImVec2(sx(theory[k + 1][1]), sy(theory[k + 1][2])), rgba, 1.5)
    end
    im.ImDrawList_AddText1(dl, im.ImVec2(R - 230, B - 16), rgba, "- - theory: " .. tostring(theoryName):sub(1, 26) .. " (pure slip)", nil)
  end
  for ci, cv in ipairs(curves) do
    local c = cv.c
    local rgba = col(cv.color[1], cv.color[2], cv.color[3], cv.color[4])
    -- consecutive samples are joined only when they are close: a wheel that locks jumps in slip, and a line across the
    -- plot between unrelated points is noise, not a curve. Points past the end of the axis are not drawn at all.
    for k = 1, #c - 1 do
      local a, b = c[k], c[k + 1]
      if a[1] <= xmax and b[1] <= xmax and math.abs(b[1] - a[1]) < 0.12 * xmax and math.abs(b[2] - a[2]) < 0.2 * ymax
          and (not cv.dotted or k % 2 == 1) then   -- a vanilla run seen per wheel is an estimate: drawn dashed
        im.ImDrawList_AddLine(dl, im.ImVec2(sx(a[1]), sy(math.max(a[2], 0))), im.ImVec2(sx(b[1]), sy(math.max(b[2], 0))), rgba, 2)
      end
    end
    for k = 1, #c do
      if c[k][1] <= xmax then im.ImDrawList_AddCircleFilled(dl, im.ImVec2(sx(c[k][1]), sy(math.max(c[k][2], 0))), 2, rgba, 8) end
    end
  end
end

-- window ------------------------------------------------------------------------------------------------------------
function M.toggle() winOpen[0] = not winOpen[0] end
function M.setView(i) viewIdx = i end
function M.isOpen() return winOpen[0] end

function M.draw()
  if not winOpen[0] then return end
  im.SetNextWindowSize(im.ImVec2(520, 760), im.Cond_FirstUseEver)
  if im.Begin("ModelTire sweep tests", winOpen) then
    im.TextWrapped("Drives the player's car by itself and records the tyre forces, so the same manoeuvre can be compared "
      .. "between tyre modes. Use a large flat area (the Gridmap). Switch the Lua tyre on and choose the mode first.")
    im.PushItemWidth(240)
    if im.BeginCombo("test", KINDS[kindIdx]) then
      for i, k in ipairs(KINDS) do if im.Selectable1(k, i == kindIdx) then kindIdx = i end end
      im.EndCombo()
    end
    im.SliderFloat("speed (km/h)", targetKmh, 20, 120, "%.0f")
    im.SliderFloat("ramp duration (s)", duration, 4, 30, "%.0f")
    if KINDS[kindIdx] == "steering ramp" then im.SliderFloat("steering at the end (0-1)", steerMax, 0.1, 1.0, "%.2f") end
    im.PopItemWidth()
    if state == "idle" then
      if im.Button("Start") then start() end
    else
      if im.Button("Stop") then finish("stopped by you") end
    end
    im.SameLine()
    if im.Button("Clear runs") then runs = {} end
    im.TextWrapped("status: " .. status)
    im.Separator()
    -- the slip axis follows the data (a run can slide past 20 degrees)
    local latMax = 20
    for _, r in ipairs(runs) do
      if r.visible and r.kind == "steering ramp" then
        for _, p in ipairs(curve(r, true)) do if p[1] > latMax then latMax = math.min(math.ceil(p[1] / 5) * 5, 45) end end
      end
    end
    im.PushItemWidth(200)
    if im.BeginCombo("curves of", VIEWS[viewIdx]) then
      for i, v in ipairs(VIEWS) do if im.Selectable1(v, i == viewIdx) then viewIdx = i end end
      im.EndCombo()
    end
    im.PopItemWidth()
    -- wheel groups from the names of the first run that has samples (F.. = front, R.. = rear)
    local names = {}
    for _, r in ipairs(runs) do
      if r.samples[1] then for i, w in ipairs(r.samples[1].w) do names[i] = tostring(w.name) end break end
    end
    if viewIdx == 1 or #names == 0 then
      plot("##swlat", 480, 230, true, latMax, "slip angle (deg)", "lateral acceleration (g)")
      plot("##swlon", 480, 230, false, 0.5, "slip ratio", "longitudinal acceleration (g)")
      plot("##swrack", 480, 200, true, latMax, "slip angle (deg)", "steering rack force (N)", nil, "steering rack force (N) vs slip angle", "rack")
    else
      local groups = {}
      if viewIdx == 2 then
        local f, r = {}, {}
        for i, n in ipairs(names) do if n:sub(1, 1) == "F" then f[#f + 1] = i else r[#r + 1] = i end end
        if #f > 0 then groups[#groups + 1] = {"front axle", f} end
        if #r > 0 then groups[#groups + 1] = {"rear axle", r} end
      else
        for i, n in ipairs(names) do groups[#groups + 1] = {"wheel " .. n, {i}} end
      end
      local hasLat, hasLon = false, false
      for _, r in ipairs(runs) do
        if r.visible then
          if r.kind == "steering ramp" then hasLat = true else hasLon = true end
        end
      end
      local pw = (hasLat and hasLon) and 245 or 490
      if hasLat then plot("##swrack", 490, 190, true, latMax, "slip angle (deg)", "steering rack force (N)", nil, "steering rack force (N) vs slip angle", "rack") end
      for gi, g in ipairs(groups) do
        if hasLat then plot("##swlat" .. gi, pw, 190, true, latMax, "slip angle (deg)", "", g[2], g[1] .. ": lateral force / load") end
        if hasLat and hasLon then im.SameLine() end
        if hasLon then plot("##swlon" .. gi, pw, 190, false, 0.5, "slip ratio", "", g[2], g[1] .. ": longitudinal force / load") end
      end
    end
    for i, r in ipairs(runs) do
      local vis = im.BoolPtr(r.visible)
      im.TextColored(im.ImVec4(r.color[1], r.color[2], r.color[3], 1), "====")
      im.SameLine()
      if im.Checkbox(string.format("%d: %s, %s (%s)##run%d", i, r.label, r.kind, r.note or "", i), vis) then r.visible = vis[0] end
    end
    if viewIdx > 1 then
      im.TextDisabled("Per axle / per wheel: a Lua-tyre run is the tyre's own force over its own load against that wheel's own slip. "
        .. "BeamNG's native tyre reports no force per wheel, so a vanilla run is dashed: the car's acceleration in g, "
        .. "assuming every wheel uses the same share of its grip.")
    end
    im.TextDisabled("Total view: force/weight is the car's measured acceleration in g (the same for every mode and for BeamNG's own tyre); slip is "
      .. "the mean of the wheels' own slip (magnitudes). Dashed: the chosen SVJ tyre's pure-slip curve at the mean static wheel "
      .. "load, flat road, no weight transfer. Run with the Lua tyre off for the BeamNG vanilla reference.")
  end
  im.End()
end

return M
