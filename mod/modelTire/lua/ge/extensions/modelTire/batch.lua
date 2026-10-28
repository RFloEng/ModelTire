-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- ModelTire: automatic test. Extension "modelTire_batch", drawn and driven from ui.lua's update.
-- Licence: GPL-3.0-or-later.
--
-- Runs the test sled's slip-angle ramp (sweep.lua, "rig slip ramp") for every chosen tyre model and ground surface, one after
-- the other, with no input from the player: for each run it sets the mode and the surface (ui.lua testSetup), reloads the vehicle
-- (a run can capsize the sled, and a reload gives a new one; a FULL mode needs the reload anyway), waits for the tyre to be ready,
-- puts the sled back at the pose it had when the test started, runs the ramp and keeps the result. The runs go to modelTire/sweeps/batch_<date>/ as one CSV each plus manifest.json (settings, tyres, what every run
-- was and how it ended). tools/report.py turns that folder into the report (tables and plots, one HTML file).
--
-- Surfaces: the Lua tyre can be told the surface only in the FULL modes (the BeamNG tyre of the other modes meets the real
-- ground, whose material the game decides); those modes, and BeamNG's own tyre, run once on whatever the sled stands on and the
-- material the game reports is written into the manifest. To compare them on other materials, move the sled to another patch
-- of the map and run the test again (the report merges folders).

local M = {}
M.dependencies = {"ui_imgui"}

local im = ui_imgui

local FAMILIES = {
  {key = "vanilla",   name = "BeamNG vanilla tyre (Lua tyre off)", vanilla = true},
  {key = "brush",     name = "Brush test tyre", mode = 0},
  {key = "adv_brush", name = "Advanced brush", mode = 5},
  {key = "rbr_light", name = "RBR 1.02 on BeamNG's tyre", mode = 1},
  {key = "svj_light", name = "SVJ Pacejka on BeamNG's tyre", mode = 4, svj = true},
  {key = "rbr_full",  name = "FULL RBR tyre", mode = 2, surfaces = "full"},
  {key = "svj_full",  name = "FULL SVJ Pacejka tyre", mode = 3, svj = true, surfaces = "svj"},
}
local famOn = {}
for _, f in ipairs(FAMILIES) do famOn[f.key] = im.BoolPtr(true) end
local surfOn = {full = {}, svj = {}}      -- surface name -> BoolPtr, made when the lists are known
local speedKmh, rampSec, maxSlip = im.FloatPtr(55), im.FloatPtr(12), im.FloatPtr(20)
local winOpen = im.BoolPtr(false)

local S = {phase = "idle", jobs = {}, cur = 0, t = 0, stable = 0, message = "idle"}
local clock = 0
local tel, telAt = nil, -1000

-- helpers ---------------------------------------------------------------------------------------------------------------
local function playerVehicle()
  return getPlayerVehicle and getPlayerVehicle(0) or be:getPlayerVehicle(0)
end

local function isSled(veh)
  if not veh then return false end
  local jb = veh.jbeam
  if jb == nil and veh.getJBeamFilename then jb = veh:getJBeamFilename() end
  return jb == "modelTire_rig"
end

local function safe(s) return (tostring(s):gsub("[^%w]+", "_")) end

local function info() return modelTire_ui and modelTire_ui.testInfo and modelTire_ui.testInfo() or nil end

local function ensureSurfaceLists()
  local inf = info()
  if not inf then return nil end
  for _, n in ipairs(inf.fullSurfaces) do if not surfOn.full[n] then surfOn.full[n] = im.BoolPtr(true) end end
  for _, n in ipairs(inf.svjSurfaces) do if not surfOn.svj[n] then surfOn.svj[n] = im.BoolPtr(true) end end
  return inf
end

local function buildJobs()
  local inf = ensureSurfaceLists()
  local jobs, skipped = {}, {}
  if not inf then return jobs, {"the control window is not ready"} end
  for _, f in ipairs(FAMILIES) do
    if famOn[f.key][0] then
      if f.svj and not inf.svjTyre then
        skipped[#skipped + 1] = f.name .. ": no SVJ tyre selected in the control window"
      elseif f.surfaces == "full" then
        for _, n in ipairs(inf.fullSurfaces) do
          if surfOn.full[n][0] then jobs[#jobs + 1] = {family = f, surface = n, setup = {mode = f.mode, fullSurface = n}} end
        end
      elseif f.surfaces == "svj" then
        for _, n in ipairs(inf.svjSurfaces) do
          if surfOn.svj[n][0] then jobs[#jobs + 1] = {family = f, surface = n, setup = {mode = f.mode, svjSurface = n}} end
        end
      elseif f.vanilla then
        jobs[#jobs + 1] = {family = f, setup = {vanilla = true}}
      else
        jobs[#jobs + 1] = {family = f, setup = {mode = f.mode}}
      end
    end
  end
  for i, j in ipairs(jobs) do
    j.setup.forceReload = true
    j.idx = i
    j.tag = string.format("%02d_%s%s", i, j.family.key, j.surface and ("_" .. safe(j.surface)) or "")
    j.status = "waiting"
  end
  return jobs, skipped
end

-- results and report ----------------------------------------------------------------------------------------------------
local function movavg(v, n)
  local out, h = {}, math.floor(n / 2)
  for i = 1, #v do
    local a, b, s = math.max(1, i - h), math.min(#v, i + h), 0
    for k = a, b do s = s + v[k] end
    out[i] = s / (b - a + 1)
  end
  return out
end

-- curve: {{slip deg, g}, ...} in time order -> the numbers of the report and the smoothed curve (thinned) for the plots
local function analyse(curve)
  if not curve or #curve < 10 then return nil end
  local sl, g = {}, {}
  for i, p in ipairs(curve) do sl[i], g[i] = p[1], p[2] end
  local sm = movavg(g, 7)
  local n, ip = #sm, 1
  for i = 1, n do if sm[i] > sm[ip] then ip = i end end
  local function at(x)
    if sl[n] < x then return nil end
    local best = 1
    for i = 1, n do if math.abs(sl[i] - x) < math.abs(sl[best] - x) then best = i end end
    return sm[best]
  end
  local s0, s1, tail, cnt = 0, 0, 0, 0
  for i = 1, n do if sl[i] <= 2 then s0 = s0 + sl[i] * sm[i]; s1 = s1 + sl[i] * sl[i] end end
  for i = math.max(1, math.floor(n * 0.9)), n do tail = tail + sm[i]; cnt = cnt + 1 end
  local rs = 0
  for i = 1, n do rs = rs + (g[i] - sm[i]) ^ 2 end
  local m = {peak = sm[ip], slipPeak = sl[ip], stiff = s1 > 0 and s0 / s1 or nil, g5 = at(5), g10 = at(10), gEnd = tail / cnt,
    noise = (sm[ip] > 0) and math.sqrt(rs / n) / sm[ip] or nil, maxSlip = sl[n]}
  m.keep = m.peak > 0 and m.gEnd / m.peak or nil
  local pts, step = {}, math.max(1, math.floor(n / 120))
  for i = 1, n, step do pts[#pts + 1] = {sl[i], sm[i]} end
  return m, pts
end

local FAM_COLOR = {vanilla = "#2b7bd6", brush = "#e07b1f", adv_brush = "#8a5a2b", rbr_light = "#3aa04a", svj_light = "#d8479a", rbr_full = "#b59a14", svj_full = "#7d56d6"}
local function hsv(k, n)       -- distinct colours for the surfaces of one family
  local h = (k - 1) / math.max(1, n) * 300
  local c, x = 0.75, 0.75 * (1 - math.abs((h / 60) % 2 - 1))
  local r, g, b = 0, 0, 0
  if h < 60 then r, g, b = c, x, 0 elseif h < 120 then r, g, b = x, c, 0 elseif h < 180 then r, g, b = 0, c, x
  elseif h < 240 then r, g, b = 0, x, c else r, g, b = x, 0, c end
  local m = 0.1
  return string.format("#%02x%02x%02x", math.floor((r + m) * 255), math.floor((g + m) * 255), math.floor((b + m) * 255))
end

-- an SVG plot of curves: list of {pts, color, dash}
local function svgPlot(curves, title, xmax, ymax)
  local W, H, L, T, R, B = 760, 380, 52, 28, 10, 40
  local o = {string.format("<svg xmlns='http://www.w3.org/2000/svg' width='%d' height='%d' viewBox='0 0 %d %d' font-family='sans-serif' font-size='11'>", W, H, W, H)}
  o[#o + 1] = string.format("<rect x='0' y='0' width='%d' height='%d' fill='#fff'/>", W, H)
  local function sx(x) return L + math.min(x / xmax, 1) * (W - L - R) end
  local function sy(y) return H - B - (y / ymax) * (H - T - B) end
  for k = 0, 4 do
    local y = sy(ymax * k / 4)
    o[#o + 1] = string.format("<line x1='%d' y1='%.1f' x2='%d' y2='%.1f' stroke='#e3e6ea'/><text x='%d' y='%.1f' text-anchor='end'>%.2f</text>", L, y, W - R, y, L - 4, y + 4, ymax * k / 4)
    local x = sx(xmax * k / 4)
    o[#o + 1] = string.format("<text x='%.1f' y='%d' text-anchor='middle'>%.0f</text>", x, H - B + 14, xmax * k / 4)
  end
  o[#o + 1] = string.format("<text x='%d' y='%d' text-anchor='middle'>mean slip angle (deg)</text>", (L + W - R) / 2, H - 6)
  o[#o + 1] = string.format("<text x='%d' y='14' font-weight='bold'>%s</text>", L, title)
  for _, c in ipairs(curves) do
    local pts = {}
    for _, p in ipairs(c.pts) do pts[#pts + 1] = string.format("%.1f,%.1f", sx(p[1]), sy(p[2])) end
    o[#o + 1] = string.format("<polyline fill='none' stroke='%s' stroke-width='%s' %s points='%s'/>", c.color, c.dash and "1.4" or "2",
      c.dash and "stroke-dasharray='5 3'" or "", table.concat(pts, " "))
  end
  o[#o + 1] = "</svg>"
  return table.concat(o)
end

local function esc(s) return (tostring(s):gsub("&", "&amp;"):gsub("<", "&lt;")) end
local function f2(x, f) return x and string.format(f or "%.2f", x) or "-" end
local function swatches(list)
  local l = {}
  for _, c in ipairs(list) do l[#l + 1] = "<span style='color:" .. c.color .. "'>&#9632;</span> " .. esc(c.label) end
  return "<p class='small'>" .. table.concat(l, " &nbsp; ") .. "</p>"
end

-- report.html and summary.csv in the batch folder, written by the game itself (tools/report.py makes a richer one with Python)
local function writeReport()
  if not S.folder then return end
  local ok = {}
  for _, j in ipairs(S.jobs) do if j.status == "ok" and j.m then ok[#ok + 1] = j end end
  if #ok == 0 then return end
  local function label(j) return j.surface or ("ground: " .. tostring(j.detected or "?")) end
  local refs, forced = {}, {rbr_full = {}, svj_full = {}}
  for _, j in ipairs(ok) do
    if j.family.surfaces then forced[j.family.key][#forced[j.family.key] + 1] = j else refs[#refs + 1] = j end
  end
  local xmax, ymax = 5, 0.5
  for _, j in ipairs(ok) do xmax = math.max(xmax, j.m.maxSlip); ymax = math.max(ymax, j.m.peak * 1.15) end
  local h = {}
  h[#h + 1] = "<!doctype html><html><head><meta charset='utf-8'><title>ModelTire tyre model test</title><style>"
    .. "body{font-family:system-ui,Segoe UI,Arial,sans-serif;max-width:1100px;margin:24px auto;padding:0 16px;color:#1b1f24}"
    .. "table{border-collapse:collapse;font-size:13px;margin:8px 0 18px}th,td{border:1px solid #cdd3da;padding:4px 8px;text-align:right}"
    .. "th{background:#eef1f4}td.l,th.l{text-align:left}.bad{color:#b3441a}.small{color:#555;font-size:12px}"
    .. "h2{margin-top:30px;border-bottom:1px solid #cdd3da;padding-bottom:4px}</style></head><body>"
  h[#h + 1] = "<h1>ModelTire tyre model test</h1>"
  h[#h + 1] = string.format("<p class='small'>%s. Test sled, %.0f km/h, slip angle ramp to %.0f degrees over %.0f s. SVJ tyre: %s, RBR tyre %s. "
    .. "Lateral force over load (g) against the mean slip angle of the wheels.</p>", esc(S.created), S.speedKmh, S.maxSlip, S.rampSec,
    esc(S.svjTyre or "none"), esc(S.rbrTyre or "?"))
  h[#h + 1] = "<h2>Summary</h2><table><tr><th class='l'>tyre model</th><th class='l'>surface</th><th>peak (g)</th><th>slip at peak (deg)</th><th>stiffness (g/deg)</th>"
    .. "<th>g at 5&deg;</th><th>g at 10&deg;</th><th>g at the end</th><th>kept after the peak</th><th>noise</th><th>wheel speed jitter</th><th>reached (deg)</th></tr>"
  for _, j in ipairs(ok) do
    local m = j.m
    local jit = j.kappaSd and string.format("<td%s>%.3f</td>", j.kappaSd > 0.05 and " class='bad'" or "", j.kappaSd) or "<td>-</td>"
    h[#h + 1] = string.format("<tr><td class='l'>%s</td><td class='l'>%s</td><td><b>%s</b></td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td>%s<td>%s</td></tr>",
      esc(j.family.name), esc(label(j)), f2(m.peak), f2(m.slipPeak, "%.1f"), f2(m.stiff, "%.3f"), f2(m.g5), f2(m.g10), f2(m.gEnd),
      m.keep and string.format("%.0f%%", m.keep * 100) or "-", m.noise and string.format("%.1f%%", m.noise * 100) or "-", jit, f2(m.maxSlip, "%.1f"))
  end
  h[#h + 1] = "</table>"
  if #refs > 0 then
    local cs = {}
    for _, j in ipairs(refs) do cs[#cs + 1] = {pts = j.pts, color = FAM_COLOR[j.family.key] or "#444", label = j.family.name} end
    h[#h + 1] = "<h2>Tyre models on the ground under the sled</h2>" .. svgPlot(cs, "lateral force / load (g)", xmax, ymax) .. swatches(cs)
  end
  for _, key in ipairs({"rbr_full", "svj_full"}) do
    local list = forced[key]
    if #list > 0 then
      local cs, names = {}, {}
      for _, j in ipairs(refs) do cs[#cs + 1] = {pts = j.pts, color = FAM_COLOR[j.family.key] or "#444", dash = true} end
      for k, j in ipairs(list) do
        cs[#cs + 1] = {pts = j.pts, color = hsv(k, #list)}
        names[#names + 1] = {color = hsv(k, #list), label = j.surface}
      end
      h[#h + 1] = "<h2>" .. esc(list[1].family.name) .. ", every surface</h2>"
        .. svgPlot(cs, "lateral force / load (g); dashed: the tyre models on the ground under the sled", xmax, ymax) .. swatches(names)
    end
  end
  local bad = {}
  for _, j in ipairs(S.jobs) do if j.status ~= "ok" then bad[#bad + 1] = j end end
  h[#h + 1] = "<h2>Runs that did not give a result</h2>"
  if #bad == 0 then
    h[#h + 1] = "<p>none.</p>"
  else
    h[#h + 1] = "<table><tr><th class='l'>run</th><th class='l'>status</th><th class='l'>reason</th></tr>"
    for _, j in ipairs(bad) do
      h[#h + 1] = string.format("<tr><td class='l'>%s</td><td class='l'>%s</td><td class='l'>%s</td></tr>", esc(j.tag), esc(j.status), esc(j.reason or ""))
    end
    h[#h + 1] = "</table>"
  end
  h[#h + 1] = "<h2>How to read this</h2><ul class='small'><li>BeamNG's own tyre and the modes that keep BeamNG's tyre on the ground ran on the ground under the "
    .. "sled; only the FULL modes were given a surface. To compare the others on another material, move the sled and run again.</li>"
    .. "<li>The sled has no steering, no drive and no weight transfer: these curves are the tyre alone.</li>"
    .. "<li>Wheel speed jitter: the spread of a wheel's slip ratio over the run. A free-rolling wheel stays near 0 (BeamNG's own tyre: 0.004); red = above 0.05, a wheel speed that rings and starves the lateral force.</li><li>One run per combination: a small difference is not necessarily real. tools/report.py makes the same report with more plots (needs Python and matplotlib).</li></ul></body></html>"
  local base = "/modelTire/sweeps/" .. S.folder .. "/"
  local csvl = {"family,surface,peak_g,slip_at_peak_deg,stiffness_g_per_deg,g_at_5deg,g_at_10deg,g_end,kept,noise,max_slip_deg"}
  for _, j in ipairs(ok) do
    local m = j.m
    csvl[#csvl + 1] = string.format("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", j.family.key, label(j), f2(m.peak, "%.4f"), f2(m.slipPeak), f2(m.stiff, "%.4f"),
      f2(m.g5, "%.4f"), f2(m.g10, "%.4f"), f2(m.gEnd, "%.4f"), f2(m.keep, "%.3f"), f2(m.noise, "%.4f"), f2(m.maxSlip, "%.1f"))
  end
  local good = pcall(function()
    writeFile(base .. "report.html", table.concat(h, "\n"))
    writeFile(base .. "summary.csv", table.concat(csvl, "\n"))
  end)
  S.reportPath = good and (base .. "report.html") or nil
  return good
end

local function perJobSeconds()
  local accel = (speedKmh[0] / 3.6) / 2.0     -- the sled speeds up at 2 m/s^2
  return accel + 1.5 + rampSec[0] + 0.5 + 14  -- + hold, ramp end, and reload, set-up and settle between runs
end

local function writeManifest()
  if not S.folder then return end
  local inf = info() or {}
  local m = {
    created = S.created, folder = S.folder, finished = S.finished or false, aborted = S.aborted or false,
    speedKmh = S.speedKmh, rampSec = S.rampSec, maxSlip = S.maxSlip, vehicle = "modelTire_rig",
    svjTyre = inf.svjTyre, svjFile = inf.svjFile, rbrTyre = inf.rbrTyre, rbrGripScale = inf.rbrGripScale, svjGripScale = inf.svjGripScale,
    families = {}, jobs = {},
  }
  for _, f in ipairs(FAMILIES) do m.families[#m.families + 1] = {key = f.key, name = f.name, surfaces = f.surfaces or "ground"} end
  for _, j in ipairs(S.jobs) do
    m.jobs[#m.jobs + 1] = {idx = j.idx, family = j.family.key, familyName = j.family.name, surface = j.surface, tag = j.tag,
      status = j.status, reason = j.reason, samples = j.samples, file = j.file, detectedSurface = j.detected, model = j.model,
      staticLoad = j.staticLoad, wheelJitter = j.kappaSd}
  end
  pcall(function()
    if FS and FS.directoryCreate then FS:directoryCreate("modelTire/sweeps/" .. S.folder, true) end
    jsonWriteFile("/modelTire/sweeps/" .. S.folder .. "/manifest.json", m, true)
  end)
end

-- the tyre is ready when the telemetry says it is in the mode of the job (a FULL mode needs the reload to have taken)
local function ready(job)
  if job.setup.vanilla then      -- (no telemetry while the Lua tyre is off: give the reload time and see the sled back)
    return S.t >= 5 and isSled(playerVehicle())
  end
  if not tel or clock - telAt > 0.6 then return false end
  if not tel.enabled then return false end
  local mode = job.family.mode
  local full = (mode == 2 or mode == 3)
  if full and tel.fullState ~= "on" then return false end
  if (not full) and tel.fullState == "on" then return false end
  local m = tostring(tel.model or "")
  if mode == 3 and not m:find("FULL SVJ", 1, true) then return false end
  if mode == 2 and not (m:find("FULL RBR", 1, true) and m:find(job.surface, 1, true)) then return false end
  if mode == 4 and not m:find("SVJ Pacejka on BeamNG", 1, true) then return false end
  if mode == 5 and not m:find("Advanced brush", 1, true) then return false end
  if mode == 1 and not m:find("RBR 1.02", 1, true) then return false end
  return true
end

local function finishAll(aborted)
  S.phase = "idle"
  S.finished = not aborted
  S.aborted = aborted or false
  writeManifest()
  local ok, bad = 0, 0
  for _, j in ipairs(S.jobs) do if j.status == "ok" then ok = ok + 1 elseif j.status ~= "waiting" then bad = bad + 1 end end
  local path = "modelTire/sweeps/" .. tostring(S.folder)
  local ok2, up = pcall(function() return FS:getUserPath() end)
  local wrote = writeReport()
  S.message = string.format("%s: %d runs ok, %d failed or skipped. Folder: %s%s. %s",
    aborted and "stopped" or "finished", ok, bad, ok2 and tostring(up) or "", path,
    wrote and "The report (report.html) and summary.csv are in it: press Open report." or "No report could be written.")
  log("I", "modelTire_batch", S.message)
end

local function nextJob()
  S.cur = S.cur + 1
  local j = S.jobs[S.cur]
  if not j then finishAll(false); return end
  j.status = "running"
  S.phase, S.t, S.stable = "setup", 0, 0
  S.message = string.format("run %d of %d: %s%s", S.cur, #S.jobs, j.family.name, j.surface and (", " .. j.surface) or "")
  log("I", "modelTire_batch", S.message)
end

local function failJob(j, reason)
  j.status = "failed"
  j.reason = reason
  log("W", "modelTire_batch", string.format("run %d (%s) failed: %s", j.idx, j.tag, tostring(reason)))
  writeManifest()
  nextJob()
end

-- control ---------------------------------------------------------------------------------------------------------------
function M.start()
  if S.phase ~= "idle" then return end
  local veh = playerVehicle()
  if not isSled(veh) then S.message = "the automatic test needs the ModelTire test sled (spawn it and sit in it)"; return end
  local jobs, skipped = buildJobs()
  if #jobs == 0 then S.message = "nothing to run" .. (skipped[1] and (": " .. skipped[1]) or ""); return end
  S.jobs, S.cur, S.skipped = jobs, 0, skipped
  local p, q = veh:getPosition(), quat(veh:getRotation())
  S.pose = {p = {x = p.x, y = p.y, z = p.z}, q = {x = q.x, y = q.y, z = q.z, w = q.w}}
  S.speedKmh, S.rampSec, S.maxSlip = speedKmh[0], rampSec[0], maxSlip[0]
  S.created = os.date("%Y-%m-%d %H:%M:%S")
  S.folder = "batch_" .. os.date("%Y%m%d_%H%M%S")
  S.finished, S.aborted = false, false
  S.startedClock = clock
  writeManifest()
  nextJob()
end

function M.abort()
  if S.phase == "idle" then return end
  if modelTire_sweep then modelTire_sweep.stopRun() end
  local veh = playerVehicle()
  if veh then veh:queueLuaCommand("if modelTire_rig then modelTire_rig.stop() end") end
  local j = S.jobs[S.cur]
  if j and j.status == "running" then j.status = "aborted" end
  finishAll(true)
end

-- Opening the report. The game's own calls take its virtual paths ("/modelTire/sweeps/<folder>/report.html", relative to the user
-- folder): Engine.Platform.openFile opens a file with the program Windows has for it (the browser, for html) and
-- Engine.Platform.exploreFolder shows it in the file explorer. The full Windows path is also shown and can be copied.
local function winPath(rel)
  local ok, up = pcall(function() return FS:getUserPath() end)
  up = ok and tostring(up) or ""
  if up ~= "" and not up:match("[\\/]$") then up = up .. "/" end
  return ((up .. rel:gsub("^/", "")):gsub("/", "\\"))
end

function M.openReport()
  if not S.reportPath then return false end
  local ok1 = pcall(function() Engine.Platform.openFile(S.reportPath) end)
  if not ok1 then ok1 = pcall(function() Engine.Platform.openFile(winPath(S.reportPath)) end) end
  S.openNote = ok1 and "asked Windows to open the report; if nothing opened, use 'Show in explorer' or copy the path"
    or "the game could not open it: use 'Show in explorer' or copy the path into a browser"
  return ok1
end

function M.showFolder()
  if not S.reportPath then return end
  local ok1 = pcall(function() Engine.Platform.exploreFolder(S.reportPath) end)
  if not ok1 then ok1 = pcall(function() Engine.Platform.exploreFolder((S.reportPath:gsub("/[^/]*$", "/"))) end) end
  S.openNote = ok1 and "asked Windows to show the folder" or "the game could not open the folder: copy the path"
end

function M.copyPath()
  if not S.reportPath then return end
  pcall(function() im.SetClipboardText(winPath(S.reportPath)) end)
  S.openNote = "path copied: paste it into a browser's address bar"
end

function M.toggle() winOpen[0] = not winOpen[0] end
function M.isOpen() return winOpen[0] end
function M.onTelemetry(data) tel = data; telAt = clock end

-- the state machine, every frame -----------------------------------------------------------------------------------
function M.update(dt)
  clock = clock + dt
  if S.phase == "idle" then return end
  S.t = S.t + dt
  local j = S.jobs[S.cur]
  local veh = playerVehicle()
  if not isSled(veh) then
    if S.t > 15 and S.phase ~= "setup" then M.abort(); S.message = "stopped: the sled is no longer the active vehicle" end
    -- (right after a reload the vehicle is briefly missing; give it time)
    if S.phase ~= "ready" and S.phase ~= "setup" then return end
  end

  if S.phase == "setup" then
    local reloaded = modelTire_ui.testSetup(j.setup)
    j.reloaded = reloaded
    tel = nil
    S.phase, S.t, S.stable = "ready", 0, 0
  elseif S.phase == "ready" then
    if ready(j) then S.stable = S.stable + dt else S.stable = 0 end
    if S.stable >= 1.5 then S.phase, S.t, S.stable = "teleport", 0, 0
    elseif S.t > 90 then failJob(j, "the tyre mode did not become active (telemetry: " .. tostring(tel and tel.model) .. ")") end
  elseif S.phase == "teleport" then
    -- the reload put the sled where the last run left it: back to the start pose
    local p, q = S.pose.p, S.pose.q
    veh:setPositionRotation(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    S.phase, S.t, S.stable = "settle", 0, 0
  elseif S.phase == "settle" then
    -- the sled rests on its springs again: still for two seconds
    if veh:getVelocity():length() < 0.15 then S.stable = S.stable + dt else S.stable = 0 end
    if S.stable >= 2.0 and S.t >= 3 then S.phase, S.t = "run", 0
    elseif S.t > 30 then failJob(j, "the sled did not come to rest after the reload") end
  elseif S.phase == "run" then
    local ok, why = modelTire_sweep.runRig{speedKmh = S.speedKmh, maxSlip = S.maxSlip, rampSec = S.rampSec, dir = S.folder, tag = j.tag}
    if not ok then failJob(j, tostring(why)); return end
    S.phase, S.t = "wait", 0
  elseif S.phase == "wait" then
    local limit = (S.speedKmh / 3.6) / 2 + 1.5 + S.rampSec + 40
    if S.t > 1 and not modelTire_sweep.isBusy() then
      local r = modelTire_sweep.lastResult() or {}
      j.samples, j.file, j.reason = r.samples, r.path, r.reason
      j.detected, j.model, j.staticLoad = r.surface, r.model, r.staticLoad
      j.kappaSd = r.kappaSd
      if r.curve then j.m, j.pts = analyse(r.curve) end
      if r.ok and (r.samples or 0) >= 20 then j.status = "ok" else j.status = "failed"; j.reason = j.reason or "no result" end
      writeManifest()
      nextJob()
    elseif S.t > limit then
      modelTire_sweep.stopRun()
      j.status = "failed"; j.reason = "timeout"
      writeManifest()
      nextJob()
    end
  end
end

-- window ----------------------------------------------------------------------------------------------------------------
local function checkList(title, list, names)
  if im.CollapsingHeader1(title) then
    if im.Button("all##" .. title) then for _, n in ipairs(names) do list[n][0] = true end end
    im.SameLine()
    if im.Button("none##" .. title) then for _, n in ipairs(names) do list[n][0] = false end end
    for _, n in ipairs(names) do im.Checkbox(n .. "##" .. title, list[n]) end
  end
end

function M.draw()
  if not winOpen[0] then return end
  im.SetNextWindowSize(im.ImVec2(520, 640), im.Cond_FirstUseEver)
  if im.Begin("ModelTire automatic test", winOpen) then
    local busy = S.phase ~= "idle"
    local inf = ensureSurfaceLists()
    im.TextWrapped("Runs the test sled's slip-angle ramp for every chosen tyre model and surface, one after the other, and saves the "
      .. "results in one folder, with a report you can open from here when it is done (also in the sweep window's plots). Spawn the ModelTire "
      .. "test sled on a large flat area first, switch the Lua tyre on in the control window and choose the SVJ tyre and the RBR tyre "
      .. "you want tested. Don't touch the controls while it runs.")
    if not isSled(playerVehicle()) then
      im.TextColored(im.ImVec4(1, 0.5, 0.3, 1), "The active vehicle is not the ModelTire test sled.")
    end
    if busy then im.BeginDisabled(true) end
    im.Separator()
    im.Text("Tyre models")
    for _, f in ipairs(FAMILIES) do im.Checkbox(f.name .. "##fam", famOn[f.key]) end
    if inf then
      checkList("surfaces for FULL RBR (" .. #inf.fullSurfaces .. ")", surfOn.full, inf.fullSurfaces)
      checkList("surfaces for FULL SVJ Pacejka (" .. #inf.svjSurfaces .. ")", surfOn.svj, inf.svjSurfaces)
      im.TextDisabled(string.format("SVJ tyre: %s   RBR tyre number: %s", tostring(inf.svjTyre or "none selected"), tostring(inf.rbrTyre)))
    end
    im.PushItemWidth(240)
    im.SliderFloat("speed (km/h)", speedKmh, 20, 120, "%.0f")
    im.SliderFloat("ramp duration (s)", rampSec, 6, 30, "%.0f")
    im.SliderFloat("slip angle at the end (deg)", maxSlip, 8, 40, "%.0f")
    im.PopItemWidth()
    if busy then im.EndDisabled() end

    local jobs, skipped = {}, {}
    if not busy then jobs, skipped = buildJobs() end
    if busy then
      if im.Button("Stop") then M.abort() end
    else
      if im.Button("Start automatic test") then M.start() end
      im.SameLine()
      im.Text(string.format("%d runs, about %.0f minutes", #jobs, #jobs * perJobSeconds() / 60))
    end
    for _, s in ipairs(skipped) do im.TextColored(im.ImVec4(1, 0.8, 0.2, 1), "skipped: " .. s) end
    im.Separator()
    im.TextWrapped(S.message)
    if S.phase ~= "idle" or #S.jobs > 0 then
      local ok, bad, wait = 0, 0, 0
      for _, j in ipairs(S.jobs) do
        if j.status == "ok" then ok = ok + 1 elseif j.status == "waiting" or j.status == "running" then wait = wait + 1 else bad = bad + 1 end
      end
      im.Text(string.format("done %d, failed %d, still to do %d   [%s]", ok, bad, wait, S.phase))
      for _, j in ipairs(S.jobs) do
        if j.status == "failed" or j.status == "aborted" then
          im.TextColored(im.ImVec4(1, 0.5, 0.3, 1), string.format("%s: %s", j.tag, tostring(j.reason)))
        end
      end
    end
    -- the results, in the game: one line per finished run
    local done = {}
    for _, j in ipairs(S.jobs) do if j.status == "ok" and j.m then done[#done + 1] = j end end
    if #done > 0 then
      if S.reportPath then
        if im.Button("Open report") then M.openReport() end
        im.SameLine()
        if im.Button("Show in explorer") then M.showFolder() end
        im.SameLine()
        if im.Button("Copy path") then M.copyPath() end
        im.TextWrapped(winPath(S.reportPath))
        if S.openNote then im.TextDisabled(S.openNote) end
      end
      im.Columns(6, "batchResults", true)
      for _, hd in ipairs({"tyre model", "surface", "peak g", "at deg", "g at end", "kept"}) do im.Text(hd); im.NextColumn() end
      im.Separator()
      for _, j in ipairs(done) do
        im.Text(j.family.key); im.NextColumn()
        im.Text(tostring(j.surface or ("ground: " .. tostring(j.detected or "?")))); im.NextColumn()
        im.Text(string.format("%.2f", j.m.peak)); im.NextColumn()
        im.Text(string.format("%.1f", j.m.slipPeak)); im.NextColumn()
        im.Text(string.format("%.2f", j.m.gEnd)); im.NextColumn()
        im.Text(j.m.keep and string.format("%.0f%%", j.m.keep * 100) or "-"); im.NextColumn()
      end
      im.Columns(1)
    end
    im.TextDisabled("Only the FULL modes can be given a surface; BeamNG's tyre and the other modes run on the ground under the sled. "
      .. "To compare them on another material, move the sled to another patch and run the test again: the report merges folders.")
  end
  im.End()
end

return M
