-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- modelTire_svj: reads SVJ (Standard Vehicle JSON, https://github.com/RFloEng/SVJ-standard-vehicle-json) files
-- and turns the tyre sets in them into tyre tables for the Pacejka model (pacejka.lua). Licence: GPL-3.0-or-later.
--
-- Files go into  <BeamNG user folder>/modelTire/svj/  (any *.json: a whole-vehicle SVJ, or a tyre-only SVJ such
-- as examples/tire_mf62_245_40r18.svj.json from the SVJ repository). Every entry of  tires.sets  is offered.
--
-- What is read (SVJ spec section 11): dimensions, construction (vertical stiffness/damping, rolling radius),
-- reference (Fz0, pressure, speed), pacejka (MF52 / MF62 coefficient groups), relaxation, pressure.
-- tmeasy and brush parameters are listed but not simulated yet; external_models (.tir, FTire ...) are not read.

local M = {}
local abs = math.abs

M.folderRelative = "modelTire/svj/"

local GROUPS = {"lateral", "longitudinal", "aligning", "overturning", "rolling_resistance", "combined"}

local function isNum(x) return type(x) == "number" and x == x and x ~= math.huge and x ~= -math.huge end

local function firstNum(...)
  for k = 1, select("#", ...) do
    local v = select(k, ...)
    if isNum(v) then return v end
  end
  return nil
end

local function copyNumbers(src)
  if type(src) ~= "table" then return nil end
  local t = {}
  for k, v in pairs(src) do
    if isNum(v) then t[k] = v end
  end
  return t
end

-- Which models a tyre set offers.
local function modelsOf(set)
  local list = {}
  local pj = set.pacejka
  if type(pj) == "table" and type(pj.lateral) == "table" and type(pj.longitudinal) == "table" then
    list[#list + 1] = {id = "pacejka", label = "Pacejka " .. tostring(pj.model or "MF"), ok = true}
  end
  if type(set.tmeasy) == "table" then list[#list + 1] = {id = "tmeasy", label = "TMeasy (not simulated yet)", ok = false} end
  if type(set.brush) == "table" then list[#list + 1] = {id = "brush", label = "Brush (not simulated yet)", ok = false} end
  if type(set.external_models) == "table" and #set.external_models > 0 then
    list[#list + 1] = {id = "external", label = "External model files (not read)", ok = false}
  end
  return list
end

-- Numbers shown in the list so a tyre can be chosen without opening the file.
local function infoOf(set)
  local dim, size = set.dimensions or {}, set.size or {}
  local pj = type(set.pacejka) == "table" and set.pacejka or nil
  local lat = pj and pj.lateral or {}
  local info = {
    width = firstNum(dim.section_width, size.section_width_m),
    diameter = firstNum(dim.overall_diameter, size.overall_diameter_m),
    rimIn = firstNum(dim.rim_diameter_code, size.rim_diameter_in),
    estimated = (set._est == true) or set.source == "estimated",
  }
  if isNum(lat.pDy1) then info.mu = lat.pDy1 end
  if isNum(lat.pKy1) and isNum(lat.pKy2) then
    -- cornering stiffness per newton of nominal load (1/rad) at the nominal load
    info.cornering = abs(lat.pKy1) * math.sin((lat.pKy4 or 2) * math.atan(1 / math.max(lat.pKy2, 0.05)))
  end
  local m = pj and pj._metrics and pj._metrics.lateral
  if m and isNum(m.r2) then info.r2 = m.r2 end
  -- a fit that hit its limits or collapsed: no grip, grip at the 3.5 bound, or an impossible stiffness
  if info.mu and (info.mu < 0.5 or info.mu > 2.6) then info.suspect = true end
  if info.cornering and (info.cornering < 3 or info.cornering > 40) then info.suspect = true end
  if info.r2 and info.r2 < 0.8 then info.suspect = true end
  return info
end

-- A short description of a set for the list.
local function labelOf(file, key, set, info)
  local size = set.size_code or (set.size and set.size.size_code) or "?"
  local parts = {tostring(size)}
  if info.mu then parts[#parts + 1] = string.format("mu %.2f", info.mu) end
  if info.cornering then parts[#parts + 1] = string.format("C %.1f", info.cornering) end
  local short = tostring(key):match("_(%x%x%x%x%x%x)$")
  local models = {}
  for _, m in ipairs(modelsOf(set)) do models[#models + 1] = m.id == "pacejka" and "MF" or m.id end
  local head = table.concat(parts, "  ")
  if short then head = head .. "  #" .. short else head = tostring(key) .. "  " .. head end
  return string.format("%s  [%s]", head, #models > 0 and table.concat(models, ",") or "no model")
end

-- Entries of one decoded SVJ document.
function M.entriesOfDocument(doc, file)
  local out = {}
  local sets = type(doc) == "table" and type(doc.tires) == "table" and doc.tires.sets
  if type(sets) ~= "table" then return out end
  local keys = {}
  for k in pairs(sets) do keys[#keys + 1] = k end
  table.sort(keys)
  for _, key in ipairs(keys) do
    local set = sets[key]
    if type(set) == "table" then
      local info = infoOf(set)
      out[#out + 1] = {file = file, key = key, set = set, models = modelsOf(set), info = info,
        label = labelOf(file, key, set, info), origin = doc._metadata and doc._metadata.data_origin}
    end
  end
  return out
end

-- Scans the folder. Returns the entries and the folder path (for the window).
function M.scan()
  local entries, problems = {}, {}
  local rel = M.folderRelative
  if FS and FS.directoryExists and not FS:directoryExists(rel) then FS:directoryCreate(rel, true) end
  local path = (FS and FS.getUserPath and FS:getUserPath() or "") .. rel
  local files = FS and FS:findFiles("/" .. rel, "*.json", -1, true, false) or {}
  table.sort(files)
  for _, f in ipairs(files) do
    local doc = jsonReadFile(f)
    local base = f:match("([^/\\]+)$") or f
    if doc == nil then
      problems[#problems + 1] = base .. ": not valid JSON"
    else
      local e = M.entriesOfDocument(doc, base)
      if #e == 0 then problems[#problems + 1] = base .. ": no tires.sets" end
      for _, x in ipairs(e) do entries[#entries + 1] = x end
    end
  end
  return entries, path, problems
end

-- Builds the tyre table the vehicle side needs (numbers and strings only, so it can be serialised).
-- Returns nil and a message when the set has no usable model.
function M.buildTyre(entry)
  local set = entry.set
  local pj = set.pacejka
  if type(pj) ~= "table" or type(pj.lateral) ~= "table" or type(pj.longitudinal) ~= "table" then
    return nil, "the tyre set has no pacejka lateral/longitudinal coefficients"
  end
  local T = {
    name = entry.key, file = entry.file, modelName = tostring(pj.model or "MF62"),
    description = tostring(set.description or ""), size = tostring(set.size_code or ""),
  }
  for _, g in ipairs(GROUPS) do
    local name = (g == "rolling_resistance") and "rolling" or g
    local c = copyNumbers(pj[g])
    if c then T[name] = c end
  end
  local dim, con, ref, rel = set.dimensions or {}, set.construction or {}, set.reference or {}, set.relaxation or {}
  local size = set.size or {}
  -- nominal load of the coefficients: pacejka.reference_load_N, else reference.load. Fitted libraries (e.g. the
  -- anonymised tyre library) do not state it: then the car's static wheel load is used (set on the vehicle side).
  local fz0 = firstNum(pj.reference_load_N, ref.load)
  if fz0 and fz0 > 0 then T.Fz0 = fz0 else T.noFz0 = true end
  if isNum(ref.pressure) and ref.pressure > 0 then T.p0 = ref.pressure end
  if isNum(ref.speed) and ref.speed > 0 then T.V0 = ref.speed end
  if isNum(set.pressure) and set.pressure > 0 then T.pressure = set.pressure end
  -- unloaded radius: overall diameter, else rim + 2 x sidewall
  local od = firstNum(dim.overall_diameter, size.overall_diameter_m)
  if od and od > 0.2 then
    T.R0 = od * 0.5
  elseif isNum(dim.section_width) and isNum(dim.aspect_ratio) and isNum(dim.rim_diameter_code) then
    T.R0 = dim.rim_diameter_code * 0.0254 * 0.5 + dim.section_width * dim.aspect_ratio
  end
  if isNum(con.rolling_radius) and con.rolling_radius > 0.1 then T.Re = con.rolling_radius end
  local kz = firstNum(con.vertical_stiffness, set.vertical_stiffness_n_m)
  if kz and kz > 1000 then T.kz = kz end
  local cz = firstNum(con.vertical_damping, set.vertical_damping_n_s_m)
  if cz and cz >= 0 then T.cz = cz end
  -- relaxation length: lateral and longitudinal, or one value (relaxation_length_m) for both
  local rl = set.relaxation_length_m
  local sx, sy = firstNum(rel.longitudinal, rl), firstNum(rel.lateral, rl)
  if sx then T.sigmaX = sx end
  if sy then T.sigmaY = sy end
  local w = firstNum(dim.section_width, size.section_width_m)
  if w then T.width = w end
  -- slip angle of the lateral force peak (degrees in AC-derived files), used for the pneumatic trail when the file has
  -- no aligning-moment coefficients
  local fla = firstNum(set.friction_limit_angle)
  if fla and fla > 1 and fla < 30 then T.slipPeak = math.rad(fla) end
  T.estimated = (set._est == true) or set.source == "estimated"
  if pj._metrics and pj._metrics.lateral and isNum(pj._metrics.lateral.r2) then T.fitR2 = pj._metrics.lateral.r2 end
  if isNum(con.mass) then T.mass = con.mass end
  T.flipsSign = isNum(T.lateral and T.lateral.pKy1) and T.lateral.pKy1 > 0
  return T
end

-- A multi-line description for the window.
function M.describe(entry)
  local set = entry.set
  local dim, con, ref = set.dimensions or {}, set.construction or {}, set.reference or {}
  local size = set.size or {}
  local lines = {}
  lines[#lines + 1] = tostring(set.description or entry.key)
  local o = entry.origin
  if type(o) == "table" then
    lines[#lines + 1] = string.format("data: %s, confidence %s", tostring(o.type or "?"), tostring(o.confidence or "?"))
  elseif set.source then
    lines[#lines + 1] = "data: " .. tostring(set.source)
  end
  lines[#lines + 1] = string.format("size %s, width %s m, diameter %s m", tostring(set.size_code or size.size_code or "?"),
    tostring(dim.section_width or size.section_width_m or "?"), tostring(dim.overall_diameter or size.overall_diameter_m or "?"))
  local fz0 = firstNum(set.pacejka and set.pacejka.reference_load_N, ref.load)
  lines[#lines + 1] = string.format("reference: load %s, pressure %s Pa, speed %s m/s",
    fz0 and (string.format("%.0f N", fz0)) or "not stated (the car's static wheel load is used)",
    tostring(ref.pressure or set.pressure or "?"), tostring(ref.speed or "?"))
  lines[#lines + 1] = string.format("vertical stiffness %s N/m, relaxation length %s m",
    tostring(con.vertical_stiffness or set.vertical_stiffness_n_m or "?"),
    tostring((set.relaxation and set.relaxation.lateral) or set.relaxation_length_m or "?"))
  local i = entry.info or {}
  if i.mu then
    lines[#lines + 1] = string.format("peak grip mu %.2f, cornering coefficient %.1f /rad%s", i.mu, i.cornering or 0,
      i.r2 and string.format(", fit R2 %.3f", i.r2) or "")
  end
  return lines
end

return M
