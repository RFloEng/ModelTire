-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- modelTire_rbr102data: tyre types and surfaces for the RBR 1.02 tyre port.
-- Licence: GPL-3.0-or-later. All values are our own first guesses (not RBR's data). Units: SI (m, N, Pa, K, J).
--
-- Tyre type fields follow RBR's CTyre::CTyreTypes keys. Tables by load have 8 entries on loadAxis,
-- by temperature 8 on temperatureAxis, by air pressure 4 on pressureAxis, by contact pressure 6.
-- Surface classes (sFric, wetSlope, wearFtr): 1 tarmac, 2 gravel, 3 snow, 4 ice, 5 grass/mud, 6 sand.

local M = {}

local common = {
  -- geometry (the outer radius comes from the BeamNG tyre)
  treadDepth = 0.008, carcassWidth = 0.205, treadWidth = 0.19, carcassThickness = 0.012,
  sidewallHeight = 0.10, treadCoverage = 0.85,
  -- carcass
  carcassStiffnessLateral = 180000,   -- N/m
  carcassStiffnessTorsional = 30000,  -- N*m/rad (about 330 kN/m at the contact for R = 0.3 m)
  carcassStiffnessVertical = 150000,  -- N/m (soil solve only)
  pressureRatio = 1.0,
  treadElasticity = 5e6,              -- Pa
  referenceFriction = 1.0,
  -- tables
  pressureAxis = {120000, 180000, 240000, 300000},
  stfFct_P = {0.85, 0.97, 1.05, 1.12},
  contactPressureAxis = {0, 100000, 200000, 300000, 400000, 600000},
  contactConductance = {1.0, 1.0, 1.05, 1.1, 1.15, 1.2},
  temperatureAxis = {253, 273, 293, 313, 333, 353, 373, 393},
  rubMyFct_T = {0.80, 0.86, 0.92, 0.97, 1.00, 0.99, 0.95, 0.88},
  rubWear_T = {0.5, 0.6, 0.8, 1.0, 1.2, 1.6, 2.4, 3.5},
  loadAxis = {0, 1000, 2000, 3000, 4000, 5000, 6500, 8000},
  crnStf_L = {5000, 22000, 40000, 54000, 65000, 73000, 82000, 88000},     -- N per unit lateral slip
  trctStf_L = {6000, 25000, 46000, 64000, 80000, 92000, 105000, 115000},  -- N per unit slip ratio
  crnMyFct_L = {1.25, 1.20, 1.12, 1.06, 1.00, 0.96, 0.91, 0.87},
  trctMyFct_L = {1.30, 1.24, 1.16, 1.09, 1.03, 0.99, 0.94, 0.90},
  slpPkCrn_L = {0.08, 0.09, 0.10, 0.11, 0.12, 0.125, 0.13, 0.135},
  slpPkTrct_L = {0.07, 0.08, 0.09, 0.10, 0.11, 0.115, 0.12, 0.125},
  -- surface classes
  sFric = {1.0, 0.9, 0.85, 0.9, 0.9, 0.9},
  wetSlope = {-0.25, -0.1, 0, 0, -0.1, 0},
  wearFtr = {0.92, 0.9, 0.95, 0.95, 0.9, 0.9},
  wetnessDesign = 0.0, wetnessWearFtr = 0.9,
  -- heat
  surfaceCapacity = {0.3, 1500}, treadCapacity = {2.0, 1500}, carcassCapacity = {6.0, 1500}, -- {kg, J/(kg K)}
  treadConductivity = 15, carcassConductivity = 1.0,
  initTemperature = 303, heatAffectsPressure = true,
  wearScale = 1.0,
  quirk102 = true, -- 1.02 scales static friction by the temperature grip twice, kinetic not at all
}

local function variant(over)
  local t = {}
  for k, v in pairs(common) do t[k] = v end
  for k, v in pairs(over) do t[k] = v end
  return t
end

M.tyres = {
  tarmac = variant({}),
  tarmacWet = variant({treadCoverage = 0.72, treadDepth = 0.007, wetnessDesign = 0.6,
    sFric = {0.95, 0.85, 0.8, 0.9, 0.85, 0.85}, wetSlope = {-0.05, 0, 0, 0, 0, 0}}),
  gravel = variant({treadDepth = 0.013, treadCoverage = 0.62, carcassStiffnessLateral = 150000,
    carcassStiffnessTorsional = 26000,
    crnStf_L = {4000, 16000, 29000, 39000, 47000, 53000, 60000, 65000},
    trctStf_L = {5000, 19000, 34000, 47000, 58000, 66000, 75000, 82000},
    slpPkCrn_L = {0.14, 0.15, 0.17, 0.18, 0.19, 0.20, 0.21, 0.22},
    slpPkTrct_L = {0.12, 0.13, 0.15, 0.16, 0.17, 0.18, 0.19, 0.20},
    sFric = {0.88, 1.05, 0.9, 0.9, 1.0, 1.0}}),
  snow = variant({treadDepth = 0.012, treadCoverage = 0.55, carcassStiffnessLateral = 140000,
    slpPkCrn_L = {0.18, 0.19, 0.20, 0.21, 0.22, 0.23, 0.24, 0.25},
    slpPkTrct_L = {0.15, 0.16, 0.17, 0.18, 0.19, 0.20, 0.21, 0.22},
    sFric = {0.75, 0.95, 1.2, 1.5, 0.95, 0.95},
    rubMyFct_T = {0.95, 1.0, 0.98, 0.94, 0.90, 0.86, 0.82, 0.78}}),
}

-- Surfaces by BeamNG ground model name. static/kinetic: hard ground; staticSoil/kineticSoil: on top of the
-- soil layer. sinkFactor: soil softness (tread area / sinkFactor = soil stiffness), soilThickness m,
-- soilDensity kg/m^3, water 0..1, rollingResistance (torque = patchLength/2 * rr * load).
local hard = {class = 1, static = 1.0, kinetic = 0.85, staticSoil = 1.0, kineticSoil = 0.85,
  sinkFactor = 0, soilThickness = 0, soilDensity = 0, water = 0, rollingResistance = 0.05}
local function surf(over)
  local t = {}
  for k, v in pairs(hard) do t[k] = v end
  for k, v in pairs(over) do t[k] = v end
  return t
end

M.surfaces = {
  ASPHALT = surf({}),
  ASPHALT_PREPPED = surf({static = 1.05, kinetic = 0.9}),
  ASPHALT_OLD = surf({static = 0.95, kinetic = 0.8}),
  ASPHALT_WET = surf({static = 0.78, kinetic = 0.62, water = 0.8}),
  CONCRETE = surf({static = 0.98, kinetic = 0.84}),
  COBBLESTONE = surf({static = 0.85, kinetic = 0.72}),
  RUMBLE_STRIP = surf({static = 0.95, kinetic = 0.8}),
  ROCK = surf({static = 0.85, kinetic = 0.75, rollingResistance = 0.08}),
  METAL = surf({static = 0.6, kinetic = 0.5}),
  WOOD = surf({static = 0.7, kinetic = 0.6}),
  PLASTIC = surf({static = 0.6, kinetic = 0.5}),
  KICKPLATE = surf({static = 0.6, kinetic = 0.5}),
  GRAVEL = surf({class = 2, static = 0.8, kinetic = 0.7, staticSoil = 0.62, kineticSoil = 0.58,
    sinkFactor = 2e-8, soilThickness = 0.03, soilDensity = 1600, rollingResistance = 0.15}),
  DIRT = surf({class = 2, static = 0.82, kinetic = 0.72, staticSoil = 0.66, kineticSoil = 0.6,
    sinkFactor = 1.5e-8, soilThickness = 0.02, soilDensity = 1500, rollingResistance = 0.12}),
  DIRT_DUSTY = surf({class = 2, static = 0.8, kinetic = 0.7, staticSoil = 0.6, kineticSoil = 0.55,
    sinkFactor = 2e-8, soilThickness = 0.025, soilDensity = 1400, rollingResistance = 0.14}),
  GRASS = surf({class = 5, static = 0.7, kinetic = 0.6, staticSoil = 0.5, kineticSoil = 0.45,
    sinkFactor = 3e-8, soilThickness = 0.02, soilDensity = 1200, water = 0.2, rollingResistance = 0.2}),
  MUD = surf({class = 5, static = 0.55, kinetic = 0.45, staticSoil = 0.35, kineticSoil = 0.3,
    sinkFactor = 8e-8, soilThickness = 0.07, soilDensity = 1700, water = 0.7, rollingResistance = 0.35}),
  SAND = surf({class = 6, static = 0.7, kinetic = 0.62, staticSoil = 0.5, kineticSoil = 0.45,
    sinkFactor = 7e-8, soilThickness = 0.08, soilDensity = 1700, rollingResistance = 0.4}),
  SNOW = surf({class = 3, static = 0.5, kinetic = 0.4, staticSoil = 0.35, kineticSoil = 0.3,
    sinkFactor = 6e-8, soilThickness = 0.05, soilDensity = 400, water = 0.1, rollingResistance = 0.2}),
  ICE = surf({class = 4, static = 0.15, kinetic = 0.1, water = 0.2, rollingResistance = 0.03}),
}
M.defaultSurface = M.surfaces.ASPHALT

-- Optional: the player's own RBR data, made by tools/rbr_import.py into a separate unpacked mod
-- (modelTire_rbrdata). Not shipped with this mod.
M.rbr = nil
local ok, user = pcall(require, "modelTire_rbrdata")
if ok and type(user) == "table" and type(user.tyres) == "table" and type(user.surfaces) == "table" then
  local engine = user.engine or {}
  local set = {tyres = {}, names = {}, surfaces = user.surfaces, surfaceMap = user.surfaceMap or {},
    source = user.source}
  for i, t in ipairs(user.tyres) do
    local T = {}
    for k, val in pairs(engine) do T[k] = val end
    for k, val in pairs(t) do T[k] = val end
    T.heatAffectsPressure = true
    T.quirk102 = true
    T.wearScale = 1.0
    set.tyres[i] = T
    set.names[i] = t.name
  end
  set.defaultSurface = user.surfaces.Tarmac_Fine_Clean_Dry
  if set.defaultSurface and #set.tyres > 0 then M.rbr = set end
end

return M
