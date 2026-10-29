-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 RFloEng
-- modelTire: control window (game engine Lua, extension "modelTire_ui").
-- Switches the Lua tyre on for the player's vehicle, picks the tyre model, edits its settings and shows live data.
-- Licence: GPL-3.0-or-later.

local M = {}
M.dependencies = {"ui_imgui", "modelTire_svj", "modelTire_graphs", "modelTire_sweep", "modelTire_batch"}

local im = ui_imgui
local ffi = require("ffi")

local windowOpen = im.BoolPtr(true)
local wantEnabled = im.BoolPtr(false)
local telemetry = nil
local telemetryAge = 999
local resendTimer = 0
local showForces = im.BoolPtr(true)
local forceScale = im.FloatPtr(0.25)   -- metres of arrow per kN

-- Modes -------------------------------------------------------------------------------------------------------
-- 0 test tyre (brush)  1 RBR tyre on BeamNG's tyre  2 FULL RBR tyre  3 FULL SVJ Pacejka tyre  4 SVJ Pacejka on BeamNG's tyre
-- 5 advanced brush (the brush with load, pressure, temperature and wear)  6 FULL advanced brush (the same brush on the FULL tyre path)
local function isFull(m) return m == 2 or m == 3 or m == 6 end
local function isSvj(m) return m == 3 or m == 4 end
local modeOrder = {0, 5, 1, 4, 2, 3, 6}
local modePtr = im.IntPtr(0)
local function mode() return modePtr[0] end
local modeNames = {
  [0] = "Test tyre (simple brush model)",
  [1] = "RBR tyre on BeamNG's tyre",
  [2] = "FULL RBR tyre",
  [3] = "FULL SVJ Pacejka tyre",
  [4] = "SVJ Pacejka tyre on BeamNG's tyre",
  [5] = "Advanced brush (load, pressure, temperature, wear)",
  [6] = "FULL Advanced brush",
}
local modeHelp = {
  [0] = "A simple brush tyre that replaces BeamNG's grip. Only to test that a Lua tyre can drive the car.",
  [1] = "RBR's tyre computes the grip, but BeamNG's own soft tyre still touches the ground and carries the load. "
    .. "Both flex, so it feels stiffer. Needs no vehicle reload.",
  [2] = "BeamNG's tyre no longer touches the ground. RBR's tyre measures the wheel height over the ground and "
    .. "computes load, sinking into loose ground and grip itself. Needs 'Reload vehicle'.",
  [3] = "Like FULL RBR, but the grip comes from a Pacejka Magic Formula tyre read from an SVJ file "
    .. "(see the tyre list below). Needs 'Reload vehicle'.",
  [4] = "The Pacejka Magic Formula tyre of an SVJ file computes the grip, but BeamNG's own soft tyre still touches the "
    .. "ground and carries the load, as in mode 1. Needs no vehicle reload.",
  [5] = "The brush tyre of the test mode with the things a real tyre does on top: its stiffness per N falls as the load rises, "
    .. "it follows the tyre's pressure, grip depends on temperature (best at about 75 C) and the tyre heats up and wears from sliding. "
    .. "BeamNG's own tyre still carries the load, as in mode 1. Needs no vehicle reload.",
  [6] = "The Advanced brush (mode 5) on the FULL tyre path, like FULL RBR and FULL SVJ: BeamNG's tyre no longer touches the ground, the wheel "
    .. "height over the ground gives the load (a spring, 'fullBrushKz') and the brush gives the forces, with the enveloping contact on steps. "
    .. "Same brush numbers as mode 5 (see the test tyre settings). Needs 'Reload vehicle'.",
}

-- Settings ----------------------------------------------------------------------------------------------------
-- name, kind (f float, i int, b bool), min, max, default (must match tyre.lua), group, modes it applies to, help
local G_TUNE, G_STAB, G_ADV, G_TEST = "tune", "stab", "adv", "test"
local paramDefs = {
  -- the settings most worth touching
  {"rbrGripScale", "f", 0.3, 1.5, 0.95, G_TUNE, {1, 2},
    "Multiplies all RBR surface grip. RBR tarmac grips at about mu 1.4-1.5, more than most BeamNG tyres, so a car "
    .. "can roll over in corners. 1.0 = RBR's own numbers; 0.6-0.8 feels closer to BeamNG cars."},
  {"fzFilterMs", "f", 0, 60, 15, G_STAB, {0, 1, 4, 5},
    "Smoothing of the load BeamNG reports for the wheel, in milliseconds. BeamNG's contact load is noisy; a friction "
    .. "limit that follows it makes a parked car creep or slide a little. Higher = steadier at low speed, a bit slower "
    .. "to react to load changes (0 = off)."},
  {"fullSoilSoftness", "f", 0.1, 3.0, 0.6, G_TUNE, {2},
    "How soft loose ground is in the full RBR mode (gravel, dirt, mud, grass, snow). RBR's numbers are for a light rally "
    .. "car; a heavier BeamNG car sinks further. 1 = RBR's soil as it is; lower = firmer ground, the tyre sinks less; "
    .. "higher = softer, sinks more. Mud is RBR's soft wet dirt."},
  {"mzScale", "f", 0.0, 3.0, 1.0, G_TUNE, {1, 2, 3, 4, 6},
    "Strength of the self-aligning torque: what the steering wheel feels (force feedback). 1 = as computed. "
    .. "Raise for more feel, lower if the force feedback is too strong or shakes."},
  {"rbrTyre", "i", 1, 40, 1, G_TUNE, {1, 2},
    "Which RBR tyre. With your RBR data: 1 tarmac dry, 2 tarmac intermediate, 3 tarmac wet, 4 gravel dry, "
    .. "5 gravel intermediate, 6 gravel wet, 7 snow. Without it: 1 tarmac, 2 wet tarmac, 3 gravel, 4 snow."},
  {"rbrLatStiffScale", "f", 0.3, 2.0, 1.0, G_TUNE, {1, 2},
    "Sideways and twisting stiffness of RBR's carcass. Lower = softer. Useful in mode 'RBR tyre on BeamNG's tyre', "
    .. "where BeamNG's tyre flexes too and the car feels stiff."},
  {"rbrScaleToWheel", "b", nil, nil, true, G_TUNE, {1, 2},
    "Fit the RBR tyre's width, stiffness and load tables to each BeamNG wheel: a narrow tyre then carries less than "
    .. "a wide one. Off = the same RBR tyre on every car."},
  {"rbrUseRbrData", "b", nil, nil, true, G_TUNE, {1, 2},
    "Use your own RBR data (made with tools/rbr_import.py) instead of the mod's built-in numbers. Has no effect "
    .. "if the data is not installed."},
  -- damping and stability
  {"rbrLowSpeedZeta", "f", 0, 2, 0.25, G_STAB, {1, 2},
    "Damping of RBR's low-speed 'sticking' spring, as a fraction of critical. 0 = RBR's undamped spring (it rings in "
    .. "BeamNG). Around 0.25 stops the shaking; much higher turns velocity noise into force noise."},
  {"fullLowSpeedVertZeta", "f", 0, 2, 0.3, G_STAB, {2, 3, 6},
    "Extra vertical damping below about 4 m/s so a parked car does not shake (0 = off). Full modes only."},
  {"fullUnsprungMass", "f", 10, 100, 30, G_STAB, {2, 3, 6},
    "Mass in kg used to size that damping: about wheel + hub + brake of this car."},
  {"fullVertDamping", "f", 0, 3000, 600, G_STAB, {2, 3, 6},
    "Vertical damping of the tyre in N*s/m. RBR has none (the suspension damper does the work). Raise it if the "
    .. "wheel bounces after bumps."},
  {"fullEnvelope", "b", nil, nil, true, G_STAB, {2, 3, 6},
    "Enveloping contact for steps and kerbs. A tyre that senses the ground at one point meets a step as a sudden jump. "
    .. "With this on, the wheel also samples the ground ahead and behind the hub (like a rigid wheel of the tyre's radius): "
    .. "the contact moves to the edge of the step and the force tilts forward, so the car climbs it gradually."},
  {"fullEnvEvery", "i", 1, 8, 2, G_ADV, {2, 3, 6},
    "Update the enveloping ground samples every N physics steps (1 = every step: most accurate, slower; 7 raycasts per wheel)."},
  {"fullHeightTau", "f", 0, 10, 1.5, G_STAB, {2, 3, 6},
    "Milliseconds of smoothing of the hub height over the ground. Takes the edge off single-step jumps; too high makes "
    .. "the tyre spongy (0 = off)."},
  {"fullSlewKN", "f", 0, 30, 4, G_STAB, {2, 3, 6},
    "Most the tyre force may change per millisecond, in kN (a spike filter). Protects the wheel beams from sudden "
    .. "impacts. 0 = off; lower = smoother but slower to react."},
  {"fullMaxLoadG", "f", 2, 20, 6, G_STAB, {2, 3, 6},
    "Cap of the vertical load per wheel, in multiples of the car's static wheel load. A freshly loaded car sits with its rim on the ground, far lower than the tyre expects; without a cap the tyre would throw it into the air."},
  {"fullFadeIn", "f", 0, 2, 0.3, G_STAB, {2, 3, 6},
    "Seconds over which the tyre forces fade in after the tyre is switched on or the car is reset."},
  {"svjModded", "b", nil, nil, false, G_TUNE, {3, 4},
    "MODDED PACEJKA: adds a hydroplaning layer to the Pacejka tyre. Above a critical speed (about 95 km/h at 2.2 bar, "
    .. "higher with deep grooves, lower for wide tyres) a wet road lifts part of the contact on a wedge of water: grip "
    .. "and cornering stiffness collapse with speed. Needs water: wet ground (e.g. ASPHALT_WET) or the water depth "
    .. "setting below."},
  {"hydroWaterMm", "f", 0, 10, 0, G_TUNE, {3, 4},
    "Modded Pacejka: depth of the water film in mm. 0 = take it from the ground model (wet asphalt 0.6 mm, wet gravel "
    .. "1 mm, mud 1.5 mm, dry 0). Set it above 0 to force a depth on every ground (a flooded road is 2-5 mm)."},
  {"hydroTreadMm", "f", 1, 12, 7, G_TUNE, {3, 4},
    "Modded Pacejka: tread depth in mm. Deeper grooves drain more water and raise the hydroplaning speed."},
  {"hydroVoid", "f", 0, 0.6, 0.3, G_TUNE, {3, 4},
    "Modded Pacejka: fraction of the tread that is open groove (0 = racing slick, 0.3 = road tyre, 0.4+ = rain or "
    .. "off-road tread)."},
  {"hydroMuFloor", "f", 0.01, 0.5, 0.08, G_TUNE, {3, 4},
    "Modded Pacejka: friction scale left when the contact floats completely on water (the ground's own grip still "
    .. "applies on top)."},
  {"svjSurfaceShape", "b", nil, nil, true, G_TUNE, {3, 4},
    "The ground changes the SHAPE of the Pacejka curves, not only their height. On loose or slippery ground the peak "
    .. "is lower and is reached at a larger slip (real road tyres peak at about 18 % slip on dry asphalt and about 30 % "
    .. "on snow and ice), so the tyre feels vaguer and slides more progressively. Off = only the grip is scaled."},
  {"svjFz0Scale", "f", 0.3, 3.0, 1.0, G_TUNE, {3, 4},
    "Library tyres do not say which load their coefficients were made for, so the car's static load on each wheel is "
    .. "used. This scales that load: above 1 the tyre acts as if built for a heavier car (stiffer, more grip "
    .. "capacity); below 1 for a lighter one. Use it to fine-tune how much grip and stiffness a library tyre gives."},
  {"svjFz0", "f", 0, 12000, 0, G_ADV, {3, 4},
    "Force the nominal load (N) of the Pacejka coefficients. 0 = automatic (the file's own value, else the car's "
    .. "static wheel load). Use it if you know the load the tyre was made for."},
  {"svjRelaxScale", "f", 0.3, 5.0, 1.0, G_ADV, {3, 4},
    "Scales the relaxation lengths of the tyre (how far it rolls before the force has built up). Library tyres have "
    .. "short ones (about 0.08 m): higher = softer, slower response; lower = sharper."},
  {"svjZeta", "f", 0, 2, 0.25, G_STAB, {3, 4},
    "Same as the low-speed damping above, for the Pacejka contact spring: stops a parked tyre ringing."},
  {"svjProgressive", "b", nil, nil, true, G_STAB, {3, 6},
    "Pacejka has no bump stop. On = the vertical spring stiffens near big deflections (landings), so the rim never "
    .. "reaches the ground. Off = a linear spring straight from the SVJ file."},
  {"svjBumpDef", "f", 0.01, 0.1, 0.04, G_STAB, {3, 6},
    "Deflection in m at which the progressive stiffening equals the linear spring. Smaller = hard stop sooner."},
  -- advanced
  {"rbrQuirk102", "b", nil, nil, true, G_ADV, {1, 2},
    "RBR 1.02 applies the tyre-temperature grip twice to static friction and not to kinetic friction (a bug of the "
    .. "original). On = like RBR."},
  {"rbrLowSpeedDamping", "f", 0, 10000, 0, G_ADV, {1, 2},
    "Fixed damping in N*s/m of RBR's sticking spring, added to the damping ratio above. 0 = vanilla RBR."},
  {"svjRadiusFromWheel", "b", nil, nil, true, G_ADV, {3, 4},
    "Measure how much the Pacejka tyre is squashed from the car's own wheel radius, so the car keeps its ride "
    .. "height whichever SVJ tyre you pick. Off = from the SVJ tyre's own radius."},
  {"svjUseCarPressure", "b", nil, nil, false, G_ADV, {3, 4},
    "Use the car's tyre pressure in the Pacejka pressure terms. Off = the pressure written in the SVJ file."},
  {"svjUseTyreDamping", "b", nil, nil, true, G_ADV, {3},
    "Use the vertical damping of the SVJ tyre. Off = fullVertDamping."},
  {"fullGroundEvery", "i", 1, 8, 1, G_ADV, {2, 3, 6},
    "Read the ground height every N physics steps. 1 = every step (most accurate, slightly slower)."},
  {"fullIgnoreCheck", "b", nil, nil, false, G_ADV, {2, 3, 6},
    "Skip the check that the tyre ring has no ground collision. Only if the window says 'needs reload' even "
    .. "after reloading."},
  {"debugDraw", "b", nil, nil, true, G_ADV, {1, 2, 3, 4, 6},
    "Draw force lines at each wheel: green = gripping, red = sliding, blue = load."},
  -- test tyre and the patch used by modes 0 and 1
  {"fullBrushKz", "f", 80000, 400000, 200000, G_TEST, {6}, "FULL Advanced brush: vertical stiffness of the tyre in N/m (load = stiffness x deflection of the free radius over the hub height)."},
  {"mu", "f", 0.3, 2.0, 1.10, G_TEST, {0, 5, 6}, "Test tyre: peak friction coefficient."},
  {"muSlide", "f", 0.2, 2.0, 0.85, G_TEST, {0, 5, 6}, "Test tyre: friction coefficient when sliding fast."},
  {"stribeckVel", "f", 0.2, 10.0, 2.5, G_TEST, {0, 5, 6}, "Test tyre: slip speed (m/s) over which grip falls from peak to sliding."},
  {"muLongScale", "f", 0.7, 1.4, 1.05, G_TEST, {0, 5, 6}, "Test tyre: longitudinal grip relative to sideways grip."},
  {"loadSens", "f", 0.0, 0.5, 0.12, G_TEST, {0, 5, 6}, "Test tyre: how much grip drops as the load rises."},
  {"fzNominal", "f", 1000, 8000, 3500, G_TEST, {0, 5, 6}, "Test tyre: load in N where the grip numbers apply."},
  {"cornerCoef", "f", 4, 60, 28, G_TEST, {0, 5, 6}, "Test tyre: cornering stiffness per N of load. The sideways force peaks at about 3 x mu / this number radians of slip angle."},
  {"longCoef", "f", 4, 60, 30, G_TEST, {0, 5, 6}, "Test tyre: longitudinal slip stiffness per N of load. The force peaks at about 3 x mu / this number of slip ratio."},
  {"relaxLenY", "f", 0.05, 1.0, 0.35, G_TEST, {0, 5, 6}, "Test tyre: distance (m) the sideways force takes to build up."},
  {"relaxLenX", "f", 0.03, 0.6, 0.15, G_TEST, {0, 5, 6}, "Test tyre: distance (m) the longitudinal force takes to build up."},
  {"dampRatio", "f", 0.0, 2.0, 0.5, G_TEST, {0, 5, 6}, "Test tyre: damping of the carcass spring."},
  {"trail", "f", 0.0, 0.08, 0.035, G_TEST, {0, 5, 6}, "Test tyre: pneumatic trail in m (self-aligning torque)."},
  {"crr", "f", 0.0, 0.05, 0.012, G_TEST, {0, 1, 4, 5, 6}, "Rolling resistance coefficient."},
  {"brushRimTorque", "b", nil, nil, true, G_TEST, {0, 5, 6}, "Test tyre and advanced brush: apply the longitudinal force at the hub and its wheel-spin torque directly on the rim nodes (as the FULL modes do) instead of in the tread nodes. Off = the old way, which makes the wheel speed ring on a stiff test tyre."},
  {"nativeFriction", "f", 0.0, 1.0, 0.0, G_TEST, {0, 1, 4, 5}, "Fraction of BeamNG's own tyre grip that is kept (0 = off)."},
  {"patchBand", "f", 0.002, 0.05, 0.012, G_TEST, {0, 1, 4, 5}, "Height band (m) of the tyre nodes that count as the contact patch."},
  {"maxPatchNodes", "i", 1, 16, 6, G_TEST, {0, 1, 4, 5}, "Most nodes used in the contact patch."},
  {"patchEverySteps", "i", 1, 20, 4, G_TEST, {0, 1, 4, 5}, "Re-select the patch nodes every N physics steps."},
  {"slipFromTread", "b", nil, nil, false, G_TEST, {0, 5, 6}, "Test tyre: take slip from the tread nodes instead of hub and wheel speed."},
  {"fzFallback", "b", nil, nil, false, G_TEST, {0, 1, 4, 5}, "If BeamNG reports no wheel load, use the car's static weight share."},
  -- Advanced brush
  {"advLoadShape", "f", 0.5, 6.0, 2.0, G_TEST, {5, 6}, "Advanced brush: how the stiffness per N falls as the load rises. 0.5 = a lot, 6 = hardly (linear)."},
  {"advPressureNom", "f", 100, 350, 200, G_TEST, {5, 6}, "Advanced brush: tyre pressure in kPa (gauge) where the stiffness and grip numbers apply. The pressure is read from BeamNG's tyre."},
  {"advPressureK", "f", 0.0, 1.5, 0.6, G_TEST, {5, 6}, "Advanced brush: the stiffness follows (pressure / nominal) to this power."},
  {"advPressureMu", "f", 0.0, 2.0, 0.4, G_TEST, {5, 6}, "Advanced brush: grip lost for a pressure away from the nominal (squared)."},
  {"advTopt", "f", 40, 120, 75, G_TEST, {5, 6}, "Advanced brush: tyre temperature (deg C) with the best grip."},
  {"advTwidth", "f", 10, 80, 35, G_TEST, {5, 6}, "Advanced brush: how far from the best temperature (deg C) the grip falls off (gaussian width)."},
  {"advTfloor", "f", 0.2, 1.0, 0.7, G_TEST, {5, 6}, "Advanced brush: grip factor of a tyre far too cold or too hot."},
  {"advTamb", "f", -20, 50, 20, G_TEST, {5, 6}, "Advanced brush: air temperature (deg C): tyres cool towards it."},
  {"advTstart", "f", -20, 120, 75, G_TEST, {5, 6}, "Advanced brush: the temperature a tyre starts at, and returns to with the reset button (deg C)."},
  {"advHeatCap", "f", 1000, 30000, 6000, G_TEST, {5, 6}, "Advanced brush: heat capacity of the tyre that heats up (J/K): a bigger number heats and cools more slowly."},
  {"advHeatShare", "f", 0.02, 1.0, 0.15, G_TEST, {5, 6}, "Advanced brush: share of the sliding friction power that heats the tyre."},
  {"advRollHeat", "f", 0.0, 0.05, 0.012, G_TEST, {5, 6}, "Advanced brush: rolling heat in W per N of load per m/s of wheel speed."},
  {"advCoolBase", "f", 1, 40, 8, G_TEST, {5, 6}, "Advanced brush: cooling at rest in W/K."},
  {"advCoolSpeed", "f", 0, 5, 1.2, G_TEST, {5, 6}, "Advanced brush: extra cooling in W/K per m/s of speed."},
  {"advTimeScale", "f", 1, 100, 1, G_TEST, {5, 6}, "Advanced brush: heating, cooling and wear run this many times faster (to see them in a short test)."},
  {"advWearJ", "f", 2e5, 5e7, 1.2e7, G_TEST, {5, 6}, "Advanced brush: sliding friction energy (J) that wears the tread out."},
  {"advWearLoss", "f", 0.0, 0.6, 0.25, G_TEST, {5, 6}, "Advanced brush: grip lost with the tread worn out."},
}
local ptrs, defs = {}, {}
for _, d in ipairs(paramDefs) do
  defs[d[1]] = d
  if d[2] == "f" then ptrs[d[1]] = im.FloatPtr(d[5])
  elseif d[2] == "i" then ptrs[d[1]] = im.IntPtr(d[5])
  else ptrs[d[1]] = im.BoolPtr(d[5]) end
end

-- FULL mode state --------------------------------------------------------------------------------------------
-- vehicles (by object id) whose tyre ring gets no ground collision when their JBeam is (re)loaded
local fullIds = {}
local fullTyre = im.BoolPtr(false)            -- derived from the mode (2 and 3)
local fullRimCollision = im.BoolPtr(true)     -- the rim keeps colliding (safety stop) unless unticked
local autoSurface = im.BoolPtr(true)
local reloadHint = false
local fullSurfaces = {
  "Tarmac_Fine_Clean_Dry", "Tarmac_Fine_Clean_Damp", "Tarmac_Fine_Clean_Wet", "Tarmac_Coarse_Clean_Dry",
  "Gravel_Fine_Shallow_Dry", "Gravel_Medium_Shallow_Dry", "Gravel_Medium_Deep_Dry", "Gravel_Coarse_Deep_Dry",
  "Gravel_Coarse_Deeper_Dry", "Gravel_Medium_Deep_Wet", "Dirt_Hard_Dry", "Dirt_Soft_Dry", "Dirt_Soft_Wet",
  "Grass_Medium_Dry", "Grass_Soft_Wet", "SnowOnGravel_Shallow", "SnowOnGravel_Medium", "SnowOnGravel_Deep",
  "SnowOnIce_Shallow",
}
local fullSurfaceIdx = 1

-- SVJ / Pacejka tyre --------------------------------------------------------------------------------------------
local svjEntries, svjPath, svjProblems = {}, "", {}
local svjIdx = 0
local svjModelIdx = 1
local svjSurfaceIdx = 1
local svjGripScale = im.FloatPtr(1.25)
local svjSurfaces = {
  {"Dry asphalt", 1.0}, {"Damp asphalt", 0.85}, {"Wet asphalt", 0.70}, {"Packed gravel", 0.60},
  {"Loose gravel", 0.50}, {"Dirt", 0.55}, {"Snow", 0.30}, {"Ice", 0.12},
}
local svjMessage = ""
local svjFilterBuf = im.ArrayChar(64)
local svjOnlyFit = im.BoolPtr(true)
local svjHideSuspect = im.BoolPtr(true)
local svjSortMu = im.BoolPtr(false)
local svjFiltered = {}            -- indices into svjEntries after the filter
local svjFilterKey = ""
local svjMatches = 0
local svjSuspectCount = 0
local svjLoaded = false
local MAX_ROWS = 400

local function svjSelected() return svjEntries[svjIdx] end

local function svjRescan()
  local ok, entries, path, problems = pcall(function()
    local e, p, pr = modelTire_svj.scan()
    return e, p, pr
  end)
  if ok then
    local prevKey = svjSelected() and (svjSelected().file .. "|" .. svjSelected().key)
    svjEntries, svjPath, svjProblems = entries, path, problems
    svjIdx = (#svjEntries > 0) and 1 or 0
    for i, e in ipairs(svjEntries) do
      if prevKey == e.file .. "|" .. e.key then svjIdx = i end
    end
    svjMessage = ""
    svjLoaded = true
    svjFilterKey = ""
    svjSuspectCount = 0
    for _, e in ipairs(svjEntries) do if e.info and e.info.suspect then svjSuspectCount = svjSuspectCount + 1 end end
  else
    svjMessage = "scan failed: " .. tostring(entries)
  end
end

local function currentParams()
  local t = {}
  for _, d in ipairs(paramDefs) do t[d[1]] = ptrs[d[1]][0] end
  local m = mode()
  t.useRbr102 = m >= 1 and m ~= 5 and m ~= 6
  t.advBrush = (m == 5 or m == 6)
  t.fullTyre = isFull(m)
  t.fullModel = (m == 3) and "svj" or ((m == 6) and "brush" or "rbr")
  t.fullBrushGrip = svjSurfaces[svjSurfaceIdx][2]
  t.lightModel = (m == 4) and "svj" or "rbr"
  t.fullSurface = fullSurfaces[fullSurfaceIdx]
  t.fullAutoSurface = autoSurface[0]
  t.svjGrip = svjSurfaces[svjSurfaceIdx][2] * svjGripScale[0]
  t.svjGripScale = svjGripScale[0]
  return t
end

-- Called by BeamNG while a vehicle's JBeam is compiled, just before it goes to the physics engine.
-- Makes every tyre-ring node of a marked vehicle non-colliding: BeamNG's tyre then carries no load and RBR's
-- own contact model does (see tyre.lua, stepWheelFull). The rim (hub) nodes keep colliding as a safety stop.
local function onJbeamLoadingPhase2(loadingProgress, objID, vehicle)
  if not fullIds[objID] then return end
  local count, wheelsSeen = 0, 0
  for _, wheel in pairs(vehicle.wheels or {}) do
    if type(wheel) == "table" and wheel.treadNodes then
      wheelsSeen = wheelsSeen + 1
      for _, node in pairs(wheel.treadNodes) do
        if type(node) == "number" then node = vehicle.nodes and vehicle.nodes[node] end
        if type(node) == "table" then
          node.collision = false
          node.staticCollision = false
          count = count + 1
        end
      end
      if not fullRimCollision[0] and wheel.nodes then
        for _, node in pairs(wheel.nodes) do
          if type(node) == "number" then node = vehicle.nodes and vehicle.nodes[node] end
          if type(node) == "table" then
            node.collision = false
            node.staticCollision = false
            count = count + 1
          end
        end
      end
    end
  end
  vehicle.rbrFullTyre = true -- the vehicle Lua checks this flag
  log("I", "modelTire", string.format("full tyre: vehicle %s, %d wheels, %d ring nodes without collision",
    tostring(objID), wheelsSeen, count))
end

local function playerVehicle()
  return getPlayerVehicle and getPlayerVehicle(0) or be:getPlayerVehicle(0)
end

-- The player's own RBR tyres: modelTire/rbrtyres/*.json in the BeamNG user folder. A file holds one object or a list:
--   {"name": "my gravel", "base": "gravel_dry", "set": {"carcassStiffnessLateral": 300000, "sFric": [0.8, 0.8, 1, 0.2, 0.8, 0.7]}}
-- "base" is the name of the tyre it starts from (one in the list), "set" replaces any of that tyre's fields.
local rbrCustom, rbrCustomMessage = {}, ""
local function rbrCustomRescan()
  rbrCustom, rbrCustomMessage = {}, ""
  local rel = "modelTire/rbrtyres"
  if not (FS and FS.directoryExists) then return end
  if not FS:directoryExists(rel) then FS:directoryCreate(rel, true) end
  local files = FS:findFiles("/" .. rel, "*.json", -1, true, false) or {}
  if #files == 0 and jsonWriteFile then
    pcall(jsonWriteFile, "/" .. rel .. "/example_custom_tyre.json",
      {name = "example: softer gravel", base = "gravel_dry", set = {carcassStiffnessLateral = 300000}}, true)
    files = FS:findFiles("/" .. rel, "*.json", -1, true, false) or {}
  end
  for _, f in ipairs(files) do
    local ok, doc = pcall(jsonReadFile, f)
    if ok and type(doc) == "table" then
      local items = doc.name and {doc} or doc
      for _, c in ipairs(items) do
        if type(c) == "table" and type(c.name) == "string" and type(c.set) == "table" then
          rbrCustom[#rbrCustom + 1] = {name = c.name, base = c.base or "", set = c.set}
        end
      end
    end
  end
  rbrCustomMessage = string.format("%d custom tyres in %s", #rbrCustom, rel)
end

local function sendToVehicle(cmd)
  local veh = playerVehicle()
  if not veh then return end
  veh:queueLuaCommand("extensions.load('modelTire_tyre'); if modelTire_tyre then " .. cmd .. " end")
end

local function pushParams()
  sendToVehicle("modelTire_tyre.setParams(" .. serialize(currentParams()) .. ")")
end

-- Builds the selected SVJ tyre and sends it to the player's vehicle.
local function pushSvjTyre()
  local e = svjSelected()
  if not e then return end
  local T, err = modelTire_svj.buildTyre(e)
  if not T then
    svjMessage = tostring(err)
    return
  end
  svjMessage = "sent: " .. e.key
  if modelTire_graphs then modelTire_graphs.setTyre(modelTire_svj.buildTyre(e), e.key) end
  sendToVehicle("modelTire_tyre.setSvjTyre(" .. serialize(T) .. ")")
end

local rbrScanned = false
local function applyEnabled()
  if not rbrScanned then rbrScanned = true; rbrCustomRescan() end
  sendToVehicle("modelTire_tyre.setRbrCustom(" .. serialize(rbrCustom) .. ")")
  if isSvj(mode()) then pushSvjTyre() end
  pushParams()
  if wantEnabled[0] then
    sendToVehicle("modelTire_tyre.enable()")
  else
    sendToVehicle("modelTire_tyre.disable()")
  end
end

-- Ground material under each wheel ---------------------------------------------------------------------------
-- Called by the vehicle about 10 times a second with the world position of the ground under each wheel. On maps
-- with a terrain object the game can say which terrain material (and so which ground model) lies there; the answer
-- goes back to the vehicle. Maps with no terrain (the garage and showroom levels) have no terrain: the surface stays manual.
local groundNameCache = {}
local lastGround = {kind = "unknown", names = {}}
-- Mesh probe. On maps whose road is a mesh (not terrain) the game has no direct "ground model here" call. A ray down
-- from the wheel finds the scene object; its materials are looked up in the scene and the first one that carries a
-- groundType gives the ground model. What every step found is kept in probeLines (shown in the window) so a map that
-- does not work can be diagnosed from the screen.
local probeLines, probeSeen = {}, {}
local meshGroundCache = {}
local function probeNote(msg)
  if probeSeen[msg] then return end
  probeSeen[msg] = true
  probeLines[#probeLines + 1] = msg
  if #probeLines > 8 then table.remove(probeLines, 1) end
  log("I", "modelTire", "mesh probe: " .. msg)
end

local function materialGroundType(matName)
  local c = meshGroundCache[matName]
  if c ~= nil then return c or nil end
  local found = false
  local mat = scenetree and scenetree.findObject and scenetree.findObject(matName)
  if mat then
    local tries = {
      function() return mat:getField("groundType", 0) end,
      function() return mat.groundType end,
      function() return mat:getGroundType() end,
    }
    for k, f in ipairs(tries) do
      local ok, v = pcall(f)
      if ok and type(v) == "string" and v ~= "" then found = string.upper(v); break end
    end
    if not found then probeNote("material " .. matName .. " found, but no groundType could be read") end
  else
    probeNote("material " .. matName .. " not found in the scene")
  end
  meshGroundCache[matName] = found
  return found or nil
end

-- Decal roads have no collision geometry of their own (a ray only hits the terrain below), but each carries a
-- material, and that material may carry a ground type. The roads are listed once per level (middle line, half width,
-- material ground type) and a wheel point is tested against the segments.
local decalRoads, decalRoadCount, decalCheckAt = nil, -1, 0
local function buildDecalRoads()
  decalRoads = {}
  local names = scenetree and scenetree.findClassObjects and scenetree.findClassObjects("DecalRoad") or {}
  decalRoadCount = #names
  for _, name in ipairs(names) do
    local road = scenetree.findObject(name)
    if road then
      local okm, matName = pcall(function() return road:getField("material", 0) end)
      -- decals that are paint on an existing road (tyre marks, cracks, lines, patches) are not a surface of their own
      local lower = okm and type(matName) == "string" and string.lower(matName) or ""
      local overlay = lower:find("rubber", 1, true) or lower:find("skid", 1, true) or lower:find("mark", 1, true)
        or lower:find("crack", 1, true) or lower:find("line", 1, true) or lower:find("stripe", 1, true)
        or lower:find("paint", 1, true) or lower:find("patch", 1, true)
      local g = (okm and lower ~= "" and not overlay) and materialGroundType(matName) or nil
      if g then
        local ok, edges = pcall(function()
          local n = road:getEdgeCount()
          local pts = {}
          for i = 0, n - 1 do
            local m, l, r = road:getMiddleEdgePosition(i), road:getLeftEdgePosition(i), road:getRightEdgePosition(i)
            pts[#pts + 1] = {m.x, m.y, m.z, 0.5 * math.sqrt((l.x - r.x) ^ 2 + (l.y - r.y) ^ 2)}
          end
          return pts
        end)
        if ok and #edges > 1 then decalRoads[#decalRoads + 1] = {name = tostring(name), mat = matName, ground = g, pts = edges} end
      end
    end
  end
  probeNote(string.format("%d decal roads, %d with a readable ground type", decalRoadCount, #decalRoads))
end

local function groundFromDecalRoad(p)
  -- rebuilt when the level changes (the number of decal roads is checked every 5 s)
  local now = os.clock()
  if decalRoads and now - (decalCheckAt or 0) > 5 then
    decalCheckAt = now
    local n = scenetree and scenetree.findClassObjects and #scenetree.findClassObjects("DecalRoad") or 0
    if n ~= decalRoadCount then decalRoads = nil end
  end
  if not decalRoads then buildDecalRoads() end
  local best, bestD, bestRoad = nil, math.huge, nil
  for _, r in ipairs(decalRoads) do
    local pts = r.pts
    for i = 1, #pts - 1 do
      local a, b = pts[i], pts[i + 1]
      local dx, dy = b[1] - a[1], b[2] - a[2]
      local l2 = dx * dx + dy * dy
      local u = l2 > 1e-9 and ((p[1] - a[1]) * dx + (p[2] - a[2]) * dy) / l2 or 0
      u = u < 0 and 0 or (u > 1 and 1 or u)
      local cx, cy, cz = a[1] + dx * u, a[2] + dy * u, a[3] + (b[3] - a[3]) * u
      local d = math.sqrt((p[1] - cx) ^ 2 + (p[2] - cy) ^ 2)
      local hw = a[4] + (b[4] - a[4]) * u
      if d <= hw and math.abs(p[3] - cz) < 3 and d / math.max(hw, 0.1) < bestD then
        best, bestD, bestRoad = r.ground, d / math.max(hw, 0.1), r
      end
    end
  end
  if best then probeNote(string.format("on decal road %s (material %s): %s", bestRoad.name, bestRoad.mat, best)) end
  return best
end

-- returns the ground model name under the world point p ({x, y, z}) when it is not terrain, else nil
local function groundFromRay(p)
  if not (Engine and Engine.castRay) then probeNote("Engine.castRay is not available"); return nil end
  local a, b = vec3(p[1], p[2], p[3] + 0.6), vec3(p[1], p[2], p[3] - 1.2)
  local ok, hit = pcall(Engine.castRay, a, b, true, false)
  if not ok then probeNote("castRay failed: " .. tostring(hit)); return nil end
  -- a bridge or building may only have render geometry for the ray: try that when the collision pass found only terrain
  if not (hit and hit.object) or (pcall(function() return hit.object:getClassName() end) and hit.object:getClassName() == "TerrainBlock") then
    local ok2, hit2 = pcall(Engine.castRay, a, b, false, true)
    if ok2 and hit2 and hit2.object then hit = hit2; probeNote("collision ray found no mesh; render-geometry ray did") end
  end
  if not hit or not hit.object then probeNote("ray down found nothing (no collision geometry under the wheel)"); return nil end
  local o = hit.object
  local okc, cls = pcall(function() return o:getClassName() end)
  cls = okc and cls or "?"
  if cls == "TerrainBlock" then probeNote("ray down hit the terrain"); return nil end
  local okn, name = pcall(function() return o:getName() end)
  local okm, mats = pcall(function() return o:getMaterialNames() end)
  if not okm or type(mats) ~= "table" or #mats == 0 then
    probeNote(string.format("hit %s %s: no material names (%s)", cls, okn and tostring(name) or "?", okm and "empty" or tostring(mats)))
    return nil
  end
  local first, all = nil, {}
  for _, m in ipairs(mats) do
    local g = materialGroundType(m)
    if g then all[#all + 1] = m .. "=" .. g; first = first or g end
  end
  probeNote(string.format("hit %s %s: %d materials, groundTypes: %s", cls, okn and tostring(name) or "?", #mats,
    #all > 0 and table.concat(all, ", ") or "none"))
  return first
end

local function probeGround(vid, pts)
  local veh = be and be.getObjectByID and be:getObjectByID(vid)
  if not veh or type(pts) ~= "table" then return end
  local names, kind = {}, "none"
  local terrain = core_terrain and core_terrain.getTerrain and core_terrain.getTerrain()
  if terrain then
    kind = "terrain"
    for i, p in ipairs(pts) do
      local ok, idx = pcall(function() return terrain:getMaterialIdxWs(vec3(p[1], p[2], p[3])) end)
      if ok and idx ~= nil then
        local name = groundNameCache[idx]
        if name == nil then
          local okm, gm = pcall(function()
            local mtl = terrain:getMaterial(idx)
            local g = mtl and mtl.getGroundmodelName and mtl:getGroundmodelName() or nil
            if not g or g == "" then g = mtl and mtl.getInternalName and mtl:getInternalName() or nil end
            if not g or g == "" then g = terrain.getMaterialName and terrain:getMaterialName(idx) or nil end
            return g
          end)
          name = (okm and gm) and string.upper(tostring(gm)) or ""
          groundNameCache[idx] = name
        end
        names[i] = name
      end
    end
  end
  -- anything that is not terrain (mesh roads, bridges, objects): ask a ray, and let it override the terrain below it
  local meshHits = 0
  for i, p in ipairs(pts) do
    local g = groundFromRay(p) or groundFromDecalRoad(p)
    if g then names[i] = g; meshHits = meshHits + 1 end
  end
  if meshHits > 0 and kind == "none" then kind = "mesh" end
  lastGround = {kind = kind, names = names}
  veh:queueLuaCommand("if modelTire_tyre then modelTire_tyre.setGround(" ..
    serialize({kind = kind, names = names}) .. ") end")
end

-- Window ---------------------------------------------------------------------------------------------------------
local function fmt(x, f)
  if type(x) ~= "number" or x ~= x then return "-" end
  return string.format(f or "%.0f", x)
end

local function tip(text)
  if im.IsItemHovered() then im.SetTooltip(text) end
end

local function helpMark(text)
  im.SameLine()
  im.TextDisabled("(?)")
  if im.IsItemHovered() then
    im.BeginTooltip()
    im.PushTextWrapPos(im.GetFontSize() * 28)
    im.TextUnformatted(text)
    im.PopTextWrapPos()
    im.EndTooltip()
  end
end

local function appliesToMode(d, m)
  for _, x in ipairs(d[7]) do if x == m then return true end end
  return false
end

-- Draws the settings of one group; settings that do not apply to the chosen mode are greyed out.
local function drawGroup(group)
  local changed = false
  local m = mode()
  im.PushItemWidth(220)
  for _, d in ipairs(paramDefs) do
    if d[6] == group and d[1] ~= "rbrTyre" then -- the RBR tyre has its own picker
      local name, kind = d[1], d[2]
      local applies = appliesToMode(d, m)
      if not applies then im.BeginDisabled(true) end
      if kind == "f" then
        if im.SliderFloat(name, ptrs[name], d[3], d[4], "%.3f") then changed = true end
      elseif kind == "i" then
        if im.SliderInt(name, ptrs[name], d[3], d[4]) then changed = true end
      else
        if im.Checkbox(name, ptrs[name]) then changed = true end
      end
      helpMark(d[8] .. (applies and "" or "\n\n(Does not apply to the mode you chose.)"))
      if not applies then im.EndDisabled() end
    end
  end
  im.PopItemWidth()
  if changed then pushParams() end
end

-- RBR tyre picker: the tyres of the player's RBR data (or the built-in ones), then the custom files.
local function drawRbrTyrePicker(live)
  local m = mode()
  local usable = (m == 1 or m == 2)
  if not usable then im.BeginDisabled(true) end
  local list = telemetry and telemetry.rbrTyreList
  if list and #list > 0 then
    local i = math.max(1, math.min(ptrs.rbrTyre[0], #list))
    im.PushItemWidth(260)
    if im.BeginCombo("RBR tyre", tostring(list[i])) then
      for k, name in ipairs(list) do
        if im.Selectable1(name, k == i) then ptrs.rbrTyre[0] = k; pushParams() end
      end
      im.EndCombo()
    end
    im.PopItemWidth()
  else
    im.TextDisabled("RBR tyre: switch the Lua tyre on to see the list.")
  end
  helpMark("The RBR tyre used in modes 1 and 2: the tyres found in your RBR data (made with tools/rbr_import.py), then "
    .. "your own. Make your own by dropping a .json file in the BeamNG user folder, modelTire/rbrtyres/: it names an "
    .. "existing tyre as its base and lists the fields it changes (see the example file the mod writes there).")
  im.SameLine()
  if im.Button("Rescan tyre files") then rbrCustomRescan(); applyEnabled() end
  if rbrCustomMessage ~= "" then im.TextDisabled(rbrCustomMessage) end
  if not usable then im.EndDisabled() end
end

local function setMode(newMode)
  local oldFull = fullTyre[0]
  modePtr[0] = newMode
  local needFull = isFull(newMode)
  fullTyre[0] = needFull
  local veh = playerVehicle()
  if needFull ~= oldFull then
    if veh then fullIds[veh:getID()] = needFull or nil end
    reloadHint = true
  end
  if isSvj(newMode) then
    if not svjLoaded then svjRescan() end
    pushSvjTyre()
  end
  pushParams()
end

-- Used by the automatic test (batch.lua): sets the tyre mode and the ground surface without the window.
-- setup: {vanilla = true} (Lua tyre off) or {mode = 0..4, fullSurface = "Tarmac_Fine_Clean_Dry", svjSurface = "Dry asphalt"}, plus
-- forceReload = true to reload the vehicle whatever the mode.
-- Returns true when the vehicle had to be reloaded (a FULL mode was switched on or off); the reload is started here and the
-- tyre switches itself on again afterwards (onUpdate), so the caller waits for the telemetry to say so.
local function testSetup(s)
  local before = fullTyre[0]
  if s.vanilla then
    if fullTyre[0] then setMode(1) end     -- leaving a FULL mode: the flag must go so that the reload gives the tyre ring its collision back
    wantEnabled[0] = false
  else
    wantEnabled[0] = true
    autoSurface[0] = false                 -- the surface under test is the one named, not the one under the map
    for k, name in ipairs(fullSurfaces) do if name == s.fullSurface then fullSurfaceIdx = k end end
    for k, sfc in ipairs(svjSurfaces) do if sfc[1] == s.svjSurface then svjSurfaceIdx = k end end
    setMode(s.mode)
  end
  applyEnabled()
  local reload = reloadHint or (fullTyre[0] ~= before) or s.forceReload   -- the automatic test reloads before every run
  if reload then
    reloadHint = false
    if core_vehicle_manager then core_vehicle_manager.reloadVehicle(0) end
    telemetry = nil
    telemetryAge = 999
  end
  return reload
end

-- what the automatic test needs to know about the current setup
local function testInfo()
  local e = svjSelected()
  local names = {}
  for _, sfc in ipairs(svjSurfaces) do names[#names + 1] = sfc[1] end
  return {fullSurfaces = fullSurfaces, svjSurfaces = names, svjTyre = e and e.key or nil, svjFile = e and e.file or nil,
    rbrTyre = ptrs.rbrTyre and ptrs.rbrTyre[0] or nil, svjGripScale = svjGripScale[0], rbrGripScale = ptrs.rbrGripScale and ptrs.rbrGripScale[0] or nil}
end

local function drawLiveData(live)
  if not (live and telemetry.enabled and telemetry.wheels) then return end
  im.Columns(11, "rbrTyreWheels", true)
  for _, h in ipairs({"wheel", "Fz N (avg 0.1 s)", "Fz noise", "Fx N", "Fy N", "slip x", "slip y", "mu", "nodes", "tilt deg", "surface"}) do
    im.Text(h); im.NextColumn()
  end
  im.Separator()
  for _, w in ipairs(telemetry.wheels) do
    im.Text(tostring(w.name) .. (w.sliding and " *" or "")); im.NextColumn()
    im.Text(fmt(w.Fz)); im.NextColumn()
    im.Text(fmt(w.FzSpread)); im.NextColumn()
    im.Text(fmt(w.Fx)); im.NextColumn()
    im.Text(fmt(w.Fy)); im.NextColumn()
    im.Text(fmt(w.sx, "%.2f")); im.NextColumn()
    im.Text(fmt(w.sy, "%.2f")); im.NextColumn()
    im.Text(fmt(w.mu, "%.2f")); im.NextColumn()
    im.Text(fmt(w.nodes)); im.NextColumn()
    im.Text(fmt(w.env, "%.0f")); im.NextColumn()
    im.Text(tostring(w.surface or "")); im.NextColumn()
  end
  im.Columns(1)
  im.TextDisabled("* = sliding. Slip in m/s. Fz noise = spread of the load in 0.1 s (high = shaking). tilt deg = angle of the contact "
    .. "when the wheel meets a step or kerb with its front or back (0 on flat ground).")
  if telemetry.wheels[1] and telemetry.wheels[1].treadT then
    im.Columns(8, "rbrTyreThermal", true)
    for _, h in ipairs({"wheel", "surf C", "tread C", "carc C", "wear %", "sink mm", "slide w", "Mz Nm"}) do
      im.Text(h); im.NextColumn()
    end
    im.Separator()
    for _, w in ipairs(telemetry.wheels) do
      im.Text(tostring(w.name)); im.NextColumn()
      im.Text(fmt(w.surfT, "%.1f")); im.NextColumn()
      im.Text(fmt(w.treadT, "%.1f")); im.NextColumn()
      im.Text(fmt(w.carcassT, "%.1f")); im.NextColumn()
      im.Text(fmt((w.wear or 0) * 100, "%.2f")); im.NextColumn()
      im.Text(fmt((w.sink or 0) * 1000, "%.1f")); im.NextColumn()
      im.Text(fmt(w.slideW, "%.2f")); im.NextColumn()
      im.Text(fmt(w.Mz, "%.0f")); im.NextColumn()
    end
    im.Columns(1)
  end
end

local function drawSurfaceSection(live)
  local m = mode()
  if not isFull(m) then
    im.TextDisabled("In this mode BeamNG's tyre still touches the ground, so the surface comes from the game itself.")
    return
  end
  if im.Checkbox("read the surface from the map's terrain", autoSurface) then pushParams() end
  helpMark("On maps built on a terrain, the game can say which ground model (asphalt, gravel, dirt, grass, snow ...) "
    .. "lies under each wheel, and the tyre switches surface by itself. On the few maps without a terrain (garage, showrooms) there "
    .. "is no terrain to ask, so the surface you choose below is used. Roads that are separate meshes or decals over "
    .. "terrain can read as the terrain beneath them: check the 'surface' column and untick this if it is wrong.")
  local kind = live and telemetry.groundKind or nil
  if autoSurface[0] then
    if kind == "terrain" then
      im.TextColored(im.ImVec4(0.4, 1, 0.5, 1), "terrain found: surface is read from the ground")
    elseif kind == "mesh" then
      im.TextColored(im.ImVec4(0.4, 1, 0.5, 1), "mesh road found: surface is read from its material")
    elseif kind == "none" then
      im.TextColored(im.ImVec4(1, 0.8, 0.2, 1), "no terrain on this map: using the surface chosen below")
    else
      im.TextDisabled("waiting for the first ground reading ...")
    end
  end
  if autoSurface[0] and #probeLines > 0 then
    im.TextDisabled("ground probe (meshes, objects):")
    for _, l in ipairs(probeLines) do im.TextWrapped(l) end
  end
  local manualOnly = autoSurface[0] and (kind == "terrain" or kind == "mesh")
  if manualOnly then im.BeginDisabled(true) end
  im.PushItemWidth(260)
  if m == 2 then
    if im.BeginCombo("surface (manual)", fullSurfaces[fullSurfaceIdx]) then
      for k, name in ipairs(fullSurfaces) do
        if im.Selectable1(name, k == fullSurfaceIdx) then fullSurfaceIdx = k; pushParams() end
      end
      im.EndCombo()
    end
  else
    local cur = svjSurfaces[svjSurfaceIdx]
    if im.BeginCombo("surface (manual)", cur[1] .. string.format("  (x%.2f)", cur[2])) then
      for k, sfc in ipairs(svjSurfaces) do
        if im.Selectable1(sfc[1] .. string.format("  (x%.2f)", sfc[2]), k == svjSurfaceIdx) then
          svjSurfaceIdx = k
          pushParams()
        end
      end
      im.EndCombo()
    end
  end
  im.PopItemWidth()
  if manualOnly then im.EndDisabled() end
  if m == 3 then
    im.PushItemWidth(220)
    if im.SliderFloat("extra grip scale", svjGripScale, 0.3, 1.6, "%.2f") then pushParams() end
    im.PopItemWidth()
    helpMark("Multiplies the grip of the Pacejka tyre on top of the surface (detected or chosen). "
      .. "Lower it if the car rolls over in corners.")
  end
end

-- Rebuilds the visible list when the filter inputs change.
local function wheelInfoOf(live)
  local w = live and telemetry and telemetry.wheelInfo
  if type(w) == "table" and w.radius then return w end
  return nil
end

local function rebuildFilter(live)
  local wi = wheelInfoOf(live)
  local text = string.lower(ffi.string(svjFilterBuf))
  local key = table.concat({text, tostring(svjOnlyFit[0]), tostring(svjHideSuspect[0]), tostring(svjSortMu[0]),
    wi and string.format("%.3f|%.3f", wi.radius, wi.width or 0) or "-", tostring(#svjEntries)}, "|")
  if key == svjFilterKey then return end
  svjFilterKey = key
  local list = {}
  for idx, e in ipairs(svjEntries) do
    local info = e.info or {}
    local ok = true
    if svjHideSuspect[0] and info.suspect then ok = false end
    if ok and not (e.models[1] and e.models[1].ok) then ok = false end
    if ok and text ~= "" and not string.find(string.lower(e.label), text, 1, true) then ok = false end
    local score = 0
    if ok and svjOnlyFit[0] and wi then
      if info.diameter then
        local dd = math.abs(info.diameter - 2 * wi.radius) / (2 * wi.radius)
        local dw = (info.width and wi.width and wi.width > 0.05) and math.abs(info.width - wi.width) / wi.width or 0
        if dd > 0.04 or dw > 0.15 then ok = false else score = dd + dw end
      else
        ok = false
      end
    end
    if ok then list[#list + 1] = {idx = idx, score = score, mu = info.mu or 0} end
  end
  if svjOnlyFit[0] and wi then
    table.sort(list, function(a, b) if a.score ~= b.score then return a.score < b.score end return a.idx < b.idx end)
  elseif svjSortMu[0] then
    table.sort(list, function(a, b) if a.mu ~= b.mu then return a.mu > b.mu end return a.idx < b.idx end)
  end
  svjMatches = #list
  svjFiltered = {}
  for k = 1, math.min(#list, MAX_ROWS) do svjFiltered[k] = list[k].idx end
end

local function drawSvjSection(live)
  if not isSvj(mode()) then
    im.TextDisabled("Choose one of the SVJ Pacejka modes to pick a tyre from an SVJ file.")
    return
  end
  if im.Button(svjLoaded and "Rescan SVJ folder" or "Load SVJ tyres") then svjRescan(); pushSvjTyre() end
  im.SameLine()
  im.TextDisabled(svjLoaded and string.format("%d tyre sets (%d marked suspect)", #svjEntries, svjSuspectCount) or "not loaded yet")
  im.Text("Folder: " .. tostring(svjPath ~= "" and svjPath or "(press Load to create it)"))
  if svjLoaded and #svjEntries == 0 then
    im.TextWrapped("No tyre sets found. Put a tyre library (tires.svj.json) or any .svj.json file in that folder and press Rescan. "
      .. "Needs no typing of values: every Pacejka tyre in the files is listed below.")
  end
  for _, pr in ipairs(svjProblems) do im.TextColored(im.ImVec4(1, 0.5, 0.3, 1), "  skipped " .. pr) end
  if #svjEntries == 0 then return end

  -- filters
  local wi = wheelInfoOf(live)
  im.PushItemWidth(180)
  im.InputText("search (size, e.g. 205/55)", svjFilterBuf, 64)
  im.PopItemWidth()
  helpMark("Type part of a tyre size such as 205/55 or R16 to narrow the list.")
  if wi then
    im.Checkbox("only tyres that fit this car", svjOnlyFit)
    helpMark(string.format("Keeps tyres whose outer diameter is within 4%% of this car's wheels (radius %.3f m) and whose "
      .. "width is within 15%% of %.0f mm, closest first.", wi.radius, (wi.width or 0) * 1000))
  else
    svjOnlyFit[0] = false
    im.TextDisabled("(the 'fits this car' filter appears once the car reports its wheels)")
  end
  im.Checkbox("hide suspect fits", svjHideSuspect)
  helpMark("Hides tyres whose fitted coefficients look broken (no grip, or grip or stiffness at the fit's limits).")
  im.SameLine()
  im.Checkbox("sort by grip", svjSortMu)
  helpMark("Highest peak grip first (ignored while 'only tyres that fit this car' is on, which sorts by closeness).")
  rebuildFilter(live)
  im.Text(string.format("%d tyres match%s", svjMatches, svjMatches > MAX_ROWS and string.format(" (first %d shown: narrow the search)", MAX_ROWS) or ""))

  -- list
  if im.BeginChild1("svjlist", im.ImVec2(0, 190), true) then
    for _, idx in ipairs(svjFiltered) do
      local e = svjEntries[idx]
      local label = e.label .. (e.info and e.info.suspect and "  (!)" or "") .. "##" .. idx
      if im.Selectable1(label, idx == svjIdx) then
        svjIdx = idx
        svjModelIdx = 1
        pushSvjTyre()
      end
    end
  end
  im.EndChild()

  local cur = svjSelected()
  if cur then
    im.Text("Selected: " .. cur.label)
    for _, line in ipairs(modelTire_svj.describe(cur)) do im.TextDisabled("  " .. line) end
    if cur.info and cur.info.suspect then
      im.TextColored(im.ImVec4(1, 0.5, 0.3, 1), "  This tyre's fit looks suspect: expect odd behaviour.")
    end
    if cur.info and cur.info.estimated then
      im.TextDisabled("  Estimated data (no measurement behind it).")
    end
    local w1 = live and telemetry.wheels and telemetry.wheels[1]
    if w1 and w1.fz0 then im.TextDisabled(string.format("  nominal load used: %.0f N per wheel", w1.fz0)) end
    im.PushItemWidth(260)
    local mlabel = cur.models[svjModelIdx] and cur.models[svjModelIdx].label or "none"
    if im.BeginCombo("tyre model", mlabel) then
      for k, mm in ipairs(cur.models) do
        if im.Selectable1(mm.label, k == svjModelIdx) and mm.ok then svjModelIdx = k end
      end
      im.EndCombo()
    end
    im.PopItemWidth()
  end
  if svjMessage ~= "" then im.TextDisabled("  " .. svjMessage) end
end

local function drawWindow()
  im.SetNextWindowSize(im.ImVec2(620, 700), im.Cond_FirstUseEver)
  if not im.Begin("ModelTire", windowOpen) then
    im.End()
    return
  end

  -- master switch --------------------------------------------------------------------------------------------
  if im.Checkbox("Lua tyre on the player's vehicle", wantEnabled) then applyEnabled() end
  helpMark("Master switch. Off = BeamNG's own tyre (everything below is greyed out and does nothing). "
    .. "On = the tyre model you choose below replaces BeamNG's grip.")
  im.SameLine()
  if im.Button("Reset defaults") then
    for _, d in ipairs(paramDefs) do ptrs[d[1]][0] = d[5] end
    svjGripScale[0] = 1.25
    pushParams()
  end

  local live = telemetry and telemetryAge < 1
  if live then
    im.Text(string.format("%s   |   step cost %s us (all wheels)",
      telemetry.enabled and ("ON: " .. tostring(telemetry.model)) or "OFF: BeamNG's own tyre",
      fmt(telemetry.stepUs, "%.1f")))
  else
    im.Text("No data from the vehicle yet (spawn a car, then tick the box).")
  end

  -- outside the greyed area: the sweep tests also run with the Lua tyre off (the BeamNG vanilla reference run)
  if modelTire_sweep and im.Button("Sweep tests (in-game) ...") then modelTire_sweep.toggle() end
  if modelTire_batch then
    im.SameLine()
    if im.Button("Automatic test + report ...") then modelTire_batch.toggle() end
  end
  helpMark("Opens a window that drives the car by itself (steering, braking or throttle ramp) and plots the result, to compare "
    .. "tyre modes, BeamNG's own tyre and the theoretical Pacejka curve. Works with the Lua tyre off too: that run is the "
    .. "BeamNG vanilla reference.")

  if not wantEnabled[0] then im.BeginDisabled(true) end

  -- 1. mode ---------------------------------------------------------------------------------------------------
  if im.CollapsingHeader1("1. Tyre mode", im.TreeNodeFlags_DefaultOpen) then
    for _, m in ipairs(modeOrder) do
      if im.RadioButton2(modeNames[m], modePtr, m) then setMode(m) end
      tip(modeHelp[m])
    end
    im.TextWrapped(modeHelp[mode()])
    if mode() == 5 or mode() == 6 then
      if im.Button("Reset tyre temperature and wear") then sendToVehicle("modelTire_tyre.resetAdv()") end
      im.TextDisabled("The tyres start at the air temperature and new. Raise 'advTimeScale' to see them heat up and wear in a short test.")
    end
    if isFull(mode()) then
      if im.Checkbox("rim keeps ground collision (safety stop)", fullRimCollision) then reloadHint = true end
      helpMark("The wheel rim is much smaller than the tyre and never touches the ground in normal driving; with this "
        .. "on it stops the car sinking through the road if the tyre forces ever fail. Untick for a pure RBR/SVJ "
        .. "contact (then press Reload vehicle).")
    end
    if isFull(mode()) or fullTyre[0] then
      if im.Button("Reload vehicle") then
        if core_vehicle_manager then core_vehicle_manager.reloadVehicle(0) end
        reloadHint = false
        telemetry = nil
        telemetryAge = 999
        live = false -- the rest of this frame must not read the cleared telemetry
      end
      helpMark("BeamNG decides which tyre nodes collide when the vehicle is loaded, so switching a FULL mode on or off "
        .. "needs a reload. The tyre re-enables itself afterwards.")
      local state = (live and telemetry and telemetry.fullState) or "?"
      if reloadHint then
        im.TextColored(im.ImVec4(1, 0.8, 0.2, 1), "Press 'Reload vehicle' to apply the change.")
      elseif fullTyre[0] and state ~= "on" then
        im.TextColored(im.ImVec4(1, 0.5, 0.3, 1), "Full tyre state: " .. tostring(state) .. " (press 'Reload vehicle')")
      elseif fullTyre[0] then
        im.TextColored(im.ImVec4(0.4, 1, 0.5, 1), "Full tyre state: on")
      end
    elseif reloadHint then
      im.TextColored(im.ImVec4(1, 0.8, 0.2, 1), "Reload the vehicle to give BeamNG's tyre its ground contact back.")
      if im.Button("Reload vehicle") then
        if core_vehicle_manager then core_vehicle_manager.reloadVehicle(0) end
        reloadHint = false
        telemetry = nil
        telemetryAge = 999
        live = false
      end
    end
  end

  -- 2. surface -----------------------------------------------------------------------------------------------
  if im.CollapsingHeader1("2. Ground surface", im.TreeNodeFlags_DefaultOpen) then
    drawSurfaceSection(live)
  end

  -- 3. SVJ tyre ----------------------------------------------------------------------------------------------
  if isSvj(mode()) and im.CollapsingHeader1("3. SVJ tyre (Pacejka)", im.TreeNodeFlags_DefaultOpen) then
    drawSvjSection(live)
  end

  do
    if not modelTire_graphs and extensions and extensions.load then pcall(extensions.load, "modelTire_graphs") end
    local gm = modelTire_graphs
    local gp = gm and gm.showPtr() or im.BoolPtr(false)
    if not isSvj(mode()) or not gm then im.BeginDisabled(true) end
    if im.Checkbox("show force-versus-slip graphs (one floating window per wheel)", gp) then
      if gm then gm.setShown(gp[0]) end
    end
    helpMark("Opens a window for each wheel with the tyre's sideways force against slip angle and its longitudinal force "
      .. "against slip ratio, a family of curves for different loads (so you see how the weight changes the grip), and "
      .. "a dot where the wheel is now. Every window can be moved, resized and closed. Needs one of the two Pacejka modes.")
    if not isSvj(mode()) or not gm then im.EndDisabled() end
    if not gm then im.TextColored(im.ImVec4(1, 0.5, 0.3, 1), "graphs module not loaded: press Ctrl+L to reload the mod's Lua") end
  end

  -- live data --------------------------------------------------------------------------------------------------
  if im.CollapsingHeader1("Live data", im.TreeNodeFlags_DefaultOpen) then
    im.Checkbox("draw force arrows in the 3D view", showForces)
    helpMark("Arrows at each wheel's contact point: blue = the load (normal force), green = the grip force the tyre gives "
      .. "(red when the tyre is sliding). The length is the force; the friction arrow is the sum of the longitudinal "
      .. "and sideways forces. Updated 10 times a second; they stay attached to the car in between.")
    im.SameLine()
    im.PushItemWidth(120)
    im.SliderFloat("m per kN", forceScale, 0.05, 1.0, "%.2f")
    im.PopItemWidth()
    drawLiveData(live)
  end

  -- settings ---------------------------------------------------------------------------------------------------
  if im.CollapsingHeader1("Tuning: grip and steering feel", im.TreeNodeFlags_DefaultOpen) then
    im.TextWrapped("Hover the (?) next to a setting for what it does. Settings that do not apply to the chosen mode are greyed out.")
    drawRbrTyrePicker(live)
    drawGroup(G_TUNE)
  end
  if im.CollapsingHeader1("Damping and stability") then
    drawGroup(G_STAB)
  end
  if im.CollapsingHeader1("Advanced") then
    drawGroup(G_ADV)
  end
  if im.CollapsingHeader1("Test tyre and contact patch (modes 0 and 1)") then
    drawGroup(G_TEST)
  end

  if not wantEnabled[0] then im.EndDisabled() end
  im.End()
end

-- called by the vehicle (tyre.lua) about 10 times a second
local function onTelemetry(data)
  local veh = playerVehicle()
  if veh and data and data.vid and data.vid ~= veh:getId() then return end
  telemetry = data
  telemetryAge = 0
  if modelTire_sweep then modelTire_sweep.onTelemetry(data) end
  if modelTire_batch then modelTire_batch.onTelemetry(data) end
end

local function onUpdate(dtReal)
  telemetryAge = telemetryAge + dtReal
  -- a vehicle reload (part change, new car) starts with the Lua tyre off: turn it back on
  resendTimer = resendTimer + dtReal
  if resendTimer > 1 then
    resendTimer = 0
    if wantEnabled[0] and telemetry and telemetryAge < 1 and not telemetry.enabled then applyEnabled() end
    if wantEnabled[0] and telemetryAge > 2 then applyEnabled() end
  end
  if windowOpen[0] then drawWindow() end
  if modelTire_sweep then modelTire_sweep.update(dtReal); modelTire_sweep.draw() end
  if modelTire_batch then modelTire_batch.update(dtReal); modelTire_batch.draw() end
  if modelTire_graphs then
    modelTire_graphs.draw(telemetry, telemetry and telemetry.enabled and telemetryAge < 0.5 or false)
  end
end

-- Force arrows -----------------------------------------------------------------------------------------------------
-- Drawn by the game side every frame from the last telemetry (10 Hz), relative to the vehicle's position, so they follow
-- the car smoothly. Blue: load. Green: friction force (red when sliding).
local function onPreRender()
  if not showForces[0] or not wantEnabled[0] or not telemetry or telemetryAge > 0.5 or not telemetry.enabled then return end
  local veh = playerVehicle()
  if not veh or not telemetry.wheels or not debugDrawer then return end
  local base = veh:getPosition()
  local k = forceScale[0] / 1000
  for _, w in ipairs(telemetry.wheels) do
    local v = w.vec
    if v then
      local p = vec3(base.x + v[1], base.y + v[2], base.z + v[3])
      local t = vec3(v[4] * k, v[5] * k, v[6] * k)
      local n = vec3(v[7] * k, v[8] * k, v[9] * k)
      local gripCol = w.sliding and ColorF(1, 0.2, 0.15, 1) or ColorF(0.2, 1, 0.3, 1)
      debugDrawer:drawLine(p, p + n, ColorF(0.3, 0.55, 1, 1))
      debugDrawer:drawSphere(p + n, 0.025, ColorF(0.3, 0.55, 1, 1))
      debugDrawer:drawLine(p, p + t, gripCol)
      debugDrawer:drawSphere(p + t, 0.025, gripCol)
      debugDrawer:drawLine(p + n, p + n + t, ColorF(1, 1, 1, 0.6))   -- the resultant seen from the load arrow tip
    end
  end
end

local function toggleWindow()
  windowOpen[0] = not windowOpen[0]
end

-- the single-wheel rig has no way to stand without its controller: load it the moment the vehicle exists
local function loadRigController(vid)
  local veh = vid and be:getObjectByID(vid) or nil
  if not veh then return end
  local jb = veh.jbeam
  if jb == nil and veh.getJBeamFilename then jb = veh:getJBeamFilename() end
  if jb == "modelTire_wheelrig" then veh:queueLuaCommand("extensions.load('modelTire_wheelrig')") end
end

local function onVehicleSpawned(vid) loadRigController(vid) end
local function onVehicleReplaced(vid) loadRigController(vid) end

local function onVehicleSwitched()
  telemetry = nil
  telemetryAge = 999
  if wantEnabled[0] then applyEnabled() end
end

M.onJbeamLoadingPhase2 = onJbeamLoadingPhase2
M.onUpdate = onUpdate
M.onPreRender = onPreRender
M.onTelemetry = onTelemetry
M.probeGround = probeGround
M.toggleWindow = toggleWindow
M.testSetup = testSetup
M.testInfo = testInfo
M.onVehicleSwitched = onVehicleSwitched
M.onVehicleSpawned = onVehicleSpawned
M.onVehicleReplaced = onVehicleReplaced

return M
