# Data you must provide yourself

This repository ships **no tyre data from Richard Burns Rally and none from any tyre manufacturer or SVJ library.** The
mod contains the models (code); the numbers they run on come from files you supply. Do not commit those files to a fork
or send them to others: the `.gitignore` already excludes them.

## 1. RBR tyres (modes 1 and 2)

Without RBR data the mod runs on its own placeholder tyres (`rbr102data.lua`: four tyres with rough first-guess
numbers, not RBR's). To use the real RBR 1.02 numbers you need your **own legal copy of Richard Burns Rally** and the
importer:

```
python tools/rbr_import.py --rbr "D:\RSF"
python tools/rbr_import.py --physics-rbz "C:\path\to\physics.rbz"
```

It reads the 7 tyre types (`Physics/michelin/tyres.lsp`) and the surface table (`Physics/physics.lsp`) from your install
and writes a private extra mod, `modelTire_rbrdata`, into your BeamNG user folder (`mods/unpacked`). The main mod
finds it by itself (setting `rbrUseRbrData`). Keep that folder private; the tool writes a note saying so into it.
Vanilla RBR 1.02 data is expected; NGP/RSF installs are handled (the tool pulls the retail `physics.rbz`).

Your own RBR tyres: put `.json` files in the BeamNG user folder `modelTire/rbrtyres/`; each names a base tyre and the
fields it changes (see `mod/modelTire/README.md`, "Choosing the RBR tyre").

## 2. Pacejka tyres (modes 3 and 4)

The Pacejka tyre needs an **SVJ tyre file** (Standard Vehicle JSON, `tires` section with MF 5.2 / 6.1 / 6.2 coefficient
sets): <https://github.com/RFloEng/SVJ-standard-vehicle-json>. Put one or several `*.svj.json` files into the BeamNG user
folder `modelTire/svj/`; the mod lists every tyre set in them and you pick one in the window. Where to get tyre
data: the SVJ project's examples, tyre files you fitted or measured yourself, `.tir` files you are licensed to use
(convert them to SVJ), or a library you built. Check the licence of every file you use before sharing it.

Missing nominal load, aligning or rolling coefficients are replaced by documented fallbacks (see the mod manual).

## What is and is not in the repository

| Item | In the repository |
| --- | --- |
| Models (RBR 1.02 port, Pacejka, brush) | yes, GPL-3.0 |
| Placeholder tyres and surfaces (own guesses) | yes |
| RBR tyres, surfaces, physics files | **no**, generated locally by `rbr_import.py` |
| SVJ tyre files / libraries | **no**, you provide them |
