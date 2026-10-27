# Notices

ModelTire: Copyright (C) 2026 RFloEng. Licensed under the GNU General Public License, version 3 or (at your option) any
later version (see `LICENSE`). Repository: <https://github.com/RFloEng/ModelTire>.

# Third-party notices

## rerbr (GPL-3.0)
`mod/modelTire/lua/vehicle/extensions/modelTire/rbr102.lua` is a Lua port of the Richard Burns Rally tyre model as
decoded and published by the **rerbr** project, <https://github.com/JodusNodus/rerbr> (`crates/rbr-physics/src/tyre/`),
which is licensed GPL-3.0 ("Copyright (C) the ReRBR authors", repository owner JodusNodus). The port is a derived work and keeps that
licence; this repository is therefore GPL as a whole. The port keeps the 1.02 behaviour and leaves
out the NGP changes; the list of what is 1.02 and what is NGP is in `docs/RBR_vs_NGP_tyre.md`.
`tools/rbr_import.py` also uses constants of the RBR 1.02 engine that rerbr documents.

## SVJ (Apache-2.0)
The Pacejka tyre reads tyre files in the **Standard Vehicle JSON** format, <https://github.com/RFloEng/SVJ-standard-vehicle-json>
(Apache-2.0, repository owner RFloEng, the same GitHub account as this repository). The reader (`svj.lua`) was written from the format specification; no code of that project is included.
No SVJ tyre file is included in this repository (see `docs/DATA.md`).

## Pacejka Magic Formula
`pacejka.lua` implements the published Magic Formula 6.1/6.2 equations (Besselink, Schmeitz, Pacejka, "An improved
Magic Formula/Swift tyre model that can handle inflation pressure changes", Vehicle System Dynamics 48, 2010; Pacejka,
"Tyre and Vehicle Dynamics"). Equations are not copyrightable; the code is original.

## Trademarks
Richard Burns Rally, BeamNG.drive and BeamNG are trademarks of their owners. This project is not affiliated with or
endorsed by them. It contains no game assets, no RBR files and no RBR data.
