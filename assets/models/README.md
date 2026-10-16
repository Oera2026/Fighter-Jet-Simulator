# Airframe models

| File | Model | Drawn as | Source |
|---|---|---|---|
| `f16.glb` | F-16 Fighting Falcon - Fighter Jet - Free | F-16C (outside and cockpit) | https://sketchfab.com/3d-models/f-16-fighting-falcon-fighter-jet-free-f0b00989e5634764848ef2c235c64db5 |
| `f15e.glb` | F-15E Strike Eagle - Fighter Jet - Free | F-15EX | https://sketchfab.com/3d-models/f-15e-strike-eagle-fighter-jet-free-fff7d75490474e9b964d90cc031c8d01 |
| `typhoon.glb` | Eurofighter Typhoon - Fighter Jet - Free | Typhoon | https://sketchfab.com/3d-models/eurofighter-typhoon-fighter-jet-free-992bcc8987964ca09d55410330aa8579 |
| `f22.glb` | F-22 Raptor - Fighter Jet - Free | F-22A | https://sketchfab.com/3d-models/f-22-raptor-fighter-jet-free-508de5c48845456bb033fb267ebe1d1e |
| `f35.glb` | F-35 Lightning II - Fighter Jet - Free | F-35B | https://sketchfab.com/3d-models/f-35-lightning-ii-fighter-jet-free-b1ab1c0090e34b0fbfe667e706023e6d |

All five are by **bohmerang** (https://sketchfab.com/bohmerang) and licensed
[CC BY-NC-SA 4.0](http://creativecommons.org/licenses/by-nc-sa/4.0/):
attribution, non-commercial use, derivatives shared alike. The game's
CREDITS screen carries the attribution.

The A-10C has no model here and is drawn from its procedural airframe
(`include/fastjet/graphics/jet_airframes.hpp`).

## How they are used

- Placement, nozzle exits and the F-35B fit are in
  `include/fastjet/graphics/jet_assets.hpp`; `tests/test_model_glb.cpp`
  checks them against the files.
- The series shares one layout: X forward, Y up, Z along the span, 0.1 m
  units, nodes named airframe / canopy / cockpit / hud / landingOn /
  landingOff / landingOnLight (case varies).
- Each asset sits with its main wheels on the flight model's main gear
  station and its lowest tyre on the gear's contact plane.
- Only the F-16C is flown from its asset's cockpit. The other jets keep
  their procedural cockpits and, from the seat, their procedural hulls.
- The F-35 model has no moving parts. The F-35B's lift-system doors,
  weapons-bay doors and swivelling 3BSM are the procedural ones, shifted
  onto the model and drawn only while they move. Its wings are the
  F-35C's size (13.1 m span; the F-35B's is 10.7 m).
