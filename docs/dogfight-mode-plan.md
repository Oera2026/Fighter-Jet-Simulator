# Dogfight Mode — Implementation Plan

A realism-first air combat mode: within-visual-range (WVR) basic fighter
manoeuvres (BFM) against AI adversaries that fly the **same** 6-DoF physics,
control laws and pilot-physiology model as the player. Winning should come from
energy management, turn performance and weapons employment inside real
envelopes, not from game rules.

---

## 0. Status and revisions (2026-09-24)

### What is built

The guns-only MVP slice now runs from Phase 0 to Phase 4, plus a first cut of Phase 7. It is playable from the
main menu (**DOGFIGHT**) or with `--dogfight [merge|offensive|defensive|range] --skill
[novice|veteran|ace] --bandit TYPE`. `--watch` lets the AI fly your jet too. In flight:
**F** fires the gun, **F9** starts a new fight, **F10** cycles the set-up, **F11** cycles the
bandit's skill and **R** restarts the fight.

| Piece | File | Tests |
|---|---|---|
| Aircraft entity (Phase 0) | `include/fastjet/sim/aircraft.hpp` | `test_aircraft_entity`: bit-identical to the old loop after 60 s (air work on three airframes, runway takeoff) |
| World: jets, 4,096-round pool, events | `sim/world.hpp` | swept hit tests, kill credit, mid-airs |
| Guns and ballistics (Phase 2) | `sim/gun.hpp`, `sim/combat_specs.hpp` | `test_guns`: analytic parabola and drag decay, rate and spin-up, tunnelling, predictor = live round (0 m error), hits at 600 m |
| Component damage (Phase 3) | `sim/damage_model.hpp` | engine and thrust, twin-engine survival, wing roll-off, A-10 toughness |
| Geometry and energy readouts (Phase 1) | `sim/air_combat_geometry.hpp` | `test_air_combat_geometry`: range, Vc, ATA, AA, HCA, turn rate, Ps against analytic values |
| BFM AI (Phase 4) | `sim/bfm_pilot.hpp` | `test_bfm_ai`: gunnery kills in all 5 airframes, no terrain impacts or mid-airs, skill ordering, bit-exact replay |
| Engagement and rules (Phase 1/7) | `sim/engagement.hpp` | outcome, stats, range targets |
| Viewer | `src/f16_sim_viewer.cpp`, `graphics/tracer_renderer.hpp`, HUD combat block | bandit drawn in both views; tracers (1 in 5); TD box or locator; director pipper with range ring; R, Vc, AA, TR, Ps, ammo; damage cautions; outcome and debrief banner |

The ownship is now `world.aircraft[0]` in every mode. Free flight is a world with one jet in it.

### Changes to the original plan

1. **Vertical slice first.** Phases 0–4 were built as thin layers across the whole stack, so a
   playable 1v1 existed early. Each layer then got its own tests, instead of four sequential
   phases with nothing to fly until Phase 4.
2. **One control layer for every manoeuvre.** Each manoeuvre (pure, lead or lag pursuit,
   break, jink, merge, extension, hard deck) is expressed as *aim point + G ceiling + throttle*.
   A single lift-vector controller flies all of them: it banks the lift vector onto the
   required specific force (gravity included) and closes on measured Nz. Line-of-sight-rate
   feed-forward removes the standing tracking error against a turning target. For guns
   tracks the error is taken off the gun line, not the velocity vector. This one controller
   works on the FBW jets and on the A-10's direct linkage, which needs added pitch-rate
   damping.
3. **Director gunsight shares the live ballistics.** The pipper and the AI's trigger use
   `Ballistics::step`, the same integrator the rounds use, so the sight cannot promise a hit
   the rounds won't deliver. The test requires 0 m difference.
4. **Hit sweeps in the target's frame.** Closure reaches 1,400 m/s (7 m per step), so the
   round's path is swept relative to the moving target.
5. **Skill is physiology and perception.** The tiers differ in reaction delay (the AI works
   from a delayed track), willing G, tracking noise, trigger discipline, reassessment rate,
   energy management and **greyout awareness**: the AI eases off when the OFC's vision-loss
   model starts to grey out, as a pilot would. Without this, AI pilots sustaining 7.5 G or
   more went GLOC and crashed.
6. **Training rules are part of the AI.** Left-to-left merge, no forward-quarter gun shots,
   a 150 m minimum range, CPA-based collision avoidance (predicted miss under 200 m within
   4 s), a hard deck that uses the real pull-out altitude loss R(1−cos γ), and a transonic
   chase cap. Without these, head-on merges ended in mid-airs and fights ran away to
   Mach 1.4 on the deck.
7. **G onset limit.** The AI loads the jet at 7 G/s and unloads at 12 G/s instead of stepping
   the stick.
8. **Separate turbulence per jet**, seeded per aircraft index. Jet 0 keeps the original stream,
   which is what lets Phase 0 be bit-exact.

### AI behaviour (24 seeds each, 180 s limit)

| Set-up | Pairing (own vs bandit) | W / L / timeout | Unforced ground impacts | Mid-airs |
|---|---|---|---|---|
| Offensive perch | Ace vs Novice | 9 / 0 / 15 | 0 | 0 |
| Offensive perch | Ace vs Veteran | 12 / 0 / 12 | 0 | 0 |
| Offensive perch | Novice vs Ace | 0 / 8 / 16 | 0 | 0 |
| Defensive perch | Novice vs Ace | 0 / 9 / 15 | 0 | 0 |
| Head-on merge | Veteran vs Veteran | 0 / 0 / 24 | 0 | 0 |

Across 240 AI-vs-AI fights there were zero unforced terrain impacts and zero mid-airs.

### Findings in existing systems

- **FLCS at high dynamic pressure.** Above ~420 m/s at low altitude (Mach 1.3+), coupled
  roll and pull inputs on the F-16 law produce Nz excursions of 11–40 G and roll rates above
  the rated 308 °/s. A straight stick step stays at or below 11.8 G. Auto-GCAS recoveries
  that coincide with GLOC in this regime oscillate. The AI now avoids that corner of the
  envelope, but a human can still reach it. The FLCS law needs work here. It is not fixed
  in this slice because it predates it.
- The HOTAS speedbrake switch is overridden every step by the keyboard toggle (pre-existing
  behaviour, preserved exactly).

### Next steps (in priority order)

1. **Neutral fights are too passive.** Equal-skill merges almost always time out. Add the
   one-circle/two-circle choice from relative turn performance, vertical moves (high and low
   yo-yo, oblique turns) and a willingness to trade energy for angles when a shot is close.
2. **Gun trigger on HOTAS.** Keyboard **F** is bindable; add `btn_trigger` to `InputConfig`
   and gamepad right trigger.
3. **Visual cues.** Hit sparks and a smoke trail on a damaged bandit, gun sound, a
   distance-scaled spotting dot (Relaxed preset), and padlock view.
4. Dogfight setup screen in the menu system (airframes, set-up, skill, weather, realism
   preset). Today the set-up is chosen with F10/F11 and CLI flags.
5. Then Phases 5–8 as originally planned (IR missiles and flares, sensors, multi-bandit,
   debrief replay from the deterministic event log, distinct airframe models).

### Evade mode (2026-09-25)

A defensive mode built on the same world: a bandit with radar missiles and a gun starts in your
six and hunts you. It pulls part of Phase 5 (missiles, countermeasures) and Phase 6 (radar, RWR)
forward, radar-guided instead of IR, because the RWR lock and launch warnings are what the mode is about.
Menu **EVADE** or `--evade [easy|medium|hard|expert]`. **C** dispenses chaff and **F11** cycles
the difficulty. **F10** now cycles merge → offensive → defensive → range → evade.

| Piece | File | Tests |
|---|---|---|
| AIM-120C-class missile: motor, Mach drag, induced drag, G and lift limits, autopilot lag, PN, datalink → inertial → active seeker, proximity fuze, blast-frag damage | `sim/missile.hpp` | `test_missiles`: Mach 3.2 burnout, lift-limited turn when slow, PN miss < 2 m, flyout = live missile bit for bit, head-on > beam > tail zones |
| Fire-control radar (search, lock, memory, gimbal, notch, chaff) and RWR | `sim/radar.hpp`, `sim/world.hpp` | notch needs beaming *and* look-down; chaff only inside the doppler gate; lock → launch heard on the RWR |
| Launch doctrine from flyouts (Rmax, no-escape, shoot-shoot) | `sim/missile_shooter.hpp` | via `test_evade_mode` |
| Difficulty profiles, rules (bingo, Winchester, escape), stats | `sim/engagement.hpp` | `test_evade_mode` |
| Missile-defence AI (beam, notch the launcher, chaff, last-ditch break) for `--watch` and tests | `sim/evasion_pilot.hpp` | survives MEDIUM ≥ 4/8 and ≥ EXPERT + 3 |
| RWR tones (new-emitter chirps, lock beep, launch warble), panel RWR scope, HUD `LOCK 6`/`LAUNCH 6`, chaff count, debrief | `audio/audio_engine.hpp`, `graphics/cockpit_gauges.hpp`, `hud_collimator.hpp` | `test_render_fidelity` (gauge placement) |

Outcomes over 24 seeds per case (the "competent defence" is `EvasionPilot`):

| Level | Straight and level | Run in reheat | Competent defence survives |
|---|---|---|---|
| EASY (novice, gun) | killed 24/24 | survives 24/24 | 24/24 |
| MEDIUM (veteran, 2 missiles, Rmax shots, 10 nm) | killed 24/24 | survives 24/24 | 15/24 |
| HARD (ace, 4, no-escape shots, 6.5 nm) | killed 24/24 | killed 24/24 | 6/24 |
| EXPERT (ace, 4, shoot-shoot, 6 nm) | killed 24/24 | killed 24/24 | 2/24 |

Zero unforced ground impacts or mid-airs in all 480 fights.

Findings on the way:

- **BFM gun tracking fixed at high speed.** Between 20 Hz gun-solution refreshes, the AI held the lead
  direction in body axes, so it turned with the nose and the tracking loop saw a frozen error.
  Above ~300 m/s every skill level bobbed ±1 G behind a straight target and never fired. The
  lead is now held as an offset from the bandit. In `test_bfm_ai`, AI-vs-AI fights decided before
  the time limit went from 6/24 to 12/24, and ace-over-novice perch conversions from 3/12 to 8/12.
- **Chase speed cap by dynamic pressure.** The BFM pilot's transonic chase cap is now also limited
  to q ≤ 75 kPa (the FLCS problem corner). In evade the bandit may chase at up to 95 kPa while more
  than 6 km out, which lets identical jets match speeds.
- **The dogfight bandit could not kill a predictable player (2026-09-26).** Measured over 24 seeds
  against scripted players, a jet flying straight in reheat was never hit, and novices and veterans
  emptied all 511 rounds at a steady 5 G turner for no hits. There were three causes:
  (1) The AI's roll loop (1.6 stick/rad on a 300°/s rate-command FLCS) rocked the wings ±10° at
  2 Hz whenever it held a bank, and the nose yawed with them. It is now a roll-rate command,
  3/s × bank error, scaled by the jet's full-stick roll rate.
  (2) Inside 1.5 km the perceived track carried the bandit's acceleration forward, not just his
  velocity. Before, a 5 G turn was led from where the target had been heading a reaction time
  earlier, 10–15 m behind it. Beyond 1.5 km perception is unchanged, which leaves the evade
  missile-defence figures where they were.
  (3) The dogfight bandit now chases to Mach 1.4 (still under the q limits), so running short of
  full reheat no longer escapes.
  Now veterans and aces kill a 5 G turner 24/24 from every set-up, and an ace behind a straight
  runner kills 24/24. `test_bfm_ai` checks both.
- **A supersonic run is a real escape.** An AMRAAM launched from Mach 1.5 cannot run down a Mach 1.5
  target from more than ~11 km in the tail. HARD therefore starts inside the no-escape zone.
- `World::step` keeps the new subsystems in one out-of-line call. Inlining them changed FMA
  contraction in the aircraft step and broke `test_aircraft_entity`'s bit-exact check under
  `-march=native`.

Next for evade: IR missiles (AIM-9X) with flares and a missile approach warner on the jets that
have one, an F-22 LPI radar that the RWR hears late, and a realism toggle for the HUD
`LOCK`/`LAUNCH` cue (an assist: the real F-16 shows threats only on the RWR scope).

---

## 1. Design pillars

1. **Same physics for everyone.** AI pilots produce the same `PilotCommands` and
   throttle a human does and go through the same FLCS, OFC (GLOC, Auto-GCAS),
   engine and RK4 integrator. An AI can never turn tighter, accelerate harder or
   ignore G than the airframe allows. This is the single most important realism
   rule and it is enforced structurally, not by tuning.
2. **Real envelopes, visible to the player.** Guns fly ballistic rounds with real
   muzzle velocity and rate of fire; missiles have real motors, seeker limits and
   minimum/maximum ranges. The HUD shows the envelope so the player learns *why*
   a shot missed.
3. **Energy is the currency.** Turn rate, turn radius, specific excess power and
   corner speed come out of the existing aero/propulsion models — the mode adds
   readouts, not special cases.
4. **Realism with honest assists.** Every assist (target box, lead cue, relaxed
   GLOC) is an explicit, labelled setting, off in the "Realistic" preset.
5. **Testable like the rest of the sim.** Every system lands with deterministic
   closed-loop tests in the style of `tests/test_ofc_closed_loop.cpp`.

## 2. What already exists to build on

| Area | Existing asset | Where |
|---|---|---|
| Flight dynamics | Validated 6-DoF RK4 for five airframes, corrected compressibility, per-profile limits | `include/fastjet/fdm/`, `aero/` |
| Flight controls | G/alpha-limited FBW (F-16/F-15EX/Typhoon/F-22), feel-limited A-10, roll-rate trim | `include/fastjet/flcs/aircraft_flcs.hpp`, `f16_flcs.hpp` |
| Pilot physiology | GLOC exposure model, greyout/blackout fade | `onboard_flight_computer.hpp` |
| Safety systems | Auto-GCAS, departure recovery (airframe-aware) | `onboard_flight_computer.hpp` |
| World | Procedural terrain, collision, atmosphere, wind/turbulence | `environment/`, `graphics/terrain_field.hpp` |
| Visuals | Cockpit + chase camera, collimated HUD, MFDs, GLB model renderer with camera-relative matrices | `graphics/render_engine.hpp`, `model_glb.hpp`, `hud_collimator.hpp` |
| Audio | Engine synthesis, sample one-shots (over-G, stall) | `audio/audio_engine.hpp` |
| UI | Menu system, settings persistence | `ui/` |
| Test harness | Closed-loop per-airframe sim with IMU→OFC→FLCS→engine→RK4 | `tests/test_ofc_closed_loop.cpp` (`ClosedLoopSim`) |

## 3. Gaps

- **Single-aircraft main loop.** Every per-aircraft object (engine, gear, FLCS,
  OFC, mass, forces, integrator step, collision) is a local in `main()` of
  `src/f16_sim_viewer.cpp`. There is no aircraft entity to instantiate twice.
- **One visual model.** `assets/models/f16.glb` is the only airframe model, and
  the renderer draws exactly one aircraft (the ownship).
- **No weapons, sensors, AI, damage, or mission flow.**

## 4. Architecture

```
World (fixed capacity, zero heap in the 200 Hz loop)
 ├── Aircraft[kMaxAircraft = 8]       ← extracted from main(); one per jet
 │     ├── FlightState, MassProperties, forces
 │     ├── MultiEngine, LandingGear, AircraftFLCS, OnBoardFlightComputer
 │     ├── PilotSource: Human(InputManager) | AI(BfmPilot)
 │     ├── WeaponsStore (gun rounds, missiles, flares)
 │     ├── Sensors (radar/IRST track, RWR, visual)
 │     └── Damage / alive state
 ├── Projectile pool[kMaxRounds ≈ 4096]   ← gun rounds, swept-segment hit tests
 ├── Missile pool[kMaxMissiles = 32]
 ├── Countermeasure pool (flares)
 └── Engagement (setup, scoring, end conditions, event log for debrief)
```

- **`sim::Aircraft`** owns exactly what `ClosedLoopSim` owns today plus the
  viewer's gear, wind and collision handling, and exposes
  `step(dt, PilotCommands, throttle, const WorldView&)`. The ownship becomes
  `aircraft[0]`; the viewer loop becomes "gather inputs → step world → render".
- **Pilot sources are interchangeable.** The human path is today's
  `InputManager` + keyboard smoothing. The AI path produces the same struct. AI
  jets run their own OFC, so Auto-GCAS protects them from terrain and GLOC can
  knock an AI pilot out mid-fight exactly as it can the player.
- **Rendering others.** Draw each non-ownship `Aircraft` with
  `ModelGLB::relative_matrix(state, camera_origin, WORLD)` (already camera-relative,
  so no precision loss at range), plus a distance-scaled "spotting dot" so a
  bandit is visible at realistic ranges on a 1080p screen.
- **Determinism.** Fixed 200 Hz step, seeded RNG per engagement → an
  engagement can be replayed from its inputs (debrief, regression tests).

## 5. Phases

Each phase ends playable and fully tested. Sizes: S ≈ days, M ≈ 1–2 weeks,
L ≈ 2–4 weeks of focused work.

### Phase 0 — Extract the aircraft entity (M)
- Move the per-aircraft pipeline out of `main()` into `sim::Aircraft`; add
  `sim::World` holding the fixed array.
- **No behaviour change.** Acceptance: every existing suite passes, and a new
  test flies the same inputs through the old path and `sim::Aircraft` with
  bit-identical state after 60 s.

### Phase 1 — Second aircraft, BFM range (M)
- Spawn N aircraft; render them; chase camera can cycle targets.
- **"Range" sub-mode:** a scripted target (straight-and-level, constant-G turn,
  weave) for gunnery and pursuit practice.
- HUD additions: target designator box, range, closure (Vc), aspect angle,
  antenna-train angle, own turn rate, Ps (specific excess power) readout.
- Mid-air collision between aircraft (bounding capsules).
- Tests: target script follows its path; HUD geometry (range, aspect) matches
  analytic values for known placements.

### Phase 2 — Guns (L)
- Per-airframe gun data (verify figures against primary sources before shipping):

  | Airframe | Gun | Rate | Muzzle vel. | Rounds |
  |---|---|---|---|---|
  | F-16C | M61A1 20 mm | ~6,000 rpm | ~1,050 m/s | 511 |
  | F-15EX | M61A1 20 mm | ~6,000 rpm | ~1,050 m/s | ~510 (verify) |
  | F-22A | M61A2 20 mm | ~6,000 rpm | ~1,050 m/s | 480 |
  | Typhoon | Mauser BK-27 27 mm | ~1,700 rpm | ~1,025 m/s | 150 |
  | A-10C | GAU-8/A 30 mm | ~3,900 rpm | ~1,010 m/s | 1,174 |

- Rounds are pooled projectiles inheriting the shooter's velocity, with gravity,
  drag (atmosphere-dependent) and a dispersion cone; hits via swept segment vs
  target capsules each step (rounds move ~5 m per step, so point tests miss).
- Gunsight: LCOS pipper and an EEGS-style funnel computed by integrating a
  projected bullet stream against the target's current turn — the same
  ballistics as the real rounds, so the funnel is honest.
- Recoil/vibration on the camera rig; gun audio; ammo counter on HUD/MFD.
- Tests: projectile drop/time-of-flight vs analytic ballistics; a
  non-manoeuvring target at 600 m is hit when the pipper is on it; funnel
  predicts the rounds' actual position within tolerance.

### Phase 3 — Damage model (M)
- Component damage rather than hit points: engine(s) (thrust loss, fire), fuel
  leak (fuel_kg drain), control surfaces (reduced actuator authority/rate in
  `Actuator`), hydraulics, pilot. Twin-engine jets survive losing one engine;
  the A-10's redundancy is a real advantage.
- Damage feeds the existing models (thrust limit, actuator limits, mass), so a
  damaged jet *flies* damaged.
- Ejection and kill/score events.

### Phase 4 — AI BFM pilot (L, the core of the mode)
- Layered pilot: **tactical layer** (situation assessment every ~0.2 s:
  offensive / defensive / neutral from range, aspect, angle-off, energy state)
  → **manoeuvre layer** (pure/lead/lag pursuit, high/low yo-yo, break turn,
  jink, scissors, one-circle vs two-circle choice from relative turn
  performance, extension/separation) → **control layer** (G/roll/throttle
  controllers that output `PilotCommands`).
- Energy logic uses the jet's real corner speed and Ps (computed from the aero
  and engine models at the current condition, not tables).
- Skill tiers as physiological/perceptual limits, not cheats: reaction delay,
  G tolerance (feeds the GLOC model), tracking error, tactical choices, and how
  often it checks six.
- AI uses Auto-GCAS like the player; it will also respect a minimum altitude
  "hard deck" setting.
- Tests: AI-vs-AI 1v1 in identical jets always terminates (kill, bingo fuel or
  timeout) with no terrain impacts; an offensive AI converts a 3,000 ft
  perch set-up into a guns solution within a bounded time; a defensive AI
  defeats a gun pass from a lower-skill attacker a measurable share of the time.

### Phase 5 — Short-range missiles & countermeasures (L)
- IR missiles per airframe: AIM-9X (F-16, F-15EX, F-22), ASRAAM/IRIS-T
  (Typhoon), AIM-9M (A-10). Rocket motor with burn time, drag, max-G
  airframe limit, proportional navigation guidance, seeker gimbal/field-of-view
  limits, off-boresight capability per variant, fuze and warhead lethal radius.
- Seeker tone (growl → lock), uncage, dynamic launch zone (Rmin / Rmax /
  no-escape) on the HUD.
- Flares with burn time and IR intensity; seeker decoy probability from
  relative intensity, aspect and flare/target separation.
- Tests: PN intercept of a non-manoeuvring target; missile cannot pull more than
  its limit; launches outside Rmax fall short; seeker loses a target outside
  its gimbal.

### Phase 6 — Sensors & awareness (M)
- Radar air-combat modes (boresight, vertical scan, auto-acquisition) with
  realistic lock ranges; RWR for AI radar locks; padlock view (hold eyes on the
  bandit, respecting head-movement limits); optional IRST.
- Visual detection limits: the bandit is only boxed on HUD when a sensor or the
  eyes could plausibly see it (assist setting can relax this).

### Phase 7 — Game flow & UI (M)
- Main menu: **DOGFIGHT** entry → setup screen: own/adversary airframe,
  adversary count (1v1 first, then 1v2/2v2), start geometry (neutral head-on
  merge at 2 nm / offensive perch / defensive perch / high-aspect BVR-to-merge),
  altitude, weather, AI skill, weapons (guns only / guns + IR), realism preset.
- Realism presets: **Realistic** (all effects, no assists), **Standard** (target
  box, lead cue), **Relaxed** (reduced GLOC onset, spotting dots) — all built
  from individual toggles in `UserSettings`.
- End-of-engagement debrief: outcome, shots/hits, time in G, energy history,
  plus a top-down replay from the deterministic event log (optionally export
  Tacview ACMI for external review).

### Phase 8 — Performance & polish (S–M)
- Budget: 8 aircraft + 4,096 rounds + 32 missiles within ~1 ms of the 5 ms
  physics step on the target machine; profile `compute_forces_and_moments`
  (4 calls per RK4 step per aircraft).
- Distinct visual models for the F-15EX, Typhoon, F-22 and A-10 (currently all
  render as the F-16; sourcing licensed GLBs is a dependency).
- Audio: gun, missile launch/growl, RWR, Betty calls.

## 6. Milestones

1. **MVP — Guns BFM (Phases 0–4):** 1v1, same-type jets, guns only, neutral /
   offensive / defensive set-ups, three AI skill tiers. This is already a
   complete, realistic training product.
2. **Missiles (Phase 5):** short-range IR + flares.
3. **Full mode (Phases 6–8):** sensors, multi-bandit, mixed airframes, debrief.

## 7. Testing strategy

- Deterministic closed-loop tests for every system (as in
  `test_ofc_closed_loop.cpp`); engagements replayed from seeds in CI.
- Analytic checks where physics allows: ballistics, PN miss distance, turn-rate
  and radius vs `(n, V)` relations.
- Behavioural statistics for AI: win-rate bands per skill tier across seeded
  set-ups, zero terrain impacts, bounded engagement length.
- Performance test with the full entity budget.

## 8. Decisions needed from you

Defaults taken for the MVP (change any of them): guns-only first; BVR left out; same-type
matchups by default, with mixed types allowed through `--bandit`; the F-16 model for every
airframe; component damage.

1. **Missiles in the first release, or guns-only MVP first?** (Recommended:
   guns-only MVP — it exercises every system except guidance and is where BFM
   realism lives.)
2. **BVR (AIM-120/Meteor) in scope at all?** It is a different game (radar,
   datalink, notching); recommended to leave out of this mode.
3. **Mixed-type matchups at launch** (e.g. F-16 vs Typhoon), or same-type only
   until the AI is proven? Mixed types need the AI's energy logic to be solid.
4. **Aircraft models:** source/licence GLBs for the other four airframes, or
   accept the F-16 model for all in the MVP.
5. **Damage model depth:** component damage (recommended, realistic) vs simple
   hit points (faster to build).
