# F-35B Lightning II (STOVL) — Simulator Reference Data

Compiled 2026-10-06 from public sources. Every number has a source tag `[Sn]` (URLs at the bottom).
**EST** = engineering estimate (not public); the reasoning is given next to it. Use the published
numbers where they exist. Where sources disagree, both values are shown and one is marked **USE**.

Unit conversions: 1 lbf = 4.448 N · 1 lb = 0.4536 kg · 1 ft = 0.3048 m · 1 kt = 0.5144 m/s · 1 slug·ft² = 1.3558 kg·m²

---

## 1. Geometry

| Item | SI | Imperial | Source |
|---|---|---|---|
| Length | 15.6 m | 51.2 ft | [S1][S2] |
| Wingspan | 10.7 m | 35 ft | [S1][S2] |
| Height | 4.36 m | 14.3 ft | [S2][S3] |
| Wing reference area | 42.7 m² | 460 ft² | [S1][S2] |
| Aspect ratio | 2.66 | — | [S3] (b²/S = 2.68) |
| Horizontal tail span | 6.65 m | 21.8 ft | [S2] |
| Wing root chord / tip chord | 6.46 m / 1.57 m | 21.19 ft / 5.15 ft | [S31] (student planform measurement, F-35A, same wing as B) |
| Taper ratio | 0.243 | — | [S31] |
| LE sweep / c/4 sweep / TE sweep | 34.1° / 24.2° / −13.0° | — | [S31] |
| MAC | 4.51 m | 14.8 ft | [S31] (matches trapezoid formula) |
| Total horizontal tail area (2 stabilators) | 11.96 m² | 128.7 ft² | [S31] (measured from planform) |
| Total vertical tail area (2 fins) | 7.95 m² | 85.6 ft² | [S31]; tail taper 0.59 |
| Vertical tail cant | ~25° outboard | — | **EST** (measured from frontal photos) |
| Wheelbase / main gear track | 6.02 m / 4.34 m | 19.75 ft / 14.24 ft | [S31] for F-35A; **EST** B is within ±0.2 m |

## 2. Mass

| Item | SI | Imperial | Source / note |
|---|---|---|---|
| Empty weight | 14,729 kg | 32,472 lb | [S3] variant table. Older LM figure 32,300 lb [S36]. **USE 32,472** |
| Internal fuel | 5,942 kg | 13,100 lb | [S1] LM 2019 card. MINDEF gives 13,500 lb / 6,125 kg [S2]. **USE 13,100** |
| Weapons payload (max) | 6,800 kg | 15,000 lb | [S1] |
| Max takeoff weight | 27,200 kg | 60,000 lb class | [S3] |
| Typical combat weight (50 % fuel, 4×AIM-120, pilot) | ~18,600 kg | ~41,000 lb | **EST**: 32,472 + 6,550 + 4×335 + 250 + gun ammo/misc ≈ 41,000 |
| Vertical lift bring-back (VLBB) KPP | — | 2×1,000 lb JDAM + 2×AIM-120 + reserve fuel for the STOVL recovery profile | [S9] |
| Max vertical landing weight | ~17,500 kg | ~38,500 lb | **EST**: hover thrust 40,650 lbf [S4] divided by T/W ≈ 1.05–1.06 needed for control margin, HGI and suckdown. Check: 32,472 + 2,100 (JDAM) + 700 (AMRAAM) + 250 pilot + ~2,500 fuel ≈ 38,000 lb meets the KPP |
| Max hover weight (OGE, ISA SL) | ~18,000 kg | ~39,500 lb | **EST**: T/W ≈ 1.03 OGE. Reduce about 1 % per °C above ISA (hot-day thrust loss) |
| Pilot + seat boarding mass range | 46.7–111.1 kg | 103–245 lb (nude) | [S25] |

### Moments of inertia (all **EST**)
Method: F-16 values (Stevens & Lewis / NASA TP-1538: Ixx 9,496, Iyy 55,814, Izz 63,100, Ixz 982 slug·ft² at 20,500 lb [S35])
scaled by I ∝ m·L². Roll uses the span ratio (10.7/9.45)², and pitch/yaw use the length ratio (15.6/15.0)². Scaled to 41,000 lb.

| Axis | kg·m² | slug·ft² |
|---|---|---|
| Ixx | 33,900 | 25,000 |
| Iyy | 162,700 | 120,000 |
| Izz | 189,800 | 140,000 |
| Ixz | 2,700 | 2,000 |

At full internal fuel plus stores (~47,000 lb), multiply by ~1.15. At empty plus pilot, multiply by ~0.82.

## 3. Propulsion — F135-PW-600 + Rolls-Royce LiftSystem

### 3.1 Main engine
| Item | SI | Imperial | Source |
|---|---|---|---|
| Max (AB) thrust | 169 kN / 182 kN | 38,000 lbf (LM) / 41,000 lbf class (P&W) | [S1] / [S4] |
| Mil (dry) thrust | 116 kN / 120 kN | 26,000 lbf (LM) / 27,000 lbf class (P&W) | [S1] / [S4] |
| Short-takeoff thrust class | 181.2 kN | 40,740 lbf | [S4] |
| Hover thrust class (total vertical) | 180.8 kN | 40,650 lbf (LM: 40,500 vertical) | [S4] / [S1] |
| Engine length | 9.37 m | 369 in | [S4] |
| Main inlet dia / max dia | 1.09 m / 1.17 m | 43 in / 46 in | [S4] |
| Lift fan inlet dia / max dia | 1.30 m / 1.35 m | 51 in / 53 in | [S4] |
| TSFC dry (mil, SL static) | ~22 g/(kN·s) | ~0.78 lb/(lbf·h) | **EST** (F119/F100-class low-bypass turbofan, BPR ≈ 0.57) |
| TSFC max AB | ~54 g/(kN·s) | ~1.9 lb/(lbf·h) | **EST** |
| TSFC STOVL hover | ~18 g/(kN·s) of *lift* | ~0.65 lb/(lbf·h) | **EST**: the shaft-driven fan raises the effective bypass ratio. Hover fuel burn ≈ 40,000 × 0.65 / 3600 ≈ 7 lb/s ≈ 3.3 kg/s |
| Spool idle→mil / mil→max AB | ~4.0 s / ~1.5 s | — | **EST** (typical modern FADEC) |
| Hover thrust response (ΔT per pilot input) | fast: lift-fan blade/vane + nozzle area modulation | — | lift off to 20 ft within 5 s of thrust request [S12] |

### 3.2 STOVL lift split (hover)
| Post | Thrust | Share | Source |
|---|---|---|---|
| LiftFan (cold, 2-stage counter-rotating, 50 in) | 18,680 lbf (83.1 kN) P&W / ">20,000 lbf" (89 kN) RR | ~46 % | [S6] / [S5] |
| 3BSM main nozzle (hot, **dry only**) | 18,680 lbf (83.1 kN) P&W / 18,000 lbf (80 kN) RR | ~46 % | [S6] / [S5] |
| Roll posts (bypass air, 2×) | 3,290 lbf total (14.6 kN) P&W / 1,950 lbf **each** (8.7 kN) RR | ~8 % | [S6] / [S5] |
| Total | 40,650 lbf (P&W) / 41,900 lbf (RR sum) | 100 % | [S4] / [S7] |
| LiftFan shaft power (driveshaft + clutch) | up to 29,000 shp (21.6 MW) | — | [S5] |

**USE** the P&W consistent set: 18,680 / 18,680 / 2 × 1,645 = 40,650 lbf. The RR numbers are per-component maxima.
In the sim, scale all three together with throttle/thrust demand. Trim pitch by redistributing fan vs 3BSM thrust (±10 % authority **EST**). Command roll by splitting roll-post flow left/right. Do the split by area modulation: total roll-post thrust stays roughly constant.

### 3.3 Nozzles, vectoring, rates
| Item | Value | Source |
|---|---|---|
| 3BSM pitch range | 0° (aft, conventional) → 95° down (slightly forward of vertical) | [S5] |
| 3BSM slew rate | 95° in 2.5 s → ~38 °/s | [S5][S14] |
| 3BSM yaw (lateral) vectoring | used for hover yaw control; angle not public → **EST ±12°** | yaw role [S7]; angle EST |
| Afterburner | 3BSM "reheat capable during conventional flight" only. **No AB in STOVL mode** | [S5] |
| LiftFan variable-area vane-box nozzle | vectors fan thrust through **54° in < 2 s** | [S8] |
| Vane-box angle split | **EST** ~−5° (slightly forward, for braking) to +49° aft (STO/acceleration) | EST |
| Roll-post nozzles | hydraulically (fueldraulically) actuated, area-modulated, under each wing | [S5][S7] |
| LiftFan max operating airspeed (doors open, fan engaged) | 250 kt | [S7] |

### 3.4 Mode conversion (STOVL/HOOK button)
| Item | Value | Source |
|---|---|---|
| Initiation | single "STOVL/HOOK" button. The B has no hook; the button commands conversion | [S3] |
| Sequence | (1) all STOVL doors open: lift-fan upper inlet + lower exhaust doors, roll-nozzle doors, dorsal auxiliary-inlet doors, aft 3BSM doors; (2) clutch engages and carbon plates spin the fan up to engine speed; (3) mechanical lock-up once speeds match; (4) 3BSM begins vectoring down | [S11][S12] |
| Conversion time (CTOL → STOVL) | ~15 s | [S12] |
| Reconversion time (STOVL → CTOL) | ~10 s; doors shut in ≤ 10 s (≈5 s observed) | [S34] (forum, secondary) |
| Max conversion airspeed | 250 KCAS (LiftFan limit). First in-flight conversion was at 210 kt / 5,000 ft. Flown at 180 kt engaged | [S7] / [S10] |
| Typical test conversion | convert at 200 kt, decelerate to 60 kt, then decelerating approach to a 0-kt hover at 150 ft | [S13] |
| Protection | FCS provides "conversion speed window protection", "performance deficit protection", FOD warning | [S16] (Tomlinson 2011) |

### 3.5 Lift-post locations relative to CG (all **EST**, body axes, x fwd, y right, z down)
Method: lift fan centred just aft of the canopy at ~5.3 m from the nose. The 3BSM exit is under the aft fuselage at ~12.5 m.
The pitch-moment balance 18,680·(x_cg − 5.3) = 18,680·(12.5 − x_cg) puts x_cg ≈ 8.9 m from the nose (57 % L), which is plausible.

| Post | x (m) | y (m) | z (m) | Thrust line in hover |
|---|---|---|---|---|
| LiftFan nozzle exit | +3.6 | 0 | +0.6 | down, vane-box vectored aft |
| 3BSM nozzle exit | −3.6 | 0 | +0.7 | down (95°), lateral ±12° |
| Roll post L / R | −0.3 | ∓2.4 | +0.3 | down. Differential gives ±1,950 lbf × 2.4 m ≈ ±21 kN·m roll moment |
| Main gear | −0.6 | ±2.17 | +1.9 | — |
| Nose gear | +5.4 | 0 | +1.9 | — |

## 4. Performance and limits

| Item | Value | Source |
|---|---|---|
| Max Mach | 1.6 | [S1] |
| Positive g limit | +7.0 g | [S1] |
| Negative g limit | −3.0 g (F-35 test aircraft AF-2; B assumed same) | [S37] (page not re-fetched; 403) / **EST** for B |
| AoA limiter | 50° (trimmed alpha 50°; F-35A cleared to 50°, flown to beyond limiter) | [S3][S19]. B assumed same in CTOL mode |
| Sustained turn (spec, M0.8/15k ft) | 4.5 g (lowered from 5.0 g) | [S18] |
| Transonic accel M0.8→1.2 | spec relaxed by 16 s vs original | [S18] |
| Max roll rate | ~200 °/s subsonic clean | **EST** (fighter FBW limiter typical; F-16 ≈ 240 °/s) |
| Service ceiling | 50,000 ft (15,240 m) | [S3] (F-35 infobox) |
| Combat radius (internal fuel, STOVL profile) | > 450 nmi (833 km) LM. 505 nmi (935 km) demonstrated/estimated in SAR | [S1] / [S9] |
| Range (internal fuel) | > 900 nmi (1,667 km) | [S1] |
| STO deck run (KPP) | 600 ft (183 m) from LHA/LHD (450 ft UK ski-jump), SL tropical day, 10 kt WOD, with 2×1,000 lb JDAM + 2×AIM-120 + fuel for 450 nmi | [S9] |
| STO demonstrated | 471 ft (144 m), same load | [S9] |
| STO speed at end of deck | ~70–90 KCAS | **EST** (needs nozzle/vane deflection plus wing lift, T/W ≈ 0.8) |
| Slow landing (STOVL mode, semi-jetborne) | demonstrated at 90, 75 (groundspeed) and 70 kt. Pattern flown at 80, 60, 50, 40 kt | [S33][S13] |
| SRVL (shipborne rolling VL, UK) | ~57 KIAS approach | [S22] |
| CTOL approach speed | ~140–150 KCAS (AoA ≈ 12–13°) | **EST** |
| Max descent rate in VL (stick-commanded) | 7 ft/s (2.1 m/s). Tested 4–12 ft/s | [S16] (Norris, AvWeek 2011) |
| Hover height before VL | 150 ft (46 m) typical test | [S14] |
| Ground effect / suckdown | the cool lift-fan curtain acts as a "dam" that keeps hot 3BSM exhaust aft and reduces HGI | [S21] |
| Hot-day / HGI risk | HGI and thrust deficit in hot climates can cause uncontrolled sink and hard landing | [S20] |
| Suckdown magnitude | **EST** −3 % to −5 % of thrust at wheel height < 3 m (h/D_fan < 2). Fountain partly offsets it near touchdown | EST |
| HGI thrust loss | **EST** ≈ 1 % thrust per +1 °C inlet temperature rise. Model inlet ΔT ≈ 5–15 °C below 15 ft, worse with tailwind | EST |

## 5. Flight control (Unified / STOVL control laws)

Hardware: triplex-redundant fly-by-wire [S3]. Electro-hydrostatic actuators (EHA) are the **sole** primary flight-control
actuation, driven from the 270 VDC bus [S30]. Vehicle Management Computers command them. Relaxed static stability [S3].
Inceptors: right-hand side-stick (RHI) and left-hand throttle (LHI) [S17]. Origin: VAAC Harrier "unified" control laws [S15][S16].

### 5.1 Inceptor mapping (published: Walker, Fuller & Wurth 2013, AIAA 2013-4243, tabulated in [S15]; Tomlinson 2011 in [S16])

| Inceptor | Wingborne (CTOL & STOVL high speed) | Semi-jetborne (blend) | Jetborne / hover |
|---|---|---|---|
| Stick fore/aft | **flight-path-angle rate** γ̇ (pitch rate / AoA / g command in CTOL). Centred = hold γ | blend γ̇ → ḣ | **vertical velocity** ḣ (aft = up). Centred = hold height. Max down 7 ft/s |
| Stick lateral | **roll rate** p. Centred = hold bank | roll-rate → attitude blend (~130→100 kt per Farley/VAAC) | **bank angle** φ (lateral accel/translation). Release → wings level |
| Throttle (LHI) | CTOL: thrust. STOVL mode: **longitudinal acceleration** V̇ (detent = hold speed) | V̇ | V̇ / **ground-speed** command. Below 25 kt the aft detent = zero ground speed. ±30 kt closed-loop GS fwd/aft either side of detent |
| Pedals | **sideslip** β (coordinated) | blend β → ψ̇ | **yaw rate** ψ̇. Centred = hold heading |

Hands-off in hover: "the airplane simply hovers in place" [S12]. Centred inceptors hold height, position and heading.
In wingborne STOVL mode they hold speed, bank and γ [S16] (Farley 2006).

### 5.2 Modes and buttons
| Feature | Behaviour | Source |
|---|---|---|
| STOVL/HOOK button | starts auto-conversion (§3.4). Pressing again reconverts | [S3][S12] |
| "Decelerate-to-hover" button | auto-decelerates to hover; then the throttle commands accel/decel | [S16] (Norris 2011) |
| TRC (translational rate command) | pilot-selected (button) for precision hover. Stick displacement ∝ ground velocity; release → stop and hold position | [S16][S17] |
| ACAH (attitude command/attitude hold) | lateral mode in hover (bank command) | [S15] |
| Protections | AoA/β limiting, departure protection, performance-deficit protection, conversion-speed-window protection | [S16] |

### 5.3 Suggested sim implementation (**EST**, for this codebase)
- Blend factor `k = clamp((V_cas − 40)/(100 − 40), 0, 1)`. 40 kt is the pitch/pedal switch point and 100–130 kt the roll blend [S16].
- Stick pitch: `cmd = (1−k)·ḣ_cmd + k·γ̇_cmd`, with ḣ_max = +10/−7 ft/s and γ̇_max ≈ 10 °/s.
- Stick roll: `φ_cmd` up to ±20° in hover (TRC: v_y up to ±10 kt). Roll rate up to 200 °/s wingborne.
- Throttle: V̇ = ±0.25 g full deflection in STOVL. Below 25 kt the aft detent → GS = 0 (hold position).
- Pedals: ψ̇_max = 30 °/s in hover. β_max = 10° wingborne.
- Effectors in hover: ḣ → total lift-post thrust. Pitch → fan/3BSM split + vane-box. Roll → roll-post split. Yaw → 3BSM lateral. Fwd accel → vane-box + 3BSM angle + pitch attitude (≤ ±10°).

## 6. Avionics / cockpit modelling

| Item | Value | Source |
|---|---|---|
| Panoramic Cockpit Display (PCD) | 20 × 8 in (508 × 203 mm) AMLCD touch screen, electronically split into two 10 × 8 in halves (redundant). User-configurable "portal" windows | [S23] |
| HUD | none. All HUD symbology is on the helmet | [S3] |
| HMDS Gen III (Collins/Elbit) | biocular, 40° × 30° FOV. Projects DAS imagery for "see-through-airframe" view | [S24] |
| Inceptors | side-stick right, throttle left (active inceptors) | [S15][S17] |
| Canopy | one-piece tinted transparency, **hinged at the front**, with an *internal structural frame* (a visible arch, not a separate windscreen bow) | [S3] |
| Ejection seat | Martin-Baker US16E: 0/0, max 600 KEAS, ceiling 50,000 ft, twin catapult on side rails | [S25][S3] |
| Radar | AN/APG-81 X-band AESA. Air-to-air active/passive, SAR, EW modes | [S29] |
| APG-81 detection range | **EST**: ~150 km (81 nmi) vs 1 m², ~225 km (120 nmi) vs 5 m². R ∝ σ^¼ from the 1 m² anchor (typical large fighter AESA). FOV ±60° az/el (fixed array) | EST |
| DAS | AN/AAQ-37: 6 IR apertures, 360° spherical (missile warning, IRST, night vision) | [S3][S24] |
| EOTS | AN/AAQ-40 under the nose (faceted window) | [S3] |
| RCS | "lower than a metal golf ball" at certain aspects. Public estimate ≈ 0.0015 m² (−28 dBsm) frontal X-band (**EST**/press) | [S3] / [S38] |
| Gun (F-35B) | GAU-22/A 25 mm 4-barrel in centreline missionized gun pod, **220 rds**. B pod differs from C pod | [S3][S27] |
| GAU-22/A | 3,300 rds/min (220 rds = 4.0 s), MV 1,085 m/s (3,560 ft/s) TP/HEI, 1,036 m/s API, gun 104 kg/230 lb, recoil 16.5 kN | [S26] |
| Internal AAM | 4× AIM-120 (2 per bay). The B bays are smaller (lift fan), so the Sidekick 6-AMRAAM rack is **not** compatible | [S28] |
| External | 6 underwing/fuselage stations ("beast mode"). B outboard stations limited to 1,500 lb (680 kg) | [S3] |

## 7. Aero estimates for a polar model (all **EST**, with reasoning)

| Parameter | Value | Reasoning |
|---|---|---|
| CD0 subsonic (M ≤ 0.8), clean | **0.021** (range 0.019–0.023) | Skin friction alone ≈ 0.007 (VT FRICTION code [S31]). Add ~2.5–3× for form, inlets, interference, gaps and RAM roughness. The B has a fatter spine (lift fan) than the A: +0.001 |
| Drag-divergence Mach | 0.88 | Thin wing (t/c ≈ 5 %), c/4 sweep 24° |
| CD0 peak (M 1.05–1.15) | 0.046 (≈2.2× subsonic) | blunt high-volume fuselage. Poor transonic acceleration [S18] |
| CD0 at M 1.6 | 0.040 | wave drag decays ∝ 1/√(M²−1) partially |
| Oswald e / K (subsonic) | e ≈ 0.80, K = 1/(π·2.66·0.80) = **0.150** | AR 2.66 |
| K at M 1.2 / 1.6 | 0.22 / 0.30 | supersonic K ≈ √(M²−1)/(4·CLα)-type growth + LE suction loss |
| CLα (wing-body, M 0.3) | **3.3 /rad (0.058 /deg)** | DATCOM low-AR: 2πA/(2+√(4+A²β²/η²·(1+tan²Λc/2/β²))) = 3.10, with Λc/2 ≈ 12.4°. +~7 % body/LEX lift |
| CLα at M 0.9 / M 1.4 | 3.8 / 2.6 /rad | Prandtl-Glauert to M 0.9; supersonic 4/√(M²−1) capped by low-AR |
| CLmax (clean, flaps auto) | **1.5 at α ≈ 35°**. Linear to ~22°, gentle break, no hard stall up to 50° (CL ≈ 1.3) | chined forebody/LEX vortex lift; trimmed 50° α [S3] |
| CLmax (STOVL/approach flaps) | 1.7 (powered lift excluded) | EST |
| Cm0 / static margin | ~−5 % MAC subsonic (unstable), +~10 % supersonic | relaxed stability [S3] |
| Check | M 1.6 at 36 kft, W 41 klb: CL 0.105, CD ≈ 0.043, D ≈ 16.5 klbf ≈ AB thrust available at altitude (~17–19 klbf) | consistent with the M 1.6 limit |

## 8. Visual / procedural 3D model notes

| Feature | Description (dimensions **EST** from photos/drawings unless sourced) |
|---|---|
| Lift-fan inlet door | Single large door directly behind the canopy, **hinged at its aft edge** and opening forward-up to ~70–80°. ~1.9 m long × 1.4 m wide (covers the 53 in fan). X-35B used side-hinged doors [S36b]. Inner face light grey |
| Lift-fan exhaust (lower) doors | Under the fuselage below the fan: two doors hinged at their outboard edges, opening sideways to ~90°. The vane box (row of ~9 louvres) is visible between them |
| Auxiliary inlet doors | Dorsal, immediately aft of the lift-fan door: a pair of small side-by-side doors (~0.6 × 0.4 m each), hinged at the aft edge, opening up ~40° [S11] (existence/location) |
| 3BSM | Round nozzle with three canted segments (cuts at ~20–30° to the axis). Rotates down through 95°. 3BSM doors under the aft fuselage open sideways for clearance. Nozzle exit dia ~1.0 m; serrated "LO" edges on the outer flaps |
| Roll-post doors | One under each wing root (~0.5 × 0.25 m), just outboard of the main gear bays, about at y ≈ ±2.4 m; flush when closed |
| Weapons bays | Two bays either side of the centreline, each with 2 doors (inboard + outboard, hinged at the bay edges, saw-tooth ends). The B bays are shorter than the A's, ~3.6 m long [S28] (smaller) |
| Canopy | One-piece, front-hinged, internal frame arch behind the headrest, gold/ITO tint [S3] |
| Inlets | Diverterless supersonic inlets (DSI) with compression bump and forward-swept three-sided cowl, both sides of the cockpit [S3] |
| Tails | Twin all-moving vertical fins canted ~25° outboard; all-moving stabilators span 6.65 m [S2] |
| Wing | Trapezoid, LE 34°, TE forward-swept −13°, sawtooth panel edges [S31] |
| Gun pod | Centreline pod ~3.4 m long, faceted LO shape (B-specific) [S27] |
| Colour | Overall **FS 36170** "Have Glass" neutral grey, all variants [S32]. RAM-treated edges/panels appear slightly darker (≈FS 36118 tint, **EST**). Grey-on-grey low-vis markings |

---

## Sources
- [S1] Lockheed Martin F-35B product card (2019): https://www.f35.com/content/dam/lockheed-martin/aero/f35/documents/FG19-00608_002%20Product%20Card%20F-35B%20media.pdf
- [S2] Singapore MINDEF F-35B fact sheet: https://www.mindef.gov.sg/news-and-events/latest-releases/28feb24_fs2/
- [S3] Wikipedia, Lockheed Martin F-35 Lightning II (variant table, design section, citing LM/GAO): https://en.wikipedia.org/wiki/Lockheed_Martin_F-35_Lightning_II
- [S4] Pratt & Whitney F135-PW-600 product card (2020): https://filecache.mediaroom.com/mr5mr_prattwhitney/181857/download/F135-STOVL-Product-Card-2020.PDF
- [S5] Rolls-Royce LiftSystem datasheet VCOMB 4202: https://www.mylearning.rolls-royce.com/~/media/Files/R/Rolls-Royce/documents/defence/VCOMB-4202-Rolls-Royce-LiftSystem-datasheet.pdf
- [S6] Wikipedia, Pratt & Whitney F135 (STOVL thrust split): https://en.wikipedia.org/wiki/Pratt_%26_Whitney_F135
- [S7] Wikipedia, Rolls-Royce LiftSystem: https://en.wikipedia.org/wiki/Rolls-Royce_LiftSystem
- [S8] FlightGlobal, "STOVL JSF starts to take shape" (vane box 54° < 2 s): https://flightglobal.com/stovl-jsf-starts-to-take-shape/62861.article
- [S9] DoD F-35 Selected Acquisition Report, Dec 2018 (KPPs): https://www.esd.whs.mil/Portals/54/Documents/FOID/Reading%20Room/Selected_Acquisition_Reports/19-F-1098_DOC_33_F-35_SAR_Dec_2018.pdf
- [S10] Lockheed Martin, "F-35B Begins In-Flight STOVL Operations" (2010): https://news.lockheedmartin.com/2010-01-07-Lockheed-Martin-F-35B-Begins-In-Flight-STOVL-Operations
- [S11] Lockheed Martin, "F-35B Succeeds in STOVL Propulsion Ground Test" (2008): https://news.lockheedmartin.com/2008-05-29-Lockheed-Martin-F-35B-Succeeds-in-STOVL-Propulsion-Ground-Test
- [S12] Code One, "Going Vertical": https://www.codeonemagazine.com/f35_article.html?item_id=13
- [S13] Air & Space Forces, "Making STOVL Hover": https://www.airandspaceforces.com/makingstovlhover/
- [S14] Lockheed Martin, "The F-35B: First Descent": https://www.lockheedmartin.com/en-us/news/features/history/f35b.html
- [S15] "Taming the Tilt" (arXiv 2608.20300), Table 2 summarising Walker/Fuller/Wurth AIAA 2013-4243: https://arxiv.org/pdf/2608.20300
- [S16] F-16.net thread quoting Tomlinson ("Test Flying the JSF", 2011), Norris (AvWeek "Vertical Validation", 2011), Farley (2006). Secondary: https://www.f-16.net/forum/viewtopic.php?t=24054&p=252995
- [S17] The Air Current, "Seeking lessons for eVTOL pilot training in the F-35": https://theaircurrent.com/technology/special-report-evtol-pilot-training-f-35/
- [S18] FlightGlobal, "Pentagon lowers F-35 performance bar" (DOT&E FY2012): https://www.flightglobal.com/pentagon-lowers-f-35-performance-bar/108424.article
- [S19] The Aviationist, F-35 AoA testing: https://theaviationist.com/2012/11/20/f-35-aoa/
- [S20] Military Times, F-35B landing in hot environments (HGI): https://www.militarytimes.com/air/2019/06/12/the-marine-corps-no-1-priority-for-the-f-35-involves-a-rough-landing-in-hot-environments
- [S21] The Register, F-35B first vertical landing (fan "dam" vs HGI): https://www.theregister.com/2010/03/19/f35b_vertical_landing/
- [S22] The War Zone, F-35B SRVL: https://www.twz.com/13233/f-35b-pilots-will-make-rolling-landings-like-this-to-board-royal-navy-carriers
- [S23] Military & Aerospace Electronics, F-35 panoramic display: https://www.militaryaerospace.com/sensors/article/14276510/panoramic-display-f-35-cockpit-avionics
- [S24] Military Embedded Systems, F-35 Gen III HMDS (biocular 40x30° FOV): https://militaryembedded.com/avionics/displays/f-35-gen-iii-helmet-mounted-display-system-delivered-for-software-integration-to-lockheed-martin
- [S25] Martin-Baker US16E: https://martin-baker.com/ejection-seats/us16e/
- [S26] GD-OTS GAU-22/A datasheet (2024): https://gd-ots.com/wp-content/uploads/2024/09/400001948-GAU-22-A-2024-09-V04.pdf
- [S27] The War Zone, F-35B and F-35C gun pods differ: https://www.twz.com/air/f-35b-and-f-35c-use-different-25mm-gun-pods-unique-to-each-variant
- [S28] The War Zone, Sidekick 6× AIM-120 (A/C only): https://twz.com/adapter-for-f-35-internal-carriage-of-six-aim-120-missiles-is-progressing
- [S29] Northrop Grumman AN/APG-81: https://www.northropgrumman.com/what-we-do/air/an-apg-81-active-electronically-scanned-array-aesa-fire-control-radar
- [S30] Moog aircraft EHA (F-35 sole actuation): https://www.moog.com/products/actuators-servoactuators/actuation-technologies/electrohydrostatic/aircraft-eha.html
- [S31] Virginia Tech (Mason) F-35A student design report (planform, MAC, tails, gear, CD0 friction): https://archive.aoe.vt.edu/mason/Mason_f/F35ASpr11.pdf
- [S32] MigFlug, "Why every fighter jet is painted grey" (FS 36170 on F-35): https://migflug.com/afterburner/?p=290555
- [S33] Defense-Update, "F-35B Demonstrates Hover, STOVL Capability" (2010): https://defense-update.com/20100319_f-35b-demonstrate-stovl.html
- [S34] F-16.net, conversion timing (secondary): https://www.f-16.net/forum/viewtopic.php?p=221190
- [S35] NASA TP-1538 (Nguyen et al. 1979) F-16 data, as in Stevens & Lewis; also `include/fastjet/aero/tp1538_tables.hpp`: https://ntrs.nasa.gov/citations/19800005879
- [S36] National Interest, "What Makes the F-35B So Special" (32,300 lb empty): https://nationalinterest.org/blog/reboot/what-makes-f-35b-so-special-it-lands-helicopter-184174 · [S36b] FlightGlobal, lift-fan inlet door redesign (aft-hinged): https://www.flightglobal.com/lift-fan-inlet-door-problems-led-to-rethink-/74066.article
- [S37] Edwards AFB, F-35 AF-2 (+9 g / −3 g): https://www.edwards.af.mil/News/Display/Article/829156/workhorse-becomes-first-f-35-to-achieve-1000-flight-hours/
- [S38] GlobalSecurity, stealth aircraft RCS estimates: https://www.globalsecurity.org/military/world/stealth-aircraft-rcs.htm
