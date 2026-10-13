# Fast Jet 6-DoF Flight Dynamics Model & RK4 Integrator

A deterministic, zero-allocation, 6-Degrees-of-Freedom (6-DoF) rigid-body Flight Dynamics Model (FDM) and 4th-Order Runge-Kutta (RK4) numerical integrator for an F-16 flight simulator, implemented in modern C++20.

---

## Key Technical Specifications

### 1. Zero-Allocation Architecture
- **Zero dynamic memory allocation (`new` / `malloc`)** in the hot integration loop.
- All state vectors, kinematic derivatives, intermediate RK4 stage buffers ($k_1, k_2, k_3, k_4$), and matrix operators are stack-allocated value types.
- Execution speed: **~5,000,000 steps/second (~200 ns per step)** on standard hardware, offering a **>24,000x real-time margin** over the 200 Hz ($dt = 0.005$ s) requirement.
- Fully decoupled from any rendering or UI loop.

### 2. State Vector & Coordinate Frames
Defined in [`include/fastjet/fdm/flight_state.hpp`](include/fastjet/fdm/flight_state.hpp):
- **Position ($\mathbf{p}_{NED}$)**: North-East-Down Cartesian frame $[x, y, z]^T$ in meters. Altitude is geometric $-z$.
- **Linear Velocity ($\mathbf{v}_b$)**: Body-fixed frame $[u, v, w]^T$ in m/s ($u$: nose, $v$: right wing, $w$: belly).
- **Angular Velocity ($\boldsymbol{\omega}_b$)**: Body-fixed frame $[p, q, r]^T$ in rad/s ($p$: roll, $q$: pitch, $r$: yaw).
- **Attitude Quaternion ($\mathbf{q}$)**: Unit quaternion $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$ adhering to Hamilton convention. Avoids gimbal lock entirely and provides exact Direction Cosine Matrices $C_{b/n}$ and $C_{n/b}$.

### 3. Core Equations of Motion
Defined in [`include/fastjet/fdm/six_dof_fdm.hpp`](include/fastjet/fdm/six_dof_fdm.hpp):
- **Position Kinematics**:
  $$\dot{\mathbf{p}}_{NED} = C_{n/b} \mathbf{v}_b$$
- **Linear Accelerations (Newton's 2nd Law in rotating body frame)**:
  $$\dot{\mathbf{v}}_b = \frac{\mathbf{F}_{ext}^b}{m} + C_{b/n} \mathbf{g}_{NED} - \boldsymbol{\omega}_b \times \mathbf{v}_b$$
- **Angular Accelerations (Euler's Rotational Equations)**:
  $$\dot{\boldsymbol{\omega}}_b = \mathbf{I}^{-1} \left( \mathbf{M}_{ext}^b - \boldsymbol{\omega}_b \times (\mathbf{I} \boldsymbol{\omega}_b) \right)$$
- **Attitude Kinematics**:
  $$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes [0, p, q, r]^T$$

### 4. F-16C Mass & Inertia Properties
Defined in [`include/fastjet/fdm/mass_properties.hpp`](include/fastjet/fdm/mass_properties.hpp) based on NASA / Stevens & Lewis F-16 Block 50 standards:
- **Empty Weight**: $20,500\text{ lbs} = 9,298.6436\text{ kg}$ ($m = 9,298.64\text{ kg}$)
- **Inertia Tensor**:
  $$I_{xx} = 9,496\text{ slug}\cdot\text{ft}^2 = 12,874.85\text{ kg}\cdot\text{m}^2$$
  $$I_{yy} = 55,814\text{ slug}\cdot\text{ft}^2 = 75,673.62\text{ kg}\cdot\text{m}^2$$
  $$I_{zz} = 63,100\text{ slug}\cdot\text{ft}^2 = 85,552.11\text{ kg}\cdot\text{m}^2$$
  $$I_{xz} = 982\text{ slug}\cdot\text{ft}^2 = 1,331.41\text{ kg}\cdot\text{m}^2$$
  $$I_{xy} = I_{yz} = 0$$
- Analytical inverse $\mathbf{I}^{-1}$ is precomputed at initialization, eliminating runtime matrix inversion.

### 5. 1976 US Standard Atmosphere Model
Defined in [`include/fastjet/environment/atmosphere1976.hpp`](include/fastjet/environment/atmosphere1976.hpp):
- Computes temperature $T$, ambient pressure $P$, density $\rho$, local speed of sound $a$, dynamic pressure $\bar{q} = \frac{1}{2}\rho V^2$, and Mach number $M$ up to 86 km.
- Accommodates geometric-to-geopotential altitude conversion ($H = \frac{R_E h}{R_E + h}$).

### 6. NASA TP-1538 Aerodynamics & Multidimensional Interpolation
Defined in [`include/fastjet/aero/`](include/fastjet/aero/):
- **Multidimensional Interpolation** ([`interpolator.hpp`](include/fastjet/aero/interpolator.hpp)):
  High-performance 1D linear, 2D bilinear, and 3D trilinear interpolation with boundary clamping and zero heap allocation.
- **Lookup Tables** ([`tp1538_tables.hpp`](include/fastjet/aero/tp1538_tables.hpp)):
  NASA TP-1538 wind-tunnel lookup tables in contiguous cache-aligned memory:
  - Longitudinal: $C_L(\alpha, \delta_e, \delta_{lef})$, $C_D(\alpha, \delta_e, \delta_{lef})$, $C_m(\alpha, \delta_e, \delta_{lef})$ ($\alpha \in [-90^\circ, 90^\circ]$, $\delta_e \in [-25^\circ, 25^\circ]$, $\delta_{lef} \in [0^\circ, 25^\circ]$). The wind-tunnel data covers $-20^\circ$ to $45^\circ$; the rows beyond carry the jet through the post-stall region to a flat plate at $\pm 90^\circ$ (no lift, $C_D \approx 1.8$, a growing nose-down moment, and a stabilator losing authority in the wing's wake). Past $90^\circ$ the jet is flying tail first: the force stays normal to the wing and the tail-first flow swaps the jet's ends.
  - Lateral-Directional: $C_Y(\beta, \delta_r)$, $C_l(\beta, \delta_a, \delta_r)$, $C_n(\beta, \delta_a, \delta_r)$ ($\beta \in [-30^\circ, 30^\circ]$, $\delta_a \in [-20^\circ, 20^\circ]$, $\delta_r \in [-30^\circ, 30^\circ]$).
  - Dynamic Damping: $C_{m_q}, C_{L_q}, C_{D_q}, C_{l_p}, C_{l_r}, C_{n_p}, C_{n_r}, C_{Y_p}, C_{Y_r}$ as functions of $\alpha$.
- **Force & Moment Synthesizer** ([`f16_aero_model.hpp`](include/fastjet/aero/f16_aero_model.hpp)):
  Synthesizes body-frame forces and moments from dynamic pressure $\bar{q}$, wing area $S = 300\text{ ft}^2$ ($27.87\text{ m}^2$), chord $\bar{c} = 11.32\text{ ft}$ ($3.45\text{ m}$), and wingspan $b = 30.0\text{ ft}$ ($9.14\text{ m}$):
  $$\mathbf{F}_{aero}^b = \bar{q} S \begin{bmatrix} C_L \sin\alpha - C_D \cos\alpha \\ C_Y \\ -C_L \cos\alpha - C_D \sin\alpha \end{bmatrix}, \quad \mathbf{M}_{aero}^b = \bar{q} S \begin{bmatrix} b \, C_l \\ \bar{c} \, C_m \\ b \, C_n \end{bmatrix}$$
- **Relaxed Longitudinal Static Stability (Open-Loop Instability)**:
  Correctly models positive pitch stiffness ($\frac{\partial C_m}{\partial \alpha} > 0$) at subsonic cruise. Under open-loop fixed controls without FLCS, any pitch perturbation produces exponential pitch divergence.

#### Compressibility & Speedbrake Corrections
The TP-1538 tables are low-speed wind-tunnel data carrying no Mach dependence. These are extended across the transonic and supersonic range the airframe can actually reach:
- **Prandtl-Glauert scaling** $1/\sqrt{|1 - M^2|}$ applied to sideforce and moments, capped below $M = 1$ to avoid the singularity and switching to the Ackeret form supersonically. Lift goes through [`compressible_lift.hpp`](include/fastjet/aero/compressible_lift.hpp), shared with the other four airframes:
  - the Prandtl-Glauert gain applies only where the flow is attached, fading out between $12^\circ$ and $30^\circ$ of alpha;
  - the maximum usable lift coefficient falls with Mach (shock-induced separation): about 3/4 of its low-speed value at M0.9 and 3/5 at M1.2;
  - drag due to lift follows the lift actually made, $\propto C_L^2$, and grows through the transonic region as leading-edge suction is lost (about 1.8 times the low-speed value by M1.05).

  Scaling lift alone made every jet more efficient as it neared Mach 1 (L/D at a fixed alpha rose from about 7 at M0.3 to 11 at M0.85), and fighters sustained 8–9 G at 30,000 ft.
- **Supersonic stability**: the aerodynamic centre moves aft through the transonic region, adding $\Delta C_m = -0.12\,C_L$ by M1.2. The table's relaxed stability is a subsonic property.
- **Wave drag** rising from a critical Mach of 0.86 through a transonic peak of $\Delta C_D = 0.026$ at $M = 1.05$, decaying to a supersonic plateau of 0.017.
- **Mach tuck**: the aft aerodynamic-centre shift contributes $\Delta C_m = -0.030$ nose-down through the transonic region.
- **Speedbrake**: $\Delta C_D = +0.025$ at full extension, linear in position, with a small nose-up pitch increment.
- **Supersonic drag rise**: beyond the transonic peak, $C_{D,\text{wave}}$ grows again as $0.006\,(M - M_{\text{peak}})^2$ rather than settling to a flat plateau.
- **Dynamic-pressure drag**: $\Delta C_D = 0.095\,\left(\frac{q - q_0}{q_0}\right)^2$ for $q > q_0 = 60\ \text{kPa}$.

  The F-16's low-altitude speed limit is set by dynamic pressure, not Mach: the placarded 800 KEAS
  corresponds to $q \approx 102$ kPa, reached near Mach 1.2 on the deck but not until beyond Mach 2
  at altitude. A drag term depending only on Mach cannot reproduce both ends of the envelope, since
  it penalises high altitude — where the real aircraft is fastest — just as hard as sea level.
  This term represents losses that scale with absolute air loading: inlet spillage and bleed, hinge
  moments, and skin friction on a heated boundary layer.

##### Validated performance

| Condition | Simulated | Real F-16C |
|---|---|---|
| Max level speed, 11,000 m | Mach 1.93 | Mach 2.05 |
| Max level speed, sea level | Mach 1.13 ($q$ = 90 kPa) | Mach ~1.05, $q$ limit 102 kPa |
| Max dry (MIL) speed, 11,000 m | Mach 1.0 | subsonic–low supersonic |
| Military thrust, SLS | 75.6 kN | 76.3 kN (17,155 lbf) |
| Max afterburner, SLS | 129.0 kN | 131.6 kN (29,588 lbf) |
| Thrust-to-weight, combat | 1.17 | ~1.10 |

These are pinned by `test_performance_envelope.cpp`, which flies the full 6-DoF model with an
altitude-hold autopilot rather than asserting on constants.

Turn performance, with half fuel and the combat payload (pinned by `test_flight_realism.cpp`):

| F-16C | Simulated | Published |
|---|---|---|
| Sustained turn, sea level, M0.6 | 8.5 G, 23°/s | ~20–21°/s |
| Sustained turn, 15,000 ft, M0.8 | 7.6 G, 16.5°/s | ~15°/s |
| Sustained turn, 30,000 ft, M0.9 | 3.6 G, 7.2°/s | 3.5–4.5 G |
| Maximum lift coefficient, M0.3 → M0.9 | 1.51 → 1.18 | falls with Mach |

The other four airframes use synthesised polars ([`aircraft_aero_model.hpp`](include/fastjet/aero/aircraft_aero_model.hpp)) with the same compressibility treatment and a per-airframe stall: lift rounds off to $C_{L,max}$, holds, then falls away to a flat plate. Drag due to lift is $K\,C_{L_\alpha}\,C_L \tan\alpha$, which equals the polar's $K C_L^2$ on the lift line but keeps climbing with alpha once the lift is capped. The straight-winged A-10 stalls more sharply than the delta and chined wings of the Typhoon and F-22, whose vortex lift holds to higher alpha.

All corrections default to `mach = 0`, which reproduces the original incompressible table data bit-for-bit.

### 7. Digital Fly-By-Wire Flight Control System (FLCS) & Actuators
Defined in [`include/fastjet/flcs/`](include/fastjet/flcs/):
- **Hydraulic Actuators** ([`actuator.hpp`](include/fastjet/flcs/actuator.hpp)):
  - First-order lag dynamics ($\dot{\delta} = \frac{1}{\tau}(\delta_{cmd} - \delta)$).
  - Stabilator: $\tau = 0.05$ s, rate limit = $60^\circ/\text{s}$, limits = $[-25^\circ, +25^\circ]$.
  - Flaperon: $\tau = 0.05$ s, rate limit = $52^\circ/\text{s}$, limits = $[-20^\circ, +20^\circ]$.
  - Rudder: $\tau = 0.05$ s, rate limit = $120^\circ/\text{s}$, limits = $[-30^\circ, +30^\circ]$.
  - Automatic Leading-Edge Flap (LEF) schedule vs $\alpha$ and $\bar{q}$.
- **Pitch Axis Control Law** ([`pitch_controller.hpp`](include/fastjet/flcs/pitch_controller.hpp)):
  - Blends pitch rate ($q_{cmd}$) at low dynamic pressure ($\bar{q} \le 2000$ Pa) and normal acceleration ($N_{z,cmd}$) at high dynamic pressure ($\bar{q} \ge 8000$ Pa).
  - Hard G-limiter strictly clamped to $[-3.0\text{ G}, +9.0\text{ G}]$. Full aft stick settles at the limit wherever the wing can make it, and anything past it is pushed back.
  - Hard Angle-of-Attack limiter actively overrides pitch-up command to cap $\alpha \le 25.5^\circ$, and a negative limit at $-12^\circ$.
  - Closed-loop PI controller with pitch rate damping and integrator anti-windup, gain-scheduled down above 30 kPa of dynamic pressure. Unscheduled, a full pull at Mach 1.2 on the deck diverged to about 50 G.
  - Rolling-pull G relief: the limit drops by 1.8 G at full roll rate and the integrator holds while rolling, as on the real jet. Integrating through a full-rate roll wound up trim that fired the jet past 11 G when the roll stopped.
  - The F-15EX, Typhoon and F-22 share one G-command law ([`aircraft_flcs.hpp`](include/fastjet/flcs/aircraft_flcs.hpp)) with per-airframe gains and the same protections; the A-10's feel system limits a push to about $-3$ G.
- **Roll & Yaw Axes Control Laws** ([`lateral_directional_controller.hpp`](include/fastjet/flcs/lateral_directional_controller.hpp)):
  - Roll-rate command up to $300^\circ/\text{s}$ with automatic roll attenuation at high AoA ($\alpha \in [15^\circ, 25.5^\circ]$).
  - Aileron-Rudder Interconnect (ARI) providing turn coordination and eliminating adverse yaw.
  - Yaw rate damper and sideslip feedback for Dutch roll suppression.

### 8. Propulsion, Fuel System & Landing Gear
Defined in [`include/fastjet/propulsion/`](include/fastjet/propulsion/), [`include/fastjet/fdm/fuel_system.hpp`](include/fastjet/fdm/fuel_system.hpp) and [`include/fastjet/gear/`](include/fastjet/gear/):
- **F110-GE-129 Engine Model** ([`f110_engine.hpp`](include/fastjet/propulsion/f110_engine.hpp)):
  - Detent schedule (Cutoff / Idle / Military / Afterburner) driving sea-level static ratings of 4.45 kN idle, 75.6 kN military, and 129 kN full reheat.
  - **Installed thrust lapse** with density ratio and Mach: $F = F_{SL} \cdot \sigma^{n} (1 + k_{ram} M^2)$, where $n = 0.85$ dry and $1.00$ in reheat. Thrust falls from 75.5 kN at sea level to 23.6 kN at 12 km.
  - **Spool dynamics** as a first-order lag with direction-dependent time constants ($\tau_{up} = 3.2$ s, $\tau_{down} = 1.6$ s, $\tau_{AB} = 0.6$ s), integrated by the exact discrete solution so the response is stable at any timestep. In the flyable jets ([`multi_engine.hpp`](include/fastjet/propulsion/multi_engine.hpp)) the core and the afterburner are separate: the afterburner lights only once the core reaches 80% of military thrust, so idle to full reheat takes about 6 s in the F-16 (4 s in the F-22), not 0.6 s.
  - **Fuel burn** via thrust-specific fuel consumption (0.75 lb/lbf/hr dry, 2.00 wet) with flame-out when the tanks run dry.
- **Variable Mass & Inertia** ([`fuel_system.hpp`](include/fastjet/fdm/fuel_system.hpp)):
  - Gross mass and the full inertia tensor are rebuilt every timestep from the fuel remaining. A full internal load of 3,175 kg raises $I_{yy}$ by 34%, measurably reducing pitch agility.
  - Every jet also carries a fixed combat payload: pilot, full gun ammunition and a typical air-to-air loadout with its launchers (F-16 800 kg, F-15EX 1,300, Typhoon 1,100, F-22 1,500, A-10 1,500, F-35B 1,050 with its 220-round gun pod).
- **Three-Point Landing Gear** ([`landing_gear.hpp`](include/fastjet/gear/landing_gear.hpp)):
  - Oleo-pneumatic struts modelled as spring-dampers, sized so the static stance carries **12% on the nose and 88% on the mains** — matching the real aircraft's CG position — with the reaction balancing weight to within 0.01%.
  - **Tyre cornering-stiffness** side-force model (force proportional to slip angle, saturating at the friction limit) rather than pure Coulomb friction, which is what makes ground handling stable and self-centring.
  - Lateral friction is capped at $\mu = 0.65$, deliberately below the geometric rollover threshold ($b/h = 1.13/1.59 = 0.71$), so the aircraft slides before it tips.
  - **Speed-scheduled nosewheel steering** tapering as $1/V^2$ above taxi speed, holding commanded lateral acceleration within the rollover limit.
  - Differential toe braking ($\mu = 0.55$), and structural failure above a 4.5 m/s sink rate.

### 9. Controller Input Processing Subsystem
Defined in [`include/fastjet/input/`](include/fastjet/input/):
- **Direct Hardware Polling** ([`joystick_driver.hpp`](include/fastjet/input/joystick_driver.hpp)):
  - SDL3 direct hardware polling backend (`SDL3InputDriver`) reading multi-axis analog controllers and hats/buttons with microsecond-level latency (~5 µs hardware polling, ~35 ns end-to-end update latency).
  - Mock driver (`VirtualHOTASDriver`) for automated deterministic testing and headless simulation.
- **Signal Conditioning** ([`signal_conditioner.hpp`](include/fastjet/input/signal_conditioner.hpp)):
  - Per-axis min/center/max calibration offsets and inversion toggles.
  - Configurable inner deadbands (suppressing center stick jitter/drift) and outer deadbands (guaranteeing 100% full-throw command at physical endstops).
- **Force-Sensing Side-Stick Simulation Curves**:
  - Emulates the rigid force-sensing feel of the real F-16 transducer side-stick on displacement desktop joysticks using the selectable polynomial transfer function:
    $$y = (1 - c)x + c x^3, \quad c \in [0.0, 1.0]$$
  - Tunable curvature $c$ provides ultra-precise micro-corrections around center stick while preserving full command authority at outer deflection.
- **Avionics Flight Controls** ([`avionics_controls.hpp`](include/fastjet/input/avionics_controls.hpp)):
  - 4-way digital pitch and roll trim hat with smooth rate slew ($0.05/\text{s}$) and manual zeroing.
  - Multi-detent throttle quadrant controller mapping raw throttle to Cutoff, Ground/Flight Idle, Military Power, and Afterburner (threshold at $0.85$).
  - Speedbrake deployment switch (Retracted, 50%, Extended) with physical hydraulic deployment slew ($60\%/\text{s}$).
  - Differential left/right toe brakes.
- **Configuration Persistence** ([`input_config.hpp`](include/fastjet/input/input_config.hpp)):
  - Structured JSON serialization and deserialization for all axes, deadbands, curvature $c$, and button/hat mappings without external dependencies.

### 10. First-Person Graphics, Collimated HUD & Cockpit Avionics
Defined in [`include/fastjet/graphics/`](include/fastjet/graphics/):
- **Camera Rig & G-Induced Head Dynamics** ([`camera_rig.hpp`](include/fastjet/graphics/camera_rig.hpp)):
  - First-person camera positioned at the true F-16 cockpit Design Eye Point (DEP: $[+1.80, 0.00, -0.65]\text{ m}$).
  - 2nd-order critically damped spring-damper ($\omega_n = 25\text{ rad/s}, \zeta = 0.85$) simulating instantaneous cockpit G-load head displacement (downward compression under $+9\text{G}$, upward under $-3\text{G}$, lateral tilt under $N_y$).
- **Optical-Infinity Collimated HUD** ([`hud_collimator.hpp`](include/fastjet/graphics/hud_collimator.hpp)):
  - Projects symbology at optical infinity ($R \to \infty$) so reticles remain horizon-locked without parallax as the pilot's head translates inside the cockpit.
  - Hardware Stencil Buffer clipping constrained to the physical 3D combiner glass polygon so symbology is clipped outside the glass viewing aperture.
  - Flight Path Marker (FPM) tracking true inertial velocity vector angles $(\gamma, \chi)$ driven by $(\theta - \alpha)$ vertically and $(\psi + \beta)$ horizontally.
  - Conformal pitch ladder, heading tape, Calibrated Airspeed (CAS) box, barometric altitude box, and digital G-meter with peak G memory.
- **Flight Instruments (Render-to-Texture)** ([`flight_instruments.hpp`](include/fastjet/graphics/flight_instruments.hpp)):
  - Off-screen $512 \times 512$ Framebuffer Object (FBO) dynamically rendering the three-light AoA indexer (slow chevron, on-speed donut, fast chevron), Attitude Director Indicator (ADI) ball with sky/ground split, and Mach number tape.
  - Mapped directly as a material texture onto the cockpit dashboard MFD panel.
- **Live Cockpit Gauges** ([`cockpit_gauges.hpp`](include/fastjet/graphics/cockpit_gauges.hpp)):
  - Every painted instrument in the asset cockpit has a working face laid 1.5 mm over it: airspeed/Mach, counter-pointer altimeter (with the below-10,000 ft crosshatch), ADI, HSI, AOA and VVI tapes, fuel quantity and totalizer, fuel flow, oil, nozzle, RPM, FTIT, cabin altitude, hydraulic A/B and the clock.
  - The DED shows a CNI page with the local time; the eyebrow warning lamps (ENG FIRE, ENGINE, HYD/OIL PRESS, FLCS, TO/LDG CONFIG) and the three gear-down lamps follow the aircraft's state.
  - The HSI shows the runway course with centreline deviation, and a TACAN bearing pointer and DME to the field.
  - Pointers move through first-order lags, as the mechanical instruments do: engine gauges follow the spool, and the VVI trails the climb rate.
  - Placements were measured by mapping each painted dial's texture rectangle back through the cockpit mesh ([`ModelGLB::cockpit_gauges()`](include/fastjet/graphics/model_glb.hpp)). Round dials leave their corners transparent, so the painted bezels and screws still show.
- **Atmosphere & Environment** ([`sky_ground_renderer.hpp`](include/fastjet/graphics/sky_ground_renderer.hpp)):
  - Analytic single-scattering sky: wavelength-dependent Rayleigh ($\beta \propto \lambda^{-4}$) plus Henyey-Greenstein Mie aerosol scattering, with a sun disc and aureole.
  - Radiance along a view ray, where $\tau$ is the vertical optical depth and $m$ the air mass:

$$L = \left(\tau_R\,\Phi_R(\theta) + \tau_M\,\Phi_M(\theta)\right) m \cdot \frac{1 - e^{-\tau m}}{\tau m}$$

  - The trailing factor is the mean self-attenuation along the path; without it the horizon saturates to white.
  - Sky darkens and deepens with altitude via $\rho = e^{-h/H}$, $H = 8{,}500\text{ m}$.
  - **The terrain haze and the sky are the same function.** Aerial perspective fades ground into the radiance the sky quad would draw along that exact ray, and converges on the sky's own air mass in the far field, so the horizon has no seam by construction.
  - Aerosol haze is integrated through the boundary layer ($H_{aer} = 1{,}200\text{ m}$) rather than applied per unit distance, so the ground stays crisp from altitude but washes out along the deck.
- **Procedural Terrain** ([`terrain_field.hpp`](include/fastjet/graphics/terrain_field.hpp), [`terrain_mesh.hpp`](include/fastjet/graphics/terrain_mesh.hpp)):
  - Ridged fractional Brownian motion over a hash-based value-noise lattice: 5 octaves, non-integer lacunarity 2.13 to avoid axis-aligned repeats. Peaks reach $620\text{ m}$.
  - A pure function of position, so the mesh builder and any future collision query cannot disagree; identical on every machine and every run.
  - The airfield sits in a guaranteed-flat basin ($r = 2{,}600\text{ m}$, blending out over $5{,}200\text{ m}$) so takeoff and landing stay predictable.
  - Four concentric LOD rings ($125\text{ m}$ to $3{,}200\text{ m}$ spacing) reaching $91\text{ km}$: ~27k triangles total, built once at init, zero per-frame allocation.
  - Shaded by elevation and slope (lowland green, dry upland, exposed rock, snow only on high shallow ground) with sun and sky-ambient lighting in linear space.
  - $3,000\text{ m} \times 60\text{ m}$ runway with centerline, edge lines, threshold piano keys, touchdown-zone bars, and aiming-point blocks. The terrain mesh cuts a hole for the airfield rather than z-fighting decals that sit $20\text{ cm}$ above it.
- **Rendering Pipeline** ([`render_engine.hpp`](include/fastjet/graphics/render_engine.hpp), [`hdr_pipeline.hpp`](include/fastjet/graphics/hdr_pipeline.hpp)):
  - The world renders as linear radiance into a multisampled `R11F_G11F_B10F` target, then gets energy-conserving bloom (13-tap downsample, tent upsample), the ACES filmic curve, lens vignette and G-LOC greyout in one composite pass. The cockpit, HUD and menus are drawn afterwards with the same curve, so they stay crisp.
  - Sun colour and sky ambient come from a CPU mirror of the sky shader ([`scene_lighting.hpp`](include/fastjet/graphics/scene_lighting.hpp)), so every surface is lit by the sky it sits under. All shaders share one GLSL library ([`shader_library.hpp`](include/fastjet/graphics/shader_library.hpp)).
- **Airframe and Cockpit** ([`model_glb.hpp`](include/fastjet/graphics/model_glb.hpp)):
  - glTF metallic-roughness shading: sRGB base colour, packed roughness/metal and tangent-space normal maps, a GGX sun term, and reflections of the real sky and ground. The gold-tinted canopy uses dual-source blending to reflect and tint at once.
  - The cockpit view flies from the asset's textured cockpit. The live MFD pages are mapped onto its two display screens, and the HUD is masked by its combiner glass. [`cockpit_geometry.hpp`](include/fastjet/graphics/cockpit_geometry.hpp) remains as the fallback.
  - The asset is authored Y-up and converted to body axes at load. Each view seats it where it matters: wheels on the gear contact points in the chase view, canopy and HUD around the design eye point in the cockpit.
- **Shadows, Clouds and Effects**:
  - Sun shadow map fitted tightly around the aircraft ([`shadow_map.hpp`](include/fastjet/graphics/shadow_map.hpp)), texel-snapped and in origin-relative coordinates. It gives self-shadowing, the jet's shadow on the runway (fading with height), and canopy-frame shadows across the cockpit.
  - Scattered cumulus deck with cloud shadows ([`cloud_layer.hpp`](include/fastjet/graphics/cloud_layer.hpp)), built from one wind-drifted field baked to a map in slices across frames; cirrus is a baked tiling texture in the sky. The afterburner plume has shock diamonds ([`exhaust_plume.hpp`](include/fastjet/graphics/exhaust_plume.hpp)).
  - The ground material (strip farmland with hedgerows, woods, lakes, rock, snow, mown infield) is baked once into three nested world-space textures ([`terrain_textures.hpp`](include/fastjet/graphics/terrain_textures.hpp)). Pavement, rubber deposits, worn paint and emissive runway lights are shaded procedurally.
  - The settings quality preset ([`render_quality.hpp`](include/fastjet/graphics/render_quality.hpp)) sets scene MSAA, shadow resolution, bloom, cirrus and exhaust, and applies without a restart.

### 11. Threading and Frame Pacing

The frame itself is GPU-bound: physics costs about 3 µs per 200 Hz step and the CPU submits a frame in under 1 ms, so the render loop stays on one thread (the GL context's) and the other cores take the work that would otherwise stall it.

- **Worker pool** ([`thread_pool.hpp`](include/fastjet/core/thread_pool.hpp)): one worker per hardware thread beyond the main one. `parallel_for` has the caller work through chunks as well, so it cannot deadlock, even nested or with every worker busy.
- **Loading**: the airframe file is read and its three 4096² textures decoded in parallel on the workers, while the main thread builds the terrain, whose mesh and texture bakes are themselves sampled across the pool. Results are bit-identical to a serial build. Engine start-up is about 2.3× faster (1.12 s to 0.49 s on a 2-core/4-thread laptop).
- **Audio** ([`audio_engine.hpp`](include/fastjet/audio/audio_engine.hpp)): samples are mixed on SDL's audio thread, so a long frame can no longer starve the device; the main thread only sets parameters, under SDL's stream lock.
- **Frame pacing** ([`frame_pacer.hpp`](include/fastjet/graphics/frame_pacer.hpp)): a fence after each swap keeps the loop at most two frames ahead of the GPU. Without it a GPU-bound loop ran dozens of frames ahead and then stalled, so measured frame times swung between 0.5 ms and 100 ms; with it they hold within about 1 ms of the GPU's own rate, and input is sampled close to when it is drawn.
- **GPU savings with unchanged images** (verified by pixel diff): the cockpit interior is drawn before the hull so early-Z rejects the hidden hull; the display atlases have no unused depth buffer, the MFD atlas is sized to the screens the asset cockpit has, and neither redraws in the chase view. At 1080p Ultra that is about +13% frame rate in both views.
- **`--profile`** prints the frame-time distribution, CPU time per loop stage and GPU time per render pass (timestamp queries that never stall, [`gpu_profiler.hpp`](include/fastjet/graphics/gpu_profiler.hpp)) every two seconds.

### 12. Evade Mode: Surviving a Missile Shot

From the main menu (**EVADE**) or `--evade [easy|medium|hard|expert]`. The menu takes two steps: pick the threat level (each shows the bandit, his weapons, his head start and how long you must survive), then your jet (it opens on the one you're flying; the bandit flies the same type), then **START EVADE**. A bandit starts in your six and hunts you with radar missiles and his gun. You win by outlasting his fuel, by leaving him with nothing to shoot, by getting away, or by shooting him down. Every level flies the same physics. Difficulty changes the pilot, his missile load, his launch discipline and his head start ([`engagement.hpp`](include/fastjet/sim/engagement.hpp)):

| Level | Bandit | Weapons | Start | Shoots when |
|---|---|---|---|---|
| EASY | Novice | Gun | 3 nm astern, co-speed | In gun range |
| MEDIUM | Veteran | 2 × AIM-120 + gun | 10 nm, +300 m, +20 m/s | The missile can reach you if you hold course (Rmax), never from inside 3 km |
| HARD | Ace | 4 × AIM-120 + gun | 6.5 nm, +600 m, +40 m/s | You can't outrun it even turning cold in reheat (no-escape zone) |
| EXPERT | Ace | 4 × AIM-120, two per shot | 6 nm, +900 m, +60 m/s | No-escape, two-missile salvo |

- **Missile** ([`missile.hpp`](include/fastjet/sim/missile.hpp)): an AIM-120C-class point mass with a 5 s boost to Mach 3+, Mach-dependent drag, induced drag from the G it pulls, a 40 G airframe limit and a lift limit from dynamic pressure (a slow missile can't turn), a 0.2 s autopilot lag and proportional navigation. Guidance goes datalink → inertial → active seeker, which searches a basket around the predicted target. It has a proximity fuze and a blast-fragmentation warhead that feeds the component damage model.
- **Radar and RWR** ([`radar.hpp`](include/fastjet/sim/radar.hpp)): the bandit's radar searches, locks after the operator's acquisition time, coasts on memory through dropouts and loses lock outside its gimbal or range. Radars and seekers are pulse-doppler, so a target flying 90° to the line of sight with ground behind it sits in the clutter **notch** and disappears. Chaff blooms into the tracker's range, angle and doppler gates, so it only works when you are beaming. Your RWR hears the search, the lock and the launch. The A-10 has no air-to-air radar.
- **Launch decisions** ([`missile_shooter.hpp`](include/fastjet/sim/missile_shooter.hpp)) fly the missile's own dynamics and guidance forward from the radar track, so a shot is only taken when the missile can get there. The test requires the prediction to match the live missile exactly.
- **Cockpit**: the RWR azimuth scope on the upper-left panel shows the emitter ("16"), a lock diamond and a flashing "M" for an inbound missile. The HUD shows `LOCK 6` / `LAUNCH 6` (clock bearing) and the chaff count. The headset carries the RWR tones: three chirps for a new emitter, a steady beep-beep while you are locked, and a fast warble while a missile is guided at you.
- **What works is what works in the jet**: fly straight and you die at every level. Running in reheat drags a max-range shot out of energy but not a no-escape shot. Against those, beam the missile (put it at 3 or 9 o'clock), get low so there's ground behind you, chaff while beaming, and break hard across its line of sight in the last second or two. [`evasion_pilot.hpp`](include/fastjet/sim/evasion_pilot.hpp) flies exactly that from the RWR alone (`--evade hard --watch`). Across 24 seeds it survives the EASY/MEDIUM/HARD/EXPERT bandit 24/16/1/0 times.

### 13. Fire-Control Radar (FCR Page)

The left MFD shows the F-16's APG-68-class radar page ([`radar_scope.hpp`](include/fastjet/sim/radar_scope.hpp), drawn in [`flight_instruments.hpp`](include/fastjet/graphics/flight_instruments.hpp)), in every mode and for every jet except the A-10, which has no air-to-air radar. It is a B-scope: azimuth across (±60°), range up from your jet at the bottom edge.

- **RWS search** sweeps a beam stabilised against roll and pitch through a 4-bar pattern at 65°/s. `O` picks the azimuth width, centred on the nose: `A6` (±60°, one full scan in about 7.4 s), `A3` (±30°, 3.7 s) or `A1` (±10°, 1.2 s). Narrow scans show their edges as dashed lines. A jet shows as a brick only when the beam crosses it and the radar can actually see it. It has to be inside the scan volume and detection range, and outside the doppler notch: a jet beaming you with ground behind it vanishes, as it does from the bandit's radar. Bricks stay where the jet was painted and fade over three frames. RWS tells you nothing about heading, speed or height.
- **Around the scope**: the OSB labels (`CRM`/`RWS`, range scale with its arrows, `A6`, `4B`), an artificial horizon, and the antenna azimuth and elevation carets. The acquisition cursor sits on the nearest newest brick, and the two numbers beside it are the top and bottom of the altitude block the scan covers at that range, in thousands of feet. A jet above or below that block is invisible until you move the antenna with `=` / `-`.
- **STT**: `L` locks the brick under the cursor through the jet's own radar, so the other jet's RWR hears the lock. The target gets a filled square with a velocity vector and its altitude. The top line shows aspect (`15L`: 150° off the target's tail, you on its left), its track, its ground speed and the closure in knots. The range scale follows the target. Through a notch the lock coasts on memory and the symbol flashes. If the lock breaks, the radar goes back to RWS and waits for you to lock again. If the jet has left the radar's view by the time you designate its brick, the lock fails and the console says so.
- **ACM (dogfight)**: RWS is a long-range tool. Its 4 bars cover only about 10° of elevation, so a bandit a mile away, above or below the nose in a turning fight, is never painted. `K` switches to air combat mode. It uses fast scans referenced to the nose and locks the first jet it finds inside 10 nm, with no designation: `20` (20×30°, the HUD field of view), `60` (10×60°, a vertical slice from 10° below the nose to 50° above it, for a bandit up in your lift vector) and `BORE` (a fixed beam down the nose). `O` cycles the three patterns. The scope shows the scan's azimuth corridor and its elevation band on the elevation scale. `L` drops a lock and the scan starts again; `K` returns to CRM and your RWS range scale.
- **Datalink picture**: the radar sees only what is inside its ±60° cone and its current scan, which in a turning fight is rarely the bandit. Over every mode the page also draws the Link 16 picture: each other jet is a red hollow diamond at its bearing and range, with a line along its track and its altitude in thousands of feet. When the radar locks it, the green radar symbol sits inside the diamond. A jet off the side of the scope (abeam or behind you) is a red arrowhead pinned to that side at its range. One beyond the range scale is an arrowhead on the top edge.
- **Sweep display** (right MFD during a dogfight or evade, [`sweep_scope.hpp`](include/fastjet/sim/sweep_scope.hpp)): a heading-up 360° scope with you in the middle. A beam turns once every 2.4 s. Each jet it crosses lights up as a red blip, with its track and altitude, and fades until the next pass paints it again. Blips hold where the jet was when painted. The scale (1, 2, 5, 10, 20 or 40 nm) fits the nearest jet. Dashed lines mark the FCR's ±60° cone, and the bottom line calls the nearest jet by clock position, range and altitude. A fighter's own radar only scans the sector ahead, so this is the all-round picture an AWACS would send over the datalink, drawn the way a search radar draws it.

### 14. Limits of the Airframe

- **Over-G** ([`damage_model.hpp`](include/fastjet/sim/damage_model.hpp)): each jet's structure is rated for its G limit (+9/−3 for the fighters, +7.33/−3 for the A-10). Past 1.03 times the limit the jet is logged as overstressed. Past 1.2 times the wings take a permanent set: less roll authority, a weaker structure and a red `WING OVERSTRESS` caution. At 1.5 times the airframe fails (`STRUCTURE FAILED`, a loss marked `(OVER-G)` in the debrief). The load is filtered over 50 ms, so a single integration step's spike does not count, and it is the air load only, not the landing gear. The fly-by-wire jets stay inside their limits by themselves; the mechanically controlled A-10 does not.
- **Pulling G takes effort**: from the keyboard a press of the pull key is a firm ~6 G pull. Hold it and the stick walks on to the stop over about a second, reaching the full 9 G. A hardware stick has the same shape through its force curve (80% deflection is about 6 G). The cockpit `OVER-G WARNING` lights within 3% of the limit, and G-LOC waits for anyone who holds 8–9 G too long.
- **Stall**: every jet can stall. Past the stall, lift falls away and drag climbs towards a flat plate's; the flight control system's AoA limiter keeps a fly-by-wire jet out of it, and the A-10 needs the pilot to.

---

## Directory Structure

```
.
├── CMakeLists.txt
├── LICENSE
├── Makefile
├── README.md
├── include/
│   └── fastjet/
│       ├── aero/
│       │   ├── control_surfaces.hpp  # F-16 control surface deflections and limits
│       │   ├── compressible_lift.hpp # Mach effects on lift, CLmax and drag due to lift
│       │   ├── f16_aero_model.hpp    # Total force & moment synthesizer
│       │   ├── interpolator.hpp      # Zero-allocation 1D/2D/3D interpolators
│       │   └── tp1538_tables.hpp     # NASA TP-1538 wind-tunnel lookup tables
│       ├── core/
│       │   └── thread_pool.hpp       # Worker pool: parallel_for and background jobs
│       ├── environment/
│       │   └── atmosphere1976.hpp    # 1976 US Standard Atmosphere
│       ├── fdm/
│       │   ├── aircraft_forces.hpp   # Body-frame forces & moments interface
│       │   ├── flight_state.hpp      # FlightState & FlightStateDeriv structs
│       │   ├── fuel_system.hpp       # Variable gross mass & inertia from fuel state
│       │   ├── mass_properties.hpp   # F-16C mass & precomputed inertia inverse
│       │   ├── rk4_integrator.hpp    # Deterministic zero-allocation RK4 integrator
│       │   └── six_dof_fdm.hpp       # Core 6-DoF equations of motion
│       ├── gear/
│       │   └── landing_gear.hpp      # Three-point struts, tyre friction, steering
│       ├── propulsion/
│       │   └── f110_engine.hpp       # Thrust lapse, spool dynamics, fuel burn
│       ├── flcs/
│       │   ├── actuator.hpp          # Hydraulic actuator dynamics (lag, rate limits)
│       │   ├── f16_flcs.hpp          # Integrated FBW Flight Control System
│       │   ├── imu_sensor.hpp        # Synthetic IMU sensor model (Nz, Ny, rates)
│       │   ├── lateral_directional_controller.hpp # Roll-rate, ARI, yaw damper
│       │   ├── pilot_commands.hpp    # Cockpit inceptor command inputs
│       │   └── pitch_controller.hpp  # Blended q/Nz, +9G & 25.5 AoA limiters
│       ├── graphics/
│       │   ├── camera_rig.hpp        # F-16 DEP camera rig with G-load spring-damper
│       │   ├── cloud_layer.hpp       # Cumulus deck and its baked shadow map
│       │   ├── cockpit_gauges.hpp    # Live faces for the cockpit's dials, DED and lamps
│       │   ├── cockpit_geometry.hpp  # Fallback cockpit shell, MFD screens, combiner glass
│       │   ├── exhaust_plume.hpp     # Afterburner plume and nozzle glow
│       │   ├── flight_instruments.hpp# FBO render-to-texture AoA indexer, ADI, Mach tape
│       │   ├── frame_context.hpp     # Per-frame lighting, clouds and shadow for all passes
│       │   ├── frame_pacer.hpp       # Fence-based bound on frames queued ahead of the GPU
│       │   ├── gl_common.hpp         # Mat4, Color4, and OpenGL utilities
│       │   ├── gpu_profiler.hpp      # Per-pass GPU timing (--profile)
│       │   ├── hdr_pipeline.hpp      # HDR target, bloom, ACES composite, G-LOC overlay
│       │   ├── hud_collimator.hpp    # Optical-infinity collimated HUD and FPM
│       │   ├── instrument_canvas.hpp # Off-screen 2D vector/text canvas for the displays
│       │   ├── model_glb.hpp         # glTF airframe/cockpit loader and PBR renderer
│       │   ├── render_engine.hpp     # Master multi-pass graphics rendering engine
│       │   ├── render_quality.hpp    # Quality presets and weather state
│       │   ├── scene_lighting.hpp    # CPU scattering mirror: sun colour, sky ambient
│       │   ├── shader.hpp            # GLSL compilation and uniform bindings
│       │   ├── shader_library.hpp    # Shared GLSL: noise, tonemap, atmosphere, BRDF
│       │   ├── shadow_map.hpp        # Sun shadow map fitted around the aircraft
│       │   ├── sky_ground_renderer.hpp# Scattering sky, aerial perspective, airfield
│       │   ├── terrain_field.hpp     # Deterministic ridged-fBm procedural heightfield
│       │   ├── terrain_mesh.hpp      # Concentric LOD ring mesh built from the field
│       │   └── terrain_textures.hpp  # Ground material baked into nested textures
│       ├── input/
│       │   ├── avionics_controls.hpp # Digital trim hat, throttle detents, speedbrakes
│       │   ├── input_config.hpp      # Controller profiles & pure C++20 JSON persistence
│       │   ├── input_manager.hpp     # End-to-end input processing pipeline
│       │   ├── joystick_driver.hpp   # SDL3 hardware poller & VirtualHOTASDriver
│       │   └── signal_conditioner.hpp# Deadbands, calibration, & force-sensing curves
│       └── math/
│           ├── matrix3x3.hpp         # 3x3 matrix arithmetic and inversion
│           ├── quaternion.hpp        # Unit quaternion with DCM rotations
│           └── vector3.hpp           # 3D vector arithmetic and cross/dot products
├── src/
│   └── f16_sim_viewer.cpp            # Interactive real-time 3D flight sim demo
└── tests/
    ├── test_actuators.cpp            # Actuator lag, rate limits, and deflection bounds
    ├── test_aero_coefficients.cpp    # Validates NASA TP-1538 lookups and dCm/dalpha > 0
    ├── test_atmosphere.cpp           # Atmosphere verification vs NOAA 1976 tables
    ├── test_avionics_controls.cpp    # Digital trim hat, throttle detents, speedbrakes
    ├── test_ballistic.cpp            # Ballistic parabolic flight & equilibrium test
    ├── test_camera_rig.cpp           # F-16 DEP camera rig & G-load head dynamics
    ├── test_compressibility.cpp      # Prandtl-Glauert, wave drag, Mach tuck, speedbrake
    ├── test_config_persistence.cpp   # JSON configuration load/save round-trip test
    ├── test_flcs_level_flight.cpp    # 60-second hands-off level flight stabilization
    ├── test_flcs_pullup.cpp          # Full aft stick +9G and 25.5 AoA limiter test
    ├── test_flight_instruments.cpp   # FBO render-to-texture and instrument logic
    ├── test_flight_realism.cpp       # Turn performance, stall, Mach lift, G limits, spool, over-G
    ├── test_force_curves.cpp         # F-16 force-sensing polynomial response curves
    ├── test_gyroscopic.cpp           # Rotational dynamics & momentum conservation
    ├── test_hud_collimation.cpp      # Collimated optical infinity math & FPM projection
    ├── test_input_pipeline.cpp       # End-to-end SDL3 polling and FLCS driving latency
    ├── test_integrated_flight.cpp    # Takeoff, landing, fuel burn across the full stack
    ├── test_interpolator.cpp         # Multidimensional interpolation verification
    ├── test_landing_gear.cpp         # Strut statics, braking, steering, gear failure
    ├── test_performance.cpp          # Zero-heap allocation check & 6M step/s benchmark
    ├── test_pitch_divergence.cpp     # Open-loop pitch divergence test (No FLCS)
    ├── test_propulsion.cpp           # Thrust lapse, spool lag, fuel burn, variable mass
    ├── test_render_fidelity.cpp      # Lighting, shadows, HDR, clouds, model orientation
    ├── test_thread_pool.cpp          # Worker pool and deterministic parallel terrain bake
    ├── test_render_pipeline.cpp      # Multi-pass OpenGL graphics pipeline verification
    ├── test_signal_conditioning.cpp  # Dual deadbands, calibration, and inversion
    ├── test_throttle_control.cpp     # Throttle slew, afterburner accel, deceleration
    ├── test_performance_envelope.cpp # Top speed, T/W and thrust lapse vs real F-16C
    ├── test_terrain_field.cpp        # Flat airfield basin, determinism, normals, bounds
    └── test_terrain_render.cpp       # Winding/culling, runway over terrain, horizon seam
```

---

## Building and Running Tests

### Requirements
- A C++20 compiler (GCC 11+ or Clang 14+)
- SDL3, libepoxy and OpenGL — needed only for the viewer and the graphics tests. The 19 physics test suites build with no external dependencies.

### Make

To compile and run all 26 verification test suites:

```bash
make run_all
```

To run the interactive real-time 3D cockpit flight simulator:

```bash
make viewer
./bin/f16_sim_viewer
```

> **Note:** the Makefile builds with `-march=native`, which tunes for the building machine and produces binaries that may fault elsewhere. Use the CMake build for anything portable or distributable.

### Simulator Controls

| Key | Function |
|-----|----------|
| Arrow Keys | Pitch and roll stick commands (hold the pull to walk the stick on to the stop: ~6 G on the press, 9 G after about a second) |
| A / D | Rudder pedals (also nosewheel steering on the ground) |
| Shift / Ctrl | Throttle increase / decrease (1.5 s idle to full reheat) |
| PageUp / PageDown | Alternative throttle bindings |
| I | Invert hardware throttle axis (toggle forward=full power vs forward=cutoff) |
| J | Ignore a hardware throttle lever (keyboard only) |
| 1 / 2 / 3 / 4 | Snap throttle to Cutoff / Idle / Military / Max afterburner |
| B | Speedbrake extend / retract (2 s hydraulic travel) |
| Space | Wheel brakes, for the rollout |
| G | Landing gear up / down |
| T | Trim reset |
| R | Reset simulation (in a fight: restart it) |
| F | Fire gun |
| C | Dispense chaff (two bundles per press) |
| L | Radar: lock the contact under the cursor (STT) / drop the lock |
| K | Radar mode: CRM (RWS search) / ACM (dogfight auto-lock) |
| O | Radar scan: A6 / A3 / A1 azimuth (CRM), 20×30 / 10×60 / BORE (ACM) |
| `[` / `]` | Radar range scale down / up (5, 10, 20, 40, 80 nm) |
| `=` / `-` | Radar antenna elevation up / down (hold) |
| P | Left MFD page: FCR radar / FLCS |
| F3 / F4 / F5 / F6 / F7 / F12 | Select the F-16C / F-15EX / Typhoon / F-22A / A-10C / F-35B (F8 cycles; M opens the airframe menu, keys 1–6) |
| S | F-35B STOVL/HOOK button: convert to STOVL (~15 s) / reconvert to CTOL. HOTAS: throttle button 7 (`btn_stovl_mode`) |
| F9 / F10 / F11 | New fight / cycle set-up (merge, perch, range, evade) / cycle bandit skill or evade difficulty |
| Esc | Exit |

Run with `--profile` to print frame timing and the GPU cost of each render pass every two seconds.

#### F-35B STOVL

The F-35B (`--aircraft f35b`, or F12) flies conventionally until the STOVL/HOOK button (S) converts it: the LiftFan and roll-post doors open, the clutch engages and the swivelling nozzle comes down. Conversion needs less than 250 kt. Once converted, the unified control laws change what the inceptors do:

- **Throttle** commands acceleration, not thrust. Its hold detent at 50% holds speed, and below 25 kt it holds zero ground speed (a hover). Fore or aft of the detent the jet accelerates or decelerates. When the jet converts, the keyboard lever snaps to the detent. Shift/Ctrl stop on the detent with a click: release and press again to go through it.
- **Stick** fore/aft commands climb or sink rate in the hover (aft = up, at most 7 ft/s down). Lateral stick banks the jet to translate sideways.
- **Pedals** command yaw rate.

Start scenarios:

| Flag | Start |
|------|-------|
| `--hover` | Converted, hovering with the wheels 50 ft above the runway at vertical-landing weight (17,200 kg / 37,900 lb, about 3,300 lb of fuel) |
| `--stovl-approach` | Converted, 120 kt at 1,000 ft, 3 nm short of the runway |

Both flags select the F-35B if another jet was asked for. A runway start works too: select STOVL before the roll for a short takeoff.

### CMake

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

CMake builds with portable flags by default. Add `-DFASTJET_NATIVE_ARCH=ON` to opt into `-march=native`, or `-DFASTJET_BUILD_VIEWER=OFF` to build the physics tests alone when SDL3 is unavailable.

Individual test targets:
```bash
make test_input_pipeline     # Validates SDL3 polling and end-to-end FLCS driving latency
make test_signal_conditioning# Validates inner/outer deadbands and axis calibration
make test_force_curves       # Validates polynomial y = (1-c)*x + c*x^3 force curves
make test_avionics_controls  # Validates digital trim hats, throttle detents, speedbrakes
make test_config_persistence # Validates JSON configuration serialization and parsing
make test_actuators          # Validates hydraulic lag, rate limits, and deflection bounds
make test_flcs_level_flight  # Validates 60s hands-off level flight stability with FLCS
make test_flcs_pullup        # Validates full aft stick +9G and 25.5 deg AoA limiters
make test_pitch_divergence   # Validates open-loop pitch divergence under 200 Hz RK4
make test_aero_coefficients  # Validates NASA TP-1538 coefficients and dCm/dalpha > 0
make test_interpolator       # Validates 1D, 2D, and 3D interpolation and clamping
make test_atmosphere         # Validates atmosphere properties across troposphere/stratosphere
make test_ballistic          # Validates trajectory against closed-form analytical equations
make test_gyroscopic         # Validates torque-free angular momentum conservation
make test_performance        # Validates zero heap allocation and measures throughput
make test_missiles           # Validates missile kinematics, PN, launch zones, notch, chaff, radar lock and RWR
make test_evade_mode         # Validates evade set-ups, lock warning, end rules and what defeats each difficulty
make test_radar_scope        # Validates the FCR scans, what they can see, STT lock, ACM auto-acquisition and range scales
make test_flight_realism     # Validates turn performance, stall, Mach effects on lift, G limits, spool-up and over-G damage
```

---

## Example Usage

```cpp
#include "fastjet/fdm/flight_state.hpp"
#include "fastjet/fdm/rk4_integrator.hpp"
#include "fastjet/environment/atmosphere1976.hpp"

using namespace fastjet::math;
using namespace fastjet::fdm;
using namespace fastjet::environment;

int main() {
    // 1. Initialize F-16 mass properties and 200 Hz integrator (dt = 0.005s)
    const MassProperties f16_mass = MassProperties::create_clean_f16();
    const RK4Integrator integrator(0.005);

    // 2. Set initial flight condition (cruise at 10,000 m altitude, 250 m/s airspeed)
    FlightState state;
    state.pos_ned = Vector3(0.0, 0.0, -10000.0); // NED z = -10,000 m (altitude = 10,000 m)
    state.vel_b   = Vector3(250.0, 0.0, 0.0);     // 250 m/s along forward body axis
    state.omega_b = Vector3::zero();
    state.q_att   = Quaternion::identity();       // Level flight

    double sim_time = 0.0;

    // 3. Hot simulation loop (allocates zero heap memory)
    for (int step = 0; step < 2000; ++step) { // 10 seconds of simulation
        // Query air data for aerodynamics
        const AirData air = Atmosphere1976::compute(state.altitude(), state.airspeed());

        // Step simulation using custom force provider lambda
        integrator.step(state, sim_time, f16_mass, [&](double t, const FlightState& s) noexcept {
            AircraftForces forces;
            // Example: Trim lift force opposing gravity
            forces.force_b = Vector3(0.0, 0.0, -f16_mass.mass_kg * SixDoFFDM::GRAVITY_ACCEL);
            return forces;
        });
    }

    return 0;
}
```
