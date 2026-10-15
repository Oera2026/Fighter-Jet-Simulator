#pragma once

#include "pilot_commands.hpp"
#include "imu_sensor.hpp"
#include "../aircraft/aircraft_config.hpp"
#include "../aero/compressible_lift.hpp"
#include "../fdm/flight_state.hpp"
#include "../environment/atmosphere1976.hpp"
#include "../environment/ground_collision.hpp"
#include "../math/vector3.hpp"
#include <cmath>
#include <algorithm>
#include <array>

namespace fastjet {
namespace flcs {

/**
 * @brief On-Board Flight Computer (OFC) — autonomous pilot-safety override systems.
 *
 * Implements three independent, real-world inspired safety systems:
 *
 *  1. GLOC Auto-Recovery (G-induced Loss Of Consciousness)
 *     Monitors the pilot's G-exposure with a simplified Franks/Goodwin physiological
 *     model. When exposure accumulates past the incapacitation threshold the FLCS takes
 *     autonomous control — an unloaded roll to wings level, then a G-limited pull to a
 *     shallow climb — until the pilot regains full authority.  Visual blackout fraction
 *     is exposed for screen-space rendering.
 *
 *  2. Auto-GCAS (Automatic Ground Collision Avoidance System)
 *     Mirrors the NASA/AFRL/Lockheed system that has saved 11+ F-16 pilots since 2014,
 *     and its defining property: it is nuisance-free, so pilots fly 100 ft passes,
 *     terrain following and canyon runs with it ON.  Every tick it flies the
 *     recovery it would perform — the current manoeuvre carrying on through the
 *     system's reaction time, a roll at the achievable rate, then the airframe's
 *     recovery G (nominal 5 G, less when slow) to a capped climb — through a
 *     handful of fixed time steps, and reads the terrain under every point of it
 *     (see predict_clearance).  It fires only at the last instant: when the
 *     predicted minimum clearance falls to the terrain buffer (30 ft by default,
 *     see Airframe::gcas_buffer_m).  Climb, turn and load already on the jet are
 *     part of the prediction, so a pilot who is already climbing over a ridge or
 *     holding a level turn is left alone.  Up to three escapes are predicted:
 *     wings level, and a climbing turn either way (the escape that follows a
 *     canyon bend; NASA's multi-trajectory scheme).  The system fires only when
 *     every one of them is down to the buffer, and flies the best.  Recovery is a
 *     two-phase manoeuvre — roll to the recovery bank first, then pull — escalating
 *     toward a capped ceiling only if the flown recovery falls short of its own
 *     prediction (never the airframe's full G limit: both are scaled to the
 *     configured airframe).  It commands the throttle alongside the stick: idle
 *     while still rolling (unless the jet is too slow to spare the energy), then
 *     full power once pulling, and never back again within one recovery.  Past
 *     135 deg of bank the pull is faded to a 0 G unload (see bank_pull_scale),
 *     because pulling while inverted drives the nose into the terrain.  It hands
 *     back once the jet is climbing with terrain clearance assured, or to a
 *     deliberate pilot input once the save is no longer marginal; after such a
 *     breakout it re-engages only for a last-ditch save (which then cannot be
 *     broken out of), so a pilot flying low on purpose is not fought tick by tick.
 *
 *  3. Tumble / Spin Auto-Recovery
 *     Detects a departure (deep stall, flat spin, violent tumble) and applies
 *     anti-spin rudder, roll damping and a stall-breaking pitch command.
 *
 * Authority order is fixed: departure recovery stands Auto-GCAS down before it
 * engages (a pull-out margin means nothing in a spin), but an Auto-GCAS recovery
 * already under way keeps the pitch and roll axes, with the anti-spin rudder still
 * applied underneath it.  Every pilot-intent test reads the *raw* inceptor
 * positions, never the OFC's own output.
 *
 * Zero dynamic heap allocation.  Thread-safe if external synchronisation is provided.
 */
class OnBoardFlightComputer {
public:

    // =========================================================================
    // Public constants
    // =========================================================================

    // --- GLOC thresholds ----------------------------------------------------
    /// Gz below which the pilot fully recovers (no exposure accumulation).
    static constexpr double GLOC_BASE_TOLERANCE_G = 6.5;
    /// Gz at which rapid incapacitation onset begins.
    static constexpr double GLOC_RAPID_ONSET_G    = 8.2;
    /// Gz where immediate GLOC risk is very high (sustained ~1 s).
    static constexpr double GLOC_IMMEDIATE_G      = 9.5;
    /// Exposure accumulator value that triggers full GLOC.
    static constexpr double GLOC_TRIP_THRESHOLD   = 20.0;
    /// Passive recovery rate [accumulator units / s] when Gz < base tolerance.
    static constexpr double GLOC_RECOVERY_RATE    = 5.0;
    /// Minimum unconscious duration [s] (fast recovery to keep sim responsive).
    static constexpr double GLOC_MIN_DURATION_S   = 2.5;
    /// Maximum unconscious duration [s].
    static constexpr double GLOC_MAX_DURATION_S   = 4.5;
    /// Pilot authority ramp-up duration after regaining consciousness [s].
    static constexpr double GLOC_AUTHORITY_RAMP_S = 1.5;
    /// Roll angle below which GLOC recovery switches from wing-levelling to climb [rad].
    static constexpr double GLOC_WINGS_LEVEL_RAD  = 20.0 * (M_PI / 180.0);
    /// Bank beyond which the GLOC roll-out is fully unloaded [rad]; between this and
    /// GLOC_WINGS_LEVEL_RAD the climb command is blended in.
    static constexpr double GLOC_BLEND_RAD        = 45.0 * (M_PI / 180.0);
    /// Flight path the GLOC autopilot settles on [rad], and how fast it captures it [s].
    static constexpr double GLOC_TARGET_FPA_RAD   = 3.0 * (M_PI / 180.0);
    static constexpr double GLOC_CAPTURE_S        = 1.5;
    /// The GLOC autopilot never pulls more than this [G] (or GLOC_MAX_G_FRAC of the
    /// airframe's limit): an unconscious pilot must not be put straight back under
    /// the G that knocked him out.
    static constexpr double GLOC_MAX_RECOVERY_G   = 5.5;
    static constexpr double GLOC_MAX_G_FRAC       = 0.65;
    /// The authority ramp back to the pilot waits for the jet to be settled —
    /// wings within this bank [rad] and the flight path within this of the
    /// target [rad] — or at most this long [s].
    static constexpr double GLOC_SETTLED_BANK_RAD = 15.0 * (M_PI / 180.0);
    static constexpr double GLOC_SETTLED_FPA_RAD  = 6.0 * (M_PI / 180.0);
    static constexpr double GLOC_SETTLE_MAX_S     = 8.0;
    /// Until the stick has been let go once after a GLOC, the pitch it commands
    /// is held below this G [G]: a hand still clamped on the stick from the pull
    /// that caused the GLOC must not drive straight back into it.
    static constexpr double GLOC_RAMP_G_LIMIT     = GLOC_BASE_TOLERANCE_G - 1.0;

    // --- GCAS thresholds ----------------------------------------------------
    /// Default terrain buffer [m] (= 30 ft): the predicted recovery must keep at
    /// least this much clearance, or it is started now.  It doubles as the floor
    /// the system lets the pilot fly at: 50 ft passes are clear of it, a jet
    /// settling below 30 ft is flown back up.  See Airframe::gcas_buffer_m.
    static constexpr double GCAS_DEFAULT_BUFFER_M = 30.0 * 0.3048;
    /// Extra predicted clearance [m] needed before a recovery hands back (or a
    /// pilot may break out).  Hysteresis: released just above the buffer, the
    /// next tick's prediction would fire again.
    static constexpr double GCAS_RELEASE_MARGIN_M = 10.0;
    /// Time the current manoeuvre is predicted to carry on before the recovery
    /// starts to act [s]: decision, the FLCS command shaping and actuator lag
    /// before the G starts to build.
    static constexpr double GCAS_REACTION_S       = 0.10;
    /// First-order lag of the load factor toward the recovery command [s] at
    /// GCAS_G_ONSET_Q_REF_PA (default; tuned per airframe): the pitch loop's G
    /// onset as flown closed-loop by these flight control laws.  It scales with
    /// 1/sqrt(q) — control power grows with dynamic pressure — within the
    /// bounds below: measured 63% rise times run from ~0.45 s at 180 m/s to
    /// ~0.22 s at 330 m/s.  A fixed 0.2 s + 0.35 s planned a fast dive's pull
    /// 0.3 s late, and so started it ~120 m too high.
    static constexpr double GCAS_G_ONSET_TAU_S    = 0.33;
    static constexpr double GCAS_G_ONSET_Q_REF_PA = 20000.0;
    static constexpr double GCAS_G_ONSET_SCALE_MIN = 0.55;
    static constexpr double GCAS_G_ONSET_SCALE_MAX = 1.8;
    /// The predicted recovery pulls no steeper than this climb, then holds it [rad].
    /// Shared with the flown recovery, which captures it in GCAS_CLIMB_CAPTURE_S.
    static constexpr double GCAS_MAX_CLIMB_RAD    = 30.0 * (M_PI / 180.0);
    /// Climb of the steep escape, tried when terrain rises faster than the
    /// standard climb can outclimb (a 45 deg slope ahead) [rad].
    static constexpr double GCAS_STEEP_CLIMB_RAD  = 60.0 * (M_PI / 180.0);
    static constexpr double GCAS_CLIMB_CAPTURE_S  = 1.0;
    /// The prediction always looks at least this far ahead (rising terrain), and
    /// at most this far (a slow pull out of a steep dive) [s].
    static constexpr double GCAS_LOOKAHEAD_S      = 6.0;
    static constexpr double GCAS_PREDICT_MAX_S    = 14.0;
    /// Prediction time steps [s]: fine while the recovery is still being decided,
    /// coarser once it is established.  At 250 m/s the fine step samples the
    /// terrain every 25 m, finer than the smallest ridges in the terrain field.
    static constexpr double GCAS_FINE_STEP_S      = 0.10;
    static constexpr double GCAS_COARSE_STEP_S    = 0.20;
    static constexpr double GCAS_FINE_HORIZON_S   = 4.0;
    /// Above this height [m MSL] no terrain can be reached: the prediction stops
    /// sampling it (the procedural field's highest point is the canyon mesa).
    static constexpr double GCAS_TERRAIN_CEILING_M = graphics::TerrainField::MAX_HEIGHT;
    /// Turn (horizontal load factor [g]) above which the climbing-turn escape in the
    /// direction of the turn already being flown keeps that turn's G, and the bank
    /// the climbing-turn escapes use at most [rad].
    static constexpr double GCAS_MIN_TURN_G       = 0.3;
    static constexpr double GCAS_MAX_TURN_BANK_RAD = 60.0 * (M_PI / 180.0);
    /// Extra horizontal G of the tightened version of the turn being flown [g].
    static constexpr double GCAS_TURN_TIGHTEN_G   = 2.0;
    /// Share of the airframe's G limit a turning escape may keep when the pilot
    /// is already pulling more than the recovery ceiling.
    static constexpr double GCAS_TURN_LIMIT_FRAC  = 0.95;
    /// Valley-following escape (see valley_attitude): how far ahead it reads the
    /// terrain [s of flight, clamped to m], how far either side [m], the turn per
    /// unit of cross slope [g], how far above the jet the terrain must stand to
    /// be steered around [m], and the largest share of the G spent turning.
    static constexpr double GCAS_VALLEY_LOOK_S       = 1.5;
    static constexpr double GCAS_VALLEY_LOOK_MIN_M   = 200.0;
    static constexpr double GCAS_VALLEY_LOOK_MAX_M   = 500.0;
    static constexpr double GCAS_VALLEY_HALF_WIDTH_M = 80.0;
    static constexpr double GCAS_VALLEY_GAIN         = 1.5;
    static constexpr double GCAS_VALLEY_CLEAR_M      = 50.0;
    static constexpr double GCAS_VALLEY_MAX_TURN     = 0.9;
    /// ...and the climb it pulls to [rad]: steeper than the standard escape, so
    /// it is out over the rim of a deep canyon before the next bend.
    static constexpr double GCAS_VALLEY_CLIMB_RAD    = 45.0 * (M_PI / 180.0);
    /// Clearance the valley escape must predict over and above the others [m].
    static constexpr double GCAS_VALLEY_EXTRA_M      = 15.0;
    /// A climbing-turn escape replaces the latched one mid-recovery only if it is
    /// predicted to clear the terrain by this much more [m] (no flip-flopping).
    static constexpr double GCAS_ESCAPE_SWITCH_M  = 15.0;
    /// ...and no sooner than this after the last change of escape [s].
    static constexpr double GCAS_ESCAPE_SWITCH_HOLD_S = 1.0;
    /// After a pilot breakout the system stays inhibited until the stick has been
    /// relaxed (or the jet clear) this long [s]; meanwhile a recovery it starts is
    /// a last-ditch save: flown at the ceiling G, and not to be broken out of.
    static constexpr double GCAS_INHIBIT_RELAX_S  = 1.0;
    /// Stick deflections that count as a deliberate pilot breakout, and the band a
    /// stick counts as relaxed in.
    static constexpr double GCAS_BREAKOUT_PITCH   = 0.35;
    static constexpr double GCAS_BREAKOUT_ROLL    = 0.40;
    static constexpr double GCAS_RELAXED_STICK    = 0.10;
    /// Stick deflection beyond which the pilot counts as actively flying the
    /// jet, and how long his manoeuvre is then carried on before the escapes are
    /// judged [s] (on top of the reaction time).
    static constexpr double GCAS_ACTIVE_STICK     = 0.20;
    static constexpr double GCAS_PILOT_CREDIT_S   = 0.50;
    /// After a breakout, the jet being clear by this much more than the release
    /// level for GCAS_INHIBIT_RELAX_S also re-arms the system fully [m]: a pilot
    /// manoeuvring hard through a canyon may never let go of the stick.
    static constexpr double GCAS_INHIBIT_CLEAR_M  = 20.0;
    /// Nominal recovery G used for both the predicted recovery and the commanded
    /// pitch during recovery — the two stay consistent by construction.
    static constexpr double GCAS_RECOVERY_G       = 5.0;
    /// Escalation ceiling for commanded G when the flown recovery falls short of
    /// its prediction. Kept well below the airframe G limit so the save itself
    /// can't cause a structural overstress or induce GLOC.
    static constexpr double GCAS_MAX_RECOVERY_G   = 7.5;
    /// Nominal and ceiling recovery G are also capped to these fractions of the
    /// configured airframe's own positive G limit, so a 7.33 G A-10 is never asked
    /// for the 7.5 G a 9 G fighter can afford.
    static constexpr double GCAS_RECOVERY_G_FRAC  = 0.70;
    static constexpr double GCAS_MAX_G_FRAC       = 0.86;
    /// Fraction of the airframe's rated roll rate budgeted for rolling to the
    /// recovery bank (default; tuned per airframe in Airframe::from).  The rated
    /// figure is a limiter ceiling reached only at high q, after the roll has
    /// accelerated; the recovery has to plan on what it will actually get.
    static constexpr double GCAS_ROLL_RATE_FRAC   = 0.45;
    /// Bank error inside which the recovery roll command (and the predicted roll
    /// rate) is proportional rather than full rate [rad].
    static constexpr double GCAS_ROLL_BAND_RAD    = M_PI * 0.25;
    /// Roll-rate lead in the recovery roll command [s]: the bank error is taken
    /// this far ahead at the present roll rate, so a full-rate roll is checked
    /// before it overshoots the recovery bank (without it a 300 deg/s roll to a
    /// 60 deg turn escape carried on to 130 deg).
    static constexpr double GCAS_ROLL_LEAD_S      = 0.25;
    /// First-order lag of the roll rate toward its command [s] (default; tuned
    /// per airframe): roll shaping and actuator lag of the lateral laws.
    static constexpr double GCAS_ROLL_TAU_S       = 0.20;
    /// While rolling faster than this [rad/s] the flight control laws deliver
    /// only this share of a commanded pull above 1 G (they hold their pitch
    /// integrators and relieve the G through a roll): a 5 G pull flown from
    /// 95 deg of bank peaked at 3.8 G until the wings were level.
    static constexpr double GCAS_ROLLING_RATE_RAD_S = 45.0 * (M_PI / 180.0);
    static constexpr double GCAS_ROLLING_G_FRAC   = 0.70;
    /// Share of the maximum-lift load factor that the recovery is planned on. At
    /// low speed the nominal 5 G simply is not available, and planning on it
    /// engaged far too late to pull a slow dive out.
    static constexpr double GCAS_USABLE_LIFT_FRAC = 0.80;
    /// Commanded G is closed on the measured Nz. Integral trim gain [stick/(G s)]
    /// and authority for fly-by-wire laws, whose own G command gets most of the way.
    static constexpr double GCAS_NZ_TRIM_KI       = 0.25;
    static constexpr double GCAS_NZ_TRIM_LIMIT    = 0.20;
    /// Direct-linkage laws (A-10) have no G command at all: stick is elevator, and
    /// the G a given stick produces grows with q.  Estimated G per unit stick is
    /// this coefficient times q S / W (feed-forward and gain scheduling), and the
    /// Nz loop gains below are normalised by it so the loop behaves the same at
    /// 100 m/s and at Vne — fixed gains ran a 3-7 G oscillation near top speed.
    static constexpr double GCAS_DIRECT_G_PER_QSW = 0.8;
    static constexpr double GCAS_DIRECT_NZ_KP     = 0.15;  ///< [-] (normalised)
    static constexpr double GCAS_DIRECT_NZ_KI     = 1.2;   ///< [1/s] (normalised)
    /// A direct-linkage stick moves less elevator pushing than pulling (the feel
    /// system is stiffer forward): G per unit of forward stick, relative to aft.
    static constexpr double GCAS_DIRECT_PUSH_GAIN = 0.45;
    /// Roll angle below which GCAS switches from rolling to the recovery bank to
    /// pulling [rad].
    static constexpr double GCAS_WINGS_LEVEL_RAD  = 25.0 * (M_PI / 180.0);
    /// Roll angle above which an in-progress pull-up reverts to wing-levelling [rad].
    /// Wider than GCAS_WINGS_LEVEL_RAD so the two phases cannot chatter against
    /// each other while the jet settles through the band.
    static constexpr double GCAS_REBANK_RAD       = 40.0 * (M_PI / 180.0);
    /// Flight-path angle above which a recovery may hand back [rad] (0 deg = level flight).
    static constexpr double GCAS_SAFE_FPA_RAD     = 0.0  * (M_PI / 180.0);
    /// How far ahead the release check flies the jet hands-off [s]: once handed
    /// back, the jet must not be on course to need the system again this soon.
    /// Checking only the recovery's own margin handed a jet back one tick after
    /// it turned from a wall it was still flying straight at.
    static constexpr double GCAS_RELEASE_HORIZON_S = GCAS_LOOKAHEAD_S;
    /// Absolute hard deck: the jet must never be allowed to descend through this
    /// altitude AGL. Enforced unconditionally alongside (not instead of) the
    /// predicted recovery, as a last-resort backstop [m] (= 30 ft).
    static constexpr double GCAS_HARD_DECK_M      = 30.0 * 0.3048;
    /// Below this height AGL [m] (~1,000 ft) a deep stall is not broken
    /// nose-down: the dive that follows a stall break needs more than is left.
    static constexpr double TUMBLE_STALL_BREAK_FLOOR_M = 300.0;
    /// Throttle commanded while rolling to wings-level during a recovery: cut back
    /// toward idle so the dive doesn't keep accelerating into the ground while the
    /// jet is still banked and can't yet pull effectively.
    static constexpr double GCAS_ROLL_THROTTLE    = 0.15;
    /// Throttle commanded once actively pulling out: full power, so arresting the
    /// sink rate doesn't bleed the airspeed needed to sustain the recovery G and
    /// stall the jet right back toward the ground.
    static constexpr double GCAS_PULLOUT_THROTTLE = 1.0;
    /// Gear-down descents no steeper and no faster than these are treated as a
    /// landing approach rather than an emergency, so GCAS doesn't interfere with a
    /// normal (or merely firm, steep-ish) landing.  A 6 m/s limit alone was tripped
    /// by an ordinary 5-6 deg approach at A-10/F-16 approach speeds.  A real
    /// emergency dive sinks far faster and far steeper than this [m/s], [rad].
    static constexpr double GCAS_LANDING_SINK_MPS = 12.0;
    static constexpr double GCAS_LANDING_FPA_RAD  = -8.0 * (M_PI / 180.0);

    // =========================================================================
    // GLOC state machine
    // =========================================================================
    enum class GlocState {
        NORMAL,    ///< Pilot fully conscious, no impairment.
        GREYOUT,   ///< Peripheral vision narrowing; pilot retains authority.
        BLACKOUT,  ///< Vision lost but pilot marginally conscious (brief window).
        GLOC,      ///< Full unconsciousness; FLCS has autonomous authority.
        RECOVERY   ///< Regaining consciousness; authority linearly restored.
    };

    // =========================================================================
    // GCAS state machine
    // =========================================================================
    enum class GcasState {
        ARMED,       ///< Monitoring; no intervention.
        WING_LEVEL,  ///< Phase 1: rolling to the recovery bank (wings level, or the climbing-turn bank).
        PULL_UP,     ///< Phase 2: recovery G pull to a positive FPA.
        SAFE         ///< Recovery complete (or broken out of); monitoring resumes.
    };

    // =========================================================================
    // Tumble / Spin Auto-Recovery state machine
    // =========================================================================
    enum class TumbleState {
        ARMED,       ///< Monitoring for spin, tumble, or deep stall
        RECOVERING,  ///< Active autonomous anti-spin & stabilization override
        SAFE         ///< Airframe restored within normal flight envelope
    };

    // =========================================================================
    // Public state (read-only by HUD / telemetry)
    // =========================================================================
    GlocState   gloc_state{GlocState::NORMAL};
    GcasState   gcas_state{GcasState::ARMED};
    TumbleState tumble_state{TumbleState::ARMED};

    double g_exposure{0.0};         ///< G-exposure accumulator [0, GLOC_TRIP_THRESHOLD].
    double gloc_timer{0.0};         ///< Timer used for unconscious / recovery phase [s].
    double gloc_duration{0.0};      ///< Randomised unconscious duration for this event [s].
    double gloc_authority{1.0};     ///< Pilot authority fraction [0=OFC only, 1=full pilot].
    double blackout_fraction{0.0};  ///< Screen-space fade value [0=clear, 1=full black].

    double gcas_tti{999.0};         ///< Time-to-terrain-impact at current trajectory [s].
    /// Predicted terrain clearance of the best recovery, less the buffer [m]: how
    /// much later the recovery could still start (0 = it starts now).
    double gcas_margin_m{999.0};
    double tumble_persist_timer{0.0}; ///< Time out-of-control conditions have persisted [s].
    /// A spin or tumble (rotational departure) is being detected this tick, before
    /// the persistence filter has let the tumble law latch.
    bool departure_rotation{false};

    /// Latched roll direction (+1 / -1, 0 = unlatched) used while the bank angle sits
    /// in the ambiguous zone either side of inverted, where the Euler roll angle wraps
    /// between +180 and -180 deg. See @ref bank_capture_roll_cmd.
    int gcas_roll_dir{0};
    int gloc_roll_dir{0};

    /// GCAS throttle override for this tick: -1.0 = no override (pilot/auto-throttle
    /// keeps authority), otherwise a commanded throttle position in [0, 1].
    double gcas_throttle_cmd{-1.0};

    /// Integral trims on the GCAS and GLOC pitch commands, closing the commanded
    /// G on the measured Nz [stick units].
    double gcas_nz_trim{0.0};
    double gloc_nz_trim{0.0};

    /// The escape the recovery in progress is flying: the bank it rolls to [rad]
    /// and the G it pulls.  Latched when the recovery starts.
    double gcas_escape_bank{0.0};
    double gcas_escape_g{0.0};
    double gcas_escape_climb{GCAS_MAX_CLIMB_RAD};
    /// The escape being flown follows the valley (its bank is re-steered each tick).
    bool gcas_escape_valley{false};
    double gcas_escape_turn_g{0.0};
    /// The candidate escape that last cleared: tried first next tick.
    double gcas_escape_g_cap{0.0};
    int gcas_escape_hint{0};
    /// Ticks left before the escapes are re-predicted while armed and far from terrain.
    int gcas_eval_skip{0};
    /// Where the escapes were last predicted from [m, NED].
    math::Vector3 gcas_eval_pos{};
    /// The recovery in progress has reached its pull: full power from here on.
    bool gcas_pulled{false};
    /// The recovery in progress is a last-ditch save (no breakout).
    bool gcas_last_ditch{false};
    /// A pilot broke out of a recovery: only a last-ditch save re-engages until the
    /// stick has been relaxed for GCAS_INHIBIT_RELAX_S.
    bool gcas_inhibited{false};
    double gcas_relaxed_s{0.0};
    /// Time before the recovery in progress may change escape again [s].
    double gcas_switch_hold_s{0.0};

    /// Set once both stick axes have been seen near neutral since GLOC onset. A
    /// pilot who blacks out mid-pull is still holding the stick where it was, and
    /// that held deflection is not evidence of having woken up.
    bool gloc_stick_relaxed{false};
    /// The stick has not been let go since the blackout (see step_gloc).
    bool gloc_dead_hand{false};
    /// Time spent in RECOVERY so far, settled or not [s].
    double gloc_wait_s{0.0};

    // =========================================================================
    // Airframe limits and safety-system tuning
    // =========================================================================

    /**
     * @brief The airframe numbers the OFC's predictions depend on, and the per-jet
     * tuning of its recoveries.
     *
     * A single set of F-16 constants used to serve every jet: the recovery G was
     * mapped to stick through the F-16's 9 G range, departure was declared at an
     * F-16's 32 deg of alpha, and the pull-out was planned on 5 G being there
     * whatever the airspeed.  None of that holds for a 7.33 G direct-linkage A-10
     * or a 65 deg-alpha F-22.  The recovery figures (G, roll rate, G onset) are
     * what each jet's flight control laws actually deliver when the OFC flies
     * them closed loop, measured against the 6-DoF model; Airframe::from is the
     * one place they are set.
     */
    struct Airframe {
        double max_g{9.0};             ///< Positive G limit.
        double min_g{-3.0};            ///< Negative G limit.
        double alpha_limit_deg{25.2};  ///< FLCS alpha limit (stall alpha for the A-10).
        double roll_rate_dps{308.0};   ///< Rated maximum roll rate.
        double wing_area_m2{27.87};
        double cl_max{1.4};
        double ref_mass_kg{12473.7};   ///< Empty + full internal fuel: heaviest clean case.
        bool   g_command_law{true};    ///< Stick commands Nz (FBW) vs elevator (direct).

        // --- Auto-GCAS -------------------------------------------------------
        double gcas_recovery_g{5.0};      ///< Nominal fly-up G.
        double gcas_max_recovery_g{7.5};  ///< Escalation ceiling.
        double gcas_roll_dps{138.6};      ///< Roll rate the recovery plans on.
        /// Airspeed the planned roll rate is reached at, for a direct-linkage
        /// law whose roll rate grows with airspeed (0 = rate-command law).
        double gcas_roll_ref_mps{0.0};
        double gcas_g_tau_s{GCAS_G_ONSET_TAU_S}; ///< Closed-loop G onset lag at the reference q.
        /// Share of the maximum-lift load factor the recovery plans on.
        double gcas_usable_lift{GCAS_USABLE_LIFT_FRAC};
        /// Load factor the jet flies with the stick let go [G]: a G-command law
        /// holds 1 G; a direct-linkage jet with no trim noses over toward ~0 G.
        /// The hand-back check flies this, so a recovery is not released into a
        /// pushover straight back toward the terrain.
        double gcas_hands_off_g{1.0};
        double gcas_roll_tau_s{GCAS_ROLL_TAU_S}; ///< Closed-loop roll-rate lag.
        double gcas_reaction_s{GCAS_REACTION_S}; ///< Decision + stick-to-surface lag.
        double gcas_buffer_m{GCAS_DEFAULT_BUFFER_M}; ///< Terrain buffer.
        // --- GLOC auto-recovery ----------------------------------------------
        double gloc_max_g{GLOC_MAX_RECOVERY_G}; ///< Most G the GLOC autopilot pulls.

        [[nodiscard]] static constexpr Airframe from(aircraft::AircraftType type) noexcept {
            const auto cfg = aircraft::AircraftConfig::get(type);
            Airframe a;
            a.max_g           = cfg.flcs.max_g_positive;
            a.min_g           = cfg.flcs.max_g_negative;
            a.alpha_limit_deg = cfg.flcs.alpha_limit_deg;
            a.roll_rate_dps   = cfg.flcs.max_roll_rate_dps;
            a.wing_area_m2    = cfg.aero.s_ref;
            a.cl_max          = cfg.aero.cl_max;
            a.ref_mass_kg     = cfg.mass.empty_mass_kg + cfg.mass.internal_fuel_capacity_kg
                              + cfg.mass.combat_payload_kg;
            a.g_command_law   = cfg.flcs.law_type
                              != aircraft::FLCSConfig::LawType::HYDRO_SAS_AUGMENTED;

            a.gcas_recovery_g     = std::min(GCAS_RECOVERY_G, GCAS_RECOVERY_G_FRAC * a.max_g);
            a.gcas_max_recovery_g = std::max(a.gcas_recovery_g,
                                             std::min(GCAS_MAX_RECOVERY_G, GCAS_MAX_G_FRAC * a.max_g));
            a.gcas_roll_dps       = GCAS_ROLL_RATE_FRAC * a.roll_rate_dps;
            a.gloc_max_g          = std::min(GLOC_MAX_RECOVERY_G, GLOC_MAX_G_FRAC * a.max_g);

            // What each jet's laws deliver when the recovery flies them, from
            // closed-loop runs against the 6-DoF model (roll 179 -> 90 deg and the
            // 1 -> 5 G step), taken at ~85% so the plan stays on the safe side.
            switch (type) {
                case aircraft::AircraftType::F16_FIGHTING_FALCON:
                    a.gcas_roll_dps = 210.0;
                    break;
                case aircraft::AircraftType::F15EX_EAGLE_II:
                    a.gcas_roll_dps = 190.0;
                    break;
                case aircraft::AircraftType::EUROFIGHTER_TYPHOON:
                    a.gcas_roll_dps = 225.0;
                    a.gcas_g_tau_s  = 0.28;
                    break;
                case aircraft::AircraftType::F22_RAPTOR:
                    a.gcas_roll_dps = 200.0;
                    break;
                case aircraft::AircraftType::A10_THUNDERBOLT:
                    // Direct linkage: the roll rate grows with airspeed, and full
                    // aft stick runs out of elevator well short of CLmax (4.4 G at
                    // 150 m/s), so the plan uses less of the lift and less G.
                    a.gcas_roll_dps       = 95.0;
                    a.gcas_roll_ref_mps   = 150.0;
                    a.gcas_usable_lift    = 0.55;
                    a.gcas_hands_off_g    = 0.0;
                    a.gcas_recovery_g     = 4.5;
                    a.gcas_max_recovery_g = 6.0;
                    a.gcas_roll_tau_s     = 0.25;
                    break;
                case aircraft::AircraftType::F35B_LIGHTNING_II:
                    // Slower G onset at low q (0.57 s to 63% at 150 m/s).
                    a.gcas_roll_dps = 175.0;
                    a.gcas_g_tau_s  = 0.42;
                    break;
            }
            return a;
        }
    };

    Airframe airframe{Airframe::from(aircraft::AircraftType::F16_FIGHTING_FALCON)};

    // =========================================================================
    // Interface
    // =========================================================================

    constexpr OnBoardFlightComputer() noexcept = default;

    explicit constexpr OnBoardFlightComputer(aircraft::AircraftType type) noexcept
        : airframe(Airframe::from(type)) {}

    /// @brief Adopt a different airframe's limits (call on every aircraft change).
    /// Any recovery in progress was planned for the old jet, so state is reset too.
    void configure(aircraft::AircraftType type) noexcept {
        airframe = Airframe::from(type);
        reset();
    }

    /// @brief Reset all OFC state (call when the simulation resets).
    void reset() noexcept {
        gloc_state           = GlocState::NORMAL;
        gcas_state           = GcasState::ARMED;
        tumble_state         = TumbleState::ARMED;
        g_exposure           = 0.0;
        gloc_timer           = 0.0;
        gloc_duration        = 0.0;
        gloc_authority       = 1.0;
        blackout_fraction    = 0.0;
        gcas_tti             = 999.0;
        gcas_margin_m        = 999.0;
        tumble_persist_timer = 0.0;
        departure_rotation   = false;
        gcas_throttle_cmd    = -1.0;
        gcas_roll_dir        = 0;
        gloc_roll_dir        = 0;
        gcas_nz_trim         = 0.0;
        gloc_nz_trim         = 0.0;
        gcas_escape_bank     = 0.0;
        gcas_escape_g        = 0.0;
        gcas_escape_climb    = GCAS_MAX_CLIMB_RAD;
        gcas_escape_valley   = false;
        gcas_escape_turn_g   = 0.0;
        gcas_escape_hint     = 0;
        gcas_escape_g_cap    = 0.0;
        gcas_eval_skip       = 0;
        gcas_pulled          = false;
        gcas_last_ditch      = false;
        gcas_inhibited       = false;
        gcas_relaxed_s       = 0.0;
        gcas_switch_hold_s   = 0.0;
        gloc_stick_relaxed   = false;
        gloc_dead_hand       = false;
        gloc_wait_s          = 0.0;
    }

    // -------------------------------------------------------------------------
    // Status accessors (convenience, named after plan interface)
    // -------------------------------------------------------------------------
    [[nodiscard]] bool   is_gloc_active()    const noexcept { return gloc_state == GlocState::GLOC || gloc_state == GlocState::RECOVERY; }
    [[nodiscard]] bool   is_gcas_active()    const noexcept { return gcas_state == GcasState::WING_LEVEL || gcas_state == GcasState::PULL_UP; }
    [[nodiscard]] bool   is_tumble_active()  const noexcept { return tumble_state == TumbleState::RECOVERING; }
    [[nodiscard]] double blackout_fraction_value() const noexcept { return blackout_fraction; }
    [[nodiscard]] double gcas_tti_seconds()  const noexcept { return gcas_tti; }
    [[nodiscard]] double gcas_margin_meters() const noexcept { return gcas_margin_m; }
    [[nodiscard]] double g_exposure_value()  const noexcept { return g_exposure; }
    [[nodiscard]] bool   has_throttle_override() const noexcept { return gcas_throttle_cmd >= 0.0; }
    [[nodiscard]] double throttle_override_value() const noexcept { return gcas_throttle_cmd; }

    /**
     * @brief Main OFC update — intercepts pilot commands and returns safe commands.
     *
     * Call this every simulation timestep (200 Hz) after IMU is updated but
     * before the FLCS update.  When neither safety system is engaged the
     * returned commands are bit-identical to @p pilot_in.
     *
     * @param dt          Simulation timestep [s].
     * @param state       Current 6-DoF flight state.
     * @param imu         Current IMU sensor data (Nz, altitude, airspeed, …).
     * @param pilot_in    Raw pilot inceptor commands from InputManager.
     * @param gear_down   True while the landing gear is deployed. Used only to tell
     *                    an intentional landing flare/rollout apart from an
     *                    emergency low-altitude descent — Auto-GCAS still flies
     *                    the predicted recovery and the 30 ft hard deck whenever
     *                    the gear is up, no matter how gentle the sink rate looks.
     * @param powered_lift True while a STOVL jet's lift system is engaged (F-35B
     *                    converted or converting). Jet-borne flight is slow, at
     *                    large flight-path angles and close to the ground by
     *                    design: Auto-GCAS and the departure recovery would read
     *                    a hover or a vertical landing as an emergency and fight
     *                    the unified STOVL law, so both stand down (as the
     *                    F-35's Auto-GCAS does in STOVL mode). GLOC monitoring
     *                    carries on.
     * @return            Effective commands to pass to the FLCS.
     *
     * Auto-GCAS additionally drives @ref gcas_throttle_cmd this tick (see
     * @ref has_throttle_override / @ref throttle_override_value) — the caller is
     * responsible for feeding that into the engine model instead of the pilot's
     * throttle lever while it is set.
     */
    [[nodiscard]] PilotCommands update(
        double dt,
        const fdm::FlightState& state,
        const IMUData& imu,
        const PilotCommands& pilot_in,
        bool gear_down = false,
        bool powered_lift = false
    ) noexcept {
        PilotCommands effective = pilot_in;
        gcas_throttle_cmd = -1.0;

        if (powered_lift) {
            step_gloc(dt, state, imu, pilot_in, effective);
            stand_down_gcas();
            tumble_state = TumbleState::ARMED;
            tumble_persist_timer = 0.0;
            departure_rotation = false;
            return effective;
        }

        // Fixed authority order.  Each stage may only be handed the *raw* pilot
        // inceptor positions to judge pilot intent (@p pilot_in) — never the
        // running @p effective commands, which by then may be the OFC's own
        // output and would otherwise be mistaken for a conscious pilot input.
        //
        // Departure recovery runs before Auto-GCAS so that GCAS can see an
        // already-departed airframe and stand down: its predicted recovery assumes
        // a controlled pull is achievable, which is false in a spin or deep stall.
        // Conversely, once GCAS *is* recovering it keeps the pitch and roll axes
        // (a nose-down command near the ground is unconditionally fatal), while
        // the anti-spin rudder from the tumble law is left in place to help it.
        step_gloc(dt, state, imu, pilot_in, effective);
        step_tumble(dt, state, pilot_in, effective);
        step_gcas(dt, state, imu, pilot_in, effective, gear_down);

        return effective;
    }

private:

    static constexpr double G0  = 9.80665;
    static constexpr double DEG = M_PI / 180.0;

    // =========================================================================
    // Shared recovery helpers
    // =========================================================================

    /// Half-width of the ambiguous zone either side of inverted, within which the
    /// roll direction is latched rather than recomputed from the Euler angle.
    static constexpr double ROLL_AMBIGUOUS_RAD = 170.0 * (M_PI / 180.0);

    /// @brief Wrap an angle to [-pi, pi].
    [[nodiscard]] static double wrap_pi(double a) noexcept {
        return std::remainder(a, 2.0 * M_PI);
    }

    /**
     * @brief Roll stick that captures a bank angle, stable across the +/-180 deg wrap.
     *
     * A naive `(target - roll) / gain` law saturates to -1 at roll = +179 deg and
     * to +1 at roll = -179 deg.  Near inverted the Euler roll angle flips between
     * those two every few ticks, so the command dithers full-scale left/right and
     * the jet never actually rolls upright — it just shakes while the nose falls.
     * Inside the ambiguous zone a direction is therefore chosen once and held until
     * the bank error is unambiguous again; both directions are equally short from
     * there, so the only thing that matters is that the choice stays put.
     *
     * @param roll         Bank angle [rad], in [-pi, +pi].
     * @param target       Bank to capture [rad].
     * @param gain         Bank error at which the command saturates [rad].
     * @param latched_dir  In/out latch state (0 = unlatched).
     * @param lead         Bank the present roll rate will add before the jet
     *                     answers a reversal [rad] (rate damping).
     */
    [[nodiscard]] static double bank_capture_roll_cmd(
        double roll,
        double target,
        double gain,
        int& latched_dir,
        double lead = 0.0
    ) noexcept {
        const double err = wrap_pi(target - roll);
        if (std::abs(err) >= ROLL_AMBIGUOUS_RAD) {
            if (latched_dir == 0) {
                latched_dir = (err >= 0.0) ? +1 : -1;
            }
            return static_cast<double>(latched_dir);
        }
        latched_dir = 0;
        return std::clamp((err - lead) / gain, -1.0, 1.0);
    }

    [[nodiscard]] static double wings_level_roll_cmd(double roll, double gain, int& latched_dir) noexcept {
        return bank_capture_roll_cmd(roll, 0.0, gain, latched_dir);
    }

    /// Bank angle up to which the recovery G is commanded in full [rad].
    static constexpr double PULL_FADE_START_RAD = 135.0 * (M_PI / 180.0);
    /// Bank angle beyond which no pull is commanded at all [rad].
    static constexpr double PULL_FADE_END_RAD   = 165.0 * (M_PI / 180.0);

    /**
     * @brief Fraction of the recovery G to command at a given bank angle.
     *
     * Only the cos(bank) share of the lift vector opposes the descent, so past 90 deg
     * a nose-up pull has a component driving the nose *toward* the terrain.  Held
     * while the jet is inverted, that is exactly how an automatic recovery turns into
     * a crash, so the pull is faded out — to a 0 G unload, not merely to a centred
     * stick, which on a G-command law is still a 1 G pull — and the roll is allowed
     * to finish first.
     *
     * The fade deliberately starts well past 90 deg rather than at the geometric
     * crossover.  Closed-loop runs against the 6-DoF model show that between roughly
     * 90 and 135 deg of bank the pull is still clearly worth commanding: the pitch
     * loop is slow to build alpha, so pre-loading it while the wings are coming level
     * buys more clearance than the brief downward component costs — gating at 90 deg
     * instead gave up ~10 m of terrain clearance across that band.  Past ~165 deg the
     * balance inverts sharply and the pull becomes the dominant hazard; gating there
     * gains 15-20 m in the inverted split-S case and turns one low-speed inverted
     * scenario from a ground impact into a recovery.
     *
     * Note this is a fade between two angles, not a proportional cos(bank) scaling:
     * scaling the command would make the achieved vertical acceleration fall off as
     * cos^2 and needlessly weaken the recovery at moderate bank.
     */
    [[nodiscard]] static double bank_pull_scale(double roll) noexcept {
        const double upper = std::cos(PULL_FADE_START_RAD); // full pull at or below
        const double lower = std::cos(PULL_FADE_END_RAD);   // no pull at or above
        return std::clamp((std::cos(roll) - lower) / (upper - lower), 0.0, 1.0);
    }

    /**
     * @brief Bank of the lift vector about the velocity vector [rad] (+ right wing down).
     *
     * Euler roll is bank about the body axis, which drifts from the lift vector's
     * bank once alpha is large; the trajectory the recovery flies is set by where
     * the lift points.  In a near-vertical dive no bank is defined and every
     * direction of pull raises the nose, so it reads as wings level.
     */
    [[nodiscard]] static double lift_bank(const fdm::FlightState& state, const math::Vector3& vel_ned) noexcept {
        const double v = vel_ned.norm();
        if (v < 1.0) return state.euler_angles().roll;
        const math::Vector3 vh = vel_ned / v;
        const math::Vector3 up(0.0, 0.0, -1.0);
        const double up_along = up.dot(vh);
        if (std::abs(up_along) > 0.995) return 0.0;
        const math::Vector3 u = (up - vh * up_along).normalized();
        const math::Vector3 r = vh.cross(u);
        const math::Vector3 lift = -state.q_att.rotate_body_to_ned(math::Vector3(0.0, 0.0, 1.0));
        return std::atan2(lift.dot(r), lift.dot(u));
    }

    // =========================================================================
    // GLOC subsystem
    // =========================================================================

    /**
     * @brief Advance the GLOC state machine one timestep.
     *
     * The G-exposure accumulator uses separate rate terms for each G-band to
     * approximate the non-linear onset seen in centrifuge studies:
     *   < 6.5 G   : passive recovery only            (GLOC_BASE_TOLERANCE_G)
     *   6.5–8.2 G : slow onset (greyout band)        (GLOC_RAPID_ONSET_G)
     *   8.2–9.5 G : moderate onset                   (GLOC_IMMEDIATE_G)
     *   > 9.5 G   : rapid/immediate onset
     *
     * @param pilot_in    Raw pilot inceptor positions, used only to judge intent.
     * @param effective   In/out: will be overridden if pilot is incapacitated.
     */
    void step_gloc(
        double dt,
        const fdm::FlightState& state,
        const IMUData& imu,
        const PilotCommands& pilot_in,
        PilotCommands& effective
    ) noexcept {
        const double Nz = imu.Nz; // positive = sustained upward load on pilot

        // --- Accumulate / recover G exposure --------------------------------
        if (Nz > GLOC_IMMEDIATE_G) {
            g_exposure += 5.0 * (Nz - GLOC_IMMEDIATE_G) * dt;
        } else if (Nz > GLOC_RAPID_ONSET_G) {
            g_exposure += 2.0 * (Nz - GLOC_RAPID_ONSET_G) * dt;
        } else if (Nz > GLOC_BASE_TOLERANCE_G) {
            g_exposure += 0.5 * (Nz - GLOC_BASE_TOLERANCE_G) * dt;
        } else {
            // Below tolerance: passive recovery (exponential decay with fixed rate)
            g_exposure -= GLOC_RECOVERY_RATE * dt;
            if (g_exposure < 0.0) g_exposure = 0.0;
        }
        g_exposure = std::min(g_exposure, GLOC_TRIP_THRESHOLD * 2.0); // cap accumulator

        // --- Advance state machine ------------------------------------------
        switch (gloc_state) {

            case GlocState::NORMAL:
                if (g_exposure > 2.0) {
                    gloc_state = GlocState::GREYOUT;
                }
                gloc_authority = 1.0;
                break;

            case GlocState::GREYOUT:
                if (g_exposure <= 1.0) {
                    gloc_state = GlocState::NORMAL;
                } else if (g_exposure > 5.5) {
                    gloc_state = GlocState::BLACKOUT;
                }
                gloc_authority = 1.0; // pilot still has full stick authority
                break;

            case GlocState::BLACKOUT:
                if (g_exposure <= 2.0) {
                    gloc_state = GlocState::GREYOUT;
                } else if (g_exposure >= GLOC_TRIP_THRESHOLD) {
                    // Trip into full GLOC
                    gloc_state    = GlocState::GLOC;
                    gloc_timer    = 0.0;
                    gloc_stick_relaxed = false;
                    gloc_dead_hand = true;
                    gloc_nz_trim  = 0.0;
                    // Randomise unconscious duration using a simple LCG seeded by sim state
                    // (deterministic, no stdlib random needed).  std::fmod keeps the sign
                    // of its dividend, so west/south of the origin the raw fraction is
                    // negative and would shorten the blackout below GLOC_MIN_DURATION_S.
                    const double seed_frac =
                        std::abs(std::fmod(state.pos_ned.x * 0.001 + g_exposure, 1.0));
                    gloc_duration = GLOC_MIN_DURATION_S
                                  + seed_frac * (GLOC_MAX_DURATION_S - GLOC_MIN_DURATION_S);
                    gloc_authority = 0.0;
                }
                break;

            case GlocState::GLOC:
                gloc_timer += dt;
                gloc_authority = 0.0;
                if (gloc_timer >= gloc_duration) {
                    gloc_state = GlocState::RECOVERY;
                    gloc_timer = 0.0;
                    gloc_wait_s = 0.0;
                    // G exposure has dropped by now; clamp to a low residual
                    g_exposure = std::min(g_exposure, 2.0);
                }
                break;

            case GlocState::RECOVERY: {
                // The authority ramp runs once the autopilot has the jet settled
                // (wings level, on its shallow climb): handed back mid-recovery, a
                // pilot still coming round flies hands-off whatever dive the jet is
                // in — an F-35B left at -6 deg sank 900 m into Auto-GCAS's arms.
                const auto e = state.euler_angles();
                const math::Vector3 v = state.velocity_ned();
                const double fpa = std::atan2(-v.z, std::max(std::hypot(v.x, v.y), 1.0));
                const bool settled = std::abs(e.roll) < GLOC_SETTLED_BANK_RAD
                                  && std::abs(fpa - GLOC_TARGET_FPA_RAD) < GLOC_SETTLED_FPA_RAD;
                gloc_wait_s += dt;
                if (settled || gloc_wait_s > GLOC_SETTLE_MAX_S) {
                    gloc_timer += dt;
                }
                gloc_authority = std::min(1.0, gloc_timer / GLOC_AUTHORITY_RAMP_S);
                // The ramp must actually elapse.  g_exposure is clamped to 2.0 on
                // entry and decays at GLOC_RECOVERY_RATE (5.0/s), so an `||` on the
                // exposure alone ended RECOVERY after ~0.1 s and skipped the ramp
                // entirely, snapping full authority back to a still-groggy pilot.
                if (gloc_timer >= GLOC_AUTHORITY_RAMP_S) {
                    gloc_state = GlocState::NORMAL;
                    gloc_authority = 1.0;
                }
                break;
            }
        }

        // --- Compute blackout screen-fade fraction ---------------------------
        //   NORMAL / GREYOUT : 0 → light linear fade (peripheral tunnel)
        //   BLACKOUT         : 0.5 → 0.9
        //   GLOC             : 1.0 (full black)
        //   RECOVERY         : linearly clears with authority
        {
            double target_fade = 0.0;
            switch (gloc_state) {
                case GlocState::NORMAL:
                    target_fade = 0.0;
                    break;
                case GlocState::GREYOUT:
                    target_fade = 0.1 + 0.3 * std::clamp((g_exposure - 2.0) / 3.5, 0.0, 1.0);
                    break;
                case GlocState::BLACKOUT:
                    target_fade = 0.5 + 0.4 * std::clamp((g_exposure - 5.5) / 4.5, 0.0, 1.0);
                    break;
                case GlocState::GLOC:
                    target_fade = 1.0;
                    break;
                case GlocState::RECOVERY:
                    target_fade = 1.0 - gloc_authority;
                    break;
            }
            // Smooth the visual transition so it isn't jarring
            const double fade_alpha = 1.0 - std::exp(-dt / 0.15); // fast visual clearing
            blackout_fraction += (target_fade - blackout_fraction) * fade_alpha;
            blackout_fraction  = std::clamp(blackout_fraction, 0.0, 1.0);
        }

        // A hand still clamped on the stick from the pull that knocked the pilot
        // out is not flying: until the stick has been let go once, the pitch it
        // commands is held below the G-LOC threshold (also after the hand-back),
        // so the jet is not driven straight back into the blackout.
        const bool stick_centred = std::abs(pilot_in.pitch_stick) <= 0.15
                                && std::abs(pilot_in.roll_stick) <= 0.15;
        if (stick_centred) gloc_dead_hand = false;
        if (gloc_dead_hand) {
            effective.pitch_stick = std::min(effective.pitch_stick, stick_for_g(GLOC_RAMP_G_LIMIT, imu.q_bar));
        }

        // --- FLCS command override when pilot is incapacitated --------------
        if (gloc_state == GlocState::GLOC || gloc_state == GlocState::RECOVERY) {
            const auto euler = state.euler_angles();
            // Immediate pilot handover: If G load has reduced to safe level (<3G)
            // and aircraft is roughly upright, any deliberate pilot stick deflection
            // demonstrates consciousness and instantly returns 100% control to the pilot.
            // Judged on the *raw* inceptor positions: `effective` may already carry
            // the OFC's own recovery command, which would read as a conscious pilot.
            //
            // The deflection must also be a *new* one.  A pilot who greys out mid-pull
            // is still holding the stick aft, and counting that as a wake-up handed a
            // 9 G pull straight back within a fraction of a second, over and over.
            const bool stick_deflected = std::abs(pilot_in.pitch_stick) > 0.15
                                      || std::abs(pilot_in.roll_stick)  > 0.15;
            if (!stick_deflected) {
                gloc_stick_relaxed = true;
            }
            if (gloc_stick_relaxed && imu.Nz < 3.0
                && std::abs(euler.roll) < (35.0 * M_PI / 180.0)) {
                if (stick_deflected) {
                    gloc_state     = GlocState::NORMAL;
                    gloc_authority = 1.0;
                    g_exposure     = 0.0;
                    gloc_roll_dir  = 0;
                    gloc_nz_trim   = 0.0;
                    gloc_dead_hand = false;
                }
            }

            if (gloc_authority < 1.0) {
                const PilotCommands ofc_cmd = gloc_recovery_command(dt, state, imu);
                // Blend OFC authority into the pilot commands.
                const double pilot_pitch = effective.pitch_stick;
                const double pilot_share = gloc_authority;
                const double ofc_share   = 1.0 - gloc_authority;
                effective.pitch_stick  = pilot_share * pilot_pitch
                                       + ofc_share   * ofc_cmd.pitch_stick;
                effective.roll_stick   = pilot_share * effective.roll_stick
                                       + ofc_share   * ofc_cmd.roll_stick;
            }
        }
    }

    /**
     * @brief Compute OFC recovery commands during GLOC.
     *
     * Roll wings-level first, unloaded: past 90 deg of bank a pull — even the
     * 1 G a centred stick holds on a G-command law — drives the nose toward the
     * ground, and an unloaded jet also rolls fastest.  Once nearly wings level,
     * capture a shallow climb with a G command closed on the measured Nz, limited
     * well below the G that caused the blackout.  The old law held a 3 deg pitch
     * *attitude* through the stick directly: on the direct-linkage A-10 and at
     * the F-35B's low-q gains that left the jet descending for 20-30 s after an
     * inverted G-LOC, until Auto-GCAS had to step in 50 m above the ground.
     */
    [[nodiscard]] PilotCommands gloc_recovery_command(
        double dt,
        const fdm::FlightState& state,
        const IMUData& imu
    ) noexcept {
        PilotCommands cmd{};

        const math::Vector3 vel_ned = state.velocity_ned();
        const double roll  = state.euler_angles().roll;  // radians, [-π, +π]
        const double V     = std::max(state.airspeed(), 30.0);
        const double gamma = std::atan2(-vel_ned.z, std::max(std::hypot(vel_ned.x, vel_ned.y), 1.0));

        cmd.roll_stick = wings_level_roll_cmd(roll, M_PI * 0.35, gloc_roll_dir);

        // Wings level: capture a shallow climb.  Rolling: an unloaded roll-out.
        const double n_climb = std::clamp(
            std::cos(gamma) + (GLOC_TARGET_FPA_RAD - gamma) * V / (G0 * GLOC_CAPTURE_S),
            0.0, airframe.gloc_max_g);
        const double n_roll = std::max(0.0, std::cos(roll));
        const double w = std::clamp((std::abs(roll) - GLOC_WINGS_LEVEL_RAD)
                                    / (GLOC_BLEND_RAD - GLOC_WINGS_LEVEL_RAD), 0.0, 1.0);
        const double n_cmd = (1.0 - w) * n_climb + w * std::min(n_roll, n_climb);

        const double q_s_over_w = imu.q_bar * airframe.wing_area_m2 / (airframe.ref_mass_kg * G0);
        cmd.pitch_stick = pitch_command_for_g(dt, state, imu, n_cmd, q_s_over_w, gloc_nz_trim,
                                              w < 1.0);
        cmd.rudder_pedal = 0.0;
        return cmd;
    }

    // =========================================================================
    // GCAS subsystem
    // =========================================================================

    /// One escape manoeuvre: the bank it rolls to and the G it pulls.
    struct Escape {
        double bank{0.0};                 ///< [rad], + right; 0 = wings level
        double g{5.0};                    ///< Recovery load factor
        double climb{GCAS_MAX_CLIMB_RAD}; ///< Climb it pulls to, then holds [rad]
        /// Valley-following escape: the bank is steered each step away from the
        /// higher side of the terrain ahead (see valley_attitude), starting from a
        /// horizontal load factor of @ref turn_g; @ref g is then the vertical share.
        bool valley{false};
        double turn_g{0.0};               ///< Horizontal load factor carried in [g], + right
        /// Most G a turning escape may use: the recovery ceiling, or the G the
        /// pilot is already pulling in the turn it carries on (up to the limit).
        double g_cap{0.0};
    };

    /// Bank and load factor an escape flies at one instant.
    struct EscapeAttitude {
        double bank{0.0};
        double g{1.0};
    };

    /**
     * @brief Attitude of the valley-following escape at (@p x, @p y, @p h) on
     * track @p chi.
     *
     * A canyon is a chain of bends no fixed escape turn follows for the seconds a
     * climb out of it takes.  This one reads the terrain either side of a point
     * GCAS_VALLEY_LOOK_S ahead and turns away from the higher side — toward the
     * floor, away from the wall of the coming bend — for as long as that terrain
     * still stands above the jet, on top of the turn already being flown.  The
     * prediction and the flown recovery both steer by it, tick for tick.
     */
    [[nodiscard]] EscapeAttitude valley_attitude(double x, double y, double h, double chi, double V,
                                                 const Escape& e) const noexcept {
        const double look = std::clamp(V * GCAS_VALLEY_LOOK_S, GCAS_VALLEY_LOOK_MIN_M, GCAS_VALLEY_LOOK_MAX_M);
        const double cx = std::cos(chi), cy = std::sin(chi);
        const double px = x + look * cx, py = y + look * cy;
        constexpr double w = GCAS_VALLEY_HALF_WIDTH_M;
        const double h_right = environment::GroundCollision::get_terrain_height(px - w * cy, py + w * cx);
        const double h_left  = environment::GroundCollision::get_terrain_height(px + w * cy, py - w * cx);
        double n_h = e.turn_g;
        if (std::max(h_right, h_left) > h - GCAS_VALLEY_CLEAR_M) {
            n_h -= GCAS_VALLEY_GAIN * (h_right - h_left) / (2.0 * w);
        }
        EscapeAttitude a;
        a.g = std::min(std::max(e.g_cap, airframe.gcas_max_recovery_g), std::hypot(n_h, e.g));
        a.bank = std::asin(std::clamp(n_h / a.g, -GCAS_VALLEY_MAX_TURN, GCAS_VALLEY_MAX_TURN));
        return a;
    }

    /// Flight condition the predictions start from (computed once per tick).
    struct Seed {
        double x{0.0}, y{0.0}, h{0.0}; ///< North, east [m]; altitude MSL [m]
        double V{0.0};                 ///< Airspeed [m/s]
        double gamma{0.0};             ///< Flight-path angle [rad]
        double chi{0.0};               ///< Track [rad]
        double bank{0.0};              ///< Lift-vector bank [rad]
        double p{0.0};                 ///< Bank rate [rad/s]
        double nz{1.0};                ///< Load factor now
        double n_avail_per_v2{0.0};    ///< Usable lift load factor per (m/s)^2
        double p_rec{1.0};             ///< Recovery roll rate [rad/s]
        double g_tau{0.3};             ///< G onset lag at this dynamic pressure [s]
        double pilot_hold_s{0.0};      ///< How long the pilot's own manoeuvre is credited [s]
        int roll_dir{0};               ///< Latched roll direction through inverted
    };

    /**
     * @brief Load factor a recovery commands at bank @p bank, flight path
     * @p gamma and airspeed @p V.
     *
     * The escape's G, faded out past 135 deg of bank to a 0 G unload, capped where
     * the climb (the steeper of GCAS_MAX_CLIMB_RAD and @p gamma_hold) is being
     * captured, and never more than the wing can give.  Shared by the prediction
     * and the flown recovery, so the two cannot drift apart.
     */
    [[nodiscard]] static double escape_load(double g, double bank, double gamma, double V,
                                            double gamma_hold, double n_avail) noexcept {
        double n = g * bank_pull_scale(bank);
        if (gamma > 0.0) {
            const double hold = (std::cos(gamma) + (gamma_hold - gamma) * V / (G0 * GCAS_CLIMB_CAPTURE_S))
                              / std::max(std::cos(bank), 0.3);
            n = std::min(n, std::max(hold, 0.0));
        }
        return std::min(n, n_avail);
    }

    /**
     * @brief Fly @p escape forward from @p s and return the lowest terrain
     * clearance along it [m].
     *
     * A point-mass trajectory in wind axes: for @p reaction_s the current bank
     * rate and load factor carry on, and for a pilot actively flying the jet his
     * bank and G are held a further Seed::pilot_hold_s; then the jet rolls toward the escape's bank at
     * the airframe's recovery roll rate (proportional inside GCAS_ROLL_BAND_RAD, as
     * the flown roll command is) while the load factor follows escape_load through
     * the G-onset lag.  Speed changes with the climb or dive (thrust taken to
     * balance drag), and the lift available with it.  The terrain is read under
     * every step; stepping stops at GCAS_PREDICT_MAX_S, once the path has climbed
     * above every terrain on the map after the GCAS_LOOKAHEAD_S minimum, or as
     * soon as the clearance has fallen below @p give_up_below (no use following a
     * known failure further).
     */
    [[nodiscard]] double predict_clearance(const Seed& s, const Escape& e, double reaction_s,
                                           double give_up_below) const noexcept {
        // Credit for a pilot who is flying the jet: only with the reaction time
        // in front, i.e. when deciding whether to start a recovery at all.
        const double hold_s = (reaction_s > 0.0) ? s.pilot_hold_s : 0.0;
        double x = s.x, y = s.y, h = s.h, V = s.V;
        double gamma = s.gamma, chi = s.chi, bank = s.bank, n = s.nz, p = s.p;
        int dir = s.roll_dir;
        const double p_rec = s.p_rec;
        const double roll_lag = 1.0 - std::exp(-GCAS_FINE_STEP_S / airframe.gcas_roll_tau_s);
        const double gamma_hold = std::max(e.climb, s.gamma);
        const double sample_ceiling = GCAS_TERRAIN_CEILING_M + GCAS_RELEASE_MARGIN_M + 2.0 * airframe.gcas_buffer_m;
        double min_clear = 1e9;   // lowest clearance read off the terrain
        double above = 1e9;       // lowest height above the terrain ceiling where none was read
        double target_bank = e.bank;
        double target_g = e.g;
        int steps = 0;
        double t = 0.0;
        while (t < GCAS_PREDICT_MAX_S) {
            const double dt = (t < GCAS_FINE_HORIZON_S) ? GCAS_FINE_STEP_S : GCAS_COARSE_STEP_S;
            if (t >= reaction_s && t < reaction_s + hold_s) {
                p = 0.0;  // the pilot's manoeuvre held: bank and G as they are
            } else if (t >= reaction_s) {
                // The valley escape re-reads the terrain every other step (0.2 s),
                // which is all the bank can follow anyway.
                if (e.valley && (steps++ % 2 == 0)) {
                    const EscapeAttitude va = valley_attitude(x, y, h, chi, V, e);
                    target_bank = va.bank;
                    target_g = va.g;
                }
                const double err = wrap_pi(target_bank - bank);
                double p_cmd;
                if (std::abs(err) >= ROLL_AMBIGUOUS_RAD) {
                    if (dir == 0) dir = (err >= 0.0) ? +1 : -1;
                    p_cmd = dir * p_rec;
                } else {
                    dir = 0;
                    p_cmd = std::clamp((err - GCAS_ROLL_LEAD_S * p) / GCAS_ROLL_BAND_RAD, -1.0, 1.0) * p_rec;
                }
                p += (p_cmd - p) * (dt > GCAS_FINE_STEP_S ? 1.0 - std::exp(-dt / airframe.gcas_roll_tau_s) : roll_lag);
                double n_cmd = escape_load(target_g, bank, gamma, V, gamma_hold, s.n_avail_per_v2 * V * V);
                // Pulling through a fast roll, the G laws hold their integrators
                // and relieve the G: the jet makes only part of the pull.
                if (std::abs(p) > GCAS_ROLLING_RATE_RAD_S && n_cmd > 1.0) {
                    n_cmd = 1.0 + GCAS_ROLLING_G_FRAC * (n_cmd - 1.0);
                }
                n += (n_cmd - n) * (1.0 - std::exp(-dt / s.g_tau));
            }
            bank = wrap_pi(bank + p * dt);

            const double cg = std::cos(gamma);
            const double gamma_new = std::clamp(gamma + G0 / V * (n * std::cos(bank) - cg) * dt,
                                                -0.5 * M_PI + 1e-3, 0.5 * M_PI - 1e-3);
            const double chi_new = chi + G0 / V * n * std::sin(bank) / std::max(cg, 0.2) * dt;
            const double gm = 0.5 * (gamma + gamma_new);
            const double cm = 0.5 * (chi + chi_new);
            x += V * std::cos(gm) * std::cos(cm) * dt;
            y += V * std::cos(gm) * std::sin(cm) * dt;
            h += V * std::sin(gm) * dt;
            V = std::max(30.0, V - G0 * std::sin(gm) * dt);
            gamma = gamma_new;
            chi = chi_new;
            t += dt;

            // Above the highest terrain anywhere the clearance is only known to be
            // at least the height above it: a bound, not a reading, so it must not
            // stand in for the minimum of a path that later dives below it.
            if (h < sample_ceiling) {
                min_clear = std::min(min_clear, h - environment::GroundCollision::get_terrain_height(x, y));
            } else {
                above = std::min(above, h - GCAS_TERRAIN_CEILING_M);
            }
            if (min_clear < give_up_below) break;
            if (t >= GCAS_LOOKAHEAD_S && gamma > 0.0 && h > sample_ceiling) break;
        }
        return (min_clear < 1e9) ? min_clear : above;
    }

    /**
     * @brief Predicted clearance [m] if the jet were handed back now and flown
     * hands-off (@p n_hands_off at the present bank) for GCAS_RELEASE_HORIZON_S.
     */
    [[nodiscard]] static double hands_off_clearance(const Seed& s, double n_hands_off) noexcept {
        double x = s.x, y = s.y, alt = s.h, V = s.V, gamma = s.gamma, chi = s.chi;
        double min_clear = 1e9;
        constexpr double dt = GCAS_COARSE_STEP_S;
        for (double t = 0.0; t < GCAS_RELEASE_HORIZON_S; t += dt) {
            const double cg = std::cos(gamma);
            const double gamma_new = std::clamp(gamma + G0 / V * (n_hands_off * std::cos(s.bank) - cg) * dt,
                                                -0.5 * M_PI + 1e-3, 0.5 * M_PI - 1e-3);
            const double chi_new = chi + G0 / V * n_hands_off * std::sin(s.bank) / std::max(cg, 0.2) * dt;
            const double gm = 0.5 * (gamma + gamma_new);
            const double cm = 0.5 * (chi + chi_new);
            x += V * std::cos(gm) * std::cos(cm) * dt;
            y += V * std::cos(gm) * std::sin(cm) * dt;
            alt += V * std::sin(gm) * dt;
            V = std::max(30.0, V - G0 * std::sin(gm) * dt);
            gamma = gamma_new;
            chi = chi_new;
            if (alt < GCAS_TERRAIN_CEILING_M + 2.0 * GCAS_RELEASE_MARGIN_M) {
                min_clear = std::min(min_clear, alt - environment::GroundCollision::get_terrain_height(x, y));
            }
        }
        return min_clear;
    }

    /// A climbing turn with @p turn_g of horizontal load factor toward @p side,
    /// the recovery G added in the vertical, within the ceiling.
    [[nodiscard]] Escape turn_escape(double side, double turn_g, double g_pull, double g_cap) const noexcept {
        Escape e{};
        e.g = std::min(g_cap, std::hypot(turn_g, g_pull));
        e.bank = side * std::asin(std::min(turn_g / e.g, std::sin(80.0 * DEG)));
        e.climb = GCAS_MAX_CLIMB_RAD;
        return e;
    }

    /// Escape choices, best first once evaluated.
    struct EscapeChoice {
        Escape escape{};
        double clearance{-1e9};
        int index{-1};  ///< Which of the candidate escapes (see best_escape)
    };
    static constexpr int GCAS_ESCAPE_COUNT = 7;
    /// Prediction decimation while armed: with this much margin [m] the escapes
    /// are re-flown only every n+1 ticks.  Even a 300 m/s dive closes 60 m of
    /// margin in no less than ~0.2 s, five times the longest gap.
    static constexpr double GCAS_SKIP_NEAR_M     = 60.0;
    static constexpr int    GCAS_SKIP_NEAR_TICKS = 3;
    static constexpr double GCAS_SKIP_FAR_M      = 200.0;
    static constexpr int    GCAS_SKIP_FAR_TICKS  = 7;

    /**
     * @brief The best of the candidate escapes from @p s.
     *
     * Wings level first; a climbing turn each way (toward the bank already held
     * first) only if wings level does not clear by @p good_enough — that is the
     * canyon case, and costs nothing in the open.  The turn in the direction of
     * a turn already being flown keeps that turn's G (up to the ceiling), so it
     * follows a bend at least as tightly as the pilot was.
     */
    [[nodiscard]] EscapeChoice best_escape(const Seed& s, double g_pull, double reaction_s,
                                           double good_enough) const noexcept {
        const double side = (s.bank >= 0.0) ? 1.0 : -1.0;
        // The turn already being flown, carried on: its horizontal G kept (so it
        // follows a bend exactly as tightly as the pilot was), the recovery G added
        // on top in the vertical, within the ceiling.  A canyon run is a sequence of
        // such turns; a fixed 60 deg escape turn either overshot the bend into the
        // inner wall or ran wide into the outer one.
        const double turn_g = (std::abs(s.bank) < 0.5 * M_PI + 10.0 * DEG)
                            ? std::max(0.0, s.nz) * std::sin(std::abs(s.bank)) : 0.0;
        const bool keeps_turn = turn_g > GCAS_MIN_TURN_G;
        // A pilot already pulling past the recovery ceiling to follow a bend keeps
        // his G: the jet is flying it now, and anything less runs wide.
        const double g_cap = std::max(airframe.gcas_max_recovery_g,
                                      std::min(s.nz, GCAS_TURN_LIMIT_FRAC * airframe.max_g));
        const Escape kept = turn_escape(side, turn_g, g_pull, g_cap);
        // ...and tightened, for a bend the pilot is still rolling into.
        const Escape tightened = turn_escape(side, turn_g + GCAS_TURN_TIGHTEN_G, g_pull, g_cap);
        const std::array<Escape, GCAS_ESCAPE_COUNT> escapes = {{
            // Wings level, the standard climb.
            {0.0, g_pull, GCAS_MAX_CLIMB_RAD, false, 0.0},
            // Straight ahead but steeper, for a slope the standard climb cannot outclimb.
            {0.0, g_pull, GCAS_STEEP_CLIMB_RAD, false, 0.0},
            // The turn being flown, climbing out of it.
            {kept.bank, keeps_turn ? kept.g : 0.0, GCAS_MAX_CLIMB_RAD, false, 0.0},
            {tightened.bank, keeps_turn ? tightened.g : 0.0, GCAS_MAX_CLIMB_RAD, false, 0.0},
            // Climbing out along the valley.
            {0.0, g_pull, GCAS_VALLEY_CLIMB_RAD, true, side * turn_g, g_cap},
            // Climbing turns either way.
            {side * GCAS_MAX_TURN_BANK_RAD, g_pull, GCAS_MAX_CLIMB_RAD, false, 0.0},
            {-side * GCAS_MAX_TURN_BANK_RAD, g_pull, GCAS_MAX_CLIMB_RAD, false, 0.0},
        }};
        // The escape that worked last time is tried first: flying a canyon, the
        // valley escape clears tick after tick and the rest need never be flown.
        EscapeChoice best;
        const int first = std::clamp(gcas_escape_hint, 0, GCAS_ESCAPE_COUNT - 1);
        for (int k = 0; k < GCAS_ESCAPE_COUNT; ++k) {
            const int i = (k == 0) ? first : (k <= first ? k - 1 : k);
            const Escape& e = escapes[static_cast<size_t>(i)];
            if (e.g <= 0.0) continue;  // no turn being flown to carry on
            // Not worth following once it is clearly worse than the best so far.
            // The valley escape steers by the terrain it reads as it goes, so the
            // flown one strays further from its prediction than a fixed escape
            // does: it is only counted on with extra clearance in hand.
            const double handicap = e.valley ? GCAS_VALLEY_EXTRA_M : 0.0;
            const double c = predict_clearance(s, e, reaction_s,
                                               ((k == 0) ? -1e6 : best.clearance - 1.0) + handicap) - handicap;
            if (c > best.clearance + 1.0 || best.index < 0) {
                best.escape = e;
                best.clearance = c;
                best.index = i;
                if (c > good_enough) break;
            }
        }
        return best;
    }

    /// @brief Return GCAS to monitoring with no recovery state left behind.
    void stand_down_gcas() noexcept {
        gcas_state       = GcasState::ARMED;
        gcas_roll_dir    = 0;
        gcas_tti         = 999.0;
        gcas_margin_m    = 999.0;
        gcas_nz_trim     = 0.0;
        gcas_pulled      = false;
        gcas_last_ditch  = false;
        gcas_eval_skip   = 0;
    }

    /**
     * @brief Advance the Auto-GCAS state machine one timestep.
     *
     * Every tick the candidate escapes are flown forward from the present state
     * (predict_clearance) with the system's reaction time in front of them; the
     * recovery starts when the best of them has no more than the terrain buffer
     * left.  Once started it flies that escape and re-predicts it each tick from
     * where the jet actually is, with no reaction time (it is already flying it):
     * if the flown recovery is falling short of its own prediction the commanded G
     * escalates toward the capped ceiling, and once the jet is climbing with the
     * clearance assured both ways (another recovery would still clear with margin,
     * and so does the hands-off path) it hands back.
     */
    void step_gcas(
        double dt,
        const fdm::FlightState& state,
        const IMUData& imu,
        const PilotCommands& pilot_in,
        PilotCommands& effective,
        bool gear_down
    ) noexcept {
        const double V = state.airspeed(); // true airspeed [m/s]
        if (V < 10.0) {
            // Disarm rather than just bailing out: returning while a recovery was
            // in progress used to leave gcas_state latched at WING_LEVEL/PULL_UP
            // forever, so is_gcas_active() reported a recovery that was issuing no
            // commands at all — which also suppressed the tumble law indefinitely.
            stand_down_gcas();
            return;
        }

        // Current terrain height directly below the aircraft [m MSL]
        const double current_terr_h = environment::GroundCollision::get_terrain_height(
            state.pos_ned.x, state.pos_ned.y);
        // Current Altitude Above Ground Level (AGL) [m]
        const double local_agl = std::max(0.0, state.altitude() - current_terr_h);

        const math::Vector3 vel_ned = state.velocity_ned();
        const double horiz_speed = std::sqrt(vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y);
        const double vert_speed  = vel_ned.z;  // NED: positive = descending
        // FPA in radians: negative means nose below horizon
        const double fpa = std::atan2(-vert_speed, std::max(horiz_speed, 1.0));

        // Ground / Runway touch-down lockout: when resting on gear or rolling on runway/ground,
        // do not trigger GCAS
        if (gear_down && (local_agl < 2.0 || state.pos_ned.z >= -current_terr_h - 1.8) && !is_gcas_active()) {
            stand_down_gcas();
            return;
        }

        // Straight-line time to impact (telemetry), and the one threat a landing
        // approach still answers to: descending into terrain that rises above the
        // ground being landed on.
        bool rising_terrain_impact = false;
        gcas_tti = (vert_speed > 0.5) ? (local_agl / vert_speed) : 999.0;
        if (state.altitude() - vert_speed * 5.0 < GCAS_TERRAIN_CEILING_M) {
            constexpr std::array<double, 6> scan_times = {0.5, 1.0, 1.5, 2.5, 3.5, 5.0};
            for (double tau : scan_times) {
                const double terr = environment::GroundCollision::get_terrain_height(
                    state.pos_ned.x + vel_ned.x * tau, state.pos_ned.y + vel_ned.y * tau);
                if (state.altitude() - vert_speed * tau <= terr) {
                    gcas_tti = std::min(gcas_tti, tau);
                    if (terr > current_terr_h + 10.0) rising_terrain_impact = true;
                }
            }
        }

        // Landing approach: gear down and sinking no faster than a landing does.
        // The real F-16 system is inhibited outright with the gear handle down;
        // here only an actual collision with rising terrain ahead is still acted
        // on.  A gear-down dive (sink rate far above any landing) is still fully
        // protected.
        const bool landing_approach = gear_down && vert_speed < GCAS_LANDING_SINK_MPS
                                   && fpa > GCAS_LANDING_FPA_RAD;

        // --- Recovery G for this airframe -----------------------------------
        const double g_nominal = airframe.gcas_recovery_g;
        const double g_ceiling = airframe.gcas_max_recovery_g;

        // Load factor the wing can actually produce right now.  Heavy (full-fuel)
        // reference mass keeps this conservative, and so does the maximum lift
        // coefficient falling with Mach.
        const auto air = environment::Atmosphere1976::compute(state.altitude(), V);
        const double n_lift_max = air.dynamic_pressure * airframe.wing_area_m2 * airframe.cl_max
                                * aero::CompressibleLift::clmax_mach_factor(air.mach_number)
                                / (airframe.ref_mass_kg * G0);

        // Check active pilot control.  These MUST be judged on the raw inceptor
        // positions: `effective` may already hold the GLOC autopilot's or the tumble
        // law's own output, and reading that back made the OFC treat its own stick
        // command as a conscious pilot recovery — handing a diving jet to an
        // unconscious pilot.  An unconscious pilot can never break out at all.
        const bool pilot_conscious = !is_gloc_active();
        const bool pilot_breakout  = pilot_conscious
                                  && (pilot_in.pitch_stick > GCAS_BREAKOUT_PITCH
                                      || std::abs(pilot_in.roll_stick) > GCAS_BREAKOUT_ROLL);
        // A pilot already pulling harder than the recovery would is credited with
        // that G: the recovery would keep it, so the prediction does too.
        const double g_pull = (pilot_conscious && pilot_in.pitch_stick > 0.25)
                            ? std::max(g_nominal, std::min(imu.Nz, g_ceiling)) : g_nominal;

        // After a breakout, wait for the stick to be let go — or for the jet to be
        // comfortably clear — before re-arming fully.
        if (gcas_inhibited) {
            const bool relaxed = (std::abs(pilot_in.pitch_stick) < GCAS_RELAXED_STICK
                                  && std::abs(pilot_in.roll_stick) < GCAS_RELAXED_STICK)
                              || gcas_margin_m > GCAS_RELEASE_MARGIN_M + GCAS_INHIBIT_CLEAR_M;
            gcas_relaxed_s = relaxed ? gcas_relaxed_s + dt : 0.0;
            if (gcas_relaxed_s >= GCAS_INHIBIT_RELAX_S) {
                gcas_inhibited = false;
                gcas_relaxed_s = 0.0;
            }
        }

        // --- The predicted recoveries ----------------------------------------
        Seed seed;
        seed.x = state.pos_ned.x;
        seed.y = state.pos_ned.y;
        seed.h = state.altitude();
        seed.V = V;
        seed.gamma = fpa;
        seed.chi = std::atan2(vel_ned.y, vel_ned.x);
        seed.bank = lift_bank(state, vel_ned);
        seed.p = state.omega_b.x;
        seed.nz = imu.Nz;
        seed.n_avail_per_v2 = airframe.gcas_usable_lift * n_lift_max / (V * V);
        seed.p_rec = airframe.gcas_roll_dps * DEG
                   * ((airframe.gcas_roll_ref_mps > 0.0)
                      ? std::clamp(V / airframe.gcas_roll_ref_mps, 0.5, 1.3) : 1.0);
        // A pilot with the stick out of centre is flying the jet: his manoeuvre is
        // carried on a little longer before the escapes are judged.  Following a
        // canyon bend that is what keeps a hard-flown run from tripping the
        // system; a manoeuvre heading for the ground only gets worse for it, so
        // that one is caught sooner, not later.
        seed.pilot_hold_s = (pilot_conscious && (std::abs(pilot_in.pitch_stick) > GCAS_ACTIVE_STICK
                                                 || std::abs(pilot_in.roll_stick) > GCAS_ACTIVE_STICK))
                          ? GCAS_PILOT_CREDIT_S : 0.0;
        seed.g_tau = airframe.gcas_g_tau_s
                   * std::clamp(std::sqrt(GCAS_G_ONSET_Q_REF_PA / std::max(air.dynamic_pressure, 1.0)),
                                GCAS_G_ONSET_SCALE_MIN, GCAS_G_ONSET_SCALE_MAX);
        seed.roll_dir = gcas_roll_dir;

        const double buffer = airframe.gcas_buffer_m;
        const double release_level = buffer + GCAS_RELEASE_MARGIN_M;

        // --- State transitions ----------------------------------------------
        switch (gcas_state) {

            case GcasState::ARMED:
            case GcasState::SAFE: {
                // Do not newly engage on top of a departure: the predicted recovery
                // assumes a controlled pull is achievable, which it is not in a spin
                // or deep stall.  Breaking the departure comes first.  That includes
                // a spin or tumble still inside the tumble law's 0.25 s persistence
                // filter — engaging there let GCAS grab the pitch axis first and hold
                // it.  High alpha *alone* does not stand GCAS down: its own pull puts
                // the jet there near the ground, where a nose-down stall break is fatal.
                if (is_tumble_active() || departure_rotation) {
                    gcas_state    = GcasState::ARMED;
                    gcas_roll_dir = 0;
                    gcas_margin_m = 999.0;
                    break;
                }
                bool trigger = false;
                EscapeChoice choice;
                if (landing_approach) {
                    trigger = rising_terrain_impact;
                    choice.escape = Escape{0.0, g_nominal, GCAS_MAX_CLIMB_RAD, false, 0.0};
                    gcas_margin_m = 999.0;
                } else {
                    const double trigger_level = buffer;
                    const bool hard_deck = local_agl < GCAS_HARD_DECK_M && vert_speed > 0.0;
                    // A jet that has moved further than flight allows since the last
                    // prediction (repositioned without a reset) is re-predicted at once.
                    const bool moved = (state.pos_ned - gcas_eval_pos).norm() > 0.25 * gcas_margin_m;
                    if (gcas_eval_skip > 0 && !hard_deck && !moved) {
                        // Far from needing a recovery: re-predicted only every few
                        // ticks (the margin cannot close in the meantime).
                        --gcas_eval_skip;
                    } else {
                        // Any escape clearing the trigger level settles it: no recovery.
                        choice = best_escape(seed, g_pull, airframe.gcas_reaction_s, trigger_level + 1.0);
                        gcas_escape_hint = std::max(0, choice.index);
                        gcas_margin_m = choice.clearance - buffer;
                        gcas_eval_pos = state.pos_ned;
                        trigger = choice.clearance <= trigger_level || hard_deck;
                        gcas_eval_skip = (gcas_margin_m > GCAS_SKIP_FAR_M) ? GCAS_SKIP_FAR_TICKS
                                       : (gcas_margin_m > GCAS_SKIP_NEAR_M) ? GCAS_SKIP_NEAR_TICKS : 0;
                    }
                }
                if (trigger) {
                    gcas_escape_bank = choice.escape.bank;
                    gcas_escape_g    = choice.escape.g;
                    gcas_escape_climb = choice.escape.climb;
                    gcas_escape_valley = choice.escape.valley;
                    gcas_escape_turn_g = choice.escape.turn_g;
                    gcas_escape_g_cap  = choice.escape.g_cap;
                    gcas_last_ditch  = gcas_inhibited;
                    gcas_pulled      = false;
                    gcas_nz_trim     = 0.0;
                    gcas_switch_hold_s = GCAS_ESCAPE_SWITCH_HOLD_S;
                    gcas_state = (std::abs(wrap_pi(gcas_escape_bank - seed.bank)) > GCAS_WINGS_LEVEL_RAD)
                               ? GcasState::WING_LEVEL : GcasState::PULL_UP;
                } else {
                    gcas_state = (gcas_state == GcasState::SAFE) ? GcasState::SAFE : GcasState::ARMED;
                    gcas_roll_dir = 0;
                }
                break;
            }

            case GcasState::WING_LEVEL:
            case GcasState::PULL_UP: {
                // The recovery being flown, re-predicted from where the jet is.
                const Escape flying{gcas_escape_bank, gcas_escape_g, gcas_escape_climb,
                                    gcas_escape_valley, gcas_escape_turn_g, gcas_escape_g_cap};
                double clearance = predict_clearance(seed, flying, 0.0, -1e6)
                                 - (gcas_escape_valley ? GCAS_VALLEY_EXTRA_M : 0.0);
                // Should the latched escape stop working (terrain the climbing turn
                // only now reveals), take a better one — by a clear margin only.
                gcas_switch_hold_s = std::max(0.0, gcas_switch_hold_s - dt);
                if (clearance < buffer && !landing_approach && gcas_switch_hold_s <= 0.0) {
                    const EscapeChoice alt = best_escape(seed, gcas_escape_g, 0.0, release_level);
                    if (alt.clearance > buffer && alt.clearance > clearance + GCAS_ESCAPE_SWITCH_M) {
                        gcas_escape_bank   = alt.escape.bank;
                        gcas_escape_g      = alt.escape.g;
                        gcas_escape_climb  = alt.escape.climb;
                        gcas_escape_valley = alt.escape.valley;
                        gcas_escape_turn_g = alt.escape.turn_g;
                        gcas_escape_g_cap  = alt.escape.g_cap;
                        clearance          = alt.clearance;
                        gcas_switch_hold_s = GCAS_ESCAPE_SWITCH_HOLD_S;
                    }
                }
                gcas_margin_m = clearance - buffer;

                // Hand back once the jet is climbing, the recovery has margin in
                // hand, and the path it would fly hands-off from here clears the
                // terrain for the whole look-ahead: released any earlier, it is
                // still on course for the terrain and the system comes straight back.
                const bool climbing = fpa > GCAS_SAFE_FPA_RAD && vert_speed <= 0.0;
                const bool release = climbing && clearance > release_level
                                  && hands_off_clearance(seed, airframe.gcas_hands_off_g) > release_level;
                // A breakout needs the save to be comfortably made (release level):
                // the arming check, a buffer lower, then has room before it would
                // fire again.
                const bool breakout = pilot_breakout && !gcas_last_ditch && clearance > release_level;
                if (release) {
                    gcas_state = GcasState::SAFE;
                } else if (breakout) {
                    // A deliberate stick input, and the save is no longer marginal:
                    // the pilot has the jet.  Until he lets go of the stick a recovery
                    // that comes back is a last-ditch save he cannot break out of.
                    gcas_state = GcasState::SAFE;
                    gcas_inhibited = true;
                    gcas_relaxed_s = 0.0;
                } else {
                    const double bank_err = std::abs(wrap_pi(gcas_escape_bank - seed.bank));
                    if (gcas_state == GcasState::WING_LEVEL && bank_err <= GCAS_WINGS_LEVEL_RAD) {
                        gcas_state = GcasState::PULL_UP;
                    } else if (gcas_state == GcasState::PULL_UP && bank_err > GCAS_REBANK_RAD) {
                        // The jet has rolled back out of the recovery bank (turbulence,
                        // a gear-down asymmetry, the pilot fighting the roll).  Go back
                        // to rolling instead of holding a pull that is no longer lifting
                        // the nose away from the ground.  The wider threshold gives
                        // hysteresis against chattering between the two phases.
                        gcas_state = GcasState::WING_LEVEL;
                    }
                }
                break;
            }
        }

        // --- Override pilot commands when GCAS is executing ----------------
        if (is_gcas_active()) {
            // Commanded G: the escape's own, escalating toward the capped ceiling
            // only as far as the flown recovery is falling short of the buffer
            // (or at once for a last-ditch save).
            const double shortfall = std::clamp(-gcas_margin_m / (buffer + GCAS_RELEASE_MARGIN_M), 0.0, 1.0);
            // No escalation while still rolling: a ceiling-G pull through a
            // full-rate roll is how the G overshoots once the roll stops.
            const double urgency = (gcas_state != GcasState::PULL_UP) ? 0.0
                                 : gcas_last_ditch ? 1.0 : shortfall;
            // The valley-following escape re-steers its bank from where the jet is.
            double escape_g_now = gcas_escape_g;
            if (gcas_escape_valley) {
                const Escape flying{gcas_escape_bank, gcas_escape_g, gcas_escape_climb, true, gcas_escape_turn_g,
                                    gcas_escape_g_cap};
                const EscapeAttitude va = valley_attitude(seed.x, seed.y, seed.h, seed.chi, V, flying);
                gcas_escape_bank = va.bank;
                escape_g_now = va.g;
            }
            const double g_escape = std::max(escape_g_now, g_nominal);
            const double g_target = g_escape + urgency * std::max(0.0, g_ceiling - g_escape);
            const double gamma_hold = std::max(gcas_escape_climb, std::min(fpa, 0.5 * M_PI - 0.1));
            // The same load law the prediction flies: faded past 135 deg of bank to
            // a 0 G unload, and eased off as the climb is captured.
            const double n_cmd = escape_load(g_target, seed.bank, fpa, V, gamma_hold, 1e9);
            const double pitch_cmd = pitch_command_for_g(
                dt, state, imu, n_cmd, n_lift_max / airframe.cl_max, gcas_nz_trim,
                gcas_state == GcasState::PULL_UP);

            effective.roll_stick = bank_capture_roll_cmd(seed.bank, gcas_escape_bank, GCAS_ROLL_BAND_RAD,
                                                         gcas_roll_dir, GCAS_ROLL_LEAD_S * state.omega_b.x);
            // A pull is a floor the pilot may pull harder than; an unload is a
            // ceiling (when inverted, a push is what raises the nose).
            effective.pitch_stick = (n_cmd >= 1.0) ? std::max(effective.pitch_stick, pitch_cmd)
                                                   : std::min(effective.pitch_stick, pitch_cmd);

            // Leave the anti-spin rudder from the tumble law in place if it is
            // running — killing the yaw helps the recovery rather than fighting it.
            if (!is_tumble_active()) {
                effective.rudder_pedal = 0.0;
            }

            // A slow jet cannot afford to shed energy: without it the recovery G is
            // simply not there, so it gets full power from the first tick.  Once the
            // pull has begun it keeps full power to the end of the recovery: the
            // throttle never flips back to idle if the roll is disturbed.
            const bool energy_limited = airframe.gcas_usable_lift * n_lift_max < 1.25 * g_nominal;
            if (gcas_state == GcasState::PULL_UP) gcas_pulled = true;
            if (!gcas_pulled && !energy_limited && !gcas_last_ditch) {
                // Cut toward idle: still rolling, so the elevator can't pull effectively
                // yet — don't let the dive keep accelerating into the ground while it rolls.
                gcas_throttle_cmd = GCAS_ROLL_THROTTLE;
            } else {
                // Full power: arresting the sink rate at a sane G still bleeds airspeed,
                // and a recovery that stalls the jet right back toward the ground would
                // defeat the entire point of the system.
                gcas_throttle_cmd = GCAS_PULLOUT_THROTTLE;
            }
        } else {
            gcas_roll_dir   = 0;
            gcas_nz_trim    = 0.0;
            gcas_pulled     = false;
            gcas_last_ditch = false;
        }
    }

    /**
     * @brief Open-loop stick for @p g on this airframe at dynamic pressure @p q_bar.
     */
    [[nodiscard]] double stick_for_g(double g, double q_bar) const noexcept {
        if (airframe.g_command_law) {
            return (g >= 1.0) ? (g - 1.0) / std::max(1.0, airframe.max_g - 1.0)
                              : (g - 1.0) / std::max(1.0, 1.0 - airframe.min_g);
        }
        const double q_s_over_w = q_bar * airframe.wing_area_m2 / (airframe.ref_mass_kg * G0);
        const double g_per_stick = std::max(1.0, GCAS_DIRECT_G_PER_QSW * q_s_over_w);
        const double ff = (g - 1.0) / g_per_stick;
        return std::clamp(ff < 0.0 ? ff / GCAS_DIRECT_PUSH_GAIN : ff, -1.0, 1.0);
    }

    /**
     * @brief Stick command that holds @p g_target on this airframe.
     *
     * Fly-by-wire laws command Nz from stick, so the G maps straight across
     * through the airframe's own G range (both ways: a 0 G unload is a push)
     * rather than assuming the F-16's 9 G, with a small integral trim for the
     * laws that settle short of their command.  A direct-linkage law (A-10) has
     * no G command at all: its stick moves the elevator and the G that produces
     * grows with q, so the old open-loop mapping pulled it to 10-12 G.  There the
     * command is a feed-forward lift estimate closed on the measured Nz, with an
     * alpha guard standing in for the limiter the airframe does not have.
     *
     * @param q_s_over_w  Dynamic pressure x wing area / reference weight [-].
     * @param nz_trim     In/out integral trim of the loop flying this command.
     * @param trim_on     Integrate the trim this tick (the G is broadly on).
     */
    [[nodiscard]] double pitch_command_for_g(
        double dt,
        const fdm::FlightState& state,
        const IMUData& imu,
        double g_target,
        double q_s_over_w,
        double& nz_trim,
        bool trim_on
    ) const noexcept {
        const double nz_err = g_target - imu.Nz;

        if (airframe.g_command_law) {
            const double ff = stick_for_g(g_target, imu.q_bar);
            // Trim only once the G is broadly on: during onset the error is large
            // and still falling on its own, and integrating it would overshoot.
            if (trim_on && std::abs(nz_err) < 1.5) {
                nz_trim = std::clamp(nz_trim + GCAS_NZ_TRIM_KI * nz_err * dt,
                                     -GCAS_NZ_TRIM_LIMIT, GCAS_NZ_TRIM_LIMIT);
            }
            return std::clamp(ff + nz_trim, -1.0, 1.0);
        }

        const double g_per_stick = std::max(1.0, GCAS_DIRECT_G_PER_QSW * q_s_over_w);
        double ff = (g_target - 1.0) / g_per_stick;
        if (ff < 0.0) ff /= GCAS_DIRECT_PUSH_GAIN;

        const double alpha_deg  = state.alpha() * (180.0 / M_PI);
        const double alpha_over = alpha_deg - (airframe.alpha_limit_deg - 2.0);
        if (trim_on && (alpha_over < 0.0 || nz_err < 0.0)) {
            nz_trim = std::clamp(nz_trim + (GCAS_DIRECT_NZ_KI / g_per_stick) * nz_err * dt,
                                 -0.5, 0.5);
        }
        double cmd = ff + (GCAS_DIRECT_NZ_KP / g_per_stick) * nz_err + nz_trim;
        if (alpha_over > 0.0) {
            cmd -= 0.08 * alpha_over;
        }
        return std::clamp(cmd, -1.0, 1.0);
    }

    // =========================================================================
    // Tumble / Spin Auto-Recovery subsystem
    // =========================================================================

    /**
     * @brief Detects out-of-control departure (deep stall, flat spin, violent tumble)
     * and autonomously applies optimal recovery controls to stabilize the airframe,
     * immediately returning authority once safe.
     */
    void step_tumble(
        double dt,
        const fdm::FlightState& state,
        const PilotCommands& pilot_in,
        PilotCommands& effective
    ) noexcept {
        const double V = state.airspeed();
        // Low-speed lockout only prevents *arming* (e.g. while stationary on the ground).
        // It must not abort a recovery already underway — a deep stall/flat spin is
        // precisely the condition where forward airspeed collapses toward zero.
        if (V < 10.0 && tumble_state != TumbleState::RECOVERING) {
            tumble_state = TumbleState::ARMED;
            tumble_persist_timer = 0.0;
            departure_rotation = false;
            return;
        }

        const double alpha_deg = state.alpha() * (180.0 / M_PI);
        const double beta_deg  = state.beta()  * (180.0 / M_PI);
        const double q_dps     = std::abs(state.omega_b.y) * (180.0 / M_PI);
        const double r_dps     = std::abs(state.omega_b.z) * (180.0 / M_PI);
        const double pitch_yaw_rate = std::sqrt(state.omega_b.y * state.omega_b.y + state.omega_b.z * state.omega_b.z);

        // Check if pilot is deliberately commanding a roll maneuver in this direction.
        // Judged on the raw inceptor: `effective` may already carry the GLOC
        // autopilot's wings-levelling command, which is not pilot intent at all.
        const bool intentional_roll = (std::abs(pilot_in.roll_stick) > 0.15) &&
                                      ((state.omega_b.x * pilot_in.roll_stick) > 0.0);

        // Alpha bands scale with the airframe.  Fixed at the F-16's numbers, the
        // detector declared a departure at 32 deg — inside the Typhoon's 35 deg and
        // the F-22's 65 deg carefree envelopes — and then slammed full nose-down
        // on a perfectly controlled max-alpha pull.  Floors keep the F-16 values.
        const double alpha_lim         = airframe.alpha_limit_deg;
        const double deep_stall_alpha  = std::max(32.0, alpha_lim + 7.0);
        const double stall_break_alpha = std::max(22.0, alpha_lim - 3.0);
        const double safe_alpha        = std::max(20.0, alpha_lim - 5.0);

        // A pitch rate the pilot is commanding, with alpha still inside the
        // airframe's own envelope, is a manoeuvre (TVC-assisted in the F-22), not a
        // tumble.  Mirrors the intentional-roll exclusion above.
        const bool intentional_pitch = (std::abs(pilot_in.pitch_stick) > 0.15)
                                    && ((state.omega_b.y * pilot_in.pitch_stick) > 0.0)
                                    && std::abs(alpha_deg) < alpha_lim + 5.0
                                    && r_dps < 40.0;

        // Departure conditions:
        // 1. Deep stall: alpha well past the airframe's limit (or < -20 deg) with low airspeed
        const bool is_deep_stall = (alpha_deg > deep_stall_alpha || alpha_deg < -20.0) && (V < 140.0);
        // Near the ground a stall is not broken nose-down: the dive that follows
        // needs more height than is left, and a hard pull-out at low speed (the
        // pilot's or Auto-GCAS's own) puts the jet in this alpha band anyway.
        // There the pilot and Auto-GCAS keep the jet; a spin or tumble still counts.
        const double agl = state.altitude() - environment::GroundCollision::get_terrain_height(
                                                  state.pos_ned.x, state.pos_ned.y);
        const bool stall_near_ground = agl < TUMBLE_STALL_BREAK_FLOOR_M;
        // 2. Flat / steep spin: high yaw rate (> 40 deg/s) + sideslip (> 14 deg)
        const bool is_spin = (r_dps > 40.0 && std::abs(beta_deg) > 14.0);
        // 3. Violent out-of-control tumble: pitch & yaw multi-axis rates exceed safe envelope
        //    (excludes intentional roll and pitch manoeuvres)
        const bool is_tumble = !intentional_roll && !intentional_pitch
                            && ((pitch_yaw_rate > 1.50) || (q_dps > 65.0 && is_deep_stall));

        departure_rotation = is_spin || is_tumble;

        if ((is_deep_stall && !stall_near_ground) || is_spin || is_tumble) {
            tumble_persist_timer += dt;
        } else {
            tumble_persist_timer = std::max(0.0, tumble_persist_timer - dt * 2.0);
        }

        // Arming / Engagement: Trigger if departure condition persists for > 0.25 s
        if (tumble_state == TumbleState::ARMED || tumble_state == TumbleState::SAFE) {
            if (tumble_persist_timer > 0.25) {
                tumble_state = TumbleState::RECOVERING;
            }
        }

        // Handover / Release: Restore pilot authority immediately once airframe is stabilized
        if (tumble_state == TumbleState::RECOVERING) {
            const bool alpha_safe = (alpha_deg >= -15.0 && alpha_deg <= safe_alpha);
            const bool beta_safe  = (std::abs(beta_deg) <= 12.0);
            const bool pitch_yaw_safe = (pitch_yaw_rate <= 0.35); // < 20 deg/s pitch/yaw rate
            const bool roll_safe  = intentional_roll || (std::abs(state.omega_b.x) <= 0.80);
            const bool speed_safe = (V >= 60.0);

            if (alpha_safe && beta_safe && pitch_yaw_safe && roll_safe && speed_safe) {
                tumble_state = TumbleState::SAFE;
                tumble_persist_timer = 0.0;
                return; // Release control back to pilot!
            }

            // --- Autonomous Anti-Tumble / Anti-Spin Flight Control Law ---
            // 1. Anti-spin rudder: oppose spin yaw rate, proportionally.
            //    A bang-bang law slammed to full opposite rudder at a mere 10 deg/s of
            //    yaw — inside the band the release test already calls stabilised — and
            //    so drove a fresh departure the other way instead of settling.
            effective.rudder_pedal = std::clamp(-state.omega_b.z * 1.5, -1.0, 1.0);

            // Pitch and roll stay with Auto-GCAS whenever it is recovering: near the
            // ground a nose-down command is unconditionally fatal, and GCAS's own
            // commanded pull is exactly what pushes alpha into the deep-stall band
            // that trips this detector in the first place.  The anti-spin rudder above
            // is still applied — killing the yaw helps the pull-out.
            if (is_gcas_active()) {
                return;
            }

            // 2. Anti-autorotation roll damping: oppose roll rate only if not pilot commanded
            if (!intentional_roll) {
                effective.roll_stick = std::clamp(-state.omega_b.x * 0.8, -1.0, 1.0);
            }

            // 3. Deep stall breakout pitch command:
            if (alpha_deg > stall_break_alpha) {
                // Positive deep stall: full nose-down pitch to break stall & restore
                // attached flow, but never into the ground (see stall_near_ground).
                effective.pitch_stick = (stall_near_ground && !departure_rotation)
                                      ? std::max(effective.pitch_stick, 0.0) : -1.0;
            } else if (alpha_deg < -12.0) {
                // Inverted stall: pull nose up toward relative wind
                effective.pitch_stick = 1.0;
            } else {
                // Alpha within boundary: pitch rate damping to stop pitching oscillation
                effective.pitch_stick = std::clamp(-state.omega_b.y * 1.5, -0.6, 0.6);
            }
        }
    }
};

} // namespace flcs
} // namespace fastjet
