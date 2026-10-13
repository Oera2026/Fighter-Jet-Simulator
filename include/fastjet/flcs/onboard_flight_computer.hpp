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
 * Implements two independent, real-world inspired safety systems:
 *
 *  1. GLOC Auto-Recovery (G-induced Loss Of Consciousness)
 *     Monitors the pilot's G-exposure with a simplified Franks/Goodwin physiological
 *     model. When exposure accumulates past the incapacitation threshold the FLCS takes
 *     autonomous control (wings-level + gentle climb) until the pilot regains
 *     full authority.  Visual blackout fraction is exposed for screen-space rendering.
 *
 *  2. Auto-GCAS (Automatic Ground Collision Avoidance System)
 *     Mirrors the NASA/AFRL/Lockheed system that has saved 11+ F-16 pilots since 2014,
 *     and its defining property: it is nuisance-free, so pilots fly 100 ft passes,
 *     terrain following and canyon runs with it ON.  Every tick it flies the
 *     recovery it would perform — the current manoeuvre carrying on through the
 *     system's reaction time, a roll at the achievable rate, then the airframe's
 *     recovery G (nominal 5 G, less when slow) to a capped climb — through a
 *     handful of fixed time steps, and reads the terrain under every point of it.
 *     It fires only at the last instant: when the predicted minimum clearance falls
 *     to the terrain buffer (30 ft by default, see @ref gcas_buffer_m).  Climb,
 *     turn and load already on the jet are part of the prediction, so a pilot who
 *     is already climbing over a ridge or holding a level turn is left alone.  Two
 *     recoveries are predicted: wings level, and a climbing turn that keeps the
 *     turn already being flown (the escape that follows a canyon bend; NASA's
 *     multi-trajectory scheme).  The better one is flown.  Recovery is a two-phase
 *     manoeuvre — roll to the recovery bank first, then pull — escalating toward a
 *     capped ceiling only if the height left or time-to-impact becomes critical
 *     (never the airframe's full G limit: both are scaled to the configured
 *     airframe).  It commands the throttle alongside the stick: idle while still
 *     rolling (unless the jet is too slow to spare the energy), then full power once
 *     pulling.  Past 135 deg of bank the pull is faded to a 0 G unload (see
 *     bank_pull_scale), because pulling while inverted drives the nose into the
 *     terrain.  It hands back once the jet is climbing with terrain clearance
 *     assured, or to a deliberate pilot input once the jet is out of danger; after
 *     such a breakout it re-engages only for a last-ditch save (which then cannot
 *     be broken out of), so a pilot flying low on purpose is not fought tick by tick.
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

    // --- GCAS thresholds ----------------------------------------------------
    /// Default terrain buffer [m] (= 30 ft): the predicted recovery must keep at
    /// least this much clearance, or it is started now.  It doubles as the floor
    /// the system lets the pilot fly at: 50 ft passes are clear of it, a jet
    /// settling below 30 ft is flown back up.  See @ref gcas_buffer_m.
    static constexpr double GCAS_DEFAULT_BUFFER_M = 30.0 * 0.3048;
    /// Extra predicted clearance [m] needed before a recovery hands back (or a
    /// pilot may break out).  Hysteresis: released just above the buffer, the
    /// next tick's prediction would fire again.
    static constexpr double GCAS_RELEASE_MARGIN_M = 10.0;
    /// Time the current manoeuvre is predicted to carry on before the recovery
    /// starts to act [s]: decision, stick-to-surface and actuator lag.
    static constexpr double GCAS_REACTION_S       = 0.20;
    /// First-order lag of the load factor toward the recovery command [s]: the
    /// pitch loop's G onset as flown closed-loop by these flight control laws.
    static constexpr double GCAS_G_ONSET_TAU_S    = 0.35;
    /// The predicted recovery pulls no steeper than this climb, then holds it [rad].
    /// Shared with the flown recovery, which captures it in GCAS_CLIMB_CAPTURE_S.
    static constexpr double GCAS_MAX_CLIMB_RAD    = 30.0 * (M_PI / 180.0);
    static constexpr double GCAS_CLIMB_CAPTURE_S  = 1.0;
    /// The prediction always looks at least this far ahead (rising terrain), and
    /// at most this far (a slow pull out of a steep dive) [s].
    static constexpr double GCAS_LOOKAHEAD_S      = 6.0;
    static constexpr double GCAS_PREDICT_MAX_S    = 14.0;
    /// Turn (horizontal load factor [g]) below which the climbing-turn escape is
    /// not worth predicting separately, and the bank it may use at most [rad].
    static constexpr double GCAS_MIN_TURN_G       = 0.3;
    static constexpr double GCAS_MAX_TURN_BANK_RAD = 60.0 * (M_PI / 180.0);
    /// After a pilot breakout the system stays inhibited until the stick has been
    /// relaxed this long [s]; meanwhile only a last-ditch save (half the buffer)
    /// re-engages it.
    static constexpr double GCAS_INHIBIT_RELAX_S  = 1.0;
    /// Nominal recovery G used for both the predicted recovery and the commanded
    /// pitch during recovery — the two stay consistent by construction.
    static constexpr double GCAS_RECOVERY_G       = 5.0;
    /// Escalation ceiling for commanded G when time-to-impact is critical. Kept well
    /// below the airframe G limit so the save itself can't cause a structural
    /// overstress or induce GLOC.
    static constexpr double GCAS_MAX_RECOVERY_G   = 7.5;
    /// Nominal and ceiling recovery G are also capped to these fractions of the
    /// configured airframe's own positive G limit, so a 7.33 G A-10 is never asked
    /// for the 7.5 G a 9 G fighter can afford.
    static constexpr double GCAS_RECOVERY_G_FRAC  = 0.70;
    static constexpr double GCAS_MAX_G_FRAC       = 0.86;
    /// Time-to-impact below which commanded G ramps from nominal toward the ceiling [s].
    static constexpr double GCAS_CRITICAL_TTI_S   = 1.2;
    /// Fraction of the airframe's rated roll rate budgeted for rolling wings level.
    /// The rated figure is a limiter ceiling reached only at high q, after the roll
    /// has accelerated; the recovery has to plan on what it will actually get.
    static constexpr double GCAS_ROLL_RATE_FRAC   = 0.45;
    /// Time from engagement until the commanded G is actually on the jet [s]:
    /// stick-to-surface, actuator and pitch-loop lag. The dive continues unabated
    /// for this long, so its altitude is budgeted on top of the pull-out arc.
    static constexpr double GCAS_G_ONSET_S        = 0.70;
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
    /// Safety margin applied on top of the kinematic pull-out altitude.
    static constexpr double GCAS_SAFETY_MARGIN    = 1.15;
    /// Absolute minimum floor added to the kinematic pull-out altitude [m].
    static constexpr double GCAS_ALTITUDE_FLOOR_M = 35.0;
    /// Roll angle below which GCAS switches from wing-levelling to pull-up [rad].
    static constexpr double GCAS_WINGS_LEVEL_RAD  = 25.0 * (M_PI / 180.0);
    /// Roll angle above which an in-progress pull-up reverts to wing-levelling [rad].
    /// Wider than GCAS_WINGS_LEVEL_RAD so the two phases cannot chatter against
    /// each other while the jet settles through the band.
    static constexpr double GCAS_REBANK_RAD       = 40.0 * (M_PI / 180.0);
    /// Minimum descent angle below which terrain closure isn't even considered [rad].
    static constexpr double GCAS_ARM_FPA_RAD      = -2.0 * (M_PI / 180.0);
    /// Flight-path angle above which GCAS declares safe and disengages [rad] (0 deg = level flight).
    static constexpr double GCAS_SAFE_FPA_RAD     = 0.0  * (M_PI / 180.0);
    /// Minimum altitude above which GCAS will disengage after recovery [m].
    static constexpr double GCAS_SAFE_ALT_M       = 40.0;
    /// Absolute hard deck: the jet must never be allowed to descend through this
    /// altitude AGL. Enforced unconditionally alongside (not instead of) the
    /// kinematic margin check above, as a last-resort backstop [m] (= 30 ft).
    static constexpr double GCAS_HARD_DECK_M      = 30.0 * 0.3048;
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
        WING_LEVEL,  ///< Phase 1: rolling wings to level.
        PULL_UP,     ///< Phase 2: maximum G pull to positive FPA.
        SAFE         ///< Recovery successful; monitoring resumes.
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
    double tumble_persist_timer{0.0}; ///< Time out-of-control conditions have persisted [s].
    /// A spin or tumble (rotational departure) is being detected this tick, before
    /// the persistence filter has let the tumble law latch.
    bool departure_rotation{false};

    /// Latched roll direction (+1 / -1, 0 = unlatched) used while the bank angle sits
    /// in the ambiguous zone either side of inverted, where the Euler roll angle wraps
    /// between +180 and -180 deg. See @ref wings_level_roll_cmd.
    int gcas_roll_dir{0};
    int gloc_roll_dir{0};

    /// GCAS throttle override for this tick: -1.0 = no override (pilot/auto-throttle
    /// keeps authority), otherwise a commanded throttle position in [0, 1].
    double gcas_throttle_cmd{-1.0};

    /// Integral trim on the GCAS pitch command, closing the commanded recovery G
    /// on the measured Nz [stick units].
    double gcas_nz_trim{0.0};

    /// Set once both stick axes have been seen near neutral since GLOC onset. A
    /// pilot who blacks out mid-pull is still holding the stick where it was, and
    /// that held deflection is not evidence of having woken up.
    bool gloc_stick_relaxed{false};

    // =========================================================================
    // Airframe limits
    // =========================================================================

    /**
     * @brief The handful of airframe numbers the OFC's predictions depend on.
     *
     * A single set of F-16 constants used to serve all five jets: the recovery G
     * was mapped to stick through the F-16's 9 G range, departure was declared at
     * an F-16's 32 deg of alpha, and the pull-out was planned on 5 G being there
     * whatever the airspeed.  None of that holds for a 7.33 G direct-linkage A-10
     * or a 65 deg-alpha F-22.
     */
    struct Airframe {
        double max_g{9.0};             ///< Positive G limit.
        double alpha_limit_deg{25.2};  ///< FLCS alpha limit (stall alpha for the A-10).
        double roll_rate_dps{308.0};   ///< Rated maximum roll rate.
        double wing_area_m2{27.87};
        double cl_max{1.4};
        double ref_mass_kg{12473.7};   ///< Empty + full internal fuel: heaviest clean case.
        bool   g_command_law{true};    ///< Stick commands Nz (FBW) vs elevator (direct).

        [[nodiscard]] static constexpr Airframe from(aircraft::AircraftType type) noexcept {
            const auto cfg = aircraft::AircraftConfig::get(type);
            Airframe a;
            a.max_g           = cfg.flcs.max_g_positive;
            a.alpha_limit_deg = cfg.flcs.alpha_limit_deg;
            a.roll_rate_dps   = cfg.flcs.max_roll_rate_dps;
            a.wing_area_m2    = cfg.aero.s_ref;
            a.cl_max          = cfg.aero.cl_max;
            a.ref_mass_kg     = cfg.mass.empty_mass_kg + cfg.mass.internal_fuel_capacity_kg
                              + cfg.mass.combat_payload_kg;
            a.g_command_law   = cfg.flcs.law_type
                              != aircraft::FLCSConfig::LawType::HYDRO_SAS_AUGMENTED;
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
        tumble_persist_timer = 0.0;
        departure_rotation   = false;
        gcas_throttle_cmd    = -1.0;
        gcas_roll_dir        = 0;
        gloc_roll_dir        = 0;
        gcas_nz_trim         = 0.0;
        gloc_stick_relaxed   = false;
    }

    // -------------------------------------------------------------------------
    // Status accessors (convenience, named after plan interface)
    // -------------------------------------------------------------------------
    [[nodiscard]] bool   is_gloc_active()    const noexcept { return gloc_state == GlocState::GLOC || gloc_state == GlocState::RECOVERY; }
    [[nodiscard]] bool   is_gcas_active()    const noexcept { return gcas_state == GcasState::WING_LEVEL || gcas_state == GcasState::PULL_UP; }
    [[nodiscard]] bool   is_tumble_active()  const noexcept { return tumble_state == TumbleState::RECOVERING; }
    [[nodiscard]] double blackout_fraction_value() const noexcept { return blackout_fraction; }
    [[nodiscard]] double gcas_tti_seconds()  const noexcept { return gcas_tti; }
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
     *                    emergency low-altitude descent — Auto-GCAS still enforces
     *                    the kinematic margin and the 30 ft hard deck whenever the
     *                    gear is up, no matter how gentle the sink rate looks.
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
            gcas_state = GcasState::ARMED;
            gcas_roll_dir = 0;
            gcas_tti = 999.0;
            gcas_nz_trim = 0.0;
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
        // already-departed airframe and stand down: its pull-out margin assumes a
        // 5 G pull is actually achievable, which is false in a spin or deep stall.
        // Conversely, once GCAS *is* recovering it keeps the pitch and roll axes
        // (a nose-down command near the ground is unconditionally fatal), while
        // the anti-spin rudder from the tumble law is left in place to help it.
        step_gloc(dt, state, imu, pilot_in, effective);
        step_tumble(dt, state, pilot_in, effective);
        step_gcas(dt, state, imu, pilot_in, effective, gear_down);

        return effective;
    }

private:

    // =========================================================================
    // Shared recovery helpers
    // =========================================================================

    /// Half-width of the ambiguous zone either side of inverted, within which the
    /// roll direction is latched rather than recomputed from the Euler angle.
    static constexpr double ROLL_AMBIGUOUS_RAD = 170.0 * (M_PI / 180.0);

    /**
     * @brief Roll-to-wings-level stick command, stable across the +/-180 deg wrap.
     *
     * A naive `-roll / gain` law saturates to -1 at roll = +179 deg and to +1 at
     * roll = -179 deg.  Near inverted the Euler roll angle flips between those two
     * every few ticks, so the command dithers full-scale left/right and the jet
     * never actually rolls upright — it just shakes while the nose falls.  Inside
     * the ambiguous zone a direction is therefore chosen once and held until the
     * bank is unambiguous again; both directions are equally short from inverted,
     * so the only thing that matters is that the choice stays put.
     *
     * @param roll         Euler roll angle [rad], in [-pi, +pi].
     * @param gain         Roll angle at which the command saturates [rad].
     * @param latched_dir  In/out latch state (0 = unlatched).
     */
    [[nodiscard]] static double wings_level_roll_cmd(
        double roll,
        double gain,
        int& latched_dir
    ) noexcept {
        if (std::abs(roll) >= ROLL_AMBIGUOUS_RAD) {
            if (latched_dir == 0) {
                latched_dir = (roll >= 0.0) ? -1 : +1;
            }
            return static_cast<double>(latched_dir);
        }
        latched_dir = 0;
        return std::clamp(-roll / gain, -1.0, 1.0);
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
     * a crash, so the pull is faded out and the roll is allowed to finish first.
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
                    // G exposure has dropped by now; clamp to a low residual
                    g_exposure = std::min(g_exposure, 2.0);
                }
                break;

            case GlocState::RECOVERY:
                gloc_timer += dt;
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
                }
            }

            if (gloc_authority < 1.0) {
                const PilotCommands ofc_cmd = gloc_recovery_command(state);
                // Blend OFC authority into the pilot commands
                const double pilot_share = gloc_authority;
                const double ofc_share   = 1.0 - gloc_authority;
                effective.pitch_stick  = pilot_share * effective.pitch_stick
                                       + ofc_share   * ofc_cmd.pitch_stick;
                effective.roll_stick   = pilot_share * effective.roll_stick
                                       + ofc_share   * ofc_cmd.roll_stick;
            }
        }
    }

    /**
     * @brief Compute OFC recovery commands during GLOC.
     *
     * Priority: roll wings-level first, then pitch to gentle climb.
     */
    [[nodiscard]] PilotCommands gloc_recovery_command(
        const fdm::FlightState& state
    ) noexcept {
        PilotCommands cmd{};

        const auto euler = state.euler_angles();
        const double roll  = euler.roll;  // radians, [-π, +π]
        const double pitch = euler.pitch; // radians

        cmd.roll_stick = wings_level_roll_cmd(roll, M_PI * 0.35, gloc_roll_dir);

        if (std::abs(roll) > GLOC_WINGS_LEVEL_RAD) {
            // Phase 1: roll wings level as quickly as possible, unloaded.
            cmd.pitch_stick = 0.0;
        } else {
            // Phase 2: wings level — gentle positive pitch to level out
            constexpr double TARGET_PITCH_RAD = 3.0 * (M_PI / 180.0);
            const double pitch_err = TARGET_PITCH_RAD - pitch;
            cmd.pitch_stick = std::clamp(pitch_err * 2.0, -0.3, 0.6)
                            * bank_pull_scale(roll);
        }
        cmd.rudder_pedal = 0.0;
        return cmd;
    }

    // =========================================================================
    // GCAS subsystem
    // =========================================================================

    /**
     * @brief Advance the Auto-GCAS state machine one timestep.
     *
     * Engagement is decided purely by the kinematic recovery-altitude margin (so
     * shallow-but-low descents are caught as reliably as steep dives).  That margin
     * is planned on the G this airframe can actually pull at this airspeed, on the
     * circular arc a constant-G pull really flies, on the speed the jet keeps
     * gaining through it, and on the dive continuing while the G comes on.  Once
     * engaged, the commanded G stays pinned near the same nominal value the margin
     * was computed with — escalating toward a capped ceiling only when time-to-impact
     * is genuinely critical — and is closed on the measured Nz, so every flight
     * control law delivers it.  It releases control back to the pilot immediately
     * once level/safe.
     */
    void step_gcas(
        double dt,
        const fdm::FlightState& state,
        const IMUData& imu,
        const PilotCommands& pilot_in,
        PilotCommands& effective,
        bool gear_down
    ) noexcept {
        // --- Kinematic parameters ------------------------------------------
        constexpr double G0 = 9.80665;
        const double V = state.airspeed(); // true airspeed [m/s]
        if (V < 10.0) {
            // Disarm rather than just bailing out: returning while a recovery was
            // in progress used to leave gcas_state latched at WING_LEVEL/PULL_UP
            // forever, so is_gcas_active() reported a recovery that was issuing no
            // commands at all — which also suppressed the tumble law indefinitely.
            gcas_state    = GcasState::ARMED;
            gcas_roll_dir = 0;
            gcas_tti      = 999.0;
            gcas_nz_trim  = 0.0;
            return;
        }

        // Current terrain height directly below the aircraft [m MSL]
        const double current_terr_h = environment::GroundCollision::get_terrain_height(
            state.pos_ned.x, state.pos_ned.y);
        // Current Altitude Above Ground Level (AGL) [m]
        const double local_agl = std::max(0.0, state.altitude() - current_terr_h);
        const auto euler = state.euler_angles();

        // NED velocity: rotate body velocity to inertial frame
        const math::Vector3 vel_ned = state.velocity_ned();

        // Flight-path angle (negative = diving, positive = climbing)
        const double horiz_speed = std::sqrt(vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y);
        const double vert_speed  = vel_ned.z;  // NED: positive = descending
        const double sink        = std::max(0.0, vert_speed);

        // FPA in radians: negative means nose below horizon
        const double fpa = std::atan2(-vert_speed, std::max(horiz_speed, 1.0));

        // Ground / Runway touch-down lockout: when resting on gear or rolling on runway/ground,
        // do not trigger GCAS
        if (gear_down && (local_agl < 2.0 || state.pos_ned.z >= -current_terr_h - 1.8) && !is_gcas_active()) {
            gcas_state = GcasState::ARMED;
            gcas_tti = 999.0;
            return;
        }

        // Landing approach: gear down and sinking no faster than a landing does.
        // This used to be waived only below 15 m, but the 35 m altitude floor below
        // then fired on every stabilised 3 deg approach between 35 m and 15 m and
        // shoved the jet off the glideslope. The real F-16 system is inhibited
        // outright with the gear handle down; here only an actual collision with
        // rising terrain ahead is still acted on.  A gear-down dive (sink rate far
        // above any landing) is still fully protected.
        const bool landing_approach = gear_down && vert_speed < GCAS_LANDING_SINK_MPS
                                   && fpa > GCAS_LANDING_FPA_RAD;

        // --- Recovery G for this airframe -----------------------------------
        const double g_nominal = std::min(GCAS_RECOVERY_G, GCAS_RECOVERY_G_FRAC * airframe.max_g);
        const double g_ceiling = std::max(g_nominal,
            std::min(GCAS_MAX_RECOVERY_G, GCAS_MAX_G_FRAC * airframe.max_g));

        // Load factor the wing can actually produce right now.  Heavy (full-fuel)
        // reference mass keeps this conservative, and so does the maximum lift
        // coefficient falling with Mach.
        const auto air = environment::Atmosphere1976::compute(state.altitude(), V);
        const double n_lift_max = air.dynamic_pressure * airframe.wing_area_m2 * airframe.cl_max
                                * aero::CompressibleLift::clmax_mach_factor(air.mach_number)
                                / (airframe.ref_mass_kg * G0);
        const double n_pull = std::clamp(GCAS_USABLE_LIFT_FRAC * n_lift_max, 1.0, g_nominal);

        // Check active pilot control.  These MUST be judged on the raw inceptor
        // positions: `effective` may already hold the GLOC autopilot's or the tumble
        // law's own output, and reading that back made the OFC treat its own stick
        // command as a conscious pilot recovery — handing a diving jet to an
        // unconscious pilot.  An unconscious pilot can never break out at all.
        const bool pilot_conscious  = !is_gloc_active();
        // Nz alone is not evidence of a recovery: a high-alpha mush or a departure
        // loads the airframe just as hard.  Require the pilot to actually be pulling
        // *and* the jet to be answering with the G the recovery is planned on.  A
        // token 2 G tug used to stand GCAS down outright until 0.9 s from impact.
        const bool pilot_recovering = pilot_conscious
                                   && pilot_in.pitch_stick > 0.25 && imu.Nz >= 0.9 * n_pull;
        const bool pilot_breakout   = pilot_conscious
                                   && (pilot_in.pitch_stick > 0.35
                                       || std::abs(pilot_in.roll_stick) > 0.40);

        // --- Altitude needed to pull out ------------------------------------
        // A constant-G pull flies a circular arc: from dive angle gamma it loses
        // R (1 - cos gamma) with R = V^2 / ((n - 1) g).  The old V^2 sin^2 / 2a
        // treated the vertical deceleration as constant, which it is not — it
        // shrinks with cos(gamma) — and under-predicted steep dives.
        const double dive          = std::clamp(-fpa, 0.0, M_PI * 0.5);
        const double one_minus_cos = 1.0 - std::cos(dive);
        const double a_net         = std::max((n_pull - 1.0) * G0, 0.05 * G0);
        double h_arc = V * V * one_minus_cos / a_net;
        // The jet keeps accelerating down the arc, widening it: re-plan on the mean
        // of entry and exit speed squared (the exit gains 2 g h_arc).
        h_arc = (V * V + G0 * h_arc) * one_minus_cos / a_net;

        // The dive continues unabated until the G is actually on — unless the pilot
        // is already holding it.
        const double h_onset = pilot_recovering ? 0.0 : sink * GCAS_G_ONSET_S;

        // Bank costs altitude too: wings must come level *before* the pull-up can
        // start biting, so budget the extra sink accrued during that roll.
        const double roll_deg           = std::abs(euler.roll) * (180.0 / M_PI);
        const double roll_recovery_time = roll_deg / (GCAS_ROLL_RATE_FRAC * airframe.roll_rate_dps);
        const double roll_altitude_cost = sink * roll_recovery_time;

        const double h_required = (h_arc + h_onset) * GCAS_SAFETY_MARGIN
                                 + GCAS_ALTITUDE_FLOOR_M
                                 + roll_altitude_cost;

        // --- Predictive Terrain Profile Look-Ahead (NASA / AFRL DTED model) ---
        // Scan forward along the horizontal flight path to detect rising hills,
        // ridges, and mountains before the aircraft impacts them.
        double min_tti = (vert_speed > 0.5) ? (local_agl / vert_speed) : 999.0;
        bool terrain_ahead_trigger = false;
        bool rising_terrain_impact = false;

        // Look-ahead horizon: sample ahead up to 5.0 seconds
        constexpr std::array<double, 8> scan_times = {0.3, 0.6, 1.0, 1.5, 2.0, 2.8, 3.8, 5.0};
        for (double tau : scan_times) {
            const double fwd_x = state.pos_ned.x + vel_ned.x * tau;
            const double fwd_y = state.pos_ned.y + vel_ned.y * tau;
            const double terr_h_fwd = environment::GroundCollision::get_terrain_height(fwd_x, fwd_y);

            // Projected altitude if continuing along current trajectory
            const double proj_alt = state.altitude() - vert_speed * tau;
            const double proj_clearance = proj_alt - terr_h_fwd;

            if (proj_clearance <= 0.0) {
                // Future impact predicted at time tau
                min_tti = std::min(min_tti, tau);
            }

            // Rising terrain obstacle detection: if terrain ahead rises significantly
            // above the current ground level, check if our trajectory or recovery arc clears it.
            if (terr_h_fwd > current_terr_h + 10.0) {
                const double agl_vs_fwd = state.altitude() - terr_h_fwd;
                const double req_fwd = h_required + (terr_h_fwd - current_terr_h);
                if (local_agl < req_fwd || agl_vs_fwd < h_required || proj_clearance < GCAS_ALTITUDE_FLOOR_M) {
                    // Note: tau is deliberately NOT folded into min_tti here. Rising
                    // terrain that the jet still clears is a reason to recover, but it
                    // is not a time-to-impact; reporting it as one drove the commanded
                    // G up the escalation ramp against a collision that was not
                    // happening. An actual predicted impact is already caught above.
                    terrain_ahead_trigger = true;
                }
                if (proj_clearance <= 0.0) {
                    rising_terrain_impact = true;
                }
            } else if (proj_clearance <= 0.0 && tau <= 1.5) {
                // Imminent impact with terrain within 1.5 seconds
                terrain_ahead_trigger = true;
            }
        }

        gcas_tti = min_tti;

        // --- State transitions ----------------------------------------------
        switch (gcas_state) {

            case GcasState::ARMED:
            case GcasState::SAFE: {
                // Take control if EITHER:
                // 1. The kinematic margin says the current descent won't pull out in time
                // 2. The jet is descending through the 30 ft hard deck
                // 3. Terrain ahead poses a collision threat along projected flight path
                // On a landing approach only an actual collision with rising terrain
                // counts.  A pilot already pulling the recovery G is credited through
                // h_required (no onset lag), not by standing the system down.
                const bool kinematic_trigger = !landing_approach
                                            && fpa < GCAS_ARM_FPA_RAD && local_agl < h_required;
                const bool hard_deck_trigger = !landing_approach
                                            && local_agl < GCAS_HARD_DECK_M && vert_speed > 0.0;
                const bool terrain_trigger   = landing_approach ? rising_terrain_impact
                                                                : terrain_ahead_trigger;
                // Do not newly engage on top of a departure: h_required above assumes
                // a controlled pull is achievable, which it is not in a spin or deep
                // stall.  Breaking the departure comes first.  That includes a spin
                // or tumble still inside the tumble law's 0.25 s persistence filter —
                // engaging there let GCAS grab the pitch axis first and hold it.
                // High alpha *alone* does not stand GCAS down: its own pull puts the
                // jet there near the ground, where a nose-down stall break is fatal.
                if (is_tumble_active() || departure_rotation) {
                    gcas_state    = GcasState::ARMED;
                    gcas_roll_dir = 0;
                } else if (kinematic_trigger || hard_deck_trigger || terrain_trigger) {
                    if (std::abs(euler.roll) > GCAS_WINGS_LEVEL_RAD) {
                        gcas_state = GcasState::WING_LEVEL;
                    } else {
                        gcas_state = GcasState::PULL_UP;
                    }
                } else {
                    gcas_state = GcasState::ARMED;
                }
                break;
            }

            case GcasState::WING_LEVEL: {
                if (std::abs(euler.roll) <= GCAS_WINGS_LEVEL_RAD) {
                    gcas_state = GcasState::PULL_UP;
                }
                // Release immediately if no longer sinking, terrain ahead is clear, or pilot breaks out
                if ((vert_speed <= 0.0 && !terrain_ahead_trigger) || (pilot_breakout && fpa > -0.15 && !terrain_ahead_trigger)) {
                    gcas_state = GcasState::SAFE;
                }
                break;
            }

            case GcasState::PULL_UP: {
                // Immediate release back to pilot as soon as jet is safe:
                // 1. Aircraft is no longer descending (vert_speed <= 0 or FPA >= 0.0)
                // 2. Altitude AGL is above safe clearance floor
                // 3. Terrain ahead has no immediate conflict
                const bool no_longer_sinking = (vert_speed <= 0.0 || fpa >= GCAS_SAFE_FPA_RAD);
                const bool clearance_safe    = (local_agl > GCAS_SAFE_ALT_M);

                if (no_longer_sinking && clearance_safe && !terrain_ahead_trigger) {
                    gcas_state = GcasState::SAFE;
                } else if (pilot_breakout && (fpa > -0.15 || vert_speed < 15.0) && !terrain_ahead_trigger) {
                    // vert_speed is NED (positive = sinking).  This read `> -15.0`,
                    // which is true for *every* descent, so any 0.4 roll-stick twitch
                    // handed a steep dive back to the pilot however close the ground.
                    gcas_state = GcasState::SAFE;
                } else if (std::abs(euler.roll) > GCAS_REBANK_RAD) {
                    // The jet has rolled back out of the wings-level band (turbulence,
                    // a gear-down asymmetry, the pilot fighting the roll).  Go back to
                    // levelling the wings instead of holding a pull that is no longer
                    // lifting the nose away from the ground.  The wider threshold gives
                    // hysteresis against chattering between the two phases.
                    gcas_state = GcasState::WING_LEVEL;
                }
                break;
            }
        }

        // --- Override pilot commands when GCAS is executing ----------------
        if (is_gcas_active()) {
            // Commanded recovery G: nominal, ramping toward the capped ceiling only
            // as time-to-impact becomes critical...
            const double critical_urgency = std::clamp(1.0 - gcas_tti / GCAS_CRITICAL_TTI_S, 0.0, 1.0);
            const double g_tti = g_nominal + critical_urgency * (g_ceiling - g_nominal);
            // ...or as soon as the height left is too short for the nominal arc, in
            // which case fly the G the arc actually needs from here.  Waiting for
            // the time-to-impact ramp wasted the seconds in which that extra G would
            // still have made the difference on a steep, fast dive.  Bank is not
            // charged here: while banked the extra G mostly acts sideways, and the
            // height the roll-out costs is seen tick by tick as it is spent.
            const double h_avail  = std::max(1.0, local_agl - 0.5 * GCAS_ALTITUDE_FLOOR_M);
            const double g_needed = 1.0 + GCAS_SAFETY_MARGIN * V * V * one_minus_cos / (G0 * h_avail);
            const double g_target = std::clamp(std::max(g_tti, g_needed), g_nominal, g_ceiling);
            const double pitch_cmd_for_g = pitch_command_for_g(
                dt, state, imu, g_target, n_lift_max / airframe.cl_max);

            // Pulling while inverted drives the nose *into* the terrain — the single
            // most dangerous thing an automatic recovery can do — so the commanded
            // pull is faded out with bank and the roll is allowed to finish first.
            // See bank_pull_scale for where the fade sits and why.
            const double bank_scaled_pull = pitch_cmd_for_g * bank_pull_scale(euler.roll);

            effective.roll_stick = wings_level_roll_cmd(euler.roll, M_PI * 0.25, gcas_roll_dir);
            // Once the fade has taken the commanded pull to zero the jet is inverted
            // enough that a pull is pure hazard, so the pilot is held to zero-or-push
            // as well — when inverted, a push is what raises the nose.
            effective.pitch_stick = (bank_scaled_pull > 0.0)
                                  ? std::max(effective.pitch_stick, bank_scaled_pull)
                                  : std::min(effective.pitch_stick, 0.0);

            // Leave the anti-spin rudder from the tumble law in place if it is
            // running — killing the yaw helps the recovery rather than fighting it.
            if (!is_tumble_active()) {
                effective.rudder_pedal = 0.0;
            }

            // A slow jet cannot afford to shed energy: without it the recovery G is
            // simply not there, so it gets full power from the first tick.
            const bool energy_limited = GCAS_USABLE_LIFT_FRAC * n_lift_max < 1.25 * g_nominal;
            if (gcas_state == GcasState::WING_LEVEL && !energy_limited) {
                // Cut toward idle: still banked, so the elevator can't pull effectively
                // yet — don't let the dive keep accelerating into the ground while it rolls.
                gcas_throttle_cmd = GCAS_ROLL_THROTTLE;
            } else {
                // Full power: arresting the sink rate at a sane G still bleeds airspeed,
                // and a recovery that stalls the jet right back toward the ground would
                // defeat the entire point of the system.
                gcas_throttle_cmd = GCAS_PULLOUT_THROTTLE;
            }
        } else {
            gcas_roll_dir = 0;
            gcas_nz_trim  = 0.0;
        }
    }

    /**
     * @brief Stick command that holds @p g_target on this airframe.
     *
     * Fly-by-wire laws command Nz from stick, so the G maps straight across
     * through the airframe's own G range rather than assuming the F-16's 9 G,
     * with a small integral trim for the laws
     * that settle short of their command.  A direct-linkage law (A-10) has no G
     * command at all: its stick moves the elevator and the G that produces grows
     * with q, so the old open-loop mapping pulled it to 10-12 G.  There the
     * command is a feed-forward lift estimate closed on the measured Nz, with an
     * alpha guard standing in for the limiter the airframe does not have.
     *
     * @param q_s_over_w  Dynamic pressure x wing area / reference weight [-].
     */
    [[nodiscard]] double pitch_command_for_g(
        double dt,
        const fdm::FlightState& state,
        const IMUData& imu,
        double g_target,
        double q_s_over_w
    ) noexcept {
        const double nz_err = g_target - imu.Nz;

        if (airframe.g_command_law) {
            const double ff = (g_target - 1.0) / std::max(1.0, airframe.max_g - 1.0);
            // Trim only once the G is broadly on: during onset the error is large
            // and still falling on its own, and integrating it would overshoot.
            if (gcas_state == GcasState::PULL_UP && std::abs(nz_err) < 1.5) {
                gcas_nz_trim = std::clamp(gcas_nz_trim + GCAS_NZ_TRIM_KI * nz_err * dt,
                                          -GCAS_NZ_TRIM_LIMIT, GCAS_NZ_TRIM_LIMIT);
            }
            return std::clamp(ff + gcas_nz_trim, 0.0, 1.0);
        }

        const double g_per_stick = std::max(1.0, GCAS_DIRECT_G_PER_QSW * q_s_over_w);
        const double ff          = (g_target - 1.0) / g_per_stick;

        const double alpha_deg  = state.alpha() * (180.0 / M_PI);
        const double alpha_over = alpha_deg - (airframe.alpha_limit_deg - 2.0);
        if (gcas_state == GcasState::PULL_UP && (alpha_over < 0.0 || nz_err < 0.0)) {
            gcas_nz_trim = std::clamp(gcas_nz_trim + (GCAS_DIRECT_NZ_KI / g_per_stick) * nz_err * dt,
                                      -0.5, 0.5);
        }
        double cmd = ff + (GCAS_DIRECT_NZ_KP / g_per_stick) * nz_err + gcas_nz_trim;
        if (alpha_over > 0.0) {
            cmd -= 0.08 * alpha_over;
        }
        return std::clamp(cmd, 0.0, 1.0);
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
        // 2. Flat / steep spin: high yaw rate (> 40 deg/s) + sideslip (> 14 deg)
        const bool is_spin = (r_dps > 40.0 && std::abs(beta_deg) > 14.0);
        // 3. Violent out-of-control tumble: pitch & yaw multi-axis rates exceed safe envelope
        //    (excludes intentional roll and pitch manoeuvres)
        const bool is_tumble = !intentional_roll && !intentional_pitch
                            && ((pitch_yaw_rate > 1.50) || (q_dps > 65.0 && is_deep_stall));

        departure_rotation = is_spin || is_tumble;

        if (is_deep_stall || is_spin || is_tumble) {
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
                // Positive deep stall: full nose-down pitch to break stall & restore attached flow
                effective.pitch_stick = -1.0;
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
