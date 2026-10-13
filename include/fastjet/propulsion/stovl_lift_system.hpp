#pragma once

#include "stovl_status.hpp"
#include "../aircraft/aircraft_config.hpp"
#include "../fdm/aircraft_forces.hpp"
#include "../math/quaternion.hpp"
#include "../math/vector3.hpp"
#include <algorithm>
#include <cmath>

namespace fastjet::propulsion {

/// @brief Effector demands from the STOVL control law, one step's worth.
struct StovlEffectorCommands {
    double nozzle_deg{0.0};      ///< 3BSM pitch: 0 aft .. 90 down .. 95
    double nozzle_yaw_deg{0.0};  ///< 3BSM lateral vectoring, + exhaust right (nose right)
    double vane_deg{0.0};        ///< Vane box, + fan thrust vectored aft (forward force)
    double pitch_split{0.0};     ///< [-1, 1]: + moves thrust from the 3BSM to the fan (nose up)
    double roll_split{0.0};      ///< [-1, 1]: + moves roll-post flow to the left post (roll right)
};

/**
 * @brief F-35B lift system: the Rolls-Royce LiftSystem on the F135-PW-600.
 *
 * Mode conversion follows the published sequence: the STOVL doors open (LiftFan
 * inlet, auxiliary inlets, vane-box and roll-post doors), the clutch engages and
 * spins the fan up to lock-up, and only then does the 3BSM vector down; back to
 * CTOL the nozzle swings aft first, the clutch releases and the doors close.
 * Conversion to STOVL is inhibited above the fan's 250 KCAS limit, and back to
 * CTOL whenever the wing could not take the weight (the caller decides).
 *
 * Thrust: the engine's net thrust (MultiEngine, whose dry rating rises toward
 * the 40,650 lbf lift rating as the clutch engages) is split between the
 * LiftFan, the roll posts and the 3BSM in proportion to their ratings, and each
 * acts along its own vector from its own exit, so forces and moments come from
 * the geometry: fan forward of the CG, 3BSM aft, roll posts out under the wings.
 * Pitch control moves thrust between fan and 3BSM (fan inlet guide vanes and
 * nozzle area), roll control moves bypass air between the posts, yaw control
 * swings the 3BSM sideways. Total thrust is unchanged by all three.
 *
 * Near the ground three small, documented terms act on the jet lift:
 *  - suckdown: the jets entrain air from under the airframe, a lift loss that
 *    peaks at a few percent with the exits close to the ground;
 *  - fountain: the fan and nozzle jets meet on the ground and a fountain hits
 *    the fuselage between them, giving back part of that at wheel height;
 *  - hot-gas ingestion: exhaust recirculating into the inlets raises inlet
 *    temperature, ~1% thrust per kelvin, blown clear by ground speed. The
 *    fan's cold curtain keeps this small on the F-35B.
 * The fan intake also swallows its airflow from the free stream: intake
 * momentum drag at the inlet, above and ahead of the CG.
 *
 * Zero heap allocation; safe in the 200 Hz step.
 */
class StovlLiftSystem {
public:
    aircraft::StovlConfig config{};

    StovlMode mode{StovlMode::CTOL};
    double doors{0.0};           ///< Door timeline, 0 shut .. 1 all open
    double clutch{0.0};          ///< 0 declutched .. 1 locked up
    bool inhibited{false};       ///< The pilot's selection is being refused (speed / wing-borne)

    // Effector positions
    double nozzle_deg{0.0};
    double nozzle_yaw_deg{0.0};
    double vane_deg{0.0};
    double pitch_split{0.0};
    double roll_split{0.0};

    /// Suckdown: peak lift loss [share of jet lift], and the exit height it halves at [m].
    static constexpr double SUCKDOWN_MAX = 0.05;
    static constexpr double SUCKDOWN_HEIGHT_M = 6.0;
    /// Fountain: peak lift recovered, and its decay height [m].
    static constexpr double FOUNTAIN_MAX = 0.025;
    static constexpr double FOUNTAIN_HEIGHT_M = 2.0;
    /// Hot-gas ingestion: inlet temperature rise with the exits on the ground [K],
    /// its decay height [m], the ground speed that halves it [m/s], and the thrust
    /// lost per kelvin.
    static constexpr double HGI_RISE_K = 6.0;
    static constexpr double HGI_HEIGHT_M = 3.0;
    static constexpr double HGI_CLEAR_MPS = 8.0;
    static constexpr double HGI_LOSS_PER_K = 0.01;
    /// Fan IGV and roll-post valve response [s].
    static constexpr double SPLIT_TAU_S = 0.08;

    StovlLiftSystem() noexcept = default;
    explicit StovlLiftSystem(aircraft::AircraftType type) noexcept { configure(type); }

    void configure(aircraft::AircraftType type) noexcept {
        config = aircraft::AircraftConfig::get(type).stovl;
        reset();
    }

    void reset() noexcept {
        mode = StovlMode::CTOL;
        doors = clutch = 0.0;
        inhibited = false;
        nozzle_deg = nozzle_yaw_deg = vane_deg = pitch_split = roll_split = 0.0;
    }

    /// @brief Fully converted, nozzle down: for hover and approach starts.
    void start_converted() noexcept {
        if (!config.equipped) return;
        mode = StovlMode::STOVL;
        doors = clutch = 1.0;
        inhibited = false;
        nozzle_deg = 90.0;
        nozzle_yaw_deg = vane_deg = pitch_split = roll_split = 0.0;
    }

    /// @brief The lift system is (being) used: the unified STOVL law flies the jet.
    [[nodiscard]] bool engaged() const noexcept { return config.equipped && mode != StovlMode::CTOL; }

    [[nodiscard]] double stovl_rating_n() const noexcept {
        return config.lift_fan_thrust_n + config.nozzle_thrust_n + config.roll_post_thrust_n;
    }

    /**
     * @brief Runs the conversion sequencer and moves the effectors.
     *
     * @param request_stovl The STOVL/CTOL button state.
     * @param kcas          Calibrated airspeed [kt].
     * @param ctol_ok       The wing can carry the jet (or it is on its wheels).
     * @param cmd           The control law's effector demands.
     */
    void update(double dt, bool request_stovl, double kcas, bool ctol_ok,
                const StovlEffectorCommands& cmd) noexcept {
        if (!config.equipped || dt <= 0.0) return;

        bool to_stovl = request_stovl;
        inhibited = false;
        if (to_stovl && mode == StovlMode::CTOL && kcas > config.max_conversion_kcas) {
            to_stovl = false;   // fan doors may not open past the LiftFan limit
            inhibited = true;
        } else if (!to_stovl && mode == StovlMode::STOVL && !ctol_ok) {
            to_stovl = true;    // jet-borne: the wing cannot take the weight yet
            inhibited = true;
        }

        if (to_stovl) {
            doors = std::min(1.0, doors + dt / config.door_open_s);
            if (doors >= 1.0) clutch = std::min(1.0, clutch + dt / config.clutch_engage_s);
            mode = (clutch >= 1.0) ? StovlMode::STOVL : StovlMode::CONVERTING_TO_STOVL;
        } else {
            // Nozzle aft first, then the clutch, then the doors.
            if (nozzle_deg <= 1.0) clutch = std::max(0.0, clutch - dt / config.clutch_release_s);
            if (clutch <= 0.0) doors = std::max(0.0, doors - dt / config.door_close_s);
            mode = (doors <= 0.0) ? StovlMode::CTOL : StovlMode::CONVERTING_TO_CTOL;
        }

        // The 3BSM vectors only once the fan is locked up and STOVL is still wanted.
        const bool vectoring = to_stovl && clutch >= 1.0;
        slew(nozzle_deg, vectoring ? std::clamp(cmd.nozzle_deg, 0.0, config.nozzle_max_deg) : 0.0,
             config.nozzle_rate_dps * dt);
        slew(nozzle_yaw_deg, vectoring ? std::clamp(cmd.nozzle_yaw_deg, -config.nozzle_yaw_max_deg,
                                                    config.nozzle_yaw_max_deg) : 0.0,
             config.nozzle_yaw_rate_dps * dt);
        slew(vane_deg, clutch > 0.0 ? std::clamp(cmd.vane_deg, config.vane_min_deg, config.vane_max_deg) : 0.0,
             config.vane_rate_dps * dt);
        const double k = 1.0 - std::exp(-dt / SPLIT_TAU_S);
        pitch_split += (std::clamp(cmd.pitch_split, -1.0, 1.0) - pitch_split) * k;
        roll_split += (std::clamp(cmd.roll_split, -1.0, 1.0) - roll_split) * k;
    }

    /// @brief Thrust at each post [N].
    struct PostThrust {
        double fan{0.0};
        double nozzle{0.0};
        double post_left{0.0};
        double post_right{0.0};
    };

    /// @brief Roll-post doors follow the vane-box doors on the door timeline.
    [[nodiscard]] double roll_post_door() const noexcept { return std::clamp((doors - 0.3) / 0.7, 0.0, 1.0); }

    /**
     * @brief Splits the engine's net thrust between the posts.
     * @param net_thrust_n Engine net thrust [N].
     * @param dry_rating_n The engine's present maximum dry rating (MultiEngine::max_dry_rating_n).
     */
    [[nodiscard]] PostThrust split(double net_thrust_n, double dry_rating_n) const noexcept {
        PostThrust t{};
        const double per_n = net_thrust_n / std::max(1.0, dry_rating_n);
        const double fan = per_n * clutch * config.lift_fan_thrust_n;
        const double posts = per_n * roll_post_door() * config.roll_post_thrust_n;
        const double shift = pitch_split * config.fan_split_authority * fan;
        t.fan = fan + shift;
        t.nozzle = std::max(0.0, net_thrust_n - fan - posts - shift);
        t.post_left = 0.5 * posts * (1.0 + roll_split);
        t.post_right = 0.5 * posts * (1.0 - roll_split);
        return t;
    }

    /// @brief Share of jet lift lost (+) or gained (-) to suckdown and fountain at exit height @p h_m.
    [[nodiscard]] static double ground_lift_loss(double h_m) noexcept {
        const double h = std::max(0.0, h_m);
        const double r = h / SUCKDOWN_HEIGHT_M;
        return SUCKDOWN_MAX / (1.0 + r * r) - FOUNTAIN_MAX * std::exp(-h / FOUNTAIN_HEIGHT_M);
    }

    /// @brief Thrust factor from hot-gas ingestion at exit height @p h_m and ground speed @p gs_mps.
    [[nodiscard]] static double hgi_thrust_factor(double h_m, double gs_mps) noexcept {
        const double g = gs_mps / HGI_CLEAR_MPS;
        const double rise = HGI_RISE_K * std::exp(-std::max(0.0, h_m) / HGI_HEIGHT_M) / (1.0 + g * g);
        return 1.0 - HGI_LOSS_PER_K * rise;
    }

    /**
     * @brief Body-axis forces and moments of the whole propulsion system while
     * the lift system is engaged (it replaces the plain thrust along body X).
     *
     * @param net_thrust_n Engine net thrust [N].
     * @param dry_rating_n Engine's present maximum dry rating [N].
     * @param v_air_b      Velocity relative to the air, body axes [m/s].
     * @param q_att        Attitude (for the ground-effect lift, which acts vertically).
     * @param h_exit_m     Height of the jet exits above the ground [m].
     * @param gs_mps       Ground speed [m/s].
     * @param status       Optional: thrusts and jet lift written for the cockpit.
     */
    [[nodiscard]] fdm::AircraftForces compute(double net_thrust_n, double dry_rating_n,
                                              const math::Vector3& v_air_b, const math::Quaternion& q_att,
                                              double h_exit_m, double gs_mps,
                                              StovlStatus* status = nullptr) const noexcept {
        constexpr double D2R = M_PI / 180.0;
        PostThrust t = split(net_thrust_n, dry_rating_n);
        const double hgi = hgi_thrust_factor(h_exit_m, gs_mps) ;
        // Hot-gas ingestion only matters with the jets pointing at the ground.
        const double down = std::sin(nozzle_deg * D2R) * clutch;
        const double k_hgi = 1.0 - (1.0 - hgi) * down;
        t.fan *= k_hgi; t.nozzle *= k_hgi; t.post_left *= k_hgi; t.post_right *= k_hgi;

        fdm::AircraftForces out{};
        auto add = [&out](const math::Vector3& r, const math::Vector3& f) noexcept {
            out.force_b = out.force_b + f;
            out.moment_b = out.moment_b + r.cross(f);
        };

        const double v = vane_deg * D2R;
        add(config.lift_fan_pos_b, math::Vector3(std::sin(v), 0.0, -std::cos(v)) * t.fan);

        // The 3BSM exit sits on the engine axis when aft and drops as it turns down.
        const double dn = nozzle_deg * D2R, dy = nozzle_yaw_deg * D2R;
        const math::Vector3 nozzle_pos(config.nozzle_pos_b.x, 0.0, config.nozzle_pos_b.z * std::sin(dn));
        add(nozzle_pos, math::Vector3(std::cos(dn) * std::cos(dy), -std::sin(dy), -std::sin(dn) * std::cos(dy)) * t.nozzle);

        const math::Vector3 post_r = config.roll_post_pos_b;
        const math::Vector3 post_l(post_r.x, -post_r.y, post_r.z);
        add(post_l, math::Vector3(0.0, 0.0, -t.post_left));
        add(post_r, math::Vector3(0.0, 0.0, -t.post_right));

        // Vertical (NED up) share of the jets, before ground effect.
        const double jet_lift = -q_att.rotate_body_to_ned(out.force_b).z;

        // Fan intake momentum drag: its airflow brought to the aircraft's speed.
        const double fan_flow = config.fan_mass_flow_kgs * t.fan / std::max(1.0, config.lift_fan_thrust_n);
        add(config.fan_inlet_pos_b, v_air_b * (-fan_flow));

        // Suckdown and fountain act on the airframe, vertically.
        const double ge = ground_lift_loss(h_exit_m) * std::max(0.0, jet_lift) * down;
        out.force_b = out.force_b + q_att.rotate_ned_to_body(math::Vector3(0.0, 0.0, ge));

        if (status) {
            status->lift_fan_thrust_n = t.fan;
            status->nozzle_thrust_n = t.nozzle;
            status->roll_post_left_n = t.post_left;
            status->roll_post_right_n = t.post_right;
            status->jet_lift_n = jet_lift - ge;
        }
        return out;
    }

    /// @brief Most jet lift available here: full STOVL power with the jets down [N].
    [[nodiscard]] double max_jet_lift_n(double lapse, double h_exit_m, double gs_mps) const noexcept {
        return stovl_rating_n() * lapse * hgi_thrust_factor(h_exit_m, gs_mps) * (1.0 - ground_lift_loss(h_exit_m));
    }

    /// @brief Sequencer and effector state for the cockpit (thrusts come from compute()).
    void fill_status(StovlStatus& s) const noexcept {
        s.equipped = config.equipped;
        s.mode = mode;
        s.conversion = 0.5 * (doors + clutch);
        s.lift_fan_door = std::clamp(doors / 0.7, 0.0, 1.0);
        s.aux_inlet_door = std::clamp((doors - 0.1) / 0.6, 0.0, 1.0);
        s.fan_nozzle_door = roll_post_door();
        s.roll_post_door = roll_post_door();
        s.clutch = clutch;
        s.nozzle_pitch_deg = nozzle_deg;
        s.nozzle_yaw_deg = nozzle_yaw_deg;
        s.vane_box_deg = vane_deg;
        s.conversion_inhibited = inhibited;
    }

private:
    static void slew(double& x, double target, double max_step) noexcept {
        x += std::clamp(target - x, -max_step, max_step);
    }
};

} // namespace fastjet::propulsion
