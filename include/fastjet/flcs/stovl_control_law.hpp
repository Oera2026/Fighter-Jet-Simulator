#pragma once

#include "imu_sensor.hpp"
#include "pilot_commands.hpp"
#include "../aircraft/aircraft_config.hpp"
#include "../environment/atmosphere1976.hpp"
#include "../fdm/flight_state.hpp"
#include "../fdm/mass_properties.hpp"
#include "../propulsion/multi_engine.hpp"
#include "../propulsion/stovl_lift_system.hpp"
#include "../propulsion/stovl_status.hpp"
#include <algorithm>
#include <cmath>

namespace fastjet::flcs {

/**
 * @brief F-35B unified STOVL control law (after the VAAC Harrier "unified"
 * laws; inceptor mapping from Walker, Fuller & Wurth, AIAA 2013-4243, and
 * Tomlinson 2011; see docs/f35b-reference.md section 5).
 *
 * Flies the jet from conversion until it is back in CTOL. The pilot commands
 * the flight path and speed; the law decides how the wing, the aero surfaces
 * and the lift system share the work.
 *
 * Inceptors, blended with calibrated airspeed (pitch and pedals 40 -> 100 kt,
 * roll 100 -> 130 kt):
 *  - stick fore/aft: vertical speed in the hover (+10 / -7 ft/s, centred holds
 *    height), flight-path-angle rate at speed (centred holds the flight path);
 *  - stick lateral: bank angle in the hover (+-20 deg, centred holds position),
 *    roll rate with bank hold at speed;
 *  - throttle lever, relative to kStovlThrottleDetent: longitudinal acceleration
 *    (+-0.25 g, detent holds speed); below ~25 kt ground speed it becomes a
 *    ground-speed command (+-30 kt) whose detent is a zero-ground-speed hover;
 *  - pedals: yaw rate in the hover (30 deg/s, centred holds heading), sideslip
 *    through the rudder at speed with automatic turn coordination.
 * Hands-off in the hover the jet holds height, position and heading.
 *
 * Structure: the outer loops turn those commands into the accelerations and
 * attitude the jet must fly; the pitch attitude goes from a fixed hover
 * attitude to the one that gives the wing its share of the weight (up to the
 * approach AoA) as speed builds. Body-rate loops (PI, scaled by inertia so
 * they are moment demands) are then allocated aero surfaces first, in
 * proportion to their dynamic pressure, and the remainder to the lift system:
 * fan/3BSM thrust split for pitch, roll-post split for roll, 3BSM lateral
 * vectoring for yaw. The force the jet must add to the wing's (vertical and
 * along the heading) sets total thrust and the common thrust vector: vane box
 * and 3BSM angle.
 *
 * Envelope protection: bank and pitch attitude limits in the hover, a sink
 * rate limit closing in near the ground, thrust back to idle once the jet is
 * on its wheels (unless the stick asks for a lift-off or the lever for a
 * short take-off run), and no acceleration past the LiftFan's 250 KCAS.
 *
 * Zero heap allocation.
 */
class StovlControlLaw {
public:
    // --- Inceptor authority (docs/f35b-reference.md 5.3) ---------------------
    static constexpr double CLIMB_MAX_MPS = 3.05;      ///< +10 ft/s
    static constexpr double SINK_MAX_MPS = 2.13;       ///< 7 ft/s
    static constexpr double GAMMA_RATE_MAX_DPS = 10.0;
    static constexpr double HOVER_BANK_MAX_DEG = 20.0;
    static constexpr double WING_BANK_MAX_DEG = 60.0;
    static constexpr double ROLL_RATE_MAX_DPS = 200.0;
    static constexpr double YAW_RATE_MAX_DPS = 30.0;
    static constexpr double ACCEL_MAX_MPS2 = 0.25 * 9.80665;
    static constexpr double GS_COMMAND_MAX_MPS = 15.4;  ///< 30 kt either side of the detent
    /// Ground speed band where the lever's ground-speed command hands over to
    /// its acceleration command [m/s] (20 -> 35 kt).
    static constexpr double GS_BLEND_LO_MPS = 10.3;
    static constexpr double GS_BLEND_HI_MPS = 18.0;
    static constexpr double STOVL_SPEED_MAX_MPS = 128.6; ///< 250 kt
    static constexpr double FAN_LIMIT_KCAS = 250.0;      ///< LiftFan operating limit
    static constexpr double KT_PER_MPS = 1.94384;
    static constexpr double GS_LEAD_MAX_MPS = 5.0;       ///< Speed command lead over the jet
    /// Lever and stick dead zones around the detent / centre.
    static constexpr double LEVER_DEADBAND = 0.04;
    static constexpr double STICK_DEADBAND = 0.05;

    // --- Attitude and envelope --------------------------------------------
    static constexpr double HOVER_PITCH_DEG = 3.0;
    static constexpr double PITCH_MIN_DEG = -10.0;
    static constexpr double PITCH_MAX_DEG = 20.0;
    static constexpr double WING_ALPHA_MAX_DEG = 12.0;  ///< Approach AoA: the wing's share stops here
    /// Sink-rate limit at the wheels and its growth with wheel height [m/s, 1/s].
    static constexpr double TOUCHDOWN_SINK_MPS = 0.75;
    static constexpr double SINK_LIMIT_PER_M = 0.12;

    // --- Loop gains -------------------------------------------------------
    static constexpr double K_VS = 1.5;      ///< Vertical speed -> vertical accel [1/s]
    static constexpr double KI_VS = 0.6;
    static constexpr double K_HEIGHT = 0.5;  ///< Height hold [1/s]
    static constexpr double K_GS = 0.8;      ///< Ground speed -> longitudinal accel [1/s]
    static constexpr double K_LAT = 0.8;     ///< Lateral velocity -> lateral accel [1/s]
    static constexpr double K_POS = 0.3;     ///< Position hold [1/s]
    static constexpr double POS_CORR_MAX_MPS = 2.0;
    static constexpr double K_PHI = 2.5, K_THETA = 2.0, K_PSI = 1.0;       ///< Attitude [1/s]
    static constexpr double K_P = 6.0, K_Q = 5.0, K_R = 3.0;                ///< Rate [1/s]
    static constexpr double KI_P = 4.0, KI_Q = 3.0, KI_R = 2.0;             ///< Rate trim [1/s^2]
    static constexpr double RATE_TRIM_MAX = 0.6;                            ///< Trim authority [rad/s^2]

    // --- Aero control power (AircraftAeroModel), per radian ----------------
    static constexpr double CM_DE = -0.90;
    static constexpr double CL_DA = -0.22;
    static constexpr double CN_DR = -0.10;
    static constexpr double DE_LIMIT = 25.0, DA_LIMIT = 20.0, DR_LIMIT = 30.0;
    /// Aero surfaces take a share of the moment that grows over this q band [Pa].
    static constexpr double Q_AERO_LO = 300.0, Q_AERO_HI = 2500.0;

    struct Output {
        double de_deg{0.0};
        double da_deg{0.0};
        double dr_deg{0.0};
        propulsion::StovlEffectorCommands lift{};
        double lever{propulsion::kStovlThrottleDetent};  ///< Engine lever for the thrust demand
        bool ctol_ok{false};                              ///< The wing can take the jet back
    };

    // Law state, readable for the cockpit
    std::uint8_t control_phase{0};
    double commanded_vs_mps{0.0};
    double commanded_gs_mps{0.0};

    aircraft::AeroConfig aero{};

    StovlControlLaw() noexcept = default;
    explicit StovlControlLaw(aircraft::AircraftType type) noexcept { configure(type); }

    void configure(aircraft::AircraftType type) noexcept {
        aero = aircraft::AircraftConfig::get(type).aero;
        reset();
    }

    /// @brief Disengage: the next update() re-captures the flight condition.
    void reset() noexcept {
        engaged_ = false;
        control_phase = 0;
        commanded_vs_mps = commanded_gs_mps = 0.0;
    }

    /**
     * @param stick    Pilot inceptors (after the OFC).
     * @param lever    Throttle lever [0, 1].
     * @param aero_b   Aerodynamic force at the last step, body axes [N].
     * @param v_air_b  Velocity relative to the air, body axes [m/s].
     * @param wheel_height_m Height of the wheels above the ground [m].
     * @param on_wheels Main gear on the ground.
     */
    [[nodiscard]] Output update(double dt, const fdm::FlightState& state, const IMUData& imu,
                                const PilotCommands& stick, double lever, const fdm::MassProperties& mass,
                                const math::Vector3& aero_b, const math::Vector3& v_air_b,
                                const environment::AirData& air, double wheel_height_m, bool on_wheels,
                                const propulsion::StovlLiftSystem& lift,
                                const propulsion::MultiEngine& engine) noexcept {
        constexpr double G0 = 9.80665;
        constexpr double D2R = M_PI / 180.0;
        Output out{};

        const auto eul = state.euler_angles();
        const double cphi = std::cos(eul.roll), sphi = std::sin(eul.roll);
        const double cth = std::cos(eul.pitch), sth = std::sin(eul.pitch);
        const double cpsi = std::cos(eul.yaw), spsi = std::sin(eul.yaw);
        const math::Vector3 v_ned = state.velocity_ned();
        const double u_h = v_ned.x * cpsi + v_ned.y * spsi;    // along the heading
        const double v_h = -v_ned.x * spsi + v_ned.y * cpsi;   // to the right
        const double hdot = -v_ned.z;
        const double h = state.altitude();
        const double v_air = v_air_b.norm();
        const double kcas = std::sqrt(2.0 * imu.q_bar / environment::Atmosphere1976::RHO0) * KT_PER_MPS;

        if (!engaged_) engage(state, u_h, hdot);

        // Speed blends: pitch and pedals 40 -> 100 kt, roll 100 -> 130 kt.
        const double k_pitch = std::clamp((kcas - 40.0) / 60.0, 0.0, 1.0);
        const double k_roll = std::clamp((kcas - 100.0) / 30.0, 0.0, 1.0);
        control_phase = k_pitch < 0.5 ? 2 : 1;

        const double stick_p = deadband(std::clamp(stick.pitch_stick, -1.0, 1.0), STICK_DEADBAND);
        const double stick_r = deadband(std::clamp(stick.roll_stick, -1.0, 1.0), STICK_DEADBAND);
        const double pedal = deadband(std::clamp(stick.rudder_pedal, -1.0, 1.0), STICK_DEADBAND);
        const double lever_dev = deadband(std::clamp((lever - propulsion::kStovlThrottleDetent)
                                                     / propulsion::kStovlThrottleDetent, -1.0, 1.0),
                                          LEVER_DEADBAND);

        // ---- Throttle: acceleration, or ground speed near the hover --------
        const double w_gs = 1.0 - smoothstep((std::abs(gs_ref_) - GS_BLEND_LO_MPS) / (GS_BLEND_HI_MPS - GS_BLEND_LO_MPS));
        const double gs_rate = (1.0 - w_gs) * lever_dev * ACCEL_MAX_MPS2
            + w_gs * std::clamp(0.5 * (lever_dev * GS_COMMAND_MAX_MPS - gs_ref_), -ACCEL_MAX_MPS2, ACCEL_MAX_MPS2);
        gs_ref_ = std::clamp(gs_ref_ + gs_rate * dt, -GS_COMMAND_MAX_MPS, STOVL_SPEED_MAX_MPS);
        // Never run far ahead of a jet that cannot keep up (thrust-limited).
        gs_ref_ = std::clamp(gs_ref_, u_h - GS_LEAD_MAX_MPS, u_h + GS_LEAD_MAX_MPS);
        // Conversion speed window protection: no faster than the LiftFan's 250 KCAS.
        gs_ref_ = std::min(gs_ref_, u_h + (FAN_LIMIT_KCAS - 5.0 - kcas) / KT_PER_MPS);
        commanded_gs_mps = gs_ref_;

        // ---- Stick fore/aft: vertical speed, or flight-path rate at speed --
        const double vs_hover = stick_p * (stick_p > 0.0 ? CLIMB_MAX_MPS : SINK_MAX_MPS);
        const double gs_safe = std::max(std::abs(u_h), 1.0);
        const double hdot_ref_prev = hdot_ref_;
        hdot_ref_ += k_pitch * gs_safe * stick_p * GAMMA_RATE_MAX_DPS * D2R * dt
                   + (1.0 - k_pitch) * (vs_hover - hdot_ref_) * std::min(1.0, 4.0 * dt);
        const double vs_span = std::max(CLIMB_MAX_MPS, std::abs(u_h) * std::sin(20.0 * D2R));
        hdot_ref_ = std::clamp(hdot_ref_, -vs_span, vs_span);
        const double hdot_ref_rate = dt > 0.0 ? (hdot_ref_ - hdot_ref_prev) / dt : 0.0;

        // Vertical acceleration the engine can give: full power less the weight.
        const double rating = engine.max_dry_rating_n();
        const double a_up_max = rating * engine.lapse_factor(air, false) / mass.mass_kg - G0;

        // Height hold with the stick centred in the hover regime.
        double hdot_cmd = hdot_ref_;
        if (stick_p == 0.0 && k_pitch < 0.5 && !on_wheels) {
            // Captured where the jet can stop: a sink is arrested only by the
            // thrust margin, a light jet's few tenths of a G.
            if (!height_hold_) {
                height_hold_ = true;
                const double a_stop = hdot < 0.0 ? std::clamp(a_up_max, 0.3, 3.0) : 3.0;
                h_ref_ = h + hdot * std::abs(hdot) / (2.0 * a_stop);
            }
            hdot_cmd += (1.0 - k_pitch) * std::clamp(K_HEIGHT * (h_ref_ - h), -1.0, 1.0);
        } else {
            height_hold_ = false;
        }
        // Sink-rate protection closing in near the ground.
        if (k_pitch < 0.5) {
            const double sink_limit = std::clamp(TOUCHDOWN_SINK_MPS + SINK_LIMIT_PER_M * wheel_height_m,
                                                 TOUCHDOWN_SINK_MPS, SINK_MAX_MPS);
            hdot_cmd = std::max(hdot_cmd, -sink_limit);
        }
        commanded_vs_mps = hdot_cmd;

        // ---- Position hold: hands off, lever in the detent, in the hover ----
        double corr_fwd = 0.0, corr_right = 0.0;
        if (k_pitch < 0.5 && std::abs(gs_ref_) < 0.3 && lever_dev == 0.0 && stick_r == 0.0 && !on_wheels) {
            // Stop first (velocity hold), then hold where the jet came to rest.
            if (!position_hold_ && std::hypot(u_h, v_h) < 0.5) {
                position_hold_ = true;
                pos_ref_n_ = state.pos_ned.x;
                pos_ref_e_ = state.pos_ned.y;
            }
            const double en = pos_ref_n_ - state.pos_ned.x, ee = pos_ref_e_ - state.pos_ned.y;
            corr_fwd = std::clamp(K_POS * (en * cpsi + ee * spsi), -POS_CORR_MAX_MPS, POS_CORR_MAX_MPS);
            corr_right = std::clamp(K_POS * (-en * spsi + ee * cpsi), -POS_CORR_MAX_MPS, POS_CORR_MAX_MPS);
            if (!position_hold_) corr_fwd = corr_right = 0.0;
        } else {
            position_hold_ = false;
        }

        // ---- Accelerations the jet must fly ---------------------------------
        const double vs_err = hdot_cmd - hdot;
        double a_up = hdot_ref_rate + K_VS * vs_err + vs_int_;
        const double a_fwd = std::clamp(gs_rate + K_GS * (gs_ref_ + corr_fwd - u_h), -3.0, 3.0);
        const double a_lat = K_LAT * (corr_right - v_h);
        a_up = std::clamp(a_up, -3.0, 3.0);

        // ---- Attitude commands ----------------------------------------------
        // Bank: attitude command in the hover (centred holds the ground track),
        // roll rate with bank hold at speed.
        const double bank_max = (HOVER_BANK_MAX_DEG + (WING_BANK_MAX_DEG - HOVER_BANK_MAX_DEG) * k_pitch) * D2R;
        const double phi_hover = (stick_r != 0.0)
            ? stick_r * bank_max
            : std::clamp(std::atan(a_lat / G0), -10.0 * D2R, 10.0 * D2R);
        phi_ref_ = std::clamp(phi_ref_ + stick_r * ROLL_RATE_MAX_DPS * D2R * dt, -WING_BANK_MAX_DEG * D2R,
                              WING_BANK_MAX_DEG * D2R);
        const double phi_cmd = std::clamp((1.0 - k_roll) * phi_hover + k_roll * phi_ref_, -bank_max, bank_max);
        if (k_roll <= 0.0) phi_ref_ = phi_cmd;

        // ---- Force the jets must add to the airframe's ----------------------
        const math::Vector3 a_ned = state.q_att.rotate_body_to_ned(aero_b);
        const double aero_fwd = a_ned.x * cpsi + a_ned.y * spsi;
        const double f_fwd = mass.mass_kg * a_fwd - aero_fwd;
        const double f_up = mass.mass_kg * (G0 + a_up) + a_ned.z;
        // Body X and body-up components whose heading-frame sum is (f_fwd, f_up);
        // the lateral share comes from bank.
        const double jet_x = f_fwd * cth + f_up * sth;
        const double jet_up = (f_up * cth - f_fwd * sth) / std::max(cphi, 0.5);

        const double fan_share = lift.clutch * lift.config.lift_fan_thrust_n / rating;
        const double post_share = lift.roll_post_door() * lift.config.roll_post_thrust_n / rating;
        const double noz_share = 1.0 - fan_share - post_share;

        // Pitch: a steady hover attitude, going over with speed to the attitude
        // at which the wing carries what the jets do not. Even fully aft the
        // jets lift (fan vectored at most vane_max aft, roll posts, thrust
        // inclination): the lift that comes with the forward thrust needed.
        const double vane_aft = lift.config.vane_max_deg * D2R;
        const double ux_aft = fan_share * std::sin(vane_aft) + noz_share;
        const double uu_aft = fan_share * std::cos(vane_aft) + post_share;
        const double jet_lift_min = std::max(0.0, jet_x) / std::max(ux_aft, 0.2) * (uu_aft * cth * cphi + ux_aft * sth);
        const double lift_needed = (mass.mass_kg * (G0 + a_up) - jet_lift_min) / std::max(cphi, 0.5);
        const double alpha_wing = std::clamp(lift_needed / (std::max(imu.q_bar, 50.0) * aero.s_ref * aero.cl_alpha),
                                             0.0, WING_ALPHA_MAX_DEG * D2R);
        // Flight path from the command, not the measurement: following the
        // measured path would feed every climb back into more nose-up attitude.
        const double gamma = std::asin(std::clamp(hdot_cmd / std::max(v_air, 20.0), -0.5, 0.5));
        const double theta_cmd = std::clamp((1.0 - k_pitch) * HOVER_PITCH_DEG * D2R + k_pitch * (gamma + alpha_wing),
                                            PITCH_MIN_DEG * D2R, PITCH_MAX_DEG * D2R);

        // Heading: yaw rate (heading hold) in the hover, coordinated turn at speed.
        double psi_rate = pedal * YAW_RATE_MAX_DPS * D2R;
        if (pedal == 0.0 && k_pitch < 1.0) {
            // Captured where the yaw-rate loop stops the turn.
            if (!heading_hold_) { heading_hold_ = true; psi_ref_ = eul.yaw + state.omega_b.z / K_R; }
            psi_rate += std::clamp(K_PSI * wrap(psi_ref_ - eul.yaw), -10.0 * D2R, 10.0 * D2R);
        } else {
            heading_hold_ = false;
        }
        psi_rate = (1.0 - k_pitch) * psi_rate + k_pitch * G0 * std::tan(std::clamp(eul.roll, -1.3, 1.3)) / std::max(v_air, 30.0);

        // ---- Euler-rate commands -> body-rate commands -----------------------
        const double rate_lim = (30.0 + 170.0 * k_roll) * D2R;
        const double phi_dot = std::clamp(K_PHI * (phi_cmd - eul.roll), -rate_lim, rate_lim);
        const double theta_dot = std::clamp(K_THETA * (theta_cmd - eul.pitch), -15.0 * D2R, 15.0 * D2R);
        const double p_c = phi_dot - psi_rate * sth;
        const double q_c = theta_dot * cphi + psi_rate * sphi * cth;
        const double r_c = -theta_dot * sphi + psi_rate * cphi * cth;

        // ---- Rate loops: moment demands -------------------------------------
        const math::Vector3& w = state.omega_b;
        const double ep = p_c - w.x, eq = q_c - w.y, er = r_c - w.z;
        if (!on_wheels && dt > 0.0) {
            // Anti-windup: no more climb trim while the engine is already at full power.
            const bool thrust_limited = engine.dry_power >= 0.995 && vs_err > 0.0;
            if (!thrust_limited) vs_int_ = std::clamp(vs_int_ + KI_VS * std::clamp(vs_err, -0.5, 0.5) * dt, -2.0, 2.0);
            mp_int_ = std::clamp(mp_int_ + KI_P * ep * dt, -RATE_TRIM_MAX, RATE_TRIM_MAX);
            mq_int_ = std::clamp(mq_int_ + KI_Q * eq * dt, -RATE_TRIM_MAX, RATE_TRIM_MAX);
            mr_int_ = std::clamp(mr_int_ + KI_R * er * dt, -RATE_TRIM_MAX, RATE_TRIM_MAX);
        }
        const double l_cmd = mass.Ixx * (K_P * ep + mp_int_);
        const double m_cmd = mass.Iyy * (K_Q * eq + mq_int_);
        const double n_cmd = mass.Izz * (K_R * er + mr_int_);

        // ---- Thrust and its vector -------------------------------------------
        const double tilt = std::clamp(std::atan2(jet_x, jet_up), -5.0 * D2R, 90.0 * D2R);
        const double vane_cmd = std::clamp(tilt, lift.config.vane_min_deg * D2R, lift.config.vane_max_deg * D2R);
        const double x_rem = std::sin(tilt) - fan_share * std::sin(vane_cmd);
        const double up_rem = std::cos(tilt) - fan_share * std::cos(vane_cmd) - post_share;
        const double nozzle_cmd = std::clamp(std::atan2(up_rem, x_rem), 0.0, lift.config.nozzle_max_deg * D2R);

        // Thrust along what the effectors deliver now (they lag their commands).
        const double v_now = lift.vane_deg * D2R, n_now = lift.nozzle_deg * D2R;
        const double ux = fan_share * std::sin(v_now) + noz_share * std::cos(n_now);
        const double uu = fan_share * std::cos(v_now) + noz_share * std::sin(n_now) + post_share;
        // Best fit to both components; at speed the forward force comes first
        // (the wing then takes what lift comes with it, see jet_lift_min).
        double thrust = std::max(0.0, (jet_x * ux + jet_up * uu) / std::max(1e-6, ux * ux + uu * uu));
        thrust = std::max(thrust, k_pitch * jet_x / std::max(ux, 0.2));
        if (on_wheels && stick_p <= 0.0 && lever_dev <= 0.0) {
            // On the wheels with nothing asked of it (no lift-off, no short
            // take-off run on the lever): thrust back to idle.
            thrust_ground_ = std::max(0.0, thrust_ground_ - 0.5 * rating * dt);
            thrust = std::min(thrust, thrust_ground_);
            vs_int_ = 0.0;
        } else {
            thrust_ground_ = thrust;
        }
        out.lever = engine.lever_for_thrust(thrust, air);

        // ---- Moment allocation: aero surfaces first, then the lift system ---
        const double qs = imu.q_bar * aero.s_ref;
        const double w_aero = smoothstep((imu.q_bar - Q_AERO_LO) / (Q_AERO_HI - Q_AERO_LO));
        const double eff_de = qs * aero.c_bar * CM_DE * D2R;   // [N m / deg]
        const double eff_da = qs * aero.b_span * CL_DA * D2R;
        const double eff_dr = qs * aero.b_span * CN_DR * D2R;
        out.de_deg = surface(w_aero * m_cmd, eff_de, DE_LIMIT);
        out.da_deg = surface(w_aero * l_cmd, eff_da, DA_LIMIT);
        out.dr_deg = surface(w_aero * n_cmd, eff_dr, DR_LIMIT) - k_pitch * pedal * 0.5 * DR_LIMIT;

        const double per_n = engine.net_thrust_n / rating;
        const auto& c = lift.config;
        const double fan_n = per_n * lift.clutch * c.lift_fan_thrust_n;
        const double eff_split = c.fan_split_authority * fan_n
                               * (c.lift_fan_pos_b.x * std::cos(v_now) - c.nozzle_pos_b.x * std::sin(n_now));
        const double eff_roll = per_n * lift.roll_post_door() * c.roll_post_thrust_n * c.roll_post_pos_b.y;
        const double eff_yaw = -c.nozzle_pos_b.x * engine.net_thrust_n * noz_share;  // [N m / rad]

        const double m_rem = m_cmd - eff_de * out.de_deg;
        const double l_rem = l_cmd - eff_da * out.da_deg;
        const double n_rem = n_cmd - eff_dr * out.dr_deg;
        out.lift.pitch_split = eff_split > 1.0 ? std::clamp(m_rem / eff_split, -1.0, 1.0) : 0.0;
        out.lift.roll_split = eff_roll > 1.0 ? std::clamp(l_rem / eff_roll, -1.0, 1.0) : 0.0;
        out.lift.nozzle_yaw_deg = eff_yaw > 1.0 ? std::asin(std::clamp(n_rem / eff_yaw, -0.5, 0.5)) / D2R : 0.0;
        out.lift.nozzle_deg = nozzle_cmd / D2R;
        out.lift.vane_deg = vane_cmd / D2R;

        // The wing can take the jet back at the approach AoA (or it is parked).
        out.ctol_ok = on_wheels
            || qs * aero.cl_alpha * WING_ALPHA_MAX_DEG * D2R >= mass.mass_kg * G0;
        return out;
    }

private:
    bool engaged_{false};
    double gs_ref_{0.0};
    double hdot_ref_{0.0};
    double phi_ref_{0.0};
    bool height_hold_{false};
    double h_ref_{0.0};
    bool heading_hold_{false};
    double psi_ref_{0.0};
    bool position_hold_{false};
    double pos_ref_n_{0.0}, pos_ref_e_{0.0};
    double vs_int_{0.0}, mp_int_{0.0}, mq_int_{0.0}, mr_int_{0.0};
    double thrust_ground_{0.0};

    void engage(const fdm::FlightState& state, double u_h, double hdot) noexcept {
        engaged_ = true;
        gs_ref_ = std::clamp(u_h, -GS_COMMAND_MAX_MPS, STOVL_SPEED_MAX_MPS);
        hdot_ref_ = hdot;
        phi_ref_ = state.roll();
        height_hold_ = heading_hold_ = position_hold_ = false;
        vs_int_ = mp_int_ = mq_int_ = mr_int_ = 0.0;
        thrust_ground_ = 0.0;
    }

    [[nodiscard]] static double smoothstep(double t) noexcept {
        t = std::clamp(t, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }
    [[nodiscard]] static double deadband(double x, double band) noexcept {
        if (std::abs(x) <= band) return 0.0;
        return std::copysign((std::abs(x) - band) / (1.0 - band), x);
    }
    [[nodiscard]] static double wrap(double a) noexcept {
        return std::remainder(a, 2.0 * M_PI);
    }
    /// Surface deflection [deg] delivering @p moment at @p eff [N m / deg], within +-limit.
    [[nodiscard]] static double surface(double moment, double eff, double limit) noexcept {
        return std::abs(eff) > 1.0 ? std::clamp(moment / eff, -limit, limit) : 0.0;
    }
};

} // namespace fastjet::flcs
