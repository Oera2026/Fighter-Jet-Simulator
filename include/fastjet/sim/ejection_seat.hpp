#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/environment/atmosphere1976.hpp"
#include "fastjet/environment/ground_collision.hpp"
#include "fastjet/fdm/flight_state.hpp"
#include "fastjet/fdm/six_dof_fdm.hpp"
#include "fastjet/math/matrix3x3.hpp"
#include "fastjet/math/quaternion.hpp"
#include "fastjet/math/vector3.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fastjet::sim {

/**
 * @brief The escape system fitted to each airframe, from open sources.
 *
 * All four are rocket-assisted, zero-zero seats with an automatic sequencer
 * that picks its timing from airspeed and altitude when the handle is pulled:
 *
 *  - ACES II (F-16C, F-22A, A-10C). Mode 1, below 250 KEAS and 15,000 ft:
 *    the main parachute is deployed as soon as the seat clears. Mode 2, 250-600
 *    KEAS: a drogue slows and stabilises the seat first, the main chute opens
 *    about a second later. Mode 3, above 15,000 ft: the pilot rides the seat
 *    down on the drogue (out of the thin, cold air, at a survivable descent
 *    rate) and the chute opens through 15,000 ft.
 *  - ACES 5 (F-15EX): the same modes, with arm and leg restraints and head and
 *    neck protection that push the injury threshold of high-speed escapes up.
 *  - Martin-Baker Mk16A (Typhoon): drogue-stabilised, with a barostatic
 *    release that holds the pilot on the drogue down to 10,000 ft.
 *  - Martin-Baker US16E (F-35B): the Mk16 family with twin side-rail
 *    catapults, rated 0/0 to 600 KEAS.
 *
 * Masses and drag areas are representative, chosen to give the published
 * trajectory apex (~60 m from a zero-zero ejection) and a 7-8 m/s descent
 * under a fully open canopy.
 */
struct EjectionSeatSpec {
    const char* name{"ACES II"};
    double canopy_delay_s{0.30};     ///< Handle pull to catapult fire (canopy jettisoned)
    double rail_angle_deg{30.0};     ///< Seat rails tilted aft of the body vertical
    double catapult_dv_mps{14.0};    ///< Speed up the rails as the seat leaves the cockpit
    double rocket_thrust_n{22000.0}; ///< Sustainer rocket under the seat pan (~13 G on the pilot)
    double rocket_burn_s{0.30};
    double seat_mass_kg{68.0};
    double pilot_mass_kg{105.0};     ///< Pilot, equipment, survival kit and parachute
    double mode2_keas{250.0};        ///< At or above this the drogue goes first
    double mode3_alt_m{4572.0};      ///< Above this the pilot rides the drogue down (15,000 ft)
    double chute_delay_mode1_s{0.20}; ///< After rocket burnout, low and slow
    double chute_delay_mode2_s{1.00}; ///< After rocket burnout, behind the drogue
    double chute_inflation_s{1.6};   ///< Line stretch to a full canopy
    double windblast_injury_keas{600.0};
    double windblast_fatal_keas{725.0};
    double tumble_cda_m2{0.75};      ///< Seat and pilot tumbling, no canopy
    double drogue_cda_m2{1.6};       ///< Seat and pilot behind the drogue
    double canopy_cda_m2{25.0};      ///< Main parachute fully open (C-9 class)
    /// Seat position in the cockpit, body axes from the CG [m].
    double seat_x_m{1.55};
    double seat_z_m{-0.25};

    [[nodiscard]] static constexpr EjectionSeatSpec of(aircraft::AircraftType t) noexcept {
        using T = aircraft::AircraftType;
        EjectionSeatSpec s{};
        switch (t) {
            case T::F16_FIGHTING_FALCON:
                break; // ACES II, 30 deg seat-back rails
            case T::F22_RAPTOR:
                s.rail_angle_deg = 15.0;
                break;
            case T::A10_THUNDERBOLT:
                s.rail_angle_deg = 15.0;
                s.canopy_delay_s = 0.35;
                break;
            case T::F15EX_EAGLE_II:
                s.name = "ACES 5";
                s.rail_angle_deg = 15.0;
                s.windblast_injury_keas = 650.0;
                s.windblast_fatal_keas = 760.0;
                s.chute_delay_mode1_s = 0.15;
                break;
            case T::EUROFIGHTER_TYPHOON:
                s.name = "MK16A";
                s.rail_angle_deg = 16.0;
                s.canopy_delay_s = 0.30;
                s.catapult_dv_mps = 16.0;
                s.rocket_thrust_n = 20000.0;
                s.rocket_burn_s = 0.35;
                s.seat_mass_kg = 80.0;
                s.mode3_alt_m = 3048.0; // barostatic release, 10,000 ft
                s.chute_delay_mode1_s = 0.45;
                s.chute_delay_mode2_s = 1.20;
                s.windblast_injury_keas = 625.0;
                break;
            case T::F35B_LIGHTNING_II:
                // Martin-Baker US16E: Mk16 family, twin catapults on side rails,
                // 0/0 to 600 KEAS and 50,000 ft. Release altitude as the Mk16A.
                s.name = "US16E";
                s.rail_angle_deg = 18.0;
                s.catapult_dv_mps = 16.0;
                s.rocket_thrust_n = 20000.0;
                s.rocket_burn_s = 0.35;
                s.seat_mass_kg = 85.0;
                s.mode3_alt_m = 3048.0;
                s.chute_delay_mode1_s = 0.40;
                s.chute_delay_mode2_s = 1.20;
                s.windblast_injury_keas = 600.0;
                s.windblast_fatal_keas = 700.0;
                break;
        }
        return s;
    }
};

/**
 * @brief One ejection, from the handle pull to the pilot on the ground.
 *
 * The seat and pilot are a point mass with a drag area that depends on what
 * is deployed: tumbling, behind the drogue, or under a canopy that inflates
 * over a second and a half. The sequencer picks its mode once, from the
 * airspeed and altitude at the pull, as the real one does. After the man and
 * seat separate the empty seat falls on its own.
 *
 * Survival is judged on what the pilot actually goes through: the windblast
 * at the pull, the deceleration when the canopy opens, and the speed at which
 * he meets the ground. An ejection that is too late (or inverted and low)
 * puts him into the ground before the canopy is open.
 */
class EjectionSeat {
public:
    enum class Phase : uint8_t {
        IDLE,        ///< In the jet
        CANOPY,      ///< Handle pulled, canopy jettisoning
        ROCKET,      ///< Up the rails and on the rocket
        DROGUE,      ///< Seat and pilot on the drogue (or tumbling, mode 1)
        CHUTE,       ///< Seat gone, main canopy inflating or open
        LANDED,
    };
    enum class Outcome : uint8_t { PENDING, SAFE, INJURED, KILLED };

    EjectionSeatSpec spec{};
    Phase phase{Phase::IDLE};
    Outcome outcome{Outcome::PENDING};
    int mode{0};                       ///< Sequencer mode 1-3 (0 before the pull)
    double time{0.0};                  ///< Since the handle pull [s]

    // The pilot (with the seat until separation)
    math::Vector3 pos{};
    math::Vector3 vel{};
    double chute_fraction{0.0};        ///< 0 packed, 1 fully open
    double peak_decel_g{0.0};          ///< Highest deceleration on the pilot
    double eject_keas{0.0};
    double eject_alt_m{0.0};
    double landing_speed_mps{0.0};
    const char* cause{""};             ///< What hurt or killed him

    // The empty seat after separation
    bool seat_free{false};
    math::Vector3 seat_pos{};
    math::Vector3 seat_vel{};

    /// Orientation of the pilot (body axes: X forward, Z toward the feet).
    math::Quaternion attitude{};

    /// Steady wind the parachute drifts with [m/s, NED].
    math::Vector3 wind{0.0, 5.0, 0.0};

    [[nodiscard]] bool initiated() const noexcept { return phase != Phase::IDLE; }
    /// The pilot has left the cockpit (the catapult has fired).
    [[nodiscard]] bool clear_of_jet() const noexcept { return phase >= Phase::ROCKET; }
    [[nodiscard]] bool seat_attached() const noexcept { return phase == Phase::ROCKET || phase == Phase::DROGUE; }
    [[nodiscard]] bool drogue_out() const noexcept { return phase == Phase::DROGUE && mode >= 2; }
    [[nodiscard]] double mass() const noexcept { return spec.pilot_mass_kg + (seat_attached() ? spec.seat_mass_kg : 0.0); }

    void reset() noexcept { *this = EjectionSeat{}; }

    /// @brief Pull the handle. The sequencer reads the jet's airspeed and
    /// altitude now; the catapult fires after the canopy delay.
    void initiate(aircraft::AircraftType type, const fdm::FlightState& jet) noexcept {
        if (initiated()) return;
        spec = EjectionSeatSpec::of(type);
        phase = Phase::CANOPY;
        outcome = Outcome::PENDING;
        time = 0.0;
        eject_alt_m = jet.altitude();
        eject_keas = keas(jet.altitude(), jet.airspeed());
        if (eject_alt_m > spec.mode3_alt_m) mode = 3;
        else mode = eject_keas >= spec.mode2_keas ? 2 : 1;
        jet_attitude_ = jet.q_att;
        attitude = jet.q_att;
        place_in_cockpit(jet);
    }

    /// @brief Advance one step. @p jet is the aircraft being left behind (the
    /// seat rides it until the catapult fires).
    void step(double dt, const fdm::FlightState& jet) noexcept {
        if (phase == Phase::IDLE || phase == Phase::LANDED) return;
        time += dt;
        constexpr double G0 = fdm::SixDoFFDM::GRAVITY_ACCEL;

        if (phase == Phase::CANOPY) {
            place_in_cockpit(jet);
            if (time >= spec.canopy_delay_s) fire_catapult(jet);
            return;
        }

        const double since_fire = time - fire_time_;
        // Sequencer.
        if (phase == Phase::ROCKET && since_fire >= spec.rocket_burn_s) phase = Phase::DROGUE;
        if (phase == Phase::DROGUE) {
            const double delay = mode == 1 ? spec.chute_delay_mode1_s : spec.chute_delay_mode2_s;
            const bool timed_out = since_fire >= spec.rocket_burn_s + delay;
            // Mode 3 holds on the drogue until the barostat sees the mode-3 altitude.
            const bool low_enough = mode < 3 || -pos.z <= spec.mode3_alt_m;
            if (timed_out && low_enough) separate();
        }

        // Forces on the pilot (and seat).
        const double alt = std::max(0.0, -pos.z);
        const math::Vector3 v_rel = vel - wind;
        const double v = v_rel.norm();
        const auto air = environment::Atmosphere1976::compute(alt, v);
        double cda = spec.tumble_cda_m2;
        if (phase == Phase::DROGUE && mode >= 2) cda = spec.drogue_cda_m2;
        if (phase == Phase::CHUTE) {
            const double open_t = time - chute_start_;
            chute_fraction = std::clamp(open_t / spec.chute_inflation_s, 0.0, 1.0);
            // Drag area grows roughly with the square of the inflating mouth.
            cda = spec.tumble_cda_m2 * 0.5 + spec.canopy_cda_m2 * chute_fraction * chute_fraction;
        }
        const double m = mass();
        math::Vector3 drag{};
        if (v > 1e-3) drag = v_rel * (-0.5 * air.density * v * cda / m);
        math::Vector3 acc = drag + math::Vector3(0.0, 0.0, G0);
        if (phase == Phase::ROCKET) acc += rail_dir_ * (spec.rocket_thrust_n / m);
        // Load on the pilot: everything but gravity.
        const double load_g = (acc - math::Vector3(0.0, 0.0, G0)).norm() / G0;
        if (phase == Phase::CHUTE) peak_decel_g = std::max(peak_decel_g, load_g);

        vel += acc * dt;
        pos += vel * dt;
        update_attitude(dt);

        if (seat_free) {
            const math::Vector3 sv = seat_vel - wind;
            const double s = sv.norm();
            const auto sair = environment::Atmosphere1976::compute(std::max(0.0, -seat_pos.z), s);
            const math::Vector3 sd = s > 1e-3 ? sv * (-0.5 * sair.density * s * 0.5 / spec.seat_mass_kg) : math::Vector3{};
            seat_vel += (sd + math::Vector3(0.0, 0.0, G0)) * dt;
            seat_pos += seat_vel * dt;
            const double g = environment::GroundCollision::get_ground_z(seat_pos.x, seat_pos.y);
            if (seat_pos.z > g) {
                seat_pos.z = g;
                seat_vel = {};
            }
        }

        const double ground_z = environment::GroundCollision::get_ground_z(pos.x, pos.y);
        if (pos.z >= ground_z) land(ground_z);
    }

    /// @brief Knots equivalent airspeed: what the windblast actually is.
    [[nodiscard]] static double keas(double alt_m, double tas_mps) noexcept {
        const auto air = environment::Atmosphere1976::compute(std::max(0.0, alt_m), tas_mps);
        return tas_mps * std::sqrt(air.density / 1.225) / 0.514444;
    }

    /// Survivable vertical speeds at touchdown [m/s]: a parachute landing fall
    /// absorbs up to about 9; above 13 the injuries are fatal.
    static constexpr double LANDING_SAFE_MPS = 9.0;
    static constexpr double LANDING_FATAL_MPS = 13.0;
    /// Canopy opening shock limits [g].
    static constexpr double OPENING_INJURY_G = 22.0;
    static constexpr double OPENING_FATAL_G = 35.0;

private:
    math::Quaternion jet_attitude_{};
    math::Vector3 rail_dir_{0.0, 0.0, -1.0};
    double fire_time_{0.0};
    double chute_start_{0.0};

    void place_in_cockpit(const fdm::FlightState& jet) noexcept {
        pos = jet.pos_ned + jet.q_att.rotate_body_to_ned(math::Vector3(spec.seat_x_m, 0.0, spec.seat_z_m));
        vel = jet.velocity_ned();
        attitude = jet.q_att;
    }

    void fire_catapult(const fdm::FlightState& jet) noexcept {
        const double a = spec.rail_angle_deg * M_PI / 180.0;
        // Up the rails: body -Z, tilted aft by the rail angle.
        rail_dir_ = jet.q_att.rotate_body_to_ned(math::Vector3(-std::sin(a), 0.0, -std::cos(a)));
        place_in_cockpit(jet);
        vel = vel + rail_dir_ * spec.catapult_dv_mps;
        jet_attitude_ = jet.q_att;
        fire_time_ = time;
        phase = Phase::ROCKET;
        // The windblast hits as the seat clears the canopy rail.
        if (eject_keas >= spec.windblast_fatal_keas) hurt(Outcome::KILLED, "WINDBLAST");
        else if (eject_keas >= spec.windblast_injury_keas) hurt(Outcome::INJURED, "WINDBLAST");
    }

    void separate() noexcept {
        phase = Phase::CHUTE;
        chute_start_ = time;
        seat_free = true;
        seat_pos = pos + attitude.rotate_body_to_ned(math::Vector3(0.0, 0.0, 0.4));
        seat_vel = vel;
    }

    void land(double ground_z) noexcept {
        pos.z = ground_z;
        landing_speed_mps = std::max(0.0, vel.z);
        vel = {};
        phase = Phase::LANDED;
        if (chute_fraction < 0.5 || landing_speed_mps > LANDING_FATAL_MPS) {
            hurt(Outcome::KILLED, chute_fraction < 0.5 ? "CHUTE NOT OPEN" : "GROUND IMPACT");
        } else if (landing_speed_mps > LANDING_SAFE_MPS) {
            hurt(Outcome::INJURED, "HARD LANDING");
        }
        if (peak_decel_g > OPENING_FATAL_G) hurt(Outcome::KILLED, "OPENING SHOCK");
        else if (peak_decel_g > OPENING_INJURY_G) hurt(Outcome::INJURED, "OPENING SHOCK");
        if (outcome == Outcome::PENDING) outcome = Outcome::SAFE;
    }

    void hurt(Outcome o, const char* why) noexcept {
        if (static_cast<int>(o) > static_cast<int>(outcome)) {
            outcome = o;
            cause = why;
        }
    }

    /// @brief The pilot hangs from whatever is streaming behind him: on the
    /// rocket the seat keeps the jet's attitude; after that his head points
    /// back along the airflow (up, once the descent is vertical), facing the
    /// way the jet was heading.
    void update_attitude(double dt) noexcept {
        if (phase == Phase::ROCKET) {
            attitude = jet_attitude_;
            return;
        }
        const math::Vector3 v_rel = vel - wind;
        const double v = v_rel.norm();
        if (v < 0.5) return;
        const math::Vector3 down = v_rel / v; // body Z: toward the feet
        // Face the jet's old heading, square to the hanging axis.
        math::Vector3 fwd = jet_attitude_.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
        fwd = fwd - down * fwd.dot(down);
        if (fwd.norm() < 1e-3) fwd = math::Vector3(1.0, 0.0, 0.0) - down * down.x;
        fwd = fwd.normalized();
        const math::Vector3 right = down.cross(fwd);
        math::Matrix3x3 c_bn;
        // Columns of the body-to-NED matrix are the body axes in NED.
        c_bn(0, 0) = fwd.x;   c_bn(1, 0) = fwd.y;   c_bn(2, 0) = fwd.z;
        c_bn(0, 1) = right.x; c_bn(1, 1) = right.y; c_bn(2, 1) = right.z;
        c_bn(0, 2) = down.x;  c_bn(1, 2) = down.y;  c_bn(2, 2) = down.z;
        const math::Quaternion target = math::Quaternion::from_dcm_body_to_ned(c_bn);
        // Swings to the new hang over about half a second.
        const double k = 1.0 - std::exp(-dt / 0.5);
        attitude = math::Quaternion::slerp(attitude, target, k);
    }
};

[[nodiscard]] constexpr const char* to_string(EjectionSeat::Phase p) noexcept {
    switch (p) {
        case EjectionSeat::Phase::IDLE:   return "ARMED";
        case EjectionSeat::Phase::CANOPY: return "CANOPY JETTISON";
        case EjectionSeat::Phase::ROCKET: return "CATAPULT / ROCKET";
        case EjectionSeat::Phase::DROGUE: return "DROGUE";
        case EjectionSeat::Phase::CHUTE:  return "PARACHUTE";
        case EjectionSeat::Phase::LANDED: return "ON THE GROUND";
    }
    return "?";
}

[[nodiscard]] constexpr const char* to_string(EjectionSeat::Outcome o) noexcept {
    switch (o) {
        case EjectionSeat::Outcome::PENDING: return "";
        case EjectionSeat::Outcome::SAFE:    return "PILOT SAFE";
        case EjectionSeat::Outcome::INJURED: return "PILOT INJURED";
        case EjectionSeat::Outcome::KILLED:  return "PILOT KILLED";
    }
    return "?";
}

} // namespace fastjet::sim
