/**
 * @file test_stovl.cpp
 * @brief F-35B STOVL: mode conversion, lift system and the unified STOVL control law.
 *
 * Every case flies the complete sim::Aircraft pipeline (with its default 5 m/s
 * wind and turbulence) over the flat airfield pad, so the hover, the
 * transitions and the vertical landing are tested exactly as the pilot flies
 * them.
 */
#include "fastjet/sim/aircraft.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

using namespace fastjet;
using aircraft::AircraftType;
using propulsion::StovlMode;

namespace {

constexpr double DT = sim::Aircraft::DT;
constexpr double D2R = M_PI / 180.0;
constexpr double KT = 0.514444;
constexpr double HOVER_FUEL_KG = 1500.0;   ///< A light jet: ~3,300 lb of fuel

fdm::FlightState at(double alt_m, double speed_mps, double pitch_deg = 0.0) {
    fdm::FlightState s{};
    s.pos_ned = math::Vector3(0.0, 0.0, -alt_m);
    s.q_att = math::Quaternion::from_euler(0.0, pitch_deg * D2R, 0.0);
    s.vel_b = s.q_att.rotate_ned_to_body(math::Vector3(speed_mps, 0.0, 0.0));
    return s;
}

/// A light F-35B hovering, converted, at @p alt_m.
void hover_start(sim::Aircraft& jet, double alt_m, double fuel_kg = HOVER_FUEL_KG) {
    jet.reset(at(alt_m, 0.0, 3.0));
    jet.start_converted_stovl(fuel_kg);
}

sim::AircraftControls stovl_hands_off() {
    sim::AircraftControls c{};
    c.throttle = propulsion::kStovlThrottleDetent;
    c.stovl_mode = true;
    return c;
}

void run(sim::Aircraft& jet, const sim::AircraftControls& c, double seconds) {
    for (int i = 0; i < static_cast<int>(seconds / DT); ++i) jet.step(DT, c);
}

double ground_speed_fwd(const sim::Aircraft& jet) {
    const math::Vector3 v = jet.state.velocity_ned();
    const double psi = jet.state.yaw();
    return v.x * std::cos(psi) + v.y * std::sin(psi);
}

double ground_speed_right(const sim::Aircraft& jet) {
    const math::Vector3 v = jet.state.velocity_ned();
    const double psi = jet.state.yaw();
    return -v.x * std::sin(psi) + v.y * std::cos(psi);
}

double kcas(const sim::Aircraft& jet) {
    const auto air = environment::Atmosphere1976::compute(jet.state.altitude(), jet.state.airspeed());
    return std::sqrt(2.0 * air.dynamic_pressure / environment::Atmosphere1976::RHO0) / KT;
}

/// Level CTOL flight: fly @p seconds holding @p alt_m with the stick and, if
/// @p hold_kcas > 0, that speed with the throttle.
void fly_level(sim::Aircraft& jet, sim::AircraftControls& c, double alt_m, double seconds, double hold_kcas = 0.0) {
    for (int i = 0; i < static_cast<int>(seconds / DT); ++i) {
        const double vs = -jet.state.velocity_ned().z;
        if (hold_kcas > 0.0) c.throttle = std::clamp(0.45 + 0.03 * (hold_kcas - kcas(jet)), 0.1, 0.85);
        c.stick.pitch_stick = std::clamp(0.004 * (alt_m - jet.state.altitude()) - 0.03 * vs, -0.5, 0.5);
        c.stick.roll_stick = std::clamp(-1.5 * jet.state.roll(), -0.5, 0.5);
        jet.step(DT, c);
    }
    c.stick = {};
}

// ---------------------------------------------------------------------------

void test_configuration() {
    const auto cfg = aircraft::AircraftConfig::get(AircraftType::F35B_LIGHTNING_II);
    assert(cfg.stovl.equipped);
    for (auto t : {AircraftType::F16_FIGHTING_FALCON, AircraftType::F15EX_EAGLE_II,
                   AircraftType::EUROFIGHTER_TYPHOON, AircraftType::F22_RAPTOR, AircraftType::A10_THUNDERBOLT}) {
        assert(!aircraft::AircraftConfig::get(t).stovl.equipped);
        sim::Aircraft other(t);
        other.start_converted_stovl();
        assert(!other.lift.engaged());
    }
    // 40,650 lbf total, P&W split.
    const double total = cfg.stovl.lift_fan_thrust_n + cfg.stovl.nozzle_thrust_n + cfg.stovl.roll_post_thrust_n;
    assert(std::abs(total / 4.44822 - 40650.0) < 50.0);
    // Fan and nozzle in moment balance about the CG.
    const double imbalance = cfg.stovl.lift_fan_thrust_n * cfg.stovl.lift_fan_pos_b.x
                           + cfg.stovl.nozzle_thrust_n * cfg.stovl.nozzle_pos_b.x;
    assert(std::abs(imbalance) < 1000.0);
    std::cout << "  configuration: lift rating " << total / 1000.0 << " kN\n";
}

void test_conversion_timing_and_inhibit() {
    // Above the LiftFan's 250 KCAS limit the selection is refused.
    {
        sim::Aircraft jet(AircraftType::F35B_LIGHTNING_II);
        jet.reset(at(1500.0, 290.0 * KT * 1.08));
        sim::AircraftControls c{};
        c.throttle = 0.80;
        c.stovl_mode = true;
        fly_level(jet, c, 1500.0, 5.0);
        assert(kcas(jet) > 255.0);
        assert(jet.stovl.mode == StovlMode::CTOL);
        assert(jet.stovl.conversion_inhibited);
        assert(jet.stovl.lift_fan_door == 0.0);
        std::cout << "  inhibit: " << kcas(jet) << " KCAS, still CTOL\n";
    }
    // At 200 kt: doors, clutch, lock-up, then the nozzle; ~15 s in all.
    sim::Aircraft jet(AircraftType::F35B_LIGHTNING_II);
    jet.reset(at(1500.0, 200.0 * KT * 1.08));
    jet.engine.fuel_kg = 2500.0;
    jet.refresh_mass();
    sim::AircraftControls c{};
    c.throttle = 0.60;
    fly_level(jet, c, 1500.0, 20.0, 200.0);
    c.throttle = propulsion::kStovlThrottleDetent;
    c.stovl_mode = true;
    double t_stovl = -1.0;
    double alt_min = 1e9, alt_max = -1e9;
    for (int i = 0; i < static_cast<int>(25.0 / DT); ++i) {
        jet.step(DT, c);
        alt_min = std::min(alt_min, jet.state.altitude());
        alt_max = std::max(alt_max, jet.state.altitude());
        if (t_stovl < 0.0 && jet.stovl.mode == StovlMode::STOVL) t_stovl = (i + 1) * DT;
    }
    std::printf("  conversion: STOVL locked up at %.1f s, %.0f KCAS, altitude band %.0f..%.0f m\n",
                t_stovl, kcas(jet), alt_min, alt_max);
    assert(t_stovl > 11.0 && t_stovl < 15.0);
    assert(jet.stovl.lift_fan_door == 1.0 && jet.stovl.clutch == 1.0);
    assert(alt_max - alt_min < 30.0);
    assert(std::abs(kcas(jet) - 200.0) < 15.0);   // detent holds speed
}

void test_afterburner_inhibited() {
    // The engine in STOVL mode never lights the reheat, whatever the lever.
    propulsion::MultiEngine e(AircraftType::F35B_LIGHTNING_II);
    e.stovl_mode = true;
    e.lift_fan_clutch = 1.0;
    const auto sl = environment::Atmosphere1976::compute(0.0, 0.0);
    for (int i = 0; i < 2000; ++i) e.update(1.0, sl, DT);
    assert(e.ab_thrust_n == 0.0);
    assert(e.detent_state != input::ThrottleController::DetentState::AFTERBURNER);
    // ...and the total reaches the 40,650 lbf lift rating.
    assert(std::abs(e.net_thrust_n - 180810.0) < 1500.0);

    // Converting with the reheat lit cancels it.
    sim::Aircraft jet(AircraftType::F35B_LIGHTNING_II);
    jet.reset(at(1500.0, 180.0 * KT * 1.08));
    sim::AircraftControls c{};
    c.throttle = 1.0;
    fly_level(jet, c, 1500.0, 6.0);
    assert(jet.engine.ab_thrust_n > 10000.0);
    c.stovl_mode = true;
    fly_level(jet, c, 1500.0, 1.5);
    std::printf("  AB: %.0f N reheat 1.5 s into conversion\n", jet.engine.ab_thrust_n);
    assert(jet.engine.ab_thrust_n < 500.0);
}

void test_hands_off_hover() {
    sim::Aircraft jet(AircraftType::F35B_LIGHTNING_II);
    hover_start(jet, 30.0);
    std::printf("  hover margin %.3f at %.0f kg\n", jet.stovl.hover_thrust_margin, jet.mass.mass_kg);
    const auto c = stovl_hands_off();
    const math::Vector3 p0 = jet.state.pos_ned;
    const double psi0 = jet.state.yaw();
    double drift = 0.0, dh = 0.0, att = 0.0;
    for (int i = 0; i < static_cast<int>(30.0 / DT); ++i) {
        jet.step(DT, c);
        const math::Vector3 d = jet.state.pos_ned - p0;
        drift = std::max(drift, std::hypot(d.x, d.y));
        dh = std::max(dh, std::abs(d.z));
        if (i * DT > 2.0) {
            att = std::max({att, std::abs(jet.state.roll()) / D2R, std::abs(jet.state.pitch() / D2R - 3.0),
                            std::abs(jet.state.yaw() - psi0) / D2R});
        }
    }
    std::printf("  hands-off 30 s: drift %.2f m, height %.2f m, attitude %.2f deg, phase %d, margin %.3f\n",
                drift, dh, att, jet.stovl.control_phase, jet.stovl.hover_thrust_margin);
    assert(!jet.crashed && jet.stovl.mode == StovlMode::STOVL);
    assert(jet.stovl.control_phase == 2);
    assert(drift < 3.0);
    assert(dh < 1.5);
    assert(att < 3.0);
}

void test_hover_commands() {
    sim::Aircraft jet(AircraftType::F35B_LIGHTNING_II);
    hover_start(jet, 60.0);
    auto c = stovl_hands_off();
    run(jet, c, 3.0);

    // Stick: vertical speed, +10 / -7 ft/s at full deflection. A light jet
    // has ~7% thrust margin, so the climb takes several seconds to build.
    c.stick.pitch_stick = 1.0;
    run(jet, c, 10.0);
    const double climb = -jet.state.velocity_ned().z;
    c.stick.pitch_stick = -1.0;
    run(jet, c, 6.0);
    const double sink = -jet.state.velocity_ned().z;
    c.stick.pitch_stick = 0.0;
    run(jet, c, 8.0);
    const double held = -jet.state.velocity_ned().z;
    std::printf("  stick: climb %.2f m/s (cmd 3.05), sink %.2f m/s (cmd -2.13), centred %.2f m/s\n",
                climb, sink, held);
    assert(std::abs(climb - 3.05) < 0.4);
    assert(std::abs(sink + 2.13) < 0.4);
    assert(std::abs(held) < 0.3);

    // Lateral stick: bank angle, and the jet slides that way.
    c.stick.roll_stick = 0.5;
    run(jet, c, 4.0);
    const double bank = jet.state.roll() / D2R;
    const double lateral = ground_speed_right(jet);
    c.stick.roll_stick = 0.0;
    run(jet, c, 8.0);
    const double lateral_after = ground_speed_right(jet);
    std::printf("  lateral stick 0.5: bank %.1f deg, %.2f m/s right; released: %.2f m/s, bank %.1f\n",
                bank, lateral, lateral_after, jet.state.roll() / D2R);
    assert(bank > 7.0 && bank < 13.0);
    assert(lateral > 2.0);
    assert(std::abs(lateral_after) < 0.6);

    // Pedals: yaw rate.
    c.stick.rudder_pedal = 0.5;
    run(jet, c, 3.0);
    const double r = jet.state.omega_b.z / D2R;
    c.stick.rudder_pedal = 0.0;
    run(jet, c, 3.0);
    std::printf("  pedal 0.5: yaw rate %.1f deg/s (cmd 15); released %.1f deg/s\n",
                r, jet.state.omega_b.z / D2R);
    assert(std::abs(r - 15.0) < 2.5);
    assert(std::abs(jet.state.omega_b.z / D2R) < 1.0);

    // Throttle lever: ground speed near the hover, detent = zero.
    c.throttle = propulsion::kStovlThrottleDetent + 0.25;   // halfway forward
    run(jet, c, 15.0);
    const double gs_fwd = ground_speed_fwd(jet);
    const double gs_cmd = jet.stovl.commanded_gs_mps;
    c.throttle = propulsion::kStovlThrottleDetent;
    run(jet, c, 15.0);
    std::printf("  lever +0.25: ground speed %.2f m/s (cmd %.2f); detent: %.2f m/s, height %.1f m\n",
                gs_fwd, gs_cmd, ground_speed_fwd(jet), jet.state.altitude());
    assert(gs_cmd > 6.0 && gs_cmd < 9.0);
    assert(std::abs(gs_fwd - gs_cmd) < 1.0);
    assert(std::abs(ground_speed_fwd(jet)) < 0.5);
    assert(!jet.crashed);
}

void test_heavy_jet_cannot_hover() {
    sim::Aircraft jet(AircraftType::F35B_LIGHTNING_II);
    const double full = aircraft::AircraftConfig::get(AircraftType::F35B_LIGHTNING_II).mass.internal_fuel_capacity_kg;
    hover_start(jet, 200.0, full);
    auto c = stovl_hands_off();
    c.stick.pitch_stick = 1.0;   // asking for a climb
    run(jet, c, 8.0);
    const double vs = -jet.state.velocity_ned().z;
    std::printf("  heavy (%.0f kg): margin %.3f, %.2f m/s after 8 s full power, %.0f m lost, lever %.2f\n",
                jet.mass.mass_kg, jet.stovl.hover_thrust_margin, vs, 200.0 - jet.state.altitude(),
                jet.engine.dry_power);
    assert(jet.stovl.hover_thrust_margin < 0.9);
    assert(vs < -5.0);
}

void test_transition_out_and_back() {
    sim::Aircraft jet(AircraftType::F35B_LIGHTNING_II);
    hover_start(jet, 100.0, 1800.0);
    auto c = stovl_hands_off();
    run(jet, c, 2.0);

    double worst_bank = 0.0, alt_min = 1e9, alt_max = -1e9, worst_alpha = 0.0;
    auto watch = [&] {
        worst_bank = std::max(worst_bank, std::abs(jet.state.roll()) / D2R);
        alt_min = std::min(alt_min, jet.state.altitude());
        alt_max = std::max(alt_max, jet.state.altitude());
        if (jet.state.airspeed() > 40.0) worst_alpha = std::max(worst_alpha, std::abs(jet.state.alpha()) / D2R);
        assert(!jet.crashed);
    };

    // Accelerate on the lever, level, until the wing can take the jet.
    c.throttle = 1.0;
    double t = 0.0;
    for (; t < 90.0 && kcas(jet) < 200.0; t += DT) { jet.step(DT, c); watch(); }
    std::printf("  out: 200 KCAS after %.1f s at %.0f m, nozzle %.0f deg, phase %d\n",
                t, jet.state.altitude(), jet.stovl.nozzle_pitch_deg, jet.stovl.control_phase);
    assert(jet.stovl.control_phase == 1);

    // Semi-jet-borne climb: the stick is flight-path rate, centred holds the path.
    c.throttle = propulsion::kStovlThrottleDetent;
    c.stick.pitch_stick = 0.5;
    run(jet, c, 2.0);
    c.stick.pitch_stick = 0.0;
    for (t = 0.0; t < 120.0 && jet.state.altitude() < 600.0; t += DT) { jet.step(DT, c); watch(); }
    const double climb_kcas = kcas(jet);
    c.stick.pitch_stick = -0.5;
    run(jet, c, 2.0);
    c.stick.pitch_stick = 0.0;
    run(jet, c, 5.0);
    std::printf("  climbed to %.0f m at %.0f KCAS, now %.2f m/s vertical\n", jet.state.altitude(), climb_kcas,
                -jet.state.velocity_ned().z);
    assert(std::abs(climb_kcas - 200.0) < 15.0);
    assert(std::abs(jet.state.velocity_ned().z) < 3.0);

    // Back to CTOL: the lever is thrust again.
    c.stovl_mode = false;
    for (t = 0.0; t < 12.0; t += DT) { jet.step(DT, c); watch(); }
    std::printf("  CTOL after reconversion: mode %s, %.0f KCAS, %.0f m\n",
                propulsion::to_string(jet.stovl.mode), kcas(jet), jet.state.altitude());
    assert(jet.stovl.mode == StovlMode::CTOL);
    fly_level(jet, c, jet.state.altitude(), 30.0, 200.0);
    watch();
    std::printf("  CTOL level: %.0f KCAS at %.0f m\n", kcas(jet), jet.state.altitude());

    // ...and back into the hover: convert, decelerate on the lever, hover.
    c.stovl_mode = true;
    c.throttle = propulsion::kStovlThrottleDetent;
    for (t = 0.0; t < 16.0; t += DT) { jet.step(DT, c); watch(); }
    std::printf("  reconverted: %s at %.0f KCAS, %.0f m\n", propulsion::to_string(jet.stovl.mode), kcas(jet),
                jet.state.altitude());
    assert(jet.stovl.mode == StovlMode::STOVL);
    c.throttle = 0.0;
    for (t = 0.0; t < 120.0 && ground_speed_fwd(jet) > 6.0; t += DT) { jet.step(DT, c); watch(); }
    c.throttle = propulsion::kStovlThrottleDetent;
    for (t = 0.0; t < 25.0; t += DT) { jet.step(DT, c); watch(); }
    std::printf("  back in the hover: gs %.2f m/s, vs %.2f, phase %d, worst bank %.1f, alpha %.1f, alt %.0f..%.0f\n",
                ground_speed_fwd(jet), -jet.state.velocity_ned().z, jet.stovl.control_phase,
                worst_bank, worst_alpha, alt_min, alt_max);
    assert(jet.stovl.control_phase == 2);
    assert(std::abs(ground_speed_fwd(jet)) < 0.5);
    assert(worst_bank < 15.0);
    assert(worst_alpha < 20.0);
    assert(alt_min > 80.0);
    assert(!jet.ofc.is_gcas_active());
}

void test_vertical_landing() {
    sim::Aircraft jet(AircraftType::F35B_LIGHTNING_II);
    hover_start(jet, 30.0);
    auto c = stovl_hands_off();
    run(jet, c, 3.0);
    c.stick.pitch_stick = -1.0;  // full forward: the law limits the sink near the ground
    double t = 0.0;
    for (; t < 40.0 && !jet.landing_gear.mains_on_ground(); t += DT) jet.step(DT, c);
    const double touchdown = jet.landing_gear.max_sink_rate;
    c.stick.pitch_stick = 0.0;
    run(jet, c, 5.0);
    std::printf("  VL: touchdown after %.1f s at %.2f m/s sink, engine %.0f N after 5 s on the ground\n",
                t, touchdown, jet.engine.net_thrust_n);
    assert(!jet.crashed && !jet.landing_gear.collapsed);
    assert(jet.landing_gear.mains_on_ground());
    assert(touchdown > 0.3 && touchdown < 1.6);
    assert(jet.engine.net_thrust_n < 30000.0);
}

void test_conventional_limits() {
    // 7 G, not 9.
    {
        sim::Aircraft jet(AircraftType::F35B_LIGHTNING_II);
        jet.reset(at(3000.0, 300.0));
        sim::AircraftControls c{};
        c.throttle = 0.85;
        fly_level(jet, c, 3000.0, 2.0);
        c.stick.pitch_stick = 1.0;
        double nz_max = 0.0;
        for (int i = 0; i < static_cast<int>(4.0 / DT); ++i) {
            jet.step(DT, c);
            nz_max = std::max(nz_max, jet.imu.Nz);
        }
        std::printf("  CTOL full aft stick at 300 m/s: %.2f G\n", nz_max);
        assert(nz_max > 6.5 && nz_max < 7.5);
        assert(jet.stovl.mode == StovlMode::CTOL);
    }
    // ~M1.6 flat out in reheat at altitude.
    {
        sim::Aircraft jet(AircraftType::F35B_LIGHTNING_II);
        jet.reset(at(11000.0, 300.0));
        sim::AircraftControls c{};
        c.throttle = 1.0;
        const double fuel = 0.6 * jet.engine.fuel_kg;
        for (int k = 0; k < 40; ++k) {
            jet.engine.fuel_kg = fuel;
            fly_level(jet, c, 11000.0, 10.0);
        }
        const auto air = environment::Atmosphere1976::compute(jet.state.altitude(), jet.state.airspeed());
        std::printf("  CTOL max reheat level at 11 km: M%.2f\n", air.mach_number);
        assert(air.mach_number > 1.5 && air.mach_number < 1.7);
    }
}

} // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::cout << std::unitbuf << "test_stovl: F-35B lift system and unified STOVL control law\n";
    test_configuration();
    test_afterburner_inhibited();
    test_conversion_timing_and_inhibit();
    test_hands_off_hover();
    test_hover_commands();
    test_heavy_jet_cannot_hover();
    test_vertical_landing();
    test_transition_out_and_back();
    test_conventional_limits();
    std::cout << "test_stovl: all passed\n";
    return 0;
}
