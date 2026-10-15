#include "fastjet/flcs/onboard_flight_computer.hpp"
#include "fastjet/fdm/flight_state.hpp"
#include "fastjet/flcs/imu_sensor.hpp"
#include "fastjet/environment/ground_collision.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace fastjet;
using namespace fastjet::flcs;

/// Flies a hand-placed state on along its velocity (attitude frozen) until
/// Auto-GCAS engages, as the jet would close on the terrain. The recovery fires
/// at the last instant (planned to bottom out at its 30 ft buffer), so a state
/// that sat "inside the margin" of the old, earlier trigger is flown on to
/// where it now triggers. Returns false if the jet reached the ground first.
bool fly_until_gcas(OnBoardFlightComputer& ofc, fdm::FlightState& state, const IMUData& imu,
                    const PilotCommands& pilot, PilotCommands& out, double max_s = 10.0) {
    constexpr double dt = 0.005;
    for (double t = 0.0; t < max_s; t += dt) {
        out = ofc.update(dt, state, imu, pilot);
        if (ofc.is_gcas_active()) return true;
        state.pos_ned = state.pos_ned + state.velocity_ned() * dt;
        if (state.altitude() <= environment::GroundCollision::get_terrain_height(state.pos_ned.x, state.pos_ned.y))
            return false;
    }
    return false;
}

void test_gcas_does_not_intervene_on_shallow_dive() {
    std::cout << "[Test] GCAS: Shallow dive (-12 deg) at 150m does NOT trigger GCAS...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    // Airspeed 200 m/s, diving at -12 degrees (fpa ~ -0.21 rad)
    state.pos_ned = math::Vector3(0, 0, -150.0); // 150 m altitude
    state.vel_b   = math::Vector3(200.0, 0.0, 0.0);
    // Pitch down -12 degrees
    const double pitch = -12.0 * (M_PI / 180.0);
    state.q_att = math::Quaternion::from_euler(0.0, pitch, 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    PilotCommands out = ofc.update(0.005, state, imu, pilot_in);
    assert(!ofc.is_gcas_active());
    assert(out.pitch_stick == 0.0);
    std::cout << "  PASSED (GCAS remained ARMED, zero nuisance intervention)\n";
}

void test_gcas_immediate_handover_when_level() {
    std::cout << "[Test] GCAS: Hands control back to pilot as soon as descent stops...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    // Steep dive at -30 deg, low altitude (100 m) -> forces GCAS to engage
    state.pos_ned = math::Vector3(0, 0, -100.0);
    state.vel_b   = math::Vector3(220.0, 0.0, 0.0);
    const double steep_pitch = -30.0 * (M_PI / 180.0);
    state.q_att = math::Quaternion::from_euler(0.0, steep_pitch, 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    // First step: triggers GCAS
    (void)ofc.update(0.005, state, imu, pilot_in);
    assert(ofc.is_gcas_active());
    std::cout << "  GCAS engaged on critical dive (state=" << static_cast<int>(ofc.gcas_state) << ")\n";

    // Simulate recovery: jet rotates to level flight (fpa >= 0.0, pitch >= 0.0, zero vertical descent)
    state.q_att = math::Quaternion::from_euler(0.0, 2.0 * (M_PI / 180.0), 0.0);
    state.pos_ned = math::Vector3(0, 0, -80.0); // 80 m altitude, climbing

    (void)ofc.update(0.005, state, imu, pilot_in);
    // GCAS must release immediately!
    assert(!ofc.is_gcas_active());
    assert(ofc.gcas_state == OnBoardFlightComputer::GcasState::SAFE);
    std::cout << "  PASSED (GCAS immediately disengaged and handed control back to pilot upon leveling out)\n";
}

void test_gcas_pilot_breakout_override() {
    std::cout << "[Test] GCAS: Pilot stick override yields control back to pilot...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -120.0);
    state.vel_b   = math::Vector3(200.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::from_euler(0.0, -25.0 * (M_PI / 180.0), 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    (void)ofc.update(0.005, state, imu, pilot_in);
    assert(ofc.is_gcas_active());

    // Pilot forcefully pulls stick (+0.60) to command recovery
    PilotCommands pilot_pull{0.60, 0.0, 0.0};
    // Pitch has begun arresting
    state.q_att = math::Quaternion::from_euler(0.0, -6.0 * (M_PI / 180.0), 0.0);

    (void)ofc.update(0.005, state, imu, pilot_pull);
    assert(!ofc.is_gcas_active());
    assert(ofc.gcas_state == OnBoardFlightComputer::GcasState::SAFE);
    std::cout << "  PASSED (Pilot pull-up breakout took immediate control)\n";
}

void test_gcas_engages_on_shallow_but_low_dive() {
    std::cout << "[Test] GCAS: Shallow (-15 deg) but dangerously low (50m) dive DOES trigger...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -50.0); // 50 m AGL
    state.vel_b   = math::Vector3(300.0, 0.0, 0.0);
    const double pitch = -15.0 * (M_PI / 180.0);
    state.q_att = math::Quaternion::from_euler(0.0, pitch, 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    (void)ofc.update(0.005, state, imu, pilot_in);
    assert(ofc.is_gcas_active());
    std::cout << "  PASSED (engagement is judged on recovery margin, not dive steepness)\n";
}

void test_gcas_commands_nominal_g_not_max_g() {
    std::cout << "[Test] GCAS: Dive pull-up commands nominal recovery G, not max-G...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    // Engaged right at the margin: the nominal 5 G arc still fits.
    state.pos_ned = math::Vector3(0, 0, -480.0);
    state.vel_b   = math::Vector3(250.0, 0.0, 0.0);
    const double pitch = -35.0 * (M_PI / 180.0);
    state.q_att = math::Quaternion::from_euler(0.0, pitch, 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    PilotCommands out{};
    assert(fly_until_gcas(ofc, state, imu, pilot_in, out));
    assert(ofc.is_gcas_active());
    // pitch_stick == 1.0 would demand the airframe's full 9G limit; the nominal
    // 5G recovery target should sit around (5-1)/(9-1) = 0.5, well short of max.
    assert(out.pitch_stick > 0.3 && out.pitch_stick < 0.7);
    std::cout << "  PASSED (pitch_stick=" << out.pitch_stick << ", not a max-G 1.0 pull)\n";
}

void test_gcas_escalates_when_nominal_arc_no_longer_fits() {
    std::cout << "[Test] GCAS: Too low for the nominal arc -> escalates, but only to the capped ceiling...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    // 150 m AGL, 35 deg dive at 250 m/s: a 5 G arc needs ~300 m.  Holding 5 G
    // until time-to-impact dropped below 1.2 s wasted the seconds in which the
    // extra G still made the difference.
    state.pos_ned = math::Vector3(0, 0, -150.0);
    state.vel_b   = math::Vector3(250.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::from_euler(0.0, -35.0 * (M_PI / 180.0), 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands out = ofc.update(0.005, state, imu, PilotCommands{});
    assert(ofc.is_gcas_active());
    const double ceiling_stick = (OnBoardFlightComputer::GCAS_MAX_RECOVERY_G - 1.0) / (9.0 - 1.0);
    assert(out.pitch_stick > 0.7);
    assert(out.pitch_stick <= ceiling_stick + 1e-9); // never the airframe's full 9 G
    std::cout << "  PASSED (pitch_stick=" << out.pitch_stick << " <= ceiling " << ceiling_stick << ")\n";
}

void test_gcas_hard_deck_engages_without_dive() {
    std::cout << "[Test] GCAS: Hard 30ft deck engages on a near-level sink with no dive trigger...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -8.0); // 8 m AGL, under the 9.144 m (30 ft) deck
    state.vel_b   = math::Vector3(150.0, 0.0, 0.0);
    // Nearly level -- far shallower than the kinematic arming threshold.
    const double pitch = -0.5 * (M_PI / 180.0);
    state.q_att = math::Quaternion::from_euler(0.0, pitch, 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    // Gear up: this must never be waved through as a landing.
    (void)ofc.update(0.005, state, imu, pilot_in, /*gear_down=*/false);
    assert(ofc.is_gcas_active());
    std::cout << "  PASSED (hard deck is an unconditional backstop, not just a dive detector)\n";
}

void test_gcas_does_not_disturb_normal_landing() {
    std::cout << "[Test] GCAS: Normal landing flare (gear down) is not disturbed...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -5.0); // 5 m AGL, in the flare
    state.vel_b   = math::Vector3(75.0, 0.0, 0.0); // ~146 kt approach speed
    const double pitch = -2.0 * (M_PI / 180.0); // gentle glideslope
    state.q_att = math::Quaternion::from_euler(0.0, pitch, 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    (void)ofc.update(0.005, state, imu, pilot_in, /*gear_down=*/true);
    assert(!ofc.is_gcas_active());
    assert(!ofc.has_throttle_override());
    std::cout << "  PASSED (gear-down landing flare is left alone)\n";
}

void test_gcas_throttle_authority() {
    std::cout << "[Test] GCAS: Commands reduced throttle while banking, full power while pulling out...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -150.0);
    state.vel_b   = math::Vector3(220.0, 0.0, 0.0);
    // Steep dive AND heavily banked -> engages via WING_LEVEL first.
    state.q_att = math::Quaternion::from_euler(60.0 * (M_PI / 180.0), -30.0 * (M_PI / 180.0), 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    (void)ofc.update(0.005, state, imu, pilot_in);
    assert(ofc.gcas_state == OnBoardFlightComputer::GcasState::WING_LEVEL);
    assert(ofc.has_throttle_override());
    assert(ofc.throttle_override_value() < 0.3);

    // Roll through to the escape's bank (wings level, or a climbing turn's)
    // -> PULL_UP, full power.
    for (double bank = 60.0; bank > -90.0 && ofc.gcas_state == OnBoardFlightComputer::GcasState::WING_LEVEL;
         bank -= 1.0) {
        state.q_att = math::Quaternion::from_euler(bank * (M_PI / 180.0), -30.0 * (M_PI / 180.0), 0.0);
        (void)ofc.update(0.005, state, imu, pilot_in);
    }
    assert(ofc.gcas_state == OnBoardFlightComputer::GcasState::PULL_UP);
    assert(ofc.has_throttle_override());
    assert(ofc.throttle_override_value() == 1.0);
    std::cout << "  PASSED (idle while banking, full power once pulling out)\n";
}

void test_gloc_prompt_pilot_handover() {
    std::cout << "[Test] GLOC: Pilot inceptor command immediately restores 100% authority once G is safe...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -1500.0);
    state.vel_b   = math::Vector3(250.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::identity();

    IMUData imu{};
    imu.Nz = 10.0; // Extreme G
    PilotCommands neutral{0.0, 0.0, 0.0};

    // Drive into GLOC
    while (ofc.gloc_state != OnBoardFlightComputer::GlocState::GLOC) {
        (void)ofc.update(0.01, state, imu, neutral);
    }
    assert(ofc.is_gloc_active());
    std::cout << "  GLOC entered. Authority = " << ofc.gloc_authority << "\n";

    // G drops to safe level (1.2G), wings are level
    imu.Nz = 1.2;
    // Pilot provides conscious stick input
    PilotCommands pilot_awake{0.35, 0.0, 0.0};
    (void)ofc.update(0.01, state, imu, pilot_awake);

    // Pilot authority must be instantly 100% and state NORMAL
    assert(!ofc.is_gloc_active());
    assert(ofc.gloc_authority == 1.0);
    assert(ofc.gloc_state == OnBoardFlightComputer::GlocState::NORMAL);
    std::cout << "  PASSED (Pilot stick movement instantly restored 100% authority)\n";
}

void test_tumble_spin_auto_recovery() {
    std::cout << "[Test] Tumble: Out-of-control spin & deep stall triggers auto-recovery...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -3000.0);
    // Severe flat spin: high yaw rate (1.2 rad/s ~ 68 deg/s), high alpha (38 deg), decaying airspeed (75 m/s)
    state.vel_b = math::Vector3(75.0, 25.0, 60.0);
    state.omega_b = math::Vector3(0.5, 0.4, 1.2); // violent multi-axis tumble with right spin

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_frozen{0.0, 0.0, 0.0};

    // Step for 0.3 s (exceeds 0.25 s persistence filter)
    for (int i = 0; i < 60; ++i) {
        (void)ofc.update(0.005, state, imu, pilot_frozen);
    }

    assert(ofc.is_tumble_active());
    assert(ofc.tumble_state == OnBoardFlightComputer::TumbleState::RECOVERING);

    // Verify recovery commands: full opposite left rudder against right spin, nose-down pitch to break deep stall
    PilotCommands rec_cmd = ofc.update(0.005, state, imu, pilot_frozen);
    assert(rec_cmd.rudder_pedal < -0.80); // full left anti-spin rudder
    assert(rec_cmd.pitch_stick < -0.80);  // full nose-down pitch to break deep stall
    std::cout << "  PASSED (Auto-Tumble engaged: commanded anti-spin rudder "
              << rec_cmd.rudder_pedal << " and nose-down pitch " << rec_cmd.pitch_stick << ")\n";

    // Simulate airframe stabilizing: rotation stopped, alpha returned to 8 deg, airspeed restored to 160 m/s
    state.vel_b = math::Vector3(160.0, 0.0, 10.0);
    state.omega_b = math::Vector3(0.05, 0.05, 0.05); // tranquil rates (< 3 deg/s)

    (void)ofc.update(0.005, state, imu, pilot_frozen);
    assert(!ofc.is_tumble_active());
    assert(ofc.tumble_state == OnBoardFlightComputer::TumbleState::SAFE);
    std::cout << "  PASSED (Tumble Auto-Recovery disengaged and handed control back to pilot once stabilized)\n";
}

void test_tumble_does_not_intervene_on_fast_roll() {
    std::cout << "[Test] Tumble: Rapid controlled roll (250 deg/s) does NOT trigger auto-tumble override...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -2500.0);
    state.vel_b   = math::Vector3(250.0, 0.0, 0.0);
    // 250 deg/s roll rate (~4.36 rad/s)
    state.omega_b = math::Vector3(250.0 * (M_PI / 180.0), 0.0, 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_roll{0.0, 0.90, 0.0}; // Pilot actively commanding roll stick

    // Simulate 2 full seconds (400 steps) of fast continuous rolling
    for (int i = 0; i < 400; ++i) {
        PilotCommands cmd = ofc.update(0.005, state, imu, pilot_roll);
        assert(!ofc.is_tumble_active());
        assert(cmd.roll_stick == pilot_roll.roll_stick); // Zero interference with roll command!
    }
    std::cout << "  PASSED (OFC remained ARMED with zero roll interference over 2s / 500 deg of roll)\n";
}

void test_gcas_avoids_mountain_ahead_in_level_flight() {
    std::cout << "[Test] GCAS: Forward terrain look-ahead detects rising mountain and commands pull-up...\n";
    OnBoardFlightComputer ofc;

    // Find an elevated mountain point
    double peak_x = 0.0, peak_y = 0.0, peak_h = 0.0;
    for (double x = 20000.0; x <= 45000.0; x += 500.0) {
        for (double y = -20000.0; y <= 20000.0; y += 500.0) {
            const double h = environment::GroundCollision::get_terrain_height(x, y);
            if (h > 300.0) {
                peak_x = x;
                peak_y = y;
                peak_h = h;
                break;
            }
        }
        if (peak_h > 300.0) break;
    }
    assert(peak_h > 300.0);

    // Place jet 800m before the peak along the X-axis, heading directly towards the peak (+X)
    // Flying level at an altitude below the peak (e.g. 50m below the peak)
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(peak_x - 800.0, peak_y, -(peak_h - 50.0));
    state.vel_b   = math::Vector3(250.0, 0.0, 0.0); // 250 m/s toward peak
    state.q_att   = math::Quaternion::identity();   // Perfectly level flight

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_hands_off{0.0, 0.0, 0.0};

    // Even though the aircraft is flying perfectly level (fpa = 0.0),
    // predictive terrain look-ahead must sense the rising mountain ahead and
    // engage before the slope, with the 5 G pull-up still enough.
    PilotCommands out{};
    assert(fly_until_gcas(ofc, state, imu, pilot_hands_off, out, 3.0));
    assert(ofc.is_gcas_active());
    assert(state.pos_ned.x < peak_x - 300.0);
    assert(out.pitch_stick > 0.40); // Commanding autonomous pull-up
    assert(ofc.gcas_tti_seconds() < 4.0);

    std::cout << "  PASSED (Rising mountain at " << peak_h << "m MSL detected ahead; GCAS commanded pull-up stick="
              << out.pitch_stick << ", TTI=" << ofc.gcas_tti_seconds() << "s)\n";
}

void test_gcas_elevated_terrain_dive() {
    std::cout << "[Test] GCAS: Dive over elevated mountain terrain uses true AGL, not MSL altitude...\n";
    OnBoardFlightComputer ofc;

    // Find a mountain point
    double hill_x = 0.0, hill_y = 0.0, hill_h = 0.0;
    for (double x = 25000.0; x <= 45000.0; x += 500.0) {
        for (double y = -20000.0; y <= 20000.0; y += 500.0) {
            const double h = environment::GroundCollision::get_terrain_height(x, y);
            if (h > 250.0) {
                hill_x = x;
                hill_y = y;
                hill_h = h;
                break;
            }
        }
        if (hill_h > 250.0) break;
    }
    assert(hill_h > 250.0);

    // Place aircraft at 60m AGL above the mountain (MSL altitude = hill_h + 60m)
    // Diving at -20 degrees at 220 m/s
    // With old MSL logic, altitude was (hill_h + 60) > 310m, so it ignored the dive!
    // With true terrain AGL (60m), GCAS must trigger!
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(hill_x, hill_y, -(hill_h + 60.0));
    state.vel_b   = math::Vector3(220.0, 0.0, 0.0);
    const double pitch = -20.0 * (M_PI / 180.0);
    state.q_att   = math::Quaternion::from_euler(0.0, pitch, 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    (void)ofc.update(0.005, state, imu, pilot_in);
    assert(ofc.is_gcas_active());

    std::cout << "  PASSED (GCAS engaged at " << (hill_h + 60.0) << "m MSL because terrain elevation is "
              << hill_h << "m -> true AGL is only 60m)\n";
}

// ===========================================================================
// Regression tests: loss-of-control failure modes in complex attitudes.
// Each of these drove the jet into the ground before being fixed.
// ===========================================================================

void test_gcas_never_pulls_while_inverted() {
    std::cout << "[Test] GCAS: Inverted recovery unloads instead of pulling into the ground...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -90.0); // 90 m AGL, well inside the margin
    state.vel_b   = math::Vector3(220.0, 0.0, 0.0);
    // Inverted, nose 10 deg below the horizon.
    state.q_att = math::Quaternion::from_euler(179.0 * (M_PI / 180.0),
                                               -10.0 * (M_PI / 180.0), 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    PilotCommands out{};
    assert(fly_until_gcas(ofc, state, imu, pilot_in, out, 1.0));
    assert(ofc.is_gcas_active());
    // Only cos(bank) of a pull opposes the descent. Inverted that term is negative,
    // so a nose-up command would drive the nose *into* the terrain.
    assert(out.pitch_stick <= 0.0);
    // ...and it must be rolling hard toward upright rather than sitting there.
    assert(std::abs(out.roll_stick) > 0.9);
    std::cout << "  PASSED (pitch_stick=" << out.pitch_stick << " <= 0 while inverted, rolling upright"
              << " roll_stick=" << out.roll_stick << ")\n";
}

void test_gcas_pull_fades_out_when_inverted() {
    std::cout << "[Test] GCAS: Commanded pull is held at moderate bank and faded out when inverted...\n";
    double prev = 1e9;
    double pull_at_zero_bank = 0.0;
    const double nominal_stick = (OnBoardFlightComputer::GCAS_RECOVERY_G - 1.0) / (9.0 - 1.0);
    for (double bank_deg : {0.0, 45.0, 90.0, 135.0, 150.0, 165.0, 179.0}) {
        OnBoardFlightComputer ofc;
        fdm::FlightState state{};
        state.pos_ned = math::Vector3(0, 0, -70.0);
        state.vel_b   = math::Vector3(220.0, 0.0, 0.0);
        state.q_att = math::Quaternion::from_euler(bank_deg * (M_PI / 180.0),
                                                   -15.0 * (M_PI / 180.0), 0.0);
        IMUData imu{};
        imu.Nz = 1.0;
        PilotCommands pilot_in{0.0, 0.0, 0.0};

        PilotCommands out{};
        assert(fly_until_gcas(ofc, state, imu, pilot_in, out, 1.0));
        // Never increases with bank once the roll to upright is the bigger job
        // (below that a later trigger may escalate the pull).
        if (bank_deg >= 90.0) assert(out.pitch_stick <= prev + 1e-9);
        if (bank_deg == 0.0) pull_at_zero_bank = out.pitch_stick;
        // Full recovery G is preserved out to where the pre-load is still worth more
        // than the downward component it costs (see bank_pull_scale).
        if (bank_deg <= 135.0) assert(out.pitch_stick >= nominal_stick - 1e-9);
        // Beyond the fade the pull must be gone: it would drive the nose at the ground.
        if (bank_deg >= 165.0) assert(out.pitch_stick <= 0.0);
        std::cout << "    bank=" << bank_deg << " deg -> pitch_stick=" << out.pitch_stick << "\n";
        prev = out.pitch_stick;
    }
    assert(pull_at_zero_bank > 0.4);
    std::cout << "  PASSED (full pull held to 135 deg, faded to zero by 165 deg)\n";
}

void test_gcas_roll_command_stable_across_180_wrap() {
    std::cout << "[Test] GCAS: Roll command does not dither across the +/-180 deg Euler wrap...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -90.0);
    state.vel_b   = math::Vector3(220.0, 0.0, 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    // The jet hangs either side of inverted, as it does under turbulence. A naive
    // -roll law saturates to -1 at +179 deg and +1 at -179 deg, so the stick slams
    // full-scale left/right every tick and the jet never actually rolls upright.
    double prev = 0.0;
    for (int i = 0; i < 20; ++i) {
        const double roll_deg = (i % 2 == 0) ? 179.4 : 180.6; // 180.6 wraps to -179.4
        state.q_att = math::Quaternion::from_euler(roll_deg * (M_PI / 180.0),
                                                   -12.0 * (M_PI / 180.0), 0.0);
        PilotCommands out = ofc.update(0.005, state, imu, pilot_in);
        assert(ofc.is_gcas_active());
        if (i > 0) {
            // No full-scale sign reversal between consecutive ticks.
            assert(!(prev * out.roll_stick < 0.0 && std::abs(out.roll_stick) > 0.9));
        }
        prev = out.roll_stick;
    }
    std::cout << "  PASSED (roll direction latched through the wrap, no full-scale reversals)\n";
}

void test_tumble_never_commands_nose_down_during_gcas_recovery() {
    std::cout << "[Test] Tumble: Never commands nose-down while GCAS is recovering near terrain...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};
    PilotCommands out{};
    // A low dive that GCAS is pulling out of...
    state.pos_ned = math::Vector3(0, 0, -60.0);
    state.vel_b   = math::Vector3(200.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::from_euler(0.0, -15.0 * (M_PI / 180.0), 0.0);
    assert(fly_until_gcas(ofc, state, imu, pilot_in, out, 2.0));

    // ...reaches high alpha (35 deg) at 130 m/s, 34 m AGL: inside the deep-stall
    // detector's band, and exactly what a hard GCAS pull-out produces at low speed.
    state.pos_ned = math::Vector3(state.pos_ned.x, 0, -34.0);
    const double V = 130.0, a = 35.0 * (M_PI / 180.0);
    state.vel_b   = math::Vector3(V * std::cos(a), 0.0, V * std::sin(a));
    state.q_att   = math::Quaternion::from_euler(0.0, 30.0 * (M_PI / 180.0), 0.0);
    state.omega_b = math::Vector3(0.0, 0.1, 0.0);
    imu.Nz = 1.5;
    for (int i = 0; i < 80; ++i) out = ofc.update(0.005, state, imu, pilot_in);

    assert(ofc.is_gcas_active());
    // The tumble law used to win the pitch axis outside a narrow 2.0 s
    // time-to-impact window and command full nose-down at 34 m AGL.
    assert(out.pitch_stick > 0.0);
    std::cout << "  GCAS keeps the pitch axis: pitch_stick=" << out.pitch_stick
              << ", tti=" << ofc.gcas_tti_seconds() << "s\n";

    // The same state with no recovery in progress (the pilot's own hard pull,
    // which GCAS predicts will clear): no nose-down stall break at 34 m.
    OnBoardFlightComputer fresh;
    for (int i = 0; i < 80; ++i) out = fresh.update(0.005, state, imu, pilot_in);
    assert(!fresh.is_tumble_active());
    assert(out.pitch_stick >= 0.0);
    std::cout << "  PASSED (no stall break near the ground: pitch_stick=" << out.pitch_stick << ")\n";
}

void test_gcas_stands_down_for_established_departure() {
    std::cout << "[Test] GCAS: Stands down when the airframe has already departed...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -3000.0); // high up: no terrain threat yet
    state.vel_b   = math::Vector3(75.0, 25.0, 60.0);
    state.omega_b = math::Vector3(0.5, 0.4, 1.2); // violent flat spin

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    for (int i = 0; i < 60; ++i) (void)ofc.update(0.005, state, imu, pilot_in);
    assert(ofc.is_tumble_active());

    // Now the spinning jet arrives at low altitude. GCAS must not engage on top of
    // the departure: its pull-out margin assumes a 5 G pull is achievable, which is
    // false in a spin. Breaking the departure comes first.
    state.pos_ned = math::Vector3(0, 0, -60.0);
    PilotCommands out = ofc.update(0.005, state, imu, pilot_in);
    assert(ofc.is_tumble_active());
    assert(!ofc.is_gcas_active());
    assert(out.pitch_stick < -0.8); // deep-stall breakout still owns the pitch axis
    std::cout << "  PASSED (departure recovery retains authority, pitch_stick="
              << out.pitch_stick << ")\n";
}

void test_gcas_does_not_latch_when_airspeed_collapses() {
    std::cout << "[Test] GCAS: Disarms instead of latching 'active' when airspeed collapses...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -100.0);
    state.vel_b   = math::Vector3(220.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::from_euler(0.0, -30.0 * (M_PI / 180.0), 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands pilot_in{0.0, 0.0, 0.0};

    (void)ofc.update(0.005, state, imu, pilot_in);
    assert(ofc.is_gcas_active());

    // Airspeed falls below the 10 m/s guard. Bailing out early used to leave the
    // state machine latched at PULL_UP forever: is_gcas_active() reported a
    // recovery that was issuing no commands and no throttle override at all.
    state.vel_b = math::Vector3(5.0, 0.0, 0.0);
    (void)ofc.update(0.005, state, imu, pilot_in);
    assert(!ofc.is_gcas_active());
    assert(!ofc.has_throttle_override());
    assert(ofc.gcas_state == OnBoardFlightComputer::GcasState::ARMED);
    std::cout << "  PASSED (state disarmed cleanly, no phantom recovery)\n";
}

void test_gcas_ignores_its_own_commands_as_pilot_input() {
    std::cout << "[Test] GCAS: Does not mistake the GLOC autopilot's stick for a conscious pilot...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -2000.0);
    state.vel_b   = math::Vector3(250.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::identity();

    IMUData imu{};
    imu.Nz = 10.0;
    PilotCommands neutral{0.0, 0.0, 0.0};

    while (ofc.gloc_state != OnBoardFlightComputer::GlocState::GLOC) {
        (void)ofc.update(0.01, state, imu, neutral);
    }

    // Unconscious pilot, hands off the stick, jet rolled into a steep banked dive
    // toward terrain. The GLOC autopilot commands a hard wings-level roll; reading
    // that command back as pilot input used to trip GCAS's "pilot breakout" and
    // hand a diving jet to an unconscious pilot.
    imu.Nz = 1.2;
    state.q_att   = math::Quaternion::from_euler(70.0 * (M_PI / 180.0),
                                                 -25.0 * (M_PI / 180.0), 0.0);
    state.pos_ned = math::Vector3(0, 0, -150.0);

    (void)ofc.update(0.005, state, imu, neutral);
    assert(ofc.is_gloc_active());   // pilot did not "wake up"
    assert(ofc.is_gcas_active());   // GCAS did not hand back control
    std::cout << "  PASSED (GLOC held, GCAS retained authority over the unconscious pilot)\n";
}

void test_gloc_duration_respects_minimum_everywhere() {
    std::cout << "[Test] GLOC: Blackout duration stays within bounds at negative map coordinates...\n";
    for (double x : {0.0, 5000.0, -5000.0, -12345.0, -987654.0}) {
        OnBoardFlightComputer ofc;
        fdm::FlightState state{};
        state.pos_ned = math::Vector3(x, 0.0, -2000.0);
        state.vel_b   = math::Vector3(250.0, 0.0, 0.0);
        IMUData imu{};
        imu.Nz = 10.0;
        PilotCommands neutral{0.0, 0.0, 0.0};
        while (ofc.gloc_state != OnBoardFlightComputer::GlocState::GLOC) {
            (void)ofc.update(0.01, state, imu, neutral);
        }
        // std::fmod keeps the sign of its dividend, so west of the origin the raw
        // seed was negative and shortened the blackout below the documented minimum.
        assert(ofc.gloc_duration >= OnBoardFlightComputer::GLOC_MIN_DURATION_S);
        assert(ofc.gloc_duration <= OnBoardFlightComputer::GLOC_MAX_DURATION_S);
    }
    std::cout << "  PASSED (duration within [" << OnBoardFlightComputer::GLOC_MIN_DURATION_S
              << ", " << OnBoardFlightComputer::GLOC_MAX_DURATION_S << "] s at all positions)\n";
}

void test_gloc_authority_ramp_actually_ramps() {
    std::cout << "[Test] GLOC: Authority is ramped back in, not snapped, after regaining consciousness...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -3000.0);
    state.vel_b   = math::Vector3(250.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::identity();

    IMUData imu{};
    imu.Nz = 10.0;
    PilotCommands neutral{0.0, 0.0, 0.0};

    while (ofc.gloc_state != OnBoardFlightComputer::GlocState::GLOC) {
        (void)ofc.update(0.01, state, imu, neutral);
    }

    // G comes off; ride out the unconscious period with the stick untouched.
    imu.Nz = 1.0;
    while (ofc.gloc_state == OnBoardFlightComputer::GlocState::GLOC) {
        (void)ofc.update(0.01, state, imu, neutral);
    }
    assert(ofc.gloc_state == OnBoardFlightComputer::GlocState::RECOVERY);

    // g_exposure is clamped to 2.0 on entry and decays at 5.0/s, so an exit test of
    // `timer >= ramp || g_exposure < 1.5` ended RECOVERY after ~0.1 s and skipped the
    // ramp entirely. Half a ramp in, authority must still be partial.
    int ticks = 0;
    while (ofc.gloc_state == OnBoardFlightComputer::GlocState::RECOVERY && ticks < 1000) {
        (void)ofc.update(0.01, state, imu, neutral);
        ++ticks;
        if (ticks == 50) { // 0.5 s into a 1.5 s ramp
            assert(ofc.gloc_authority > 0.0 && ofc.gloc_authority < 1.0);
        }
    }
    const double ramp_s = ticks * 0.01;
    assert(ramp_s > OnBoardFlightComputer::GLOC_AUTHORITY_RAMP_S * 0.8);
    std::cout << "  PASSED (recovery ramp took " << ramp_s << " s, authority restored gradually)\n";
}

// ===========================================================================
// Regressions: reacting too late, too early, or to the wrong thing.
// ===========================================================================

void test_gloc_held_stick_is_not_a_wakeup() {
    std::cout << "[Test] GLOC: A stick held through the blackout does not count as waking up...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -3000.0);
    state.vel_b   = math::Vector3(250.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::identity();

    IMUData imu{};
    imu.Nz = 10.0;
    const PilotCommands full_aft{1.0, 0.0, 0.0};
    while (ofc.gloc_state != OnBoardFlightComputer::GlocState::GLOC) {
        (void)ofc.update(0.01, state, imu, full_aft);
    }

    // The autopilot unloads the jet, but the (unconscious) pilot's hand is still
    // where it was.  Reading that as a deliberate input handed a 9 G pull straight
    // back within a fraction of a second, over and over.
    imu.Nz = 1.2;
    for (int i = 0; i < 20; ++i) (void)ofc.update(0.01, state, imu, full_aft);
    assert(ofc.gloc_state == OnBoardFlightComputer::GlocState::GLOC);

    // Releasing and then re-applying the stick is a deliberate, conscious act.
    (void)ofc.update(0.01, state, imu, PilotCommands{});
    (void)ofc.update(0.01, state, imu, PilotCommands{0.35, 0.0, 0.0});
    assert(ofc.gloc_state == OnBoardFlightComputer::GlocState::NORMAL);
    std::cout << "  PASSED (held stick ignored; release-and-reapply restored authority)\n";
}

void test_gcas_roll_twitch_does_not_hand_back_a_steep_dive() {
    std::cout << "[Test] GCAS: A roll-stick twitch mid-pull does not hand back a steep dive...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -150.0);
    state.vel_b   = math::Vector3(220.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::from_euler(0.0, -30.0 * (M_PI / 180.0), 0.0);

    IMUData imu{};
    imu.Nz = 1.0;
    (void)ofc.update(0.005, state, imu, PilotCommands{});
    assert(ofc.gcas_state == OnBoardFlightComputer::GcasState::PULL_UP);

    // Still 30 deg nose-down, sinking 110 m/s.  The breakout test compared the
    // NED sink rate against `> -15`, which every descent satisfies.
    (void)ofc.update(0.005, state, imu, PilotCommands{0.0, 0.5, 0.0});
    assert(ofc.is_gcas_active());
    std::cout << "  PASSED (GCAS kept the recovery)\n";
}

void test_gcas_leaves_gear_down_approach_alone() {
    std::cout << "[Test] GCAS: Gear-down approaches are left alone; a gear-down dive is not...\n";
    struct Case { double agl, speed, gamma_deg; bool expect_active; };
    // 3 deg at 30 m sat inside the old 35 m floor above the 15 m flare lockout;
    // a steep-ish 6 deg approach sinks faster than the old 6 m/s limit.
    constexpr Case cases[] = {{30.0, 80.0, -3.0, false},
                              {25.0, 85.0, -6.0, false},
                              {60.0, 100.0, -20.0, true}};
    for (const Case& c : cases) {
        OnBoardFlightComputer ofc;
        fdm::FlightState state{};
        state.pos_ned = math::Vector3(0, 0, -c.agl);
        state.vel_b   = math::Vector3(c.speed, 0.0, 0.0);
        state.q_att   = math::Quaternion::from_euler(0.0, c.gamma_deg * (M_PI / 180.0), 0.0);
        IMUData imu{};
        imu.Nz = 1.0;
        (void)ofc.update(0.005, state, imu, PilotCommands{}, /*gear_down=*/true);
        assert(ofc.is_gcas_active() == c.expect_active);
    }
    std::cout << "  PASSED (3 deg and 6 deg approaches untouched, 20 deg dive recovered)\n";
}

void test_gcas_engages_earlier_when_slow() {
    std::cout << "[Test] GCAS: A slow dive is planned on the G actually available...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    // 100 m/s, 20 deg dive at 180 m.  At this speed the F-16 has ~1.9 G of lift
    // in hand, not 5; a margin computed at 5 G said 52 m was plenty.
    state.pos_ned = math::Vector3(0, 0, -180.0);
    state.vel_b   = math::Vector3(100.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::from_euler(0.0, -20.0 * (M_PI / 180.0), 0.0);
    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands out{};
    assert(fly_until_gcas(ofc, state, imu, PilotCommands{}, out));
    // ...so it engages well above that: the ~1.9 G pull-out needs ~100 m.
    std::cout << "    engaged at " << state.altitude() << " m\n";
    assert(state.altitude() > 100.0);
    std::cout << "  PASSED\n";
}

void test_gcas_token_pull_does_not_stand_gcas_down() {
    std::cout << "[Test] GCAS: A token 2.5 G tug does not stand GCAS down when 5 G is needed...\n";
    OnBoardFlightComputer ofc;
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -480.0);
    state.vel_b   = math::Vector3(250.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::from_euler(0.0, -35.0 * (M_PI / 180.0), 0.0);
    IMUData imu{};
    imu.Nz = 2.5;
    PilotCommands out{};
    assert(fly_until_gcas(ofc, state, imu, PilotCommands{0.3, 0.0, 0.0}, out));
    assert(ofc.is_gcas_active());
    std::cout << "  PASSED\n";
}

void test_gcas_recovery_g_mapped_through_airframe_range() {
    std::cout << "[Test] OFC: Adopts the configured airframe's limits (not the F-16's)...\n";
    OnBoardFlightComputer ofc(aircraft::AircraftType::A10_THUNDERBOLT);
    assert(ofc.airframe.max_g == 7.33);
    assert(!ofc.airframe.g_command_law);      // direct linkage: G closed on Nz
    assert(ofc.airframe.alpha_limit_deg == 18.0);
    ofc.configure(aircraft::AircraftType::F16_FIGHTING_FALCON);
    assert(ofc.airframe.max_g == 9.0 && ofc.airframe.g_command_law);

    // And a 5 G recovery maps through that airframe's own stick range.
    fdm::FlightState state{};
    state.pos_ned = math::Vector3(0, 0, -480.0);
    state.vel_b   = math::Vector3(250.0, 0.0, 0.0);
    state.q_att   = math::Quaternion::from_euler(0.0, -35.0 * (M_PI / 180.0), 0.0);
    IMUData imu{};
    imu.Nz = 1.0;
    PilotCommands out{};
    assert(fly_until_gcas(ofc, state, imu, PilotCommands{}, out));
    assert(ofc.is_gcas_active());
    assert(std::abs(out.pitch_stick - (5.0 - 1.0) / (9.0 - 1.0)) < 0.02);
    std::cout << "  PASSED\n";
}

void test_tumble_departure_alpha_scales_with_airframe() {
    std::cout << "[Test] Tumble: 34 deg alpha is a departure for the F-16, not for the Typhoon...\n";
    auto run = [](aircraft::AircraftType type) {
        OnBoardFlightComputer ofc(type);
        fdm::FlightState state{};
        state.pos_ned = math::Vector3(0, 0, -3000.0);
        const double a = 34.0 * (M_PI / 180.0);
        state.vel_b = math::Vector3(100.0 * std::cos(a), 0.0, 100.0 * std::sin(a));
        state.q_att = math::Quaternion::from_euler(0.0, a, 0.0); // level flight path
        IMUData imu{};
        imu.Nz = 1.0;
        for (int i = 0; i < 100; ++i) (void)ofc.update(0.005, state, imu, PilotCommands{1.0, 0.0, 0.0});
        return ofc.is_tumble_active();
    };
    assert(run(aircraft::AircraftType::F16_FIGHTING_FALCON));   // past its 25 deg limit
    assert(!run(aircraft::AircraftType::EUROFIGHTER_TYPHOON));  // inside its 35 deg envelope
    std::cout << "  PASSED\n";
}

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  OFC Flight Computer Control Handover Verification      \n";
    std::cout << "=========================================================\n";
    test_gcas_does_not_intervene_on_shallow_dive();
    test_gcas_immediate_handover_when_level();
    test_gcas_pilot_breakout_override();
    test_gcas_engages_on_shallow_but_low_dive();
    test_gcas_commands_nominal_g_not_max_g();
    test_gcas_escalates_when_nominal_arc_no_longer_fits();
    test_gcas_hard_deck_engages_without_dive();
    test_gcas_does_not_disturb_normal_landing();
    test_gcas_throttle_authority();
    test_gcas_avoids_mountain_ahead_in_level_flight();
    test_gcas_elevated_terrain_dive();
    test_gloc_prompt_pilot_handover();
    test_tumble_spin_auto_recovery();
    test_tumble_does_not_intervene_on_fast_roll();

    std::cout << "\n--- Loss-of-control regressions (complex attitudes) ---\n";
    test_gcas_never_pulls_while_inverted();
    test_gcas_pull_fades_out_when_inverted();
    test_gcas_roll_command_stable_across_180_wrap();
    test_tumble_never_commands_nose_down_during_gcas_recovery();
    test_gcas_stands_down_for_established_departure();
    test_gcas_does_not_latch_when_airspeed_collapses();
    test_gcas_ignores_its_own_commands_as_pilot_input();
    test_gloc_duration_respects_minimum_everywhere();
    test_gloc_authority_ramp_actually_ramps();

    std::cout << "\n--- Reaction regressions (too late / too early / wrong trigger) ---\n";
    test_gloc_held_stick_is_not_a_wakeup();
    test_gcas_roll_twitch_does_not_hand_back_a_steep_dive();
    test_gcas_leaves_gear_down_approach_alone();
    test_gcas_engages_earlier_when_slow();
    test_gcas_token_pull_does_not_stand_gcas_down();
    test_gcas_recovery_g_mapped_through_airframe_range();
    test_tumble_departure_alpha_scales_with_airframe();

    std::cout << "\nAll OFC Control Handover & Tumble tests PASSED successfully!\n";
    return 0;
}
