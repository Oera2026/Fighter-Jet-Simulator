/**
 * @file test_flcs_handling.cpp
 * @brief Handling qualities of every jet's flight control laws, flown through
 * the full sim::Aircraft pipeline in calm air, from the findings of a control
 * law review:
 *
 *  - Hands off after a full aft pull at low speed, the Typhoon hung at 34 deg
 *    alpha, the F-22 at 60 and the F-35B at 50 for 8 s or more: a G command
 *    with nothing to say how much alpha to spend on 1 G.
 *  - At low dynamic pressure the generic G law was weaker than the Typhoon's
 *    and F-22's instability: a 1 G step took 2 s and settled short.
 *  - The roll laws rolled about the fuselage, so at high alpha every roll
 *    swapped alpha for sideslip (23-37 deg of it on the Typhoon, F-22 and
 *    F-35B at 30-50 deg alpha), and stopping a full-rate roll overran by
 *    100 deg.
 *  - At approach speed the F-15EX and Typhoon pitch laws (blended pitch-rate
 *    and G commands that disagreed) let a pilot tracking pitch attitude set up
 *    a pilot-induced oscillation.
 */
#include "fastjet/sim/aircraft.hpp"
#include <cassert>
#include <cmath>
#include <complex>
#include <iostream>
#include <memory>
#include <vector>

using namespace fastjet;
using aircraft::AircraftType;

namespace {

constexpr double DT = 0.005;
constexpr double G0 = 9.80665;
constexpr double D2R = M_PI / 180.0;
constexpr double R2D = 180.0 / M_PI;

constexpr AircraftType FBW[] = {AircraftType::F16_FIGHTING_FALCON, AircraftType::F15EX_EAGLE_II,
                                AircraftType::EUROFIGHTER_TYPHOON, AircraftType::F22_RAPTOR,
                                AircraftType::F35B_LIGHTNING_II};

struct Cond { double alt, V; bool gear; };

/// True airspeed at which the jet (half fuel, combat payload) flies 1 G at @p alpha_deg.
double speed_for_alpha(AircraftType t, double alt, double alpha_deg) {
    sim::Aircraft a(t);
    a.engine.fuel_kg *= 0.5;
    a.refresh_mass();
    const aero::AircraftAeroModel m(t);
    fdm::FlightState s{};
    s.vel_b = {100.0 * std::cos(alpha_deg * D2R), 0.0, 100.0 * std::sin(alpha_deg * D2R)};
    aero::ControlSurfaces c{};
    if (t == AircraftType::F16_FIGHTING_FALCON) c.delta_lef = flcs::F16FLCS::compute_lef_schedule(alpha_deg, 4000.0);
    const auto k = m.compute_coefficients(s, c, 4000.0, 0.25);
    const double cn = k.CL * std::cos(alpha_deg * D2R) + k.CD * std::sin(alpha_deg * D2R);
    const double rho = environment::Atmosphere1976::compute(alt, 100.0).density;
    return std::sqrt(2.0 * a.mass.mass_kg * G0 / (rho * aircraft::AircraftConfig::get(t).aero.s_ref * cn));
}

Cond slow(AircraftType t) { return {6000.0, speed_for_alpha(t, 6000.0, 8.0), false}; }
Cond approach(AircraftType t) { return {600.0, speed_for_alpha(t, 600.0, 12.0), true}; }
Cond corner() { return {3000.0, 190.0, false}; }

/// A jet in calm air, trimmed hands-off at 1 G, speed held on the throttle.
struct Rig {
    std::unique_ptr<sim::Aircraft> jet;
    aircraft::AircraftConfig cfg;
    double V_ref, lever{0.7}, thr_int{0.0};
    bool hold_speed{true};

    Rig(AircraftType t, const Cond& c) : jet(std::make_unique<sim::Aircraft>(t)), cfg(aircraft::AircraftConfig::get(t)), V_ref(c.V) {
        fdm::FlightState s{};
        s.pos_ned = {0.0, 0.0, -c.alt};
        const auto air = environment::Atmosphere1976::compute(c.alt, c.V);
        const double a0 = std::clamp(jet->mass.mass_kg * G0 / (air.dynamic_pressure * cfg.aero.s_ref) / cfg.aero.cl_alpha, 0.0, 0.25);
        s.vel_b = {c.V * std::cos(a0), 0.0, c.V * std::sin(a0)};
        s.q_att = math::Quaternion::from_euler(0.0, a0, 0.0);
        jet->reset(s);
        jet->wind.base_wind_speed_mps = 0.0;
        jet->landing_gear.deployed = c.gear;
        jet->engine.fuel_kg *= 0.5;
        jet->refresh_mass();
        for (int i = 0; i < 3000; ++i) (void)jet->engine.update(lever, air, DT);
        for (int i = 0; i < static_cast<int>(7.0 / DT); ++i) step(0.0, wings());
    }
    double alpha() const { return jet->state.alpha() * R2D; }
    double beta() const { return jet->state.beta() * R2D; }
    double phi() const { return jet->state.roll() * R2D; }
    double wings() const { return std::clamp(-0.05 * phi() - 0.01 * jet->imu.p_deg, -0.3, 0.3); }
    void step(double pitch, double roll = 0.0) {
        if (hold_speed) {
            const double e = V_ref - jet->state.airspeed();
            thr_int = std::clamp(thr_int + 0.02 * e * DT, -1.0, 1.0);
            lever = std::clamp(0.6 + 0.05 * e + thr_int, 0.0, 1.0);
        }
        sim::AircraftControls c{};
        c.throttle = lever;
        c.stick.pitch_stick = pitch;
        c.stick.roll_stick = roll;
        jet->step(DT, c);
    }
    bool departed() const { return jet->ofc.is_tumble_active(); }
};

double stick_for_g(AircraftType t, double dnz) {
    return dnz / (aircraft::AircraftConfig::get(t).flcs.max_g_positive - 1.0);
}

struct Step { double final_g, rise_s, overshoot, settle_s, release_min; };

/// Hold a G-command stick step 4 s, then release it 4 s.
Step pitch_step(AircraftType t, const Cond& c, double dnz) {
    Rig rig(t, c);
    const double nz0 = rig.jet->imu.Nz;
    std::vector<double> nz;
    for (int i = 0; i < 800; ++i) { rig.step(stick_for_g(t, dnz)); nz.push_back(rig.jet->imu.Nz); }
    Step s{};
    for (int i = 600; i < 800; ++i) s.final_g += nz[i] / 200.0;
    const double d = s.final_g - nz0;
    double t10 = -1, t90 = -1, peak = -9;
    for (size_t i = 0; i < nz.size(); ++i) {
        if (t10 < 0 && (nz[i] - nz0) >= 0.1 * d) t10 = i * DT;
        if (t90 < 0 && (nz[i] - nz0) >= 0.9 * d) t90 = i * DT;
        peak = std::max(peak, nz[i]);
        if (std::abs(nz[i] - s.final_g) > std::max(0.05 * d, 0.1)) s.settle_s = (i + 1) * DT;
    }
    s.rise_s = t90 - t10;
    s.overshoot = (peak - s.final_g) / d;
    s.release_min = 99.0;
    for (int i = 0; i < 800; ++i) { rig.step(0.0); s.release_min = std::min(s.release_min, rig.jet->imu.Nz); }
    return s;
}

struct Roll { double p_ss, overshoot, beta_max, travel_after; bool departed; };  // p_ss: flight-path roll rate [deg/s]

/// Full lateral stick for @p hold_s, then centred; optionally holding alpha with the stick.
Roll roll(AircraftType t, const Cond& c, double hold_s, double alpha_hold = -1.0) {
    Rig rig(t, c);
    double ai = 0.0;
    auto pitch = [&] {
        if (alpha_hold < 0.0) return 0.0;
        const double e = alpha_hold - rig.alpha();
        ai = std::clamp(ai + 0.08 * e * DT, -1.0, 1.0);
        return std::clamp(0.04 * e + ai - 0.01 * rig.jet->imu.q_deg, -1.0, 1.0);
    };
    if (alpha_hold > 0.0) {
        rig.hold_speed = false;
        rig.lever = 1.0;
        for (int i = 0; i < 600; ++i) rig.step(pitch(), rig.wings());
    }
    Roll r{};
    std::vector<double> p;
    const int n = static_cast<int>(hold_s / DT);
    for (int i = 0; i < n; ++i) {
        rig.step(pitch(), 1.0);
        // Roll rate about the flight path: at high alpha most of it is yaw.
        const double a = rig.jet->state.alpha();
        p.push_back(rig.jet->imu.p_deg * std::cos(a) + rig.jet->imu.r_deg * std::sin(a));
        r.beta_max = std::max(r.beta_max, std::abs(rig.beta()));
        r.departed = r.departed || rig.departed();
    }
    double peak = 0.0;
    for (int i = n / 2; i < n; ++i) r.p_ss += p[static_cast<size_t>(i)] / (n - n / 2);
    for (double x : p) peak = std::max(peak, x);
    r.overshoot = (peak - r.p_ss) / std::max(1.0, r.p_ss);
    double prev = rig.phi();
    for (int i = 0; i < 400; ++i) {
        rig.step(pitch(), 0.0);
        double d = rig.phi() - prev;
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        r.travel_after += d;
        prev = rig.phi();
        r.beta_max = std::max(r.beta_max, std::abs(rig.beta()));
        r.departed = r.departed || rig.departed();
    }
    return r;
}

/// |pitch attitude / stick| at @p w [rad/s]: a small stick sinusoid, the
/// attitude's linear drift removed.
double attitude_gain(AircraftType t, const Cond& c, double w) {
    Rig rig(t, c);
    const double A = stick_for_g(t, 0.25);
    const double T = 2.0 * M_PI / w;
    const int n_settle = static_cast<int>(2.0 * T / DT), n_meas = static_cast<int>(std::round(3.0 * T / DT));
    std::vector<double> th, tm;
    std::complex<double> S{0.0, 0.0};
    for (int i = 0; i < n_settle + n_meas; ++i) {
        const double u = A * std::sin(w * i * DT);
        rig.step(u, rig.wings());
        if (i >= n_settle) {
            th.push_back(rig.jet->state.pitch() * R2D);
            tm.push_back((i + 1) * DT);
            S += u * std::exp(std::complex<double>(0.0, -w * i * DT));
        }
    }
    const double n = static_cast<double>(th.size());
    double st = 0, sy = 0, stt = 0, sty = 0;
    for (size_t i = 0; i < th.size(); ++i) { st += tm[i]; sy += th[i]; stt += tm[i] * tm[i]; sty += tm[i] * th[i]; }
    const double b = (n * sty - st * sy) / (n * stt - st * st), a = (sy - b * st) / n;
    std::complex<double> Th{0.0, 0.0};
    for (size_t i = 0; i < th.size(); ++i) Th += (th[i] - (a + b * tm[i])) * std::exp(std::complex<double>(0.0, -w * tm[i]));
    return std::abs(Th / S);
}

/// Pitch-attitude tracking of a sum of sines by a pilot with a 0.2 s delay and
/// a 0.1 s lag, gain set for a crossover at @p wc [rad/s]: RMS error over RMS
/// command. Above 1 the pilot does worse than leaving the stick alone (a PIO).
double tracking_error(AircraftType t, const Cond& c, double wc) {
    const double kp = std::sqrt(1.0 + 0.01 * wc * wc) / attitude_gain(t, c, wc);
    Rig rig(t, c);
    const int delay = static_cast<int>(0.20 / DT);
    std::vector<double> buf(static_cast<size_t>(delay + 1), 0.0);
    const double th0 = rig.jet->state.pitch() * R2D;
    const double amp[] = {2.0, 1.5, 1.0, 0.6, 0.3}, w[] = {0.35, 0.8, 1.5, 2.7, 4.3}, ph[] = {0.0, 1.3, 2.1, 0.7, 4.0};
    double u = 0.0, e2 = 0.0, c2 = 0.0;
    for (int i = 0; i < static_cast<int>(30.0 / DT); ++i) {
        double cmd = 0.0;
        for (int k = 0; k < 5; ++k) cmd += amp[k] * std::sin(w[k] * i * DT + ph[k]);
        const double e = cmd - (rig.jet->state.pitch() * R2D - th0);
        buf[static_cast<size_t>(i % (delay + 1))] = e;
        u += (kp * buf[static_cast<size_t>((i + 1) % (delay + 1))] - u) * (1.0 - std::exp(-DT / 0.1));
        rig.step(u, rig.wings());
        if (i * DT > 5.0) { e2 += e * e; c2 += cmd * cmd; }
    }
    return std::sqrt(e2 / c2);
}

// ---------------------------------------------------------------------------

void test_pitch_steps() {
    std::cout << "[HANDLING] G steps: quick, no overshoot, no sag on release; low q included...\n";
    for (AircraftType t : FBW) {
        const Step hi = pitch_step(t, corner(), 2.0);
        const Step lo = pitch_step(t, slow(t), 1.0);
        std::cout << "    " << aircraft::to_short_string(t) << ": 3 km 190 m/s +2 G -> " << hi.final_g << " G, rise " << hi.rise_s
                  << " s, overshoot " << hi.overshoot * 100.0 << "%, settled " << hi.settle_s << " s, release min "
                  << hi.release_min << " G; 6 km slow +1 G -> " << lo.final_g << " G, rise " << lo.rise_s << " s\n";
        assert(std::abs(hi.final_g - 3.0) < 0.05);
        assert(hi.rise_s < 0.45 && hi.overshoot < 0.08 && hi.settle_s < 1.0);
        assert(hi.release_min > 0.85);
        // At low q the F-16 still blends in its pitch-rate command.
        const double short_of = t == AircraftType::F16_FIGHTING_FALCON ? 0.30 : 0.12;
        assert(lo.final_g > 2.0 - short_of && lo.final_g < 2.1);
        assert(lo.rise_s < 1.0);
    }
    std::cout << "  -> PASSED\n";
}

void test_no_hang_at_high_alpha() {
    std::cout << "[HANDLING] Hands off after a full aft pull at low speed, alpha comes back...\n";
    for (AircraftType t : FBW) {
        Rig rig(t, slow(t));
        const auto& f = rig.cfg.flcs;
        double alpha_max = -90.0;
        for (int i = 0; i < 1000; ++i) { rig.step(1.0); alpha_max = std::max(alpha_max, rig.alpha()); }
        double hang = 0.0, alpha_4s = 0.0;
        for (int i = 0; i < 1600; ++i) {
            rig.step(0.0);
            if (rig.alpha() > f.alpha_limit_deg - 5.0) hang = (i + 1) * DT;
            if (i == 800) alpha_4s = rig.alpha();
            assert(!rig.departed());
        }
        const double hands_off = t == AircraftType::F16_FIGHTING_FALCON ? flcs::PitchController::AOA_LIMITER_ONSET_DEG
                                                                          : f.pitch.alpha_hands_off_deg;
        std::cout << "    " << aircraft::to_short_string(t) << ": full aft " << alpha_max << " deg (limit " << f.alpha_limit_deg
                  << "), above limit-5 for " << hang << " s after release, " << alpha_4s << " deg 4 s later\n";
        assert(alpha_max > f.alpha_limit_deg - 3.0 && alpha_max < f.alpha_limit_deg + 3.0);
        assert(hang < 2.0);
        assert(alpha_4s < hands_off + 10.0);
    }
    std::cout << "  -> PASSED\n";
}

void test_rolls_are_coordinated_and_stop() {
    std::cout << "[HANDLING] Full-stick rolls: rated rate, little sideslip, a crisp stop...\n";
    for (AircraftType t : FBW) {
        const double rated = aircraft::AircraftConfig::get(t).flcs.max_roll_rate_dps;
        const Roll fast = roll(t, corner(), 1.5);
        const Roll low = roll(t, slow(t), 1.5);
        const Roll app = roll(t, approach(t), 1.0);
        std::cout << "    " << aircraft::to_short_string(t) << ": 190 m/s " << fast.p_ss << " deg/s (rated " << rated
                  << "), overshoot " << fast.overshoot * 100.0 << "%, beta " << fast.beta_max << ", stops in "
                  << fast.travel_after << " deg; slow beta " << low.beta_max << ", approach beta " << app.beta_max << "\n";
        assert(fast.p_ss > 0.95 * rated && fast.overshoot < 0.12);
        assert(fast.beta_max < 3.0 && fast.travel_after < 90.0);
        assert(low.beta_max < 6.0 && app.beta_max < 6.0);
        assert(!fast.departed && !low.departed && !app.departed);
    }
    std::cout << "  -> PASSED\n";
}

void test_high_alpha_rolls_do_not_depart() {
    std::cout << "[HANDLING] Full-stick rolls at high alpha roll about the flight path...\n";
    for (AircraftType t : FBW) {
        const double lim = aircraft::AircraftConfig::get(t).flcs.alpha_limit_deg;
        for (double a : {std::min(lim - 3.0, 0.9 * lim), std::min(lim - 5.0, 35.0)}) {
            const Roll r = roll(t, slow(t), 2.0, a);
            std::cout << "    " << aircraft::to_short_string(t) << " at " << a << " deg: " << r.p_ss << " deg/s, beta max "
                      << r.beta_max << " deg, " << r.travel_after << " deg after release\n";
            assert(r.beta_max < 7.0);
            assert(r.p_ss > 8.0 && r.travel_after < 60.0);
            assert(!r.departed);
        }
    }
    std::cout << "  -> PASSED\n";
}

void test_no_pio_in_tracking() {
    std::cout << "[HANDLING] Pitch-attitude tracking: no pilot-induced oscillation...\n";
    for (AircraftType t : FBW) {
        const double app = tracking_error(t, approach(t), 2.0);
        const double cr = tracking_error(t, corner(), 2.0);
        const double cr_hi = tracking_error(t, corner(), 4.0);
        std::cout << "    " << aircraft::to_short_string(t) << ": error/command approach " << app << ", 190 m/s " << cr
                  << ", aggressive (4 rad/s crossover) " << cr_hi << "\n";
        assert(app < 1.0 && cr < 0.7 && cr_hi < 0.9);
    }
    std::cout << "  -> PASSED\n";
}

} // namespace

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Flight Control Law Handling Qualities                  \n";
    std::cout << "=========================================================\n";
    test_pitch_steps();
    test_no_hang_at_high_alpha();
    test_rolls_are_coordinated_and_stop();
    test_high_alpha_rolls_do_not_depart();
    test_no_pio_in_tracking();
    std::cout << "\nAll FLCS handling tests passed successfully!\n";
    return 0;
}
