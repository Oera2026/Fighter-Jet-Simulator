#pragma once

#include "fastjet/math/vector3.hpp"
#include "fastjet/math/matrix3x3.hpp"
#include "fastjet/math/quaternion.hpp"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace fastjet::environment {

/// @brief Dynamic atmospheric wind and turbulence model
/// Models:
/// 1. Steady mean horizontal wind with logarithmic boundary layer shear
/// 2. Dryden-inspired continuous atmospheric turbulence and crosswind gusts
class WindTurbulenceModel {
public:
    // Mean wind properties at 10m reference height
    double base_wind_speed_mps = 5.0;   // ~10 knots moderate breeze
    double wind_heading_rad    = 0.785; // 045 degrees (crosswind relative to runway 000)
    bool turbulence_enabled    = true;
    /// Off: still air with the gusts kept (their intensity still follows
    /// base_wind_speed_mps). An air combat runs in still air because its
    /// missiles and rounds fly in still air.
    bool mean_wind_enabled     = true;

private:
    // Internal turbulence filter states (first-order low-pass shaping of white noise)
    double gust_u_ = 0.0;
    double gust_v_ = 0.0;
    double gust_w_ = 0.0;

    // Linear congruential generator for zero-allocation white noise
    uint32_t rng_state_ = 0x87654321;
    double noise() noexcept {
        rng_state_ = rng_state_ * 1664525u + 1013904223u;
        return (static_cast<double>(static_cast<int32_t>(rng_state_)) / 2147483648.0);
    }

public:
    WindTurbulenceModel() = default;

    /// @brief Restart the gust noise from @p seed (0 keeps the default sequence).
    /// Jets sharing one sky each need their own stream.
    void seed(uint32_t s) noexcept {
        rng_state_ = (s == 0) ? 0x87654321u : s;
        gust_u_ = gust_v_ = gust_w_ = 0.0;
    }

    /// @brief Update turbulence states for the current timestep
    /// @param dt Timestep in seconds
    /// @param altitude_m Geometric altitude AGL [m]
    /// @param airspeed_mps True airspeed [m/s]
    void update(double dt, double altitude_m, double airspeed_mps) noexcept {
        if (!turbulence_enabled) {
            gust_u_ = 0.0;
            gust_v_ = 0.0;
            gust_w_ = 0.0;
            return;
        }

        // Turbulence intensity sigma scales with base wind speed and decreases with altitude
        // Higher intensity close to terrain boundary layer (< 500m)
        const double alt = (std::max)(1.0, altitude_m);
        const double bnd_layer_factor = 1.0 + 1.5 * std::exp(-alt / 350.0);
        const double sigma = 0.06 * base_wind_speed_mps * bnd_layer_factor;

        // Characteristic turbulence scale lengths (MIL-F-8785C Dryden low-altitude form)
        // L_w = h / (0.177 + 0.000823*h)^1.2, capped between 10m and 500m
        const double L_w = std::clamp(alt * 0.5, 15.0, 450.0);
        const double V = (std::max)(airspeed_mps, 15.0);

        // Filter time constants tau = L / V
        const double tau_u = (std::max)(dt * 2.0, (2.0 * L_w) / V);
        const double tau_w = (std::max)(dt * 2.0, L_w / V);

        // Discrete low-pass filter integration
        const double alpha_u = 1.0 - std::exp(-dt / tau_u);
        const double alpha_w = 1.0 - std::exp(-dt / tau_w);

        gust_u_ += (noise() * sigma * 1.4 - gust_u_) * alpha_u;
        gust_v_ += (noise() * sigma * 1.4 - gust_v_) * alpha_u;
        gust_w_ += (noise() * sigma * 1.0 - gust_w_) * alpha_w;
    }

    /// @brief Mean ambient wind vector in NED coordinate frame [m/s]
    /// Incorporates logarithmic atmospheric boundary layer wind shear
    /// @param altitude_m Altitude above ground [m]
    math::Vector3 compute_mean_wind_ned(double altitude_m) const noexcept {
        if (!mean_wind_enabled) return math::Vector3(0.0, 0.0, 0.0);
        // Logarithmic wind profile: V(z) = V_ref * ln(z / z0) / ln(z_ref / z0)
        // z0 ~ 0.05m (airport runway / open terrain)
        constexpr double z0 = 0.05;
        constexpr double z_ref = 10.0;
        const double z = std::clamp(altitude_m, 0.5, 12000.0);

        const double log_factor = std::log(z / z0) / std::log(z_ref / z0);
        // Cap wind speed scaling at upper altitudes (jet stream limit)
        const double shear_mult = std::clamp(log_factor, 0.3, 3.5);
        const double speed = base_wind_speed_mps * shear_mult;

        // Wind blowing FROM wind_heading towards opposite direction
        // NED: North = +X, East = +Y
        const double u_ned = -speed * std::cos(wind_heading_rad);
        const double v_ned = -speed * std::sin(wind_heading_rad);

        return math::Vector3(u_ned, v_ned, 0.0);
    }

    /// @brief Total wind vector in aircraft body coordinates [m/s]
    /// Combines mean wind and instantaneous turbulence gusts
    /// @param altitude_m Altitude above ground [m]
    /// @param q_att Attitude quaternion of aircraft
    math::Vector3 compute_total_wind_body(double altitude_m, const math::Quaternion& q_att) const noexcept {
        const math::Vector3 wind_ned = compute_mean_wind_ned(altitude_m);
        const math::Matrix3x3 C_bn = q_att.to_dcm_ned_to_body();
        const math::Vector3 wind_b = C_bn * wind_ned;

        return math::Vector3(wind_b.x + gust_u_,
                             wind_b.y + gust_v_,
                             wind_b.z + gust_w_);
    }

    /// @brief Compute relative aerodynamic velocity vector v_rel in body frame [m/s]
    /// v_rel = v_body - wind_body
    math::Vector3 compute_relative_velocity(const math::Vector3& vel_b,
                                            double altitude_m,
                                            const math::Quaternion& q_att) const noexcept {
        const math::Vector3 wind_b = compute_total_wind_body(altitude_m, q_att);
        return math::Vector3(vel_b.x - wind_b.x,
                             vel_b.y - wind_b.y,
                             vel_b.z - wind_b.z);
    }
};

} // namespace fastjet::environment
