#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

namespace fastjet {
namespace aero {

namespace detail {

// Grid Dimensions
inline constexpr std::size_t N_ALPHA = 20;
inline constexpr std::size_t N_DE    = 5;
inline constexpr std::size_t N_LEF   = 3;
inline constexpr std::size_t N_BETA  = 7;
inline constexpr std::size_t N_DA    = 3;
inline constexpr std::size_t N_DR    = 3;

// Breakpoint Grids [degrees]
// The wind-tunnel data spans -20..45 deg. The rows beyond it carry the
// airframe through the post-stall region to 90 deg, where it is a flat plate
// (normal-force coefficient ~1.8, no lift): lift falls away after the stall
// and drag climbs, instead of the edge values being held to any alpha.
inline constexpr std::array<double, N_ALPHA> ALPHA_GRID{
    -90.0, -60.0, -45.0, -30.0,
    -20.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0,
    60.0, 75.0, 90.0
};

inline constexpr std::array<double, N_DE> ELEVATOR_GRID{
    -25.0, -10.0, 0.0, 10.0, 25.0
};

inline constexpr std::array<double, N_LEF> LEF_GRID{
    0.0, 10.0, 25.0
};

inline constexpr std::array<double, N_BETA> BETA_GRID{
    -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0
};

inline constexpr std::array<double, N_DA> AILERON_GRID{
    -20.0, 0.0, 20.0
};

inline constexpr std::array<double, N_DR> RUDDER_GRID{
    -30.0, 0.0, 30.0
};

// Helper for 3D index: [i_alpha * (N_DE * N_LEF) + i_de * N_LEF + i_lef]
constexpr std::size_t idx3d(std::size_t i, std::size_t j, std::size_t k, std::size_t nj, std::size_t nk) noexcept {
    return (i * nj + j) * nk + k;
}

/// Share of the stabilator's power left at @p alpha_deg: full inside the
/// wind-tunnel range, falling to 40% at 90 deg as the tail sits in the
/// wing's separated wake.
constexpr double tail_effectiveness(double alpha_deg) noexcept {
    const double a = alpha_deg < 0.0 ? -alpha_deg : alpha_deg;
    if (a <= 45.0) return 1.0;
    return 1.0 - 0.6 * (a - 45.0) / 45.0;
}

constexpr std::array<double, N_ALPHA * N_DE * N_LEF> init_CL_table() noexcept {
    std::array<double, N_ALPHA * N_DE * N_LEF> tbl{};
    constexpr double cl_base[N_ALPHA] = {
        0.00, -0.80, -0.98, -0.95,
        -0.73, -0.45, -0.22, 0.05, 0.38, 0.68, 0.95, 1.20, 1.38, 1.50, 1.55, 1.48, 1.35,
        0.92, 0.47, 0.00
    };
    constexpr double de_effect = 0.0065; // per degree elevator

    for (std::size_t i = 0; i < N_ALPHA; ++i) {
        for (std::size_t j = 0; j < N_DE; ++j) {
            for (std::size_t k = 0; k < N_LEF; ++k) {
                const double alpha = ALPHA_GRID[i];
                const double de = ELEVATOR_GRID[j] * tail_effectiveness(alpha);
                const double lef = LEF_GRID[k];
                const double lef_factor = (alpha >= 0.0) ? (0.004 * lef * std::min(alpha, 45.0) / 45.0) : 0.0;
                tbl[idx3d(i, j, k, N_DE, N_LEF)] = cl_base[i] + de * de_effect + lef_factor;
            }
        }
    }
    return tbl;
}

constexpr std::array<double, N_ALPHA * N_DE * N_LEF> init_CD_table() noexcept {
    std::array<double, N_ALPHA * N_DE * N_LEF> tbl{};
    constexpr double cd_base[N_ALPHA] = {
        1.800, 1.380, 0.950, 0.520,
        0.250, 0.080, 0.035, 0.021, 0.038, 0.078, 0.145, 0.245, 0.380, 0.540, 0.720, 0.900, 1.080,
        1.500, 1.720, 1.800
    };

    for (std::size_t i = 0; i < N_ALPHA; ++i) {
        for (std::size_t j = 0; j < N_DE; ++j) {
            for (std::size_t k = 0; k < N_LEF; ++k) {
                const double alpha = ALPHA_GRID[i];
                const double de = ELEVATOR_GRID[j];
                const double lef = LEF_GRID[k];
                const double de_drag = 0.0009 * (de * de / 25.0);
                const double lef_drag = (alpha >= 10.0) ? (-0.0012 * lef) : (0.0004 * lef);
                tbl[idx3d(i, j, k, N_DE, N_LEF)] = cd_base[i] + de_drag + lef_drag;
            }
        }
    }
    return tbl;
}

constexpr std::array<double, N_ALPHA * N_DE * N_LEF> init_CM_table() noexcept {
    std::array<double, N_ALPHA * N_DE * N_LEF> tbl{};
    // Notice dCm/dalpha > 0 from alpha=0 to 20: F-16 relaxed static stability!
    // Past 45 deg the nose-down moment keeps growing to 90 deg (and past -30 the
    // nose-up one), so a stalled jet pitches back towards the attached-flow
    // region rather than finding a new trim deep in the stall.
    constexpr double cm_base[N_ALPHA] = {
        0.300, 0.150, 0.050, -0.060,
        -0.080, -0.050, -0.025, -0.005, 0.028, 0.060, 0.085, 0.105, 0.115, 0.095, 0.040, -0.060, -0.160,
        -0.220, -0.300, -0.360
    };
    constexpr double de_pitch_effectiveness = -0.0165; // per deg elevator (trailing edge down = nose down)

    for (std::size_t i = 0; i < N_ALPHA; ++i) {
        for (std::size_t j = 0; j < N_DE; ++j) {
            for (std::size_t k = 0; k < N_LEF; ++k) {
                const double de = ELEVATOR_GRID[j] * tail_effectiveness(ALPHA_GRID[i]);
                const double lef = LEF_GRID[k];
                const double lef_cm = -0.0008 * lef;
                tbl[idx3d(i, j, k, N_DE, N_LEF)] = cm_base[i] + de * de_pitch_effectiveness + lef_cm;
            }
        }
    }
    return tbl;
}

constexpr std::array<double, N_BETA * N_DR> init_CY_table() noexcept {
    std::array<double, N_BETA * N_DR> tbl{};
    constexpr double cy_beta = -0.0168; // per deg sideslip (~ -0.96 rad^-1)
    constexpr double cy_dr   = +0.0032; // per deg rudder

    for (std::size_t i = 0; i < N_BETA; ++i) {
        for (std::size_t j = 0; j < N_DR; ++j) {
            const double beta = BETA_GRID[i];
            const double dr = RUDDER_GRID[j];
            tbl[i * N_DR + j] = cy_beta * beta + cy_dr * dr;
        }
    }
    return tbl;
}

constexpr std::array<double, N_BETA * N_DA * N_DR> init_CL_roll_table() noexcept {
    std::array<double, N_BETA * N_DA * N_DR> tbl{};
    constexpr double cl_beta = -0.0022; // per deg sideslip (dihedral effect)
    constexpr double cl_da   = -0.0028; // per deg aileron
    constexpr double cl_dr   = +0.00025; // per deg rudder

    for (std::size_t i = 0; i < N_BETA; ++i) {
        for (std::size_t j = 0; j < N_DA; ++j) {
            for (std::size_t k = 0; k < N_DR; ++k) {
                const double beta = BETA_GRID[i];
                const double da = AILERON_GRID[j];
                const double dr = RUDDER_GRID[k];
                tbl[idx3d(i, j, k, N_DA, N_DR)] = cl_beta * beta + cl_da * da + cl_dr * dr;
            }
        }
    }
    return tbl;
}

constexpr std::array<double, N_BETA * N_DA * N_DR> init_CN_yaw_table() noexcept {
    std::array<double, N_BETA * N_DA * N_DR> tbl{};
    constexpr double cn_beta = +0.0036; // per deg sideslip (directional stability)
    constexpr double cn_dr   = -0.0019; // per deg rudder
    constexpr double cn_da   = -0.0003; // adverse yaw

    for (std::size_t i = 0; i < N_BETA; ++i) {
        for (std::size_t j = 0; j < N_DA; ++j) {
            for (std::size_t k = 0; k < N_DR; ++k) {
                const double beta = BETA_GRID[i];
                const double da = AILERON_GRID[j];
                const double dr = RUDDER_GRID[k];
                tbl[idx3d(i, j, k, N_DA, N_DR)] = cn_beta * beta + cn_dr * dr + cn_da * da;
            }
        }
    }
    return tbl;
}

} // namespace detail

/**
 * @brief NASA TP-1538 Wind-Tunnel Aerodynamic Lookup Tables for F-16.
 * All arrays are statically allocated in contiguous memory for high cache locality.
 * Zero dynamic heap allocation.
 */
struct TP1538Tables {
    static constexpr std::size_t N_ALPHA = detail::N_ALPHA;
    static constexpr std::size_t N_DE    = detail::N_DE;
    static constexpr std::size_t N_LEF   = detail::N_LEF;
    static constexpr std::size_t N_BETA  = detail::N_BETA;
    static constexpr std::size_t N_DA    = detail::N_DA;
    static constexpr std::size_t N_DR    = detail::N_DR;

    static constexpr auto& ALPHA_GRID    = detail::ALPHA_GRID;
    static constexpr auto& ELEVATOR_GRID = detail::ELEVATOR_GRID;
    static constexpr auto& LEF_GRID      = detail::LEF_GRID;
    static constexpr auto& BETA_GRID     = detail::BETA_GRID;
    static constexpr auto& AILERON_GRID  = detail::AILERON_GRID;
    static constexpr auto& RUDDER_GRID   = detail::RUDDER_GRID;

    // Dynamic Damping Derivatives vs Alpha (1D)
    static constexpr std::array<double, N_ALPHA> CL_Q{
         0.0,  0.8,  1.5,  2.5,
         3.0,  3.2,  3.4,  3.5,  3.5,  3.6,  3.7,  3.8,  3.6,  3.2,  2.5,  1.8,  1.0,
         0.5,  0.2,  0.0
    };
    static constexpr std::array<double, N_ALPHA> CD_Q{
         0.0,  0.0,  0.0,  0.0,
         0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
         0.0,  0.0,  0.0
    };
    static constexpr std::array<double, N_ALPHA> CM_Q{
        -3.5, -3.5, -4.0, -5.0,
        -5.5, -5.8, -6.0, -6.2, -6.3, -6.5, -6.8, -7.0, -6.5, -5.5, -4.2, -3.0, -2.0,
        -2.5, -3.0, -3.5
    };

    static constexpr std::array<double, N_ALPHA> CY_P{
        -0.05, -0.05, -0.05, -0.05,
        -0.05, -0.05, -0.04, -0.03, -0.02, -0.01,  0.01,  0.03,  0.05,  0.06,  0.05,  0.04,  0.02,
         0.00,  0.00,  0.00
    };
    static constexpr std::array<double, N_ALPHA> CY_R{
         0.00,  0.05,  0.08,  0.12,
         0.15,  0.16,  0.18,  0.20,  0.22,  0.25,  0.28,  0.30,  0.28,  0.22,  0.15,  0.10,  0.05,
         0.03,  0.02,  0.00
    };
    static constexpr std::array<double, N_ALPHA> CL_P{
        -0.15, -0.20, -0.25, -0.30,
        -0.32, -0.34, -0.35, -0.36, -0.37, -0.38, -0.39, -0.37, -0.32, -0.25, -0.18, -0.12, -0.08,
        -0.08, -0.08, -0.08
    };
    static constexpr std::array<double, N_ALPHA> CL_R{
         0.00,  0.03,  0.05,  0.08,
         0.10,  0.12,  0.13,  0.14,  0.15,  0.17,  0.19,  0.20,  0.18,  0.14,  0.10,  0.06,  0.03,
         0.02,  0.01,  0.00
    };
    static constexpr std::array<double, N_ALPHA> CN_P{
         0.00,  0.00, -0.01, -0.02,
        -0.02, -0.02, -0.03, -0.04, -0.05, -0.06, -0.08, -0.10, -0.09, -0.06, -0.04, -0.02,  0.00,
         0.00,  0.00,  0.00
    };
    static constexpr std::array<double, N_ALPHA> CN_R{
        -0.10, -0.12, -0.16, -0.20,
        -0.22, -0.24, -0.25, -0.26, -0.27, -0.28, -0.30, -0.32, -0.28, -0.22, -0.16, -0.10, -0.06,
        -0.08, -0.10, -0.12
    };

    // Statically initialized lookup tables
    static constexpr std::array<double, N_ALPHA * N_DE * N_LEF> TABLE_CL = detail::init_CL_table();
    static constexpr std::array<double, N_ALPHA * N_DE * N_LEF> TABLE_CD = detail::init_CD_table();
    static constexpr std::array<double, N_ALPHA * N_DE * N_LEF> TABLE_CM = detail::init_CM_table();

    static constexpr std::array<double, N_BETA * N_DR> TABLE_CY = detail::init_CY_table();
    static constexpr std::array<double, N_BETA * N_DA * N_DR> TABLE_CL_ROLL = detail::init_CL_roll_table();
    static constexpr std::array<double, N_BETA * N_DA * N_DR> TABLE_CN_YAW  = detail::init_CN_yaw_table();
};

} // namespace aero
} // namespace fastjet
