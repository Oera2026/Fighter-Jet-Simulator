#pragma once

#include "vector3.hpp"
#include "matrix3x3.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace fastjet {
namespace math {

/**
 * @brief Unit quaternion for attitude representation (Hamilton convention).
 * Represents rotation from NED frame to Body frame:
 *   v_body = C_{b/n} * v_ned
 *   v_ned  = C_{n/b} * v_body = (C_{b/n})^T * v_body
 *
 * Quaternion components: q = w + x*i + y*j + z*k
 * Zero heap allocation, fully deterministic.
 */
struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Quaternion() noexcept = default;
    constexpr Quaternion(double w_, double x_, double y_, double z_) noexcept
        : w(w_), x(x_), y(y_), z(z_) {}

    static constexpr Quaternion identity() noexcept {
        return {1.0, 0.0, 0.0, 0.0};
    }

    [[nodiscard]] constexpr Quaternion operator+(const Quaternion& rhs) const noexcept {
        return {w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z};
    }

    [[nodiscard]] constexpr Quaternion operator-(const Quaternion& rhs) const noexcept {
        return {w - rhs.w, x - rhs.x, y - rhs.y, z - rhs.z};
    }

    [[nodiscard]] constexpr Quaternion operator*(double scalar) const noexcept {
        return {w * scalar, x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] constexpr Quaternion operator/(double scalar) const noexcept {
        const double inv = 1.0 / scalar;
        return {w * inv, x * inv, y * inv, z * inv};
    }

    constexpr Quaternion& operator+=(const Quaternion& rhs) noexcept {
        w += rhs.w;
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    constexpr Quaternion& operator*=(double scalar) noexcept {
        w *= scalar;
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    /**
     * @brief Hamilton quaternion multiplication: this * rhs
     */
    [[nodiscard]] constexpr Quaternion operator*(const Quaternion& r) const noexcept {
        return {
            w * r.w - x * r.x - y * r.y - z * r.z,
            w * r.x + x * r.w + y * r.z - z * r.y,
            w * r.y - x * r.z + y * r.w + z * r.x,
            w * r.z + x * r.y - y * r.x + z * r.w
        };
    }

    [[nodiscard]] constexpr double norm_squared() const noexcept {
        return w * w + x * x + y * y + z * z;
    }

    [[nodiscard]] double norm() const noexcept {
        return std::sqrt(norm_squared());
    }

    [[nodiscard]] Quaternion conjugate() const noexcept {
        return {w, -x, -y, -z};
    }

    void normalize() noexcept {
        const double n = norm();
        if (n > 1e-15) {
            const double inv = 1.0 / n;
            w *= inv;
            x *= inv;
            y *= inv;
            z *= inv;
        } else {
            w = 1.0;
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }
    }

    [[nodiscard]] Quaternion normalized() const noexcept {
        Quaternion q = *this;
        q.normalize();
        return q;
    }

    /**
     * @brief Direction Cosine Matrix (DCM) transforming from NED frame to Body frame:
     * v_body = C_b_n * v_ned
     */
    [[nodiscard]] constexpr Matrix3x3 to_dcm_ned_to_body() const noexcept {
        const double xx = x * x;
        const double yy = y * y;
        const double zz = z * z;
        const double xy = x * y;
        const double xz = x * z;
        const double yz = y * z;
        const double wx = w * x;
        const double wy = w * y;
        const double wz = w * z;

        return {
            1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz),       2.0 * (xz - wy),
            2.0 * (xy - wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx),
            2.0 * (xz + wy),       2.0 * (yz - wx),       1.0 - 2.0 * (xx + yy)
        };
    }

    /**
     * @brief Direction Cosine Matrix (DCM) transforming from Body frame to NED frame:
     * v_ned = C_n_b * v_body = (C_b_n)^T * v_body
     */
    [[nodiscard]] constexpr Matrix3x3 to_dcm_body_to_ned() const noexcept {
        return to_dcm_ned_to_body().transpose();
    }

    /**
     * @brief Transforms vector from NED to Body frame: v_b = C_b_n * v_n
     */
    [[nodiscard]] constexpr Vector3 rotate_ned_to_body(const Vector3& v_ned) const noexcept {
        return to_dcm_ned_to_body() * v_ned;
    }

    /**
     * @brief Transforms vector from Body to NED frame: v_n = C_n_b * v_b
     */
    [[nodiscard]] constexpr Vector3 rotate_body_to_ned(const Vector3& v_body) const noexcept {
        return to_dcm_body_to_ned() * v_body;
    }

    /**
     * @brief Computes quaternion time derivative from angular velocity in body frame:
     * \dot{q} = 0.5 * q \otimes [0, p, q, r]
     *
     * @param omega_b Body angular velocity vector [p, q, r] in rad/s
     */
    [[nodiscard]] constexpr Quaternion derivative(const Vector3& omega_b) const noexcept {
        const double p = omega_b.x;
        const double q = omega_b.y;
        const double r = omega_b.z;

        return {
            0.5 * (-x * p - y * q - z * r),
            0.5 * ( w * p + y * r - z * q),
            0.5 * ( w * q - x * r + z * p),
            0.5 * ( w * r + x * q - y * p)
        };
    }

    /**
     * @brief Construct quaternion from 3-2-1 (yaw-pitch-roll) Euler angles in radians.
     */
    static Quaternion from_euler(double roll, double pitch, double yaw) noexcept {
        const double cr = std::cos(roll * 0.5);
        const double sr = std::sin(roll * 0.5);
        const double cp = std::cos(pitch * 0.5);
        const double sp = std::sin(pitch * 0.5);
        const double cy = std::cos(yaw * 0.5);
        const double sy = std::sin(yaw * 0.5);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    /**
     * @brief Quaternion of a NED-to-body DCM (the inverse of to_dcm_ned_to_body),
     * by Shepperd's method: divide by the largest component, so the result
     * stays accurate at every attitude.
     */
    [[nodiscard]] static Quaternion from_dcm_ned_to_body(const Matrix3x3& m) noexcept {
        const double trace = m(0, 0) + m(1, 1) + m(2, 2);
        Quaternion q;
        if (trace >= m(0, 0) && trace >= m(1, 1) && trace >= m(2, 2)) {
            const double w4 = 2.0 * std::sqrt(std::max(0.0, 1.0 + trace));
            q = {0.25 * w4, (m(1, 2) - m(2, 1)) / w4, (m(2, 0) - m(0, 2)) / w4, (m(0, 1) - m(1, 0)) / w4};
        } else if (m(0, 0) >= m(1, 1) && m(0, 0) >= m(2, 2)) {
            const double x4 = 2.0 * std::sqrt(std::max(0.0, 1.0 + m(0, 0) - m(1, 1) - m(2, 2)));
            q = {(m(1, 2) - m(2, 1)) / x4, 0.25 * x4, (m(0, 1) + m(1, 0)) / x4, (m(0, 2) + m(2, 0)) / x4};
        } else if (m(1, 1) >= m(2, 2)) {
            const double y4 = 2.0 * std::sqrt(std::max(0.0, 1.0 - m(0, 0) + m(1, 1) - m(2, 2)));
            q = {(m(2, 0) - m(0, 2)) / y4, (m(0, 1) + m(1, 0)) / y4, 0.25 * y4, (m(1, 2) + m(2, 1)) / y4};
        } else {
            const double z4 = 2.0 * std::sqrt(std::max(0.0, 1.0 - m(0, 0) - m(1, 1) + m(2, 2)));
            q = {(m(0, 1) - m(1, 0)) / z4, (m(0, 2) + m(2, 0)) / z4, (m(1, 2) + m(2, 1)) / z4, 0.25 * z4};
        }
        q.normalize();
        return q;
    }

    /// @brief Quaternion of a body-to-NED DCM (columns are the body axes in NED).
    [[nodiscard]] static Quaternion from_dcm_body_to_ned(const Matrix3x3& c) noexcept {
        return from_dcm_ned_to_body(c.transpose());
    }

    /**
     * @brief Spherical Linear Interpolation (slerp) between two unit quaternions.
     * @param q0 Start quaternion (t = 0)
     * @param q1 End quaternion (t = 1)
     * @param t Interpolation parameter in [0, 1]
     */
    [[nodiscard]] static Quaternion slerp(const Quaternion& q0, const Quaternion& q1, double t) noexcept {
        double dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;
        Quaternion target = q1;
        if (dot < 0.0) {
            dot = -dot;
            target = {-q1.w, -q1.x, -q1.y, -q1.z};
        }
        if (dot > 0.9995) {
            Quaternion q{
                q0.w + t * (target.w - q0.w),
                q0.x + t * (target.x - q0.x),
                q0.y + t * (target.y - q0.y),
                q0.z + t * (target.z - q0.z)
            };
            q.normalize();
            return q;
        }
        const double theta_0 = std::acos(std::clamp(dot, -1.0, 1.0));
        const double theta = theta_0 * t;
        const double sin_theta = std::sin(theta);
        const double sin_theta_0 = std::sin(theta_0);
        const double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        const double s1 = sin_theta / sin_theta_0;
        return {
            s0 * q0.w + s1 * target.w,
            s0 * q0.x + s1 * target.x,
            s0 * q0.y + s1 * target.y,
            s0 * q0.z + s1 * target.z
        };
    }

    /**
     * @brief Extract Euler angles (roll, pitch, yaw) in radians.
     * Note: Subject to gimbal lock at pitch = +/- pi/2, but quaternion integration itself is not.
     */
    struct EulerAngles {
        double roll{0.0};   // phi
        double pitch{0.0};  // theta
        double yaw{0.0};    // psi
    };

    [[nodiscard]] EulerAngles to_euler() const noexcept {
        EulerAngles ea;
        // Pitch (theta)
        const double sin_pitch = -2.0 * (x * z - w * y);
        if (std::abs(sin_pitch) >= 1.0) {
            ea.pitch = std::copysign(M_PI / 2.0, sin_pitch);
        } else {
            ea.pitch = std::asin(sin_pitch);
        }

        // Roll (phi)
        ea.roll = std::atan2(2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y));

        // Yaw (psi)
        ea.yaw = std::atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z));

        return ea;
    }
};

inline std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    os << "[" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]";
    return os;
}

} // namespace math
} // namespace fastjet
