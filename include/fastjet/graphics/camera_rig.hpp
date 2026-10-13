#pragma once

#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/terrain_field.hpp"
#include "fastjet/fdm/flight_state.hpp"
#include "fastjet/flcs/imu_sensor.hpp"
#include <algorithm>

namespace fastjet::graphics {

enum class CameraMode {
    COCKPIT = 0,
    CHASE   = 1,
    PADLOCK = 2
};

/// @brief Camera rig managing the pilot's Design Eye Point (DEP) and G-load spring-damper dynamics
class CameraRig {
public:
    // F-16 Cockpit Design Eye Point (DEP) relative to aircraft CG in body frame [m]
    // +X = Forward (+1.80m), +Y = Right (0.0m), +Z = Down (-0.65m = 0.65m above CG)
    static constexpr float DEP_X = 1.80f;
    static constexpr float DEP_Y = 0.00f;
    static constexpr float DEP_Z = -0.65f;

    // Physical neck/harness limits [m]
    static constexpr float LIMIT_X = 0.05f;   // Fore/Aft limit (+/- 5 cm)
    static constexpr float LIMIT_Y = 0.06f;   // Lateral limit (+/- 6 cm)
    static constexpr float LIMIT_Z_MIN = -0.03f; // Upward limit (3 cm before canopy)
    static constexpr float LIMIT_Z_MAX = 0.06f;  // Downward limit (6 cm into seat)

private:
    CameraMode mode_ = CameraMode::COCKPIT;

    // 2nd-order spring-damper state for head displacement relative to DEP [m]
    math::Vector3 disp_b_{0.0, 0.0, 0.0};     // [m]
    math::Vector3 vel_b_{0.0, 0.0, 0.0};      // [m/s]

    // Spring-damper parameters
    double omega_n_ = 25.0; // Natural frequency [rad/s] (~4 Hz head response)
    double zeta_    = 0.85; // Damping ratio (critically damped, no oscillatory nausea)

    // Sensitivity gains [m/G]
    double k_x_ = 0.008;  // Fore/aft displacement per Gx [m/G]
    double k_y_ = 0.012;  // Lateral displacement per Gy [m/G]
    double k_z_ = 0.0045; // Vertical displacement per Gz [m/G]

    // Head look orientation angles relative to cockpit boresight [rad]
    float head_yaw_   = 0.0f; // Left (-) / Right (+) [rad]
    float head_pitch_ = 0.0f; // Down (-) / Up (+) [rad]

    // Chase camera parameters
    float chase_distance_ = 16.0f; // [m]
    float chase_height_   = 3.2f;  // [m]
    float chase_yaw_      = 0.0f;  // [rad] Orbit yaw
    float chase_pitch_    = 0.0f;  // [rad] Orbit pitch

    // Anatomical head rotation limits inside F-16 cockpit
    static constexpr float HEAD_YAW_LIMIT   = 1.92f;  // ~110 degrees left/right
    static constexpr float HEAD_PITCH_MIN   = -1.13f; // ~65 degrees down (towards consoles/MFDs)
    static constexpr float HEAD_PITCH_MAX   = 1.48f;  // ~85 degrees up (canopy apex)

public:
    CameraRig() = default;

    void set_mode(CameraMode m) noexcept { mode_ = m; }
    CameraMode mode() const noexcept { return mode_; }
    void toggle_mode() noexcept {
        if (mode_ == CameraMode::COCKPIT) {
            mode_ = CameraMode::CHASE;
        } else {
            mode_ = CameraMode::COCKPIT;
            reset_head_look();
        }
    }

    void toggle_padlock() noexcept {
        if (mode_ == CameraMode::PADLOCK) {
            mode_ = CameraMode::COCKPIT;
            reset_head_look();
        } else {
            mode_ = CameraMode::PADLOCK;
        }
    }

    /// @brief Track target in body coordinates respecting physiological neck limits
    /// @param dt Timestep in seconds
    /// @param target_dir_b Unit line-of-sight vector to target in ownship body frame (+X nose, +Y right, +Z belly)
    void update_padlock(double dt, const math::Vector3& target_dir_b, bool has_target) noexcept {
        if (mode_ != CameraMode::PADLOCK) return;
        if (!has_target) {
            // Smoothly recenter head if target lost
            head_yaw_ += (0.0f - head_yaw_) * std::clamp(static_cast<float>(dt * 4.0), 0.0f, 1.0f);
            head_pitch_ += (0.0f - head_pitch_) * std::clamp(static_cast<float>(dt * 4.0), 0.0f, 1.0f);
            return;
        }

        // Body frame: +X forward, +Y right, +Z down
        // Target yaw: rotation around +Z (right is positive) -> atan2(Y, X)
        // Target pitch: elevation above horizon (up is positive) -> atan2(-Z, sqrt(X*X + Y*Y))
        const float target_yaw = static_cast<float>(std::atan2(target_dir_b.y, target_dir_b.x));
        const double ground_proj = std::sqrt(target_dir_b.x * target_dir_b.x + target_dir_b.y * target_dir_b.y);
        const float target_pitch = static_cast<float>(std::atan2(-target_dir_b.z, ground_proj));

        // Clamped by human neck mobility inside bubble canopy
        const float clamped_yaw = std::clamp(target_yaw, -HEAD_YAW_LIMIT, HEAD_YAW_LIMIT);
        const float clamped_pitch = std::clamp(target_pitch, HEAD_PITCH_MIN, HEAD_PITCH_MAX);

        // Smooth anatomical tracking slew (~8 rad/s head rate)
        const float slew = std::clamp(static_cast<float>(dt * 8.0), 0.0f, 1.0f);
        head_yaw_ += (clamped_yaw - head_yaw_) * slew;
        head_pitch_ += (clamped_pitch - head_pitch_) * slew;
    }

    void zoom_chase(float delta) noexcept {
        chase_distance_ = std::clamp(chase_distance_ - delta, 6.0f, 45.0f);
    }

    /// @brief Reset head displacement and look angles to center boresight
    void reset() noexcept {
        disp_b_ = math::Vector3::zero();
        vel_b_ = math::Vector3::zero();
        head_yaw_ = 0.0f;
        head_pitch_ = 0.0f;
        chase_yaw_ = 0.0f;
        chase_pitch_ = 0.0f;
    }

    /// @brief Add mouse look delta angles [rad]
    void add_head_look(float delta_yaw, float delta_pitch) noexcept {
        if (mode_ == CameraMode::COCKPIT) {
            head_yaw_   = std::clamp(head_yaw_ + delta_yaw, -HEAD_YAW_LIMIT, HEAD_YAW_LIMIT);
            head_pitch_ = std::clamp(head_pitch_ + delta_pitch, HEAD_PITCH_MIN, HEAD_PITCH_MAX);
        } else {
            chase_yaw_   += delta_yaw;
            chase_pitch_ = std::clamp(chase_pitch_ + delta_pitch, -1.35f, 1.35f);
        }
    }

    /// @brief Recenter head look forward
    void reset_head_look() noexcept {
        head_yaw_ = 0.0f;
        head_pitch_ = 0.0f;
        chase_yaw_ = 0.0f;
        chase_pitch_ = 0.0f;
    }

    float head_yaw() const noexcept { return head_yaw_; }
    float head_pitch() const noexcept { return head_pitch_; }
    float chase_distance() const noexcept { return chase_distance_; }

    /// @brief 3x3 rotation matrix for pilot head orientation in camera space
    Mat4 head_look_matrix() const noexcept {
        const float cy = std::cos(head_yaw_);
        const float sy = std::sin(head_yaw_);
        const float cp = std::cos(head_pitch_);
        const float sp = std::sin(head_pitch_);

        Mat4 r = Mat4::identity();
        r(0, 0) =  cy;
        r(0, 1) =  0.0f;
        r(0, 2) =  sy;

        r(1, 0) = -sp * sy;
        r(1, 1) =  cp;
        r(1, 2) =  sp * cy;

        r(2, 0) = -cp * sy;
        r(2, 1) = -sp;
        r(2, 2) =  cp * cy;

        return r;
    }

    /// @brief Update spring-damper dynamics based on cockpit IMU G-loads
    /// @param dt Timestep in seconds
    /// @param imu IMU sensor data containing Nz, Ny, Nx
    void update(double dt, const flcs::IMUData& imu) noexcept {
        // Target steady-state displacement in body coordinates
        // At 1G wings-level trim: Nz = 1.0, Ny = 0, Nx = 0 -> target = [0, 0, 0]
        const double target_x = -k_x_ * imu.Nx;
        const double target_y = -k_y_ * imu.Ny;
        const double target_z =  k_z_ * (imu.Nz - 1.0); // +Z is downward into seat

        // 2nd-order system: ddot_x = -omega_n^2 * (x - target) - 2*zeta*omega_n * dot_x
        const double omega2 = omega_n_ * omega_n_;
        const double two_zeta_omega = 2.0 * zeta_ * omega_n_;

        const double accel_x = -omega2 * (disp_b_.x - target_x) - two_zeta_omega * vel_b_.x;
        const double accel_y = -omega2 * (disp_b_.y - target_y) - two_zeta_omega * vel_b_.y;
        const double accel_z = -omega2 * (disp_b_.z - target_z) - two_zeta_omega * vel_b_.z;

        // Semi-implicit Euler integration
        vel_b_.x += accel_x * dt;
        vel_b_.y += accel_y * dt;
        vel_b_.z += accel_z * dt;

        disp_b_.x += vel_b_.x * dt;
        disp_b_.y += vel_b_.y * dt;
        disp_b_.z += vel_b_.z * dt;

        // Clamp to physical anatomical/cockpit limits
        disp_b_.x = std::clamp(disp_b_.x, -static_cast<double>(LIMIT_X), static_cast<double>(LIMIT_X));
        disp_b_.y = std::clamp(disp_b_.y, -static_cast<double>(LIMIT_Y), static_cast<double>(LIMIT_Y));
        disp_b_.z = std::clamp(disp_b_.z, static_cast<double>(LIMIT_Z_MIN), static_cast<double>(LIMIT_Z_MAX));
    }

    /// @brief Toggle G-induced head displacement in cockpit view (default false for rock-solid MSFS/real-jet stability)
    bool g_head_motion_{false};

    /// @brief Instantaneous pilot eye position in body coordinates [m]
    /// Anchored to nominal DEP by default so the cockpit and HUD stay rock-solid in place on screen like MSFS and real jets
    math::Vector3 eye_pos_body() const noexcept {
        if (g_head_motion_) {
            return math::Vector3(DEP_X + disp_b_.x, DEP_Y + disp_b_.y, DEP_Z + disp_b_.z);
        }
        return math::Vector3(DEP_X, DEP_Y, DEP_Z);
    }

    /// @brief Instantaneous pilot head displacement from nominal DEP [m]
    math::Vector3 head_displacement() const noexcept {
        return disp_b_;
    }

    void set_g_head_motion(bool enabled) noexcept { g_head_motion_ = enabled; }
    [[nodiscard]] bool g_head_motion() const noexcept { return g_head_motion_; }

    /// @brief World-space eye position in NED frame [m]
    math::Vector3 eye_pos_ned(const fdm::FlightState& state) const noexcept {
        if (mode_ != CameraMode::CHASE) {
            const math::Vector3 eye_b = eye_pos_body();
            return state.pos_ned + state.q_att.rotate_body_to_ned(eye_b);
        } else {
            // Chase camera: position behind and above aircraft, rotated by orbit angles
            const float cy = std::cos(chase_yaw_);
            const float sy = std::sin(chase_yaw_);
            const float cp = std::cos(chase_pitch_);
            const float sp = std::sin(chase_pitch_);

            // Offset in body coordinates: -X is behind, +Y is right, -Z is up
            const double ox = -static_cast<double>(chase_distance_ * cp * cy);
            const double oy =  static_cast<double>(chase_distance_ * cp * sy);
            const double oz = -static_cast<double>(chase_height_) - static_cast<double>(chase_distance_ * sp);

            const math::Vector3 offset_ned = state.q_att.rotate_body_to_ned(math::Vector3(ox, oy, oz));
            math::Vector3 pos = state.pos_ned + offset_ned;
            // Prevent chase camera from dipping below the local terrain surface
            const double terr_h = static_cast<double>(TerrainField::height(
                static_cast<float>(pos.x), static_cast<float>(pos.y)));
            const double max_cam_z = -terr_h - 0.75; // at least 0.75m above ground
            if (pos.z > max_cam_z) {
                pos.z = max_cam_z;
            }
            return pos;
        }
    }

    /// @brief LookAt view matrix for Chase camera
    Mat4 compute_chase_view_matrix(const fdm::FlightState& state) const noexcept {
        const math::Vector3 eye = eye_pos_ned(state);
        // Look at point slightly ahead and above CG (cockpit canopy center)
        const math::Vector3 target = state.pos_ned + state.q_att.rotate_body_to_ned(math::Vector3(0.0, 0.0, -0.65));

        math::Vector3 f = (target - eye).normalized();
        math::Vector3 up_approx = state.q_att.rotate_body_to_ned(math::Vector3(0.0, 0.0, -1.0));

        math::Vector3 s = f.cross(up_approx).normalized();
        if (s.norm_squared() < 1e-6) {
            s = math::Vector3(0.0, 1.0, 0.0);
        }
        math::Vector3 u = s.cross(f);

        Mat4 v = Mat4::identity();
        v(0, 0) = static_cast<float>(s.x);
        v(0, 1) = static_cast<float>(s.y);
        v(0, 2) = static_cast<float>(s.z);
        v(0, 3) = -static_cast<float>(s.dot(eye));

        v(1, 0) = static_cast<float>(u.x);
        v(1, 1) = static_cast<float>(u.y);
        v(1, 2) = static_cast<float>(u.z);
        v(1, 3) = -static_cast<float>(u.dot(eye));

        v(2, 0) = -static_cast<float>(f.x);
        v(2, 1) = -static_cast<float>(f.y);
        v(2, 2) = -static_cast<float>(f.z);
        v(2, 3) = static_cast<float>(f.dot(eye));

        return v;
    }

    /// @brief Compute OpenGL camera View Matrix for external 3D world rendering
    /// @param state Current aircraft flight state
    Mat4 compute_world_view_matrix(const fdm::FlightState& state) const noexcept {
        if (mode_ == CameraMode::CHASE) {
            return compute_chase_view_matrix(state);
        }

        // DCM C_bn converts NED vectors to Body vectors
        const math::Matrix3x3 C_bn = state.q_att.to_dcm_ned_to_body();

        // Base boresight camera orientation:
        // Row 0 = Row 1 of C_bn (Body Y = Cam Right)
        // Row 1 = -Row 2 of C_bn (Body -Z = Cam Up)
        // Row 2 = -Row 0 of C_bn (Body -X = Cam Backwards)
        Mat4 base_rot = Mat4::identity();
        base_rot(0, 0) = static_cast<float>(C_bn(1, 0));
        base_rot(0, 1) = static_cast<float>(C_bn(1, 1));
        base_rot(0, 2) = static_cast<float>(C_bn(1, 2));

        base_rot(1, 0) = -static_cast<float>(C_bn(2, 0));
        base_rot(1, 1) = -static_cast<float>(C_bn(2, 1));
        base_rot(1, 2) = -static_cast<float>(C_bn(2, 2));

        base_rot(2, 0) = -static_cast<float>(C_bn(0, 0));
        base_rot(2, 1) = -static_cast<float>(C_bn(0, 1));
        base_rot(2, 2) = -static_cast<float>(C_bn(0, 2));

        // Apply head look rotation
        const Mat4 rot = head_look_matrix() * base_rot;

        Mat4 view = rot;
        const math::Vector3 eye_ned = eye_pos_ned(state);
        view(0, 3) = -(rot(0, 0) * static_cast<float>(eye_ned.x) +
                       rot(0, 1) * static_cast<float>(eye_ned.y) +
                       rot(0, 2) * static_cast<float>(eye_ned.z));
        view(1, 3) = -(rot(1, 0) * static_cast<float>(eye_ned.x) +
                       rot(1, 1) * static_cast<float>(eye_ned.y) +
                       rot(1, 2) * static_cast<float>(eye_ned.z));
        view(2, 3) = -(rot(2, 0) * static_cast<float>(eye_ned.x) +
                       rot(2, 1) * static_cast<float>(eye_ned.y) +
                       rot(2, 2) * static_cast<float>(eye_ned.z));

        return view;
    }

    /// @brief Compute OpenGL View Matrix for cockpit-local geometry
    /// Cockpit geometry is defined in body coordinates [m].
    Mat4 compute_cockpit_view_matrix() const noexcept {
        const math::Vector3 eye_b = eye_pos_body();

        // Base view mapping:
        // X_cam = +Y_b
        // Y_cam = -Z_b
        // Z_cam = -X_b
        Mat4 base_view = Mat4::zero();
        base_view(0, 1) =  1.0f;
        base_view(1, 2) = -1.0f;
        base_view(2, 0) = -1.0f;
        base_view(3, 3) =  1.0f;

        base_view(0, 3) = -static_cast<float>(eye_b.y);
        base_view(1, 3) =  static_cast<float>(eye_b.z);
        base_view(2, 3) =  static_cast<float>(eye_b.x);

        // Apply head look orientation
        return head_look_matrix() * base_view;
    }

    /// @brief Optical infinity rotation matrix: converts world directions to camera eye-space rays
    /// Independent of camera translation (zero parallax at infinity)
    Mat4 compute_infinity_view_matrix(const fdm::FlightState& state) const noexcept {
        if (mode_ == CameraMode::CHASE) {
            Mat4 v = compute_chase_view_matrix(state);
            v(0, 3) = 0.0f;
            v(1, 3) = 0.0f;
            v(2, 3) = 0.0f;
            return v;
        }

        const math::Matrix3x3 C_bn = state.q_att.to_dcm_ned_to_body();

        Mat4 base_rot = Mat4::identity();
        base_rot(0, 0) = static_cast<float>(C_bn(1, 0));
        base_rot(0, 1) = static_cast<float>(C_bn(1, 1));
        base_rot(0, 2) = static_cast<float>(C_bn(1, 2));

        base_rot(1, 0) = -static_cast<float>(C_bn(2, 0));
        base_rot(1, 1) = -static_cast<float>(C_bn(2, 1));
        base_rot(1, 2) = -static_cast<float>(C_bn(2, 2));

        base_rot(2, 0) = -static_cast<float>(C_bn(0, 0));
        base_rot(2, 1) = -static_cast<float>(C_bn(0, 1));
        base_rot(2, 2) = -static_cast<float>(C_bn(0, 2));

        return head_look_matrix() * base_rot;
    }
};

} // namespace fastjet::graphics
