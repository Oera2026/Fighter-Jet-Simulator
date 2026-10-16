#pragma once

#include "fastjet/graphics/jet_airframes.hpp"

#include <optional>
#include <vector>

namespace fastjet::graphics {

/// @brief A jet drawn from a glTF asset, and where the asset sits.
///
/// The assets are one series (bohmerang's "Fighter Jet - Free" models on
/// Sketchfab, the F-16C's included): X forward, Y up, Z along the span, in
/// units of 0.1 m, with the same node names (airframe, canopy, cockpit, hud,
/// landingOn/Off, ...). Their gear stands taller than the flight model's, so
/// each is seated on its main wheels: those on the flight model's main gear
/// station, the lowest tyre on the wheels' contact plane.
struct AirframeAsset {
    const char* path = nullptr;            ///< nullptr: the jet has no asset
    ModelPlacement placement{};
    std::array<NozzleExit, 2> nozzles{};   ///< Exhausts on the asset, body axes (world alignment) [m]
    int nozzle_count = 1;
    std::optional<AssetCutaway> cutaway;   ///< Skin that a moving part replaces
};

namespace asset_build {

inline constexpr float kScale = 0.1f; ///< Model units to metres

/// @brief Placement seating an asset on its main wheels.
/// @param main_x Main wheels' contact station [model units, X forward]
/// @param tyre_y Lowest tyre point, gear down [model units, Y up]
[[nodiscard]] inline ModelPlacement on_gear(aircraft::AircraftType type, float main_x, float tyre_y) {
    const aircraft::LandingGearConfig& g = aircraft::AircraftConfig::get(type).gear;
    ModelPlacement p;
    p.scale = kScale;
    p.world_x = static_cast<float>(g.main_r_pos_b.x) - main_x * kScale;
    p.world_z = airframe_info(type).contact_z + tyre_y * kScale;
    // Never flown from: the cockpit view draws the procedural airframe.
    p.cockpit_x = p.world_x;
    p.cockpit_z = p.world_z;
    return p;
}

/// @brief A nozzle exit measured on an asset [model units] in body axes.
[[nodiscard]] inline NozzleExit nozzle(const ModelPlacement& p, float x, float y, float z, float radius) {
    return NozzleExit{x * kScale + p.world_x, z * kScale, -y * kScale + p.world_z, radius};
}

/// @brief Twin nozzles either side of the centreline.
inline void twin(AirframeAsset& a, float x, float y, float z, float radius) {
    a.nozzles = {nozzle(a.placement, x, y, -z, radius), nozzle(a.placement, x, y, z, radius)};
    a.nozzle_count = 2;
}

} // namespace asset_build

/// @brief The F-35B over its asset (the series' F-35, which has no moving
/// parts): the procedural doors, weapons-bay doors and 3BSM are laid over it,
/// each group shifted from the procedural skin onto the asset's [m].
/// The asset stands deeper than the procedural airframe: spine 0.85 m higher
/// at the LiftFan door, belly 0.35-0.49 m higher, wing 0.70 m higher, and its
/// nozzle 0.54 m higher (5 cm aft).
[[nodiscard]] inline airframe_build::V3 f35b_asset_offset(int joint) noexcept {
    using namespace airframe_build;
    switch (joint) {
        case kLiftFanDoor:
        case kAuxDoors:  return {0.0f, 0.0f, -0.85f};
        case kFanDoorL:
        case kFanDoorR:
        case kVaneBox:   return {0.0f, 0.0f, -0.49f};
        case kRollDoorL:
        case kRollDoorR: return {0.0f, 0.0f, -0.70f};
        case kBsmDoorL:
        case kBsmDoorR:  return {0.0f, 0.0f, -0.26f};
        case kBsmA:
        case kBsmB:
        case kBsmC:      return {0.055f, 0.0f, -0.538f};
        default:         return {0.0f, 0.0f, -0.35f}; // weapons-bay doors
    }
}

/// @brief The F-35B's moving parts for its asset, shifted onto the asset's
/// skin. Each shows only while it moves; the closed asset skin stands for it
/// at rest. The 3BSM shows as a whole once it swivels, when the asset's own
/// fixed nozzle (the cutaway) gives way to it. The vane box sits inside the
/// asset's closed belly and is left out.
[[nodiscard]] inline std::vector<ProceduralPart> f35b_asset_parts() {
    using namespace airframe_build;
    ModelPlacement unused;
    std::vector<ProceduralPart> out;
    for (ProceduralPart& part : build_airframe(aircraft::AircraftType::F35B_LIGHTNING_II, unused)) {
        if (part.joint < 0 || part.joint == kVaneBox) continue;
        const airframe_build::V3 d = f35b_asset_offset(part.joint);
        for (size_t v = 0; v + 5 < part.vertices.size(); v += 6) {
            part.vertices[v] += d.x;
            part.vertices[v + 1] += d.y;
            part.vertices[v + 2] += d.z;
        }
        part.shown_by = part.joint >= kBsmA && part.joint <= kBsmC ? kBsmC : part.joint;
        out.push_back(std::move(part));
    }
    return out;
}

/// @brief f35b_stovl_pose() for the parts of f35b_asset_parts(): every joint
/// carried along its group's shift, and the lift jets' exits with it.
[[nodiscard]] inline StovlPose f35b_asset_pose(const propulsion::StovlStatus& st, const WeaponBayDoors& bay = {}) noexcept {
    using namespace airframe_build;
    StovlPose pose = f35b_stovl_pose(st, bay);
    for (int j = 0; j < kF35bJointCount; ++j) {
        const V3 d = f35b_asset_offset(j);
        auto& m = pose.joints.m[static_cast<size_t>(j)];
        m = Mat4::translate(d.x, d.y, d.z) * m * Mat4::translate(-d.x, -d.y, -d.z);
    }
    pose.nozzle_pos = pose.nozzle_pos + f35b_asset_offset(kBsmC);
    pose.fan_pos = pose.fan_pos + f35b_asset_offset(kVaneBox);
    for (V3& p : pose.roll_pos) p = p + f35b_asset_offset(kRollDoorL);
    return pose;
}

/// @brief The asset @p type is drawn from outside the cockpit, if any. The
/// A-10C has none and keeps its procedural model; the F-16C's asset is the
/// renderer's own (ModelGLB's defaults) and is not listed here.
[[nodiscard]] inline AirframeAsset airframe_asset(aircraft::AircraftType type) {
    using aircraft::AircraftType;
    using namespace asset_build;
    AirframeAsset a;
    switch (type) {
        case AircraftType::F15EX_EAGLE_II:
            // The F-15E Strike Eagle: the F-15EX's airframe.
            a.path = "assets/models/f15e.glb";
            a.placement = on_gear(type, 23.41f, -24.11f);
            twin(a, -36.81f, 0.0f, 6.63f, 0.35f);
            break;
        case AircraftType::EUROFIGHTER_TYPHOON:
            a.path = "assets/models/typhoon.glb";
            a.placement = on_gear(type, 11.67f, -24.10f);
            twin(a, -41.66f, -0.45f, 4.90f, 0.31f);
            break;
        case AircraftType::F22_RAPTOR:
            // Two-dimensional nozzles: the plume leaves the middle of each.
            a.path = "assets/models/f22.glb";
            a.placement = on_gear(type, 18.40f, -20.52f);
            twin(a, -33.0f, 0.0f, 6.40f, 0.40f);
            break;
        case AircraftType::F35B_LIGHTNING_II: {
            a.path = "assets/models/f35.glb";
            a.placement = on_gear(type, 1.13f, -26.47f);
            a.nozzles[0] = nozzle(a.placement, -40.62f, -1.59f, 0.0f, 0.40f);
            a.nozzle_count = 1;
            // The fixed nozzle, aft of where the asset's fuselage ends.
            const NozzleExit& n = a.nozzles[0];
            a.cutaway = AssetCutaway{-4.10f, n.y, n.z, 0.75f, airframe_build::kBsmC};
            break;
        }
        case AircraftType::F16_FIGHTING_FALCON:
        case AircraftType::A10_THUNDERBOLT:
            break;
    }
    return a;
}

} // namespace fastjet::graphics
