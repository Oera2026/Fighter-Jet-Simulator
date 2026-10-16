// The airframe assets (assets/models/*.glb): each one parses, its images
// decode and its nodes classify into the parts the renderer draws; each
// asset jet sits on the flight model's gear, its exhausts leave its nozzles,
// and the F-35B's moving parts fit the asset they are laid over.

#define CGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#include "fastjet/graphics/jet_assets.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace fastjet;
using namespace fastjet::graphics;
using aircraft::AircraftType;

namespace {

/// One mesh vertex in body axes [m] (world alignment) and its part.
struct BodyVertex {
    double x, y, z;
    F16PartType part;
};

/// Parses @p path (also from build/ and build/tests/), checks every image
/// decodes, and returns its vertices placed by @p placement.
std::vector<BodyVertex> load_asset(const std::string& path, const ModelPlacement& placement) {
    cgltf_options options = {};
    cgltf_data* data = nullptr;
    std::string actual;
    cgltf_result result = cgltf_result_file_not_found;
    for (const std::string& p : {path, "../" + path, "../../" + path}) {
        result = cgltf_parse_file(&options, p.c_str(), &data);
        if (result == cgltf_result_success) {
            actual = p;
            break;
        }
    }
    if (result != cgltf_result_success) {
        std::cerr << "Failed to parse " << path << ": error " << result << "\n";
        assert(false);
    }
    result = cgltf_load_buffers(&options, data, actual.c_str());
    assert(result == cgltf_result_success);

    for (cgltf_size i = 0; i < data->images_count; ++i) {
        const cgltf_image& img = data->images[i];
        assert(img.buffer_view);
        const auto* bytes = static_cast<const uint8_t*>(img.buffer_view->buffer->data) + img.buffer_view->offset;
        int w = 0, h = 0, comp = 0;
        unsigned char* pixels = stbi_load_from_memory(bytes, static_cast<int>(img.buffer_view->size), &w, &h, &comp, 4);
        if (!pixels) std::cerr << path << " image " << i << ": " << stbi_failure_reason() << "\n";
        assert(pixels && w > 0 && h > 0);
        stbi_image_free(pixels);
    }

    std::vector<BodyVertex> out;
    for (cgltf_size ni = 0; ni < data->nodes_count; ++ni) {
        const cgltf_node& node = data->nodes[ni];
        if (!node.mesh) continue;
        std::string name = node.name ? node.name : "";
        if (node.parent && node.parent->name) name += " " + std::string(node.parent->name);
        const F16PartType part = ModelGLB::classify_node(name);
        float m[16];
        cgltf_node_transform_world(&node, m);
        for (cgltf_size pi = 0; pi < node.mesh->primitives_count; ++pi) {
            const cgltf_primitive& prim = node.mesh->primitives[pi];
            for (cgltf_size ai = 0; ai < prim.attributes_count; ++ai) {
                if (prim.attributes[ai].type != cgltf_attribute_type_position) continue;
                const cgltf_accessor* acc = prim.attributes[ai].data;
                std::vector<float> pos(acc->count * 3);
                cgltf_accessor_unpack_floats(acc, pos.data(), pos.size());
                for (size_t v = 0; v < acc->count; ++v) {
                    const float px = pos[v * 3], py = pos[v * 3 + 1], pz = pos[v * 3 + 2];
                    const float tx = m[0] * px + m[4] * py + m[8] * pz + m[12];
                    const float ty = m[1] * px + m[5] * py + m[9] * pz + m[13];
                    const float tz = m[2] * px + m[6] * py + m[10] * pz + m[14];
                    // Model (X fwd, Y up, Z span) to body (X fwd, Y right, Z down).
                    out.push_back({tx * placement.scale + placement.world_x, tz * placement.scale,
                                   -ty * placement.scale + placement.world_z, part});
                }
            }
        }
    }
    cgltf_free(data);
    return out;
}

void test_node_names() {
    // The series spells its node names in either case.
    assert(ModelGLB::classify_node("Object_16 F-16-landingOn_6") == F16PartType::GEAR_DOWN);
    assert(ModelGLB::classify_node("Object_16 LandingOn_6") == F16PartType::GEAR_DOWN);
    assert(ModelGLB::classify_node("Object_14 LandingOff_5") == F16PartType::GEAR_UP);
    assert(ModelGLB::classify_node("Object_18 F-35-landingOnLight_7") == F16PartType::LIGHTS);
    assert(ModelGLB::classify_node("Object_18 LandingLight_7") == F16PartType::LIGHTS);
    assert(ModelGLB::classify_node("Object_6 Canopy_1") == F16PartType::CANOPY);
    assert(ModelGLB::classify_node("Object_8 Cockpit_2") == F16PartType::COCKPIT);
    assert(ModelGLB::classify_node("Object_10 HUD_3") == F16PartType::HUD);
    assert(ModelGLB::classify_node("Object_20 WeaponRails_8") == F16PartType::RAILS);
    assert(ModelGLB::classify_node("Object_12 F-35-instrGlass-sensor_4") == F16PartType::AIRFRAME);
    std::cout << "  node names: OK\n";
}

void test_f16_asset() {
    const std::vector<BodyVertex> v = load_asset("assets/models/f16.glb", ModelPlacement{});
    assert(!v.empty());
    std::cout << "  F-16C asset: " << v.size() << " vertices, images decode\n";
}

/// An asset jet: its parts, its stance on the gear, its size, its nozzles.
void test_asset(AircraftType type, double real_length_m) {
    const AirframeAsset asset = airframe_asset(type);
    assert(asset.path);
    const std::vector<BodyVertex> v = load_asset(asset.path, asset.placement);
    std::map<F16PartType, int> count;
    for (const BodyVertex& b : v) ++count[b.part];
    for (F16PartType p : {F16PartType::AIRFRAME, F16PartType::CANOPY, F16PartType::COCKPIT, F16PartType::GEAR_UP,
                          F16PartType::GEAR_DOWN, F16PartType::LIGHTS}) {
        assert(count[p] > 0);
    }

    // Wheels: the lowest tyre on the contact plane, the main wheels on the
    // flight model's main gear station and no more than 10 cm above the plane.
    const AirframeInfo info = airframe_info(type);
    const aircraft::LandingGearConfig& gear = aircraft::AircraftConfig::get(type).gear;
    double lowest = -1e9, main_low = -1e9;
    for (const BodyVertex& b : v) {
        if (b.part != F16PartType::GEAR_DOWN) continue;
        lowest = std::max(lowest, b.z);
        if (std::fabs(b.y) > 1.0) main_low = std::max(main_low, b.z);
    }
    double main_x = 0.0;
    int main_n = 0;
    for (const BodyVertex& b : v) {
        if (b.part == F16PartType::GEAR_DOWN && std::fabs(b.y) > 1.0 && b.z > main_low - 0.03) {
            main_x += b.x;
            ++main_n;
        }
    }
    main_x /= std::max(main_n, 1);
    std::printf("  %s: lowest tyre %.3f m (contact %.3f), main wheels %.3f m above it at x %.2f (gear %.2f)\n",
                std::string(aircraft::to_string(type)).c_str(), lowest, info.contact_z, lowest - main_low, main_x, gear.main_r_pos_b.x);
    assert(std::fabs(lowest - info.contact_z) < 0.01);
    assert(lowest - main_low < 0.10);
    assert(std::fabs(main_x - gear.main_r_pos_b.x) < 0.10);

    // Size: 0.1 m model units give the real jet's length.
    double x_min = 1e9, x_max = -1e9;
    for (const BodyVertex& b : v) {
        if (b.part != F16PartType::AIRFRAME) continue;
        x_min = std::min(x_min, b.x);
        x_max = std::max(x_max, b.x);
    }
    const double length = x_max - x_min;
    std::printf("    length %.2f m (real %.2f m)\n", length, real_length_m);
    assert(std::fabs(length / real_length_m - 1.0) < 0.02);

    // Exhausts leave from inside a nozzle: within 0.3 m of each exit's plane
    // there is skin all round its axis, above, below and either side.
    for (int n = 0; n < asset.nozzle_count; ++n) {
        const NozzleExit& e = asset.nozzles[static_cast<size_t>(n)];
        bool up = false, down = false, left = false, right = false;
        for (const BodyVertex& b : v) {
            if (b.part != F16PartType::AIRFRAME || std::fabs(b.x - e.x) > 0.3) continue;
            const double dy = b.y - e.y, dz = b.z - e.z;
            if (std::hypot(dy, dz) > 2.0 * e.radius) continue;
            up |= dz < -0.5 * e.radius;
            down |= dz > 0.5 * e.radius;
            left |= dy < -0.5 * e.radius;
            right |= dy > 0.5 * e.radius;
        }
        std::printf("    nozzle %d at (%.2f, %.2f, %.2f): ring %s\n", n, e.x, e.y, e.z,
                    up && down && left && right ? "closed" : "OPEN");
        assert(up && down && left && right);
    }
}

/// The F-35B's procedural moving parts over its asset.
void test_f35b_fit() {
    using namespace airframe_build;
    // Every part shows only once it moves; the vane box stays inside.
    const std::vector<ProceduralPart> parts = f35b_asset_parts();
    assert(!parts.empty());
    for (const ProceduralPart& p : parts) {
        assert(p.joint >= 0 && p.joint != kVaneBox);
        assert(p.shown_by == (p.joint >= kBsmA && p.joint <= kBsmC ? static_cast<int>(kBsmC) : p.joint));
    }

    // At rest every joint is the identity: nothing is laid over the asset.
    const StovlPose rest = f35b_asset_pose(propulsion::StovlStatus{});
    for (int j = 0; j < kF35bJointCount; ++j) {
        const Mat4& m = rest.joints.m[static_cast<size_t>(j)];
        const Mat4 id = Mat4::identity();
        for (size_t e = 0; e < 16; ++e) assert(std::fabs(m.m[e] - id.m[e]) < 1e-5f);
    }
    // ... and the 3BSM's exit is the asset's own nozzle exit.
    const NozzleExit asset_nozzle = airframe_asset(AircraftType::F35B_LIGHTNING_II).nozzles[0];
    std::printf("  F-35B 3BSM exit (%.3f, %.3f, %.3f), asset nozzle (%.3f, %.3f, %.3f)\n", rest.nozzle_pos.x,
                rest.nozzle_pos.y, rest.nozzle_pos.z, asset_nozzle.x, asset_nozzle.y, asset_nozzle.z);
    assert(std::fabs(rest.nozzle_pos.x - asset_nozzle.x) < 0.06);
    assert(std::fabs(rest.nozzle_pos.y - asset_nozzle.y) < 0.01);
    assert(std::fabs(rest.nozzle_pos.z - asset_nozzle.z) < 0.03);

    // Converted: the doors swing about their hinges where the shifted parts
    // now are, and the lift jets leave from the shifted exits.
    propulsion::StovlStatus st;
    st.equipped = true;
    st.mode = propulsion::StovlMode::STOVL;
    st.lift_fan_door = st.aux_inlet_door = st.fan_nozzle_door = st.roll_post_door = 1.0;
    st.nozzle_pitch_deg = 90.0;
    const StovlPose plain = f35b_stovl_pose(st);
    const StovlPose fitted = f35b_asset_pose(st);
    const F35bRig& r = f35b_rig();
    const V3 d = f35b_asset_offset(kLiftFanDoor);
    const V3 hinge = transform(fitted.joints.m[kLiftFanDoor], r.fan_door_hinge + d);
    assert(std::fabs(hinge.x - (r.fan_door_hinge + d).x) < 1e-4f);
    assert(std::fabs(hinge.z - (r.fan_door_hinge + d).z) < 1e-4f);
    const V3 dn = f35b_asset_offset(kBsmC);
    assert(std::fabs(fitted.nozzle_pos.z - (plain.nozzle_pos.z + dn.z)) < 1e-4f);
    assert(fitted.nozzle_dir.z > 0.9f); // still blowing down
    const V3 dr = f35b_asset_offset(kRollDoorL);
    assert(std::fabs(fitted.roll_pos[0].z - (plain.roll_pos[0].z + dr.z)) < 1e-6f);
    std::cout << "  F-35B moving parts fit the asset: OK\n";
}

} // namespace

int main() {
    std::cout << "Airframe assets\n";
    test_node_names();
    test_f16_asset();
    test_asset(AircraftType::F15EX_EAGLE_II, 19.43);      // F-15E
    test_asset(AircraftType::EUROFIGHTER_TYPHOON, 15.96);
    test_asset(AircraftType::F22_RAPTOR, 18.92);
    test_asset(AircraftType::F35B_LIGHTNING_II, 15.61);
    assert(!airframe_asset(AircraftType::A10_THUNDERBOLT).path);
    test_f35b_fit();
    std::cout << "All verification passed!\n";
    return 0;
}
