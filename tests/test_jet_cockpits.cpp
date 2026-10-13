// Procedural cockpits (F-15EX, Typhoon, F-22A, A-10C, F-35B): geometry that
// has to hold for the pilot's view to be believable. Nothing pokes through
// the canopy glass, nothing sits inside the pilot's head, every display and
// instrument face turns toward the eye, the Typhoon carries its own
// glareshield (MDEF, DWP, Get-U-Home display) and wide-angle HUD, and the
// F-35B its panoramic display and no HUD at all. The F-35B's lift system
// poses its doors and swivels its nozzle as the flight model says.

#include "fastjet/graphics/jet_cockpits.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

using namespace fastjet;
using namespace fastjet::graphics;
using aircraft::AircraftType;
using cockpit_build::EYE_X;
using cockpit_build::EYE_Z;

namespace {

constexpr AircraftType kTypes[5] = {AircraftType::F15EX_EAGLE_II, AircraftType::EUROFIGHTER_TYPHOON,
                                    AircraftType::F22_RAPTOR, AircraftType::A10_THUNDERBOLT,
                                    AircraftType::F35B_LIGHTNING_II};

const char* name(AircraftType t) {
    switch (t) {
        case AircraftType::F15EX_EAGLE_II: return "F-15EX";
        case AircraftType::EUROFIGHTER_TYPHOON: return "Typhoon";
        case AircraftType::F22_RAPTOR: return "F-22A";
        case AircraftType::A10_THUNDERBOLT: return "A-10C";
        case AircraftType::F35B_LIGHTNING_II: return "F-35B";
        default: return "?";
    }
}

/// How far outside the canopy glass a point is [m], negative inside. Only
/// meaningful above the sill line, where the glass is.
double outside_canopy(const CanopyFit& fit, double x, double y, double z) {
    const CanopyStation st = canopy_station(fit, static_cast<float>(x - EYE_X));
    const double sill = EYE_Z + fit.sill_dz;
    const double crown = EYE_Z + st.crown_dz;
    const double u = y / st.span, v = (z - sill) / (crown - sill);
    const double r = std::sqrt(u * u + v * v);
    // Scale the normalised excess back to metres by the smaller semi-axis.
    return (r - 1.0) * std::min(static_cast<double>(st.span), std::abs(crown - sill));
}

/// Shell and combiner stay inside the glass wherever the glass is (above
/// the sill, between the windscreen foot and the canopy's aft end).
void test_inside_canopy() {
    std::cout << "[Test] Cockpits: nothing pokes through the canopy glass... " << std::flush;
    bool ok = true;
    for (AircraftType t : kTypes) {
        const CanopyFit fit = airframe_info(t).canopy;
        const CockpitLayout lay = build_jet_cockpit(t);
        auto check = [&](const std::vector<Vertex3D>& verts, const char* what) {
            double worst = -1.0;
            double wx = 0, wy = 0, wz = 0;
            for (const Vertex3D& v : verts) {
                const double x = v.pos[0], y = v.pos[1], z = v.pos[2];
                const double dx = x - EYE_X;
                if (dx > fit.ws_dx - 0.15 || dx < fit.aft_dx + 0.10) continue;
                if (z > EYE_Z + fit.sill_dz - 0.05) continue; // at or below the sill
                const double out = outside_canopy(fit, x, y, z);
                if (out > worst) { worst = out; wx = x; wy = y; wz = z; }
            }
            if (worst > 0.002) {
                std::printf("\n  %s %s pokes %.1f mm through the canopy at (%.3f, %.3f, %.3f)", name(t), what,
                            worst * 1000.0, wx, wy, wz);
                std::fflush(stdout);
                ok = false;
            }
        };
        check(lay.shell, "shell");
        check(lay.combiner, "HUD combiner");
    }
    if (!ok) std::cout << "\n";
    assert(ok && "cockpit geometry outside the canopy");
    std::cout << "PASS\n";
}

/// Nothing inside the pilot's helmeted head: a 0.135 m sphere centred
/// 0.08 m behind the eye.
void test_head_clearance() {
    std::cout << "[Test] Cockpits: clear of the pilot's head... " << std::flush;
    for (AircraftType t : kTypes) {
        const CockpitLayout lay = build_jet_cockpit(t);
        const double cx = EYE_X - 0.08, cz = EYE_Z - 0.02;
        for (const Vertex3D& v : lay.shell) {
            const double dx = v.pos[0] - cx, dy = v.pos[1], dz = v.pos[2] - cz;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < 0.135) {
                std::printf("\n  %s shell vertex (%.3f, %.3f, %.3f) inside the head\n", name(t), v.pos[0],
                            v.pos[1], v.pos[2]);
                assert(false && "cockpit geometry inside the pilot's head");
            }
        }
    }
    std::cout << "PASS\n";
}

/// Every live display and gauge face lies ahead of the eye, inside the
/// cockpit, turned toward the eye and upright.
void test_faces_toward_eye() {
    std::cout << "[Test] Cockpits: displays and instruments face the pilot... " << std::flush;
    for (AircraftType t : kTypes) {
        const CockpitLayout lay = build_jet_cockpit(t);
        std::vector<ScreenQuad> all = lay.screens;
        all.insert(all.end(), lay.gauges.begin(), lay.gauges.end());
        assert(!all.empty());
        for (const ScreenQuad& q : all) {
            const auto& c = q.corners;
            double cxs = 0, cys = 0, czs = 0;
            for (const auto& p : c) { cxs += p[0] * 0.25; cys += p[1] * 0.25; czs += p[2] * 0.25; }
            assert(cxs > EYE_X + 0.15 && cxs < EYE_X + 0.95 && std::abs(cys) < 0.50);
            // Normal from the corner order (TL, TR, BR, BL as seen).
            const double ux = c[1][0] - c[0][0], uy = c[1][1] - c[0][1], uz = c[1][2] - c[0][2];
            const double wx = c[3][0] - c[0][0], wy = c[3][1] - c[0][1], wz = c[3][2] - c[0][2];
            double nx = wy * uz - wz * uy, ny = wz * ux - wx * uz, nz = wx * uy - wy * ux;
            const double nl = std::sqrt(nx * nx + ny * ny + nz * nz);
            nx /= nl; ny /= nl; nz /= nl;
            double ex = EYE_X - cxs, ey = -cys, ez = EYE_Z - czs;
            const double el = std::sqrt(ex * ex + ey * ey + ez * ez);
            ex /= el; ey /= el; ez /= el;
            assert(nx * ex + ny * ey + nz * ez > 0.5);
            // Reads upright: the top edge is higher, or on a face lying flat
            // on a console, farther from the pilot.
            const double top_z = c[0][2] + c[1][2], bot_z = c[2][2] + c[3][2];
            assert(top_z < bot_z - 0.002 || (std::abs(top_z - bot_z) <= 0.002 && c[0][0] + c[1][0] > c[2][0] + c[3][0]));
        }
    }
    std::cout << "PASS\n";
}

int count(const CockpitLayout& lay, Gauge g) {
    int n = 0;
    const ScreenQuad ref = CockpitGauges::screen(g, {});
    for (const ScreenQuad& q : lay.gauges) n += (q.u0 == ref.u0 && q.v0 == ref.v0) ? 1 : 0;
    return n;
}

/// The Typhoon's own panel: MDEF and DWP on the glareshield either side
/// of the HUD, the Get-U-Home display under the right one, three MHDDs,
/// the three-greens, and no analogue fuel gauge, clock or standby dials.
void test_typhoon_layout() {
    std::cout << "[Test] Typhoon: glareshield panels, MHDDs, no analogue dials... " << std::flush;
    const CockpitLayout lay = build_jet_cockpit(AircraftType::EUROFIGHTER_TYPHOON);
    assert(count(lay, Gauge::MDEF) == 1);
    assert(count(lay, Gauge::DWP) == 1);
    assert(count(lay, Gauge::GUH) == 1);
    assert(count(lay, Gauge::GEAR_LAMP) == 3);
    for (Gauge g : {Gauge::FUEL_QTY, Gauge::CLOCK, Gauge::ADI, Gauge::ASI, Gauge::ALT, Gauge::DED}) {
        assert(count(lay, g) == 0);
    }
    assert(lay.screens.size() == 3);

    auto centre = [](const ScreenQuad& q) {
        double y = 0, z = 0;
        for (const auto& p : q.corners) { y += p[1] * 0.25; z += p[2] * 0.25; }
        return std::pair<double, double>{y, z};
    };
    const ScreenQuad* mdef = nullptr;
    const ScreenQuad* dwp = nullptr;
    const ScreenQuad* guh = nullptr;
    for (const ScreenQuad& q : lay.gauges) {
        if (q.u0 == CockpitGauges::screen(Gauge::MDEF, {}).u0 && q.v0 == CockpitGauges::screen(Gauge::MDEF, {}).v0) mdef = &q;
        if (q.u0 == CockpitGauges::screen(Gauge::DWP, {}).u0 && q.v0 == CockpitGauges::screen(Gauge::DWP, {}).v0) dwp = &q;
        if (q.u0 == CockpitGauges::screen(Gauge::GUH, {}).u0 && q.v0 == CockpitGauges::screen(Gauge::GUH, {}).v0) guh = &q;
    }
    // MDEF left, DWP right, both above every MHDD (on the glareshield).
    double mhdd_top = 1e9;
    for (const ScreenQuad& s : lay.screens) mhdd_top = std::min(mhdd_top, static_cast<double>(s.corners[0][2]));
    assert(centre(*mdef).first < -0.15 && centre(*dwp).first > 0.15);
    assert(mdef->corners[2][2] < mhdd_top && dwp->corners[2][2] < mhdd_top);
    // Get-U-Home: right of the right MHDD, below the DWP.
    assert(centre(*guh).first > 0.30 && centre(*guh).second > centre(*dwp).second);

    // Wide-angle HUD: the combiner spans at least 28 x 19 degrees from the eye.
    double az_min = 1e9, az_max = -1e9, el_min = 1e9, el_max = -1e9;
    for (const Vertex3D& v : lay.combiner) {
        const double dx = v.pos[0] - EYE_X;
        az_min = std::min(az_min, std::atan2(v.pos[1], dx));
        az_max = std::max(az_max, std::atan2(v.pos[1], dx));
        el_min = std::min(el_min, std::atan2(EYE_Z - v.pos[2], dx));
        el_max = std::max(el_max, std::atan2(EYE_Z - v.pos[2], dx));
    }
    const double deg = 180.0 / M_PI;
    std::printf("(HUD %.0f x %.0f deg, %+.1f..%+.1f) ", (az_max - az_min) * deg, (el_max - el_min) * deg, el_min * deg,
                el_max * deg);
    assert((az_max - az_min) * deg >= 28.0);
    assert((el_max - el_min) * deg >= 19.0);
    assert(el_min * deg < -1.0); // the horizon is inside the glass
    std::cout << "PASS\n";
}

/// The F-35B: no HUD (no combiner glass; the helmet carries the
/// symbology), the 20 x 8 in Panoramic Cockpit Display as two 10 x 8 in
/// halves side by side, the standby display under it, side-stick right
/// and one throttle.
void test_f35b_layout() {
    std::cout << "[Test] F-35B: panoramic display, standby display, no HUD... " << std::flush;
    const CockpitLayout lay = build_jet_cockpit(AircraftType::F35B_LIGHTNING_II);
    assert(lay.helmet_display);
    assert(lay.combiner.empty());
    assert(lay.screens.size() == 2);
    assert(count(lay, Gauge::ISFD) == 1);
    for (Gauge g : {Gauge::FUEL_QTY, Gauge::ADI, Gauge::HSI, Gauge::DED, Gauge::GEAR_LAMP, Gauge::MDEF}) {
        assert(count(lay, g) == 0);
    }
    // Together the two halves are 508 x 203 mm, left half on the left.
    double y_min = 1e9, y_max = -1e9, z_min = 1e9, z_max = -1e9;
    for (const ScreenQuad& q : lay.screens) {
        for (const auto& p : q.corners) {
            y_min = std::min(y_min, static_cast<double>(p[1]));
            y_max = std::max(y_max, static_cast<double>(p[1]));
            z_min = std::min(z_min, static_cast<double>(p[2]));
            z_max = std::max(z_max, static_cast<double>(p[2]));
        }
    }
    std::printf("(PCD %.0f x %.0f mm) ", (y_max - y_min) * 1000.0, (z_max - z_min) * 1000.0);
    assert(std::abs((y_max - y_min) - 0.508) < 0.003);
    assert(std::abs((z_max - z_min) - 0.203) < 0.012); // the face is raked: its height in Z is a little less
    assert(lay.screens[0].corners[0][1] < 0.0f && lay.screens[1].corners[1][1] > 0.0f);
    assert(lay.screens[0].u1 <= lay.screens[1].u0 + 1e-6f);
    // The standby display sits under the PCD, on the centreline.
    for (const ScreenQuad& q : lay.gauges) {
        if (q.u0 != CockpitGauges::screen(Gauge::ISFD, {}).u0) continue;
        assert(std::abs(q.corners[0][1] + q.corners[1][1]) < 0.01);
        assert(q.corners[0][2] > z_max);
    }
    std::cout << "PASS\n";
}

/// The F-35B's lift system poses its airframe: everything stowed in CTOL;
/// converted, the LiftFan door stands its forward edge up, the lower doors
/// hang open and the 3BSM points the jet straight down (95 deg: a little
/// forward of vertical), under the tail; the nozzle's yaw swings it sideways.
void test_f35b_lift_system_pose() {
    std::cout << "[Test] F-35B: doors and 3BSM follow the lift system... " << std::flush;
    using airframe_build::V3;
    propulsion::StovlStatus st;
    st.equipped = true;
    const StovlPose stowed = f35b_stovl_pose(st);
    assert(stowed.joints.count == airframe_build::kF35bJointCount);
    for (int j = 0; j < stowed.joints.count; ++j) {
        const Mat4& m = stowed.joints.m[static_cast<size_t>(j)];
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) assert(std::abs(m(r, c) - (r == c ? 1.0f : 0.0f)) < 1e-4f);
        }
    }
    const AirframeInfo info = airframe_info(AircraftType::F35B_LIGHTNING_II);
    assert(info.stovl);
    assert(std::abs(stowed.nozzle_pos.x - info.nozzles[0].x) < 1e-3f && std::abs(stowed.nozzle_dir.x + 1.0f) < 1e-4f);

    st.lift_fan_door = st.aux_inlet_door = st.fan_nozzle_door = st.roll_post_door = 1.0;
    st.nozzle_pitch_deg = 95.0;
    const StovlPose open = f35b_stovl_pose(st);
    // LiftFan door: its forward edge (1.85 m ahead of the hinge) rises ~1.8 m.
    const V3 fwd_edge = airframe_build::transform(open.joints.m[airframe_build::kLiftFanDoor],
                                                  {airframe_build::f35b_layout::kFanDoorFwd, 0.0f, -0.86f});
    assert(fwd_edge.z < -0.86f - 1.6f);
    // Lower doors: their inboard edges swing down below the belly.
    const V3 inboard = airframe_build::transform(open.joints.m[airframe_build::kFanDoorL], {3.5f, -0.05f, 0.69f});
    assert(inboard.z > 0.69f + 0.35f && inboard.y < -0.45f);
    // 3BSM: 95 deg down, its exit below and ahead of the stowed one.
    const double defl = std::acos(std::clamp(-static_cast<double>(open.nozzle_dir.x), -1.0, 1.0)) * 180.0 / M_PI;
    std::printf("(nozzle %.1f deg, exit %.2f m lower) ", defl, open.nozzle_pos.z - stowed.nozzle_pos.z);
    assert(std::abs(defl - 95.0) < 0.5 && std::abs(open.nozzle_dir.y) < 1e-3f && open.nozzle_dir.z > 0.99f);
    assert(open.nozzle_pos.z > stowed.nozzle_pos.z + 0.5f && open.nozzle_pos.x > stowed.nozzle_pos.x);
    // One source of truth: the lift jets leave the airframe where the flight
    // model applies their thrust (within 0.3 m).
    const auto cfg = aircraft::AircraftConfig::get(AircraftType::F35B_LIGHTNING_II).stovl;
    auto near = [](const V3& p, const math::Vector3& q) {
        return std::abs(p.x - q.x) < 0.3 && std::abs(p.y - q.y) < 0.3 && std::abs(p.z - q.z) < 0.3;
    };
    std::printf("(3BSM exit %.2f %.2f %.2f, fan %.2f %.2f %.2f) ", open.nozzle_pos.x, open.nozzle_pos.y, open.nozzle_pos.z,
                open.fan_pos.x, open.fan_pos.y, open.fan_pos.z);
    assert(near(open.nozzle_pos, cfg.nozzle_pos_b));
    assert(near(open.fan_pos, cfg.lift_fan_pos_b));
    assert(near(open.roll_pos[1], cfg.roll_post_pos_b));
    assert(near(open.roll_pos[0], math::Vector3(cfg.roll_post_pos_b.x, -cfg.roll_post_pos_b.y, cfg.roll_post_pos_b.z)));
    // Every intermediate angle is reached with the nozzle in the plane of symmetry.
    for (double d = 5.0; d < 95.0; d += 7.5) {
        st.nozzle_pitch_deg = d;
        const StovlPose p = f35b_stovl_pose(st);
        const double got = std::acos(std::clamp(-static_cast<double>(p.nozzle_dir.x), -1.0, 1.0)) * 180.0 / M_PI;
        assert(std::abs(got - d) < 0.6 && std::abs(p.nozzle_dir.y) < 1e-3f);
    }
    // Yaw vectoring: + swings the jet to the right.
    st.nozzle_pitch_deg = 90.0;
    st.nozzle_yaw_deg = 10.0;
    const StovlPose yawed = f35b_stovl_pose(st);
    assert(yawed.nozzle_dir.y > 0.15f);
    // The other jets have no moving parts.
    assert(airframe_pose(AircraftType::F22_RAPTOR, st).count == 0);
    std::cout << "PASS\n";
}

} // namespace

int main() {
    test_inside_canopy();
    test_head_clearance();
    test_faces_toward_eye();
    test_typhoon_layout();
    test_f35b_layout();
    test_f35b_lift_system_pose();
    std::cout << "All jet cockpit tests passed.\n";
    return 0;
}
