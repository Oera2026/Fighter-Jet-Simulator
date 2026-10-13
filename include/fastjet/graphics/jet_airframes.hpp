#pragma once

#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/graphics/model_glb.hpp"
#include "fastjet/propulsion/stovl_status.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace fastjet::graphics {

/// @brief The canopy and cockpit opening, relative to the pilot's eye
/// [m, body axes: +dx forward, +dz down]. Shared by the airframe (its glass
/// and the hole in its skin) and the cockpit shell (its frames and sills),
/// so the two meet.
struct CanopyFit {
    float ws_dx = 1.05f;      ///< Windscreen foot on the coaming, ahead of the eye
    float coaming_dz = 0.22f; ///< Coaming below the eye
    float bow_dx = 0.35f;     ///< Windscreen arch: the full canopy section starts here
    float full_dx = -0.55f;   ///< Full section ends (aft of the eye)
    float aft_dx = -1.95f;    ///< Canopy fairs into the spine here
    float aft_dz = 0.08f;     ///< Spine top at the canopy's end, below the eye
    float crown_dz = -0.42f;  ///< Crown above the eye (negative: up)
    float sill_dz = 0.25f;    ///< Sill line below the eye
    float half_span = 0.56f;  ///< Canopy half-width at the sill
    float tub_aft_dx = -0.75f;///< Cockpit's aft bulkhead (the skin is open from here to the windscreen)
    bool frameless = false;   ///< One-piece canopy (F-22): no windscreen arch
};

/// @brief One engine exhaust in body axes (world alignment) [m].
struct NozzleExit {
    float x, y, z, radius;
};

/// @brief What the renderer needs to know of an airframe besides its mesh.
struct AirframeInfo {
    float eye_x = 1.80f;          ///< Design eye point in the airframe's body axes [m]
    float eye_z = -0.65f;
    float contact_z = 1.56f;      ///< Wheels' contact plane below the CG on the ground [m]
    CanopyFit canopy{};
    std::array<NozzleExit, 2> nozzles{};
    int nozzle_count = 1;
    bool afterburner = true;      ///< Reheat plume (the A-10's turbofans have none)
    bool stovl = false;           ///< Lift system: doors, LiftFan, roll posts and a swivelling nozzle (F-35B)
};

namespace airframe_build {

struct V3 {
    float x, y, z;
};
inline V3 operator+(V3 a, V3 b) noexcept { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) noexcept { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float k) noexcept { return {a.x * k, a.y * k, a.z * k}; }
inline V3 cross(V3 a, V3 b) noexcept { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float dot(V3 a, V3 b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 normalize(V3 a) noexcept {
    const float l = std::sqrt(dot(a, a));
    return l > 1e-9f ? a * (1.0f / l) : V3{0.0f, 0.0f, -1.0f};
}
inline V3 mirror(V3 a) noexcept { return {a.x, -a.y, a.z}; }

/// @brief Triangle soup with normals, into one ProceduralPart.
class Mesh {
public:
    explicit Mesh(ProceduralPart& part) : v_(part.vertices) {}

    void vertex(V3 p, V3 n) {
        v_.insert(v_.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
    }
    /// Flat-shaded triangle (the shader turns faces toward the eye).
    void tri(V3 a, V3 b, V3 c) {
        const V3 n = normalize(cross(b - a, c - a));
        vertex(a, n);
        vertex(b, n);
        vertex(c, n);
    }
    void quad(V3 a, V3 b, V3 c, V3 d) {
        tri(a, b, c);
        tri(a, c, d);
    }
    void tri_smooth(V3 a, V3 na, V3 b, V3 nb, V3 c, V3 nc) {
        vertex(a, na);
        vertex(b, nb);
        vertex(c, nc);
    }

private:
    std::vector<float>& v_;
};

/// @brief A fuselage (or intake, or nacelle) cross-section: a superellipse,
/// boxier as the exponents grow (2 = ellipse).
struct Section {
    float x;
    float half_w;
    float z_top, z_bot;
    float n_top = 2.2f, n_bot = 2.2f;
    float y_c = 0.0f;
};

[[nodiscard]] inline V3 ring_point(const Section& s, float th) noexcept {
    const float sn = std::sin(th), cs = std::cos(th);
    const float zc = 0.5f * (s.z_top + s.z_bot);
    const float n = cs >= 0.0f ? s.n_top : s.n_bot;
    const float e = 2.0f / n;
    const float y = s.y_c + s.half_w * (sn < 0.0f ? -1.0f : 1.0f) * std::pow(std::fabs(sn), e);
    const float z = cs >= 0.0f ? zc - (zc - s.z_top) * std::pow(cs, e) : zc + (s.z_bot - zc) * std::pow(-cs, e);
    return {s.x, y, z};
}

/// @brief The skin is left open over the cockpit (or a door): between x0
/// and x1, the upper (or, below, the lower) faces within half_w of the
/// section's centre are not built. Sections must stand at x0 and x1.
struct Cut {
    float x0 = 0.0f, x1 = 0.0f, half_w = 0.0f;
    bool below = false;
    [[nodiscard]] bool active() const noexcept { return x1 > x0; }
};

/// @brief Lofts the sections (nose first) into a smooth skin, open where
/// any of @p cuts says.
inline void loft(Mesh& m, const std::vector<Section>& secs, int sides, std::initializer_list<Cut> cuts) {
    const int ns = static_cast<int>(secs.size());
    if (ns < 2) return;
    std::vector<std::vector<V3>> P(static_cast<size_t>(ns), std::vector<V3>(static_cast<size_t>(sides)));
    for (int i = 0; i < ns; ++i) {
        for (int k = 0; k < sides; ++k) {
            const float th = 6.2831853f * static_cast<float>(k) / static_cast<float>(sides);
            P[static_cast<size_t>(i)][static_cast<size_t>(k)] = ring_point(secs[static_cast<size_t>(i)], th);
        }
    }
    auto at = [&](int i, int k) -> V3 {
        i = std::clamp(i, 0, ns - 1);
        k = (k % sides + sides) % sides;
        return P[static_cast<size_t>(i)][static_cast<size_t>(k)];
    };
    auto normal = [&](int i, int k) -> V3 {
        const V3 along = at(i + 1, k) - at(i - 1, k);
        const V3 around = at(i, k + 1) - at(i, k - 1);
        V3 n = normalize(cross(around, along));
        const Section& s = secs[static_cast<size_t>(i)];
        const V3 centre{s.x, s.y_c, 0.5f * (s.z_top + s.z_bot)};
        if (dot(n, at(i, k) - centre) < 0.0f) n = n * -1.0f;
        return n;
    };
    for (int i = 0; i + 1 < ns; ++i) {
        const Section& a = secs[static_cast<size_t>(i)];
        const Section& b = secs[static_cast<size_t>(i + 1)];
        const float zc = 0.5f * (a.z_top + a.z_bot);
        for (int k = 0; k < sides; ++k) {
            const V3 p00 = at(i, k), p01 = at(i, k + 1), p10 = at(i + 1, k), p11 = at(i + 1, k + 1);
            bool open = false;
            for (const Cut& cut : cuts) {
                if (!cut.active() || std::min(a.x, b.x) < cut.x0 - 1e-4f || std::max(a.x, b.x) > cut.x1 + 1e-4f) continue;
                const bool side = cut.below ? (p00.z > zc && p01.z > zc) : (p00.z < zc && p01.z < zc);
                open = open || (side && std::fabs(p00.y - a.y_c) < cut.half_w && std::fabs(p01.y - a.y_c) < cut.half_w);
            }
            if (open) continue;
            const V3 n00 = normal(i, k), n01 = normal(i, k + 1), n10 = normal(i + 1, k), n11 = normal(i + 1, k + 1);
            m.tri_smooth(p00, n00, p10, n10, p11, n11);
            m.tri_smooth(p00, n00, p11, n11, p01, n01);
        }
    }
}

inline void loft(Mesh& m, const std::vector<Section>& secs, int sides = 28, const Cut& cut = {}) {
    loft(m, secs, sides, {cut});
}

/// @brief Closes a section with a flat fan (intake mouths, nozzle faces).
inline void cap(Mesh& m, const Section& s, int sides = 28) {
    const V3 c{s.x, s.y_c, 0.5f * (s.z_top + s.z_bot)};
    for (int k = 0; k < sides; ++k) {
        const float t0 = 6.2831853f * static_cast<float>(k) / static_cast<float>(sides);
        const float t1 = 6.2831853f * static_cast<float>(k + 1) / static_cast<float>(sides);
        m.tri(c, ring_point(s, t0), ring_point(s, t1));
    }
}

/// @brief A lifting surface with a diamond section, thickest at 40 % chord.
/// Points go root leading edge, tip leading edge, tip trailing edge, root
/// trailing edge.
inline void surface(Mesh& m, V3 rle, V3 tle, V3 tte, V3 rte, float t_root, float t_tip) {
    const V3 up = normalize(cross(tle - rle, rte - rle));
    const V3 r_mid = rle + (rte - rle) * 0.4f;
    const V3 t_mid = tle + (tte - tle) * 0.4f;
    const V3 rt = r_mid + up * (0.5f * t_root), rb = r_mid - up * (0.5f * t_root);
    const V3 tt = t_mid + up * (0.5f * t_tip), tb = t_mid - up * (0.5f * t_tip);
    m.quad(rle, tle, tt, rt);
    m.quad(rt, tt, tte, rte);
    m.quad(rle, rb, tb, tle);
    m.quad(rb, rte, tte, tb);
    // Tip and root faces.
    m.tri(tle, tb, tt);
    m.tri(tt, tb, tte);
    m.tri(rle, rt, rb);
    m.tri(rt, rte, rb);
}

/// @brief A surface and its mirror image across the centreline.
inline void surface_pair(Mesh& m, V3 rle, V3 tle, V3 tte, V3 rte, float t_root, float t_tip) {
    surface(m, rle, tle, tte, rte, t_root, t_tip);
    surface(m, mirror(rle), mirror(tle), mirror(tte), mirror(rte), t_root, t_tip);
}

/// @brief A round tube from p0 to p1, radius r0 to r1, smooth sides and
/// closed ends.
inline void tube(Mesh& m, V3 p0, V3 p1, float r0, float r1, int sides = 16, bool caps = true) {
    const V3 d = normalize(p1 - p0);
    const V3 ref = std::fabs(d.z) > 0.9f ? V3{1.0f, 0.0f, 0.0f} : V3{0.0f, 0.0f, -1.0f};
    const V3 s = normalize(cross(d, ref));
    const V3 u = cross(s, d);
    auto dir = [&](int k) {
        const float a = 6.2831853f * static_cast<float>(k) / static_cast<float>(sides);
        return s * std::cos(a) + u * std::sin(a);
    };
    for (int k = 0; k < sides; ++k) {
        const V3 n0 = dir(k), n1 = dir(k + 1);
        const V3 a = p0 + n0 * r0, b = p1 + n0 * r1, c = p1 + n1 * r1, e = p0 + n1 * r0;
        m.tri_smooth(a, n0, b, n0, c, n1);
        m.tri_smooth(a, n0, c, n1, e, n1);
        if (caps) {
            m.tri(p0, e, a);
            m.tri(p1, b, c);
        }
    }
}

/// @brief Axis-aligned box.
inline void box(Mesh& m, V3 c, V3 h) {
    const float x0 = c.x - h.x, x1 = c.x + h.x, y0 = c.y - h.y, y1 = c.y + h.y, z0 = c.z - h.z, z1 = c.z + h.z;
    const V3 p[8] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                     {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
    m.quad(p[0], p[1], p[2], p[3]);
    m.quad(p[4], p[7], p[6], p[5]);
    m.quad(p[0], p[4], p[5], p[1]);
    m.quad(p[1], p[5], p[6], p[2]);
    m.quad(p[2], p[6], p[7], p[3]);
    m.quad(p[3], p[7], p[4], p[0]);
}

} // namespace airframe_build

/// @brief The canopy's cross-section @p dx ahead of the eye: crown height
/// (relative to the eye) and half-span at the sill.
struct CanopyStation {
    float crown_dz;
    float span;
};
[[nodiscard]] inline CanopyStation canopy_station(const CanopyFit& c, float dx) noexcept {
    if (dx > c.bow_dx) {
        // Windscreen: a smooth rise from the coaming to the arch.
        const float f = std::clamp((c.ws_dx - dx) / (c.ws_dx - c.bow_dx), 0.0f, 1.0f);
        const float k = std::sin(f * 1.5707963f);
        return {c.coaming_dz + k * (c.crown_dz - c.coaming_dz), c.half_span * (0.78f + 0.22f * k)};
    }
    if (dx < c.full_dx) {
        // Aft fairing down to the spine.
        const float f = std::clamp((c.full_dx - dx) / (c.full_dx - c.aft_dx), 0.0f, 1.0f);
        const float k = 1.0f - f * f;
        return {c.aft_dz + k * (c.crown_dz - c.aft_dz), c.half_span * (0.70f + 0.30f * k)};
    }
    return {c.crown_dz, c.half_span};
}

/// @brief A point on the canopy at station @p dx, @p a radians round from
/// the crown (beyond +-pi/2 it tucks under the sill line), relative to the
/// eye at (eye_x, eye_z). @p inset pulls it inside the glass (frames).
[[nodiscard]] inline std::array<float, 3> canopy_point(const CanopyFit& c, float eye_x, float eye_z, float dx, float a,
                                                       float inset = 0.0f) noexcept {
    const CanopyStation st = canopy_station(c, dx);
    const float sill = eye_z + c.sill_dz;
    const float crown = eye_z + st.crown_dz + inset;
    return {eye_x + dx, (st.span - inset) * std::sin(a), sill + (crown - sill) * std::cos(a)};
}

namespace airframe_build {

/// @brief The canopy glass: the windscreen rising from the coaming to the
/// arch, the full section over the pilot, then fairing down into the spine.
inline void canopy_glass(Mesh& m, const CanopyFit& c, float eye_x, float eye_z) {
    constexpr int kStations = 18;
    constexpr int kArc = 18;
    constexpr float kMaxAngle = 1.75f; // just past the sill line, so it tucks under the skin
    std::vector<std::vector<V3>> P(kStations + 1, std::vector<V3>(kArc + 1));
    for (int i = 0; i <= kStations; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(kStations);
        const float dx = c.ws_dx + t * (c.aft_dx - c.ws_dx);
        for (int j = 0; j <= kArc; ++j) {
            const float a = -kMaxAngle + 2.0f * kMaxAngle * static_cast<float>(j) / static_cast<float>(kArc);
            const auto q = canopy_point(c, eye_x, eye_z, dx, a);
            P[static_cast<size_t>(i)][static_cast<size_t>(j)] = {q[0], q[1], q[2]};
        }
    }
    for (int i = 0; i < kStations; ++i) {
        for (int j = 0; j < kArc; ++j) {
            const V3 a = P[static_cast<size_t>(i)][static_cast<size_t>(j)];
            const V3 b = P[static_cast<size_t>(i + 1)][static_cast<size_t>(j)];
            const V3 d = P[static_cast<size_t>(i + 1)][static_cast<size_t>(j + 1)];
            const V3 e = P[static_cast<size_t>(i)][static_cast<size_t>(j + 1)];
            m.quad(a, b, d, e);
        }
    }
}

/// @brief Paint and metal finishes.
inline ProceduralPart paint(float r, float g, float b, float roughness = 0.55f) {
    ProceduralPart p;
    p.part_type = F16PartType::AIRFRAME;
    p.base = {r, g, b, 1.0f};
    p.roughness = roughness;
    return p;
}
inline ProceduralPart metal(float shade) {
    ProceduralPart p;
    p.part_type = F16PartType::AIRFRAME;
    p.base = {shade, shade * 0.97f, shade * 0.93f, 1.0f};
    p.metallic = 0.8f;
    p.roughness = 0.42f;
    return p;
}
inline ProceduralPart glass(float r, float g, float b) {
    ProceduralPart p;
    p.part_type = F16PartType::CANOPY;
    p.base = {r, g, b, 0.3f};
    p.roughness = 0.05f;
    p.transparent = true;
    return p;
}

/// @brief All parts of one airframe.
struct Parts {
    ProceduralPart skin;
    ProceduralPart dark = paint(0.012f, 0.012f, 0.013f, 0.8f);   ///< Intake mouths, nozzle interiors
    ProceduralPart nozzle = metal(0.16f);
    ProceduralPart canopy = glass(0.92f, 0.90f, 0.82f);
    ProceduralPart gear = paint(0.42f, 0.42f, 0.42f, 0.5f);
    ProceduralPart tyres = paint(0.018f, 0.018f, 0.018f, 0.9f);
    /// Seen through the canopy from outside only: the cockpit view draws
    /// its own (jet_cockpits.hpp) in their place.
    ProceduralPart interior = paint(0.035f, 0.036f, 0.040f, 0.8f);
    ProceduralPart pilot = paint(0.30f, 0.30f, 0.28f, 0.5f);

    explicit Parts(ProceduralPart s) : skin(std::move(s)) {
        gear.part_type = F16PartType::GEAR_DOWN;
        tyres.part_type = F16PartType::GEAR_DOWN;
        interior.part_type = F16PartType::COCKPIT;
        pilot.part_type = F16PartType::COCKPIT;
    }
    [[nodiscard]] std::vector<ProceduralPart> list() && {
        return {std::move(skin),   std::move(dark), std::move(nozzle),   std::move(canopy),
                std::move(gear),   std::move(tyres), std::move(interior), std::move(pilot)};
    }
};

/// @brief A low-poly sphere (helmets).
inline void sphere(Mesh& m, V3 c, float r) {
    std::vector<Section> secs;
    for (int i = 0; i <= 8; ++i) {
        const float phi = 3.14159265f * static_cast<float>(i) / 8.0f;
        const float rr = std::max(r * std::sin(phi), 0.002f);
        secs.push_back(Section{c.x + r * std::cos(phi), rr, c.z - rr, c.z + rr, 2.0f, 2.0f, c.y});
    }
    loft(m, secs, 14);
}

/// @brief The cockpit as seen from outside: the tub under the canopy, the
/// panel and glare shield, the seat and the pilot's helmet and shoulders.
inline void outside_cockpit(Parts& p, const AirframeInfo& a) {
    const CanopyFit& c = a.canopy;
    Mesh m(p.interior);
    const float x0 = a.eye_x + c.tub_aft_dx, x1 = a.eye_x + c.ws_dx - 0.25f;
    const float hw = c.half_span - 0.03f;
    const float sill = a.eye_z + c.sill_dz, floor = a.eye_z + 0.80f;
    m.quad({x0, -hw, floor}, {x1, -hw, floor}, {x1, hw, floor}, {x0, hw, floor});
    for (float side : {-1.0f, 1.0f}) {
        m.quad({x0, side * hw, sill}, {x1, side * hw, sill}, {x1, side * hw, floor}, {x0, side * hw, floor});
    }
    m.quad({x0, -hw, sill - 0.05f}, {x0, hw, sill - 0.05f}, {x0, hw, floor}, {x0, -hw, floor});
    // Panel and glare shield.
    const float px = a.eye_x + 0.70f;
    m.quad({px, -hw, a.eye_z + 0.12f}, {px, hw, a.eye_z + 0.12f}, {px - 0.10f, hw, floor}, {px - 0.10f, -hw, floor});
    m.quad({px - 0.04f, -hw, a.eye_z + 0.08f}, {px - 0.04f, hw, a.eye_z + 0.08f},
           {a.eye_x + c.ws_dx, hw * 0.8f, a.eye_z + c.coaming_dz}, {a.eye_x + c.ws_dx, -hw * 0.8f, a.eye_z + c.coaming_dz});
    // The turtledeck under the canopy aft of the seat: dark, riding just
    // above the skin's crest (which runs straight from the sill at the
    // tub's aft end up to the canopy's aft end), seen through the glass
    // from the seat and from outside.
    {
        Mesh deck(p.dark);
        const float xa = a.eye_x + c.aft_dx, za = a.eye_z + c.aft_dz;
        const float hw_d = c.half_span * 0.72f;
        loft(deck, {{x0, hw_d, sill - 0.010f, sill + 0.25f, 4.0f, 2.5f}, {xa + 0.02f, hw_d, za - 0.003f, za + 0.25f, 4.0f, 2.5f}},
             24);
    }
    // Seat headbox, helmet and shoulders.
    box(m, {a.eye_x - 0.30f, 0.0f, a.eye_z - 0.02f}, {0.08f, 0.16f, 0.22f});
    box(m, {a.eye_x - 0.25f, 0.0f, a.eye_z + 0.40f}, {0.10f, 0.24f, 0.30f});
    Mesh pilot(p.pilot);
    sphere(pilot, {a.eye_x - 0.06f, 0.0f, a.eye_z - 0.03f}, 0.14f);
    box(pilot, {a.eye_x - 0.12f, 0.0f, a.eye_z + 0.28f}, {0.10f, 0.21f, 0.12f});
}

/// @brief A round nozzle: a short tapering can, dark inside.
inline void round_nozzle(Parts& p, const NozzleExit& n, float length) {
    Mesh metal_mesh(p.nozzle);
    tube(metal_mesh, {n.x + length, n.y, n.z}, {n.x, n.y, n.z}, n.radius * 1.08f, n.radius, 18, false);
    Mesh dark(p.dark);
    cap(dark, Section{n.x + 0.06f, n.radius * 0.96f, n.z - n.radius * 0.96f, n.z + n.radius * 0.96f, 2.0f, 2.0f, n.y}, 18);
}

/// @brief Main and nose gear, from the flight model's strut stations.
inline void gear(Parts& p, float nose_x, float nose_y, float main_x, float main_y, float attach_z, float contact_z,
                 float nose_r, float main_r) {
    Mesh strut(p.gear);
    Mesh tyre(p.tyres);
    const float axle_n = contact_z - nose_r, axle_m = contact_z - main_r;
    tube(strut, {nose_x, nose_y, attach_z}, {nose_x, nose_y, axle_n}, 0.07f, 0.06f, 10);
    tube(tyre, {nose_x, nose_y - 0.09f, axle_n}, {nose_x, nose_y + 0.09f, axle_n}, nose_r, nose_r, 18);
    for (float side : {-1.0f, 1.0f}) {
        tube(strut, {main_x, side * main_y, attach_z}, {main_x, side * main_y, axle_m}, 0.09f, 0.08f, 10);
        tube(tyre, {main_x, side * main_y - 0.14f, axle_m}, {main_x, side * main_y + 0.14f, axle_m}, main_r, main_r, 18);
    }
}

// ---------------------------------------------------------------------------
// The airframes. Body axes, metres, CG at the origin, wheels on the contact
// plane: the world alignment needs no offset.
// ---------------------------------------------------------------------------

[[nodiscard]] inline AirframeInfo f15ex_info() {
    AirframeInfo a;
    a.eye_x = 5.30f;
    a.eye_z = -0.92f;
    a.contact_z = 1.83f;
    a.canopy = CanopyFit{};
    a.nozzles = {NozzleExit{-9.70f, -0.58f, 0.10f, 0.50f}, NozzleExit{-9.70f, 0.58f, 0.10f, 0.50f}};
    a.nozzle_count = 2;
    return a;
}

/// F-15EX Eagle II: long nose, bubble canopy, the big rectangular intakes
/// either side of the cockpit, the high shoulder wing on top of them, twin
/// vertical fins on booms outboard of the engines, stabilators below them.
[[nodiscard]] inline std::vector<ProceduralPart> f15ex(const AirframeInfo& a) {
    Parts p(paint(0.20f, 0.215f, 0.235f));
    Mesh s(p.skin);
    const CanopyFit& c = a.canopy;
    const float ws = a.eye_x + c.ws_dx, tub = a.eye_x + c.tub_aft_dx, aft = a.eye_x + c.aft_dx;
    loft(s,
         {{9.70f, 0.03f, -0.36f, -0.32f},
          {9.20f, 0.24f, -0.46f, -0.18f, 2.0f, 2.0f},
          {8.40f, 0.42f, -0.56f, -0.02f, 2.2f, 2.0f},
          {7.40f, 0.55f, -0.66f, 0.14f, 2.5f, 2.2f},
          {ws, 0.63f, a.eye_z + c.coaming_dz, 0.25f, 3.0f, 2.4f},
          {5.70f, 0.66f, a.eye_z + c.sill_dz, 0.32f, 4.0f, 2.5f},
          {tub, 0.68f, a.eye_z + c.sill_dz, 0.36f, 4.0f, 2.5f},
          {aft, 0.70f, a.eye_z + c.aft_dz, 0.42f, 3.5f, 3.0f},
          {2.00f, 0.80f, -0.78f, 0.60f, 3.5f, 3.5f},
          {0.00f, 1.10f, -0.66f, 0.75f, 4.0f, 4.0f},
          {-4.00f, 1.15f, -0.58f, 0.75f, 4.0f, 4.0f},
          {-8.40f, 1.05f, -0.52f, 0.62f, 4.0f, 4.0f},
          {-9.20f, 0.95f, -0.40f, 0.50f, 4.0f, 4.0f}},
         32, Cut{tub, ws, c.half_span + 0.01f});
    // Intakes, the wing riding on their tops, booms, fins and stabilators.
    for (float side : {-1.0f, 1.0f}) {
        const std::vector<Section> intake = {
            {5.45f, 0.40f, -0.64f, 0.55f, 8.0f, 8.0f, side * 1.15f},
            {4.20f, 0.44f, -0.62f, 0.66f, 8.0f, 8.0f, side * 1.17f},
            {2.00f, 0.45f, -0.58f, 0.72f, 8.0f, 8.0f, side * 1.17f},
            {-1.00f, 0.42f, -0.56f, 0.72f, 8.0f, 8.0f, side * 1.10f},
            {-3.00f, 0.20f, -0.50f, 0.60f, 8.0f, 8.0f, side * 0.95f}};
        loft(s, intake, 24);
        Mesh mouth(p.dark);
        cap(mouth, Section{5.43f, 0.36f, -0.60f, 0.51f, 8.0f, 8.0f, side * 1.15f}, 24);
        tube(s, {-4.6f, side * 1.42f, -0.32f}, {-9.75f, side * 1.42f, -0.32f}, 0.22f, 0.16f, 12);
    }
    surface_pair(s, {0.90f, 1.55f, -0.55f}, {-3.95f, 6.52f, -0.47f}, {-5.55f, 6.52f, -0.47f}, {-4.70f, 1.55f, -0.55f}, 0.30f, 0.08f);
    surface_pair(s, {-6.30f, 1.40f, -0.25f}, {-8.20f, 4.30f, -0.25f}, {-9.30f, 4.30f, -0.25f}, {-9.40f, 1.40f, -0.25f}, 0.18f, 0.06f);
    surface_pair(s, {-5.70f, 1.42f, -0.50f}, {-7.90f, 1.42f, -3.75f}, {-9.00f, 1.42f, -3.75f}, {-9.25f, 1.42f, -0.50f}, 0.22f, 0.06f);
    outside_cockpit(p, a);
    Mesh glass_mesh(p.canopy);
    canopy_glass(glass_mesh, c, a.eye_x, a.eye_z);
    for (int i = 0; i < a.nozzle_count; ++i) round_nozzle(p, a.nozzles[static_cast<size_t>(i)], 0.55f);
    gear(p, 4.80f, 0.0f, -0.70f, 1.38f, 0.40f, a.contact_z, 0.30f, 0.42f);
    return std::move(p).list();
}

[[nodiscard]] inline AirframeInfo typhoon_info() {
    AirframeInfo a;
    a.eye_x = 4.45f;
    a.eye_z = -0.88f;
    a.contact_z = 1.68f;
    a.canopy.ws_dx = 1.00f;
    a.canopy.bow_dx = 0.30f;
    a.canopy.crown_dz = -0.45f; // a tall bubble: the wide-angle combiner fits under the windscreen
    a.canopy.full_dx = -0.50f;
    a.canopy.aft_dx = -1.60f;
    a.canopy.aft_dz = 0.05f;
    a.canopy.half_span = 0.55f;
    a.nozzles = {NozzleExit{-7.75f, -0.42f, 0.12f, 0.43f}, NozzleExit{-7.75f, 0.42f, 0.12f, 0.43f}};
    a.nozzle_count = 2;
    return a;
}

/// Eurofighter Typhoon: slim nose, the chin intake under the cockpit, the
/// canards just aft of the canopy's front, the big cranked delta, a single
/// fin and two close-set nozzles.
[[nodiscard]] inline std::vector<ProceduralPart> typhoon(const AirframeInfo& a) {
    // RAF Camouflage Grey (BS381C 626), a touch lighter than the F-16's top grey.
    Parts p(paint(0.200f, 0.208f, 0.222f, 0.60f));
    Mesh s(p.skin);
    const CanopyFit& c = a.canopy;
    const float ws = a.eye_x + c.ws_dx, tub = a.eye_x + c.tub_aft_dx, aft = a.eye_x + c.aft_dx;
    loft(s,
         {{8.30f, 0.03f, -0.36f, -0.32f},
          {7.60f, 0.28f, -0.50f, -0.12f, 2.0f, 2.0f},
          {6.60f, 0.46f, -0.61f, 0.06f, 2.2f, 2.0f},
          {ws, 0.57f, a.eye_z + c.coaming_dz, 0.20f, 3.0f, 2.2f},
          {4.80f, 0.62f, a.eye_z + c.sill_dz, 0.28f, 4.0f, 2.5f},
          {tub, 0.64f, a.eye_z + c.sill_dz, 0.32f, 4.0f, 2.5f},
          {aft, 0.66f, a.eye_z + c.aft_dz, 0.38f, 3.0f, 3.0f},
          {1.50f, 0.72f, -0.78f, 0.45f, 3.0f, 3.0f},
          {-1.00f, 0.80f, -0.70f, 0.45f, 3.5f, 3.0f},
          {-5.00f, 0.80f, -0.62f, 0.42f, 3.5f, 3.0f},
          {-7.20f, 0.82f, -0.50f, 0.38f, 3.0f, 3.0f}},
         32, Cut{tub, ws, c.half_span + 0.01f});
    // Chin intake.
    loft(s,
         {{2.90f, 0.70f, 0.25f, 1.00f, 3.0f, 5.0f},
          {0.50f, 0.75f, 0.30f, 1.00f, 3.0f, 5.0f},
          {-2.50f, 0.72f, 0.30f, 0.85f, 3.0f, 4.0f},
          {-4.80f, 0.50f, 0.30f, 0.50f, 3.0f, 3.0f}},
         24);
    Mesh mouth(p.dark);
    cap(mouth, Section{2.88f, 0.64f, 0.32f, 0.96f, 3.0f, 5.0f}, 24);
    // Canards, the delta, the fin.
    surface_pair(s, {4.60f, 0.55f, -0.05f}, {3.50f, 2.15f, 0.00f}, {3.10f, 2.15f, 0.00f}, {2.95f, 0.55f, -0.05f}, 0.10f, 0.04f);
    surface_pair(s, {1.00f, 0.75f, 0.25f}, {-5.26f, 5.47f, 0.15f}, {-6.26f, 5.47f, 0.15f}, {-7.00f, 0.75f, 0.25f}, 0.36f, 0.06f);
    surface(s, {-3.20f, 0.0f, -0.62f}, {-6.30f, 0.0f, -3.65f}, {-7.30f, 0.0f, -3.65f}, {-7.45f, 0.0f, -0.55f}, 0.24f, 0.06f);
    outside_cockpit(p, a);
    Mesh glass_mesh(p.canopy);
    canopy_glass(glass_mesh, c, a.eye_x, a.eye_z);
    for (int i = 0; i < a.nozzle_count; ++i) round_nozzle(p, a.nozzles[static_cast<size_t>(i)], 0.55f);
    gear(p, 3.80f, 0.0f, -0.60f, 1.40f, 0.35f, a.contact_z, 0.28f, 0.38f);
    return std::move(p).list();
}

[[nodiscard]] inline AirframeInfo f22_info() {
    AirframeInfo a;
    a.eye_x = 4.70f;
    a.eye_z = -0.90f;
    a.contact_z = 1.93f;
    a.canopy.ws_dx = 1.45f;
    a.canopy.coaming_dz = 0.20f;
    a.canopy.bow_dx = 0.45f;
    a.canopy.full_dx = -0.40f;
    a.canopy.aft_dx = -2.10f;
    a.canopy.aft_dz = 0.02f;
    a.canopy.crown_dz = -0.46f;
    a.canopy.half_span = 0.58f;
    a.canopy.frameless = true;
    a.nozzles = {NozzleExit{-8.80f, -0.62f, 0.08f, 0.40f}, NozzleExit{-8.80f, 0.62f, 0.08f, 0.40f}};
    a.nozzle_count = 2;
    return a;
}

/// F-22A Raptor: chined, faceted nose, a frameless bubble, caret intakes
/// beside the cockpit, the diamond wing and stabilators, canted fins and
/// flat two-dimensional nozzles.
[[nodiscard]] inline std::vector<ProceduralPart> f22(const AirframeInfo& a) {
    Parts p(paint(0.165f, 0.172f, 0.180f, 0.42f));
    p.canopy = glass(0.95f, 0.82f, 0.50f); // gold-film canopy
    Mesh s(p.skin);
    const CanopyFit& c = a.canopy;
    const float ws = a.eye_x + c.ws_dx, tub = a.eye_x + c.tub_aft_dx, aft = a.eye_x + c.aft_dx;
    loft(s,
         {{8.90f, 0.03f, -0.36f, -0.34f, 1.6f, 1.6f},
          {8.20f, 0.42f, -0.50f, -0.16f, 1.6f, 1.6f},
          {7.20f, 0.62f, -0.61f, 0.04f, 1.7f, 1.7f},
          {ws, 0.72f, a.eye_z + c.coaming_dz, 0.18f, 2.4f, 1.8f},
          {5.40f, 0.76f, a.eye_z + c.sill_dz, 0.28f, 3.5f, 2.0f},
          {tub, 0.78f, a.eye_z + c.sill_dz, 0.34f, 3.5f, 2.2f},
          {aft, 0.82f, a.eye_z + c.aft_dz, 0.40f, 2.6f, 2.4f},
          {1.00f, 1.10f, -0.80f, 0.55f, 2.6f, 2.6f},
          {-2.00f, 1.35f, -0.70f, 0.60f, 3.0f, 3.0f},
          {-6.00f, 1.30f, -0.62f, 0.55f, 3.5f, 3.0f},
          {-8.40f, 1.05f, -0.45f, 0.40f, 4.0f, 4.0f}},
         32, Cut{tub, ws, c.half_span + 0.01f});
    for (float side : {-1.0f, 1.0f}) {
        loft(s,
             {{3.30f, 0.38f, -0.45f, 0.60f, 6.0f, 6.0f, side * 1.18f},
              {1.00f, 0.40f, -0.42f, 0.62f, 6.0f, 6.0f, side * 1.20f},
              {-1.50f, 0.30f, -0.42f, 0.55f, 6.0f, 6.0f, side * 1.15f},
              {-3.00f, 0.10f, -0.40f, 0.40f, 6.0f, 6.0f, side * 1.05f}},
             20);
        Mesh mouth(p.dark);
        cap(mouth, Section{3.28f, 0.34f, -0.41f, 0.56f, 6.0f, 6.0f, side * 1.18f}, 20);
        // Flat nozzle: a box tapering into the slot.
        const NozzleExit& n = a.nozzles[side < 0.0f ? 0 : 1];
        Mesh nz(p.nozzle);
        box(nz, {n.x + 0.30f, n.y, n.z}, {0.30f, 0.44f, 0.28f});
        Mesh slot(p.dark);
        slot.quad({n.x - 0.005f, n.y - 0.38f, n.z - 0.20f}, {n.x - 0.005f, n.y + 0.38f, n.z - 0.20f},
                  {n.x - 0.005f, n.y + 0.38f, n.z + 0.20f}, {n.x - 0.005f, n.y - 0.38f, n.z + 0.20f});
    }
    surface_pair(s, {1.60f, 1.45f, 0.00f}, {-3.10f, 6.78f, 0.06f}, {-4.20f, 6.78f, 0.06f}, {-5.80f, 1.45f, 0.00f}, 0.30f, 0.06f);
    surface_pair(s, {-5.70f, 1.35f, 0.05f}, {-7.70f, 4.40f, 0.05f}, {-9.00f, 4.40f, 0.05f}, {-9.20f, 1.35f, 0.05f}, 0.16f, 0.05f);
    surface_pair(s, {-4.50f, 1.30f, -0.62f}, {-6.60f, 2.43f, -2.74f}, {-7.90f, 2.43f, -2.74f}, {-7.80f, 1.30f, -0.62f}, 0.22f, 0.06f);
    outside_cockpit(p, a);
    Mesh glass_mesh(p.canopy);
    canopy_glass(glass_mesh, c, a.eye_x, a.eye_z);
    gear(p, 4.60f, 0.0f, -0.80f, 1.70f, 0.45f, a.contact_z, 0.30f, 0.42f);
    return std::move(p).list();
}

[[nodiscard]] inline AirframeInfo a10c_info() {
    AirframeInfo a;
    a.eye_x = 4.10f;
    a.eye_z = -0.85f;
    a.contact_z = 1.73f;
    a.canopy.ws_dx = 0.95f;
    a.canopy.coaming_dz = 0.21f;
    a.canopy.bow_dx = 0.25f;
    a.canopy.full_dx = -0.55f;
    a.canopy.aft_dx = -1.35f;
    a.canopy.aft_dz = 0.05f;
    a.nozzles = {NozzleExit{-6.05f, -1.35f, -1.45f, 0.44f}, NozzleExit{-6.05f, 1.35f, -1.45f, 0.44f}};
    a.nozzle_count = 2;
    a.afterburner = false;
    return a;
}

/// A-10C Thunderbolt II: blunt nose with the GAU-8's muzzle, the straight
/// low wing with its gear pods, two TF34 pods high on the rear fuselage,
/// and the twin fins on the tips of the tailplane.
[[nodiscard]] inline std::vector<ProceduralPart> a10c(const AirframeInfo& a) {
    Parts p(paint(0.105f, 0.112f, 0.118f, 0.62f));
    Mesh s(p.skin);
    const CanopyFit& c = a.canopy;
    const float ws = a.eye_x + c.ws_dx, tub = a.eye_x + c.tub_aft_dx, aft = a.eye_x + c.aft_dx;
    loft(s,
         {{7.40f, 0.10f, -0.26f, 0.05f},
          {7.00f, 0.45f, -0.48f, 0.30f, 2.2f, 2.2f},
          {6.10f, 0.62f, -0.60f, 0.55f, 2.5f, 2.5f},
          {ws, 0.70f, a.eye_z + c.coaming_dz, 0.65f, 3.2f, 2.6f},
          {4.40f, 0.72f, a.eye_z + c.sill_dz, 0.70f, 4.0f, 2.8f},
          {tub, 0.74f, a.eye_z + c.sill_dz, 0.72f, 4.0f, 2.8f},
          {aft, 0.76f, a.eye_z + c.aft_dz, 0.75f, 3.0f, 3.0f},
          {0.00f, 0.78f, -0.72f, 0.80f, 3.0f, 3.0f},
          {-3.00f, 0.70f, -0.62f, 0.65f, 3.0f, 3.0f},
          {-6.00f, 0.45f, -0.45f, 0.35f, 2.5f, 2.5f},
          {-8.40f, 0.18f, -0.30f, 0.05f, 2.2f, 2.2f}},
         32, Cut{tub, ws, c.half_span + 0.01f});
    {
        Mesh gun(p.nozzle);
        tube(gun, {7.30f, 0.0f, 0.12f}, {8.10f, 0.0f, 0.12f}, 0.11f, 0.10f, 12);
    }
    // Straight wing: flat centre section, outer panels with dihedral; gear pods under it.
    surface_pair(s, {0.60f, 0.70f, 0.65f}, {0.55f, 3.00f, 0.65f}, {-2.35f, 3.00f, 0.65f}, {-2.40f, 0.70f, 0.65f}, 0.45f, 0.42f);
    surface_pair(s, {0.55f, 3.00f, 0.65f}, {0.25f, 8.77f, -0.04f}, {-1.25f, 8.77f, -0.04f}, {-2.35f, 3.00f, 0.65f}, 0.42f, 0.16f);
    for (float side : {-1.0f, 1.0f}) {
        tube(s, {1.30f, side * 2.62f, 0.92f}, {-1.70f, side * 2.62f, 0.92f}, 0.34f, 0.22f, 14);
        // Engine nacelle on its pylon.
        const NozzleExit& n = a.nozzles[side < 0.0f ? 0 : 1];
        tube(s, {-2.55f, n.y, n.z}, {-5.60f, n.y, n.z}, 0.63f, 0.52f, 20, false);
        Mesh fan(p.dark);
        cap(fan, Section{-2.58f, 0.58f, n.z - 0.58f, n.z + 0.58f, 2.0f, 2.0f, n.y}, 20);
        surface(s, {-3.00f, side * 0.45f, -0.56f}, {-3.20f, side * 1.10f, -0.95f}, {-4.90f, side * 1.10f, -0.95f},
                {-4.90f, side * 0.45f, -0.56f}, 0.12f, 0.10f);
        round_nozzle(p, n, 0.45f);
    }
    // Tailplane and the twin fins on its tips.
    surface_pair(s, {-6.35f, 0.20f, -0.32f}, {-6.55f, 2.75f, -0.32f}, {-8.10f, 2.75f, -0.32f}, {-8.25f, 0.20f, -0.32f}, 0.18f, 0.10f);
    for (float side : {-1.0f, 1.0f}) {
        surface(s, {-6.75f, side * 2.75f, 0.35f}, {-7.05f, side * 2.75f, -2.15f}, {-8.35f, side * 2.75f, -2.15f},
                {-8.45f, side * 2.75f, 0.35f}, 0.16f, 0.08f);
    }
    outside_cockpit(p, a);
    Mesh glass_mesh(p.canopy);
    canopy_glass(glass_mesh, c, a.eye_x, a.eye_z);
    gear(p, 4.20f, 0.30f, -0.40f, 2.62f, 0.35f, a.contact_z, 0.30f, 0.45f);
    return std::move(p).list();
}

// ---------------------------------------------------------------------------
// F-35B Lightning II. Layout from docs/f35b-reference.md (sections 1 and 8);
// the lift posts and gear stand where the flight model's configuration
// (aircraft::AircraftConfig, one source of truth) puts them: the LiftFan's
// exit, the three-bearing swivel module (3BSM) placed so its exit at full
// travel is the configured nozzle position, the roll posts, the wheels.
// ---------------------------------------------------------------------------

/// Lift-system layout [m, body axes] and door travel [rad].
namespace f35b_layout {
inline constexpr aircraft::AircraftConfig kConfig = aircraft::AircraftConfig::get(aircraft::AircraftType::F35B_LIGHTNING_II);
inline constexpr aircraft::StovlConfig kCfg = kConfig.stovl;
inline constexpr float kFanX = static_cast<float>(kCfg.lift_fan_pos_b.x);
// The upper door (1.85 x 1.4 m) reaches forward to the canopy, its hinge aft
// of the fan; the vane box and the lower doors are centred under the fan.
inline constexpr float kFanDoorFwd = kFanX + 0.60f, kFanDoorAft = kFanX - 1.25f, kFanDoorHalfW = 0.70f;
inline constexpr float kAuxFwd = kFanDoorAft - 0.05f, kAuxAft = kFanDoorAft - 0.45f, kAuxHalfW = 0.60f; // two 0.6 x 0.4 m doors
inline constexpr float kFanExitFwd = kFanX + 0.50f, kFanExitAft = kFanX - 0.70f, kFanExitHalfW = 0.55f;
inline constexpr float kRollX = static_cast<float>(kCfg.roll_post_pos_b.x), kRollY = static_cast<float>(kCfg.roll_post_pos_b.y);
inline constexpr float kRollHalfL = 0.25f, kRollHalfW = 0.125f;
inline constexpr float kBsmExitR = 0.50f, kBsmRing = 0.58f;
inline constexpr float kBsmZ = -0.05f;                                      // engine axis
inline constexpr float kBsmL1 = 0.50f, kBsmL2 = 0.60f, kBsmL3 = 0.40f;     // front, middle, rear+nozzle segments
/// Bearing cant: the two canted bearings each tilt by this, so a half turn
/// of the middle segment against the rear one swings the nozzle 4x as far.
inline constexpr float kBsmCant = 0.41452f;   // 23.75 deg: 95 deg at full travel
// At full travel the rear bearing's centre lies L2 cos(2 cant) aft of the
// middle one's, and the nozzle points 5 deg forward of straight down: the
// middle bearing goes where that lands the exit on the configured position.
inline constexpr float kBsm2 = static_cast<float>(kCfg.nozzle_pos_b.x) + kBsmL2 * 0.67559f - kBsmL3 * 0.08716f;
inline constexpr float kBsm1 = kBsm2 + kBsmL1, kBsm3 = kBsm2 - kBsmL2, kBsmExit = kBsm3 - kBsmL3;
inline constexpr float kBsmDoorFwd = kBsm1 + 0.55f, kBsmDoorHalfW = 0.42f;
inline constexpr float kFanDoorOpen = 1.31f;  // 75 deg, forward edge up
inline constexpr float kAuxOpen = 0.70f;      // 40 deg
inline constexpr float kSideDoorOpen = 1.57f; // 90 deg: lower fan, roll-post and 3BSM doors
inline constexpr float kVaneZ = 0.22f;        // Louvre pivots
// Wing: trapezoid, LE 34.1 deg, TE -13 deg, centreline root 6.43 m, tip 1.57 m.
inline constexpr float kWingZ = 0.08f, kWingRootY = 1.20f, kWingTipY = 5.35f;
inline constexpr float kWingLe0 = 2.70f, kWingTe0 = -3.73f, kWingTanLe = 0.677f, kWingTanTe = 0.231f;
inline constexpr float kWingTRoot = 0.27f, kWingTTip = 0.08f;
}  // namespace f35b_layout

[[nodiscard]] inline AirframeInfo f35b_info() {
    using namespace f35b_layout;
    AirframeInfo a;
    a.eye_x = kFanDoorFwd + 1.90f; // the canopy fairs into the LiftFan door
    a.eye_z = -0.75f;
    a.contact_z = 1.90f;
    a.canopy.ws_dx = 1.25f;
    a.canopy.coaming_dz = 0.20f;
    a.canopy.bow_dx = 0.35f;
    a.canopy.full_dx = -0.45f;
    a.canopy.aft_dx = -1.85f; // faired into the LiftFan door's hump
    a.canopy.aft_dz = -0.10f;
    a.canopy.crown_dz = -0.46f;
    a.canopy.half_span = 0.58f;
    a.canopy.frameless = true; // no windscreen bow: one transparency, its arch is internal (behind the headrest)
    a.nozzles = {NozzleExit{kBsmExit, 0.0f, kBsmZ, kBsmExitR}, NozzleExit{}};
    a.nozzle_count = 1;
    a.stovl = true;
    return a;
}

/// The F-35B's moving parts: one joint each, in JointPose order.
enum F35bJoint : int {
    kLiftFanDoor,  ///< Upper LiftFan inlet door, hinged at its aft edge
    kAuxDoors,     ///< The pair of auxiliary engine inlet doors behind it, hinged aft
    kFanDoorL,     ///< Lower LiftFan doors, hinged at their outboard edges
    kFanDoorR,
    kRollDoorL,    ///< Roll-post doors under the wing roots
    kRollDoorR,
    kBsmDoorL,     ///< 3BSM clearance doors under the aft fuselage
    kBsmDoorR,
    kVaneBox,      ///< LiftFan exit louvres (a shear: every vane turns about its own pivot)
    kBsmA,         ///< 3BSM front segment: rolls about the duct axis
    kBsmB,         ///< Middle segment, on the first canted bearing
    kBsmC,         ///< Rear segment and nozzle, on the second
    kF35bJointCount
};
static_assert(kF35bJointCount <= JointPose::kMax);


/// @brief The fuselage sections, nose first. Stations stand at every door's
/// edges, where the skin is cut open under it.
[[nodiscard]] inline const std::vector<Section>& f35b_body() {
    using namespace f35b_layout;
    static const std::vector<Section> kBody = [] {
        const AirframeInfo a = f35b_info();
        const CanopyFit& c = a.canopy;
        const float sill = a.eye_z + c.sill_dz;
        return std::vector<Section>{
            {9.10f, 0.03f, -0.17f, -0.11f, 1.6f, 1.6f}, // chined, faceted radome
            {8.55f, 0.34f, -0.40f, 0.13f, 1.6f, 1.7f},
            {7.85f, 0.60f, -0.57f, 0.32f, 1.8f, 2.0f},
            {a.eye_x + c.ws_dx, 0.74f, a.eye_z + c.coaming_dz, 0.44f, 2.4f, 2.4f},
            {a.eye_x + 0.50f, 0.80f, sill, 0.52f, 3.5f, 3.0f},
            {a.eye_x + c.tub_aft_dx, 0.88f, sill, 0.60f, 3.5f, 3.0f},
            {a.eye_x + c.aft_dx, 0.97f, a.eye_z + c.aft_dz, 0.66f, 3.0f, 4.0f},
            {kFanDoorFwd, 0.99f, -0.86f, 0.66f, 3.0f, 4.0f},   // the LiftFan hump
            {kFanExitFwd, 1.04f, -0.87f, 0.68f, 3.0f, 4.0f},
            {kFanExitAft, 1.18f, -0.86f, 0.70f, 3.0f, 4.0f},
            {kFanDoorAft, 1.22f, -0.84f, 0.70f, 3.0f, 4.0f},
            {kAuxFwd, 1.22f, -0.84f, 0.70f, 3.0f, 4.0f},
            {kAuxAft, 1.25f, -0.81f, 0.70f, 3.0f, 4.0f},
            {-0.60f, 1.30f, -0.70f, 0.66f, 3.0f, 4.0f},
            {-1.80f, 1.20f, -0.64f, 0.60f, 3.0f, 3.5f},
            {kBsmDoorFwd, 1.00f, -0.60f, 0.56f, 2.8f, 3.0f},
            {kBsm1, kBsmRing, kBsmZ - kBsmRing, kBsmZ + kBsmRing, 2.0f, 2.0f}, // round: the 3BSM's front bearing
        };
    }();
    return kBody;
}

/// @brief Height of a loft's skin at (x, y): its top (or bottom) surface,
/// the sections interpolated in x. Facets lie inside it by their sag.
[[nodiscard]] inline float skin_z(const std::vector<Section>& secs, float x, float y, bool top) noexcept {
    size_t i = 0;
    while (i + 2 < secs.size() && secs[i + 1].x > x) ++i;
    const Section& a = secs[i];
    const Section& b = secs[i + 1];
    const float t = std::clamp((a.x - x) / (a.x - b.x), 0.0f, 1.0f);
    auto mix = [t](float p, float q) { return p + t * (q - p); };
    const float zt = mix(a.z_top, b.z_top), zb = mix(a.z_bot, b.z_bot);
    const float zc = 0.5f * (zt + zb);
    const float n = top ? mix(a.n_top, b.n_top) : mix(a.n_bot, b.n_bot);
    const float u = std::clamp(std::fabs(y - mix(a.y_c, b.y_c)) / mix(a.half_w, b.half_w), 0.0f, 1.0f);
    const float k = std::pow(1.0f - std::pow(u, n), 1.0f / n);
    return top ? zc - (zc - zt) * k : zc + (zb - zc) * k;
}

/// @brief Height of the F-35B wing's lower surface at (x, y) (its diamond
/// section, thickest at 40 % chord).
[[nodiscard]] inline float f35b_wing_lower_z(float x, float y) noexcept {
    using namespace f35b_layout;
    const float ay = std::fabs(y);
    const float le = kWingLe0 - ay * kWingTanLe, te = kWingTe0 + ay * kWingTanTe;
    const float s = std::clamp((ay - kWingRootY) / (kWingTipY - kWingRootY), 0.0f, 1.0f);
    const float thick = kWingTRoot + s * (kWingTTip - kWingTRoot);
    const float f = std::clamp((le - x) / (le - te), 0.0f, 1.0f);
    return kWingZ + 0.5f * thick * (f < 0.4f ? f / 0.4f : (1.0f - f) / 0.6f);
}

/// @brief A door lying on the skin, @p proud off it, from x0 to x1 and y0 to
/// y1: its outside in @p outer, the inside face (@p thick in) in @p inner.
template <class SurfaceZ>
inline void door_panel(Mesh& outer, Mesh& inner, float x0, float x1, float y0, float y1, float proud, float thick,
                       SurfaceZ&& surface_z, bool top) {
    constexpr int N = 6;
    const float out = top ? -1.0f : 1.0f; // away from the fuselage
    auto at = [&](int i, int j, float off) {
        const float x = x0 + (x1 - x0) * static_cast<float>(i) / N;
        const float y = y0 + (y1 - y0) * static_cast<float>(j) / N;
        return V3{x, y, surface_z(x, y) + out * off};
    };
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            outer.quad(at(i, j, proud), at(i + 1, j, proud), at(i + 1, j + 1, proud), at(i, j + 1, proud));
            inner.quad(at(i, j, proud - thick), at(i, j + 1, proud - thick), at(i + 1, j + 1, proud - thick),
                       at(i + 1, j, proud - thick));
        }
    }
    // Edges: the door's thickness all round.
    for (int k = 0; k < N; ++k) {
        outer.quad(at(k, 0, proud), at(k + 1, 0, proud), at(k + 1, 0, proud - thick), at(k, 0, proud - thick));
        outer.quad(at(k, N, proud), at(k, N, proud - thick), at(k + 1, N, proud - thick), at(k + 1, N, proud));
        outer.quad(at(0, k, proud), at(0, k, proud - thick), at(0, k + 1, proud - thick), at(0, k + 1, proud));
        outer.quad(at(N, k, proud), at(N, k + 1, proud), at(N, k + 1, proud - thick), at(N, k, proud - thick));
    }
}

/// @brief The bay under a door: four walls from the skin's edge down (or up)
/// to a floor at @p z_floor, and the floor. Open at the skin.
template <class SurfaceZ>
inline void door_well(Mesh& m, float x0, float x1, float hw, float z_floor, SurfaceZ&& surface_z, bool floor = true) {
    constexpr int N = 6;
    auto rim = [&](float x, float y) { return V3{x, y, surface_z(x, y)}; };
    for (int k = 0; k < N; ++k) {
        const float ya = -hw + 2.0f * hw * static_cast<float>(k) / N, yb = -hw + 2.0f * hw * static_cast<float>(k + 1) / N;
        const float xa = x0 + (x1 - x0) * static_cast<float>(k) / N, xb = x0 + (x1 - x0) * static_cast<float>(k + 1) / N;
        for (float x : {x0, x1}) m.quad(rim(x, ya), rim(x, yb), {x, yb, z_floor}, {x, ya, z_floor});
        for (float y : {-hw, hw}) m.quad(rim(xa, y), rim(xb, y), {xb, y, z_floor}, {xa, y, z_floor});
    }
    if (floor) m.quad({x0, -hw, z_floor}, {x1, -hw, z_floor}, {x1, hw, z_floor}, {x0, hw, z_floor});
}

/// @brief Saw-tooth RAM coating along a door edge at x, across y0..y1: the
/// low-observable serrations round every F-35 door, @p dir pointing the
/// teeth fore (+1) or aft (-1).
template <class SurfaceZ>
inline void sawtooth(Mesh& m, float x, float y0, float y1, float depth, float dir, int teeth, SurfaceZ&& surface_z,
                     float proud) {
    for (int k = 0; k < teeth; ++k) {
        const float ya = y0 + (y1 - y0) * static_cast<float>(k) / static_cast<float>(teeth);
        const float yb = y0 + (y1 - y0) * static_cast<float>(k + 1) / static_cast<float>(teeth);
        const float ym = 0.5f * (ya + yb), xt = x + dir * depth;
        m.tri({x, ya, surface_z(x, ya) - proud}, {x, yb, surface_z(x, yb) - proud}, {xt, ym, surface_z(xt, ym) - proud});
    }
}

/// @brief A circle of radius r centred on c in the plane normal to n (n in
/// the x-z plane), at angle t from its top.
[[nodiscard]] inline V3 bearing_point(V3 c, V3 n, float r, float t) noexcept {
    const V3 u{0.0f, 1.0f, 0.0f};
    const V3 v = normalize(cross(n, u)); // up (-z) on a duct pointing aft
    return c + v * (r * std::cos(t)) + u * (r * std::sin(t));
}

/// @brief A duct segment between two circles (the 3BSM's bearing planes).
inline void duct(Mesh& m, V3 ca, V3 na, float ra, V3 cb, V3 nb, float rb, int sides = 28) {
    for (int k = 0; k < sides; ++k) {
        const float t0 = 6.2831853f * static_cast<float>(k) / static_cast<float>(sides);
        const float t1 = 6.2831853f * static_cast<float>(k + 1) / static_cast<float>(sides);
        const V3 a0 = bearing_point(ca, na, ra, t0), a1 = bearing_point(ca, na, ra, t1);
        const V3 b0 = bearing_point(cb, nb, rb, t0), b1 = bearing_point(cb, nb, rb, t1);
        m.tri_smooth(a0, normalize(a0 - ca), b0, normalize(b0 - cb), b1, normalize(b1 - cb));
        m.tri_smooth(a0, normalize(a0 - ca), b1, normalize(b1 - cb), a1, normalize(a1 - ca));
    }
}

/// @brief Rotation by @p a about the line through @p p along @p axis.
[[nodiscard]] inline Mat4 rotation_about(V3 p, V3 axis, float a) noexcept {
    const V3 k = normalize(axis);
    const float c = std::cos(a), s = std::sin(a), C = 1.0f - c;
    Mat4 m = Mat4::identity();
    m(0, 0) = c + k.x * k.x * C;       m(0, 1) = k.x * k.y * C - k.z * s; m(0, 2) = k.x * k.z * C + k.y * s;
    m(1, 0) = k.y * k.x * C + k.z * s; m(1, 1) = c + k.y * k.y * C;       m(1, 2) = k.y * k.z * C - k.x * s;
    m(2, 0) = k.z * k.x * C - k.y * s; m(2, 1) = k.z * k.y * C + k.x * s; m(2, 2) = c + k.z * k.z * C;
    // Keep p fixed: translation p - R p.
    m(0, 3) = p.x - (m(0, 0) * p.x + m(0, 1) * p.y + m(0, 2) * p.z);
    m(1, 3) = p.y - (m(1, 0) * p.x + m(1, 1) * p.y + m(1, 2) * p.z);
    m(2, 3) = p.z - (m(2, 0) * p.x + m(2, 1) * p.y + m(2, 2) * p.z);
    return m;
}

/// @brief @p m applied to a point (w = 1) or a direction (w = 0).
[[nodiscard]] inline V3 transform(const Mat4& m, V3 p, float w = 1.0f) noexcept {
    return {m(0, 0) * p.x + m(0, 1) * p.y + m(0, 2) * p.z + m(0, 3) * w,
            m(1, 0) * p.x + m(1, 1) * p.y + m(1, 2) * p.z + m(1, 3) * w,
            m(2, 0) * p.x + m(2, 1) * p.y + m(2, 2) * p.z + m(2, 3) * w};
}

/// @brief Everything the F-35B's pose needs that does not change: hinge
/// lines, bearing planes and the 3BSM's deflection against segment turn.
struct F35bRig {
    V3 fan_door_hinge, aux_hinge;
    V3 fan_exit_hinge[2], roll_hinge[2], bsm_door_hinge[2];
    V3 c1, c2, c3, exit;   ///< 3BSM bearing centres and nozzle exit
    V3 n2, n3;             ///< Canted bearing normals (pointing aft)
    V3 fan_exit, roll_exit[2];
    static constexpr int kTable = 37;
    std::array<float, kTable> deflection{}; ///< Nozzle deflection [rad] at segment turn i * 5 deg
};

/// @brief Direction of the swivelled nozzle (before the front bearing's
/// roll) when the middle and rear segments turn by +psi and -psi.
[[nodiscard]] inline V3 bsm_direction(const F35bRig& r, float psi) noexcept {
    const Mat4 m = rotation_about(r.c2, r.n2, psi) * rotation_about(r.c3, r.n3, -psi);
    return transform(m, {-1.0f, 0.0f, 0.0f}, 0.0f);
}

[[nodiscard]] inline const F35bRig& f35b_rig() {
    using namespace f35b_layout;
    static const F35bRig kRig = [] {
        F35bRig r;
        const std::vector<Section>& body = f35b_body();
        auto top = [&](float x) { return skin_z(body, x, 0.0f, true); };
        r.fan_door_hinge = {kFanDoorAft, 0.0f, top(kFanDoorAft) - 0.010f};
        r.aux_hinge = {kAuxAft, 0.0f, top(kAuxAft) - 0.010f};
        for (int i = 0; i < 2; ++i) {
            const float side = i == 0 ? -1.0f : 1.0f;
            const float xf = 0.5f * (kFanExitFwd + kFanExitAft);
            r.fan_exit_hinge[i] = {xf, side * kFanExitHalfW, skin_z(body, xf, kFanExitHalfW, false) + 0.010f};
            r.roll_hinge[i] = {kRollX, side * (kRollY + kRollHalfW), f35b_wing_lower_z(kRollX, kRollY + kRollHalfW) + 0.010f};
            const float xb = 0.5f * (kBsmDoorFwd + kBsm1);
            r.bsm_door_hinge[i] = {xb, side * kBsmDoorHalfW, skin_z(body, xb, kBsmDoorHalfW, false) + 0.010f};
            const math::Vector3& rp = kCfg.roll_post_pos_b;
            r.roll_exit[i] = {static_cast<float>(rp.x), side * static_cast<float>(rp.y), static_cast<float>(rp.z)};
        }
        const math::Vector3& fp = kCfg.lift_fan_pos_b;
        r.fan_exit = {static_cast<float>(fp.x), static_cast<float>(fp.y), static_cast<float>(fp.z)};
        r.c1 = {kBsm1, 0.0f, kBsmZ};
        r.c2 = {kBsm2, 0.0f, kBsmZ};
        r.c3 = {kBsm3, 0.0f, kBsmZ};
        r.exit = {kBsmExit, 0.0f, kBsmZ};
        r.n2 = {-std::cos(kBsmCant), 0.0f, std::sin(kBsmCant)};
        r.n3 = {-std::cos(kBsmCant), 0.0f, -std::sin(kBsmCant)};
        for (int i = 0; i < F35bRig::kTable; ++i) {
            const V3 d = bsm_direction(r, 3.14159265f * static_cast<float>(i) / (F35bRig::kTable - 1));
            r.deflection[static_cast<size_t>(i)] = std::acos(std::clamp(-d.x, -1.0f, 1.0f));
        }
        return r;
    }();
    return kRig;
}

/// F-35B: the chined nose and one-piece canopy, DSI bumps and their
/// forward-swept cowls beside the cockpit, the LiftFan hump behind the
/// canopy, the trapezoid wing, twin canted fins and all-moving stabilators
/// on tail booms, and the three-bearing swivel nozzle. The lift system's
/// doors, the vane box and the nozzle segments are separate parts, posed
/// each frame by stovl_pose().
[[nodiscard]] inline std::vector<ProceduralPart> f35b(const AirframeInfo& a) {
    using namespace f35b_layout;
    // FS 36170 "Have Glass" grey; RAM-treated edges a shade darker (~FS 36118).
    const ProceduralPart finish = paint(0.084f, 0.088f, 0.093f, 0.46f);
    Parts p(finish);
    p.canopy = glass(0.95f, 0.86f, 0.62f); // gold ITO film
    ProceduralPart coating = paint(0.056f, 0.060f, 0.066f, 0.40f);
    Mesh s(p.skin);
    const CanopyFit& c = a.canopy;
    const float ws = a.eye_x + c.ws_dx, tub = a.eye_x + c.tub_aft_dx;
    const std::vector<Section>& body = f35b_body();
    auto top = [&](float x, float y) { return skin_z(body, x, y, true); };
    auto belly = [&](float x, float y) { return skin_z(body, x, y, false); };
    loft(s, body, 32,
         {Cut{tub, ws, c.half_span + 0.01f}, Cut{kFanDoorAft, kFanDoorFwd, kFanDoorHalfW - 0.04f},
          Cut{kAuxAft, kAuxFwd, kAuxHalfW - 0.04f}, Cut{kFanExitAft, kFanExitFwd, kFanExitHalfW - 0.04f, true},
          Cut{kBsm1, kBsmDoorFwd, kBsmDoorHalfW - 0.04f, true}});

    for (float side : {-1.0f, 1.0f}) {
        // DSI: the compression bump on the fuselage side, then the inlet
        // whose cowl wraps it, its trunk blending aft into the body.
        loft(s,
             {{6.55f, 0.02f, -0.22f, -0.18f, 2.0f, 2.0f, side * 0.70f},
              {6.10f, 0.16f, -0.36f, 0.00f, 2.0f, 2.0f, side * 0.74f},
              {5.65f, 0.20f, -0.40f, 0.04f, 2.0f, 2.0f, side * 0.76f},
              {5.30f, 0.10f, -0.32f, -0.04f, 2.0f, 2.0f, side * 0.72f}},
             16);
        loft(s,
             {{5.65f, 0.34f, -0.48f, 0.40f, 3.5f, 3.0f, side * 1.00f},
              {4.90f, 0.38f, -0.62f, 0.48f, 3.5f, 3.0f, side * 1.04f},
              {3.30f, 0.36f, -0.70f, 0.55f, 3.5f, 3.0f, side * 1.06f},
              {1.20f, 0.24f, -0.66f, 0.55f, 3.0f, 3.0f, side * 0.98f},
              {-0.60f, 0.06f, -0.58f, 0.40f, 3.0f, 3.0f, side * 0.85f}},
             24);
        Mesh mouth(p.dark);
        cap(mouth, Section{5.64f, 0.30f, -0.44f, 0.36f, 3.5f, 3.0f, side * 1.00f}, 24);
        // Tail boom carrying the fin and stabilator.
        loft(s,
             {{-2.60f, 0.05f, -0.56f, -0.46f, 3.0f, 3.0f, side * 1.05f},
              {-3.30f, 0.30f, -0.68f, 0.30f, 3.0f, 3.0f, side * 1.05f},
              {-5.85f, 0.28f, -0.65f, 0.25f, 3.0f, 3.0f, side * 1.05f},
              {-6.55f, 0.08f, -0.48f, 0.10f, 3.0f, 3.0f, side * 1.05f}},
             16);
        // Fin, canted 25 deg outboard.
        surface(s, {-3.35f, side * 1.05f, -0.65f}, {-4.75f, side * 1.90f, -2.46f}, {-6.23f, side * 1.90f, -2.46f},
                {-5.85f, side * 1.05f, -0.65f}, 0.13f, 0.05f);
    }
    // Chine shelf from under the inlets out to the wing's leading edge.
    surface_pair(s, {5.50f, 0.95f, 0.10f}, {1.95f, 1.45f, 0.08f}, {1.00f, 1.45f, 0.08f}, {1.00f, 0.95f, 0.10f}, 0.10f, 0.06f);
    // Wing (LE 34 deg, TE -13 deg) and stabilators (6.65 m span).
    surface_pair(s, {kWingLe0 - kWingRootY * kWingTanLe, kWingRootY, kWingZ},
                 {kWingLe0 - kWingTipY * kWingTanLe, kWingTipY, kWingZ},
                 {kWingLe0 - kWingTipY * kWingTanLe - 1.57f, kWingTipY, kWingZ},
                 {kWingTe0 + kWingRootY * kWingTanTe, kWingRootY, kWingZ}, kWingTRoot, kWingTTip);
    surface_pair(s, {-3.80f, 1.00f, 0.12f}, {-5.75f, 3.325f, 0.12f}, {-6.55f, 3.325f, 0.12f}, {-6.35f, 1.00f, 0.12f}, 0.14f, 0.04f);
    {
        // EOTS: the faceted sapphire window under the nose.
        Mesh eots(p.canopy);
        const float z = belly(7.85f, 0.0f) - 0.01f;
        const V3 q[4] = {{8.05f, 0.0f, z}, {7.80f, 0.11f, z}, {7.65f, 0.0f, z}, {7.80f, -0.11f, z}};
        const V3 apex{7.82f, 0.0f, z + 0.10f};
        for (int i = 0; i < 4; ++i) eots.tri(q[i], q[(i + 1) % 4], apex);
    }

    // The lift system's openings: the bays under the doors (rigid).
    Mesh bay(p.dark);
    door_well(bay, kFanDoorAft, kFanDoorFwd, kFanDoorHalfW - 0.04f, -0.30f, top, false);
    door_well(bay, kAuxAft, kAuxFwd, kAuxHalfW - 0.04f, -0.35f, top);
    door_well(bay, kFanExitAft, kFanExitFwd, kFanExitHalfW - 0.04f, 0.02f, belly, false);
    door_well(bay, kBsm1, kBsmDoorFwd, kBsmDoorHalfW - 0.04f, 0.25f, belly);
    {
        // LiftFan face: two-stage, 50 in, seen through the open inlet door.
        constexpr float kFanR = 0.64f, kFanZ = -0.30f;
        const float xc = 0.5f * (kFanExitFwd + kFanExitAft);
        Mesh fan(p.nozzle);
        Mesh blades(p.dark);
        for (int k = 0; k < 24; ++k) {
            const float t = 6.2831853f * static_cast<float>(k) / 24.0f;
            const float ca = std::cos(t), sa = std::sin(t), cb = std::cos(t + 0.10f), sb = std::sin(t + 0.10f);
            const float ce = std::cos(t + 0.2618f), se = std::sin(t + 0.2618f);
            fan.tri({xc, 0.0f, kFanZ}, {xc + kFanR * ca, kFanR * sa, kFanZ}, {xc + kFanR * ce, kFanR * se, kFanZ});
            blades.quad({xc + 0.18f * ca, 0.18f * sa, kFanZ - 0.004f}, {xc + kFanR * ca, kFanR * sa, kFanZ - 0.004f},
                        {xc + kFanR * cb, kFanR * sb, kFanZ - 0.004f}, {xc + 0.18f * cb, 0.18f * sb, kFanZ - 0.004f});
        }
        Mesh hub(p.nozzle);
        tube(hub, {xc, 0.0f, kFanZ}, {xc, 0.0f, kFanZ - 0.12f}, 0.17f, 0.05f, 16);
    }
    {
        // Saw-tooth RAM coating fore and aft of the dorsal doors.
        Mesh rim(coating);
        sawtooth(rim, kFanDoorFwd + 0.01f, -kFanDoorHalfW, kFanDoorHalfW, 0.10f, 1.0f, 7, top, 0.004f);
        sawtooth(rim, kAuxAft - 0.01f, -kAuxHalfW, kAuxHalfW, 0.08f, -1.0f, 6, top, 0.004f);
        // RAM-treated leading edges of the wing roots' chine.
        for (float side : {-1.0f, 1.0f}) {
            rim.quad({5.64f, side * 0.66f, -0.47f}, {5.64f, side * 1.34f, -0.47f}, {5.60f, side * 1.34f, -0.50f},
                     {5.60f, side * 0.66f, -0.50f});
        }
    }

    std::vector<ProceduralPart> moving;
    auto add = [&](ProceduralPart part, int joint) -> ProceduralPart& {
        part.joint = joint;
        moving.push_back(std::move(part));
        return moving.back();
    };
    moving.reserve(32); // references from add() stay valid: never more parts than this
    const ProceduralPart inside = paint(0.36f, 0.37f, 0.38f, 0.55f); // light grey door inner faces
    auto door = [&](int joint, float x0, float x1, float y0, float y1, auto&& surface_z, bool on_top) {
        ProceduralPart& o = add(finish, joint);
        ProceduralPart& i = add(inside, joint);
        Mesh mo(o), mi(i);
        door_panel(mo, mi, x0, x1, y0, y1, 0.010f, 0.035f, surface_z, on_top);
    };
    door(kLiftFanDoor, kFanDoorAft, kFanDoorFwd, -kFanDoorHalfW, kFanDoorHalfW, top, true);
    door(kAuxDoors, kAuxAft, kAuxFwd, -kAuxHalfW, -0.005f, top, true);
    door(kAuxDoors, kAuxAft, kAuxFwd, 0.005f, kAuxHalfW, top, true);
    door(kFanDoorL, kFanExitAft, kFanExitFwd, -kFanExitHalfW, 0.0f, belly, false);
    door(kFanDoorR, kFanExitAft, kFanExitFwd, 0.0f, kFanExitHalfW, belly, false);
    door(kBsmDoorL, kBsm1, kBsmDoorFwd, -kBsmDoorHalfW, 0.0f, belly, false);
    door(kBsmDoorR, kBsm1, kBsmDoorFwd, 0.0f, kBsmDoorHalfW, belly, false);
    auto wing = [](float x, float y) { return f35b_wing_lower_z(x, y); };
    door(kRollDoorL, kRollX - kRollHalfL, kRollX + kRollHalfL, -kRollY - kRollHalfW, -kRollY + kRollHalfW, wing, false);
    door(kRollDoorR, kRollX - kRollHalfL, kRollX + kRollHalfL, kRollY - kRollHalfW, kRollY + kRollHalfW, wing, false);
    {
        // Roll-post nozzle exits in the wing, under their doors.
        Mesh post(p.dark);
        for (float side : {-1.0f, 1.0f}) {
            const float y0 = side * kRollY - 0.08f, y1 = side * kRollY + 0.08f, x0 = kRollX - 0.18f, x1 = kRollX + 0.18f;
            post.quad({x0, y0, wing(x0, y0) + 0.004f}, {x1, y0, wing(x1, y0) + 0.004f}, {x1, y1, wing(x1, y1) + 0.004f},
                      {x0, y1, wing(x0, y1) + 0.004f});
        }
    }
    {
        // Vane box: nine louvres across the LiftFan exit.
        ProceduralPart& vanes = add(metal(0.20f), kVaneBox);
        Mesh v(vanes);
        for (int k = 0; k < 9; ++k) {
            const float x = kFanExitAft + 0.10f + (kFanExitFwd - kFanExitAft - 0.20f) * static_cast<float>(k) / 8.0f;
            v.quad({x, -kFanExitHalfW + 0.06f, kVaneZ - 0.10f}, {x, kFanExitHalfW - 0.06f, kVaneZ - 0.10f},
                   {x, kFanExitHalfW - 0.06f, kVaneZ + 0.10f}, {x, -kFanExitHalfW + 0.06f, kVaneZ + 0.10f});
        }
    }
    {
        // 3BSM: front segment (rolls), middle and rear segments on their
        // canted bearings, a flange at each bearing; the nozzle with its
        // serrated outer flaps and the dark core.
        const F35bRig& r = f35b_rig();
        const V3 ax{-1.0f, 0.0f, 0.0f};
        ProceduralPart& sa = add(metal(0.17f), kBsmA);
        ProceduralPart& sb = add(metal(0.15f), kBsmB);
        ProceduralPart& sc = add(metal(0.13f), kBsmC);
        ProceduralPart& core = add(paint(0.012f, 0.012f, 0.013f, 0.8f), kBsmC);
        Mesh ma(sa), mb(sb), mc(sc), mk(core);
        duct(ma, r.c1, ax, kBsmRing, r.c2, r.n2, kBsmRing);
        duct(mb, r.c2, r.n2, kBsmRing, r.c3, r.n3, kBsmRing);
        duct(mc, r.c3, r.n3, kBsmRing, r.exit, ax, kBsmExitR);
        duct(mb, r.c2 + r.n2 * -0.02f, r.n2, kBsmRing + 0.015f, r.c2 + r.n2 * 0.02f, r.n2, kBsmRing + 0.015f);
        duct(mc, r.c3 + r.n3 * -0.02f, r.n3, kBsmRing + 0.015f, r.c3 + r.n3 * 0.02f, r.n3, kBsmRing + 0.015f);
        constexpr int kTeeth = 14;
        for (int k = 0; k < kTeeth; ++k) {
            const float t0 = 6.2831853f * static_cast<float>(k) / kTeeth, t1 = 6.2831853f * static_cast<float>(k + 1) / kTeeth;
            const V3 a0 = bearing_point(r.exit, ax, kBsmExitR, t0), a1 = bearing_point(r.exit, ax, kBsmExitR, t1);
            const V3 tip = bearing_point(r.exit + ax * 0.09f, ax, kBsmExitR * 0.97f, 0.5f * (t0 + t1));
            mc.tri(a0, a1, tip);
        }
        cap(mk, Section{kBsmExit + 0.10f, kBsmExitR * 0.95f, kBsmZ - kBsmExitR * 0.95f, kBsmZ + kBsmExitR * 0.95f, 2.0f, 2.0f}, 24);
    }

    outside_cockpit(p, a);
    Mesh glass_mesh(p.canopy);
    canopy_glass(glass_mesh, c, a.eye_x, a.eye_z);
    {
        // Gear, where the flight model's struts are: a single nose wheel
        // under the cockpit; main legs out in the wing roots.
        Mesh strut(p.gear), tyre(p.tyres);
        const auto& g = kConfig.gear;
        const float nx = static_cast<float>(g.nose_pos_b.x), mx = static_cast<float>(g.main_r_pos_b.x);
        const float my = static_cast<float>(g.main_r_pos_b.y);
        const float axle_n = a.contact_z - 0.30f, axle_m = a.contact_z - 0.40f;
        tube(strut, {nx, 0.0f, belly(nx, 0.0f) - 0.05f}, {nx, 0.0f, axle_n}, 0.07f, 0.06f, 10);
        tube(tyre, {nx, -0.09f, axle_n}, {nx, 0.09f, axle_n}, 0.30f, 0.30f, 18);
        for (float side : {-1.0f, 1.0f}) {
            tube(strut, {mx, side * (my - 0.15f), 0.20f}, {mx, side * (my - 0.07f), axle_m}, 0.09f, 0.08f, 10);
            tube(tyre, {mx, side * my - 0.14f, axle_m}, {mx, side * my + 0.14f, axle_m}, 0.40f, 0.40f, 18);
        }
    }
    std::vector<ProceduralPart> out = std::move(p).list();
    out.push_back(std::move(coating));
    for (ProceduralPart& m : moving) out.push_back(std::move(m));
    return out;
}

} // namespace airframe_build

/// @brief Eye point, gear stance, canopy and nozzles of @p type's airframe.
/// The F-16C's mesh is its asset; only its nozzle is described here.
[[nodiscard]] inline AirframeInfo airframe_info(aircraft::AircraftType type) {
    using aircraft::AircraftType;
    switch (type) {
        case AircraftType::F15EX_EAGLE_II:      return airframe_build::f15ex_info();
        case AircraftType::EUROFIGHTER_TYPHOON: return airframe_build::typhoon_info();
        case AircraftType::F22_RAPTOR:          return airframe_build::f22_info();
        case AircraftType::A10_THUNDERBOLT:     return airframe_build::a10c_info();
        case AircraftType::F35B_LIGHTNING_II:   return airframe_build::f35b_info();
        case AircraftType::F16_FIGHTING_FALCON: break;
    }
    AirframeInfo f16;
    const math::Vector3 n = ModelGLB::nozzle_exit_body();
    f16.nozzles[0] = NozzleExit{static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z), 0.40f};
    return f16;
}

/// @brief The procedural airframe for @p type (empty for the F-16C, which
/// is drawn from its asset), and where it sits for each alignment.
[[nodiscard]] inline std::vector<ProceduralPart> build_airframe(aircraft::AircraftType type, ModelPlacement& placement) {
    using aircraft::AircraftType;
    const AirframeInfo info = airframe_info(type);
    placement = ModelPlacement{};
    placement.scale = 1.0f;
    placement.world_x = 0.0f;
    placement.world_z = 0.0f;
    // The cockpit view puts the airframe's eye point on the design eye point.
    placement.cockpit_x = 1.80f - info.eye_x;
    placement.cockpit_z = -0.65f - info.eye_z;
    switch (type) {
        case AircraftType::F15EX_EAGLE_II:      return airframe_build::f15ex(info);
        case AircraftType::EUROFIGHTER_TYPHOON: return airframe_build::typhoon(info);
        case AircraftType::F22_RAPTOR:          return airframe_build::f22(info);
        case AircraftType::A10_THUNDERBOLT:     return airframe_build::a10c(info);
        case AircraftType::F35B_LIGHTNING_II:   return airframe_build::f35b(info);
        case AircraftType::F16_FIGHTING_FALCON: break;
    }
    return {};
}

/// @brief The F-35B lift system's pose this frame: transforms for its doors,
/// vane box and 3BSM segments, and where each lift jet leaves the airframe
/// (body axes, world alignment) and which way it blows.
struct StovlPose {
    JointPose joints{};
    airframe_build::V3 nozzle_pos{}, nozzle_dir{-1.0f, 0.0f, 0.0f}; ///< 3BSM exit
    airframe_build::V3 fan_pos{}, fan_dir{0.0f, 0.0f, 1.0f};         ///< LiftFan vane box
    std::array<airframe_build::V3, 2> roll_pos{};                     ///< Roll posts, left then right (blowing down)
};

/// @brief Poses the F-35B from the flight model's lift-system snapshot.
/// Cheap (a few small matrices): called once per F-35B per frame.
[[nodiscard]] inline StovlPose f35b_stovl_pose(const propulsion::StovlStatus& st) noexcept {
    using namespace airframe_build;
    using namespace airframe_build::f35b_layout;
    const F35bRig& r = f35b_rig();
    StovlPose out;
    auto& J = out.joints.m;
    out.joints.count = kF35bJointCount;
    auto frac = [](double v) { return static_cast<float>(std::clamp(v, 0.0, 1.0)); };
    constexpr float kDeg = 3.14159265f / 180.0f;
    const V3 X{1.0f, 0.0f, 0.0f}, Y{0.0f, 1.0f, 0.0f};

    // Doors: the dorsal ones lift their forward edges; the lower fan,
    // roll-post and 3BSM doors swing down about their outboard edges.
    J[kLiftFanDoor] = rotation_about(r.fan_door_hinge, Y, kFanDoorOpen * frac(st.lift_fan_door));
    J[kAuxDoors] = rotation_about(r.aux_hinge, Y, kAuxOpen * frac(st.aux_inlet_door));
    const float fan = kSideDoorOpen * frac(st.fan_nozzle_door), roll = kSideDoorOpen * frac(st.roll_post_door);
    for (int i = 0; i < 2; ++i) {
        const float sgn = i == 0 ? 1.0f : -1.0f;
        J[static_cast<size_t>(kFanDoorL + i)] = rotation_about(r.fan_exit_hinge[i], X, sgn * fan);
        J[static_cast<size_t>(kRollDoorL + i)] = rotation_about(r.roll_hinge[i], X, sgn * roll);
        J[static_cast<size_t>(kBsmDoorL + i)] = rotation_about(r.bsm_door_hinge[i], X, sgn * fan);
    }

    // Vane box: every louvre turns about its own pivot, a shear in x by z.
    const float vane = static_cast<float>(std::clamp(st.vane_box_deg, -10.0, 60.0)) * kDeg;
    J[kVaneBox] = Mat4::identity();
    J[kVaneBox](0, 2) = -std::tan(vane);
    J[kVaneBox](0, 3) = kVaneZ * std::tan(vane);
    out.fan_pos = r.fan_exit;
    out.fan_dir = {-std::sin(vane), 0.0f, std::cos(vane)};

    // 3BSM: the middle and rear segments counter-rotate on their canted
    // bearings to swing the nozzle down; the front segment rolls the whole
    // swivel back into the vertical plane, then off it for yaw.
    const float defl = static_cast<float>(std::clamp(st.nozzle_pitch_deg, 0.0, 95.0)) * kDeg;
    int i = 0;
    while (i + 2 < F35bRig::kTable && r.deflection[static_cast<size_t>(i + 1)] < defl) ++i;
    const float d0 = r.deflection[static_cast<size_t>(i)], d1 = r.deflection[static_cast<size_t>(i + 1)];
    const float t = d1 > d0 ? std::clamp((defl - d0) / (d1 - d0), 0.0f, 1.0f) : 0.0f;
    const float psi = (static_cast<float>(i) + t) * 3.14159265f / (F35bRig::kTable - 1);
    const V3 d = bsm_direction(r, psi);
    const float roll_in = std::hypot(d.y, d.z) > 1e-6f ? std::atan2(d.y, d.z) : 0.0f;
    const float yaw = static_cast<float>(std::clamp(st.nozzle_yaw_deg, -15.0, 15.0)) * kDeg;
    J[kBsmA] = rotation_about(r.c1, X, roll_in - yaw);
    J[kBsmB] = J[kBsmA] * rotation_about(r.c2, r.n2, psi);
    J[kBsmC] = J[kBsmB] * rotation_about(r.c3, r.n3, -psi);
    out.nozzle_pos = transform(J[kBsmC], r.exit);
    out.nozzle_dir = transform(J[kBsmC], {-1.0f, 0.0f, 0.0f}, 0.0f);
    out.roll_pos = {r.roll_exit[0], r.roll_exit[1]};
    return out;
}

/// @brief Moving-part pose of @p type's airframe (none but the F-35B's).
[[nodiscard]] inline JointPose airframe_pose(aircraft::AircraftType type, const propulsion::StovlStatus& st) noexcept {
    if (type == aircraft::AircraftType::F35B_LIGHTNING_II) return f35b_stovl_pose(st).joints;
    return {};
}

} // namespace fastjet::graphics
