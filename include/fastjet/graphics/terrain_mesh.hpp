#pragma once

#include "fastjet/core/thread_pool.hpp"
#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/terrain_field.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace fastjet::graphics {

/// @brief Static terrain mesh built from TerrainField as concentric LOD rings,
/// plus a high-resolution strip that follows the canyon.
///
/// The aircraft spends most of its time near the airfield, so resolution is
/// spent there: an inner patch at fine spacing, then successively coarser rings
/// out to the visible horizon. Rings are built once at startup into a single
/// interleaved buffer, keeping the zero-allocation-per-frame property of the
/// rest of the simulation.
///
/// The canyon is far too narrow for the rings (its gorge is 130 m wide; the
/// rings there are 150-1000 m cells), so it gets a strip of its own: a grid in
/// the canyon's (along, across) coordinates, 25 m across the gorge and walls
/// and coarser out on the mesa. Ring cells that the strip covers are left out.
///
/// Vertex layout: pos(3), normal(3), height-above-field(1) = 7 floats.
/// Colour is derived in the fragment shader from elevation and slope, so the
/// buffer stays small and the palette can change without a rebuild.
class TerrainMesh {
private:
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLsizei vertex_count_ = 0;
    bool initialized_ = false;

    struct Ring {
        float half_extent; // [m] outer half-size of this ring
        float cell;        // [m] grid spacing
    };

    // The airfield decals are drawn as a separate coplanar layer. Rather than
    // fight them for depth (20 cm of separation is far below the precision of a
    // 120 km depth range), the terrain simply leaves a hole for the apron and
    // the decals fill it. Kept slightly inside the apron edge so there is no gap.
    static constexpr float APRON_X0 = -690.0f;
    static constexpr float APRON_X1 =  3690.0f;
    static constexpr float APRON_Y0 = -890.0f;
    static constexpr float APRON_Y1 =   890.0f;

    /// @brief True if a cell lies within the airfield footprint and must be
    /// skipped so the apron and its markings own those pixels outright.
    [[nodiscard]] static bool in_airfield(float x0, float y0, float x1, float y1) noexcept {
        return x1 > APRON_X0 && x0 < APRON_X1 &&
               y1 > APRON_Y0 && y0 < APRON_Y1;
    }

    // Ring 0 is a solid patch; the rest are hollow frames around it. Spacing
    // roughly doubles each step, so screen-space triangle size stays even.
    static constexpr Ring RINGS[] = {
        {  4500.0f,    50.0f }, // Basin & immediate airfield vicinity (high fidelity)
        { 14000.0f,   150.0f }, // Near mountain foothills & ridges
        { 36000.0f,   400.0f }, // Mid mountain ranges
        { 65000.0f,  1000.0f }, // Outer mountain chains
        { 95000.0f,  2500.0f }, // Horizon silhouettes
    };
    static constexpr int RING_COUNT = 5;

    // ---- Canyon strip ------------------------------------------------------
    using Canyon = TerrainField::Canyon;
    /// Rows along the canyon axis [m].
    static constexpr float STRIP_ROW = 60.0f;
    /// Across-canyon spacing: fine over the gorge and walls, coarser on the mesa.
    static constexpr float STRIP_FINE = 25.0f, STRIP_FINE_HALF = 600.0f;
    static constexpr float STRIP_MID = 70.0f, STRIP_MID_HALF = 1400.0f;
    static constexpr float STRIP_COARSE = 160.0f;
    /// Ring cells reach this far into the strip before they are cut away
    /// (one diagonal of the coarsest ring that meets it).
    static constexpr float STRIP_OVERLAP = 1500.0f;
    /// Where the rings and the strip overlap, the strip is sunk this far so
    /// the two coincident surfaces do not fight for depth.
    static constexpr float STRIP_SINK = 12.0f;

    static void push_vertex(std::vector<float>& v, float x, float y, float h, float normal_span) {
        float nx, ny, nz;
        // Differentiate over a span tied to feature size, not cell size: this
        // keeps lighting continuous across the LOD ring seams, where cell size
        // jumps by 3x and per-cell normals would visibly crease.
        TerrainField::normal_ned(x, y, normal_span, nx, ny, nz);

        v.push_back(x);
        v.push_back(y);
        v.push_back(-h); // NED: elevation is negative z
        v.push_back(nx);
        v.push_back(ny);
        v.push_back(nz);
        v.push_back(h);
    }

    static void push_vertex(std::vector<float>& v, float x, float y) {
        push_vertex(v, x, y, TerrainField::height(x, y), 35.0f);
    }

    /// @brief Emit one cell as two triangles.
    ///
    /// Winding is counter-clockwise as seen from above the terrain. NED has +X
    /// north, +Y east and +Z *down*, so it is left-handed with respect to the
    /// usual screen convention: walking the cell in +X then +Y order gives a
    /// clockwise face when viewed from above. The reversed vertex order below
    /// is what makes GL_CCW front-facing correct for a camera looking down.
    static void push_quad(std::vector<float>& v, float x0, float y0, float x1, float y1) {
        push_vertex(v, x0, y0);
        push_vertex(v, x1, y1);
        push_vertex(v, x1, y0);

        push_vertex(v, x0, y0);
        push_vertex(v, x0, y1);
        push_vertex(v, x1, y1);
    }

    /// @brief Cell fully covered by the canyon strip (so the rings leave it out).
    [[nodiscard]] static bool under_strip(float x0, float y0, float x1, float y1) noexcept {
        constexpr float margin = 10.0f;
        return Canyon::in_strip(x0, y0, margin) && Canyon::in_strip(x1, y0, margin) &&
               Canyon::in_strip(x0, y1, margin) && Canyon::in_strip(x1, y1, margin);
    }

    /// @brief A vertical curtain hanging from the edge (xa, ya)-(xb, yb),
    /// drawn from both sides.
    ///
    /// Where a fine ring meets a coarse one, the coarse cell's straight edge
    /// cuts under the fine ring's sampled edge and leaves slivers of sky. The
    /// skirt hangs below the fine edge and fills them; it is shaded with the
    /// ground's own normal, so it reads as more ground.
    static void push_skirt(std::vector<float>& v, float xa, float ya, float xb, float yb, float depth) {
        const float ha = TerrainField::height(xa, ya);
        const float hb = TerrainField::height(xb, yb);
        float nax, nay, naz, nbx, nby, nbz;
        TerrainField::normal_ned(xa, ya, 35.0f, nax, nay, naz);
        TerrainField::normal_ned(xb, yb, 35.0f, nbx, nby, nbz);
        auto vert = [&](float x, float y, float h, float nx, float ny, float nz, float hv) {
            v.insert(v.end(), {x, y, -h, nx, ny, nz, hv});
        };
        const float a_top[7] = {xa, ya, ha, nax, nay, naz, ha};
        const float b_top[7] = {xb, yb, hb, nbx, nby, nbz, hb};
        const float a_bot[7] = {xa, ya, ha - depth, nax, nay, naz, ha};
        const float b_bot[7] = {xb, yb, hb - depth, nbx, nby, nbz, hb};
        auto put = [&](const float* p) { vert(p[0], p[1], p[2], p[3], p[4], p[5], p[6]); };
        // Both windings, so the curtain survives back-face culling from either side.
        put(a_top); put(b_top); put(b_bot);
        put(a_top); put(b_bot); put(a_bot);
        put(a_top); put(b_bot); put(b_top);
        put(a_top); put(a_bot); put(b_bot);
    }

    /// @brief Skirts around the outside of the square of half-size @p ext, at
    /// the fine ring's spacing @p cell.
    static void push_ring_skirt(std::vector<float>& v, float cx, float ext, float cell, float depth) {
        const int n = static_cast<int>(std::lround(2.0f * ext / cell));
        for (int i = 0; i < n; ++i) {
            const float a = -ext + static_cast<float>(i) * cell;
            const float b = a + cell;
            // North and south edges (constant X), then west and east (constant Y).
            for (float side : {-ext, ext}) {
                if (!under_strip(cx + side - 1.0f, a, cx + side + 1.0f, b)) push_skirt(v, cx + side, a, cx + side, b, depth);
                if (!under_strip(cx + a, side - 1.0f, cx + b, side + 1.0f)) push_skirt(v, cx + a, side, cx + b, side, depth);
            }
        }
    }

    /// @brief Across-canyon sample positions [m from the centreline].
    [[nodiscard]] static std::vector<float> strip_columns() {
        std::vector<float> d;
        const float W = Canyon::STRIP_HALF;
        auto step_at = [](float ad) {
            return ad < STRIP_FINE_HALF ? STRIP_FINE : ad < STRIP_MID_HALF ? STRIP_MID : STRIP_COARSE;
        };
        // March outward from the centre on each side, then mirror.
        std::vector<float> half{0.0f};
        float a = 0.0f;
        while (a < W) {
            a = std::min(W, a + step_at(a + 0.5f));
            half.push_back(a);
        }
        for (auto it = half.rbegin(); it != half.rend(); ++it) d.push_back(-*it);
        for (size_t i = 1; i < half.size(); ++i) d.push_back(half[i]);
        return d;
    }

    /// @brief Strip vertex at row @p s, across distance @p d.
    static void strip_vertex(std::vector<float>& v, float s, float d) {
        float x, y;
        Canyon::from_axis(s, Canyon::across_at(s, d), x, y);
        const float W = Canyon::STRIP_HALF;
        const float s_lo = -Canyon::STRIP_EXTEND, s_hi = Canyon::LENGTH + Canyon::STRIP_EXTEND;
        // Sink where the rings still overlap the strip, at its edges and ends.
        auto ramp = [](float e0, float e1, float u) {
            const float t = std::clamp((u - e0) / (e1 - e0), 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        };
        const float edge = std::max({ramp(W - STRIP_OVERLAP, W, std::fabs(d)),
                                     ramp(s_lo + STRIP_OVERLAP, s_lo, s), ramp(s_hi - STRIP_OVERLAP, s_hi, s)});
        const float h = TerrainField::height(x, y);
        // Crisp normals over the walls, ring-matched ones where the strip meets the rings.
        const float span = std::fabs(d) < STRIP_MID_HALF ? 8.0f : 35.0f;
        push_vertex(v, x, y, h - STRIP_SINK * edge, span);
        // The height attribute stays the true elevation, for the material.
        v.back() = h;
    }

    static void build_canyon_strip(std::vector<float>& verts) {
        const std::vector<float> cols = strip_columns();
        const float s0 = -Canyon::STRIP_EXTEND;
        const int rows = static_cast<int>(std::ceil((Canyon::LENGTH + 2.0f * Canyon::STRIP_EXTEND) / STRIP_ROW));
        const auto nc = static_cast<int>(cols.size());

        // Every row of vertices is sampled once, in parallel.
        std::vector<std::vector<float>> grid(static_cast<size_t>(rows + 1));
        core::ThreadPool::shared().parallel_for(0, rows + 1, [&](int i) {
            const float s = std::min(s0 + static_cast<float>(i) * STRIP_ROW, Canyon::LENGTH + Canyon::STRIP_EXTEND);
            auto& row = grid[static_cast<size_t>(i)];
            row.reserve(static_cast<size_t>(nc) * 7);
            for (float d : cols) strip_vertex(row, s, d);
        });

        // Rows run along the axis like X, columns across it like Y, and the
        // axis frame has the same handedness as NED: the ring winding applies.
        auto copy = [&](int i, int j) {
            const float* p = grid[static_cast<size_t>(i)].data() + static_cast<size_t>(j) * 7;
            verts.insert(verts.end(), p, p + 7);
        };
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j + 1 < nc; ++j) {
                copy(i, j); copy(i + 1, j + 1); copy(i + 1, j);
                copy(i, j); copy(i, j + 1);     copy(i + 1, j + 1);
            }
        }
    }

    void build() {
        std::vector<float> verts;
        // Sized for the ring table (~105k cells), the canyon strip (~64k
        // cells) and the seam skirts, at 6 verts x 7 floats per cell.
        verts.reserve(7'500'000);

        const float cx = TerrainField::FIELD_CENTER_X;

        float prev_extent = 0.0f;
        float prev_cell = 0.0f;
        for (int r = 0; r < RING_COUNT; ++r) {
            const float ext  = RINGS[r].half_extent;
            const float cell = RINGS[r].cell;

            // Snap the ring bounds to the cell grid so quads align exactly.
            const float span = std::ceil(2.0f * ext / cell) * cell;
            const float lo_x = cx - span * 0.5f;
            const float lo_y = -span * 0.5f;

            const int n = static_cast<int>(span / cell);
            // Rows are sampled across the thread pool, each into its own
            // buffer, then joined in row order: the mesh is identical to a
            // serial build.
            std::vector<std::vector<float>> rows(static_cast<size_t>(n));
            core::ThreadPool::shared().parallel_for(0, n, [&](int i) {
                std::vector<float>& row = rows[static_cast<size_t>(i)];
                const float x0 = lo_x + static_cast<float>(i) * cell;
                const float x1 = x0 + cell;
                for (int j = 0; j < n; ++j) {
                    const float y0 = lo_y + static_cast<float>(j) * cell;
                    const float y1 = y0 + cell;

                    if (prev_extent > 0.0f) {
                        const bool inside_x = (x0 >= cx - prev_extent) && (x1 <= cx + prev_extent);
                        const bool inside_y = (y0 >= -prev_extent) && (y1 <= prev_extent);
                        if (inside_x && inside_y) continue;
                    }

                    if (in_airfield(x0, y0, x1, y1)) continue;
                    if (under_strip(x0, y0, x1, y1)) continue;

                    push_quad(row, x0, y0, x1, y1);
                }
            });
            for (const auto& row : rows) verts.insert(verts.end(), row.begin(), row.end());
            // Close the seam to the finer ring inside this one.
            if (prev_extent > 0.0f) push_ring_skirt(verts, cx, prev_extent, prev_cell, 10.0f + 0.15f * cell);
            prev_extent = span * 0.5f;
            prev_cell = cell;
        }

        build_canyon_strip(verts);

        vertex_count_ = static_cast<GLsizei>(verts.size() / 7);

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);

        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
                     verts.data(), GL_STATIC_DRAW);

        constexpr GLsizei stride = 7 * sizeof(float);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(6 * sizeof(float)));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);
    }

public:
    TerrainMesh() = default;
    ~TerrainMesh() { destroy(); }

    TerrainMesh(const TerrainMesh&) = delete;
    TerrainMesh& operator=(const TerrainMesh&) = delete;

    void init() {
        if (initialized_) return;
        build();
        initialized_ = true;
    }

    void destroy() noexcept {
        if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
        if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
        vertex_count_ = 0;
        initialized_ = false;
    }

    void draw() const noexcept {
        if (!vao_) return;
        glBindVertexArray(vao_);
        glDrawArrays(GL_TRIANGLES, 0, vertex_count_);
        glBindVertexArray(0);
    }

    [[nodiscard]] GLsizei vertex_count() const noexcept { return vertex_count_; }
    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }
};

} // namespace fastjet::graphics
