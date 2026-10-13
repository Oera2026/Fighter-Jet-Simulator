#pragma once

#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/shader.hpp"
#include <array>
#include <cmath>
#include <vector>

namespace fastjet::graphics {

/// @brief Vertex structure for 3D cockpit geometry
struct Vertex3D {
    float pos[3];
    float normal[3];
    float uv[2];
    float color[4];
};

/// @brief A display surface in body axes: corners top-left, top-right,
/// bottom-right, bottom-left, and the instrument-atlas cell it shows.
struct ScreenQuad {
    std::array<std::array<float, 3>, 4> corners{};
    float u0 = 0.0f, u1 = 1.0f; ///< Atlas U, left to right
    float v0 = 0.0f, v1 = 1.0f; ///< Atlas V, bottom to top
    float brightness = 1.0f;    ///< Output scale: 1 for self-lit displays
};

/// @brief Low-poly F-16 cockpit shell, combiner glass polygon, and instrument MFD panels
class CockpitGeometry {
private:
    GLuint shell_vao_ = 0;
    GLuint shell_vbo_ = 0;
    GLsizei shell_vertex_count_ = 0;

    GLuint mfd_vao_ = 0;
    GLuint mfd_vbo_ = 0;
    GLsizei mfd_vertex_count_ = 0;

    GLuint glass_vao_ = 0;
    GLuint glass_vbo_ = 0;
    GLsizei glass_vertex_count_ = 0;

    GLuint gauge_vao_ = 0;
    GLuint gauge_vbo_ = 0;
    GLsizei gauge_vertex_count_ = 0;

    bool initialized_ = false;

    // The built-in shell, displays and combiner, kept so they can be
    // restored after another airframe's cockpit has been loaded.
    std::vector<Vertex3D> default_shell_;
    std::vector<Vertex3D> default_mfd_;
    std::vector<Vertex3D> default_glass_;

    /// @brief Replaces a buffer's vertices (its vertex array keeps its layout).
    static GLsizei upload_vertices(GLuint vbo, const std::vector<Vertex3D>& v) {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(Vertex3D)), v.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        return static_cast<GLsizei>(v.size());
    }

    void build_cockpit_shell() {
        std::vector<Vertex3D> vertices;

        auto add_quad = [&](const float p0[3], const float p1[3], const float p2[3], const float p3[3],
                            const float norm[3], const Color4& col) {
            Vertex3D v0{{p0[0], p0[1], p0[2]}, {norm[0], norm[1], norm[2]}, {0.0f, 0.0f}, {col.r, col.g, col.b, col.a}};
            Vertex3D v1{{p1[0], p1[1], p1[2]}, {norm[0], norm[1], norm[2]}, {1.0f, 0.0f}, {col.r, col.g, col.b, col.a}};
            Vertex3D v2{{p2[0], p2[1], p2[2]}, {norm[0], norm[1], norm[2]}, {1.0f, 1.0f}, {col.r, col.g, col.b, col.a}};
            Vertex3D v3{{p3[0], p3[1], p3[2]}, {norm[0], norm[1], norm[2]}, {0.0f, 1.0f}, {col.r, col.g, col.b, col.a}};

            // Triangle 1: 0, 1, 2
            vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v2);
            // Triangle 2: 0, 2, 3
            vertices.push_back(v0); vertices.push_back(v2); vertices.push_back(v3);
        };

        // Interior colours are linear reflectances to match the world shaders.
        // Real F-16 cockpits are FS 36231 dark gull grey with a near-black
        // anti-glare shield; both are much darker than they look in photographs.
        const Color4 panel_grey  {0.035f, 0.038f, 0.042f, 1.0f};
        const Color4 glare_black {0.014f, 0.014f, 0.016f, 1.0f};
        const Color4 console_grey{0.030f, 0.032f, 0.035f, 1.0f};
        const Color4 rail_grey   {0.045f, 0.047f, 0.050f, 1.0f};
        const Color4 stick_black {0.020f, 0.020f, 0.022f, 1.0f};
        const Color4 seat_grey   {0.026f, 0.027f, 0.030f, 1.0f};

        // Panel fittings. Keys and bezels are moulded in a lighter grey than
        // the panel face, which is what makes the button rows legible against
        // it under cockpit lighting.
        const Color4 button_grey    {0.058f, 0.060f, 0.063f, 1.0f};
        const Color4 gauge_face     {0.048f, 0.050f, 0.052f, 1.0f};

        // Self-lit fittings. These are emissive in the real cockpit, so they
        // are given reflectances well above the surrounding paint; the shell
        // shader's ambient term then keeps them visible in shadow.
        const Color4 scratchpad_green {0.030f, 0.190f, 0.070f, 1.0f};
        const Color4 indicator_green  {0.040f, 0.230f, 0.090f, 1.0f};
        const Color4 caution_amber    {0.240f, 0.150f, 0.020f, 1.0f};
        const Color4 guard_red        {0.200f, 0.030f, 0.025f, 1.0f};
        const Color4 switch_silver    {0.090f, 0.092f, 0.095f, 1.0f};

        // Convenience wrapper: quad from four explicit corners.
        auto quad = [&](float ax, float ay, float az, float bx, float by, float bz,
                        float cx, float cy, float cz, float dx, float dy, float dz,
                        float nx, float ny, float nz, const Color4& col) {
            const float a[3]{ax, ay, az}, b[3]{bx, by, bz}, c[3]{cx, cy, cz}, d[3]{dx, dy, dz};
            const float n[3]{nx, ny, nz};
            add_quad(a, b, c, d, n, col);
        };

        // -------------------------------------------------------------------
        // 1. Glare shield: the anti-glare coaming that wraps the top of the
        //    instrument panel and carries the HUD combiner.
        // -------------------------------------------------------------------
        // The shield meets the top edge of the instrument panel at Z = -0.72 and
        // runs forward to the coaming lip, passing just under the combiner.
        quad(2.40f, -0.38f, -0.62f,  2.40f,  0.38f, -0.62f,
             2.52f,  0.32f, -0.66f,  2.52f, -0.32f, -0.66f,
             0.0f, 0.0f, -1.0f, glare_black);

        // Coaming lip: a raised front edge, so the shield reads as a solid
        // moulding rather than a painted rectangle.
        quad(2.52f, -0.32f, -0.66f,  2.52f,  0.32f, -0.66f,
             2.54f,  0.32f, -0.60f,  2.54f, -0.32f, -0.60f,
             -0.30f, 0.0f, -0.95f, glare_black);

        // Shield sides, closing the gap down to the panel face.
        quad(2.40f, -0.38f, -0.62f,  2.52f, -0.32f, -0.66f,
             2.52f, -0.32f, -0.58f,  2.40f, -0.38f, -0.56f,
             0.0f, -1.0f, 0.0f, glare_black);
        quad(2.40f,  0.38f, -0.62f,  2.40f,  0.38f, -0.56f,
             2.52f,  0.32f, -0.58f,  2.52f,  0.32f, -0.66f,
             0.0f, 1.0f, 0.0f, glare_black);

        // -------------------------------------------------------------------
        // 2. Main instrument panel, canted back toward the pilot.
        // -------------------------------------------------------------------
        // The panel face is one plane, tilted so its top edge (under the glare
        // shield) sits further forward than its bottom edge. Every fitting
        // below is placed on that plane and floated a few millimetres proud of
        // it, so the panel reads as a machined assembly of separate boxes
        // rather than a painted flat.
        //
        // Panel plane: X = 2.40 at Z = -0.62, X = 2.12 at Z = -0.32.
        //
        // The plane is set by what the pilot can actually see. From the design
        // eye point at Z = -0.65 with a 60 deg vertical FOV, the panel has to
        // start just *below* eye level (Z greater than -0.65) or it looms into
        // the forward view, and anything much below Z = -0.42 falls off the
        // bottom of the screen. That leaves a usable band of about 20 cm, so
        // the panel is raked well forward and the whole layout is fitted into
        // it. A flatter, deeper panel would be more faithful to the airframe
        // but would put both MFDs in the pilot's lap, out of frame.
        auto panel_x = [](float z, float proud) {
            const float tt = (z - (-0.62f)) / ((-0.32f) - (-0.62f));
            return 2.40f + tt * (2.12f - 2.40f) - proud;
        };

        // Backing face.
        quad(2.40f, -0.40f, -0.62f,  2.40f,  0.40f, -0.62f,
             2.12f,  0.40f, -0.32f,  2.12f, -0.40f, -0.32f,
             -0.73f, 0.0f, -0.68f, panel_grey);

        // A raised box on the panel face: front plate plus a thin surround so
        // it catches the light along its edges.
        auto panel_box = [&](float y_c, float z_c, float half_w, float half_h,
                             float proud, const Color4& face_col) {
            const float y0 = y_c - half_w, y1 = y_c + half_w;
            const float z0 = z_c - half_h, z1 = z_c + half_h;
            const float x0 = panel_x(z0, proud);
            const float x1 = panel_x(z1, proud);
            quad(x0, y0, z0,  x0, y1, z0,
                 x1, y1, z1,  x1, y0, z1,
                 -1.0f, 0.0f, 0.0f, face_col);
        };

        // ---- Display bezels ------------------------------------------------
        // Three screens, as on a Block 50: two MFDs at the top flanking the
        // upfront controls, and the large colour display low and centre.
        const float mfd_half   = 0.060f;
        const float mfd_z      = -0.552f;
        const float mfd_y_off  = 0.215f;

        for (float side : {-1.0f, 1.0f}) {
            // Bezel surround, slightly larger than the glass it frames.
            panel_box(side * mfd_y_off, mfd_z, mfd_half + 0.020f, mfd_half + 0.020f,
                      0.004f, rail_grey);
            // Recessed well the screen sits in.
            panel_box(side * mfd_y_off, mfd_z, mfd_half + 0.003f, mfd_half + 0.003f,
                      0.001f, glare_black);
        }

        // Centre colour display: bigger than the MFDs and set lower, where the
        // reference cockpit carries its horizontal situation page.
        const float cpd_half_w = 0.070f;
        const float cpd_half_h = 0.040f;
        const float cpd_z      = -0.470f;
        panel_box(0.0f, cpd_z, cpd_half_w + 0.022f, cpd_half_h + 0.020f, 0.004f, rail_grey);
        panel_box(0.0f, cpd_z, cpd_half_w + 0.003f, cpd_half_h + 0.003f, 0.001f, glare_black);

        // ---- Option-select buttons around each display ---------------------
        // The row of 20 small keys ringing every MFD. They are what makes the
        // panel read as an F-16 rather than a generic dashboard, so they are
        // modelled individually rather than painted on.
        auto osb_ring = [&](float y_c, float z_c, float half_w, float half_h) {
            const float key_w = 0.009f;
            const float key_h = 0.007f;
            const float gap   = 0.004f;   // clear of the bezel edge

            for (int i = 0; i < 5; ++i) {
                const float f = (static_cast<float>(i) - 2.0f) * (half_w * 0.42f);
                // Top and bottom rows.
                panel_box(y_c + f, z_c - half_h - gap - key_h, key_w, key_h, 0.008f, button_grey);
                panel_box(y_c + f, z_c + half_h + gap + key_h, key_w, key_h, 0.008f, button_grey);
                // Left and right columns.
                const float g = (static_cast<float>(i) - 2.0f) * (half_h * 0.42f);
                panel_box(y_c - half_w - gap - key_w, z_c + g, key_w, key_h, 0.008f, button_grey);
                panel_box(y_c + half_w + gap + key_w, z_c + g, key_w, key_h, 0.008f, button_grey);
            }
        };

        for (float side : {-1.0f, 1.0f}) {
            osb_ring(side * mfd_y_off, mfd_z, mfd_half + 0.028f, mfd_half + 0.028f);
        }
        osb_ring(0.0f, cpd_z, cpd_half_w + 0.030f, cpd_half_h + 0.030f);

        // ---- Integrated Control Panel (ICP) --------------------------------
        // The upfront control keypad between the two MFDs: a scratchpad display
        // over a 4x5 key matrix, with the rocker switches down either side.
        {
            const float icp_z = -0.556f;

            // Housing.
            panel_box(0.0f, icp_z, 0.070f, 0.062f, 0.004f, rail_grey);

            // Scratchpad: the small green readout showing UHF/nav data.
            panel_box(0.0f, icp_z - 0.038f, 0.058f, 0.018f, 0.008f, scratchpad_green);

            // Keypad: four rows of five keys, below the scratchpad.
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 5; ++col) {
                    const float ky = (static_cast<float>(col) - 2.0f) * 0.025f;
                    const float kz = icp_z - 0.008f + static_cast<float>(row) * 0.017f;
                    panel_box(ky, kz, 0.009f, 0.006f, 0.009f, button_grey);
                }
            }
        }

        // ---- Threat warning panel, upper left ------------------------------
        // The round RWR azimuth scope and its caution lights.
        {
            const float rwr_y = -0.322f;
            const float rwr_z = -0.588f;
            panel_box(rwr_y, rwr_z, 0.040f, 0.036f, 0.004f, rail_grey);
            panel_box(rwr_y, rwr_z, 0.033f, 0.029f, 0.001f, glare_black);

            // Threat caution lamps beneath it.
            for (int i = 0; i < 3; ++i) {
                const float ly = rwr_y - 0.024f + static_cast<float>(i) * 0.024f;
                panel_box(ly, rwr_z + 0.048f, 0.009f, 0.006f, 0.007f, caution_amber);
            }
        }

        // ---- Engine and fuel gauge stack, right side -----------------------
        // Oil pressure, fuel flow, RPM, nozzle position and FTIT, stacked in a
        // column outboard of the right MFD, plus the fuel totalizer below.
        {
            const float gy = 0.338f;
            for (int i = 0; i < 4; ++i) {
                const float gz = -0.598f + static_cast<float>(i) * 0.028f;
                panel_box(gy, gz, 0.030f, 0.012f, 0.004f, rail_grey);
                panel_box(gy, gz, 0.025f, 0.008f, 0.002f, gauge_face);
            }

            // Fuel quantity totalizer: a taller box with its own bezel.
            panel_box(gy, -0.470f, 0.030f, 0.016f, 0.004f, rail_grey);
            panel_box(gy, -0.470f, 0.025f, 0.011f, 0.002f, scratchpad_green);
        }

        // ---- Standby flight instruments, left of the centre display --------
        // The mechanical backups: standby ADI, altimeter and airspeed.
        {
            const float sy = -0.330f;
            for (int i = 0; i < 3; ++i) {
                const float sz = -0.500f + static_cast<float>(i) * 0.028f;
                panel_box(sy, sz, 0.026f, 0.012f, 0.004f, rail_grey);
                panel_box(sy, sz, 0.021f, 0.009f, 0.002f, gauge_face);
            }
        }

        // ---- Landing gear and caution panel, lower left --------------------
        {
            // Gear handle housing.
            panel_box(-0.352f, -0.432f, 0.026f, 0.018f, 0.004f, rail_grey);
            // The wheel-shaped handle itself.
            panel_box(-0.352f, -0.432f, 0.011f, 0.012f, 0.014f, button_grey);

            // Three green gear-down indicator lamps above it.
            for (int i = 0; i < 3; ++i) {
                const float ly = -0.374f + static_cast<float>(i) * 0.022f;
                panel_box(ly, -0.466f, 0.008f, 0.005f, 0.007f, indicator_green);
            }
        }

        // -------------------------------------------------------------------
        // 3. Canopy sill rails running fore/aft either side of the pilot. These
        //    are what give the cockpit its sense of enclosure in peripheral
        //    vision, and they frame the view out of the bubble canopy.
        // -------------------------------------------------------------------
        // The design eye point is at Z = -0.65, so the sill must sit BELOW that
        // (less negative Z) to read as a rail the pilot looks over rather than a
        // strut crossing the field of view. It rises gently toward the seat.
        for (float side : {-1.0f, 1.0f}) {
            const float y_in  = side * 0.42f;
            const float y_out = side * 0.54f;

            // Rail top surface: level with the shoulder line, sloping down
            // toward the nose so it never intrudes on the forward view.
            quad(2.24f, y_in, -0.44f,  2.24f, y_out, -0.44f,
                 0.90f, y_out, -0.60f,  0.90f, y_in, -0.60f,
                 0.0f, 0.0f, -1.0f, rail_grey);

            // Inboard face dropping into the tub, visible looking sideways.
            quad(2.24f, y_in, -0.44f,  0.90f, y_in, -0.60f,
                 0.90f, y_in, -0.20f,  2.24f, y_in, -0.16f,
                 -side, 0.0f, 0.0f, console_grey);
        }

        // -------------------------------------------------------------------
        // 4. Side consoles: throttle quadrant to the left, switch panels right.
        // -------------------------------------------------------------------
        for (float side : {-1.0f, 1.0f}) {
            const float y_in  = side * 0.22f;
            const float y_out = side * 0.40f;

            // Console deck.
            quad(2.12f, y_in, -0.24f,  2.12f, y_out, -0.26f,
                 1.05f, y_out, -0.30f,  1.05f, y_in, -0.28f,
                 0.0f, 0.0f, -1.0f, console_grey);

            // Inboard wall dropping into the tub.
            quad(2.12f, y_in, -0.24f,  1.05f, y_in, -0.28f,
                 1.05f, y_in, -0.05f,  2.12f, y_in, -0.05f,
                 -side, 0.0f, 0.0f, panel_grey);
        }


        // ---- Console switch panels -----------------------------------------
        // Boxes and switch bodies sitting proud of the console decks. The deck
        // slopes gently down toward the seat, so each fitting is placed by
        // interpolating along it rather than at a fixed height.
        //
        // Deck plane, per side: X = 2.12 at Z = -0.25, X = 1.05 at Z = -0.29.
        auto deck_z = [](float x) {
            const float tt = (x - 1.05f) / (2.12f - 1.05f);
            return -0.29f + tt * ((-0.25f) - (-0.29f));
        };

        // Raised box on a console deck, lying in the horizontal plane.
        auto deck_box = [&](float x_c, float y_c, float half_l, float half_w,
                            float height, const Color4& col) {
            const float z = deck_z(x_c) - height;
            quad(x_c - half_l, y_c - half_w, z,  x_c - half_l, y_c + half_w, z,
                 x_c + half_l, y_c + half_w, z,  x_c + half_l, y_c - half_w, z,
                 0.0f, 0.0f, -1.0f, col);
            // Inboard side wall, so the box has visible thickness.
            const float z_base = deck_z(x_c);
            quad(x_c - half_l, y_c - half_w, z,  x_c + half_l, y_c - half_w, z,
                 x_c + half_l, y_c - half_w, z_base,  x_c - half_l, y_c - half_w, z_base,
                 0.0f, -1.0f, 0.0f, col);
        };

        // Left console, forward to aft: emergency stores jettison under its red
        // guard, laser arm, master arm, then the autopilot panel.
        deck_box(2.02f, -0.300f, 0.030f, 0.038f, 0.012f, panel_grey);
        deck_box(2.02f, -0.300f, 0.016f, 0.020f, 0.026f, guard_red);   // guarded button

        deck_box(1.93f, -0.300f, 0.026f, 0.040f, 0.010f, panel_grey);
        for (int i = 0; i < 2; ++i) {                                   // laser / arm toggles
            deck_box(1.93f, -0.320f + static_cast<float>(i) * 0.040f, 0.006f, 0.006f, 0.024f, switch_silver);
        }

        deck_box(1.84f, -0.300f, 0.028f, 0.042f, 0.010f, panel_grey);   // master arm
        deck_box(1.84f, -0.312f, 0.007f, 0.007f, 0.028f, switch_silver);
        deck_box(1.84f, -0.288f, 0.010f, 0.014f, 0.018f, indicator_green);

        deck_box(1.36f, -0.300f, 0.040f, 0.044f, 0.010f, panel_grey);   // autopilot panel
        for (int i = 0; i < 2; ++i) {
            deck_box(1.36f, -0.322f + static_cast<float>(i) * 0.044f, 0.008f, 0.007f, 0.026f, switch_silver);
        }

        // Right console: avionics and electrical switch banks.
        deck_box(2.00f, 0.300f, 0.032f, 0.044f, 0.010f, panel_grey);
        for (int i = 0; i < 3; ++i) {
            deck_box(2.00f, 0.262f + static_cast<float>(i) * 0.038f, 0.007f, 0.007f, 0.026f, switch_silver);
        }

        deck_box(1.88f, 0.300f, 0.030f, 0.044f, 0.010f, panel_grey);
        for (int i = 0; i < 3; ++i) {
            deck_box(1.88f, 0.262f + static_cast<float>(i) * 0.038f, 0.007f, 0.007f, 0.022f, switch_silver);
        }

        // ---- Lower-left auxiliary panel -------------------------------------
        // Pitot heat, brake channel, parking brake and anti-skid, on the canted
        // face below the left edge of the main instrument panel. These sit on
        // their own small plane inboard of the console, angled toward the pilot.
        {
            const float ay = -0.375f;
            for (int i = 0; i < 4; ++i) {
                const float ax = 2.06f - static_cast<float>(i) * 0.055f;
                quad(ax - 0.024f, ay, -0.175f,  ax + 0.024f, ay, -0.175f,
                     ax + 0.024f, ay + 0.070f, -0.150f,  ax - 0.024f, ay + 0.070f, -0.150f,
                     0.0f, -0.42f, -0.91f, panel_grey);
            }
            // Parking brake handle: the one fitting on this panel that stands
            // out, as it does in the reference.
            quad(1.895f, ay + 0.020f, -0.196f,  1.945f, ay + 0.020f, -0.196f,
                 1.945f, ay + 0.048f, -0.188f,  1.895f, ay + 0.048f, -0.188f,
                 0.0f, -0.42f, -0.91f, guard_red);
        }

        // Throttle grip on the left console, roughly where the real one sits.
        quad(1.72f, -0.34f, -0.30f,  1.72f, -0.24f, -0.30f,
             1.52f, -0.24f, -0.33f,  1.52f, -0.34f, -0.33f,
             0.0f, 0.0f, -1.0f, stick_black);
        quad(1.72f, -0.34f, -0.30f,  1.52f, -0.34f, -0.33f,
             1.52f, -0.34f, -0.21f,  1.72f, -0.34f, -0.21f,
             0.0f, -1.0f, 0.0f, stick_black);

        // -------------------------------------------------------------------
        // 5. Sidestick controller on the right console. The F-16's defining
        //    cockpit feature, and a useful visual anchor for roll cues.
        // -------------------------------------------------------------------
        quad(1.62f, 0.24f, -0.30f,  1.62f, 0.34f, -0.30f,
             1.50f, 0.34f, -0.31f,  1.50f, 0.24f, -0.31f,
             0.0f, 0.0f, -1.0f, stick_black);
        quad(1.60f, 0.26f, -0.31f,  1.60f, 0.32f, -0.31f,
             1.57f, 0.32f, -0.46f,  1.57f, 0.26f, -0.46f,
             -1.0f, 0.0f, -0.15f, stick_black);
        quad(1.57f, 0.26f, -0.46f,  1.57f, 0.32f, -0.46f,
             1.60f, 0.32f, -0.48f,  1.60f, 0.26f, -0.48f,
             0.0f, 0.0f, -1.0f, stick_black);

        // -------------------------------------------------------------------
        // 6. Seat surround: the headrest and shoulder line at the edge of
        //    vision, which stops the view feeling like a floating camera.
        // -------------------------------------------------------------------
        quad(1.05f, -0.30f, -0.30f,  1.05f,  0.30f, -0.30f,
             1.02f,  0.30f, -1.05f,  1.02f, -0.30f, -1.05f,
             1.0f, 0.0f, -0.05f, seat_grey);

        // Canopy bow: the forward arch where the windscreen meets the bubble.
        //
        // The real F-16 has a frameless one-piece canopy, so the only structure
        // in the pilot's forward field of view is this thin bow well above the
        // glare shield. Anything heavier reads as a strut hanging in mid-air.
        // Kept thin and high: at 60 deg FOV this sits near the top edge of the
        // screen, framing the view the way the real canopy does without
        // blocking the HUD or the over-the-nose picture.
        const float bow_z = -1.02f;
        quad(2.34f, -0.30f, bow_z,          2.34f,  0.30f, bow_z,
             2.28f,  0.30f, bow_z + 0.04f,  2.28f, -0.30f, bow_z + 0.04f,
             -0.4f, 0.0f, -0.9f, rail_grey);

        shell_vertex_count_ = static_cast<GLsizei>(vertices.size());
        default_shell_ = vertices;

        glGenVertexArrays(1, &shell_vao_);
        glGenBuffers(1, &shell_vbo_);

        glBindVertexArray(shell_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, shell_vbo_);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex3D), vertices.data(), GL_STATIC_DRAW);

        // Layout: pos(3), normal(3), uv(2), color(4)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, pos));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, normal));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, uv));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, color));
        glEnableVertexAttribArray(3);

        glBindVertexArray(0);
    }

    void build_mfd_panel() {
        // The three display glasses. Each is a quad lying on the instrument
        // panel plane, textured from the region of the shared instrument atlas
        // that its page is drawn into:
        //   left MFD   -> UV [0.00-0.50] x [0.50-1.00]
        //   right MFD  -> UV [0.50-1.00] x [0.50-1.00]
        //   centre CPD -> UV [0.25-0.75] x [0.00-0.50]
        //
        // Positions and sizes mirror the recessed wells cut for them in
        // build_cockpit_shell(), inset slightly so the bezel edge stays visible
        // all the way around the glass.
        std::vector<Vertex3D> vertices;

        const Color4 white{1.0f, 1.0f, 1.0f, 1.0f};
        const float norm[3] = {-1.0f, 0.0f, 0.0f};

        // Panel plane, matching the shell: X = 2.40 at Z = -0.62 to X = 2.12 at Z = -0.32.
        auto panel_x = [](float z, float proud) {
            const float tt = (z - (-0.62f)) / ((-0.32f) - (-0.62f));
            return 2.40f + tt * (2.12f - 2.40f) - proud;
        };

        // Screen quad with explicit UV bounds into the atlas. Z increases
        // downward in body coordinates, so the top edge of the screen is the
        // more negative Z and takes the upper (v1) edge of the texture cell.
        auto add_screen = [&](float y_c, float z_c, float half_w, float half_h,
                              float u0, float u1, float v0, float v1) {
            const float y_l = y_c - half_w, y_r = y_c + half_w;
            const float z_t = z_c - half_h, z_b = z_c + half_h;
            // The bezel plate around each display is drawn proud of the panel
            // face in build_cockpit_shell(); the glass has to sit in front of
            // it or the bezel simply covers the screen.
            const float x_t = panel_x(z_t, 0.007f);
            const float x_b = panel_x(z_b, 0.007f);

            Vertex3D tl{{x_t, y_l, z_t}, {norm[0], norm[1], norm[2]}, {u0, v1}, {white.r, white.g, white.b, white.a}};
            Vertex3D tr{{x_t, y_r, z_t}, {norm[0], norm[1], norm[2]}, {u1, v1}, {white.r, white.g, white.b, white.a}};
            Vertex3D br{{x_b, y_r, z_b}, {norm[0], norm[1], norm[2]}, {u1, v0}, {white.r, white.g, white.b, white.a}};
            Vertex3D bl{{x_b, y_l, z_b}, {norm[0], norm[1], norm[2]}, {u0, v0}, {white.r, white.g, white.b, white.a}};

            vertices.push_back(tl); vertices.push_back(tr); vertices.push_back(br);
            vertices.push_back(tl); vertices.push_back(br); vertices.push_back(bl);
        };

        // Left MFD.
        add_screen(-0.215f, -0.552f, 0.060f, 0.060f,  0.00f, 0.50f, 0.50f, 1.00f);
        // Right MFD.
        add_screen( 0.215f, -0.552f, 0.060f, 0.060f,  0.50f, 1.00f, 0.50f, 1.00f);
        // Centre colour display.
        add_screen( 0.000f, -0.470f, 0.070f, 0.040f,  0.25f, 0.75f, 0.00f, 0.50f);

        mfd_vertex_count_ = static_cast<GLsizei>(vertices.size());
        default_mfd_ = vertices;

        glGenVertexArrays(1, &mfd_vao_);
        glGenBuffers(1, &mfd_vbo_);

        glBindVertexArray(mfd_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, mfd_vbo_);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex3D), vertices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, pos));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, normal));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, uv));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, color));
        glEnableVertexAttribArray(3);

        glBindVertexArray(0);
    }

    /// @brief Uploads display quads into `vbo`, replacing its contents.
    /// @return Vertex count uploaded
    static GLsizei upload_quads(GLuint vbo, const std::vector<ScreenQuad>& screens) {
        std::vector<Vertex3D> vertices;
        for (const ScreenQuad& q : screens) {
            // Face normal from the edges, pointing at the pilot.
            const auto& c = q.corners;
            const float ex[3] = {c[1][0] - c[0][0], c[1][1] - c[0][1], c[1][2] - c[0][2]};
            const float ey[3] = {c[3][0] - c[0][0], c[3][1] - c[0][1], c[3][2] - c[0][2]};
            float n[3] = {ex[1] * ey[2] - ex[2] * ey[1], ex[2] * ey[0] - ex[0] * ey[2], ex[0] * ey[1] - ex[1] * ey[0]};
            const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (len > 0.0f) { n[0] /= len; n[1] /= len; n[2] /= len; }
            if (n[0] > 0.0f) { n[0] = -n[0]; n[1] = -n[1]; n[2] = -n[2]; }
            const float uv[4][2] = {{q.u0, q.v1}, {q.u1, q.v1}, {q.u1, q.v0}, {q.u0, q.v0}};
            const float b = q.brightness;
            auto vert = [&](int i) {
                return Vertex3D{{c[i][0], c[i][1], c[i][2]}, {n[0], n[1], n[2]}, {uv[i][0], uv[i][1]}, {b, b, b, 1.0f}};
            };
            for (int i : {0, 1, 2, 0, 2, 3}) vertices.push_back(vert(i));
        }
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex3D)), vertices.data(),
                     GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        return static_cast<GLsizei>(vertices.size());
    }

    /// @brief VAO over `vbo` with the Vertex3D layout.
    static void make_vertex_array(GLuint& vao, GLuint& vbo) {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, pos));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, normal));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, uv));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, color));
        glEnableVertexAttribArray(3);
        glBindVertexArray(0);
    }

    void build_combiner_glass() {
        // Physical HUD Combiner Glass Polygon (tilted trapezoid in front of pilot eye):
        // Eye is at DEP: X=1.80, Y=0.0, Z=-0.65
        // Mounted atop the glare shield coaming lip (lip is at X=2.54, Z=-0.60 to X=2.40, Z=-0.62)
        // Base terminates cleanly atop the coaming at X=2.33, Z=-0.635 to prevent clipping into dashboard/ICP
        // Width: 18 cm (Y = [-0.09, +0.09])
        std::vector<Vertex3D> vertices;

        // Bottom edge (sits cleanly atop glare shield coaming):
        const float b_left[3]  = {2.33f, -0.09f, -0.635f};
        const float b_right[3] = {2.33f,  0.09f, -0.635f};
        // Top edge (slanted forward/upward):
        const float t_left[3]  = {2.38f, -0.09f, -0.77f};
        const float t_right[3] = {2.38f,  0.09f, -0.77f};

        // Normal pointing back towards pilot
        const float norm[3] = {-0.89f, 0.0f, 0.45f};
        // Combiner glass subtle optical tint (light cyan-green antireflective coating)
        const Color4 glass_tint{0.10f, 0.35f, 0.30f, 0.15f};

        Vertex3D v0{{b_left[0],  b_left[1],  b_left[2]},  {norm[0], norm[1], norm[2]}, {0.0f, 0.0f}, {glass_tint.r, glass_tint.g, glass_tint.b, glass_tint.a}};
        Vertex3D v1{{b_right[0], b_right[1], b_right[2]}, {norm[0], norm[1], norm[2]}, {1.0f, 0.0f}, {glass_tint.r, glass_tint.g, glass_tint.b, glass_tint.a}};
        Vertex3D v2{{t_right[0], t_right[1], t_right[2]}, {norm[0], norm[1], norm[2]}, {1.0f, 1.0f}, {glass_tint.r, glass_tint.g, glass_tint.b, glass_tint.a}};
        Vertex3D v3{{t_left[0],  t_left[1],  t_left[2]},  {norm[0], norm[1], norm[2]}, {0.0f, 1.0f}, {glass_tint.r, glass_tint.g, glass_tint.b, glass_tint.a}};

        vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v2);
        vertices.push_back(v0); vertices.push_back(v2); vertices.push_back(v3);

        glass_vertex_count_ = static_cast<GLsizei>(vertices.size());
        default_glass_ = vertices;

        glGenVertexArrays(1, &glass_vao_);
        glGenBuffers(1, &glass_vbo_);

        glBindVertexArray(glass_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, glass_vbo_);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex3D), vertices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, pos));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, normal));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, uv));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, color));
        glEnableVertexAttribArray(3);

        glBindVertexArray(0);
    }

public:
    CockpitGeometry() = default;

    ~CockpitGeometry() {
        destroy();
    }

    void init() {
        if (!initialized_) {
            build_cockpit_shell();
            build_mfd_panel();
            build_combiner_glass();
            make_vertex_array(gauge_vao_, gauge_vbo_);
            initialized_ = true;
        }
    }

    void destroy() noexcept {
        if (shell_vao_) { glDeleteVertexArrays(1, &shell_vao_); shell_vao_ = 0; }
        if (shell_vbo_) { glDeleteBuffers(1, &shell_vbo_); shell_vbo_ = 0; }
        if (mfd_vao_) { glDeleteVertexArrays(1, &mfd_vao_); mfd_vao_ = 0; }
        if (mfd_vbo_) { glDeleteBuffers(1, &mfd_vbo_); mfd_vbo_ = 0; }
        if (glass_vao_) { glDeleteVertexArrays(1, &glass_vao_); glass_vao_ = 0; }
        if (glass_vbo_) { glDeleteBuffers(1, &glass_vbo_); glass_vbo_ = 0; }
        if (gauge_vao_) { glDeleteVertexArrays(1, &gauge_vao_); gauge_vao_ = 0; }
        if (gauge_vbo_) { glDeleteBuffers(1, &gauge_vbo_); gauge_vbo_ = 0; }
        gauge_vertex_count_ = 0;
        initialized_ = false;
    }

    /// @brief Render cockpit shell geometry
    void draw_shell() const noexcept {
        if (shell_vao_) {
            glBindVertexArray(shell_vao_);
            glDrawArrays(GL_TRIANGLES, 0, shell_vertex_count_);
            glBindVertexArray(0);
        }
    }

    /// @brief Moves the live displays onto other screens (e.g. the airframe
    /// model's own cockpit), replacing the built-in MFD layout.
    void use_screens(const std::vector<ScreenQuad>& screens) {
        if (initialized_) mfd_vertex_count_ = upload_quads(mfd_vbo_, screens);
    }

    /// @brief Places the live instrument faces (CockpitGauges atlas cells).
    void use_gauges(const std::vector<ScreenQuad>& gauges) {
        if (initialized_) gauge_vertex_count_ = upload_quads(gauge_vbo_, gauges);
    }

    [[nodiscard]] bool has_gauges() const noexcept { return gauge_vertex_count_ > 0; }

    /// @brief Replaces the cockpit shell (another airframe's interior).
    void use_shell(const std::vector<Vertex3D>& vertices) {
        if (initialized_) shell_vertex_count_ = upload_vertices(shell_vbo_, vertices);
    }

    /// @brief Replaces the HUD combiner glass (the HUD's stencil aperture).
    void use_combiner(const std::vector<Vertex3D>& vertices) {
        if (initialized_) glass_vertex_count_ = upload_vertices(glass_vbo_, vertices);
    }

    /// @brief Back to the built-in shell, displays and combiner, with no gauges.
    void restore_default() {
        if (!initialized_) return;
        shell_vertex_count_ = upload_vertices(shell_vbo_, default_shell_);
        mfd_vertex_count_ = upload_vertices(mfd_vbo_, default_mfd_);
        glass_vertex_count_ = upload_vertices(glass_vbo_, default_glass_);
        gauge_vertex_count_ = 0;
    }

    /// @brief Render MFD display quads
    void draw_mfd() const noexcept {
        if (mfd_vao_) {
            glBindVertexArray(mfd_vao_);
            glDrawArrays(GL_TRIANGLES, 0, mfd_vertex_count_);
            glBindVertexArray(0);
        }
    }

    /// @brief Render the instrument-face quads
    void draw_gauges() const noexcept {
        if (gauge_vao_ && gauge_vertex_count_ > 0) {
            glBindVertexArray(gauge_vao_);
            glDrawArrays(GL_TRIANGLES, 0, gauge_vertex_count_);
            glBindVertexArray(0);
        }
    }

    /// @brief Render HUD combiner glass polygon (for stencil mask write and glass tint)
    void draw_combiner_glass() const noexcept {
        if (glass_vao_) {
            glBindVertexArray(glass_vao_);
            glDrawArrays(GL_TRIANGLES, 0, glass_vertex_count_);
            glBindVertexArray(0);
        }
    }

    bool is_initialized() const noexcept { return initialized_; }
};

} // namespace fastjet::graphics
