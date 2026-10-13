#pragma once

#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/shader.hpp"
#include "fastjet/graphics/menu_font.hpp"
#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/ui/menu_state.hpp"
#include "fastjet/ui/theme.hpp"
#include "fastjet/input/signal_conditioner.hpp"
#include "fastjet/input/input_config.hpp"
#include "fastjet/input/input_manager.hpp"
#include <vector>
#include <string>
#include <cctype>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <chrono>

namespace fastjet::graphics {

/// @brief Vertex structure for tactical 2D vector and font UI overlay
struct MenuVertex {
    float pos[3];
    float uv[2];
    float color[4];
};

/// @brief A screen rectangle (aspect-corrected NDC) in which the renderer draws a real
/// airframe model on top of the menu.
struct PreviewSlot {
    aircraft::AircraftType type;
    float x0, y0, x1, y1;
    float heading;  ///< Yaw of the jet seen from the camera [rad]; pi/2 = nose to the right
    float fill;     ///< Fraction of the rectangle the jet's bounding sphere spans
    float pitch;    ///< Camera elevation above the jet [rad]
};

/// @brief High-utility military tactical avionics menu system
/// Conforms to authentic avionics utility, high information density,
/// Google Stitch tabular spatial grid, dual mouse and keyboard/HOTAS controls.
class AircraftMenu {
private:
    GLuint vao_triangles_ = 0;
    GLuint vbo_triangles_ = 0;
    GLuint vao_lines_     = 0;
    GLuint vbo_lines_     = 0;
    GLuint font_tex_      = 0;
    ShaderProgram shader_;
    bool initialized_ = false;

    /// Number of selectable airframes
    static constexpr int kAircraftCount = 6;

    bool is_open_ = false;
    ui::MenuContext context_ = ui::MenuContext::BOOT_DISPATCH;
    ui::MenuTab active_tab_  = ui::MenuTab::AIRFRAME_SELECT;
    int selected_index_ = 0; // [0, kAircraftCount)

    // Interactive mouse state in normalized aspect coordinates
    float mouse_ndc_x_ = -999.0f;
    float mouse_ndc_y_ = -999.0f;
    int hovered_aircraft_idx_ = -1;
    int shown_index_ = -1;        // selection the preview last animated to
    double switch_time_ = 0.0;    // when that animation began
    std::vector<PreviewSlot> preview_slots_;
    int hovered_tab_idx_ = -1;
    bool hovered_launch_btn_ = false;

    // Simulation fuel selection slider [0.10 to 1.00 capacity fraction]
    float fuel_fraction_setting_ = 1.0f;

    // Throttle detent setting preview
    float throttle_preview_val_ = 0.65f;

    // Dynamic vertex buffers
    std::vector<MenuVertex> triangle_verts_;
    std::vector<MenuVertex> line_verts_;

    // Palette, sourced from the shared ui::Theme tokens (see apply_theme()).
    // Defaults are the default theme, so the menu renders correctly even if
    // apply_theme() is never called.
    Color4 col_bg_void, col_panel_bg, col_card_header, col_highlight_bg, col_row_alt;
    Color4 col_border_dim, col_border_outer, col_border_bright;
    Color4 col_title, col_accent_gold, col_text_normal, col_text_dim, col_text_bright;
    Color4 col_active_badge, col_spec_label, col_spec_val, col_alert_red;
    Color4 col_tab_active, col_tab_inactive, col_btn_scramble, col_btn_scramble_hov;

    [[nodiscard]] static Color4 to_color(const ui::Rgba& c) noexcept { return Color4{c.r, c.g, c.b, c.a}; }

    void add_triangle(float x0, float y0, float x1, float y1, float x2, float y2, const Color4& col) {
        triangle_verts_.push_back({{x0, y0, 0.0f}, {-1.0f, -1.0f}, {col.r, col.g, col.b, col.a}});
        triangle_verts_.push_back({{x1, y1, 0.0f}, {-1.0f, -1.0f}, {col.r, col.g, col.b, col.a}});
        triangle_verts_.push_back({{x2, y2, 0.0f}, {-1.0f, -1.0f}, {col.r, col.g, col.b, col.a}});
    }

    void add_quad(float x0, float y0, float x1, float y1, const Color4& col) {
        add_triangle(x0, y0, x1, y0, x1, y1, col);
        add_triangle(x0, y0, x1, y1, x0, y1, col);
    }

    void add_thick_line(float x0, float y0, float x1, float y1, float thickness, const Color4& col) {
        float dx = x1 - x0;
        float dy = y1 - y0;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-6f) return;
        float nx = -dy / len * (thickness * 0.5f);
        float ny =  dx / len * (thickness * 0.5f);
        add_triangle(x0 + nx, y0 + ny, x0 - nx, y0 - ny, x1 - nx, y1 - ny, col);
        add_triangle(x0 + nx, y0 + ny, x1 - nx, y1 - ny, x1 + nx, y1 + ny, col);
    }

    void add_line(float x0, float y0, float x1, float y1, const Color4& col) {
        line_verts_.push_back({{x0, y0, 0.0f}, {-1.0f, -1.0f}, {col.r, col.g, col.b, col.a}});
        line_verts_.push_back({{x1, y1, 0.0f}, {-1.0f, -1.0f}, {col.r, col.g, col.b, col.a}});
    }

    void add_rect_outline(float x0, float y0, float x1, float y1, const Color4& col) {
        add_line(x0, y0, x1, y0, col);
        add_line(x1, y0, x1, y1, col);
        add_line(x1, y1, x0, y1, col);
        add_line(x0, y1, x0, y0, col);
    }

    void add_corner_brackets(float x0, float y0, float x1, float y1, float len, const Color4& col) {
        add_line(x0, y1, x0 + len, y1, col);
        add_line(x0, y1, x0, y1 - len, col);
        add_line(x1, y1, x1 - len, y1, col);
        add_line(x1, y1, x1, y1 - len, col);
        add_line(x0, y0, x0 + len, y0, col);
        add_line(x0, y0, x0, y0 + len, col);
        add_line(x1, y0, x1 - len, y0, col);
        add_line(x1, y0, x1, y0 + len, col);
    }

    void add_textured_quad(float x0, float y0, float x1, float y1,
                           float u0, float v0, float u1, float v1,
                           const Color4& col) {
        triangle_verts_.push_back({{x0, y0, 0.0f}, {u0, v1}, {col.r, col.g, col.b, col.a}});
        triangle_verts_.push_back({{x1, y0, 0.0f}, {u1, v1}, {col.r, col.g, col.b, col.a}});
        triangle_verts_.push_back({{x1, y1, 0.0f}, {u1, v0}, {col.r, col.g, col.b, col.a}});

        triangle_verts_.push_back({{x0, y0, 0.0f}, {u0, v1}, {col.r, col.g, col.b, col.a}});
        triangle_verts_.push_back({{x1, y1, 0.0f}, {u1, v0}, {col.r, col.g, col.b, col.a}});
        triangle_verts_.push_back({{x0, y1, 0.0f}, {u0, v0}, {col.r, col.g, col.b, col.a}});
    }

    [[nodiscard]] static float char_advance(char c) noexcept {
        const MenuGlyph* g = find_menu_glyph(static_cast<unsigned char>(c));
        return g ? g->advance : 0.5f;
    }

    [[nodiscard]] static float measure_string(const char* str, float scale) noexcept {
        if (!str || !*str) return 0.0f;
        float w = 0.0f;
        for (const char* p = str; *p; ++p) {
            w += char_advance(*p) * scale;
        }
        return w;
    }

    void draw_string(const char* str, float start_x, float cy, float scale, const Color4& col) {
        if (!str || !*str) return;
        const float baseline_y = cy - 0.35f * scale;
        float pen_x = start_x;
        for (const char* p = str; *p; ++p) {
            const MenuGlyph* g = find_menu_glyph(static_cast<unsigned char>(*p));
            if (!g) continue;
            if (g->w > 0.0f && g->h > 0.0f) {
                float gx0 = pen_x + g->bearing_x * scale;
                float gx1 = gx0 + g->w * scale;
                float gy1 = baseline_y + g->bearing_y * scale;
                float gy0 = gy1 - g->h * scale;
                add_textured_quad(gx0, gy0, gx1, gy1, g->u0, g->v0, g->u1, g->v1, col);
            }
            pen_x += g->advance * scale;
        }
    }

    void draw_string_centered(const char* str, float center_x, float cy, float scale, const Color4& col) {
        if (!str || !*str) return;
        draw_string(str, center_x - measure_string(str, scale) * 0.5f, cy, scale, col);
    }

    void draw_string_right(const char* str, float right_x, float cy, float scale, const Color4& col) {
        if (!str || !*str) return;
        draw_string(str, right_x - measure_string(str, scale), cy, scale, col);
    }

public:
    AircraftMenu() { apply_theme(ui::build_theme(ui::AccessibilitySettings{})); }

    /// @brief Maps the shared design tokens onto this menu's colour roles.
    void apply_theme(const ui::Theme& theme) noexcept {
        const ui::Palette& p = theme.palette;
        col_bg_void          = to_color(p.surface_void);
        col_panel_bg         = to_color(p.surface_panel);
        col_card_header      = to_color(p.surface_header);
        col_highlight_bg     = to_color(p.selection);
        col_row_alt          = to_color(p.surface_row_alt);
        col_border_dim       = to_color(p.border_subtle);
        col_border_outer     = to_color(p.border_strong);
        col_border_bright    = to_color(p.border_bright);
        col_title            = to_color(p.text_primary);
        col_accent_gold      = to_color(p.highlight);
        col_text_normal      = to_color(p.text_secondary);
        col_text_dim         = to_color(p.text_muted);
        col_text_bright      = to_color(p.accent);
        col_active_badge     = to_color(p.success);
        col_spec_label       = to_color(p.text_label);
        col_spec_val         = to_color(p.text_primary);
        col_alert_red        = to_color(p.danger);
        col_tab_active       = to_color(p.accent_strong);
        col_tab_inactive     = to_color(p.surface_sunken);
        col_btn_scramble     = to_color(p.success_fill);
        col_btn_scramble_hov = to_color(p.success_fill_hover);
    }
    ~AircraftMenu() { destroy(); }

    bool init() {
        if (initialized_) return true;

        const char* vert_src = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec2 aUV;
            layout (location = 2) in vec4 aColor;
            uniform mat4 uProjection;
            out vec2 vUV;
            out vec4 vColor;
            void main() {
                vUV = aUV;
                vColor = aColor;
                gl_Position = uProjection * vec4(aPos, 1.0);
            }
        )";

        const char* frag_src = R"(
            #version 330 core
            in vec2 vUV;
            in vec4 vColor;
            uniform sampler2D uFontTex;
            out vec4 FragColor;
            void main() {
                if (vUV.x < 0.0) {
                    FragColor = vColor;
                } else {
                    float alpha = texture(uFontTex, vUV).r;
                    FragColor = vec4(vColor.rgb, vColor.a * alpha);
                }
            }
        )";

        if (!shader_.init_from_source(vert_src, frag_src)) {
            return false;
        }

        glGenVertexArrays(1, &vao_triangles_);
        glGenBuffers(1, &vbo_triangles_);
        glBindVertexArray(vao_triangles_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_triangles_);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MenuVertex), (void*)offsetof(MenuVertex, pos));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(MenuVertex), (void*)offsetof(MenuVertex, uv));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(MenuVertex), (void*)offsetof(MenuVertex, color));
        glEnableVertexAttribArray(2);

        glGenVertexArrays(1, &vao_lines_);
        glGenBuffers(1, &vbo_lines_);
        glBindVertexArray(vao_lines_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_lines_);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MenuVertex), (void*)offsetof(MenuVertex, pos));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(MenuVertex), (void*)offsetof(MenuVertex, uv));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(MenuVertex), (void*)offsetof(MenuVertex, color));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);

        // Upload anti-aliased TrueType font atlas
        std::vector<uint8_t> atlas_pixels(kMenuFontTexW * kMenuFontTexH, 0);
        decode_menu_font_atlas(atlas_pixels.data());

        glGenTextures(1, &font_tex_);
        glBindTexture(GL_TEXTURE_2D, font_tex_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, kMenuFontTexW, kMenuFontTexH, 0, GL_RED, GL_UNSIGNED_BYTE, atlas_pixels.data());
        glBindTexture(GL_TEXTURE_2D, 0);

        initialized_ = true;
        return true;
    }

    void destroy() {
        if (font_tex_)      { glDeleteTextures(1, &font_tex_); font_tex_ = 0; }
        if (vbo_triangles_) { glDeleteBuffers(1, &vbo_triangles_); vbo_triangles_ = 0; }
        if (vao_triangles_) { glDeleteVertexArrays(1, &vao_triangles_); vao_triangles_ = 0; }
        if (vbo_lines_)     { glDeleteBuffers(1, &vbo_lines_); vbo_lines_ = 0; }
        if (vao_lines_)     { glDeleteVertexArrays(1, &vao_lines_); vao_lines_ = 0; }
        shader_.destroy();
        initialized_ = false;
    }

    bool is_open() const noexcept { return is_open_; }
    ui::MenuContext context() const noexcept { return context_; }
    ui::MenuTab active_tab() const noexcept { return active_tab_; }
    void set_tab(ui::MenuTab tab) noexcept { active_tab_ = tab; }

    void open_as_dispatch(aircraft::AircraftType current) noexcept {
        is_open_ = true;
        context_ = ui::MenuContext::BOOT_DISPATCH;
        active_tab_ = ui::MenuTab::AIRFRAME_SELECT;
        selected_index_ = static_cast<int>(current);
        shown_index_ = -1;
        if (selected_index_ < 0 || selected_index_ >= kAircraftCount) selected_index_ = 0;
    }

    void open(aircraft::AircraftType current) noexcept {
        is_open_ = true;
        context_ = ui::MenuContext::IN_FLIGHT_RECONFIG;
        active_tab_ = ui::MenuTab::AIRFRAME_SELECT;
        selected_index_ = static_cast<int>(current);
        shown_index_ = -1;
        if (selected_index_ < 0 || selected_index_ >= kAircraftCount) selected_index_ = 0;
    }

    void close() noexcept {
        is_open_ = false;
    }

    void toggle(aircraft::AircraftType current) noexcept {
        if (is_open_) {
            close();
        } else {
            open(current);
        }
    }

    void move_up() noexcept {
        selected_index_ = (selected_index_ + kAircraftCount - 1) % kAircraftCount;
    }

    void move_down() noexcept {
        selected_index_ = (selected_index_ + 1) % kAircraftCount;
    }

    void next_tab() noexcept {
        int next = (static_cast<int>(active_tab_) + 1) % static_cast<int>(ui::MenuTab::COUNT);
        active_tab_ = static_cast<ui::MenuTab>(next);
    }

    void prev_tab() noexcept {
        int count = static_cast<int>(ui::MenuTab::COUNT);
        int prev = (static_cast<int>(active_tab_) + count - 1) % count;
        active_tab_ = static_cast<ui::MenuTab>(prev);
    }

    void select_index(int idx) noexcept {
        if (idx >= 0 && idx < kAircraftCount) {
            selected_index_ = idx;
        }
    }

    /// Where the renderer should draw real airframe models this frame (filled by render()).
    [[nodiscard]] const std::vector<PreviewSlot>& preview_slots() const noexcept { return preview_slots_; }

    int selected_index() const noexcept { return selected_index_; }

    aircraft::AircraftType get_selected_type() const noexcept {
        return static_cast<aircraft::AircraftType>(selected_index_);
    }

    float fuel_fraction() const noexcept { return fuel_fraction_setting_; }
    void set_fuel_fraction(float f) noexcept { fuel_fraction_setting_ = std::clamp(f, 0.10f, 1.00f); }

    /// @brief Update mouse position in window pixels
    void update_mouse(int mouse_x, int mouse_y, int win_w, int win_h, float aspect) noexcept {
        if (win_w <= 0 || win_h <= 0) return;
        // Transform screen coords [0, win_w], [0, win_h] into NDC aspect space [-aspect, +aspect], [-1.0, +1.0]
        mouse_ndc_x_ = (static_cast<float>(mouse_x) / static_cast<float>(win_w) * 2.0f - 1.0f) * aspect;
        mouse_ndc_y_ = 1.0f - static_cast<float>(mouse_y) / static_cast<float>(win_h) * 2.0f;
    }

    /// @brief Handle mouse click on interactive menu buttons
    /// @return true if a launch/commit fly action was triggered
    bool handle_click() noexcept {
        if (!is_open_) return false;

        // 1. Check top tabs
        if (hovered_tab_idx_ >= 0 && hovered_tab_idx_ < static_cast<int>(ui::MenuTab::COUNT)) {
            active_tab_ = static_cast<ui::MenuTab>(hovered_tab_idx_);
            return false;
        }

        // 2. Check aircraft list items
        if (hovered_aircraft_idx_ >= 0 && hovered_aircraft_idx_ < kAircraftCount) {
            selected_index_ = hovered_aircraft_idx_;
            return false;
        }

        // 3. Check commit/launch button
        if (hovered_launch_btn_) {
            close();
            return true;
        }

        return false;
    }

    void render(float aspect, aircraft::AircraftType active_aircraft,
                const input::ControllerProfile* ctrl_profile = nullptr,
                const input::InputManager* input_mgr = nullptr) {
        if (!initialized_ || !is_open_) return;

        triangle_verts_.clear();
        line_verts_.clear();
        preview_slots_.clear();

        // Modal Frame Coordinates: full-density tactical HUD canvas
        const float modal_w = std::min(1.84f, aspect * 1.76f);
        const float modal_h = 1.52f;
        const float x0 = -modal_w * 0.5f;
        const float x1 =  modal_w * 0.5f;
        const float y0 = -modal_h * 0.5f;
        const float y1 =  modal_h * 0.5f;

        // Reset hover detection for this frame
        hovered_tab_idx_ = -1;
        hovered_aircraft_idx_ = -1;
        hovered_launch_btn_ = false;

        // 1. Outer Deep Canvas Backdrop & Clean Aerospace Double Bezel Frame
        add_quad(x0, y0, x1, y1, col_bg_void);
        add_rect_outline(x0, y0, x1, y1, col_border_outer);
        add_rect_outline(x0 + 0.008f, y0 + 0.008f, x1 - 0.008f, y1 - 0.008f, col_border_dim);
        add_corner_brackets(x0, y0, x1, y1, 0.06f, col_border_bright);

        // 2. Persistent Top Tactical Header (Operational Telemetry & Tab Bar)
        const float header_y = y1 - 0.14f;
        add_quad(x0 + 0.010f, header_y, x1 - 0.010f, y1 - 0.010f, col_card_header);
        add_line(x0, header_y, x1, header_y, col_border_outer);
        add_line(x0, header_y - 0.003f, x1, header_y - 0.003f, col_border_dim);

        // Title and Mode indicator
        const char* mode_tag = (context_ == ui::MenuContext::BOOT_DISPATCH)
                             ? "SYS STATUS: PRE-FLIGHT DISPATCH READINESS"
                             : "SYS STATUS: IN-FLIGHT TACTICAL RECONFIG [M]";
        draw_string("FAST JET SIMULATOR", x0 + 0.035f, y1 - 0.052f, 0.026f, col_title);
        draw_string(mode_tag, x0 + 0.035f, y1 - 0.096f, 0.015f, col_text_dim);

        // Render Clean Aerospace Tabs [AIRFRAME] [ENVELOPE] [CONTROLS] [SORTIE]
        const char* tab_names[] = {
            "[F1] AIRFRAME",
            "[F2] ENVELOPE",
            "[F3] CONTROLS",
            "[F4] SORTIE"
        };
        const int tab_count = 4;
        const float tab_total_w = 1.04f;
        const float tab_w = tab_total_w / static_cast<float>(tab_count);
        const float tab_start_x = x1 - tab_total_w - 0.025f;
        const float tab_y0 = header_y + 0.018f;
        const float tab_y1 = y1 - 0.025f;

        for (int t = 0; t < tab_count; ++t) {
            const float tx0 = tab_start_x + static_cast<float>(t) * tab_w;
            const float tx1 = tx0 + tab_w - 0.008f;
            const bool is_active_tab = (static_cast<int>(active_tab_) == t);

            // Hover detection
            const bool is_hovered = (mouse_ndc_x_ >= tx0 && mouse_ndc_x_ <= tx1 &&
                                     mouse_ndc_y_ >= tab_y0 && mouse_ndc_y_ <= tab_y1);
            if (is_hovered) hovered_tab_idx_ = t;

            const Color4 fill_col = is_active_tab ? col_tab_active
                                  : (is_hovered ? col_highlight_bg : col_tab_inactive);
            const Color4 bdr_col  = is_active_tab ? col_border_bright
                                  : (is_hovered ? col_text_bright : col_border_dim);
            const Color4 txt_col  = is_active_tab ? col_spec_val
                                  : (is_hovered ? col_title : col_text_dim);

            add_quad(tx0, tab_y0, tx1, tab_y1, fill_col);
            add_rect_outline(tx0, tab_y0, tx1, tab_y1, bdr_col);
            draw_string_centered(tab_names[t], (tx0 + tx1) * 0.5f, (tab_y0 + tab_y1) * 0.5f, 0.016f, txt_col);
        }

        // 3. Persistent Operational Footer Bar
        const float footer_y = y0 + 0.11f;
        add_quad(x0 + 0.010f, y0 + 0.010f, x1 - 0.010f, footer_y, col_card_header);
        add_line(x0, footer_y, x1, footer_y, col_border_outer);
        add_line(x0, footer_y + 0.003f, x1, footer_y + 0.003f, col_border_dim);

        // Persistent Hotkey Navigation Guide
        draw_string("1-6/ARROWS: SELECT  |  TAB: SWITCH PAGE  |  ESC/M: DISMISS",
                    x0 + 0.035f, y0 + 0.055f, 0.016f, col_text_dim);

        // Launch / Commit Flight Action Button
        const float btn_w = 0.44f;
        const float btn_h = 0.068f;
        const float btn_x1 = x1 - 0.025f;
        const float btn_x0 = btn_x1 - btn_w;
        const float btn_y0 = y0 + 0.020f;
        const float btn_y1 = btn_y0 + btn_h;

        const bool btn_hover = (mouse_ndc_x_ >= btn_x0 && mouse_ndc_x_ <= btn_x1 &&
                                mouse_ndc_y_ >= btn_y0 && mouse_ndc_y_ <= btn_y1);
        if (btn_hover) hovered_launch_btn_ = true;

        const Color4 btn_fill = btn_hover ? col_btn_scramble_hov : col_btn_scramble;
        const Color4 btn_bdr  = btn_hover ? col_active_badge : col_text_bright;
        add_quad(btn_x0, btn_y0, btn_x1, btn_y1, btn_fill);
        add_rect_outline(btn_x0, btn_y0, btn_x1, btn_y1, btn_bdr);
        add_rect_outline(btn_x0 + 0.003f, btn_y0 + 0.003f, btn_x1 - 0.003f, btn_y1 - 0.003f, col_title);

        const char* btn_label = (context_ == ui::MenuContext::BOOT_DISPATCH)
                              ? "[ENTER] COMMIT & SCRAMBLE"
                              : "[ENTER] COMMIT AIRFRAME";
        draw_string_centered(btn_label, (btn_x0 + btn_x1) * 0.5f, (btn_y0 + btn_y1) * 0.5f, 0.018f, col_spec_val);

        // 4. Content Area Layout
        const float content_top = header_y - 0.020f;
        const float content_bot = footer_y + 0.020f;

        // Render Active Tab Pane
        switch (active_tab_) {
            case ui::MenuTab::AIRFRAME_SELECT:
            default:
                render_airframe_tab(x0, x1, content_top, content_bot, active_aircraft);
                break;
            case ui::MenuTab::FLIGHT_ENVELOPE:
                render_envelope_tab(x0, x1, content_top, content_bot);
                break;
            case ui::MenuTab::HARDWARE_CALIBRATION:
                render_calibration_tab(x0, x1, content_top, content_bot, ctrl_profile, input_mgr);
                break;
            case ui::MenuTab::SORTIE_DISPATCH:
                render_sortie_tab(x0, x1, content_top, content_bot, active_aircraft);
                break;
        }

        // Upload and Render Triangles & Lines
        upload_and_draw_buffers(aspect);
    }

private:
    struct AirframeCard {
        const char* name;       ///< Stage headline
        const char* role;       ///< Two-word mission tag
        float       mach;
        float       g_limit;
        int         engines;
        float       agility, thrust, payload, stealth; ///< Relative 0..1 ratings
        const char* tags[3];    ///< Feature chips, nullptr terminated
    };

    [[nodiscard]] static const AirframeCard& airframe_card(int i) noexcept {
        static const AirframeCard cards[kAircraftCount] = {
            {"F-16C FIGHTING FALCON",  "MULTIROLE",        2.05f, 9.0f,  1, 0.90f, 0.80f, 0.60f, 0.15f, {"FBW", nullptr, nullptr}},
            {"F-15EX EAGLE II",        "AIR SUPERIORITY",  2.50f, 9.0f,  2, 0.72f, 0.90f, 1.00f, 0.15f, {"TWIN FIN", "12 AAM", nullptr}},
            {"EUROFIGHTER TYPHOON",    "SWING ROLE",       2.00f, 9.0f,  2, 0.95f, 0.90f, 0.70f, 0.30f, {"CANARD", "SUPERCRUISE", nullptr}},
            {"F-22A RAPTOR",           "STEALTH DOMINANCE",2.25f, 9.0f,  2, 1.00f, 1.00f, 0.50f, 1.00f, {"TVC", "SUPERCRUISE", "VLO"}},
            {"A-10C THUNDERBOLT II",   "GROUND ATTACK",    0.56f, 7.33f, 2, 0.50f, 0.35f, 0.90f, 0.05f, {"30MM GAU-8", "NO AB", nullptr}},
            {"F-35B LIGHTNING II",     "STOVL STRIKE",     1.60f, 7.0f,  1, 0.60f, 0.65f, 0.55f, 0.90f, {"STOVL", "LIFT FAN", nullptr}},
        };
        return cards[std::clamp(i, 0, kAircraftCount - 1)];
    }

    static constexpr float kHalfPi = 1.57079633f;

    [[nodiscard]] static double now_seconds() noexcept {
        using clock = std::chrono::steady_clock;
        return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
    }

    void draw_stage_floor(float cx, float cy, float radius, float yaw, float pitch, float alpha) {
        const float sp = std::sin(pitch), cp = std::cos(pitch);
        const float floor_z = -0.45f * radius;
        const Color4 ring{col_border_bright.r, col_border_bright.g, col_border_bright.b, 0.55f * alpha};
        const Color4 tick{col_text_bright.r, col_text_bright.g, col_text_bright.b, 0.70f * alpha};
        constexpr float kTwoPi = 6.28318530718f;
        constexpr int kSegs = 64;
        auto at = [&](float r, float a, float& px, float& py) {
            px = cx + r * std::cos(a);
            py = cy + floor_z * cp - r * std::sin(a) * sp;
        };
        float px0, py0, px1, py1;
        for (int i = 0; i < kSegs; ++i) {
            at(radius * 1.12f, kTwoPi * i / kSegs, px0, py0);
            at(radius * 1.12f, kTwoPi * (i + 1) / kSegs, px1, py1);
            add_thick_line(px0, py0, px1, py1, 0.004f, ring);
        }
        constexpr int kTicks = 36;
        for (int i = 0; i < kTicks; ++i) {
            const float a = yaw + kTwoPi * i / kTicks;
            const float len = (i % 9 == 0) ? 0.10f : 0.04f;
            at(radius * 1.12f, a, px0, py0);
            at(radius * (1.12f + len), a, px1, py1);
            add_thick_line(px0, py0, px1, py1, 0.004f, (i % 9 == 0) ? tick : ring);
        }
    }

    void render_airframe_tab(float x0, float x1, float top, float bot, aircraft::AircraftType active_aircraft) {
        const double now = now_seconds();
        if (shown_index_ != selected_index_) {
            shown_index_ = selected_index_;
            switch_time_ = now;
        }
        const float a = std::clamp(static_cast<float>((now - switch_time_) / 0.55), 0.0f, 1.0f);
        const float ease = 1.0f - (1.0f - a) * (1.0f - a) * (1.0f - a);

        const float pad = 0.020f;
        const float total_w = x1 - x0;
        const float strip_h = 0.37f;
        const float strip_top = bot + strip_h;

        // ---- Selector strip: one card per airframe -------------------------------------------
        const float gap = 0.012f;
        const float card_w = (total_w - 2.0f * pad - gap * (kAircraftCount - 1)) / kAircraftCount;
        for (int i = 0; i < kAircraftCount; ++i) {
            const float cx0 = x0 + pad + static_cast<float>(i) * (card_w + gap);
            const float cx1 = cx0 + card_w;
            const bool sel = (i == selected_index_);
            const bool hov = (mouse_ndc_x_ >= cx0 && mouse_ndc_x_ <= cx1 &&
                              mouse_ndc_y_ >= bot && mouse_ndc_y_ <= strip_top);
            if (hov) hovered_aircraft_idx_ = i;

            add_quad(cx0, bot, cx1, strip_top, sel ? col_highlight_bg : (hov ? col_row_alt : col_panel_bg));
            add_rect_outline(cx0, bot, cx1, strip_top, sel ? col_border_bright : (hov ? col_text_bright : col_border_dim));
            if (sel) add_quad(cx0, bot, cx1, bot + 0.010f, col_text_bright);

            const float yaw = sel ? kHalfPi + static_cast<float>(now * 0.6) : kHalfPi + 0.6f;
            preview_slots_.push_back({static_cast<aircraft::AircraftType>(i), cx0 + 0.008f, bot + 0.080f,
                                      cx1 - 0.008f, strip_top - 0.008f, yaw, sel || hov ? 0.95f : 0.85f, 0.45f});

            char num[4];
            std::snprintf(num, sizeof(num), "%d", i + 1);
            draw_string(num, cx0 + 0.014f, strip_top - 0.026f, 0.018f, sel ? col_text_bright : col_text_dim);
            draw_string_centered(to_short_string(static_cast<aircraft::AircraftType>(i)).data(),
                                 (cx0 + cx1) * 0.5f, bot + 0.048f, 0.020f,
                                 sel ? col_title : (hov ? col_accent_gold : col_text_normal));
            if (i == static_cast<int>(active_aircraft)) {
                add_quad(cx1 - 0.030f, strip_top - 0.030f, cx1 - 0.014f, strip_top - 0.014f, col_active_badge);
            }
        }

        // ---- Stage: rotating 3D preview of the selected airframe -----------------------------
        const AirframeCard& card = airframe_card(selected_index_);
        const float stage_x0 = x0 + pad;
        const float stage_x1 = x0 + total_w * 0.64f;
        const float stage_y0 = strip_top + 0.020f;
        const float stage_y1 = top;
        add_quad(stage_x0, stage_y0, stage_x1, stage_y1, col_panel_bg);
        add_rect_outline(stage_x0, stage_y0, stage_x1, stage_y1, col_border_dim);
        add_corner_brackets(stage_x0, stage_y0, stage_x1, stage_y1, 0.04f, col_border_bright);

        draw_string(card.name, stage_x0 + 0.030f, stage_y1 - 0.050f, 0.036f, col_title);
        draw_string(card.role, stage_x0 + 0.032f, stage_y1 - 0.098f, 0.020f, col_text_bright);

        const float view_x0 = stage_x0 + 0.010f, view_x1 = stage_x1 - 0.010f;
        const float view_y0 = stage_y0 + 0.010f, view_y1 = stage_y1 - 0.120f;
        const float stage_cx = (view_x0 + view_x1) * 0.5f;
        const float stage_cy = (view_y0 + view_y1) * 0.5f;
        const float fill = 0.86f + 0.14f * ease;
        const float radius = std::min((view_x1 - view_x0) * 0.5f, (view_y1 - view_y0) * 0.5f) * fill;
        // Spins fast on a change of selection, then settles into a slow turntable.
        const float yaw = kHalfPi + static_cast<float>(now * 0.45) + (1.0f - ease) * (1.0f - ease) * 3.2f;
        const float pitch = 0.42f;
        draw_stage_floor(stage_cx, stage_cy, radius * 0.9f, yaw, pitch, ease);
        preview_slots_.push_back({get_selected_type(), view_x0, view_y0, view_x1, view_y1, yaw, fill, pitch});

        // ---- Stats: big numbers and segmented bars -------------------------------------------
        const float p_x0 = stage_x1 + 0.015f;
        const float p_x1 = x1 - pad;
        add_quad(p_x0, stage_y0, p_x1, stage_y1, col_panel_bg);
        add_rect_outline(p_x0, stage_y0, p_x1, stage_y1, col_border_dim);

        const float tile_gap = 0.010f;
        const float tile_w = (p_x1 - p_x0 - 0.040f - 2.0f * tile_gap) / 3.0f;
        const float tile_y1 = stage_y1 - 0.025f;
        const float tile_y0 = tile_y1 - 0.170f;
        char buf[24];
        for (int t = 0; t < 3; ++t) {
            const float tx0 = p_x0 + 0.020f + static_cast<float>(t) * (tile_w + tile_gap);
            const float tx1 = tx0 + tile_w;
            add_quad(tx0, tile_y0, tx1, tile_y1, col_bg_void);
            add_rect_outline(tx0, tile_y0, tx1, tile_y1, col_border_dim);
            const float tcx = (tx0 + tx1) * 0.5f;
            if (t == 0) {
                std::snprintf(buf, sizeof(buf), "%.2f", card.mach * ease);
                draw_string_centered(buf, tcx, tile_y1 - 0.062f, 0.032f, col_title);
                draw_string_centered("MACH", tcx, tile_y0 + 0.028f, 0.014f, col_spec_label);
            } else if (t == 1) {
                std::snprintf(buf, sizeof(buf), "%.1f", card.g_limit * ease);
                draw_string_centered(buf, tcx, tile_y1 - 0.062f, 0.032f, col_title);
                draw_string_centered("G LIMIT", tcx, tile_y0 + 0.028f, 0.014f, col_spec_label);
            } else {
                const float pip = 0.026f;
                const float pips_w = static_cast<float>(card.engines) * pip + static_cast<float>(card.engines - 1) * 0.012f;
                float px = tcx - pips_w * 0.5f;
                for (int e = 0; e < card.engines; ++e) {
                    add_quad(px, tile_y1 - 0.075f, px + pip, tile_y1 - 0.075f + pip * 1.6f, col_text_bright);
                    px += pip + 0.012f;
                }
                draw_string_centered(card.engines == 1 ? "ENGINE" : "ENGINES", tcx, tile_y0 + 0.028f, 0.014f, col_spec_label);
            }
        }

        struct Bar { const char* label; float value; };
        const Bar bars[] = {
            {"SPEED",   card.mach / 2.5f},
            {"AGILITY", card.agility},
            {"THRUST",  card.thrust},
            {"PAYLOAD", card.payload},
            {"STEALTH", card.stealth},
        };
        constexpr int kSegments = 10;
        const float bar_x0 = p_x0 + 0.030f;
        const float bar_x1 = p_x1 - 0.030f;
        const float seg_gap = 0.006f;
        const float seg_w = (bar_x1 - bar_x0 - seg_gap * (kSegments - 1)) / kSegments;
        float by = tile_y0 - 0.060f;
        for (const Bar& bar : bars) {
            draw_string(bar.label, bar_x0, by, 0.016f, col_spec_label);
            const float seg_y1 = by - 0.020f;
            const float seg_y0 = seg_y1 - 0.034f;
            for (int sgm = 0; sgm < kSegments; ++sgm) {
                const float sx0 = bar_x0 + static_cast<float>(sgm) * (seg_w + seg_gap);
                const bool lit = (static_cast<float>(sgm) + 0.5f) / kSegments <= bar.value * ease;
                add_quad(sx0, seg_y0, sx0 + seg_w, seg_y1, lit ? col_text_bright : col_row_alt);
            }
            by -= 0.108f;
        }

        // ---- Feature chips ---------------------------------------------------------------------
        float chip_x = bar_x0;
        const float chip_y1 = stage_y0 + 0.060f;
        for (const char* tag : card.tags) {
            if (!tag) break;
            const float w = measure_string(tag, 0.015f) + 0.030f;
            if (chip_x + w > bar_x1) break;
            add_quad(chip_x, chip_y1 - 0.034f, chip_x + w, chip_y1, col_highlight_bg);
            add_rect_outline(chip_x, chip_y1 - 0.034f, chip_x + w, chip_y1, col_border_bright);
            draw_string(tag, chip_x + 0.015f, chip_y1 - 0.017f, 0.015f, col_spec_val);
            chip_x += w + 0.010f;
        }
    }

    void render_envelope_tab(float x0, float x1, float top, float bot) {
        // Split-screen: Left (Flight Envelope Polar Plot) and Right (Aerodynamic Limits & Derivatives)
        const float total_w = x1 - x0;
        const float split_x = x0 + total_w * 0.50f;

        // Left Card: Flight Envelope & Lift Curve
        const float c0_x0 = x0 + 0.020f;
        const float c0_x1 = split_x - 0.015f;
        add_quad(c0_x0, bot, c0_x1, top, col_panel_bg);
        add_rect_outline(c0_x0, bot, c0_x1, top, col_border_dim);
        draw_string("AERODYNAMIC POLAR & FLIGHT ENVELOPE", c0_x0 + 0.025f, top - 0.038f, 0.020f, col_title);
        add_line(c0_x0 + 0.020f, top - 0.060f, c0_x1 - 0.020f, top - 0.060f, col_border_dim);

        // Vector plot coordinates
        const float plot_x0 = c0_x0 + 0.050f;
        const float plot_x1 = c0_x1 - 0.030f;
        const float plot_y0 = bot + 0.080f;
        const float plot_y1 = top - 0.120f;

        // Grid lines
        add_rect_outline(plot_x0, plot_y0, plot_x1, plot_y1, col_border_bright);
        for (int gy = 1; gy <= 4; ++gy) {
            float py = plot_y0 + (plot_y1 - plot_y0) * (static_cast<float>(gy) / 5.0f);
            add_line(plot_x0, py, plot_x1, py, col_border_dim);
        }
        for (int gx = 1; gx <= 4; ++gx) {
            float px = plot_x0 + (plot_x1 - plot_x0) * (static_cast<float>(gx) / 5.0f);
            add_line(px, plot_y0, px, plot_y1, col_border_dim);
        }

        // Plot aerodynamic curve (Lift CL vs Alpha or Mach)
        constexpr int kPlotPoints = 32;
        float prev_px = plot_x0;
        float prev_py = plot_y0;
        for (int i = 0; i < kPlotPoints; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(kPlotPoints - 1);
            float px = plot_x0 + t * (plot_x1 - plot_x0);
            // Non-linear lift curve slope with stall plateau
            float cl = std::sin(t * 1.8f) * 1.35f;
            float py = plot_y0 + (cl / 1.5f) * (plot_y1 - plot_y0);
            if (i > 0) {
                add_line(prev_px, prev_py, px, py, col_text_bright);
            }
            prev_px = px;
            prev_py = py;
        }

        draw_string("CL MAX ENVELOPE (POLAR SLOPE 4.0/RAD)", plot_x0, plot_y1 + 0.024f, 0.016f, col_spec_label);
        draw_string("-5 DEG AOA", plot_x0, plot_y0 - 0.035f, 0.015f, col_text_dim);
        draw_string("+30 DEG AOA (STALL)", plot_x1 - 0.22f, plot_y0 - 0.035f, 0.015f, col_alert_red);

        // Right Card: Structural G Limits, Mach Divergence & Damping Derivatives
        const float c1_x0 = split_x + 0.015f;
        const float c1_x1 = x1 - 0.020f;
        add_quad(c1_x0, bot, c1_x1, top, col_panel_bg);
        add_rect_outline(c1_x0, bot, c1_x1, top, col_border_dim);
        draw_string("AIRCRAFT COEFFICIENTS & FLCS GAINS", c1_x0 + 0.025f, top - 0.038f, 0.020f, col_title);
        add_line(c1_x0 + 0.020f, top - 0.060f, c1_x1 - 0.020f, top - 0.060f, col_border_dim);

        struct CoeffEntry { const char* name; const char* val; const char* note; };
        CoeffEntry coeffs[] = {
            {"S_REF (WING AREA)",    "27.87 M^2",     "REFERENCE PLATFORM AREA"},
            {"B_SPAN (WINGSPAN)",    "9.144 M",       "TIP-TO-TIP SPAN"},
            {"C_BAR (MAC)",          "3.450 M",       "MEAN AERODYNAMIC CHORD"},
            {"ASPECT RATIO",         "3.00",          "SUPERSONIC DELTA-TRAPEZOID"},
            {"LE SWEEP ANGLE",       "40.0 DEG",      "COMPRESSIBILITY CONTROL"},
            {"CD0 (PARASITIC DRAG)", "0.0200",        "ZERO-LIFT CLEAN DRAG"},
            {"K (INDUCED DRAG)",     "0.1600",        "INDUCED FACTOR CD = CD0 + K*CL^2"},
            {"MACH CRIT / PEAK",     "0.86 / 1.05",   "TRANSONIC DRAG RISE ONSET"},
            {"MAX G ENVELOPE",       "+9.0G / -3.0G", "DIGITAL G-LIMITER INTEGRATED"},
            {"MAX ALPHA LIMIT",      "25.2 DEG",      "AUTOPILOT CAT-I LIMITER"}
        };

        const float coeff_top = top - 0.068f;
        const float c_row_h = (coeff_top - bot - 0.015f) / 10.0f;
        for (size_t idx = 0; idx < 10; ++idx) {
            const auto& ce = coeffs[idx];
            const float ry1 = coeff_top - static_cast<float>(idx) * c_row_h;
            const float ry0 = ry1 - c_row_h;
            if (idx % 2 == 1) {
                add_quad(c1_x0 + 0.005f, ry0 + 0.004f, c1_x1 - 0.005f, ry1 - 0.004f, col_row_alt);
            }
            draw_string(ce.name, c1_x0 + 0.025f, ry1 - 0.030f, 0.015f, col_spec_label);
            draw_string_right(ce.val, c1_x1 - 0.025f, ry1 - 0.030f, 0.018f, col_spec_val);
            draw_string(ce.note, c1_x0 + 0.025f, ry0 + 0.030f, 0.014f, col_text_dim);
        }
    }

    void render_calibration_tab(float x0, float x1, float top, float bot,
                                const input::ControllerProfile* ctrl_profile,
                                const input::InputManager* input_mgr) {
        // Split Left (Live Axis Oscilloscope & Curves) and Right (Detents & Input Status)
        const float total_w = x1 - x0;
        const float split_x = x0 + total_w * 0.50f;

        // Left Card: Real-time Inceptor Curve
        const float c0_x0 = x0 + 0.020f;
        const float c0_x1 = split_x - 0.015f;
        add_quad(c0_x0, bot, c0_x1, top, col_panel_bg);
        add_rect_outline(c0_x0, bot, c0_x1, top, col_border_dim);
        draw_string("FLIGHT STICK FORCE CURVE & DEADZONES", c0_x0 + 0.025f, top - 0.038f, 0.020f, col_title);
        add_line(c0_x0 + 0.020f, top - 0.060f, c0_x1 - 0.020f, top - 0.060f, col_border_dim);

        // Curve plot
        const float plot_x0 = c0_x0 + 0.050f;
        const float plot_x1 = c0_x1 - 0.030f;
        const float plot_y0 = bot + 0.080f;
        const float plot_y1 = top - 0.120f;
        const float plot_cx = (plot_x0 + plot_x1) * 0.5f;
        const float plot_cy = (plot_y0 + plot_y1) * 0.5f;

        add_rect_outline(plot_x0, plot_y0, plot_x1, plot_y1, col_border_bright);
        add_line(plot_x0, plot_cy, plot_x1, plot_cy, col_border_dim); // Center X
        add_line(plot_cx, plot_y0, plot_cx, plot_y1, col_border_dim); // Center Y

        // Deadzones
        const float db_frac = ctrl_profile ? static_cast<float>(ctrl_profile->pitch_cal.inner_deadband) : 0.03f;
        const float db_w = (plot_x1 - plot_cx) * db_frac;
        add_quad(plot_cx - db_w, plot_y0, plot_cx + db_w, plot_y1, Color4{col_alert_red.r, col_alert_red.g, col_alert_red.b, 0.15f});

        // Live Curve
        const double curv = ctrl_profile ? ctrl_profile->pitch_cal.curvature : 0.50;
        constexpr int kSteps = 40;
        float prev_x = plot_x0;
        float prev_y = plot_cy;
        for (int i = 0; i <= kSteps; ++i) {
            float norm_in = -1.0f + 2.0f * (static_cast<float>(i) / static_cast<float>(kSteps));
            float norm_out = static_cast<float>(input::SignalConditioner::evaluate_force_curve(norm_in, curv));
            float px = plot_cx + norm_in * (plot_x1 - plot_cx);
            float py = plot_cy + norm_out * (plot_y1 - plot_cy);
            if (i > 0) add_line(prev_x, prev_y, px, py, col_text_bright);
            prev_x = px;
            prev_y = py;
        }

        // Live input dot
        const float live_pitch = input_mgr ? static_cast<float>(input_mgr->pitch_in) : 0.0f;
        const float live_out   = static_cast<float>(input::SignalConditioner::evaluate_force_curve(live_pitch, curv));
        const float dot_x = plot_cx + live_pitch * (plot_x1 - plot_cx);
        const float dot_y = plot_cy + live_out * (plot_y1 - plot_cy);
        add_quad(dot_x - 0.012f, dot_y - 0.012f, dot_x + 0.012f, dot_y + 0.012f, col_active_badge);

        draw_string("F-16 TRANSDUCER CURVATURE: Y = (1-C)*X + C*X^3", plot_x0, plot_y1 + 0.024f, 0.016f, col_spec_label);
        draw_string("-100% INCEPTOR", plot_x0, plot_y0 - 0.035f, 0.015f, col_text_dim);
        draw_string("+100% INCEPTOR", plot_x1 - 0.18f, plot_y0 - 0.035f, 0.015f, col_text_dim);

        // Right Card: Throttle Detents & Hardware Calibration Options
        const float c1_x0 = split_x + 0.015f;
        const float c1_x1 = x1 - 0.020f;
        add_quad(c1_x0, bot, c1_x1, top, col_panel_bg);
        add_rect_outline(c1_x0, bot, c1_x1, top, col_border_dim);
        draw_string("THROTTLE DETENTS & CALIBRATION", c1_x0 + 0.025f, top - 0.038f, 0.020f, col_title);
        add_line(c1_x0 + 0.020f, top - 0.060f, c1_x1 - 0.020f, top - 0.060f, col_border_dim);

        // Live Throttle Bar
        const float bar_x0 = c1_x0 + 0.040f;
        const float bar_x1 = c1_x1 - 0.040f;
        const float bar_y0 = top - 0.160f;
        const float bar_y1 = top - 0.110f;
        add_quad(bar_x0, bar_y0, bar_x1, bar_y1, col_bg_void);
        add_rect_outline(bar_x0, bar_y0, bar_x1, bar_y1, col_border_bright);

        const float thr_val = input_mgr ? static_cast<float>(input_mgr->throttle_in) : throttle_preview_val_;
        const float fill_x1 = bar_x0 + (bar_x1 - bar_x0) * std::clamp(thr_val, 0.0f, 1.0f);
        add_quad(bar_x0, bar_y0, fill_x1, bar_y1, col_text_bright);

        // Detent lines
        const float detent_mil = bar_x0 + (bar_x1 - bar_x0) * 0.85f;
        add_line(detent_mil, bar_y0 - 0.015f, detent_mil, bar_y1 + 0.015f, col_active_badge);
        draw_string("MIL (85%)", detent_mil - 0.045f, bar_y1 + 0.025f, 0.015f, col_active_badge);
        draw_string("AFTERBURNER (88-100%)", bar_x1 - 0.240f, bar_y1 + 0.025f, 0.015f, col_alert_red);

        // Calibration Hotkeys
        float info_y = top - 0.230f;
        draw_string("[I] INVERT THROTTLE AXIS", c1_x0 + 0.040f, info_y, 0.016f, col_spec_label);
        draw_string("TOGGLE WHEN HARDWARE THROTTLE READS BACKWARDS", c1_x0 + 0.040f, info_y - 0.024f, 0.014f, col_text_dim);

        info_y -= 0.075f;
        draw_string("[J] HARDWARE AXIS LOCKOUT", c1_x0 + 0.040f, info_y, 0.016f, col_spec_label);
        draw_string("ISOLATES JITTERY LEVERS AND RESTORES KEYBOARD ONLY", c1_x0 + 0.040f, info_y - 0.024f, 0.014f, col_text_dim);

        info_y -= 0.075f;
        draw_string("[1/2/3/4] RAPID DETENT SLAM", c1_x0 + 0.040f, info_y, 0.016f, col_spec_label);
        draw_string("1=CUTOFF (0%) | 2=IDLE (8%) | 3=MIL (85%) | 4=MAX AB (100%)", c1_x0 + 0.040f, info_y - 0.024f, 0.014f, col_text_dim);
    }

    void render_sortie_tab(float x0, float x1, float top, float bot, aircraft::AircraftType active_aircraft) {
        (void)active_aircraft;
        // Sortie Dispatch: Mission scenario, Fuel fraction slider, Scramble Readiness
        const float total_w = x1 - x0;
        const float split_x = x0 + total_w * 0.48f;

        // Left Card: Scramble Scenario
        const float c0_x0 = x0 + 0.020f;
        const float c0_x1 = split_x - 0.015f;
        add_quad(c0_x0, bot, c0_x1, top, col_panel_bg);
        add_rect_outline(c0_x0, bot, c0_x1, top, col_border_dim);
        draw_string("MISSION SORTIE & SCENARIO SELECTION", c0_x0 + 0.025f, top - 0.038f, 0.020f, col_title);
        add_line(c0_x0 + 0.020f, top - 0.060f, c0_x1 - 0.020f, top - 0.060f, col_border_dim);

        struct Scenario { const char* title; const char* desc; };
        Scenario scens[] = {
            {"[01] RUNWAY READY (SCRAMBLE)", "HOT ENGINES RUNNING ON RUNWAY 09L - IMMEDIATE TAKEOFF"},
            {"[02] AIRBORNE PATROL (CAP)",   "INITIAL LEVEL FLIGHT @ 15,000 FT / 450 KIAS COMBAT TRIM"},
            {"[03] SUPERSONIC SPRINT",       "HIGH ALTITUDE REHEAT ENVELOPE @ 36,000 FT / MACH 1.8+"},
            {"[04] LOW-LEVEL PENETRATION",   "HIGH SPEED 200 FT TERRAIN MASKING RUN THROUGH VALLEY"}
        };

        float sc_y = top - 0.110f;
        for (size_t s = 0; s < 4; ++s) {
            const bool is_first = (s == 0);
            if (is_first) {
                add_quad(c0_x0 + 0.015f, sc_y - 0.045f, c0_x1 - 0.015f, sc_y + 0.030f, col_highlight_bg);
                add_rect_outline(c0_x0 + 0.015f, sc_y - 0.045f, c0_x1 - 0.015f, sc_y + 0.030f, col_text_bright);
            }
            draw_string(scens[s].title, c0_x0 + 0.025f, sc_y + 0.012f, 0.018f, is_first ? col_text_bright : col_title);
            draw_string(scens[s].desc,  c0_x0 + 0.025f, sc_y - 0.022f, 0.014f, is_first ? col_spec_val : col_text_dim);
            sc_y -= 0.105f;
        }

        // Right Card: Pre-Flight Mass, Fuel Load & Systems Readiness
        const float c1_x0 = split_x + 0.015f;
        const float c1_x1 = x1 - 0.020f;
        add_quad(c1_x0, bot, c1_x1, top, col_panel_bg);
        add_rect_outline(c1_x0, bot, c1_x1, top, col_border_dim);
        draw_string("FUEL STATE & PRE-FLIGHT READINESS", c1_x0 + 0.025f, top - 0.038f, 0.020f, col_title);
        add_line(c1_x0 + 0.020f, top - 0.060f, c1_x1 - 0.020f, top - 0.060f, col_border_dim);

        draw_string("INTERNAL FUEL STATE SELECTION", c1_x0 + 0.030f, top - 0.090f, 0.016f, col_spec_label);

        // Fuel Slider
        const float s_x0 = c1_x0 + 0.030f;
        const float s_x1 = c1_x1 - 0.030f;
        const float s_y0 = top - 0.150f;
        const float s_y1 = top - 0.120f;
        add_quad(s_x0, s_y0, s_x1, s_y1, col_bg_void);
        add_rect_outline(s_x0, s_y0, s_x1, s_y1, col_border_bright);

        const float s_fill = s_x0 + (s_x1 - s_x0) * fuel_fraction_setting_;
        add_quad(s_x0, s_y0, s_fill, s_y1, col_tab_active);

        char fuel_txt[64];
        std::snprintf(fuel_txt, sizeof(fuel_txt), "FUEL CAPACITY: %.0f%% (FULL TANKS)", fuel_fraction_setting_ * 100.0f);
        draw_string(fuel_txt, s_x0, s_y0 - 0.025f, 0.016f, col_spec_val);

        // Readiness Matrix
        float chk_y = top - 0.230f;
        draw_string("PRE-FLIGHT READINESS CHECKLIST:", c1_x0 + 0.030f, chk_y, 0.016f, col_spec_label);
        chk_y -= 0.045f;

        const char* checks[] = {
            "[GO] FLCS QUAD COMPUTERS : ALL CHANNELS PASS",
            "[GO] HYDRAULIC PRESSURE A/B : 3,000 PSI NOMINAL",
            "[GO] MASS BALANCE & CG : WITHIN FLIGHT ENVELOPE",
            "[GO] CANOPY & EJECTION SEAT : ARMED",
            "[GO] TURBOFAN SPOOL INTEGRITY : PASS"
        };
        for (const char* chk : checks) {
            draw_string(chk, c1_x0 + 0.030f, chk_y, 0.015f, col_text_bright);
            chk_y -= 0.038f;
        }
    }

    void upload_and_draw_buffers(float aspect) {
        // Upload Triangles
        glBindBuffer(GL_ARRAY_BUFFER, vbo_triangles_);
        glBufferData(GL_ARRAY_BUFFER, triangle_verts_.size() * sizeof(MenuVertex), triangle_verts_.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // Upload Lines
        glBindBuffer(GL_ARRAY_BUFFER, vbo_lines_);
        glBufferData(GL_ARRAY_BUFFER, line_verts_.size() * sizeof(MenuVertex), line_verts_.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // OpenGL pipeline
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        const Mat4 proj = Mat4::ortho(-aspect, aspect, -1.0f, 1.0f, -1.0f, 1.0f);

        shader_.use();
        shader_.set_mat4("uProjection", proj);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, font_tex_);
        shader_.set_int("uFontTex", 0);

        if (!triangle_verts_.empty()) {
            glBindVertexArray(vao_triangles_);
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(triangle_verts_.size()));
        }

        if (!line_verts_.empty()) {
            glLineWidth(2.0f);
            glBindVertexArray(vao_lines_);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(line_verts_.size()));
            glLineWidth(1.0f);
        }

        glBindTexture(GL_TEXTURE_2D, 0);
        glBindVertexArray(0);
        glEnable(GL_DEPTH_TEST);
    }
};

} // namespace fastjet::graphics
