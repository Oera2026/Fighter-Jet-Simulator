#pragma once

#include "fastjet/ui/screen.hpp"
#include <array>
#include <cmath>
#include <cstdio>
#include <numbers>

namespace fastjet::ui {

/// @brief Root screen: wordmark, primary actions, animated radar backdrop.
class MainMenuScreen final : public Screen {
public:
    enum Item : WidgetId {
        START = 1,
        MISSION,
        SETTINGS,
        CREDITS,
        QUIT,
        DOGFIGHT,
        EVADE,
    };

    void layout(const LayoutContext& ctx, MenuServices& svc, std::vector<Widget>& out) override {
        compute_geometry(ctx);
        const Metrics& m = ctx.theme.metrics;
        const bool flying = svc.in_flight();
        struct Entry { Item id; const char* label; ButtonVariant variant; };
        const std::array<Entry, 7> entries = {{
            {START, flying ? "RESUME FLIGHT" : "START FLIGHT", ButtonVariant::MENU},
            {MISSION, "AIRCRAFT & MISSION", ButtonVariant::MENU},
            {DOGFIGHT, "DOGFIGHT", ButtonVariant::MENU},
            {EVADE, "EVADE", ButtonVariant::MENU},
            {SETTINGS, "SETTINGS", ButtonVariant::MENU},
            {CREDITS, "CREDITS", ButtonVariant::MENU},
            {QUIT, "QUIT", ButtonVariant::MENU},
        }};
        for (size_t i = 0; i < entries.size(); ++i) {
            Widget w;
            w.id = entries[i].id;
            w.kind = WidgetKind::BUTTON;
            w.variant = entries[i].variant;
            w.label = entries[i].label;
            w.rect = {geo_.left, geo_.buttons_top + static_cast<float>(i) * ctx.px(m.main_button_h + m.main_button_gap),
                      ctx.px(m.main_button_w), ctx.px(m.main_button_h)};
            w.nav = {0, static_cast<int>(i), 0};
            if (entries[i].id == START) w.value_text = flying ? "ESC" : "";
            out.push_back(std::move(w));
        }
        buttons_bottom_ = out.back().rect.bottom();
    }

    [[nodiscard]] WidgetId default_focus() const override { return START; }

    void on_activate(WidgetId id, MenuServices& svc) override {
        switch (id) {
            case START: svc.emit(svc.in_flight() ? MenuAction::RESUME_FLIGHT : MenuAction::START_FLIGHT); break;
            case MISSION: svc.emit(MenuAction::OPEN_MISSION_SELECT); break;
            case DOGFIGHT: svc.navigate(ScreenId::DOGFIGHT_GEOMETRY, true); break;
            case EVADE: svc.navigate(ScreenId::EVADE_DIFFICULTY, true); break;
            case SETTINGS: svc.navigate(ScreenId::SETTINGS, true); break;
            case CREDITS: svc.navigate(ScreenId::CREDITS, true); break;
            case QUIT: confirm_quit(svc); break;
            default: break;
        }
    }

    void on_back(MenuServices& svc) override {
        if (svc.in_flight()) svc.emit(MenuAction::RESUME_FLIGHT);
        else confirm_quit(svc);
    }

    void paint_background(const LayoutContext& ctx, MenuServices& svc, DrawList& dl) override {
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;
        const Rect vp = ctx.viewport;

        dl.gradient(vp, p.backdrop.faded(0.55f), p.backdrop.faded(0.95f));
        paint_grid(ctx, dl);
        paint_radar(ctx, dl);
        // Readability wash behind the text column.
        dl.gradient_h({vp.x, vp.y, vp.w * 0.60f, vp.h}, p.backdrop.with_alpha(0.93f), p.backdrop.with_alpha(0.0f));

        // Wordmark
        const float x = geo_.left;
        dl.text_v_centered("SIX-DEGREE-OF-FREEDOM FLIGHT SIMULATION", x, geo_.overline_cy, ctx.px(t.type.label),
                           p.text_label, ctx.px(t.type.tracking_wide));
        dl.text("FAST JET", x - ctx.px(3.0f), geo_.wordmark_baseline, ctx.px(t.type.display), p.text_primary, ctx.px(1.5f));
        dl.text("SIMULATOR", x, geo_.subtitle_baseline, ctx.px(t.type.display_sub), p.text_secondary,
                ctx.px(t.type.tracking_wide * 3.0f));
        dl.fill({x, geo_.rule_y, ctx.px(64.0f), ctx.px(3.0f)}, p.accent, ctx.px(1.5f));

        // Status chip: current airframe / flight state
        const std::string& status = svc.status_text();
        if (!status.empty()) {
            const float size = ctx.px(t.type.label);
            const float pad = ctx.px(t.space.md);
            const float dot = ctx.px(8.0f);
            const float w = text_width(status, size, ctx.px(t.type.tracking_button)) + dot + 3.0f * pad;
            const Rect chip{x, geo_.chip_y, w, geo_.chip_h};
            dl.rect(chip, RectStyle::outlined(p.surface_panel.faded(0.8f), p.border_subtle, ctx.px(t.border.hairline), chip.h * 0.5f));
            const float pulse = t.reduce_motion ? 1.0f : 0.6f + 0.4f * std::sin(ctx.time * 3.0f);
            const Rgba dot_col = svc.in_flight() ? p.highlight : p.success;
            dl.fill({chip.x + pad, chip.cy() - dot * 0.5f, dot, dot}, dot_col.faded(pulse), dot);
            dl.text_v_centered(status, chip.x + pad * 2.0f + dot, chip.cy(), size, p.text_secondary, ctx.px(t.type.tracking_button));
        }
    }

    void paint_overlay(const LayoutContext& ctx, MenuServices& svc, DrawList& dl) override {
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;
        const Rect vp = ctx.viewport;

        // Description of the focused action
        const char* hint = hint_for(svc.focused(), svc.in_flight());
        if (hint) {
            dl.text_v_centered(hint, geo_.left + ctx.px(t.space.xs), buttons_bottom_ + ctx.px(t.space.xxl), ctx.px(t.type.caption),
                               p.text_muted);
        }

        const float footer_cy = vp.bottom() - ctx.px(t.space.xxxl);
        paint_legend(&dl, t, ctx.s, geo_.left, footer_cy,
                     {{"ARROWS / D-PAD", "MOVE"}, {"ENTER / A", "SELECT"}, {"ESC / B", svc.in_flight() ? "RESUME" : "QUIT"}});

        const AppInfo& info = svc.app_info();
        const std::string version = "VERSION " + info.version;
        const float right = vp.right() - ctx.px(t.space.page);
        dl.text_right(version, right, footer_cy, ctx.px(t.type.label), p.text_muted.faded(0.7f), ctx.px(t.type.tracking_wide));
    }

private:
    struct Geometry {
        float left = 0.0f;
        float overline_cy = 0.0f;
        float wordmark_baseline = 0.0f;
        float subtitle_baseline = 0.0f;
        float rule_y = 0.0f;
        float chip_y = 0.0f;
        float chip_h = 0.0f;
        float buttons_top = 0.0f;
    };
    Geometry geo_{};
    float buttons_bottom_ = 0.0f;

    void compute_geometry(const LayoutContext& ctx) {
        const Theme& t = ctx.theme;
        const Rect vp = ctx.viewport;
        geo_.left = vp.x + ctx.px(t.space.page);
        geo_.overline_cy = vp.y + vp.h * 0.17f;
        geo_.wordmark_baseline = geo_.overline_cy + ctx.px(t.space.xl) + ctx.px(t.type.display) * kFontCapHeight;
        geo_.subtitle_baseline = geo_.wordmark_baseline + ctx.px(t.space.lg) + ctx.px(t.type.display_sub) * kFontCapHeight;
        geo_.rule_y = geo_.subtitle_baseline + ctx.px(t.space.xl);
        geo_.chip_y = geo_.rule_y + ctx.px(t.space.xl);
        geo_.chip_h = ctx.px(30.0f);
        geo_.buttons_top = geo_.chip_y + geo_.chip_h + ctx.px(t.space.xxxl);
    }

    static const char* hint_for(WidgetId id, bool flying) noexcept {
        switch (id) {
            case START: return flying ? "Return to the cockpit." : "Line up on runway 09 in the selected airframe.";
            case MISSION: return "Choose an airframe, review its envelope and set up the sortie.";
            case DOGFIGHT: return "1v1 guns BFM against an AI bandit. F10 cycles the set-up, F11 the skill.";
            case EVADE: return "A bandit with radar missiles hunts you from your six. Choose the threat and your jet, then survive.";
            case SETTINGS: return "Graphics, audio, controls and accessibility.";
            case CREDITS: return "Data sources and open-source libraries.";
            case QUIT: return flying ? "End this flight and close the simulator." : "Close the simulator.";
            default: return nullptr;
        }
    }

    static void confirm_quit(MenuServices& svc) {
        ModalSpec m;
        m.title = "QUIT TO DESKTOP?";
        m.message = svc.in_flight() ? "The current flight will end. Settings you have applied are already saved."
                                    : "Close Fast Jet Simulator.";
        m.buttons = {{"QUIT", ButtonVariant::DANGER, [&svc] { svc.emit(MenuAction::QUIT); }},
                     {"CANCEL", ButtonVariant::SECONDARY, nullptr}};
        m.cancel_index = 1;
        m.default_index = 1; // Safer default: an accidental ACCEPT does not quit
        svc.show_modal(std::move(m));
    }

    static void paint_grid(const LayoutContext& ctx, DrawList& dl) {
        const Theme& t = ctx.theme;
        const Rect vp = ctx.viewport;
        const float spacing = ctx.px(72.0f);
        const float drift = ctx.time * ctx.px(t.motion.background_drift);
        const float ox = std::fmod(drift + ctx.pointer_nx * ctx.px(t.motion.parallax_far), spacing);
        const float oy = std::fmod(drift * 0.5f + ctx.pointer_ny * ctx.px(t.motion.parallax_far), spacing);
        const float th = std::max(1.0f, ctx.px(1.0f));
        for (float x = vp.x - spacing + ox; x < vp.right() + spacing; x += spacing) {
            dl.line(x, vp.y, x, vp.bottom(), th, t.palette.grid_line);
        }
        for (float y = vp.y - spacing + oy; y < vp.bottom() + spacing; y += spacing) {
            dl.line(vp.x, y, vp.right(), y, th, t.palette.grid_line);
        }
    }

    static void paint_radar(const LayoutContext& ctx, DrawList& dl) {
        constexpr float kTau = 2.0f * std::numbers::pi_v<float>;
        constexpr int kRingSegments = 120;
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;
        const Rect vp = ctx.viewport;
        const float cx = vp.x + vp.w * 0.73f + ctx.pointer_nx * ctx.px(t.motion.parallax_near);
        const float cy = vp.y + vp.h * 0.50f + ctx.pointer_ny * ctx.px(t.motion.parallax_near);
        const float r = vp.h * 0.36f;
        const float th = std::max(1.0f, ctx.px(1.2f));

        // Range rings
        for (const float frac : {1.0f, 0.66f, 0.33f}) {
            const float rr = r * frac;
            for (int i = 0; i < kRingSegments; ++i) {
                const float a0 = kTau * static_cast<float>(i) / kRingSegments;
                const float a1 = kTau * static_cast<float>(i + 1) / kRingSegments;
                dl.line(cx + rr * std::cos(a0), cy + rr * std::sin(a0), cx + rr * std::cos(a1), cy + rr * std::sin(a1), th,
                        p.border_strong.faded(frac == 1.0f ? 0.45f : 0.22f));
            }
        }
        dl.line(cx - r, cy, cx + r, cy, th, p.border_subtle.faded(0.35f));
        dl.line(cx, cy - r, cx, cy + r, th, p.border_subtle.faded(0.35f));

        // Bearing ticks and labels (000 at the top, clockwise)
        for (int deg = 0; deg < 360; deg += 5) {
            const float a = kTau * static_cast<float>(deg) / 360.0f - kTau * 0.25f;
            const bool major = deg % 30 == 0;
            const float len = ctx.px(major ? 14.0f : 6.0f);
            dl.line(cx + (r - len) * std::cos(a), cy + (r - len) * std::sin(a), cx + r * std::cos(a), cy + r * std::sin(a),
                    th, p.border_bright.faded(major ? 0.55f : 0.25f));
            if (major) {
                char buf[4];
                std::snprintf(buf, sizeof(buf), "%03d", deg);
                const float lr = r + ctx.px(22.0f);
                dl.text_centered(buf, cx + lr * std::cos(a), cy + lr * std::sin(a), ctx.px(t.type.caption * 0.85f),
                                 p.text_muted.faded(0.55f), ctx.px(1.0f));
            }
        }

        // Sweep with a fading trail
        const float sweep = kTau * std::fmod(ctx.time / t.motion.sweep_period, 1.0f) - kTau * 0.25f;
        constexpr int kTrail = 28;
        constexpr float kTrailArc = 0.65f; // rad
        for (int i = kTrail; i >= 0; --i) {
            const float f = static_cast<float>(i) / kTrail;
            const float a = sweep - f * kTrailArc;
            dl.line(cx, cy, cx + r * std::cos(a), cy + r * std::sin(a), th * (i == 0 ? 2.0f : 1.4f),
                    p.accent.faded((1.0f - f) * (i == 0 ? 0.55f : 0.10f)));
        }

        // Contacts light up as the sweep passes, then decay
        struct Contact { float bearing; float range; };
        constexpr std::array<Contact, 5> kContacts = {{{0.9f, 0.72f}, {2.3f, 0.45f}, {3.6f, 0.85f}, {4.4f, 0.30f}, {5.5f, 0.60f}}};
        for (const Contact& c : kContacts) {
            const float a = c.bearing - kTau * 0.25f;
            float since = std::fmod(sweep - a, kTau);
            if (since < 0.0f) since += kTau;
            const float glow = std::exp(-since / 1.6f);
            const float d = ctx.px(7.0f);
            const float x = cx + r * c.range * std::cos(a);
            const float y = cy + r * c.range * std::sin(a);
            dl.fill({x - d * 0.5f, y - d * 0.5f, d, d}, p.accent.faded(0.15f + 0.75f * glow), d);
        }
    }
};

} // namespace fastjet::ui
