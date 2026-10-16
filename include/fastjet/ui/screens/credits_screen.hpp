#pragma once

#include "fastjet/ui/screen.hpp"
#include <algorithm>
#include <array>

namespace fastjet::ui {

/// @brief Scrolling acknowledgements for data sources and libraries.
class CreditsScreen final : public Screen {
public:
    static constexpr WidgetId kBack = 1;

    [[nodiscard]] WidgetId default_focus() const override { return kBack; }
    [[nodiscard]] std::optional<ScrollRegion> scroll_region() const override { return ScrollRegion{view_, content_h_}; }

    void layout(const LayoutContext& ctx, MenuServices&, std::vector<Widget>& out) override {
        const Theme& t = ctx.theme;
        const Metrics& m = t.metrics;
        const Rect vp = ctx.viewport;
        const float margin = ctx.px(m.panel_margin);
        const float w = std::min(vp.w - 2.0f * margin, ctx.px(m.panel_max_w * 0.72f));
        panel_ = {vp.cx() - w * 0.5f, vp.y + margin, w, vp.h - 2.0f * margin};
        footer_ = {panel_.x, panel_.bottom() - ctx.px(m.footer_h), w, ctx.px(m.footer_h)};
        const float top = panel_.y + ctx.px(m.header_h);
        view_ = {panel_.x + ctx.px(t.space.xl), top, w - ctx.px(t.space.xl) * 2.0f, footer_.y - top - ctx.px(t.space.sm)};

        content_h_ = 0.0f;
        for (const Entry& e : kEntries) content_h_ += entry_height(ctx, e);

        Widget back;
        back.id = kBack;
        back.kind = WidgetKind::BUTTON;
        back.variant = ButtonVariant::SECONDARY;
        back.label = "BACK";
        const float bh = ctx.px(m.footer_button_h);
        back.rect = {footer_.cx() - ctx.px(m.footer_button_min_w) * 0.5f, footer_.cy() - bh * 0.5f, ctx.px(m.footer_button_min_w), bh};
        back.nav = {0, 0, 0};
        out.push_back(std::move(back));
    }

    void on_activate(WidgetId id, MenuServices& svc) override {
        if (id == kBack) on_back(svc);
    }
    void on_back(MenuServices& svc) override { svc.navigate(ScreenId::MAIN, false); }

    void paint_background(const LayoutContext& ctx, MenuServices& svc, DrawList& dl) override {
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;
        dl.fill(ctx.viewport, p.backdrop);
        paint_panel(dl, t, ctx.s, panel_, t.elevation.panel);
        dl.text_centered("CREDITS", panel_.cx(), panel_.y + ctx.px(t.metrics.header_h) * 0.45f, ctx.px(t.type.title), p.text_primary,
                         ctx.px(1.0f));
        dl.text_centered(svc.app_info().title + "  /  VERSION " + svc.app_info().version, panel_.cx(),
                         panel_.y + ctx.px(t.metrics.header_h) * 0.45f + ctx.px(t.space.xxl), ctx.px(t.type.caption), p.text_muted,
                         ctx.px(t.type.tracking_button));
        dl.line(panel_.x, footer_.y, panel_.right(), footer_.y, ctx.px(t.border.hairline), p.border_subtle);
    }

    void paint_scroll_content(const LayoutContext& ctx, MenuServices&, DrawList& dl) override {
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;
        float y = view_.y - ctx.scroll_y;
        for (const Entry& e : kEntries) {
            const float h = entry_height(ctx, e);
            const float cy = y + h * 0.5f;
            switch (e.kind) {
                case Kind::HEADING:
                    dl.text_centered(e.text, view_.cx(), cy + ctx.px(t.space.sm), ctx.px(t.type.label), p.text_label,
                                     ctx.px(t.type.tracking_wide));
                    break;
                case Kind::LINE:
                    dl.text_centered(e.text, view_.cx(), cy, ctx.px(t.type.body * 0.9f), p.text_primary);
                    break;
                case Kind::DETAIL:
                    dl.text_centered(e.text, view_.cx(), cy, ctx.px(t.type.caption), p.text_muted);
                    break;
                case Kind::GAP:
                    break;
            }
            y += h;
        }
    }

private:
    enum class Kind : uint8_t { HEADING, LINE, DETAIL, GAP };
    struct Entry {
        Kind kind;
        const char* text;
    };

    static constexpr std::array<Entry, 34> kEntries = {{
        {Kind::HEADING, "FLIGHT DYNAMICS"},
        {Kind::LINE, "Six-degree-of-freedom rigid-body model"},
        {Kind::DETAIL, "Fourth-order Runge-Kutta integration at 200 Hz"},
        {Kind::GAP, ""},
        {Kind::HEADING, "AERODYNAMIC DATA"},
        {Kind::LINE, "NASA Technical Paper 1538"},
        {Kind::DETAIL, "F-16 wind-tunnel coefficient tables"},
        {Kind::GAP, ""},
        {Kind::HEADING, "ENVIRONMENT"},
        {Kind::LINE, "U.S. Standard Atmosphere, 1976"},
        {Kind::LINE, "Dryden continuous turbulence model"},
        {Kind::DETAIL, "Wind, gusts and boundary-layer shear"},
        {Kind::GAP, ""},
        {Kind::HEADING, "FLIGHT CONTROL"},
        {Kind::LINE, "Digital fly-by-wire control laws"},
        {Kind::DETAIL, "G / AoA limiting, Auto-GCAS and G-LOC protection"},
        {Kind::GAP, ""},
        {Kind::HEADING, "3D MODELS"},
        {Kind::LINE, "F-16, F-15E, Typhoon, F-22 and F-35 \"Fighter Jet - Free\""},
        {Kind::DETAIL, "by bohmerang on Sketchfab  /  CC BY-NC-SA 4.0"},
        {Kind::GAP, ""},
        {Kind::HEADING, "OPEN-SOURCE LIBRARIES"},
        {Kind::LINE, "SDL 3"},
        {Kind::DETAIL, "Windowing, input, audio  /  zlib licence"},
        {Kind::LINE, "libepoxy"},
        {Kind::DETAIL, "OpenGL function loading  /  MIT licence"},
        {Kind::LINE, "cgltf"},
        {Kind::DETAIL, "glTF 2.0 model loading  /  MIT licence"},
        {Kind::LINE, "stb_image"},
        {Kind::DETAIL, "Texture decoding  /  public domain or MIT"},
        {Kind::GAP, ""},
        {Kind::HEADING, "THANK YOU FOR FLYING"},
        {Kind::DETAIL, "Check six."},
        {Kind::GAP, ""},
    }};

    Rect panel_{}, footer_{}, view_{};
    float content_h_ = 0.0f;

    [[nodiscard]] static float entry_height(const LayoutContext& ctx, const Entry& e) {
        const Theme& t = ctx.theme;
        switch (e.kind) {
            case Kind::HEADING: return ctx.px(t.space.xxxl);
            case Kind::LINE: return ctx.px(t.type.body * 1.7f);
            case Kind::DETAIL: return ctx.px(t.type.caption * 1.8f);
            case Kind::GAP: return ctx.px(t.space.lg);
        }
        return 0.0f;
    }
};

} // namespace fastjet::ui
