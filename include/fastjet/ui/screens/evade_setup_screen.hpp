#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/ui/screen.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <string>

namespace fastjet::ui {

/// @brief What the evade set-up screens have chosen; read by the host on START_EVADE.
struct EvadeChoice {
    int difficulty = 1; ///< 0 EASY .. 3 EXPERT, the order of sim::EvadeDifficulty
    aircraft::AircraftType jet = aircraft::AircraftType::F16_FIGHTING_FALCON;
};

/// @brief One threat level as the menu describes it. The numbers mirror
/// sim::EvadeProfile (test_evade_mode checks they agree); the menu does not
/// depend on the simulation.
struct EvadeDifficultyInfo {
    const char* name;
    int survive_s;
    double start_nm;
    const char* pilot;
    const char* weapons;
    const char* start;
    const char* advice;
};

inline constexpr std::array<EvadeDifficultyInfo, 4> kEvadeDifficulties = {{
    {"EASY", 120, 3.0, "Novice pilot, slow to react", "Guns only: no missiles", "3 nm behind you, level",
     "Keep him in sight and turn into him before he gets a gun solution."},
    {"MEDIUM", 150, 10.0, "Veteran pilot", "2 AIM-120s, fired at maximum range", "10 nm behind, 1,000 ft above",
     "Long shots: turn away and run, or beam him and drop chaff."},
    {"HARD", 180, 6.5, "Ace pilot, locks you in 1 s", "4 AIM-120s, held until you cannot outrun them",
     "6.5 nm behind, 2,000 ft above, 80 kt faster", "Every shot is close: beam, chaff and break late."},
    {"EXPERT", 210, 6.0, "Ace pilot, locks you in 0.8 s", "4 AIM-120s, fired in pairs",
     "6 nm behind, 3,000 ft above, 115 kt faster", "Two missiles per shot: one defence will not beat both."},
}};

inline constexpr std::array<aircraft::AircraftType, 6> kEvadeJets = {
    aircraft::AircraftType::F16_FIGHTING_FALCON, aircraft::AircraftType::F15EX_EAGLE_II,
    aircraft::AircraftType::EUROFIGHTER_TYPHOON, aircraft::AircraftType::F22_RAPTOR,
    aircraft::AircraftType::A10_THUNDERBOLT, aircraft::AircraftType::F35B_LIGHTNING_II};

/// @brief Evade set-up, in two steps: the threat level, then your jet. Each
/// list works like a radio group: the focused entry is the choice, ENTER on
/// it (or the footer's primary button) moves on, BACK returns a step.
class EvadeSetupScreen final : public Screen {
public:
    enum class Step : uint8_t { DIFFICULTY, AIRCRAFT };

    static constexpr WidgetId kOptionBase = 1; ///< Options are kOptionBase + index
    static constexpr WidgetId kBack = 100;
    static constexpr WidgetId kGo = 101;

    EvadeSetupScreen(Step step, EvadeChoice& choice) noexcept : step_(step), choice_(choice) {}

    void on_enter(MenuServices& svc) override {
        // The jet list opens on the airframe you are flying.
        if (step_ == Step::AIRCRAFT) choice_.jet = svc.current_aircraft();
    }

    void update(float, MenuServices& svc) override {
        const WidgetId f = svc.focused();
        if (f >= kOptionBase && f < kOptionBase + static_cast<WidgetId>(option_count())) choose(static_cast<int>(f - kOptionBase));
    }

    [[nodiscard]] WidgetId default_focus() const override { return kOptionBase + static_cast<WidgetId>(chosen()); }

    void layout(const LayoutContext& ctx, MenuServices&, std::vector<Widget>& out) override {
        const Theme& t = ctx.theme;
        const Metrics& m = t.metrics;
        const Rect vp = ctx.viewport;
        const float margin = ctx.px(m.panel_margin);
        const float w = std::min(vp.w - 2.0f * margin, ctx.px(m.panel_max_w * 0.9f));
        const float h = std::min(vp.h - 2.0f * margin, ctx.px(720.0f));
        panel_ = {vp.cx() - w * 0.5f, vp.cy() - h * 0.5f, w, h};
        footer_ = {panel_.x, panel_.bottom() - ctx.px(m.footer_h), w, ctx.px(m.footer_h)};

        const float pad = ctx.px(t.space.xxl);
        const float top = panel_.y + ctx.px(m.header_h) + ctx.px(t.space.xl);
        const float list_w = std::min(ctx.px(m.main_button_w), (w - 3.0f * pad) * 0.5f);
        const float row_h = ctx.px(m.main_button_h);
        const float gap = ctx.px(m.main_button_gap);
        detail_ = {panel_.x + 2.0f * pad + list_w, top, panel_.right() - pad - (panel_.x + 2.0f * pad + list_w),
                   footer_.y - ctx.px(t.space.xl) - top};

        option_rects_.clear();
        for (int i = 0; i < option_count(); ++i) {
            Widget o;
            o.id = kOptionBase + static_cast<WidgetId>(i);
            o.kind = WidgetKind::BUTTON;
            o.variant = ButtonVariant::MENU;
            if (step_ == Step::DIFFICULTY) {
                const EvadeDifficultyInfo& d = kEvadeDifficulties[static_cast<size_t>(i)];
                o.label = d.name;
                o.value_text = "SURVIVE " + clock(d.survive_s);
            } else {
                o.label = upper(aircraft::to_short_string(kEvadeJets[static_cast<size_t>(i)]));
                o.value_text = upper(aircraft::engine_designation(kEvadeJets[static_cast<size_t>(i)]));
            }
            o.rect = {panel_.x + pad, top + static_cast<float>(i) * (row_h + gap), list_w, row_h};
            o.nav = {0, i, 0};
            option_rects_.push_back(o.rect);
            out.push_back(std::move(o));
        }

        const float bh = ctx.px(m.footer_button_h);
        const float bw = ctx.px(m.footer_button_min_w);
        const float go_w = ctx.px(m.footer_button_min_w * 1.5f);
        Widget back;
        back.id = kBack;
        back.kind = WidgetKind::BUTTON;
        back.variant = ButtonVariant::SECONDARY;
        back.label = "BACK";
        back.rect = {footer_.x + pad, footer_.cy() - bh * 0.5f, bw, bh};
        back.nav = {1, 0, 0};
        out.push_back(std::move(back));

        Widget go;
        go.id = kGo;
        go.kind = WidgetKind::BUTTON;
        go.variant = ButtonVariant::PRIMARY;
        go.label = step_ == Step::DIFFICULTY ? "NEXT" : "START EVADE";
        go.rect = {footer_.right() - pad - go_w, footer_.cy() - bh * 0.5f, go_w, bh};
        go.nav = {1, 0, 1};
        go.selected = true; // DOWN off the list lands here, not on BACK
        out.push_back(std::move(go));
    }

    void on_activate(WidgetId id, MenuServices& svc) override {
        if (id == kBack) {
            on_back(svc);
            return;
        }
        if (id >= kOptionBase && id < kOptionBase + static_cast<WidgetId>(option_count())) {
            choose(static_cast<int>(id - kOptionBase));
        } else if (id != kGo) {
            return;
        }
        if (step_ == Step::DIFFICULTY) svc.navigate(ScreenId::EVADE_AIRCRAFT, true);
        else svc.emit(MenuAction::START_EVADE);
    }

    void on_back(MenuServices& svc) override {
        svc.navigate(step_ == Step::DIFFICULTY ? ScreenId::MAIN : ScreenId::EVADE_DIFFICULTY, false);
    }

    void paint_background(const LayoutContext& ctx, MenuServices&, DrawList& dl) override {
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;
        dl.fill(ctx.viewport, p.backdrop);
        paint_panel(dl, t, ctx.s, panel_, t.elevation.panel);

        const float pad = ctx.px(t.space.xxl);
        const float title_cy = panel_.y + ctx.px(t.metrics.header_h) * 0.42f;
        dl.text_v_centered("EVADE", panel_.x + pad, title_cy, ctx.px(t.type.title), p.text_primary, ctx.px(1.0f));
        const char* step = step_ == Step::DIFFICULTY ? "STEP 1 OF 2  /  THREAT LEVEL" : "STEP 2 OF 2  /  YOUR AIRCRAFT";
        dl.text_v_centered(step, panel_.x + pad, title_cy + ctx.px(t.space.xxl), ctx.px(t.type.label), p.text_label,
                           ctx.px(t.type.tracking_wide));
        if (step_ == Step::AIRCRAFT) {
            const std::string threat = std::string("THREAT  ") + kEvadeDifficulties[static_cast<size_t>(choice_.difficulty)].name;
            dl.text_right(threat, panel_.right() - pad, title_cy, ctx.px(t.type.label), p.highlight, ctx.px(t.type.tracking_wide));
        }
        // Step progress: two bars, the current one lit.
        const float bar_w = ctx.px(40.0f);
        const float bar_y = panel_.y + ctx.px(t.metrics.header_h) - ctx.px(t.space.xs);
        for (int i = 0; i < 2; ++i) {
            const bool lit = i <= static_cast<int>(step_);
            dl.fill({panel_.x + pad + static_cast<float>(i) * (bar_w + ctx.px(t.space.sm)), bar_y, bar_w, ctx.px(3.0f)},
                    lit ? p.accent : p.border_subtle, ctx.px(1.5f));
        }
        dl.line(panel_.x, footer_.y, panel_.right(), footer_.y, ctx.px(t.border.hairline), p.border_subtle);
    }

    void paint_overlay(const LayoutContext& ctx, MenuServices& svc, DrawList& dl) override {
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;

        // Keep the choice marked while focus is on the footer.
        const int c = chosen();
        if (c < static_cast<int>(option_rects_.size()) && svc.focused() != kOptionBase + static_cast<WidgetId>(c)) {
            dl.stroke(option_rects_[static_cast<size_t>(c)], p.accent.faded(0.6f), ctx.px(t.border.regular), ctx.px(t.radius.md));
        }

        dl.rect(detail_, RectStyle::outlined(p.surface_void.faded(0.6f), p.border_subtle, ctx.px(t.border.hairline), ctx.px(t.radius.lg)));
        const float x = detail_.x + ctx.px(t.space.xl);
        const float max_w = detail_.w - 2.0f * ctx.px(t.space.xl);
        float y = detail_.y + ctx.px(t.space.xxl);
        auto heading = [&](const std::string& s) {
            dl.text_v_centered(fit_text(s, max_w, ctx.px(t.type.heading)), x, y, ctx.px(t.type.heading), p.text_primary, ctx.px(1.0f));
            y += ctx.px(t.space.xxl + t.space.sm);
        };
        auto field = [&](const char* label, const std::string& value) {
            dl.text_v_centered(label, x, y, ctx.px(t.type.label), p.text_label, ctx.px(t.type.tracking_wide));
            y += ctx.px(t.space.xl);
            dl.text_v_centered(fit_text(value, max_w, ctx.px(t.type.body * 0.9f)), x, y, ctx.px(t.type.body * 0.9f), p.text_primary);
            y += ctx.px(t.space.xxl + t.space.sm);
        };
        auto note = [&](const std::string& s) {
            dl.text_v_centered(fit_text(s, max_w, ctx.px(t.type.caption)), x, y, ctx.px(t.type.caption), p.text_muted);
            y += ctx.px(t.space.xl);
        };

        if (step_ == Step::DIFFICULTY) {
            const EvadeDifficultyInfo& d = kEvadeDifficulties[static_cast<size_t>(choice_.difficulty)];
            heading(d.name);
            field("BANDIT", d.pilot);
            field("WEAPONS", d.weapons);
            field("START", d.start);
            field("SURVIVE", clock(d.survive_s) + " until he runs out of fuel");
            note(d.advice);
            note("C drops chaff. The RWR shows his radar and any missile.");
        } else {
            const aircraft::AircraftType j = choice_.jet;
            heading(upper(aircraft::to_string(j)));
            field("ENGINES", std::string(aircraft::engine_designation(j)));
            field("PERFORMANCE", std::string(aircraft::signature_spec(j)));
            note(jet_note(j));
            note("The bandit flies the same type.");
            if (j == aircraft::AircraftType::A10_THUNDERBOLT) note("The A-10 has no radar: its bandit carries guns only.");
        }
    }

    /// @brief The choice as the list shows it (tests / tooling).
    [[nodiscard]] int chosen() const noexcept {
        if (step_ == Step::DIFFICULTY) return std::clamp(choice_.difficulty, 0, static_cast<int>(kEvadeDifficulties.size()) - 1);
        for (size_t i = 0; i < kEvadeJets.size(); ++i) {
            if (kEvadeJets[i] == choice_.jet) return static_cast<int>(i);
        }
        return 0;
    }

private:
    Step step_;
    EvadeChoice& choice_;
    Rect panel_{}, footer_{}, detail_{};
    std::vector<Rect> option_rects_;

    [[nodiscard]] int option_count() const noexcept {
        return static_cast<int>(step_ == Step::DIFFICULTY ? kEvadeDifficulties.size() : kEvadeJets.size());
    }

    void choose(int i) noexcept {
        if (step_ == Step::DIFFICULTY) choice_.difficulty = i;
        else choice_.jet = kEvadeJets[static_cast<size_t>(i)];
    }

    [[nodiscard]] static std::string clock(int s) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%d:%02d", s / 60, s % 60);
        return buf;
    }

    [[nodiscard]] static std::string upper(std::string_view s) {
        std::string r(s);
        for (char& ch : r) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
        return r;
    }

    [[nodiscard]] static const char* jet_note(aircraft::AircraftType j) noexcept {
        switch (j) {
            case aircraft::AircraftType::F16_FIGHTING_FALCON: return "Light single-engine fighter: quick to roll and turn.";
            case aircraft::AircraftType::F15EX_EAGLE_II: return "Heavy twin-engine fighter: the fastest jet here.";
            case aircraft::AircraftType::EUROFIGHTER_TYPHOON: return "Delta-canard: supercruises and climbs hard.";
            case aircraft::AircraftType::F22_RAPTOR: return "Thrust vectoring: points its nose even when slow.";
            case aircraft::AircraftType::A10_THUNDERBOLT: return "Subsonic, no afterburner: you cannot outrun anything.";
            case aircraft::AircraftType::F35B_LIGHTNING_II: return "STOVL: can hover, but 7 G and slow to accelerate.";
        }
        return "";
    }
};

} // namespace fastjet::ui
