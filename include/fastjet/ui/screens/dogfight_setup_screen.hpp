#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/sim/combat_specs.hpp"
#include "fastjet/sim/engagement.hpp"
#include "fastjet/ui/screen.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <string>

namespace fastjet::ui {

/// @brief Player choices for dogfight engagement setup
struct DogfightChoice {
    int geometry = 0; ///< 0: MERGE, 1: OFFENSIVE, 2: DEFENSIVE, 3: RANGE
    int loadout = 1;  ///< 0: GUNS_ONLY, 1: GUNS_AND_IR, 2: FULL_LOADOUT
    int skill = 1;    ///< 0: NOVICE, 1: VETERAN, 2: ACE
    aircraft::AircraftType own_jet = aircraft::AircraftType::F16_FIGHTING_FALCON;
    aircraft::AircraftType bandit_jet = aircraft::AircraftType::F16_FIGHTING_FALCON;

    [[nodiscard]] sim::StartGeometry start_geometry() const noexcept {
        switch (geometry) {
            case 0: return sim::StartGeometry::HEAD_ON_MERGE;
            case 1: return sim::StartGeometry::OFFENSIVE_PERCH;
            case 2: return sim::StartGeometry::DEFENSIVE_PERCH;
            case 3: return sim::StartGeometry::GUNNERY_RANGE;
            default: return sim::StartGeometry::HEAD_ON_MERGE;
        }
    }

    [[nodiscard]] sim::WeaponsLoadout weapons_loadout() const noexcept {
        switch (loadout) {
            case 0: return sim::WeaponsLoadout::GUNS_ONLY;
            case 1: return sim::WeaponsLoadout::GUNS_AND_IR;
            case 2: return sim::WeaponsLoadout::FULL_LOADOUT;
            default: return sim::WeaponsLoadout::GUNS_AND_IR;
        }
    }

    [[nodiscard]] sim::AiSkill ai_skill() const noexcept {
        switch (skill) {
            case 0: return sim::AiSkill::NOVICE;
            case 1: return sim::AiSkill::VETERAN;
            case 2: return sim::AiSkill::ACE;
            default: return sim::AiSkill::VETERAN;
        }
    }
};

struct DogfightGeometryInfo {
    const char* name;
    const char* subtitle;
    const char* description;
    const char* advice;
};

inline constexpr std::array<DogfightGeometryInfo, 4> kDogfightGeometries = {{
    {"HEAD-ON MERGE", "6 km Neutral Separation",
     "Both jets head-on at 300 m/s, co-altitude. Symmetrical entry into the merge.",
     "Lead turn at the pass, watch his nose position, and fight in one or two circles."},
    {"OFFENSIVE PERCH", "1.5 km Bandit Six O'Clock",
     "You start 1,500 m behind the bandit in his tail quarter with offensive advantage.",
     "Convert the perch quickly: track the pipper, match his plane of turn, squeeze the trigger."},
    {"DEFENSIVE PERCH", "1.5 km Bandit On Your Tail",
     "Bandit starts on your six o'clock inside missile and gun range.",
     "Hard break immediately, dispense flares/chaff, and spoil his tracking solution."},
    {"GUNNERY RANGE", "Target Practice",
     "Bandit flies predictable tracking profiles to practice gun lead and pipper control.",
     "Master the gyro pipper, lead angles, and trigger discipline across all airframes."},
}};

struct DogfightLoadoutInfo {
    const char* name;
    const char* subtitle;
    const char* weapons;
    const char* details;
};

inline constexpr std::array<DogfightLoadoutInfo, 3> kDogfightLoadouts = {{
    {"GUNS ONLY", "M61A1 / BK-27 Cannon", "Internal Cannon Only",
     "Pure BFM dogfight: maneuver for gun lead and gun solution inside 800 meters."},
    {"GUNS & HEAT-SEEKERS", "AIM-9X / IRIS-T / AIM-9M + Cannon", "The jet's IR Missiles + Cannon + 60 Flares",
     "High off-boresight dogfight: uncage the heat seeker, listen for growl, and dump flares to evade."},
    {"FULL COMBAT LOADOUT", "AMRAAM / Meteor + IR Missiles + Cannon", "The jet's BVR + WVR Missiles + Gun + Chaff/Flares",
     "Complete air-to-air suite: radar Fox-3 BVR shots, DLZ cues, IR dogfight missiles, and chaff/flares."},
}};

struct DogfightSkillInfo {
    const char* name;
    const char* subtitle;
    const char* description;
    const char* tactics;
};

inline constexpr std::array<DogfightSkillInfo, 3> kDogfightSkills = {{
    {"NOVICE", "Basic Flight Controls",
     "0.45 s reaction time, 72% G authority, basic pursuit tracking, no countermeasures.",
     "Predictable turns and loose energy discipline. Ideal for learning gun lead."},
    {"VETERAN", "Combat Qualified Fighter Pilot",
     "0.30 s reaction time, 85% G authority, corner-speed management, defensive flares.",
     "Maintains energy, pulls aggressive lead, and dumps flares when locked."},
    {"ACE", "Top Gun Fighter Weapons Instructor",
     "0.18 s reaction time, 95% G limit, vertical High Yo-Yos, offensive missile snapshots.",
     "Out-maneuvers overshoots with vertical out-of-plane repositions and snapshot missiles."},
}};

inline constexpr std::array<aircraft::AircraftType, 6> kDogfightJets = {
    aircraft::AircraftType::F16_FIGHTING_FALCON, aircraft::AircraftType::F15EX_EAGLE_II,
    aircraft::AircraftType::EUROFIGHTER_TYPHOON, aircraft::AircraftType::F22_RAPTOR,
    aircraft::AircraftType::A10_THUNDERBOLT, aircraft::AircraftType::F35B_LIGHTNING_II};

/// @brief Interactive 4-step setup screen for realistic air-to-air dogfights:
/// 1. Setup Geometry -> 2. Weapons Loadout -> 3. AI Pilot Skill -> 4. Airframe Selection
class DogfightSetupScreen final : public Screen {
public:
    enum class Step : uint8_t { GEOMETRY = 0, LOADOUT, SKILL, AIRCRAFT };

    static constexpr WidgetId kOptionBase = 1;
    static constexpr WidgetId kBack = 100;
    static constexpr WidgetId kGo = 101;

    DogfightSetupScreen(Step step, DogfightChoice& choice) noexcept : step_(step), choice_(choice) {}

    void on_enter(MenuServices& svc) override {
        if (step_ == Step::AIRCRAFT) {
            choice_.own_jet = svc.current_aircraft();
            choice_.bandit_jet = svc.current_aircraft();
        }
    }

    void update(float, MenuServices& svc) override {
        const WidgetId f = svc.focused();
        if (f >= kOptionBase && f < kOptionBase + static_cast<WidgetId>(option_count())) {
            choose(static_cast<int>(f - kOptionBase));
        }
    }

    [[nodiscard]] WidgetId default_focus() const override {
        return kOptionBase + static_cast<WidgetId>(chosen());
    }

    void layout(const LayoutContext& ctx, MenuServices&, std::vector<Widget>& out) override {
        const Theme& t = ctx.theme;
        const Metrics& m = t.metrics;
        const Rect vp = ctx.viewport;
        const float margin = ctx.px(m.panel_margin);
        const float w = std::min(vp.w - 2.0f * margin, ctx.px(m.panel_max_w * 0.95f));
        const float h = std::min(vp.h - 2.0f * margin, ctx.px(740.0f));
        panel_ = {vp.cx() - w * 0.5f, vp.cy() - h * 0.5f, w, h};
        footer_ = {panel_.x, panel_.bottom() - ctx.px(m.footer_h), w, ctx.px(m.footer_h)};

        const float pad = ctx.px(t.space.xxl);
        const float top = panel_.y + ctx.px(m.header_h) + ctx.px(t.space.xl);
        const float list_w = std::min(ctx.px(m.main_button_w * 1.05f), (w - 3.0f * pad) * 0.48f);
        const float row_h = ctx.px(m.main_button_h);
        const float gap = ctx.px(m.main_button_gap);
        detail_ = {panel_.x + 2.0f * pad + list_w, top, panel_.right() - pad - (panel_.x + 2.0f * pad + list_w),
                   footer_.y - ctx.px(t.space.xl) - top};

        option_rects_.clear();
        const int count = option_count();
        for (int i = 0; i < count; ++i) {
            Widget o;
            o.id = kOptionBase + static_cast<WidgetId>(i);
            o.kind = WidgetKind::BUTTON;
            o.variant = ButtonVariant::MENU;

            if (step_ == Step::GEOMETRY) {
                const auto& g = kDogfightGeometries[static_cast<size_t>(i)];
                o.label = g.name;
                o.value_text = g.subtitle;
            } else if (step_ == Step::LOADOUT) {
                const auto& l = kDogfightLoadouts[static_cast<size_t>(i)];
                o.label = l.name;
                o.value_text = l.subtitle;
            } else if (step_ == Step::SKILL) {
                const auto& s = kDogfightSkills[static_cast<size_t>(i)];
                o.label = s.name;
                o.value_text = s.subtitle;
            } else {
                const auto j = kDogfightJets[static_cast<size_t>(i)];
                o.label = upper(aircraft::to_string(j));
                o.value_text = aircraft::engine_designation(j);
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
        back.label = (step_ == Step::GEOMETRY) ? "CANCEL" : "BACK";
        back.rect = {footer_.x + pad, footer_.cy() - bh * 0.5f, bw, bh};
        back.nav = {0, count, 0};
        out.push_back(std::move(back));

        Widget go;
        go.id = kGo;
        go.kind = WidgetKind::BUTTON;
        go.variant = ButtonVariant::PRIMARY;
        go.label = (step_ == Step::AIRCRAFT) ? "ENGAGE" : "NEXT";
        go.rect = {footer_.right() - pad - go_w, footer_.cy() - bh * 0.5f, go_w, bh};
        go.nav = {0, count + 1, 0};
        go.selected = true;
        out.push_back(std::move(go));
    }

    void on_back(MenuServices& svc) override {
        switch (step_) {
            case Step::GEOMETRY: svc.navigate(ScreenId::MAIN, true); break;
            case Step::LOADOUT: svc.navigate(ScreenId::DOGFIGHT_GEOMETRY, true); break;
            case Step::SKILL: svc.navigate(ScreenId::DOGFIGHT_LOADOUT, true); break;
            case Step::AIRCRAFT: svc.navigate(ScreenId::DOGFIGHT_SKILL, true); break;
        }
    }

    void on_activate(WidgetId id, MenuServices& svc) override {
        if (id == kBack) {
            on_back(svc);
            return;
        }

        if (id == kGo || (id >= kOptionBase && id < kOptionBase + static_cast<WidgetId>(option_count()))) {
            if (id >= kOptionBase && id < kOptionBase + static_cast<WidgetId>(option_count())) {
                choose(static_cast<int>(id - kOptionBase));
            }
            switch (step_) {
                case Step::GEOMETRY: svc.navigate(ScreenId::DOGFIGHT_LOADOUT, true); break;
                case Step::LOADOUT: svc.navigate(ScreenId::DOGFIGHT_SKILL, true); break;
                case Step::SKILL: svc.navigate(ScreenId::DOGFIGHT_AIRCRAFT, true); break;
                case Step::AIRCRAFT: svc.emit(MenuAction::START_DOGFIGHT); break;
            }
        }
    }

    void paint_background(const LayoutContext& ctx, MenuServices&, DrawList& dl) override {
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;
        dl.fill(ctx.viewport, p.backdrop);
        paint_panel(dl, t, ctx.s, panel_, t.elevation.panel);

        const float pad = ctx.px(t.space.xxl);
        const float title_cy = panel_.y + ctx.px(t.metrics.header_h) * 0.42f;
        dl.text_v_centered("AIR COMBAT MANEUVERING", panel_.x + pad, title_cy, ctx.px(t.type.title), p.text_primary, ctx.px(1.0f));

        const char* step_str = "STEP 1 OF 4  /  ENGAGEMENT GEOMETRY";
        if (step_ == Step::LOADOUT) step_str = "STEP 2 OF 4  /  WEAPONS LOADOUT";
        else if (step_ == Step::SKILL) step_str = "STEP 3 OF 4  /  AI BANDIT SKILL";
        else if (step_ == Step::AIRCRAFT) step_str = "STEP 4 OF 4  /  YOUR AIRCRAFT";

        dl.text_v_centered(step_str, panel_.x + pad, title_cy + ctx.px(t.space.xxl), ctx.px(t.type.label), p.text_label,
                           ctx.px(t.type.tracking_wide));

        // Step progress: four bars
        const float bar_w = ctx.px(32.0f);
        const float bar_y = panel_.y + ctx.px(t.metrics.header_h) - ctx.px(t.space.xs);
        for (int i = 0; i < 4; ++i) {
            const bool lit = i <= static_cast<int>(step_);
            dl.fill({panel_.x + pad + static_cast<float>(i) * (bar_w + ctx.px(t.space.sm)), bar_y, bar_w, ctx.px(3.0f)},
                    lit ? p.accent : p.border_subtle, ctx.px(1.5f));
        }
        dl.line(panel_.x, footer_.y, panel_.right(), footer_.y, ctx.px(t.border.hairline), p.border_subtle);
    }

    void paint_overlay(const LayoutContext& ctx, MenuServices& svc, DrawList& dl) override {
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;

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

        if (step_ == Step::GEOMETRY) {
            const auto& g = kDogfightGeometries[static_cast<size_t>(choice_.geometry)];
            heading(g.name);
            field("ENTRY CONDITIONS", g.description);
            field("TACTICAL ADVICE", g.advice);
            note("Engage uses fixed 200 Hz physical simulation.");
            note("Zero-allocation deterministic dogfight loop.");
        } else if (step_ == Step::LOADOUT) {
            const auto& l = kDogfightLoadouts[static_cast<size_t>(choice_.loadout)];
            heading(l.name);
            field("ORDNANCE COMPLEMENT", l.weapons);
            field("TACTICAL EMPLOYMENT", l.details);
            note("Select weapon with Q / Joystick Weapon Select.");
            note("Fire missile with Enter / Joystick Pickle. Flares with X.");
        } else if (step_ == Step::SKILL) {
            const auto& s = kDogfightSkills[static_cast<size_t>(choice_.skill)];
            heading(s.name);
            field("PILOT PROFILE", s.description);
            field("ACM DOCTRINE", s.tactics);
            note("Human and AI pilots share identical FLCS & blackout physiology.");
        } else {
            const auto j = choice_.own_jet;
            heading(upper(aircraft::to_string(j)));
            field("PROPULSION", std::string(aircraft::engine_designation(j)));
            field("PERFORMANCE SPEC", std::string(aircraft::signature_spec(j)));
            note(jet_note(j));
            note("The AI bandit will fly the same aircraft type.");
        }
    }

    [[nodiscard]] int chosen() const noexcept {
        switch (step_) {
            case Step::GEOMETRY: return std::clamp(choice_.geometry, 0, static_cast<int>(kDogfightGeometries.size()) - 1);
            case Step::LOADOUT: return std::clamp(choice_.loadout, 0, static_cast<int>(kDogfightLoadouts.size()) - 1);
            case Step::SKILL: return std::clamp(choice_.skill, 0, static_cast<int>(kDogfightSkills.size()) - 1);
            case Step::AIRCRAFT: {
                for (size_t i = 0; i < kDogfightJets.size(); ++i) {
                    if (kDogfightJets[i] == choice_.own_jet) return static_cast<int>(i);
                }
                return 0;
            }
        }
        return 0;
    }

private:
    Step step_;
    DogfightChoice& choice_;
    Rect panel_{}, footer_{}, detail_{};
    std::vector<Rect> option_rects_;

    [[nodiscard]] int option_count() const noexcept {
        switch (step_) {
            case Step::GEOMETRY: return static_cast<int>(kDogfightGeometries.size());
            case Step::LOADOUT: return static_cast<int>(kDogfightLoadouts.size());
            case Step::SKILL: return static_cast<int>(kDogfightSkills.size());
            case Step::AIRCRAFT: return static_cast<int>(kDogfightJets.size());
        }
        return 0;
    }

    void choose(int i) noexcept {
        switch (step_) {
            case Step::GEOMETRY: choice_.geometry = i; break;
            case Step::LOADOUT: choice_.loadout = i; break;
            case Step::SKILL: choice_.skill = i; break;
            case Step::AIRCRAFT:
                choice_.own_jet = kDogfightJets[static_cast<size_t>(i)];
                choice_.bandit_jet = choice_.own_jet;
                break;
        }
    }

    [[nodiscard]] static std::string upper(std::string_view s) {
        std::string r(s);
        for (char& ch : r) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
        return r;
    }

    [[nodiscard]] static const char* jet_note(aircraft::AircraftType j) noexcept {
        switch (j) {
            case aircraft::AircraftType::F16_FIGHTING_FALCON: return "Agile 9 G dogfighter with 510-round M61A1 Vulcan cannon.";
            case aircraft::AircraftType::F15EX_EAGLE_II: return "Twin-engine powerhouse with massive thrust-to-weight and energy retention.";
            case aircraft::AircraftType::EUROFIGHTER_TYPHOON: return "Delta-canard with rapid pitch-rate authority and Mauser BK-27 cannon.";
            case aircraft::AircraftType::F22_RAPTOR: return "Stealth super-cruiser with 2D pitch thrust vectoring for post-stall BFM.";
            case aircraft::AircraftType::A10_THUNDERBOLT: return "GAU-8/A 30 mm Avenger cannon; subsonic turn-fighter.";
            case aircraft::AircraftType::F35B_LIGHTNING_II: return "Advanced 5th-gen STOVL stealth multirole with GAU-22/A 25 mm gun pod.";
        }
        return "";
    }
};

} // namespace fastjet::ui
