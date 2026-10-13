#pragma once

#include "fastjet/ui/input_events.hpp"
#include "fastjet/ui/screen.hpp"
#include "fastjet/ui/screens/credits_screen.hpp"
#include "fastjet/ui/screens/dogfight_setup_screen.hpp"
#include "fastjet/ui/screens/evade_setup_screen.hpp"
#include "fastjet/ui/screens/main_menu_screen.hpp"
#include "fastjet/ui/screens/settings_screen.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <climits>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

/// @file
/// @brief Menu router: owns screens, focus, pointer state, transitions,
/// modals, popups and toasts. Contains no device or graphics API calls.
///
/// Frame protocol for the host application:
///   1. handle(event) for each translated input event;
///   2. update(dt, framebuffer_w, framebuffer_h);
///   3. render(draw_list) and hand the list to graphics::UiCanvas;
///   4. drain poll_action() and act on the requests.

namespace fastjet::ui {

class MenuSystem final : private MenuServices {
public:
    using KeyNameProvider = std::function<std::string(int32_t)>;

    MenuSystem(SettingsManager& settings, AppInfo info) : settings_(settings), info_(std::move(info)) {
        screens_[index(ScreenId::MAIN)] = std::make_unique<MainMenuScreen>();
        screens_[index(ScreenId::SETTINGS)] = std::make_unique<SettingsScreen>();
        screens_[index(ScreenId::CREDITS)] = std::make_unique<CreditsScreen>();
        screens_[index(ScreenId::EVADE_DIFFICULTY)] =
            std::make_unique<EvadeSetupScreen>(EvadeSetupScreen::Step::DIFFICULTY, evade_choice_);
        screens_[index(ScreenId::EVADE_AIRCRAFT)] =
            std::make_unique<EvadeSetupScreen>(EvadeSetupScreen::Step::AIRCRAFT, evade_choice_);
        screens_[index(ScreenId::DOGFIGHT_GEOMETRY)] =
            std::make_unique<DogfightSetupScreen>(DogfightSetupScreen::Step::GEOMETRY, dogfight_choice_);
        screens_[index(ScreenId::DOGFIGHT_LOADOUT)] =
            std::make_unique<DogfightSetupScreen>(DogfightSetupScreen::Step::LOADOUT, dogfight_choice_);
        screens_[index(ScreenId::DOGFIGHT_SKILL)] =
            std::make_unique<DogfightSetupScreen>(DogfightSetupScreen::Step::SKILL, dogfight_choice_);
        screens_[index(ScreenId::DOGFIGHT_AIRCRAFT)] =
            std::make_unique<DogfightSetupScreen>(DogfightSetupScreen::Step::AIRCRAFT, dogfight_choice_);
        rebuild_theme();
        subscription_ = settings_.subscribe([this](SettingsEvent, const SettingsManager&) { rebuild_theme(); });
    }
    ~MenuSystem() override { settings_.unsubscribe(subscription_); }

    MenuSystem(const MenuSystem&) = delete;
    MenuSystem& operator=(const MenuSystem&) = delete;

    // -- Host configuration --------------------------------------------------

    void set_key_name_provider(KeyNameProvider provider) { key_names_ = std::move(provider); }
    void set_resolutions(std::vector<Resolution> list) { resolutions_ = std::move(list); }
    void set_status_text(std::string text) { status_ = std::move(text); }
    void set_current_aircraft(aircraft::AircraftType type) noexcept { current_aircraft_ = type; }
    /// @brief Difficulty index (sim::EvadeDifficulty order) the evade set-up opens on.
    void set_evade_difficulty(int difficulty) noexcept { evade_choice_.difficulty = difficulty; }
    [[nodiscard]] const DogfightChoice& dogfight_choice() const noexcept { return dogfight_choice_; }
    [[nodiscard]] DogfightChoice& dogfight_choice() noexcept { return dogfight_choice_; }
    /// @brief Quality preset the GL context was created with (restart detection).
    void set_launch_quality(QualityPreset q) noexcept { launch_quality_ = q; }

    // -- Lifecycle -----------------------------------------------------------

    /// @param in_flight True when opened as a pause menu over an active flight.
    void open(ScreenId screen, bool in_flight) {
        in_flight_ = in_flight;
        open_ = true;
        visible_ = true;
        modal_.reset();
        dropdown_.reset();
        capture_.reset();
        transition_.active = false;
        press_id_ = drag_id_ = kNoWidget;
        if (theme_.reduce_motion) open_t_ = 1.0f;
        enter_screen(screen);
    }

    /// @brief Starts the fade-out; input is released immediately.
    void close() {
        if (!open_) return;
        open_ = false;
        modal_.reset();
        dropdown_.reset();
        capture_.reset();
        press_id_ = drag_id_ = kNoWidget;
    }

    [[nodiscard]] bool is_open() const noexcept { return open_; }
    [[nodiscard]] bool is_visible() const noexcept { return visible_; }
    [[nodiscard]] bool is_capturing_key() const noexcept { return open_ && capture_.has_value(); }

    [[nodiscard]] std::optional<MenuAction> poll_action() {
        if (actions_.empty()) return std::nullopt;
        const MenuAction a = actions_.front();
        actions_.pop_front();
        return a;
    }

    /// @brief Opens straight onto a settings tab (tooling / screenshots).
    void show_settings_tab(SettingsSection section) {
        if (current_ != ScreenId::SETTINGS) enter_screen(ScreenId::SETTINGS);
        settings_screen().select_tab(section, *this);
        relayout();
    }

    // -- Frame ---------------------------------------------------------------

    void handle(const InputEvent& e) {
        if (!open_) return;
        if (e.type == InputEvent::Type::POINTER_MOVE || e.type == InputEvent::Type::POINTER_DOWN) {
            pointer_x_ = e.x;
            pointer_y_ = e.y;
        }
        // Ignore commands mid-transition so a double press cannot act on a
        // screen the user can no longer see.
        if (transition_.active && e.type != InputEvent::Type::POINTER_MOVE) return;

        switch (e.type) {
            case InputEvent::Type::NAVIGATE:
                focus_visible_ = true;
                on_nav(e.nav);
                break;
            case InputEvent::Type::KEY_CAPTURE: on_key_capture(e.scancode); break;
            case InputEvent::Type::POINTER_MOVE: on_pointer_move(e.x, e.y); break;
            case InputEvent::Type::POINTER_DOWN: on_pointer_down(e.x, e.y); break;
            case InputEvent::Type::POINTER_UP: on_pointer_up(e.x, e.y); break;
            case InputEvent::Type::SCROLL: on_scroll(e.scroll); break;
        }
        relayout();
    }

    void update(float dt, float fb_w, float fb_h) {
        fb_w_ = fb_w;
        fb_h_ = fb_h;
        if (!visible_) return;
        const Motion& mo = theme_.motion;

        if (open_) {
            open_t_ = theme_.reduce_motion ? 1.0f : std::min(1.0f, open_t_ + dt / mo.menu_open);
        } else {
            open_t_ = theme_.reduce_motion ? 0.0f : std::max(0.0f, open_t_ - dt / mo.menu_open);
            if (open_t_ <= 0.0f) visible_ = false;
        }
        if (!theme_.reduce_motion) time_ += dt;

        if (transition_.active) {
            transition_.t += dt;
            if (transition_.phase_out && transition_.t >= mo.view_out) {
                enter_screen(transition_.to);
                transition_.phase_out = false;
                transition_.t = 0.0f;
            } else if (!transition_.phase_out && transition_.t >= mo.view_in) {
                transition_.active = false;
            }
        }

        scroll_y_ += (scroll_target_ - scroll_y_) * theme_.approach(dt, mo.scroll);
        if (std::fabs(scroll_target_ - scroll_y_) < 0.5f) scroll_y_ = scroll_target_;

        if (modal_) modal_->t = theme_.reduce_motion ? 1.0f : std::min(1.0f, modal_->t + dt / mo.modal);
        if (toast_) {
            toast_->age += dt;
            if (toast_->age > mo.toast_hold + mo.toast_fade) toast_.reset();
        }

        screen().update(dt, *this);
        relayout();
        update_animations(dt);
    }

    void render(DrawList& dl) {
        if (!visible_ || fb_w_ <= 0.0f || fb_h_ <= 0.0f) return;
        const LayoutContext ctx = make_context();
        const WidgetPainter painter(theme_, scale_);
        const Motion& mo = theme_.motion;

        const float open_e = ease_out_cubic(open_t_);
        dl.push_layer(open_e, 0.0f, 0.0f, 0.985f + 0.015f * open_e, fb_w_ * 0.5f, fb_h_ * 0.5f);

        float view_opacity = 1.0f;
        float view_dx = 0.0f;
        if (transition_.active) {
            const float slide = ctx.px(mo.slide_distance) * static_cast<float>(transition_.direction);
            if (transition_.phase_out) {
                const float e = ease_in_cubic(transition_.t / mo.view_out);
                view_opacity = 1.0f - e;
                view_dx = -slide * e;
            } else {
                const float e = ease_out_cubic(transition_.t / mo.view_in);
                view_opacity = e;
                view_dx = slide * (1.0f - e);
            }
        }
        dl.push_layer(view_opacity, view_dx, 0.0f);

        const bool ring = focus_visible_ && !modal_;
        screen().paint_background(ctx, *this, dl);
        if (region_) {
            dl.push_clip(region_->view);
            dl.push_layer(screen().content_opacity(), screen().content_offset_x(), 0.0f);
            screen().paint_scroll_content(ctx, *this, dl);
            for (const Widget& w : widgets_) {
                if (w.in_scroll && intersects(w.rect, region_->view)) painter.paint(w, anims_[w.id], dl, ring && w.id == focus_id_);
            }
            dl.pop_layer();
            dl.pop_clip();
            paint_scrollbar(dl);
        }
        for (const Widget& w : widgets_) {
            if (!w.in_scroll) painter.paint(w, anims_[w.id], dl, ring && w.id == focus_id_);
        }
        screen().paint_overlay(ctx, *this, dl);
        dl.pop_layer();

        if (dropdown_) paint_dropdown(dl, painter);
        if (modal_) paint_modal(dl, painter);
        if (capture_) paint_capture(dl);
        if (toast_) paint_toast(dl);
        dl.pop_layer();
    }

    // -- Introspection (tests / tooling) ---------------------------------------

    [[nodiscard]] ScreenId current_screen() const noexcept { return current_; }
    [[nodiscard]] const std::vector<Widget>& widgets() const noexcept { return modal_ ? modal_->widgets : widgets_; }
    [[nodiscard]] WidgetId focused_id() const noexcept { return focus_id_; }
    [[nodiscard]] bool has_modal() const noexcept { return modal_.has_value(); }
    [[nodiscard]] bool has_dropdown() const noexcept { return dropdown_.has_value(); }
    [[nodiscard]] bool is_transitioning() const noexcept { return transition_.active; }
    [[nodiscard]] float scroll_offset() const noexcept { return scroll_target_; }
    [[nodiscard]] const Theme& active_theme() const noexcept { return theme_; }
    [[nodiscard]] SettingsScreen& settings_screen() { return static_cast<SettingsScreen&>(*screens_[index(ScreenId::SETTINGS)]); }
    [[nodiscard]] const std::optional<std::string>& toast_text() const noexcept { return toast_text_; }
    /// @brief Threat level and jet picked on the evade set-up screens (read on START_EVADE).
    [[nodiscard]] const EvadeChoice& evade_choice() const noexcept { return evade_choice_; }

private:
    // -- State -----------------------------------------------------------------

    struct Transition {
        bool active = false;
        bool phase_out = true;
        ScreenId to = ScreenId::MAIN;
        int direction = 1;
        float t = 0.0f;
    };

    struct ModalState {
        ModalSpec spec;
        std::vector<Widget> widgets;
        Rect card{};
        std::vector<std::string> lines;
        float t = 0.0f;
        WidgetId restore_focus = kNoWidget;
    };

    struct DropdownState {
        WidgetId anchor = kNoWidget;
        std::vector<std::string> options;
        int current = 0;
        int highlight = 0;
        int first = 0;
        std::function<void(int)> on_pick;
        Rect list{};
    };

    struct CaptureState {
        std::string title;
        std::function<bool(int32_t, std::string&)> on_key;
        std::function<void()> on_clear;
        WidgetId anchor = kNoWidget;
        std::string error;
    };

    struct Toast {
        std::string text;
        ToastKind kind = ToastKind::INFO;
        float age = 0.0f;
    };

    static constexpr WidgetId kModalBase = 0xF000;

    SettingsManager& settings_;
    SettingsManager::SubscriptionId subscription_ = 0;
    AppInfo info_;
    Theme theme_{};
    float scale_ = 1.0f;
    KeyNameProvider key_names_;
    std::vector<Resolution> resolutions_;
    std::string status_;
    aircraft::AircraftType current_aircraft_ = aircraft::AircraftType::F16_FIGHTING_FALCON;
    EvadeChoice evade_choice_{};
    DogfightChoice dogfight_choice_{};
    QualityPreset launch_quality_ = QualityPreset::ULTRA;

    std::array<std::unique_ptr<Screen>, static_cast<size_t>(ScreenId::COUNT)> screens_;
    ScreenId current_ = ScreenId::MAIN;
    bool in_flight_ = false;
    bool open_ = false;
    bool visible_ = false;
    float open_t_ = 0.0f;
    float time_ = 0.0f;
    Transition transition_{};

    std::vector<Widget> widgets_;
    std::optional<ScrollRegion> region_;
    std::unordered_map<WidgetId, WidgetAnim> anims_;
    WidgetId focus_id_ = kNoWidget;
    NavCoord last_focus_nav_{};
    WidgetId press_id_ = kNoWidget;
    WidgetId drag_id_ = kNoWidget;
    bool focus_visible_ = false;
    bool ensure_focus_visible_ = false;
    float pointer_x_ = -1.0f;
    float pointer_y_ = -1.0f;
    float scroll_y_ = 0.0f;
    float scroll_target_ = 0.0f;
    float fb_w_ = 0.0f;
    float fb_h_ = 0.0f;

    std::optional<ModalState> modal_;
    std::optional<DropdownState> dropdown_;
    std::optional<CaptureState> capture_;
    std::optional<Toast> toast_;
    std::optional<std::string> toast_text_;
    std::deque<MenuAction> actions_;

    [[nodiscard]] static size_t index(ScreenId id) noexcept { return static_cast<size_t>(id); }
    [[nodiscard]] Screen& screen() { return *screens_[index(current_)]; }
    [[nodiscard]] float px(float v) const noexcept { return v * scale_; }

    [[nodiscard]] static bool intersects(const Rect& a, const Rect& b) noexcept {
        return a.x < b.right() && a.right() > b.x && a.y < b.bottom() && a.bottom() > b.y;
    }

    void rebuild_theme() {
        theme_ = build_theme(settings_.pending().accessibility);
        // UI scale follows the committed value: rescaling live would move the
        // slider out from under the pointer while it is being dragged.
        scale_ = fb_w_ > 0.0f ? theme_.scale_for(fb_w_, fb_h_, settings_.committed().accessibility.ui_scale) : 1.0f;
    }

    [[nodiscard]] LayoutContext make_context() const {
        const float nx = (fb_w_ > 0.0f && pointer_x_ >= 0.0f) ? (pointer_x_ / fb_w_) * 2.0f - 1.0f : 0.0f;
        const float ny = (fb_h_ > 0.0f && pointer_y_ >= 0.0f) ? (pointer_y_ / fb_h_) * 2.0f - 1.0f : 0.0f;
        const bool still = theme_.reduce_motion;
        return LayoutContext{theme_, {0.0f, 0.0f, fb_w_, fb_h_}, scale_, time_, scroll_y_, still ? 0.0f : -nx, still ? 0.0f : -ny};
    }

    void enter_screen(ScreenId id) {
        current_ = id;
        anims_.clear();
        scroll_y_ = scroll_target_ = 0.0f;
        hover_reset();
        screen().on_enter(*this);
        focus_id_ = kNoWidget;
        relayout();
        focus_id_ = screen().default_focus();
        if (const Widget* w = find(widgets_, focus_id_)) last_focus_nav_ = w->nav;
    }

    void hover_reset() { press_id_ = drag_id_ = kNoWidget; }

    [[nodiscard]] std::vector<Widget>& active_widgets() { return modal_ ? modal_->widgets : widgets_; }

    [[nodiscard]] static const Widget* find(const std::vector<Widget>& list, WidgetId id) {
        for (const Widget& w : list) {
            if (w.id == id) return &w;
        }
        return nullptr;
    }

    void relayout() {
        if (fb_w_ <= 0.0f || fb_h_ <= 0.0f) return;
        scale_ = theme_.scale_for(fb_w_, fb_h_, settings_.committed().accessibility.ui_scale);
        widgets_.clear();
        const LayoutContext ctx = make_context();
        screen().layout(ctx, *this, widgets_);
        region_ = screen().scroll_region();
        const float max_scroll = region_ ? std::max(0.0f, region_->content_h - region_->view.h) : 0.0f;
        scroll_target_ = std::clamp(scroll_target_, 0.0f, max_scroll);
        scroll_y_ = std::clamp(scroll_y_, 0.0f, max_scroll);

        if (modal_) layout_modal();
        if (dropdown_) layout_dropdown();

        // Keep focus on something usable: if the focused widget vanished or
        // became disabled (e.g. APPLY after applying), move to its nearest
        // neighbour on the same row before falling back to the default.
        auto& list = active_widgets();
        const Widget* f = find(list, focus_id_);
        if (!f || f->disabled) {
            const Widget* best = nullptr;
            for (const Widget& w : list) {
                if (w.disabled || w.nav.group != last_focus_nav_.group || w.nav.row != last_focus_nav_.row) continue;
                if (!best || std::abs(w.nav.col - last_focus_nav_.col) < std::abs(best->nav.col - last_focus_nav_.col)) best = &w;
            }
            if (!best && !modal_) best = find(list, screen().default_focus());
            if (!best) {
                for (const Widget& w : list) {
                    if (!w.disabled) { best = &w; break; }
                }
            }
            focus_id_ = best ? best->id : kNoWidget;
        }
        if (const Widget* w = find(list, focus_id_)) {
            last_focus_nav_ = w->nav;
            if (ensure_focus_visible_) {
                ensure_visible(*w);
                ensure_focus_visible_ = false;
            }
        }
    }

    void update_animations(float dt) {
        const Motion& mo = theme_.motion;
        const float ks = theme_.approach(dt, mo.state);
        const float kf = theme_.approach(dt, mo.focus);
        const float kt = theme_.approach(dt, mo.toggle);
        auto step = [&](const std::vector<Widget>& list, bool active) {
            for (const Widget& w : list) {
                WidgetAnim& a = anims_[w.id];
                if (!a.seen) {
                    a.toggle = w.checked ? 1.0f : 0.0f;
                    a.seen = true;
                }
                const bool focused = active && w.id == focus_id_ && !w.disabled;
                const bool pressed = active && (w.id == press_id_ || w.id == drag_id_);
                a.hover += ((focused ? 1.0f : 0.0f) - a.hover) * ks;
                a.focus += ((focused ? 1.0f : 0.0f) - a.focus) * kf;
                a.press += ((pressed ? 1.0f : 0.0f) - a.press) * ks;
                a.toggle += ((w.checked ? 1.0f : 0.0f) - a.toggle) * kt;
            }
        };
        step(widgets_, !modal_);
        if (modal_) step(modal_->widgets, true);
    }

    // -- Focus traversal -------------------------------------------------------

    void focus(const Widget& w, bool scroll_into_view) {
        focus_id_ = w.id;
        last_focus_nav_ = w.nav;
        if (scroll_into_view) ensure_visible(w);
    }

    /// @brief Moves focus one step; returns false if nothing lies that way.
    bool move_focus(NavCommand c) {
        auto& list = active_widgets();
        const Widget* cur = find(list, focus_id_);
        if (!cur) {
            for (const Widget& w : list) {
                if (!w.disabled) { focus(w, true); return true; }
            }
            return false;
        }
        const NavCoord n = cur->nav;
        const float cx = cur->rect.cx();
        const WidgetId cur_id = cur->id;
        const Widget* best = nullptr;

        // Pick within a row: an active tab first, then the closest column centre.
        auto pick_in_row = [&](int group, int row) {
            const Widget* pick = nullptr;
            for (const Widget& w : list) {
                if (w.disabled || w.id == cur_id || w.nav.group != group || w.nav.row != row) continue;
                if (!pick || (w.selected && !pick->selected) ||
                    (w.selected == pick->selected && std::fabs(w.rect.cx() - cx) < std::fabs(pick->rect.cx() - cx))) {
                    pick = &w;
                }
            }
            return pick;
        };

        if (c == NavCommand::UP || c == NavCommand::DOWN) {
            const int dir = c == NavCommand::UP ? -1 : 1;
            int target_row = INT_MAX;
            for (const Widget& w : list) {
                if (w.disabled || w.nav.group != n.group) continue;
                const int d = (w.nav.row - n.row) * dir;
                if (d > 0 && std::abs(w.nav.row - n.row) < std::abs(target_row - n.row)) target_row = w.nav.row;
            }
            if (target_row != INT_MAX) best = pick_in_row(n.group, target_row);
            if (!best) {
                int target_group = INT_MAX;
                for (const Widget& w : list) {
                    if (w.disabled) continue;
                    const int d = (w.nav.group - n.group) * dir;
                    if (d > 0 && std::abs(w.nav.group - n.group) < std::abs(target_group - n.group)) target_group = w.nav.group;
                }
                if (target_group != INT_MAX) {
                    int edge_row = dir > 0 ? INT_MAX : INT_MIN;
                    for (const Widget& w : list) {
                        if (w.disabled || w.nav.group != target_group) continue;
                        edge_row = dir > 0 ? std::min(edge_row, w.nav.row) : std::max(edge_row, w.nav.row);
                    }
                    best = pick_in_row(target_group, edge_row);
                }
            }
        } else {
            const int dir = c == NavCommand::LEFT ? -1 : 1;
            for (const Widget& w : list) {
                if (w.disabled || w.id == cur_id || w.nav.group != n.group || w.nav.row != n.row) continue;
                const int d = (w.nav.col - n.col) * dir;
                if (d > 0 && (!best || std::abs(w.nav.col - n.col) < std::abs(best->nav.col - n.col))) best = &w;
            }
        }
        if (!best) return false;
        focus(*best, true);
        return true;
    }

    void ensure_visible(const Widget& w) {
        if (!w.in_scroll || !region_) return;
        const Rect& view = region_->view;
        const float pad = px(theme_.space.md);
        const float top = w.rect.y - view.y + scroll_y_; // content-space
        const float bottom = top + w.rect.h;
        if (top - pad < scroll_target_) scroll_target_ = top - pad;
        if (bottom + pad > scroll_target_ + view.h) scroll_target_ = bottom + pad - view.h;
        // Reveal the section heading above the first rows.
        if (top < px(theme_.metrics.section_h + theme_.metrics.row_h)) scroll_target_ = 0.0f;
        const float max_scroll = std::max(0.0f, region_->content_h - view.h);
        scroll_target_ = std::clamp(scroll_target_, 0.0f, max_scroll);
    }

    void scroll_by(float amount) {
        if (!region_) return;
        const float max_scroll = std::max(0.0f, region_->content_h - region_->view.h);
        scroll_target_ = std::clamp(scroll_target_ + amount, 0.0f, max_scroll);
    }

    // -- Input -----------------------------------------------------------------

    void on_nav(NavCommand c) {
        if (capture_) {
            if (c == NavCommand::BACK) {
                capture_.reset();
            } else if (c == NavCommand::CLEAR) {
                auto clear = capture_->on_clear;
                capture_.reset();
                if (clear) clear();
            }
            return;
        }
        if (dropdown_) {
            dropdown_nav(c);
            return;
        }
        if (modal_) {
            modal_nav(c);
            return;
        }

        const Widget* fw = find(widgets_, focus_id_);
        const std::optional<Widget> focused = fw ? std::optional<Widget>(*fw) : std::nullopt;
        switch (c) {
            case NavCommand::UP:
            case NavCommand::DOWN:
                if (!move_focus(c)) scroll_by((c == NavCommand::UP ? -1.0f : 1.0f) * px(theme_.metrics.scroll_step));
                break;
            case NavCommand::LEFT:
            case NavCommand::RIGHT: {
                const int dir = c == NavCommand::LEFT ? -1 : 1;
                if (focused && is_adjustable(focused->kind) && !focused->disabled) {
                    screen().on_adjust(focused->id, dir, false, *this);
                } else if (move_focus(c)) {
                    const Widget* now = find(widgets_, focus_id_);
                    if (now && now->kind == WidgetKind::TAB) screen().on_activate(now->id, *this);
                }
                break;
            }
            case NavCommand::ACCEPT:
                if (focused && !focused->disabled) activate(*focused);
                break;
            case NavCommand::BACK: screen().on_back(*this); break;
            case NavCommand::NEXT_TAB: screen().on_tab(+1, *this); break;
            case NavCommand::PREV_TAB: screen().on_tab(-1, *this); break;
            case NavCommand::CLEAR:
                if (focused) screen().on_clear(focused->id, *this);
                break;
        }
    }

    void activate(const Widget& w) {
        switch (w.kind) {
            case WidgetKind::TOGGLE:
            case WidgetKind::SELECTOR: screen().on_adjust(w.id, +1, true, *this); break;
            case WidgetKind::SLIDER: break;
            default: screen().on_activate(w.id, *this); break;
        }
    }

    void on_pointer_move(float x, float y) {
        focus_visible_ = false;
        if (drag_id_ != kNoWidget) {
            drag_slider(x);
            return;
        }
        if (capture_) return;
        if (dropdown_) {
            const int i = dropdown_hit(x, y);
            if (i >= 0) dropdown_->highlight = i;
            return;
        }
        const Widget* w = hit_test(x, y);
        if (w && !w->disabled && w->id != focus_id_) focus(*w, false);
    }

    void on_pointer_down(float x, float y) {
        focus_visible_ = false;
        if (capture_) {
            capture_.reset(); // Clicking away cancels a rebind
            return;
        }
        if (dropdown_) {
            const int i = dropdown_hit(x, y);
            if (i >= 0) pick_dropdown(i);
            else dropdown_.reset();
            return;
        }
        const Widget* w = hit_test(x, y);
        if (!w || w->disabled) {
            press_id_ = kNoWidget;
            return;
        }
        press_id_ = w->id;
        focus(*w, false);
        if (w->kind == WidgetKind::SLIDER && !modal_) {
            drag_id_ = w->id;
            drag_slider(x);
        }
    }

    void on_pointer_up(float x, float y) {
        const WidgetId pressed = press_id_;
        press_id_ = kNoWidget;
        if (drag_id_ != kNoWidget) {
            drag_id_ = kNoWidget;
            return;
        }
        if (pressed == kNoWidget) return;
        const Widget* w = hit_test(x, y);
        if (!w || w->id != pressed) return; // Released outside: cancel, as native buttons do
        const Widget hit = *w;
        if (modal_) {
            press_modal_button(static_cast<int>(hit.id - kModalBase));
        } else if (hit.kind == WidgetKind::SELECTOR) {
            const Rect box = WidgetPainter(theme_, scale_).selector_box(hit);
            screen().on_adjust(hit.id, x < box.cx() ? -1 : +1, false, *this);
        } else {
            activate(hit);
        }
    }

    void on_scroll(float amount) {
        if (dropdown_) {
            const int visible = std::min<int>(static_cast<int>(dropdown_->options.size()), theme_.metrics.dropdown_max_visible);
            const int max_first = std::max(0, static_cast<int>(dropdown_->options.size()) - visible);
            dropdown_->first = std::clamp(dropdown_->first - (amount > 0.0f ? 1 : -1), 0, max_first);
            return;
        }
        if (modal_ || capture_) return;
        scroll_by(-amount * px(theme_.metrics.scroll_step));
    }

    void on_key_capture(int32_t sc) {
        if (!capture_) return;
        std::string error;
        auto on_key = capture_->on_key;
        if (!on_key || on_key(sc, error)) capture_.reset();
        else capture_->error = error;
    }

    [[nodiscard]] const Widget* hit_test(float x, float y) {
        const auto& list = active_widgets();
        for (auto it = list.rbegin(); it != list.rend(); ++it) {
            if (it->in_scroll && region_ && !region_->view.contains(x, y)) continue;
            if (it->rect.contains(x, y)) return &*it;
        }
        return nullptr;
    }

    void drag_slider(float x) {
        const Widget* w = find(widgets_, drag_id_);
        if (!w) return;
        const Rect track = WidgetPainter(theme_, scale_).slider_track(*w);
        const float t = track.w > 0.0f ? std::clamp((x - track.x) / track.w, 0.0f, 1.0f) : 0.0f;
        screen().on_slider(w->id, t, *this);
    }

    // -- Modal -----------------------------------------------------------------

    void layout_modal() {
        ModalState& m = *modal_;
        const Theme& t = theme_;
        const float w = std::min(px(t.metrics.modal_w), fb_w_ - px(t.space.lg) * 2.0f);
        const float pad = px(t.space.xl);
        const float body = px(t.type.body * 0.9f);
        m.lines = wrap_text(m.spec.message, w - pad * 2.0f, body);
        const float line_h = body * 1.45f;
        const float bh = px(t.metrics.footer_button_h);
        const float h = pad + px(t.type.heading) + px(t.space.lg) + static_cast<float>(m.lines.size()) * line_h + pad + bh + pad;
        m.card = {fb_w_ * 0.5f - w * 0.5f, fb_h_ * 0.5f - h * 0.5f, w, h};

        m.widgets.clear();
        float x = m.card.right() - pad;
        const float gap = px(t.space.md);
        for (int i = static_cast<int>(m.spec.buttons.size()) - 1; i >= 0; --i) {
            const ModalButton& b = m.spec.buttons[static_cast<size_t>(i)];
            const float bw = std::max(px(t.metrics.footer_button_min_w),
                                      text_width(b.label, px(t.type.body * 0.9f), px(t.type.tracking_button)) + px(t.space.xxl) * 1.5f);
            x -= bw;
            Widget wd;
            wd.id = kModalBase + static_cast<WidgetId>(i);
            wd.kind = WidgetKind::BUTTON;
            wd.variant = b.variant;
            wd.label = b.label;
            wd.rect = {x, m.card.bottom() - pad - bh, bw, bh};
            wd.nav = {0, 0, i};
            m.widgets.insert(m.widgets.begin(), std::move(wd));
            x -= gap;
        }
    }

    void modal_nav(NavCommand c) {
        switch (c) {
            case NavCommand::LEFT:
            case NavCommand::UP: move_focus(NavCommand::LEFT); break;
            case NavCommand::RIGHT:
            case NavCommand::DOWN: move_focus(NavCommand::RIGHT); break;
            case NavCommand::ACCEPT:
                if (focus_id_ >= kModalBase) press_modal_button(static_cast<int>(focus_id_ - kModalBase));
                break;
            case NavCommand::BACK:
                if (modal_->spec.cancel_index >= 0) press_modal_button(modal_->spec.cancel_index);
                else close_modal();
                break;
            default: break;
        }
    }

    void close_modal() {
        if (!modal_) return;
        const WidgetId restore = modal_->restore_focus;
        modal_.reset();
        focus_id_ = restore;
    }

    void press_modal_button(int i) {
        if (!modal_ || i < 0 || i >= static_cast<int>(modal_->spec.buttons.size())) return;
        auto fn = modal_->spec.buttons[static_cast<size_t>(i)].on_press;
        close_modal();
        if (fn) fn(); // After closing, so the callback may open another modal
    }

    // -- Dropdown ----------------------------------------------------------------

    void layout_dropdown() {
        DropdownState& d = *dropdown_;
        const Widget* anchor = find(widgets_, d.anchor);
        if (!anchor) {
            dropdown_.reset();
            return;
        }
        const Rect box = WidgetPainter(theme_, scale_).selector_box(*anchor);
        const int n = static_cast<int>(d.options.size());
        const int visible = std::min(n, theme_.metrics.dropdown_max_visible);
        const float item_h = px(theme_.metrics.dropdown_item_h);
        const float pad = px(theme_.space.xs);
        const float h = static_cast<float>(visible) * item_h + pad * 2.0f;
        float y = box.bottom() + px(theme_.space.xs);
        if (y + h > fb_h_ - px(theme_.space.lg)) y = box.y - px(theme_.space.xs) - h; // Flip above
        d.list = {box.x, y, box.w, h};
        d.first = std::clamp(d.first, 0, std::max(0, n - visible));
    }

    [[nodiscard]] int dropdown_hit(float x, float y) const {
        const DropdownState& d = *dropdown_;
        if (!d.list.contains(x, y)) return -1;
        const float item_h = px(theme_.metrics.dropdown_item_h);
        const int i = d.first + static_cast<int>((y - d.list.y - px(theme_.space.xs)) / item_h);
        return (i >= 0 && i < static_cast<int>(d.options.size())) ? i : -1;
    }

    void dropdown_nav(NavCommand c) {
        DropdownState& d = *dropdown_;
        const int n = static_cast<int>(d.options.size());
        const int visible = std::min(n, theme_.metrics.dropdown_max_visible);
        switch (c) {
            case NavCommand::UP: d.highlight = std::max(0, d.highlight - 1); break;
            case NavCommand::DOWN: d.highlight = std::min(n - 1, d.highlight + 1); break;
            case NavCommand::ACCEPT: pick_dropdown(d.highlight); return;
            case NavCommand::BACK: dropdown_.reset(); return;
            default: return;
        }
        if (d.highlight < d.first) d.first = d.highlight;
        if (d.highlight >= d.first + visible) d.first = d.highlight - visible + 1;
    }

    void pick_dropdown(int i) {
        auto fn = dropdown_->on_pick;
        dropdown_.reset();
        if (fn) fn(i);
    }

    // -- MenuServices ------------------------------------------------------------

    SettingsManager& settings() override { return settings_; }
    const Theme& theme() const override { return theme_; }
    const AppInfo& app_info() const override { return info_; }
    bool in_flight() const override { return in_flight_; }
    const std::string& status_text() const override { return status_; }
    QualityPreset launch_quality() const override { return launch_quality_; }
    WidgetId focused() const override { return focus_id_; }

    const std::vector<Resolution>& resolutions() const override {
        static const std::vector<Resolution> kFallback = {
            {3840, 2160}, {2560, 1440}, {1920, 1080}, {1600, 900}, {1366, 768}, {1280, 720}};
        return resolutions_.empty() ? kFallback : resolutions_;
    }

    aircraft::AircraftType current_aircraft() const override { return current_aircraft_; }

    std::string key_name(int32_t sc) const override {
        if (key_names_) {
            std::string name = key_names_(sc);
            if (!name.empty()) {
                for (char& ch : name) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
                return name;
            }
        }
        return "KEY " + std::to_string(sc);
    }

    void navigate(ScreenId to, bool forward) override {
        if (theme_.reduce_motion) {
            enter_screen(to);
            return;
        }
        transition_ = Transition{true, true, to, forward ? 1 : -1, 0.0f};
    }

    void emit(MenuAction action) override { actions_.push_back(action); }

    void show_modal(ModalSpec spec) override {
        ModalState m;
        m.restore_focus = modal_ ? modal_->restore_focus : focus_id_;
        m.t = theme_.reduce_motion ? 1.0f : 0.0f;
        const int def = std::clamp(spec.default_index, 0, std::max(0, static_cast<int>(spec.buttons.size()) - 1));
        m.spec = std::move(spec);
        modal_ = std::move(m);
        focus_id_ = kModalBase + static_cast<WidgetId>(def);
        layout_modal();
    }

    void toast(std::string text, ToastKind kind) override {
        toast_text_ = text;
        toast_ = Toast{std::move(text), kind, 0.0f};
    }

    void open_dropdown(WidgetId anchor, std::vector<std::string> options, int current, std::function<void(int)> on_pick) override {
        if (options.empty()) return;
        DropdownState d;
        d.anchor = anchor;
        d.current = std::clamp(current, 0, static_cast<int>(options.size()) - 1);
        d.highlight = d.current;
        d.first = std::max(0, d.current - theme_.metrics.dropdown_max_visible / 2);
        d.options = std::move(options);
        d.on_pick = std::move(on_pick);
        dropdown_ = std::move(d);
        layout_dropdown();
    }

    void begin_key_capture(std::string title, std::function<bool(int32_t, std::string&)> on_key,
                           std::function<void()> on_clear) override {
        capture_ = CaptureState{std::move(title), std::move(on_key), std::move(on_clear), focus_id_, {}};
    }

    bool is_capturing(WidgetId id) const override { return capture_ && capture_->anchor == id; }

    void set_focus(WidgetId id) override {
        focus_id_ = id;
        ensure_focus_visible_ = true;
    }

    void reset_scroll() override { scroll_y_ = scroll_target_ = 0.0f; }

    // -- Painting ----------------------------------------------------------------

    void paint_scrollbar(DrawList& dl) const {
        if (!region_ || region_->content_h <= region_->view.h + 1.0f) return;
        const Rect& v = region_->view;
        const float max_scroll = region_->content_h - v.h;
        const float w = px(theme_.metrics.scrollbar_w);
        const float thumb_h = std::max(px(32.0f), v.h * v.h / region_->content_h);
        const float y = v.y + (v.h - thumb_h) * (scroll_y_ / max_scroll);
        const float x = v.right() - w;
        dl.fill({x, v.y, w, v.h}, theme_.palette.surface_void.faded(0.6f), w * 0.5f);
        dl.fill({x, y, w, thumb_h}, theme_.palette.border_strong, w * 0.5f);
    }

    void paint_dropdown(DrawList& dl, const WidgetPainter& painter) const {
        const DropdownState& d = *dropdown_;
        const Theme& t = theme_;
        const Palette& p = t.palette;
        const float radius = px(t.radius.md);
        dl.shadow(d.list, radius, px(t.elevation.raised.blur * 1.5f), p.shadow.faded(0.6f), px(t.elevation.raised.offset_y));
        dl.rect(d.list, RectStyle::outlined(p.surface_header, p.border_bright, px(t.border.hairline), radius));
        const float item_h = px(t.metrics.dropdown_item_h);
        const float pad = px(t.space.xs);
        const int n = static_cast<int>(d.options.size());
        const int visible = std::min(n, t.metrics.dropdown_max_visible);
        for (int k = 0; k < visible; ++k) {
            const int i = d.first + k;
            const Rect item{d.list.x + pad, d.list.y + pad + static_cast<float>(k) * item_h, d.list.w - pad * 2.0f, item_h};
            if (i == d.highlight) dl.fill(item, p.selection, px(t.radius.sm));
            const float tx = item.x + px(t.space.xxl);
            if (i == d.current) {
                // Check mark
                const float cx = item.x + px(t.space.lg);
                const float cy = item.cy();
                const float k2 = px(4.0f);
                dl.line(cx - k2, cy, cx - k2 * 0.2f, cy + k2 * 0.8f, px(2.0f), p.accent);
                dl.line(cx - k2 * 0.2f, cy + k2 * 0.8f, cx + k2 * 1.2f, cy - k2, px(2.0f), p.accent);
            }
            dl.text_v_centered(d.options[static_cast<size_t>(i)], tx, item.cy(), px(t.type.label),
                               i == d.highlight ? p.text_primary : p.text_secondary, px(t.type.tracking_button));
        }
        if (n > visible) {
            if (d.first > 0) painter.chevron(dl, d.list.right() - px(t.space.lg), d.list.y + px(t.space.md), -2, p.text_muted);
            if (d.first + visible < n) painter.chevron(dl, d.list.right() - px(t.space.lg), d.list.bottom() - px(t.space.md), +2, p.text_muted);
        }
    }

    void paint_modal(DrawList& dl, const WidgetPainter& painter) {
        const ModalState& m = *modal_;
        const Theme& t = theme_;
        const Palette& p = t.palette;
        const float e = ease_out_cubic(m.t);
        dl.fill({0.0f, 0.0f, fb_w_, fb_h_}, p.backdrop.faded(e));
        const float scale = t.motion.modal_scale_from + (1.0f - t.motion.modal_scale_from) * e;
        dl.push_layer(e, 0.0f, 0.0f, scale, m.card.cx(), m.card.cy());
        paint_panel(dl, t, scale_, m.card, t.elevation.modal);
        const float pad = px(t.space.xl);
        float y = m.card.y + pad + px(t.type.heading) * kFontCapHeight;
        dl.text(m.spec.title, m.card.x + pad, y, px(t.type.heading), p.text_primary, px(t.type.tracking_button));
        y += px(t.space.lg) + px(t.type.body * 0.9f);
        for (const std::string& line : m.lines) {
            dl.text(line, m.card.x + pad, y, px(t.type.body * 0.9f), p.text_secondary);
            y += px(t.type.body * 0.9f) * 1.45f;
        }
        for (const Widget& w : m.widgets) painter.paint(w, anims_[w.id], dl, focus_visible_ && w.id == focus_id_);
        dl.pop_layer();
    }

    void paint_capture(DrawList& dl) const {
        const CaptureState& c = *capture_;
        const Theme& t = theme_;
        const Palette& p = t.palette;
        dl.fill({0.0f, 0.0f, fb_w_, fb_h_}, p.backdrop);
        const float w = std::min(px(t.metrics.modal_w), fb_w_ - px(t.space.lg) * 2.0f);
        const float pad = px(t.space.xl);
        const float h = px(200.0f);
        const Rect card{fb_w_ * 0.5f - w * 0.5f, fb_h_ * 0.5f - h * 0.5f, w, h};
        paint_panel(dl, t, scale_, card, t.elevation.modal);
        const float pulse = t.reduce_motion ? 1.0f : 0.55f + 0.45f * std::sin(time_ * 5.0f);
        dl.stroke(card.inset(-px(t.border.focus_offset)), p.highlight.faded(pulse), px(t.border.focus), px(t.radius.lg) + px(t.border.focus_offset));
        float y = card.y + pad + px(t.type.label);
        dl.text(fit_text(c.title, w - pad * 2.0f, px(t.type.label), px(t.type.tracking_wide)), card.x + pad, y, px(t.type.label),
                p.text_label, px(t.type.tracking_wide));
        y += px(t.space.xxxl);
        dl.text("PRESS A KEY", card.x + pad, y, px(t.type.title), p.highlight, px(t.type.tracking_button));
        y += px(t.space.xxl);
        dl.text("Esc cancels. Delete clears this slot.", card.x + pad, y, px(t.type.caption), p.text_muted);
        if (!c.error.empty()) {
            y += px(t.space.xl);
            dl.text(fit_text(c.error, w - pad * 2.0f, px(t.type.caption)), card.x + pad, y, px(t.type.caption), p.danger);
        }
    }

    void paint_toast(DrawList& dl) const {
        const Toast& to = *toast_;
        const Theme& t = theme_;
        const Palette& p = t.palette;
        const Motion& mo = t.motion;
        float alpha = 1.0f;
        if (!t.reduce_motion) {
            alpha = std::min(1.0f, to.age / 0.15f);
            if (to.age > mo.toast_hold) alpha *= 1.0f - (to.age - mo.toast_hold) / mo.toast_fade;
        }
        alpha = std::clamp(alpha, 0.0f, 1.0f);
        const float size = px(t.type.body * 0.85f);
        const float pad = px(t.space.lg);
        const float dot = px(8.0f);
        const float w = std::min(fb_w_ - px(t.space.xl) * 2.0f, text_width(to.text, size) + pad * 3.0f + dot);
        const float h = px(44.0f);
        const float rise = (1.0f - ease_out_cubic(std::min(1.0f, to.age / 0.25f))) * px(12.0f);
        const Rect r{fb_w_ * 0.5f - w * 0.5f, fb_h_ - px(t.metrics.panel_margin) - px(t.metrics.footer_h) - h - px(t.space.lg) + rise, w, h};
        const Rgba accent = to.kind == ToastKind::SUCCESS ? p.success : (to.kind == ToastKind::FAILURE ? p.danger : p.accent);
        dl.push_layer(alpha);
        dl.shadow(r, h * 0.5f, px(16.0f), p.shadow.faded(0.6f), px(4.0f));
        dl.rect(r, RectStyle::outlined(p.surface_header, accent, px(t.border.hairline), h * 0.5f));
        dl.fill({r.x + pad, r.cy() - dot * 0.5f, dot, dot}, accent, dot);
        dl.text_v_centered(fit_text(to.text, w - pad * 3.0f - dot, size), r.x + pad * 2.0f + dot, r.cy(), size, p.text_primary);
        dl.pop_layer();
    }
};

} // namespace fastjet::ui
