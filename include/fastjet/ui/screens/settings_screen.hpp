#pragma once

#include "fastjet/ui/screen.hpp"
#include "fastjet/graphics/render_quality.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace fastjet::ui {

/// @brief Tabbed settings editor.
///
/// Each tab is a declarative list of rows (the "schema"). A field row binds a
/// label, hint and control type to an accessor into UserSettings; layout,
/// painting, input and the modified markers are all derived from that table.
class SettingsScreen final : public Screen {
public:
    static constexpr WidgetId kTabBase = 100;
    enum FooterButton : WidgetId { BACK = 200, DEFAULTS, REVERT, APPLY };
    static constexpr WidgetId kRowBase = 1000;
    static constexpr WidgetId kSlotsPerRow = 4; ///< field, primary key, secondary key, spare

    static constexpr int kTabCount = static_cast<int>(SettingsSection::COUNT);

    SettingsScreen() { build_schema(); }

    [[nodiscard]] SettingsSection tab() const noexcept { return tab_; }

    /// @brief Switches tab with a cross-fade; focus moves to the new content.
    void select_tab(SettingsSection section, MenuServices& svc, bool keep_focus_on_tabs = false) {
        if (section == tab_) return;
        tab_dir_ = static_cast<int>(section) > static_cast<int>(tab_) ? 1 : -1;
        tab_ = section;
        tab_anim_ = svc.theme().reduce_motion ? 1.0f : 0.0f;
        svc.reset_scroll();
        svc.set_focus(keep_focus_on_tabs ? tab_id(section) : first_field_id());
    }

    // -- Screen ---------------------------------------------------------------

    void on_enter(MenuServices& svc) override {
        tab_anim_ = 1.0f;
        indicator_init_ = false;
        (void)svc;
    }

    void update(float dt, MenuServices& svc) override {
        const Theme& t = svc.theme();
        tab_anim_ = t.reduce_motion ? 1.0f : std::min(1.0f, tab_anim_ + dt / t.motion.tab_switch);
        const float k = t.approach(dt, t.motion.focus);
        indicator_x_ += (indicator_target_x_ - indicator_x_) * k;
        indicator_w_ += (indicator_target_w_ - indicator_w_) * k;
    }

    [[nodiscard]] WidgetId default_focus() const override { return first_field_id(); }
    [[nodiscard]] std::optional<ScrollRegion> scroll_region() const override { return ScrollRegion{geo_.view, content_h_}; }
    [[nodiscard]] float content_opacity() const override { return ease_out_cubic(tab_anim_); }
    [[nodiscard]] float content_offset_x() const override {
        return (1.0f - ease_out_cubic(tab_anim_)) * slide_px_ * static_cast<float>(tab_dir_);
    }

    void layout(const LayoutContext& ctx, MenuServices& svc, std::vector<Widget>& out) override {
        const Theme& t = ctx.theme;
        const Metrics& m = t.metrics;
        const UserSettings& pend = svc.settings().pending();
        const UserSettings& comm = svc.settings().committed();
        slide_px_ = ctx.px(t.motion.slide_distance);
        refresh_resolutions(svc, pend);
        compute_geometry(ctx);

        // Tabs
        float tx = geo_.panel.x + ctx.px(t.space.xl);
        for (int i = 0; i < kTabCount; ++i) {
            const auto section = static_cast<SettingsSection>(i);
            Widget w;
            w.id = tab_id(section);
            w.kind = WidgetKind::TAB;
            w.label = kTabLabels[static_cast<size_t>(i)];
            const float tw = text_width(w.label, ctx.px(t.type.label), ctx.px(t.type.tracking_wide)) + ctx.px(t.space.xxl) * 1.5f;
            w.rect = {tx, geo_.tabs.y, tw, geo_.tabs.h};
            w.selected = section == tab_;
            w.modified = section_modified(section, pend, comm);
            w.nav = {0, 0, i};
            if (w.selected) {
                indicator_target_x_ = w.rect.x + ctx.px(t.space.lg);
                indicator_target_w_ = w.rect.w - ctx.px(t.space.lg) * 2.0f;
                if (!indicator_init_ || t.reduce_motion) {
                    indicator_x_ = indicator_target_x_;
                    indicator_w_ = indicator_target_w_;
                    indicator_init_ = true;
                }
            }
            out.push_back(std::move(w));
            tx += tw + ctx.px(t.space.xs);
        }

        // Content rows
        const auto& rows = rows_for(tab_);
        row_top_.assign(rows.size(), 0.0f);
        row_h_.assign(rows.size(), 0.0f);
        const float gutter = ctx.px(m.scrollbar_w + t.space.md);
        const float row_w = geo_.view.w - gutter;
        float y = 0.0f;
        for (size_t i = 0; i < rows.size(); ++i) {
            const RowSpec& row = rows[i];
            float h = 0.0f;
            switch (row.kind) {
                case RowKind::SECTION: h = ctx.px(m.section_h); break;
                case RowKind::NOTE:
                    h = static_cast<float>(wrap_text(row.text, row_w - ctx.px(t.space.xl) * 2.0f, ctx.px(t.type.caption)).size()) *
                            ctx.px(t.type.caption * 1.5f) + ctx.px(t.space.md);
                    break;
                case RowKind::FIELD: h = ctx.px(m.row_h); break;
                case RowKind::KEYBIND: h = ctx.px(m.row_h_compact); break;
            }
            row_top_[i] = y;
            row_h_[i] = h;
            const float gap = ctx.px(t.space.xs);
            const Rect rect{geo_.view.x, geo_.view.y + y - ctx.scroll_y + gap * 0.5f, row_w, h - gap};

            if (row.kind == RowKind::FIELD) {
                out.push_back(field_widget(ctx, svc, row.field, i, rect, pend, comm));
            } else if (row.kind == RowKind::KEYBIND) {
                const KeyBinding& b = pend.controls.binding(row.action);
                const KeyBinding& bc = comm.controls.binding(row.action);
                for (int slot = 0; slot < 2; ++slot) {
                    Widget w;
                    w.id = row_widget_id(i, 1 + slot);
                    w.kind = WidgetKind::KEYBIND;
                    w.rect = keycap_rect(ctx, rect, slot);
                    w.nav = {1, static_cast<int>(i), slot};
                    w.in_scroll = true;
                    w.label = row.text;
                    const int32_t sc = b.slot(slot);
                    w.value_text = sc == scancode::NONE ? std::string() : svc.key_name(sc);
                    w.modified = sc != bc.slot(slot);
                    w.capturing = svc.is_capturing(w.id);
                    out.push_back(std::move(w));
                }
            }
            y += h;
        }
        content_h_ = y + ctx.px(t.space.lg);

        // Footer
        const bool dirty = svc.settings().is_dirty();
        const float fh = ctx.px(m.footer_button_h);
        const float fy = geo_.footer.cy() - fh * 0.5f;
        auto footer_button = [&](WidgetId id, const char* label, ButtonVariant v, float x, bool disabled, int col) {
            Widget w;
            w.id = id;
            w.kind = WidgetKind::BUTTON;
            w.variant = v;
            w.label = label;
            w.disabled = disabled;
            w.rect = {x, fy, button_width(ctx, label), fh};
            w.nav = {2, 0, col};
            return w;
        };
        const float pad = ctx.px(t.space.xl);
        out.push_back(footer_button(BACK, "BACK", ButtonVariant::GHOST, geo_.footer.x + pad, false, 0));
        float fx = geo_.footer.right() - pad;
        const float gap = ctx.px(t.space.md);
        fx -= button_width(ctx, "APPLY");
        Widget apply = footer_button(APPLY, "APPLY", ButtonVariant::PRIMARY, fx, !dirty, 3);
        fx -= gap + button_width(ctx, "REVERT");
        Widget revert = footer_button(REVERT, "REVERT", ButtonVariant::SECONDARY, fx, !dirty, 2);
        fx -= gap + button_width(ctx, "RESET TO DEFAULTS");
        Widget defaults = footer_button(DEFAULTS, "RESET TO DEFAULTS", ButtonVariant::SECONDARY, fx, false, 1);
        out.push_back(std::move(defaults));
        out.push_back(std::move(revert));
        out.push_back(std::move(apply));
    }

    void paint_background(const LayoutContext& ctx, MenuServices& svc, DrawList& dl) override {
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;
        dl.fill(ctx.viewport, p.backdrop);
        paint_panel(dl, t, ctx.s, geo_.panel, t.elevation.panel);

        // Header
        const float x = geo_.panel.x + ctx.px(t.space.xl);
        dl.text_v_centered("SETTINGS", x, geo_.header.y + geo_.header.h * 0.40f, ctx.px(t.type.title), p.text_primary, ctx.px(1.0f));
        dl.text_v_centered("Audio and colour changes preview immediately. Apply saves them.", x,
                           geo_.header.y + geo_.header.h * 0.40f + ctx.px(t.space.xxl), ctx.px(t.type.caption), p.text_muted);
        if (svc.settings().is_dirty()) {
            const char* label = "UNSAVED CHANGES";
            const float size = ctx.px(t.type.label);
            const float pad = ctx.px(t.space.md);
            const float w = text_width(label, size, ctx.px(t.type.tracking_button)) + pad * 2.0f;
            const float h = ctx.px(28.0f);
            const Rect chip{geo_.panel.right() - ctx.px(t.space.xl) - w, geo_.header.y + geo_.header.h * 0.40f - h * 0.5f, w, h};
            dl.rect(chip, RectStyle::outlined(p.highlight.with_alpha(0.12f), p.highlight, ctx.px(t.border.hairline), h * 0.5f));
            dl.text_centered(label, chip.cx(), chip.cy(), size, p.highlight, ctx.px(t.type.tracking_button));
        }

        // Tab strip divider and animated selection indicator
        dl.line(geo_.panel.x, geo_.tabs.bottom(), geo_.panel.right(), geo_.tabs.bottom(), ctx.px(t.border.hairline), p.border_subtle);
        dl.fill({indicator_x_, geo_.tabs.bottom() - ctx.px(3.0f), indicator_w_, ctx.px(3.0f)}, p.accent, ctx.px(1.5f));

        // Footer strip
        dl.rect(geo_.footer, RectStyle{p.surface_header.faded(0.6f), p.surface_header, {0, 0, 0, 0}, 0.0f, 0.0f});
        dl.line(geo_.panel.x, geo_.footer.y, geo_.panel.right(), geo_.footer.y, ctx.px(t.border.hairline), p.border_subtle);

        // Controls legend under the panel
        const float legend_cy = geo_.panel.bottom() + (ctx.viewport.bottom() - geo_.panel.bottom()) * 0.5f;
        const std::initializer_list<LegendItem> legend = {
            {"ARROWS / D-PAD", "MOVE"}, {"LEFT / RIGHT", "ADJUST"}, {"ENTER / A", "SELECT"},
            {"Q / E  LB / RB", "SWITCH TAB"}, {"ESC / B", "BACK"}};
        const float lw = paint_legend(nullptr, t, ctx.s, 0.0f, 0.0f, legend);
        paint_legend(&dl, t, ctx.s, ctx.viewport.cx() - lw * 0.5f, legend_cy, legend);
    }

    void paint_scroll_content(const LayoutContext& ctx, MenuServices& svc, DrawList& dl) override {
        (void)svc;
        const Theme& t = ctx.theme;
        const Palette& p = t.palette;
        const auto& rows = rows_for(tab_);
        const float gutter = ctx.px(t.metrics.scrollbar_w + t.space.md);
        const float row_w = geo_.view.w - gutter;
        int keybind_index = 0;
        for (size_t i = 0; i < rows.size() && i < row_top_.size(); ++i) {
            const RowSpec& row = rows[i];
            const float top = geo_.view.y + row_top_[i] - ctx.scroll_y;
            const float h = row_h_[i];
            if (top > geo_.view.bottom() || top + h < geo_.view.y) {
                if (row.kind == RowKind::KEYBIND) ++keybind_index;
                continue;
            }
            const Rect rect{geo_.view.x, top, row_w, h};
            switch (row.kind) {
                case RowKind::SECTION: {
                    const float cy = rect.bottom() - ctx.px(t.space.lg);
                    dl.text_v_centered(row.text, rect.x + ctx.px(t.space.xl), cy, ctx.px(t.type.label), p.text_label,
                                       ctx.px(t.type.tracking_wide));
                    const float after = rect.x + ctx.px(t.space.xl) +
                                        text_width(row.text, ctx.px(t.type.label), ctx.px(t.type.tracking_wide)) + ctx.px(t.space.md);
                    if (row.columns) {
                        // Column headings above the two key-cap columns
                        for (int slot = 0; slot < 2; ++slot) {
                            const Rect cap = keycap_rect(ctx, rect, slot);
                            dl.text_centered(slot == 0 ? "PRIMARY" : "SECONDARY", cap.cx(), cy, ctx.px(t.type.caption * 0.9f),
                                             p.text_muted, ctx.px(t.type.tracking_wide * 0.6f));
                        }
                        const float first_cap_x = keycap_rect(ctx, rect, 0).x;
                        dl.line(after, cy, first_cap_x - ctx.px(t.space.lg), cy, ctx.px(t.border.hairline), p.border_subtle);
                    } else {
                        dl.line(after, cy, rect.right() - ctx.px(t.space.lg), cy, ctx.px(t.border.hairline), p.border_subtle);
                    }
                    break;
                }
                case RowKind::NOTE: {
                    const auto lines = wrap_text(row.text, rect.w - ctx.px(t.space.xl) * 2.0f, ctx.px(t.type.caption));
                    float by = rect.y + ctx.px(t.type.caption * 1.1f);
                    for (const std::string& line : lines) {
                        dl.text(line, rect.x + ctx.px(t.space.xl), by, ctx.px(t.type.caption), p.text_muted);
                        by += ctx.px(t.type.caption * 1.5f);
                    }
                    break;
                }
                case RowKind::KEYBIND: {
                    if (keybind_index++ % 2 == 1) {
                        dl.fill(rect.inset(0.0f, ctx.px(t.space.xxs)), p.surface_row_alt.faded(0.6f), ctx.px(t.radius.md));
                    }
                    dl.text_v_centered(row.text, rect.x + ctx.px(t.space.xl), rect.cy(), ctx.px(t.type.body * 0.85f),
                                       p.text_secondary, ctx.px(t.type.tracking_button * 0.5f));
                    break;
                }
                case RowKind::FIELD:
                    break; // painted by WidgetPainter
            }
        }
    }

    void paint_overlay(const LayoutContext& ctx, MenuServices&, DrawList& dl) override {
        // Edge fades hint that more content is scrollable.
        const Theme& t = ctx.theme;
        const float fade_h = ctx.px(t.space.xxl);
        const Rgba panel = t.palette.surface_panel;
        const float max_scroll = std::max(0.0f, content_h_ - geo_.view.h);
        if (ctx.scroll_y > 1.0f) {
            dl.gradient({geo_.view.x, geo_.view.y, geo_.view.w, fade_h}, panel, panel.with_alpha(0.0f));
        }
        if (ctx.scroll_y < max_scroll - 1.0f) {
            dl.gradient({geo_.view.x, geo_.view.bottom() - fade_h, geo_.view.w, fade_h}, panel.with_alpha(0.0f), panel);
        }
    }

    void on_activate(WidgetId id, MenuServices& svc) override {
        if (id >= kTabBase && id < kTabBase + kTabCount) {
            select_tab(static_cast<SettingsSection>(id - kTabBase), svc, true);
            return;
        }
        switch (id) {
            case BACK: on_back(svc); return;
            case DEFAULTS: {
                svc.settings().reset_to_defaults(tab_);
                svc.toast(std::string(kTabNames[static_cast<size_t>(tab_)]) + " defaults restored. Apply to save them.", ToastKind::INFO);
                return;
            }
            case REVERT:
                svc.settings().revert();
                svc.toast("Unsaved changes discarded.", ToastKind::INFO);
                return;
            case APPLY: apply(svc); return;
            default: break;
        }
        const auto [row, slot] = decode(id);
        const auto& rows = rows_for(tab_);
        if (row >= rows.size()) return;
        const RowSpec& spec = rows[row];
        if (spec.kind == RowKind::FIELD && spec.field.kind == WidgetKind::DROPDOWN) {
            const FieldSpec& f = spec.field;
            svc.open_dropdown(id, f.options(), f.get(svc.settings().pending()),
                              [&svc, &f](int index) { svc.settings().edit([&](UserSettings& s) { f.set(s, index); }); });
        } else if (spec.kind == RowKind::KEYBIND && slot >= 1) {
            begin_rebind(spec, slot - 1, svc);
        }
    }

    void on_adjust(WidgetId id, int direction, bool wrap, MenuServices& svc) override {
        const FieldSpec* f = field_for(id);
        if (!f || (f->enabled && !f->enabled(svc.settings().pending()))) return;
        svc.settings().edit([&](UserSettings& s) {
            const int v = f->get(s);
            const IntRange r = f->range_for(*this);
            int next = v;
            switch (f->kind) {
                case WidgetKind::TOGGLE: next = direction > 0 && !wrap ? 1 : (direction < 0 ? 0 : 1 - v); break;
                case WidgetKind::SLIDER: next = r.clamp(v + direction * r.step); break;
                default: {
                    next = v + direction;
                    if (wrap) next = (next > r.max) ? r.min : (next < r.min ? r.max : next);
                    next = r.clamp(next);
                    break;
                }
            }
            f->set(s, next);
        });
    }

    void on_slider(WidgetId id, float t01, MenuServices& svc) override {
        const FieldSpec* f = field_for(id);
        if (!f || f->kind != WidgetKind::SLIDER) return;
        const IntRange r = f->range;
        const float raw = static_cast<float>(r.min) + t01 * static_cast<float>(r.max - r.min);
        const int stepped = r.clamp(r.min + static_cast<int>(std::lround((raw - static_cast<float>(r.min)) / static_cast<float>(r.step))) * r.step);
        svc.settings().edit([&](UserSettings& s) { f->set(s, stepped); });
    }

    void on_clear(WidgetId id, MenuServices& svc) override {
        const auto [row, slot] = decode(id);
        const auto& rows = rows_for(tab_);
        if (row < rows.size() && rows[row].kind == RowKind::KEYBIND && slot >= 1) {
            const InputAction action = rows[row].action;
            svc.settings().edit([&](UserSettings& s) { s.controls.assign(action, slot - 1, scancode::NONE); });
        }
    }

    void on_tab(int direction, MenuServices& svc) override {
        const int next = (static_cast<int>(tab_) + direction + kTabCount) % kTabCount;
        const bool on_tabs = svc.focused() >= kTabBase && svc.focused() < kTabBase + kTabCount;
        select_tab(static_cast<SettingsSection>(next), svc, on_tabs);
    }

    void on_back(MenuServices& svc) override {
        if (!svc.settings().is_dirty()) {
            svc.navigate(ScreenId::MAIN, false);
            return;
        }
        ModalSpec m;
        m.title = "UNSAVED CHANGES";
        m.message = "Apply your changes before leaving settings? Discarding restores the last saved values.";
        m.buttons = {
            {"APPLY", ButtonVariant::PRIMARY, [this, &svc] { apply(svc); svc.navigate(ScreenId::MAIN, false); }},
            {"DISCARD", ButtonVariant::DANGER, [&svc] { svc.settings().revert(); svc.navigate(ScreenId::MAIN, false); }},
            {"KEEP EDITING", ButtonVariant::SECONDARY, nullptr},
        };
        m.cancel_index = 2;
        m.default_index = 0;
        svc.show_modal(std::move(m));
    }

    /// @brief Widget id of a field row by its label (for tests and tooling).
    [[nodiscard]] WidgetId find_field(SettingsSection section, std::string_view label) const {
        const auto& rows = rows_for(section);
        for (size_t i = 0; i < rows.size(); ++i) {
            if (rows[i].kind == RowKind::FIELD && rows[i].field.label == label) return row_widget_id(i, 0);
            if (rows[i].kind == RowKind::KEYBIND && rows[i].text == label) return row_widget_id(i, 1);
        }
        return kNoWidget;
    }

private:
    enum class RowKind : uint8_t { SECTION, NOTE, FIELD, KEYBIND };

    struct FieldSpec {
        WidgetKind kind = WidgetKind::TOGGLE;
        std::string label;
        std::string hint;
        std::function<int(const UserSettings&)> get;
        std::function<void(UserSettings&, int)> set;
        IntRange range{0, 1, 1};
        std::string unit;
        std::function<std::vector<std::string>()> options;          ///< SELECTOR / DROPDOWN labels
        std::function<bool(const UserSettings&)> enabled;           ///< Empty = always enabled
        std::function<std::string(const UserSettings&, MenuServices&)> dynamic_hint;

        /// Selector ranges follow the option count, which may be dynamic (resolutions).
        [[nodiscard]] IntRange range_for(const SettingsScreen&) const {
            if (kind == WidgetKind::SELECTOR || kind == WidgetKind::DROPDOWN) {
                const int n = options ? static_cast<int>(options().size()) : 1;
                return {0, std::max(0, n - 1), 1};
            }
            return range;
        }
    };

    struct RowSpec {
        RowKind kind = RowKind::FIELD;
        std::string text;   ///< Section title, note text or key-binding label
        FieldSpec field{};
        InputAction action = InputAction::COUNT;
        bool columns = false; ///< SECTION shows PRIMARY / SECONDARY headings
    };

    struct Geometry {
        Rect panel, header, tabs, view, footer;
    };

    static constexpr std::array<const char*, kTabCount> kTabLabels = {"GRAPHICS", "AUDIO", "CONTROLS", "ACCESSIBILITY"};
    static constexpr std::array<const char*, kTabCount> kTabNames = {"Graphics", "Audio", "Controls", "Accessibility"};

    std::array<std::vector<RowSpec>, kTabCount> tabs_;
    SettingsSection tab_ = SettingsSection::GRAPHICS;
    int tab_dir_ = 1;
    float tab_anim_ = 1.0f;
    float slide_px_ = 0.0f;
    float indicator_x_ = 0.0f, indicator_w_ = 0.0f;
    float indicator_target_x_ = 0.0f, indicator_target_w_ = 0.0f;
    bool indicator_init_ = false;
    Geometry geo_{};
    float content_h_ = 0.0f;
    std::vector<float> row_top_, row_h_;
    std::vector<Resolution> res_options_;

    [[nodiscard]] const std::vector<RowSpec>& rows_for(SettingsSection s) const { return tabs_[static_cast<size_t>(s)]; }
    [[nodiscard]] static WidgetId tab_id(SettingsSection s) noexcept { return kTabBase + static_cast<WidgetId>(s); }
    [[nodiscard]] static WidgetId row_widget_id(size_t row, int slot) noexcept {
        return kRowBase + static_cast<WidgetId>(row) * kSlotsPerRow + static_cast<WidgetId>(slot);
    }
    [[nodiscard]] static std::pair<size_t, int> decode(WidgetId id) noexcept {
        if (id < kRowBase) return {SIZE_MAX, -1};
        return {static_cast<size_t>((id - kRowBase) / kSlotsPerRow), static_cast<int>((id - kRowBase) % kSlotsPerRow)};
    }
    [[nodiscard]] WidgetId first_field_id() const {
        const auto& rows = rows_for(tab_);
        for (size_t i = 0; i < rows.size(); ++i) {
            if (rows[i].kind == RowKind::FIELD) return row_widget_id(i, 0);
            if (rows[i].kind == RowKind::KEYBIND) return row_widget_id(i, 1);
        }
        return tab_id(tab_);
    }
    [[nodiscard]] const FieldSpec* field_for(WidgetId id) const {
        const auto [row, slot] = decode(id);
        const auto& rows = rows_for(tab_);
        if (row >= rows.size() || slot != 0 || rows[row].kind != RowKind::FIELD) return nullptr;
        return &rows[row].field;
    }

    [[nodiscard]] static bool section_modified(SettingsSection s, const UserSettings& a, const UserSettings& b) noexcept {
        switch (s) {
            case SettingsSection::GRAPHICS: return a.graphics != b.graphics;
            case SettingsSection::AUDIO: return a.audio != b.audio;
            case SettingsSection::CONTROLS: return a.controls != b.controls;
            case SettingsSection::ACCESSIBILITY: return a.accessibility != b.accessibility;
            case SettingsSection::COUNT: break;
        }
        return false;
    }

    [[nodiscard]] static float button_width(const LayoutContext& ctx, const char* label) {
        const Theme& t = ctx.theme;
        const float text = text_width(label, ctx.px(t.type.body * 0.9f), ctx.px(t.type.tracking_button));
        return std::max(ctx.px(t.metrics.footer_button_min_w), text + ctx.px(t.space.xxl) * 1.5f);
    }

    [[nodiscard]] static Rect keycap_rect(const LayoutContext& ctx, const Rect& row, int slot) {
        const Theme& t = ctx.theme;
        const float w = ctx.px(t.metrics.keycap_w);
        const float h = ctx.px(t.metrics.keycap_h);
        const float right = row.right() - ctx.px(t.space.lg);
        const float x = right - w - (slot == 0 ? w + ctx.px(t.space.md) : 0.0f);
        return {x, row.cy() - h * 0.5f, w, h};
    }

    void compute_geometry(const LayoutContext& ctx) {
        const Theme& t = ctx.theme;
        const Metrics& m = t.metrics;
        const Rect vp = ctx.viewport;
        const float margin = ctx.px(m.panel_margin);
        const float w = std::min(vp.w - 2.0f * margin, ctx.px(m.panel_max_w));
        const float h = vp.h - 2.0f * margin;
        geo_.panel = {vp.cx() - w * 0.5f, vp.y + margin, w, h};
        geo_.header = {geo_.panel.x, geo_.panel.y, w, ctx.px(m.header_h)};
        geo_.tabs = {geo_.panel.x, geo_.header.bottom(), w, ctx.px(m.tab_h)};
        geo_.footer = {geo_.panel.x, geo_.panel.bottom() - ctx.px(m.footer_h), w, ctx.px(m.footer_h)};
        const float pad = ctx.px(t.space.lg);
        geo_.view = {geo_.panel.x + pad, geo_.tabs.bottom() + ctx.px(t.space.sm), w - 2.0f * pad,
                     geo_.footer.y - geo_.tabs.bottom() - ctx.px(t.space.sm) * 2.0f};
    }

    Widget field_widget(const LayoutContext& ctx, MenuServices& svc, const FieldSpec& f, size_t row, const Rect& rect,
                        const UserSettings& pend, const UserSettings& comm) const {
        const Theme& t = ctx.theme;
        Widget w;
        w.id = row_widget_id(row, 0);
        w.kind = f.kind;
        w.rect = rect;
        w.control = {rect.right() - ctx.px(t.space.lg) - ctx.px(t.metrics.control_w), rect.y, ctx.px(t.metrics.control_w), rect.h};
        w.nav = {1, static_cast<int>(row), 0};
        w.in_scroll = true;
        w.label = f.label;
        w.hint = f.dynamic_hint ? f.dynamic_hint(pend, svc) : f.hint;
        w.disabled = f.enabled && !f.enabled(pend);

        const int v = f.get(pend);
        const IntRange r = f.range_for(*this);
        w.modified = v != f.get(comm);
        switch (f.kind) {
            case WidgetKind::TOGGLE:
                w.checked = v != 0;
                w.value_text = w.checked ? "ON" : "OFF";
                break;
            case WidgetKind::SLIDER:
                w.value01 = r.max > r.min ? static_cast<float>(v - r.min) / static_cast<float>(r.max - r.min) : 0.0f;
                w.value_text = std::to_string(v) + f.unit;
                break;
            default: {
                const std::vector<std::string> opts = f.options();
                w.value_text = (v >= 0 && v < static_cast<int>(opts.size())) ? opts[static_cast<size_t>(v)] : std::string("-");
                w.can_decrement = v > r.min;
                w.can_increment = v < r.max;
                break;
            }
        }
        return w;
    }

    void refresh_resolutions(MenuServices& svc, const UserSettings& pend) {
        res_options_ = svc.resolutions();
        const Resolution cur = pend.graphics.resolution;
        if (std::find(res_options_.begin(), res_options_.end(), cur) == res_options_.end()) res_options_.push_back(cur);
        std::sort(res_options_.begin(), res_options_.end(), [](const Resolution& a, const Resolution& b) {
            return a.width * a.height != b.width * b.height ? a.width * a.height > b.width * b.height : a.width > b.width;
        });
    }

    void begin_rebind(const RowSpec& spec, int slot, MenuServices& svc) {
        const InputAction action = spec.action;
        const std::string label = spec.text;
        svc.begin_key_capture(
            "REBIND " + label + (slot == 0 ? " (PRIMARY)" : " (SECONDARY)"),
            [&svc, action, slot](int32_t sc, std::string& error) {
                if (is_reserved_scancode(sc)) {
                    error = svc.key_name(sc) + " is reserved for a fixed shortcut. Choose another key.";
                    return false;
                }
                std::optional<InputAction> displaced;
                svc.settings().edit([&](UserSettings& s) { displaced = s.controls.assign(action, slot, sc); });
                if (displaced) {
                    svc.toast(svc.key_name(sc) + " was moved from " + std::string(kActionLabels[static_cast<size_t>(*displaced)]) + ".",
                              ToastKind::INFO);
                }
                return true;
            },
            [&svc, action, slot] { svc.settings().edit([&](UserSettings& s) { s.controls.assign(action, slot, scancode::NONE); }); });
    }

    void apply(MenuServices& svc) {
        // Only the window's multisampling needs a new GL context.
        const bool restart = msaa_samples(svc.settings().pending().graphics.quality) != msaa_samples(svc.launch_quality());
        const SettingsStorage::SaveResult r = svc.settings().apply();
        const bool persistent = svc.settings().storage().is_persistent();
        if (!r.ok) {
            svc.toast("Applied for this session, but could not save: " + r.message, ToastKind::FAILURE);
        } else if (restart) {
            svc.toast(persistent ? "Settings saved. Cockpit edge smoothing changes after a restart."
                                 : "Settings applied. Cockpit edge smoothing changes after a restart.",
                      ToastKind::SUCCESS);
        } else {
            svc.toast(persistent ? "Settings saved." : "Settings applied.", ToastKind::SUCCESS);
        }
    }

    // -- Schema ---------------------------------------------------------------

public:
    static constexpr std::array<const char*, kInputActionCount> kActionLabels = {
        "NOSE DOWN", "NOSE UP", "ROLL LEFT", "ROLL RIGHT", "RUDDER LEFT", "RUDDER RIGHT",
        "THROTTLE UP", "THROTTLE DOWN", "WHEEL BRAKES / RECENTRE VIEW", "SPEEDBRAKE TOGGLE",
        "LANDING GEAR", "COCKPIT / CHASE VIEW", "MOUSE LOOK TOGGLE", "TRIM RESET", "RESET FLIGHT",
        "FIRE GUN", "DISPENSE CHAFF", "RADAR LOCK / UNLOCK", "RADAR RANGE UP", "RADAR RANGE DOWN",
        "RADAR ANTENNA UP", "RADAR ANTENNA DOWN", "LEFT MFD PAGE (FCR / FLCS)",
        "RADAR MODE (CRM / ACM)", "RADAR SCAN PATTERN", "FIRE MISSILE", "WEAPON SELECT (GUN / SRM / MRM)",
        "DISPENSE FLARES", "EJECT (HOLD)", "STOVL / CTOL MODE (F-35B)",
    };

private:
    template <typename T>
    using Accessor = T& (*)(UserSettings&);

    template <typename T>
    static T read(Accessor<T> acc, const UserSettings& s) {
        // The accessor only names a field; reading through it never mutates.
        return acc(const_cast<UserSettings&>(s));
    }

    static RowSpec section(std::string title, bool columns = false) {
        RowSpec r;
        r.kind = RowKind::SECTION;
        r.text = std::move(title);
        r.columns = columns;
        return r;
    }
    static RowSpec note(std::string text) {
        RowSpec r;
        r.kind = RowKind::NOTE;
        r.text = std::move(text);
        return r;
    }
    static RowSpec field(FieldSpec f) {
        RowSpec r;
        r.kind = RowKind::FIELD;
        r.field = std::move(f);
        return r;
    }

    static FieldSpec toggle(std::string label, std::string hint, Accessor<bool> acc) {
        FieldSpec f;
        f.kind = WidgetKind::TOGGLE;
        f.label = std::move(label);
        f.hint = std::move(hint);
        f.get = [acc](const UserSettings& s) { return read(acc, s) ? 1 : 0; };
        f.set = [acc](UserSettings& s, int v) { acc(s) = v != 0; };
        return f;
    }

    static FieldSpec slider(std::string label, std::string hint, Accessor<int> acc, IntRange range, std::string unit) {
        FieldSpec f;
        f.kind = WidgetKind::SLIDER;
        f.label = std::move(label);
        f.hint = std::move(hint);
        f.get = [acc](const UserSettings& s) { return read(acc, s); };
        f.set = [acc](UserSettings& s, int v) { acc(s) = v; };
        f.range = range;
        f.unit = std::move(unit);
        return f;
    }

    template <typename E>
    static FieldSpec selector(std::string label, std::string hint, Accessor<E> acc, std::vector<std::string> options) {
        FieldSpec f;
        f.kind = WidgetKind::SELECTOR;
        f.label = std::move(label);
        f.hint = std::move(hint);
        f.get = [acc](const UserSettings& s) { return static_cast<int>(read(acc, s)); };
        f.set = [acc](UserSettings& s, int v) { acc(s) = static_cast<E>(v); };
        f.options = [opts = std::move(options)] { return opts; };
        return f;
    }

    void build_schema() {
        using S = UserSettings;
        auto& gfx = tabs_[static_cast<size_t>(SettingsSection::GRAPHICS)];
        gfx.push_back(section("DISPLAY"));
        gfx.push_back(field(selector<DisplayMode>("DISPLAY MODE", "Borderless always uses the desktop resolution.",
                                                  +[](S& s) -> DisplayMode& { return s.graphics.display_mode; },
                                                  {"WINDOWED", "BORDERLESS", "FULLSCREEN"})));
        {
            FieldSpec f;
            f.kind = WidgetKind::DROPDOWN;
            f.label = "RESOLUTION";
            f.hint = "Window size, or the display mode used in fullscreen.";
            f.get = [this](const S& s) {
                const auto it = std::find(res_options_.begin(), res_options_.end(), s.graphics.resolution);
                return it == res_options_.end() ? 0 : static_cast<int>(it - res_options_.begin());
            };
            f.set = [this](S& s, int i) {
                if (i >= 0 && i < static_cast<int>(res_options_.size())) s.graphics.resolution = res_options_[static_cast<size_t>(i)];
            };
            f.options = [this] {
                std::vector<std::string> out;
                for (const Resolution& r : res_options_) out.push_back(std::to_string(r.width) + " x " + std::to_string(r.height));
                return out;
            };
            f.enabled = [](const S& s) { return s.graphics.display_mode != DisplayMode::BORDERLESS; };
            f.dynamic_hint = [](const S& s, MenuServices&) {
                return s.graphics.display_mode == DisplayMode::BORDERLESS ? std::string("Borderless matches the desktop resolution.")
                                                                          : std::string("Window size, or the display mode used in fullscreen.");
            };
            gfx.push_back(field(std::move(f)));
        }
        gfx.push_back(field(toggle("VERTICAL SYNC", "Match frames to the display refresh to prevent tearing.",
                                   +[](S& s) -> bool& { return s.graphics.vsync; })));
        gfx.push_back(field(selector<FrameRateCap>("FRAME RATE CAP", "Upper limit on frames rendered per second.",
                                                   +[](S& s) -> FrameRateCap& { return s.graphics.frame_rate_cap; },
                                                   {"30 FPS", "60 FPS", "120 FPS", "UNCAPPED"})));
        gfx.push_back(section("QUALITY & EFFECTS"));
        {
            FieldSpec f = selector<QualityPreset>("QUALITY PRESET", "",
                                                  +[](S& s) -> QualityPreset& { return s.graphics.quality; },
                                                  {"LOW", "MEDIUM", "HIGH", "ULTRA"});
            f.dynamic_hint = [](const S& s, MenuServices& svc) {
                const auto q = graphics::RenderQuality::from_preset(s.graphics.quality);
                std::string h = q.scene_samples > 0 ? std::to_string(q.scene_samples) + "x anti-aliasing"
                                                    : std::string("No anti-aliasing");
                h += q.shadow_map_size > 0 ? ", " + std::to_string(q.shadow_map_size) + " px shadows" : ", no shadows";
                h += q.bloom ? ", bloom" : "";
                h += q.cirrus ? ", high cloud." : ".";
                if (msaa_samples(s.graphics.quality) != msaa_samples(svc.launch_quality())) {
                    h += " Cockpit edge smoothing changes after a restart.";
                }
                return h;
            };
            gfx.push_back(field(std::move(f)));
        }
        gfx.push_back(field(toggle("SCREEN SHAKE", "G-induced head motion in the cockpit view (hotkey H).",
                                   +[](S& s) -> bool& { return s.graphics.screen_shake; })));

        auto& audio = tabs_[static_cast<size_t>(SettingsSection::AUDIO)];
        audio.push_back(section("VOLUME"));
        audio.push_back(field(slider("MASTER VOLUME", "Overall output level.", +[](S& s) -> int& { return s.audio.master; },
                                     limits::kVolume, "%")));
        audio.push_back(field(slider("ENGINE & AIRFLOW", "Engine spool, afterburner and wind noise.",
                                     +[](S& s) -> int& { return s.audio.engine; }, limits::kVolume, "%")));
        audio.push_back(field(slider("ALERTS & EFFECTS", "Aural warnings, gear transit and touchdown.",
                                     +[](S& s) -> int& { return s.audio.alerts; }, limits::kVolume, "%")));
        audio.push_back(field(toggle("MUTE ALL", "Silence every channel without changing the levels.",
                                     +[](S& s) -> bool& { return s.audio.muted; })));

        auto& ctl = tabs_[static_cast<size_t>(SettingsSection::CONTROLS)];
        ctl.push_back(section("FLIGHT CONTROLS"));
        ctl.push_back(field(slider("STICK SENSITIVITY", "Scales pitch and roll from the keyboard and joystick.",
                                   +[](S& s) -> int& { return s.controls.stick_sensitivity; }, limits::kStickSensitivity, "%")));
        ctl.push_back(field(slider("MOUSE LOOK SENSITIVITY", "Head movement speed when looking around the cockpit.",
                                   +[](S& s) -> int& { return s.controls.mouse_sensitivity; }, limits::kMouseSensitivity, "%")));
        ctl.push_back(field(toggle("INVERT PITCH AXIS", "Swap nose-up and nose-down on every pitch input.",
                                   +[](S& s) -> bool& { return s.controls.invert_pitch; })));
        ctl.push_back(field(toggle("INVERT MOUSE LOOK Y", "Move the mouse up to look down.",
                                   +[](S& s) -> bool& { return s.controls.invert_mouse_y; })));
        ctl.push_back(section("KEY BINDINGS", true));
        ctl.push_back(note("Select a key to rebind it, or press Delete to clear it. Esc, F1-F8, 1-5, H, I, J and M "
                           "are fixed shortcuts and cannot be bound."));
        for (size_t i = 0; i < kInputActionCount; ++i) {
            RowSpec r;
            r.kind = RowKind::KEYBIND;
            r.action = static_cast<InputAction>(i);
            r.text = kActionLabels[i];
            ctl.push_back(std::move(r));
        }

        auto& acc = tabs_[static_cast<size_t>(SettingsSection::ACCESSIBILITY)];
        acc.push_back(section("INTERFACE"));
        acc.push_back(field(slider("UI SCALE", "Size of menu text and controls. Changes when you apply.",
                                   +[](S& s) -> int& { return s.accessibility.ui_scale; }, limits::kUiScale, "%")));
        acc.push_back(field(toggle("HIGH CONTRAST", "Opaque panels, brighter borders and a yellow focus ring.",
                                   +[](S& s) -> bool& { return s.accessibility.high_contrast; })));
        acc.push_back(field(toggle("REDUCE MOTION", "Turns off menu transitions and background animation.",
                                   +[](S& s) -> bool& { return s.accessibility.reduce_motion; })));
        acc.push_back(section("COLOUR"));
        acc.push_back(field(selector<ColorblindMode>("COLOUR-BLIND FILTER",
                                                     "Recolours status indicators and corrects the cockpit view.",
                                                     +[](S& s) -> ColorblindMode& { return s.accessibility.colorblind; },
                                                     {"OFF", "PROTANOPIA", "DEUTERANOPIA", "TRITANOPIA"})));
    }
};

} // namespace fastjet::ui
