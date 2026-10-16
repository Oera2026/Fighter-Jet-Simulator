#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/ui/settings_manager.hpp"
#include "fastjet/ui/theme.hpp"
#include "fastjet/ui/widgets.hpp"
#include <functional>
#include <optional>
#include <string>
#include <vector>

/// @file
/// @brief Contracts between the menu router (MenuSystem) and screen controllers.
///
/// A Screen is the controller for one view: every frame it describes its
/// widgets from the settings model and paints its static decoration; it
/// reacts to activations by editing the model or calling MenuServices.
/// Adding a screen means writing one class, with no change to input or
/// focus handling.

namespace fastjet::ui {

enum class ScreenId : uint8_t {
    MAIN = 0,
    SETTINGS,
    CREDITS,
    EVADE_DIFFICULTY,
    EVADE_AIRCRAFT,
    DOGFIGHT_GEOMETRY,
    DOGFIGHT_LOADOUT,
    DOGFIGHT_SKILL,
    DOGFIGHT_AIRCRAFT,
    INTERCEPT_DIFFICULTY,
    INTERCEPT_AIRCRAFT,
    COUNT
};

/// @brief Requests the menu raises for the application to act on.
enum class MenuAction : uint8_t {
    START_FLIGHT,        ///< Boot: begin the sortie
    RESUME_FLIGHT,       ///< In flight: close the pause menu
    OPEN_MISSION_SELECT, ///< Hand over to the tactical aircraft/sortie menu
    START_DOGFIGHT,      ///< Begin a 1v1 guns engagement against an AI bandit
    START_EVADE,         ///< Begin evade mode with MenuSystem::evade_choice(): survive a bandit hunting you
    START_INTERCEPT,     ///< Begin intercept mode with MenuSystem::intercept_choice(): lock up and kill a bandit who runs
    QUIT,
};

struct AppInfo {
    std::string title = "FAST JET SIMULATOR";
    std::string version;
};

enum class ToastKind : uint8_t { INFO, SUCCESS, FAILURE };

struct ModalButton {
    std::string label;
    ButtonVariant variant = ButtonVariant::SECONDARY;
    std::function<void()> on_press; ///< May be empty (just closes)
};

struct ModalSpec {
    std::string title;
    std::string message;
    std::vector<ModalButton> buttons;
    int cancel_index = -1;  ///< Button triggered by BACK; -1 closes without action
    int default_index = 0;  ///< Initially focused button
};

/// @brief Scrollable viewport a screen may declare.
struct ScrollRegion {
    Rect view{};
    float content_h = 0.0f;
};

/// @brief Per-frame inputs to layout and painting.
struct LayoutContext {
    const Theme& theme;
    Rect viewport;
    float s = 1.0f;          ///< Reference-pixel to framebuffer-pixel scale
    float time = 0.0f;       ///< Seconds; frozen when reduced motion is on
    float scroll_y = 0.0f;   ///< Current scroll offset of this screen's region
    float pointer_nx = 0.0f; ///< Pointer position in [-1, 1] for parallax
    float pointer_ny = 0.0f;

    [[nodiscard]] float px(float v) const noexcept { return v * s; }
};

/// @brief Services the router offers to screens.
class MenuServices {
public:
    virtual ~MenuServices() = default;

    virtual SettingsManager& settings() = 0;
    virtual const Theme& theme() const = 0;
    virtual const AppInfo& app_info() const = 0;
    virtual bool in_flight() const = 0;
    virtual const std::string& status_text() const = 0;
    virtual const std::vector<Resolution>& resolutions() const = 0;
    virtual QualityPreset launch_quality() const = 0;
    virtual std::string key_name(int32_t scancode) const = 0;
    /// @brief The airframe the host is flying (default for aircraft pickers).
    virtual aircraft::AircraftType current_aircraft() const = 0;

    virtual void navigate(ScreenId screen, bool forward) = 0;
    virtual void emit(MenuAction action) = 0;
    virtual void show_modal(ModalSpec spec) = 0;
    virtual void toast(std::string text, ToastKind kind) = 0;
    virtual void open_dropdown(WidgetId anchor, std::vector<std::string> options, int current,
                               std::function<void(int)> on_pick) = 0;
    /// @param on_key Returns false (and sets `error`) to reject a key and keep listening.
    virtual void begin_key_capture(std::string title,
                                   std::function<bool(int32_t scancode, std::string& error)> on_key,
                                   std::function<void()> on_clear) = 0;
    virtual bool is_capturing(WidgetId id) const = 0;
    [[nodiscard]] virtual WidgetId focused() const = 0;
    virtual void set_focus(WidgetId id) = 0;
    virtual void reset_scroll() = 0;
};

class Screen {
public:
    virtual ~Screen() = default;

    /// @brief Called when the screen becomes current.
    virtual void on_enter(MenuServices&) {}
    virtual void update(float /*dt*/, MenuServices&) {}

    /// @brief Describes this frame's widgets (view-model snapshot of the model).
    virtual void layout(const LayoutContext& ctx, MenuServices& svc, std::vector<Widget>& out) = 0;
    /// @brief Scrollable region, valid after layout().
    [[nodiscard]] virtual std::optional<ScrollRegion> scroll_region() const { return std::nullopt; }
    [[nodiscard]] virtual WidgetId default_focus() const = 0;

    /// @brief Static decoration behind the widgets.
    virtual void paint_background(const LayoutContext&, MenuServices&, DrawList&) {}
    /// @brief Decoration inside the scroll region (already clipped and offset).
    virtual void paint_scroll_content(const LayoutContext&, MenuServices&, DrawList&) {}
    /// @brief Decoration drawn above the widgets.
    virtual void paint_overlay(const LayoutContext&, MenuServices&, DrawList&) {}
    /// @brief Opacity/offset for the scroll region (used for tab cross-fades).
    [[nodiscard]] virtual float content_opacity() const { return 1.0f; }
    [[nodiscard]] virtual float content_offset_x() const { return 0.0f; }

    virtual void on_activate(WidgetId, MenuServices&) {}
    /// @param wrap Cycle past the ends (ACCEPT) rather than clamping (LEFT/RIGHT).
    virtual void on_adjust(WidgetId, int /*direction*/, bool /*wrap*/, MenuServices&) {}
    virtual void on_slider(WidgetId, float /*t01*/, MenuServices&) {}
    virtual void on_clear(WidgetId, MenuServices&) {}
    virtual void on_tab(int /*direction*/, MenuServices&) {}
    /// @brief BACK pressed with no modal/popup open.
    virtual void on_back(MenuServices&) = 0;
};

} // namespace fastjet::ui
