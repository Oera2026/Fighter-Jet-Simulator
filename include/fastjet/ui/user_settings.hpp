#pragma once

#include "fastjet/ui/json.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

/// @file
/// @brief User preference model: plain data, defaults, validation and JSON mapping.
///
/// This is the "Model" of the menu MVC split. It has no knowledge of SDL,
/// OpenGL or the menu views, so it can be loaded before a window exists and
/// unit-tested headlessly. Display labels live in the view (settings schema);
/// only stable, lower-case serialisation tokens live here.

namespace fastjet::ui {

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

enum class DisplayMode : uint8_t { WINDOWED = 0, BORDERLESS, FULLSCREEN, COUNT };
enum class FrameRateCap : uint8_t { FPS_30 = 0, FPS_60, FPS_120, UNCAPPED, COUNT };
enum class QualityPreset : uint8_t { LOW = 0, MEDIUM, HIGH, ULTRA, COUNT };
enum class ColorblindMode : uint8_t { OFF = 0, PROTANOPIA, DEUTERANOPIA, TRITANOPIA, COUNT };

/// @brief The settings categories; also the order of the settings tabs.
enum class SettingsSection : uint8_t { GRAPHICS = 0, AUDIO, CONTROLS, ACCESSIBILITY, COUNT };

inline constexpr std::array<std::string_view, static_cast<size_t>(DisplayMode::COUNT)>
    kDisplayModeTokens = {"windowed", "borderless", "fullscreen"};
inline constexpr std::array<std::string_view, static_cast<size_t>(FrameRateCap::COUNT)>
    kFrameRateCapTokens = {"30", "60", "120", "uncapped"};
inline constexpr std::array<std::string_view, static_cast<size_t>(QualityPreset::COUNT)>
    kQualityTokens = {"low", "medium", "high", "ultra"};
inline constexpr std::array<std::string_view, static_cast<size_t>(ColorblindMode::COUNT)>
    kColorblindTokens = {"off", "protanopia", "deuteranopia", "tritanopia"};

/// @brief Frame rate limit in Hz; 0 means uncapped.
[[nodiscard]] constexpr int frame_rate_cap_hz(FrameRateCap cap) noexcept {
    constexpr std::array<int, static_cast<size_t>(FrameRateCap::COUNT)> kHz = {30, 60, 120, 0};
    return kHz[static_cast<size_t>(cap)];
}

/// @brief Window multisampling for a quality preset.
///
/// This covers what is drawn straight to the window: the cockpit, HUD and
/// menus. It is fixed when the GL context is created, so a change applies on
/// the next launch. The 3D world renders into its own target, whose
/// multisampling (graphics::RenderQuality) follows the preset immediately.
/// 4x is the ceiling: at 8x the full-screen post pass alone writes eight
/// samples per pixel, for edges the cockpit's 4x already resolves.
[[nodiscard]] constexpr int msaa_samples(QualityPreset q) noexcept {
    constexpr std::array<int, static_cast<size_t>(QualityPreset::COUNT)> kSamples = {0, 2, 4, 4};
    return kSamples[static_cast<size_t>(q)];
}

// ---------------------------------------------------------------------------
// Key bindings
// ---------------------------------------------------------------------------

/// @brief USB HID keyboard usage IDs.
///
/// SDL_Scancode values are defined as these HID usages, so the model can hold
/// physical key positions without including SDL. Positional codes keep WASD-
/// style layouts in place on AZERTY/QWERTZ keyboards.
namespace scancode {
inline constexpr int32_t NONE = 0;
inline constexpr int32_t A = 4, B = 5, C = 6, D = 7, E = 8, F = 9, G = 10, H = 11, I = 12, J = 13, K = 14, L = 15, M = 16, O = 18, P = 19, Q = 20, R = 21, S = 22, T = 23, V = 25, X = 27;
inline constexpr int32_t MINUS = 45, EQUALS = 46, LEFTBRACKET = 47, RIGHTBRACKET = 48;
inline constexpr int32_t NUM_1 = 30, NUM_6 = 35;
inline constexpr int32_t RETURN = 40, ESCAPE = 41, BACKSPACE = 42, TAB = 43, SPACE = 44;
inline constexpr int32_t F1 = 58, F8 = 65, F12 = 69;
inline constexpr int32_t PAGE_UP = 75, DELETE_KEY = 76, PAGE_DOWN = 78;
inline constexpr int32_t RIGHT = 79, LEFT = 80, DOWN = 81, UP = 82;
inline constexpr int32_t LCTRL = 224, LSHIFT = 225;
inline constexpr int32_t MAX_VALID = 511; ///< SDL_SCANCODE_COUNT upper bound
} // namespace scancode

/// @brief Rebindable flight actions (fixed hotkeys such as F1-F8 are not listed).
enum class InputAction : uint8_t {
    PITCH_DOWN = 0,
    PITCH_UP,
    ROLL_LEFT,
    ROLL_RIGHT,
    RUDDER_LEFT,
    RUDDER_RIGHT,
    THROTTLE_UP,
    THROTTLE_DOWN,
    WHEEL_BRAKES,
    SPEEDBRAKE,
    LANDING_GEAR,
    TOGGLE_VIEW,
    MOUSE_LOOK,
    TRIM_RESET,
    RESET_FLIGHT,
    FIRE_GUN,
    DISPENSE_CHAFF,
    RADAR_LOCK,
    RADAR_RANGE_UP,
    RADAR_RANGE_DOWN,
    ANTENNA_UP,
    ANTENNA_DOWN,
    MFD_PAGE,
    RADAR_MODE,
    RADAR_SCAN,
    FIRE_MISSILE,
    WEAPON_SELECT,
    DISPENSE_FLARE,
    EJECT,
    STOVL_MODE, ///< F-35B STOVL/CTOL conversion button (latched)
    COUNT
};
inline constexpr size_t kInputActionCount = static_cast<size_t>(InputAction::COUNT);

/// @brief Up to two physical keys per action (0 = unbound slot).
struct KeyBinding {
    int32_t primary = scancode::NONE;
    int32_t secondary = scancode::NONE;

    [[nodiscard]] constexpr bool matches(int32_t sc) const noexcept {
        return sc != scancode::NONE && (sc == primary || sc == secondary);
    }
    [[nodiscard]] constexpr int32_t slot(int index) const noexcept { return index == 0 ? primary : secondary; }
    constexpr void set_slot(int index, int32_t sc) noexcept { (index == 0 ? primary : secondary) = sc; }

    bool operator==(const KeyBinding&) const = default;
};

struct ActionInfo {
    std::string_view json_key;
    KeyBinding defaults;
};

inline constexpr std::array<ActionInfo, kInputActionCount> kActionInfo = {{
    {"pitch_down",    {scancode::UP, scancode::NONE}},
    {"pitch_up",      {scancode::DOWN, scancode::NONE}},
    {"roll_left",     {scancode::LEFT, scancode::NONE}},
    {"roll_right",    {scancode::RIGHT, scancode::NONE}},
    {"rudder_left",   {scancode::A, scancode::NONE}},
    {"rudder_right",  {scancode::D, scancode::NONE}},
    {"throttle_up",   {scancode::LSHIFT, scancode::PAGE_UP}},
    {"throttle_down", {scancode::LCTRL, scancode::PAGE_DOWN}},
    {"wheel_brakes",  {scancode::SPACE, scancode::NONE}},
    {"speedbrake",    {scancode::B, scancode::NONE}},
    {"landing_gear",  {scancode::G, scancode::NONE}},
    {"toggle_view",   {scancode::V, scancode::NONE}},
    {"mouse_look",    {scancode::TAB, scancode::NONE}},
    {"trim_reset",    {scancode::T, scancode::NONE}},
    {"reset_flight",  {scancode::R, scancode::NONE}},
    {"fire_gun",      {scancode::F, scancode::NONE}},
    {"dispense_chaff", {scancode::C, scancode::NONE}},
    {"radar_lock",       {scancode::L, scancode::NONE}},
    {"radar_range_up",   {scancode::RIGHTBRACKET, scancode::NONE}},
    {"radar_range_down", {scancode::LEFTBRACKET, scancode::NONE}},
    {"antenna_up",       {scancode::EQUALS, scancode::NONE}},
    {"antenna_down",     {scancode::MINUS, scancode::NONE}},
    {"mfd_page",         {scancode::P, scancode::NONE}},
    {"radar_mode",       {scancode::K, scancode::NONE}},
    {"radar_scan",       {scancode::O, scancode::NONE}},
    {"fire_missile",     {scancode::RETURN, scancode::NONE}},
    {"weapon_select",    {scancode::Q, scancode::NONE}},
    {"dispense_flare",   {scancode::X, scancode::NONE}},
    {"eject",            {scancode::E, scancode::NONE}},
    {"stovl_mode",       {scancode::S, scancode::NONE}},
}};

/// @brief Keys owned by fixed hotkeys (menu, camera, airframe, detents,
/// throttle hardware) that must not be captured by a rebind.
[[nodiscard]] constexpr bool is_reserved_scancode(int32_t sc) noexcept {
    return sc == scancode::ESCAPE ||
           (sc >= scancode::F1 && sc <= scancode::F8) || sc == scancode::F12 ||
           (sc >= scancode::NUM_1 && sc <= scancode::NUM_6) ||
           sc == scancode::H || sc == scancode::I || sc == scancode::J || sc == scancode::M;
}

// ---------------------------------------------------------------------------
// Settings sections
// ---------------------------------------------------------------------------

/// @brief Allowed ranges for numeric settings (single source of truth for
/// validation, sliders and tests).
struct IntRange {
    int min;
    int max;
    int step;
    [[nodiscard]] constexpr int clamp(int v) const noexcept { return std::clamp(v, min, max); }
};

namespace limits {
inline constexpr IntRange kVolume{0, 100, 5};
inline constexpr IntRange kStickSensitivity{25, 200, 5};
inline constexpr IntRange kMouseSensitivity{25, 300, 5};
inline constexpr IntRange kUiScale{75, 150, 5};
inline constexpr int kMinResolution = 640;
inline constexpr int kMaxResolution = 16384;
} // namespace limits

struct Resolution {
    int width = 1920;
    int height = 1080;
    bool operator==(const Resolution&) const = default;
};

struct GraphicsSettings {
    Resolution resolution{};
    DisplayMode display_mode = DisplayMode::WINDOWED;
    bool vsync = true;
    FrameRateCap frame_rate_cap = FrameRateCap::UNCAPPED;
    QualityPreset quality = QualityPreset::ULTRA; ///< Highest preset, the pre-settings default
    bool screen_shake = false; ///< G-induced head motion / buffet in the cockpit view
    bool operator==(const GraphicsSettings&) const = default;
};

struct AudioSettings {
    int master = 80;  ///< Percent
    int engine = 100; ///< Engine, afterburner and airflow loops, percent
    int alerts = 100; ///< Aural warnings and one-shot effects, percent
    bool muted = false;

    /// @brief Linear gain for a bus after master volume and mute.
    [[nodiscard]] constexpr float bus_gain(int bus_percent) const noexcept {
        return muted ? 0.0f : (static_cast<float>(master) / 100.0f) * (static_cast<float>(bus_percent) / 100.0f);
    }
    bool operator==(const AudioSettings&) const = default;
};

struct ControlSettings {
    int stick_sensitivity = 100; ///< Percent scaling of pitch/roll commands
    int mouse_sensitivity = 100; ///< Percent scaling of cockpit mouse-look
    bool invert_pitch = false;
    bool invert_mouse_y = false;
    std::array<KeyBinding, kInputActionCount> bindings = default_bindings();

    [[nodiscard]] static constexpr std::array<KeyBinding, kInputActionCount> default_bindings() noexcept {
        std::array<KeyBinding, kInputActionCount> b{};
        for (size_t i = 0; i < kInputActionCount; ++i) b[i] = kActionInfo[i].defaults;
        return b;
    }
    [[nodiscard]] const KeyBinding& binding(InputAction a) const noexcept { return bindings[static_cast<size_t>(a)]; }

    /// @brief Binds `sc` to one slot of `action`, removing it from wherever
    /// else it was bound so a key never drives two actions.
    /// @return The action that lost the key, if any.
    std::optional<InputAction> assign(InputAction action, int slot, int32_t sc) noexcept {
        std::optional<InputAction> displaced;
        if (sc != scancode::NONE) {
            for (size_t i = 0; i < kInputActionCount; ++i) {
                for (int s = 0; s < 2; ++s) {
                    const bool same_slot = (i == static_cast<size_t>(action) && s == slot);
                    if (!same_slot && bindings[i].slot(s) == sc) {
                        bindings[i].set_slot(s, scancode::NONE);
                        if (i != static_cast<size_t>(action)) displaced = static_cast<InputAction>(i);
                    }
                }
            }
        }
        bindings[static_cast<size_t>(action)].set_slot(slot, sc);
        return displaced;
    }
    bool operator==(const ControlSettings&) const = default;
};

struct AccessibilitySettings {
    int ui_scale = 100; ///< Percent
    ColorblindMode colorblind = ColorblindMode::OFF;
    bool high_contrast = false;
    bool reduce_motion = false; ///< Disables menu transitions and background drift
    bool operator==(const AccessibilitySettings&) const = default;
};

// ---------------------------------------------------------------------------
// Aggregate
// ---------------------------------------------------------------------------

struct UserSettings {
    /// Bumped when a field changes meaning; older files are migrated in from_json().
    static constexpr int kSchemaVersion = 1;

    GraphicsSettings graphics{};
    AudioSettings audio{};
    ControlSettings controls{};
    AccessibilitySettings accessibility{};

    bool operator==(const UserSettings&) const = default;

    /// @brief Clamps every field into its valid range. Idempotent.
    void sanitize() noexcept {
        graphics.resolution.width = std::clamp(graphics.resolution.width, limits::kMinResolution, limits::kMaxResolution);
        graphics.resolution.height = std::clamp(graphics.resolution.height, limits::kMinResolution / 2, limits::kMaxResolution);
        if (graphics.display_mode >= DisplayMode::COUNT) graphics.display_mode = DisplayMode::WINDOWED;
        if (graphics.frame_rate_cap >= FrameRateCap::COUNT) graphics.frame_rate_cap = FrameRateCap::UNCAPPED;
        if (graphics.quality >= QualityPreset::COUNT) graphics.quality = QualityPreset::ULTRA;

        audio.master = limits::kVolume.clamp(audio.master);
        audio.engine = limits::kVolume.clamp(audio.engine);
        audio.alerts = limits::kVolume.clamp(audio.alerts);

        controls.stick_sensitivity = limits::kStickSensitivity.clamp(controls.stick_sensitivity);
        controls.mouse_sensitivity = limits::kMouseSensitivity.clamp(controls.mouse_sensitivity);
        for (KeyBinding& b : controls.bindings) {
            for (int s = 0; s < 2; ++s) {
                const int32_t sc = b.slot(s);
                if (sc < scancode::NONE || sc > scancode::MAX_VALID || is_reserved_scancode(sc)) b.set_slot(s, scancode::NONE);
            }
            if (b.primary == b.secondary) b.secondary = scancode::NONE;
        }

        accessibility.ui_scale = limits::kUiScale.clamp(accessibility.ui_scale);
        if (accessibility.colorblind >= ColorblindMode::COUNT) accessibility.colorblind = ColorblindMode::OFF;
    }

    /// @brief Restores one category to its defaults, leaving the others untouched.
    void reset_section(SettingsSection section) noexcept {
        const UserSettings d{};
        switch (section) {
            case SettingsSection::GRAPHICS: graphics = d.graphics; break;
            case SettingsSection::AUDIO: audio = d.audio; break;
            case SettingsSection::CONTROLS: controls = d.controls; break;
            case SettingsSection::ACCESSIBILITY: accessibility = d.accessibility; break;
            case SettingsSection::COUNT: *this = d; break;
        }
    }

    [[nodiscard]] json::Value to_json() const {
        json::Value root;
        root.set("schema_version", kSchemaVersion);

        json::Value g;
        json::Value res;
        res.set("width", graphics.resolution.width);
        res.set("height", graphics.resolution.height);
        g.set("resolution", std::move(res));
        g.set("display_mode", token(kDisplayModeTokens, graphics.display_mode));
        g.set("vsync", graphics.vsync);
        g.set("frame_rate_cap", token(kFrameRateCapTokens, graphics.frame_rate_cap));
        g.set("quality", token(kQualityTokens, graphics.quality));
        g.set("screen_shake", graphics.screen_shake);
        root.set("graphics", std::move(g));

        json::Value a;
        a.set("master", audio.master);
        a.set("engine", audio.engine);
        a.set("alerts", audio.alerts);
        a.set("muted", audio.muted);
        root.set("audio", std::move(a));

        json::Value c;
        c.set("stick_sensitivity", controls.stick_sensitivity);
        c.set("mouse_sensitivity", controls.mouse_sensitivity);
        c.set("invert_pitch", controls.invert_pitch);
        c.set("invert_mouse_y", controls.invert_mouse_y);
        json::Value binds;
        for (size_t i = 0; i < kInputActionCount; ++i) {
            const KeyBinding& b = controls.bindings[i];
            binds.set(std::string(kActionInfo[i].json_key), json::Array{b.primary, b.secondary});
        }
        c.set("key_bindings", std::move(binds));
        root.set("controls", std::move(c));

        json::Value x;
        x.set("ui_scale", accessibility.ui_scale);
        x.set("colorblind", token(kColorblindTokens, accessibility.colorblind));
        x.set("high_contrast", accessibility.high_contrast);
        x.set("reduce_motion", accessibility.reduce_motion);
        root.set("accessibility", std::move(x));
        return root;
    }

    /// @brief Builds settings from a document. Missing or mistyped fields keep
    /// their defaults, so partial or older files still load.
    [[nodiscard]] static UserSettings from_json(const json::Value& root) {
        UserSettings s;
        if (const json::Value* g = root.find("graphics")) {
            if (const json::Value* res = g->find("resolution")) {
                s.graphics.resolution.width = int_field(*res, "width", s.graphics.resolution.width);
                s.graphics.resolution.height = int_field(*res, "height", s.graphics.resolution.height);
            }
            s.graphics.display_mode = enum_field(*g, "display_mode", kDisplayModeTokens, s.graphics.display_mode);
            s.graphics.vsync = bool_field(*g, "vsync", s.graphics.vsync);
            s.graphics.frame_rate_cap = enum_field(*g, "frame_rate_cap", kFrameRateCapTokens, s.graphics.frame_rate_cap);
            s.graphics.quality = enum_field(*g, "quality", kQualityTokens, s.graphics.quality);
            s.graphics.screen_shake = bool_field(*g, "screen_shake", s.graphics.screen_shake);
        }
        if (const json::Value* a = root.find("audio")) {
            s.audio.master = int_field(*a, "master", s.audio.master);
            s.audio.engine = int_field(*a, "engine", s.audio.engine);
            s.audio.alerts = int_field(*a, "alerts", s.audio.alerts);
            s.audio.muted = bool_field(*a, "muted", s.audio.muted);
        }
        if (const json::Value* c = root.find("controls")) {
            s.controls.stick_sensitivity = int_field(*c, "stick_sensitivity", s.controls.stick_sensitivity);
            s.controls.mouse_sensitivity = int_field(*c, "mouse_sensitivity", s.controls.mouse_sensitivity);
            s.controls.invert_pitch = bool_field(*c, "invert_pitch", s.controls.invert_pitch);
            s.controls.invert_mouse_y = bool_field(*c, "invert_mouse_y", s.controls.invert_mouse_y);
            if (const json::Value* binds = c->find("key_bindings")) {
                for (size_t i = 0; i < kInputActionCount; ++i) {
                    const json::Value* pair = binds->find(kActionInfo[i].json_key);
                    const json::Array* arr = pair ? pair->array() : nullptr;
                    if (!arr || arr->size() != 2) continue;
                    s.controls.bindings[i].primary = (*arr)[0].as_int(s.controls.bindings[i].primary);
                    s.controls.bindings[i].secondary = (*arr)[1].as_int(s.controls.bindings[i].secondary);
                }
            }
        }
        if (const json::Value* x = root.find("accessibility")) {
            s.accessibility.ui_scale = int_field(*x, "ui_scale", s.accessibility.ui_scale);
            s.accessibility.colorblind = enum_field(*x, "colorblind", kColorblindTokens, s.accessibility.colorblind);
            s.accessibility.high_contrast = bool_field(*x, "high_contrast", s.accessibility.high_contrast);
            s.accessibility.reduce_motion = bool_field(*x, "reduce_motion", s.accessibility.reduce_motion);
        }
        s.sanitize();
        return s;
    }

private:
    template <typename E, size_t N>
    [[nodiscard]] static std::string token(const std::array<std::string_view, N>& tokens, E e) {
        return std::string(tokens[static_cast<size_t>(e)]);
    }

    template <typename E, size_t N>
    [[nodiscard]] static E enum_field(const json::Value& obj, std::string_view key,
                                      const std::array<std::string_view, N>& tokens, E fallback) {
        const json::Value* v = obj.find(key);
        if (!v) return fallback;
        const std::string t = v->as_string("");
        for (size_t i = 0; i < N; ++i) {
            if (tokens[i] == t) return static_cast<E>(i);
        }
        return fallback;
    }

    [[nodiscard]] static int int_field(const json::Value& obj, std::string_view key, int fallback) {
        const json::Value* v = obj.find(key);
        return v ? v->as_int(fallback) : fallback;
    }

    [[nodiscard]] static bool bool_field(const json::Value& obj, std::string_view key, bool fallback) {
        const json::Value* v = obj.find(key);
        return v ? v->as_bool(fallback) : fallback;
    }
};

} // namespace fastjet::ui
