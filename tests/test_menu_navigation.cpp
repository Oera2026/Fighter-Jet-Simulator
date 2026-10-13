#include "../include/fastjet/ui/menu_system.hpp"
#include <cassert>
#include <iostream>

using namespace fastjet::ui;

namespace {

constexpr float kW = 1920.0f;
constexpr float kH = 1080.0f;
constexpr float kFrame = 1.0f / 60.0f;

void frames(MenuSystem& m, int n = 30) {
    for (int i = 0; i < n; ++i) m.update(kFrame, kW, kH);
}

void press(MenuSystem& m, NavCommand c) {
    m.handle(InputEvent::navigate(c));
    frames(m, 2);
}

const Widget* find(const MenuSystem& m, WidgetId id) {
    for (const Widget& w : m.widgets()) {
        if (w.id == id) return &w;
    }
    return nullptr;
}

void click(MenuSystem& m, float x, float y) {
    m.handle(InputEvent::pointer(InputEvent::Type::POINTER_MOVE, x, y));
    m.handle(InputEvent::pointer(InputEvent::Type::POINTER_DOWN, x, y));
    m.handle(InputEvent::pointer(InputEvent::Type::POINTER_UP, x, y));
    frames(m, 2);
}

/// Moves focus down until `id` is focused (bounded, so a bug cannot hang the test).
void focus_down_to(MenuSystem& m, WidgetId id) {
    for (int i = 0; i < 64 && m.focused_id() != id; ++i) press(m, NavCommand::DOWN);
    assert(m.focused_id() == id);
}

void open_settings(MenuSystem& m) {
    m.open(ScreenId::MAIN, false);
    frames(m);
    focus_down_to(m, MainMenuScreen::SETTINGS);
    press(m, NavCommand::ACCEPT);
    frames(m); // let the transition finish
    assert(m.current_screen() == ScreenId::SETTINGS && !m.is_transitioning());
}

} // namespace

void test_main_menu_keyboard_flow() {
    std::cout << "[Test] Main menu: traversal, transitions, actions... " << std::flush;
    SettingsManager s;
    s.load();
    MenuSystem m(s, AppInfo{"FAST JET SIMULATOR", "1.0.0"});
    m.open(ScreenId::MAIN, false);
    frames(m);
    assert(m.focused_id() == MainMenuScreen::START);

    press(m, NavCommand::UP); // top of the list: stays put
    assert(m.focused_id() == MainMenuScreen::START);
    press(m, NavCommand::DOWN);
    assert(m.focused_id() == MainMenuScreen::MISSION);
    press(m, NavCommand::ACCEPT);
    assert(m.poll_action() == MenuAction::OPEN_MISSION_SELECT);

    // Credits and back again
    focus_down_to(m, MainMenuScreen::CREDITS);
    press(m, NavCommand::ACCEPT);
    assert(m.is_transitioning());
    m.handle(InputEvent::navigate(NavCommand::BACK)); // ignored mid-transition
    frames(m);
    assert(m.current_screen() == ScreenId::CREDITS);
    press(m, NavCommand::BACK);
    frames(m);
    assert(m.current_screen() == ScreenId::MAIN);
    assert(!m.poll_action());
    std::cout << "PASS\n";
}

void test_evade_setup_flow() {
    std::cout << "[Test] Evade: pick the threat, then the jet, then launch... " << std::flush;
    SettingsManager s;
    s.load();
    MenuSystem m(s, AppInfo{});
    m.set_current_aircraft(fastjet::aircraft::AircraftType::F15EX_EAGLE_II);
    m.open(ScreenId::MAIN, false);
    frames(m);
    focus_down_to(m, MainMenuScreen::EVADE);
    press(m, NavCommand::ACCEPT);
    frames(m);
    assert(m.current_screen() == ScreenId::EVADE_DIFFICULTY && !m.poll_action());
    // Opens on MEDIUM; the focused level is the choice.
    assert(m.focused_id() == EvadeSetupScreen::kOptionBase + 1);
    press(m, NavCommand::DOWN);
    assert(m.evade_choice().difficulty == 2);

    // BACK from the jet list returns to the threat list with the choice kept.
    press(m, NavCommand::ACCEPT);
    frames(m);
    assert(m.current_screen() == ScreenId::EVADE_AIRCRAFT);
    assert(m.evade_choice().jet == fastjet::aircraft::AircraftType::F15EX_EAGLE_II); // Opens on the jet you fly
    press(m, NavCommand::BACK);
    frames(m);
    assert(m.current_screen() == ScreenId::EVADE_DIFFICULTY && m.focused_id() == EvadeSetupScreen::kOptionBase + 2);
    press(m, NavCommand::DOWN); // EXPERT
    focus_down_to(m, EvadeSetupScreen::kGo); // NEXT from the footer uses the choice
    press(m, NavCommand::ACCEPT);
    frames(m);
    assert(m.current_screen() == ScreenId::EVADE_AIRCRAFT && m.evade_choice().difficulty == 3);

    press(m, NavCommand::DOWN); // Typhoon
    press(m, NavCommand::DOWN); // F-22A
    assert(m.evade_choice().jet == fastjet::aircraft::AircraftType::F22_RAPTOR);
    assert(!m.poll_action());
    press(m, NavCommand::ACCEPT);
    assert(m.poll_action() == MenuAction::START_EVADE);
    assert(m.evade_choice().difficulty == 3 && m.evade_choice().jet == fastjet::aircraft::AircraftType::F22_RAPTOR);

    // The F-35B is the last jet on the list.
    m.open(ScreenId::EVADE_AIRCRAFT, false);
    frames(m);
    for (int i = 0; i < 4; ++i) press(m, NavCommand::DOWN); // From the F-15EX you fly
    assert(m.evade_choice().jet == fastjet::aircraft::AircraftType::F35B_LIGHTNING_II);

    // BACK from the threat list leaves for the main menu without launching.
    m.open(ScreenId::EVADE_DIFFICULTY, false);
    frames(m);
    press(m, NavCommand::BACK);
    frames(m);
    assert(m.current_screen() == ScreenId::MAIN && !m.poll_action());
    std::cout << "PASS\n";
}

void test_quit_confirmation() {
    std::cout << "[Test] Main menu: quit asks first, defaults to Cancel... " << std::flush;
    SettingsManager s;
    s.load();
    MenuSystem m(s, AppInfo{});
    m.open(ScreenId::MAIN, false);
    frames(m);
    focus_down_to(m, MainMenuScreen::QUIT);
    press(m, NavCommand::ACCEPT);
    assert(m.has_modal());
    press(m, NavCommand::ACCEPT); // Cancel is focused
    assert(!m.has_modal() && !m.poll_action());

    press(m, NavCommand::ACCEPT);
    press(m, NavCommand::LEFT); // Move to QUIT
    press(m, NavCommand::ACCEPT);
    assert(m.poll_action() == MenuAction::QUIT);

    // As a pause menu, BACK resumes the flight.
    m.open(ScreenId::MAIN, true);
    frames(m);
    press(m, NavCommand::BACK);
    assert(m.poll_action() == MenuAction::RESUME_FLIGHT);
    std::cout << "PASS\n";
}

void test_settings_edit_apply_revert() {
    std::cout << "[Test] Settings: adjust, tab switch, apply, unsaved-changes guard... " << std::flush;
    SettingsManager s;
    s.load();
    MenuSystem m(s, AppInfo{});
    open_settings(m);
    SettingsScreen& screen = m.settings_screen();
    assert(screen.tab() == SettingsSection::GRAPHICS);

    // Focus starts on DISPLAY MODE; RIGHT cycles it.
    assert(m.focused_id() == screen.find_field(SettingsSection::GRAPHICS, "DISPLAY MODE"));
    press(m, NavCommand::RIGHT);
    assert(s.pending().graphics.display_mode == DisplayMode::BORDERLESS);
    assert(s.is_dirty());
    // Resolution is disabled in borderless mode, so DOWN skips it.
    press(m, NavCommand::DOWN);
    assert(m.focused_id() == screen.find_field(SettingsSection::GRAPHICS, "VERTICAL SYNC"));
    press(m, NavCommand::ACCEPT);
    assert(!s.pending().graphics.vsync);

    // Audio tab: the slider moves in configured steps and previews live.
    press(m, NavCommand::NEXT_TAB);
    frames(m);
    assert(screen.tab() == SettingsSection::AUDIO);
    assert(m.focused_id() == screen.find_field(SettingsSection::AUDIO, "MASTER VOLUME"));
    const int before = s.pending().audio.master;
    press(m, NavCommand::LEFT);
    assert(s.pending().audio.master == before - limits::kVolume.step);

    // Leaving with unsaved edits raises a guard; KEEP EDITING (BACK) stays.
    press(m, NavCommand::BACK);
    assert(m.has_modal());
    press(m, NavCommand::BACK);
    assert(!m.has_modal() && m.current_screen() == ScreenId::SETTINGS);

    // APPLY from the footer commits everything and disables itself. DOWN
    // enters the footer at the button nearest the current column.
    auto in_footer = [&] { return m.focused_id() >= SettingsScreen::BACK && m.focused_id() <= SettingsScreen::APPLY; };
    for (int i = 0; i < 16 && !in_footer(); ++i) press(m, NavCommand::DOWN);
    assert(in_footer());
    for (int i = 0; i < 4; ++i) press(m, NavCommand::LEFT);
    assert(m.focused_id() == SettingsScreen::BACK);
    for (int i = 0; i < 3; ++i) press(m, NavCommand::RIGHT);
    assert(m.focused_id() == SettingsScreen::APPLY);
    press(m, NavCommand::ACCEPT);
    assert(!s.is_dirty());
    assert(s.committed().graphics.display_mode == DisplayMode::BORDERLESS);
    assert(s.committed().audio.master == before - limits::kVolume.step);
    assert(m.toast_text().has_value());
    const Widget* apply = find(m, SettingsScreen::APPLY);
    assert(apply && apply->disabled);
    assert(m.focused_id() != SettingsScreen::APPLY); // focus moved off the disabled button

    // Revert discards a fresh edit.
    press(m, NavCommand::UP);
    s.edit([](UserSettings& u) { u.audio.muted = true; });
    frames(m, 2);
    const Widget* revert = find(m, SettingsScreen::REVERT);
    assert(revert && !revert->disabled);
    s.revert();
    assert(!s.pending().audio.muted);

    press(m, NavCommand::BACK);
    frames(m);
    assert(m.current_screen() == ScreenId::MAIN); // no guard when clean
    std::cout << "PASS\n";
}

void test_pointer_toggle_and_slider_drag() {
    std::cout << "[Test] Settings: pointer toggles and slider drag... " << std::flush;
    SettingsManager s;
    s.load();
    MenuSystem m(s, AppInfo{});
    open_settings(m);
    const SettingsScreen& screen = m.settings_screen();

    const Widget* vsync = find(m, screen.find_field(SettingsSection::GRAPHICS, "VERTICAL SYNC"));
    assert(vsync);
    const bool was = s.pending().graphics.vsync;
    click(m, vsync->rect.cx(), vsync->rect.cy());
    assert(s.pending().graphics.vsync != was);

    // Tab strip by pointer
    const Widget* audio_tab = find(m, SettingsScreen::kTabBase + static_cast<WidgetId>(SettingsSection::AUDIO));
    assert(audio_tab);
    click(m, audio_tab->rect.cx(), audio_tab->rect.cy());
    frames(m);
    assert(m.settings_screen().tab() == SettingsSection::AUDIO);

    // Drag the master slider to its right end, then past its left end.
    const Widget* master = find(m, m.settings_screen().find_field(SettingsSection::AUDIO, "MASTER VOLUME"));
    assert(master);
    const float y = master->control.cy();
    m.handle(InputEvent::pointer(InputEvent::Type::POINTER_DOWN, master->control.cx(), y));
    m.handle(InputEvent::pointer(InputEvent::Type::POINTER_MOVE, master->control.right() + 200.0f, y));
    assert(s.pending().audio.master == limits::kVolume.max);
    m.handle(InputEvent::pointer(InputEvent::Type::POINTER_MOVE, master->control.x - 200.0f, y));
    assert(s.pending().audio.master == limits::kVolume.min);
    m.handle(InputEvent::pointer(InputEvent::Type::POINTER_UP, master->control.x - 200.0f, y));
    std::cout << "PASS\n";
}

void test_dropdown() {
    std::cout << "[Test] Settings: resolution dropdown by keyboard... " << std::flush;
    SettingsManager s;
    s.load();
    MenuSystem m(s, AppInfo{});
    m.set_resolutions({{2560, 1440}, {1920, 1080}, {1280, 720}});
    open_settings(m);
    press(m, NavCommand::DOWN);
    assert(m.focused_id() == m.settings_screen().find_field(SettingsSection::GRAPHICS, "RESOLUTION"));
    press(m, NavCommand::ACCEPT);
    assert(m.has_dropdown());
    press(m, NavCommand::DOWN); // 1920x1080 -> 1280x720 (list is largest first)
    press(m, NavCommand::ACCEPT);
    assert(!m.has_dropdown());
    assert((s.pending().graphics.resolution == Resolution{1280, 720}));

    press(m, NavCommand::ACCEPT);
    press(m, NavCommand::BACK); // Esc closes without changing anything
    assert(!m.has_dropdown());
    assert((s.pending().graphics.resolution == Resolution{1280, 720}));
    std::cout << "PASS\n";
}

void test_key_rebinding() {
    std::cout << "[Test] Settings: key capture, reserved keys, clear... " << std::flush;
    SettingsManager s;
    s.load();
    MenuSystem m(s, AppInfo{});
    open_settings(m);
    press(m, NavCommand::NEXT_TAB);
    press(m, NavCommand::NEXT_TAB);
    frames(m);
    assert(m.settings_screen().tab() == SettingsSection::CONTROLS);

    const WidgetId nose_down = m.settings_screen().find_field(SettingsSection::CONTROLS, "NOSE DOWN");
    focus_down_to(m, nose_down);

    press(m, NavCommand::ACCEPT);
    assert(m.is_capturing_key());
    m.handle(InputEvent::key_capture(scancode::M)); // reserved: rejected, still listening
    frames(m, 2);
    assert(m.is_capturing_key());
    m.handle(InputEvent::key_capture(scancode::F12)); // F-35B direct select
    m.handle(InputEvent::key_capture(scancode::NUM_6)); // F-35B in the airframe menu
    frames(m, 2);
    assert(m.is_capturing_key());
    assert(s.pending().controls.binding(InputAction::STOVL_MODE).primary == scancode::S);
    assert(s.pending().controls.binding(InputAction::PITCH_DOWN).primary == scancode::UP);

    m.handle(InputEvent::key_capture(scancode::A)); // steals A from RUDDER LEFT
    frames(m, 2);
    assert(!m.is_capturing_key());
    assert(s.pending().controls.binding(InputAction::PITCH_DOWN).primary == scancode::A);
    assert(s.pending().controls.binding(InputAction::RUDDER_LEFT).primary == scancode::NONE);
    assert(m.toast_text() && m.toast_text()->find("moved") != std::string::npos);

    press(m, NavCommand::CLEAR);
    assert(s.pending().controls.binding(InputAction::PITCH_DOWN).primary == scancode::NONE);

    // BACK cancels a capture without changing the binding.
    press(m, NavCommand::RIGHT); // secondary slot
    press(m, NavCommand::ACCEPT);
    assert(m.is_capturing_key());
    press(m, NavCommand::BACK);
    assert(!m.is_capturing_key());
    assert(s.pending().controls.binding(InputAction::PITCH_DOWN).secondary == scancode::NONE);

    // Focusing a row below the fold scrolls it into view.
    assert(m.scroll_offset() == 0.0f);
    press(m, NavCommand::LEFT);
    focus_down_to(m, m.settings_screen().find_field(SettingsSection::CONTROLS, "RESET FLIGHT"));
    assert(m.scroll_offset() > 0.0f);
    std::cout << "PASS\n";
}

void test_reduce_motion_and_theme() {
    std::cout << "[Test] Accessibility: reduced motion and high contrast apply live... " << std::flush;
    SettingsManager s;
    s.load();
    MenuSystem m(s, AppInfo{});
    const Rgba normal_text = m.active_theme().palette.text_muted;
    s.edit([](UserSettings& u) {
        u.accessibility.reduce_motion = true;
        u.accessibility.high_contrast = true;
    });
    assert(m.active_theme().reduce_motion);
    assert(!(m.active_theme().palette.text_muted == normal_text));

    m.open(ScreenId::MAIN, false);
    m.update(kFrame, kW, kH);
    focus_down_to(m, MainMenuScreen::SETTINGS);
    m.handle(InputEvent::navigate(NavCommand::ACCEPT));
    assert(m.current_screen() == ScreenId::SETTINGS && !m.is_transitioning()); // instant

    // Rendering produces commands and never throws on an empty frame.
    DrawList dl;
    m.update(kFrame, kW, kH);
    m.render(dl);
    assert(!dl.empty());
    std::cout << "PASS\n";
}

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Menu System Navigation & Interaction Tests\n";
    std::cout << "=========================================================\n";
    test_main_menu_keyboard_flow();
    test_quit_confirmation();
    test_evade_setup_flow();
    test_settings_edit_apply_revert();
    test_pointer_toggle_and_slider_drag();
    test_dropdown();
    test_key_rebinding();
    test_reduce_motion_and_theme();
    std::cout << "All menu navigation tests passed.\n";
    return 0;
}
