#include <SDL3/SDL.h>
#include <epoxy/gl.h>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <vector>

#include "fastjet/fdm/flight_state.hpp"
#include "fastjet/fdm/mass_properties.hpp"
#include "fastjet/fdm/rk4_integrator.hpp"
#include "fastjet/fdm/six_dof_fdm.hpp"
#include "fastjet/environment/atmosphere1976.hpp"
#include "fastjet/environment/wind_turbulence.hpp"
#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/aero/aircraft_aero_model.hpp"
#include "fastjet/flcs/aircraft_flcs.hpp"
#include "fastjet/flcs/onboard_flight_computer.hpp"
#include "fastjet/input/input_manager.hpp"
#include "fastjet/propulsion/multi_engine.hpp"
#include "fastjet/propulsion/stovl_status.hpp"
#include "fastjet/fdm/fuel_system.hpp"
#include "fastjet/gear/landing_gear.hpp"
#include "fastjet/environment/ground_collision.hpp"
#include "fastjet/sim/engagement.hpp"
#include "fastjet/sim/radar_scope.hpp"
#include "fastjet/sim/sweep_scope.hpp"
#include "fastjet/sim/world.hpp"

#define CGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#include "fastjet/graphics/render_engine.hpp"
#include "fastjet/graphics/frame_pacer.hpp"
#include "fastjet/graphics/cockpit_telemetry.hpp"
#include "fastjet/audio/audio_engine.hpp"
#include "fastjet/ui/menu_system.hpp"
#include "fastjet/ui/sdl_menu_bridge.hpp"
#include "fastjet/version.hpp"

using namespace fastjet;

namespace {

/// @brief --profile: frame-time distribution and CPU time per loop stage,
/// reported (and reset) every couple of seconds.
struct FrameStats {
    std::vector<double> frame_ms;
    double physics_ms = 0.0;
    double render_ms = 0.0;
    double present_ms = 0.0;
    double elapsed_s = 0.0;

    void add(double frame, double physics, double render, double present) {
        frame_ms.push_back(frame);
        physics_ms += physics;
        render_ms += render;
        present_ms += present;
        elapsed_s += frame / 1000.0;
    }

    [[nodiscard]] std::string report() {
        if (frame_ms.empty()) return {};
        const double n = static_cast<double>(frame_ms.size());
        std::vector<double> sorted = frame_ms;
        std::sort(sorted.begin(), sorted.end());
        auto pct = [&](double p) { return sorted[static_cast<size_t>(p * (n - 1.0))]; };
        char line[256];
        std::snprintf(line, sizeof(line),
                      "[PROFILE] %.1f fps | frame p50 %.2f p99 %.2f max %.2f ms | cpu physics %.2f  render %.2f"
                      "  present %.2f ms\n",
                      n / elapsed_s, pct(0.5), pct(0.99), sorted.back(), physics_ms / n, render_ms / n,
                      present_ms / n);
        *this = FrameStats{};
        return line;
    }
};

} // namespace

int main(int argc, char* argv[]) {
    std::cout << "=========================================================\n";
    std::cout << "  Fast Jet Flight Simulator - Realistic Multi-Aircraft Sim\n";
    std::cout << "=========================================================\n";
    std::cout << "Aircraft Available:\n";
    std::cout << "  [F3] F-16C Fighting Falcon (Single F110, Analog/Digital FBW)\n";
    std::cout << "  [F4] F-15EX Eagle II (Twin F110, Digital FBW, Mach 2.50)\n";
    std::cout << "  [F5] Eurofighter Typhoon (Twin EJ200, Canard Carefree FBW, Supercruise)\n";
    std::cout << "  [F6] F-22A Raptor (Twin F119 2D TVC, 5th-Gen FBW, Supercruise)\n";
    std::cout << "  [F7] A-10C Thunderbolt II (Twin TF34 Non-AB, Dual SAS, CAS)\n";
    std::cout << "  [F12] F-35B Lightning II (F135-PW-600 + LiftFan, STOVL, hover)\n";
    std::cout << "  [F8] Cycle Next Aircraft\n";
    std::cout << "Controls:\n";
    std::cout << "  Right Mouse Drag / Tab: Look around cockpit (scan MFDs, canopy, runway)\n";
    std::cout << "  Middle Mouse:           Recenter pilot view forward\n";
    std::cout << "  Arrow Keys / Stick:     Pitch and roll stick inceptor commands\n";
    std::cout << "  A / D:                  Rudder pedals (left / right)\n";
    std::cout << "  Shift / Ctrl:           Throttle increase / decrease (detents 1/2/3/4)\n";
    std::cout << "  B:                      Speedbrakes extend / retract\n";
    std::cout << "  Space:                  Wheel brakes (on ground) / Recenter view (air)\n";
    std::cout << "  G:                      Landing gear handle up / down\n";
    std::cout << "  F1 / F2 / V:            Cockpit View (F1) / Chase View (F2) / Toggle View (V)\n";
    std::cout << "  Mouse Wheel:            Zoom chase camera in/out (Chase mode)\n";
    std::cout << "  H:                      Toggle Cockpit G-Head Motion (Default: Fixed DEP like MSFS)\n";
    std::cout << "  T / R:                  Trim reset / Reset simulation\n";
    std::cout << "  E (hold 0.5 s):         Eject (zero-zero seat; the camera follows you down)\n";
    std::cout << "  S:                      F-35B STOVL/HOOK button: convert / reconvert (50% throttle detent holds speed)\n";
    std::cout << "  Esc / Gamepad Start:    Pause menu (settings, credits, quit)\n";
    std::cout << "  F:                      Fire gun (dogfight / evade)\n";
    std::cout << "  C:                      Dispense chaff (2 bundles per press)\n";
    std::cout << "  L:                      Radar lock / unlock (FCR page, left MFD)\n";
    std::cout << "  K:                      Radar mode: CRM (long-range search) / ACM (dogfight auto-lock)\n";
    std::cout << "  O:                      Radar scan: A6/A3/A1 azimuth (CRM), 20x30/10x60/BORE (ACM)\n";
    std::cout << "  [ / ]:                  Radar range scale down / up\n";
    std::cout << "  = / -:                  Radar antenna elevation up / down\n";
    std::cout << "  P:                      Left MFD page: FCR / FLCS\n";
    std::cout << "  F9 / F10:               New fight / cycle set-up (merge, perch, range, evade)\n";
    std::cout << "  F11:                    Cycle bandit skill (dogfight) or difficulty (evade)\n";
    std::cout << "  (Flight keys can be rebound in Settings > Controls.)\n";
    std::cout << "  --dogfight [merge|offensive|defensive|range] --skill [novice|veteran|ace] --bandit TYPE\n";
    std::cout << "  --evade [easy|medium|hard|expert]: survive a bandit with radar missiles in your six\n";
    std::cout << "  --canyon:               Start airborne at the canyon mouth (east of the field), lined up to fly it\n";
    std::cout << "  --hover:                F-35B converted, hovering 50 ft over the runway at vertical-landing weight\n";
    std::cout << "  --stovl-approach:       F-35B converted, 120 kt and 1,000 ft, 3 nm short of the runway\n";
    std::cout << "  --watch:                Demo: the AI flies your jet as well\n";
    std::cout << "  --profile:              Print frame timing and per-pass GPU cost every 2 s\n";
    std::cout << "---------------------------------------------------------\n";

    bool headless = false;
    int max_frames = -1;
    int win_w = 1920;
    int win_h = 1080;
    bool fullscreen = false;
    aircraft::AircraftType selected_aircraft = aircraft::AircraftType::F16_FIGHTING_FALCON;
    std::string screenshot_path;
    bool no_menu = false;
    bool start_chase = false;
    bool start_airborne = false;
    bool start_canyon = false;       // --canyon: airborne at the canyon's west mouth, lined up to fly it
    bool start_hover = false;        // --hover: F-35B converted, hovering over the runway
    bool start_stovl_approach = false; // --stovl-approach: F-35B converted, slow and low on finals
    double auto_eject_s = -1.0;      // --eject-after S: tooling, pulls the handle after S seconds of flight
    float look_yaw_deg = 0.0f;       // --look YAW,PITCH: tooling, initial head (or chase orbit) direction [deg]
    float look_pitch_deg = 0.0f;
    bool profile = false;            // --profile: frame timing report
    bool cli_size = false;           // --width/--height override the saved resolution for this run
    std::string settings_arg;        // --settings PATH
    bool start_dogfight = false;     // --dogfight: straight into a 1v1
    sim::EngagementSetup dogfight_setup{};
    bool bandit_type_set = false;
    ui::ScreenId boot_screen = ui::ScreenId::MAIN;
    std::optional<ui::SettingsSection> boot_tab;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--headless") headless = true;
        if (arg == "--frames" && i + 1 < argc) max_frames = std::atoi(argv[++i]);
        if (arg == "--width" && i + 1 < argc) { win_w = std::atoi(argv[++i]); cli_size = true; }
        if (arg == "--height" && i + 1 < argc) { win_h = std::atoi(argv[++i]); cli_size = true; }
        if (arg == "--settings" && i + 1 < argc) settings_arg = argv[++i];
        // Tooling: open the menu on a given screen / settings tab (screenshots).
        if (arg == "--ui-screen" && i + 1 < argc) {
            const std::string v = argv[++i];
            if (v == "settings") boot_screen = ui::ScreenId::SETTINGS;
            else if (v == "credits") boot_screen = ui::ScreenId::CREDITS;
            else if (v == "evade") boot_screen = ui::ScreenId::EVADE_DIFFICULTY;
            else if (v == "evade-jet") boot_screen = ui::ScreenId::EVADE_AIRCRAFT;
        }
        if (arg == "--ui-tab" && i + 1 < argc) {
            const std::string v = argv[++i];
            if (v == "graphics") boot_tab = ui::SettingsSection::GRAPHICS;
            else if (v == "audio") boot_tab = ui::SettingsSection::AUDIO;
            else if (v == "controls") boot_tab = ui::SettingsSection::CONTROLS;
            else if (v == "accessibility") boot_tab = ui::SettingsSection::ACCESSIBILITY;
        }
        if (arg == "--fullscreen") fullscreen = true;
        if (arg == "--screenshot" && i + 1 < argc) screenshot_path = argv[++i];
        if (arg == "--no-menu") no_menu = true;
        if (arg == "--chase") start_chase = true;
        if (arg == "--airborne") start_airborne = true;
        if (arg == "--canyon") { start_airborne = true; start_canyon = true; }
        if (arg == "--hover") start_hover = true;
        if (arg == "--stovl-approach") start_stovl_approach = true;
        if (arg == "--eject-after" && i + 1 < argc) auto_eject_s = std::atof(argv[++i]);
        if (arg == "--look" && i + 1 < argc) {
            const std::string v = argv[++i];
            look_yaw_deg = std::strtof(v.c_str(), nullptr);
            if (const auto comma = v.find(','); comma != std::string::npos) look_pitch_deg = std::strtof(v.c_str() + comma + 1, nullptr);
        }
        if (arg == "--profile") profile = true;
        if (arg == "--aircraft" && i + 1 < argc) {
            selected_aircraft = aircraft::parse_aircraft_type(argv[++i]);
        }
        if (arg == "--dogfight") {
            start_dogfight = true;
            no_menu = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                const std::string g = argv[++i];
                if (g == "offensive") dogfight_setup.geometry = sim::StartGeometry::OFFENSIVE_PERCH;
                else if (g == "defensive") dogfight_setup.geometry = sim::StartGeometry::DEFENSIVE_PERCH;
                else if (g == "range") dogfight_setup.geometry = sim::StartGeometry::GUNNERY_RANGE;
                else dogfight_setup.geometry = sim::StartGeometry::HEAD_ON_MERGE;
            }
        }
        if (arg == "--evade") {
            start_dogfight = true;
            no_menu = true;
            dogfight_setup.mode = sim::EngagementMode::EVADE;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                const std::string d = argv[++i];
                dogfight_setup.evade_difficulty = d == "easy"     ? sim::EvadeDifficulty::EASY
                                                : d == "hard"     ? sim::EvadeDifficulty::HARD
                                                : d == "expert"   ? sim::EvadeDifficulty::EXPERT
                                                                  : sim::EvadeDifficulty::MEDIUM;
            }
        }
        if (arg == "--watch") dogfight_setup.own_is_ai = true; // AI flies your jet too (demo)
        if (arg == "--skill" && i + 1 < argc) {
            const std::string k = argv[++i];
            dogfight_setup.skill = k == "novice" ? sim::AiSkill::NOVICE
                                 : k == "ace"    ? sim::AiSkill::ACE
                                                 : sim::AiSkill::VETERAN;
        }
        if (arg == "--bandit" && i + 1 < argc) {
            dogfight_setup.bandit_type = aircraft::parse_aircraft_type(argv[++i]);
            bandit_type_set = true;
        }
    }

    // Only the F-35B can start on its lift system.
    if ((start_hover || start_stovl_approach) && selected_aircraft != aircraft::AircraftType::F35B_LIGHTNING_II) {
        std::cout << "[STOVL] " << (start_hover ? "--hover" : "--stovl-approach") << " needs the F-35B: selecting it instead of the "
                  << aircraft::to_string(selected_aircraft) << ".\n";
        selected_aircraft = aircraft::AircraftType::F35B_LIGHTNING_II;
    }

    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK | SDL_INIT_GAMEPAD | SDL_INIT_AUDIO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    // User preferences. Loaded before the window exists because resolution,
    // display mode and MSAA are fixed at creation. Headless runs keep
    // settings in memory so automated captures are reproducible and never
    // touch the user's file, unless --settings names one explicitly.
    std::filesystem::path settings_path;
    if (!settings_arg.empty()) {
        settings_path = settings_arg;
    } else if (!headless) {
        if (char* pref = SDL_GetPrefPath("FastJet", "FastJetSimulator")) {
            settings_path = std::filesystem::path(pref) / "settings.json";
            SDL_free(pref);
        }
    }
    ui::SettingsManager settings{ui::SettingsStorage(settings_path)};
    {
        const auto loaded = settings.load();
        if (!loaded.message.empty()) std::cout << "[SETTINGS] " << loaded.message << "\n";
    }
    const ui::GraphicsSettings boot_gfx = settings.committed().graphics;
    if (!cli_size) {
        win_w = boot_gfx.resolution.width;
        win_h = boot_gfx.resolution.height;
    }
    if (boot_gfx.display_mode != ui::DisplayMode::WINDOWED && !headless) fullscreen = true;

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    // Window multisampling (cockpit, HUD, menus) from the quality preset.
    const int msaa = ui::msaa_samples(boot_gfx.quality);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, msaa > 0 ? 1 : 0);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, msaa);

    Uint32 window_flags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY;
    if (headless) window_flags |= SDL_WINDOW_HIDDEN;
    if (fullscreen) window_flags |= SDL_WINDOW_FULLSCREEN;

    SDL_Window* window = SDL_CreateWindow("F-16 Flight Sim - Cockpit View", win_w, win_h, window_flags);
    if (!window) {
        std::cerr << "Failed to create SDL window: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    SDL_GLContext gl_ctx = SDL_GL_CreateContext(window);
    if (!gl_ctx) {
        std::cerr << "Failed to create GL context: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

#if defined(_WIN32) || !defined(__has_include) || !__has_include(<epoxy/gl.h>)
    FastJetGLFunctions::instance().load();
#endif

    if (headless) {
        SDL_GL_SetSwapInterval(0); // Never block an offscreen capture on the display
    } else {
        ui::sdl::apply_video(window, boot_gfx, /*resize_window=*/!cli_size);
    }

    // Query actual physical pixel dimensions (for High-DPI / Retina / fractional scale displays)
    int pixel_w = win_w;
    int pixel_h = win_h;
    SDL_GetWindowSizeInPixels(window, &pixel_w, &pixel_h);

    // 1. Initialize Graphics Engine
    graphics::RenderEngine renderer;
    if (!renderer.init(pixel_w, pixel_h, 60.0f)) {
        std::cerr << "Failed to initialize graphics render engine!\n";
        SDL_GL_DestroyContext(gl_ctx);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // 2. Physics lives in sim::World (see the flight state set-up below).

    auto joy_driver = std::make_shared<input::SDL3InputDriver>();
    if (joy_driver->initialize() && joy_driver->get_device_count() > 0) {
        std::cout << "[HOTAS] Controller detected: " << joy_driver->get_device_name(0) << "\n";
    } else {
        std::cout << "[HOTAS] No physical joystick detected. Keyboard control active.\n";
    }
    input::InputManager input_mgr(joy_driver);

    // 3. Audio & Environmental Simulation Subsystems
    audio::AudioEngine audio_engine;
    if (!headless) {
        if (audio_engine.init()) {
            std::cout << "[AUDIO] Procedural cockpit sound engine online (SDL3).\n";
        }
    }

    // 4. Menu system and settings observers. The menu edits the settings
    //    model only; these observers push changes out to each subsystem.
    ui::MenuSystem main_menu(settings, ui::AppInfo{"FAST JET SIMULATOR", kVersionString});
    main_menu.set_key_name_provider(ui::sdl::key_name);
    main_menu.set_resolutions(ui::sdl::query_resolutions(window));
    main_menu.set_launch_quality(boot_gfx.quality);
    main_menu.set_current_aircraft(selected_aircraft);
    ui::sdl::MenuInputTranslator menu_input;
    ui::sdl::FrameLimiter frame_limiter;
    ui::DrawList ui_draw_list;

    // Preview-safe settings follow the pending copy, so sliders and colour
    // options respond while the user is still editing.
    auto apply_live_settings = [&](const ui::UserSettings& s) {
        audio_engine.set_bus_gains(s.audio.bus_gain(s.audio.engine), s.audio.bus_gain(s.audio.alerts));
        renderer.set_color_filter(s.accessibility.colorblind);
        renderer.apply_theme(ui::build_theme(s.accessibility));
    };
    ui::GraphicsSettings applied_gfx = boot_gfx;
    auto apply_committed_settings = [&](const ui::UserSettings& s) {
        if (!headless && ui::sdl::video_differs(s.graphics, applied_gfx)) {
            ui::sdl::apply_video(window, s.graphics);
        }
        applied_gfx = s.graphics;
        renderer.camera_rig().set_g_head_motion(s.graphics.screen_shake);
        // Scene quality (world MSAA, shadows, bloom, clouds) applies live.
        renderer.set_quality(graphics::RenderQuality::from_preset(s.graphics.quality));
    };
    settings.subscribe([&](ui::SettingsEvent event, const ui::SettingsManager& m) {
        apply_live_settings(m.pending());
        if (event == ui::SettingsEvent::APPLIED) apply_committed_settings(m.committed());
    });
    apply_live_settings(settings.committed());
    apply_committed_settings(settings.committed());

    // Pilot head tracking and mouse-look state
    bool right_mouse_down = false;
    bool mouse_look_toggle = false;

    // Compute static stance altitude for the selected airframe resting on ground
    auto compute_ground_stance_z = [](aircraft::AircraftType type) noexcept -> double {
        const auto cfg = aircraft::AircraftConfig::get(type);
        const double uncompressed_h = cfg.gear.main_l_pos_b.z + cfg.gear.rest_length;
        const double static_weight = (cfg.mass.empty_mass_kg + cfg.mass.internal_fuel_capacity_kg
                                      + cfg.mass.combat_payload_kg) * 9.80665;
        const double main_load_per_strut = (static_weight * 0.85) * 0.5;
        const double static_compression = main_load_per_strut / cfg.gear.main_spring_k;
        const double stance_h = uncompressed_h - std::clamp(static_compression, 0.05, cfg.gear.max_stroke * 0.8);
        return -stance_h; // In NED, down is positive, so altitude above ground is negative Z
    };

    // The STOVL starts fly the F-35B on its lift system. Any other jet
    // selected later falls back to the runway line-up.
    auto stovl_start = [&]() noexcept {
        return (start_hover || start_stovl_approach) && selected_aircraft == aircraft::AircraftType::F35B_LIGHTNING_II;
    };

    // Initial state: Runway 09 threshold lineup, gear resting on asphalt, ready for takeoff roll
    auto reset_flight_state = [&]() -> fdm::FlightState {
        fdm::FlightState s{};
        if (stovl_start()) {
            s.omega_b = math::Vector3::zero();
            s.q_att   = math::Quaternion::identity(); // Runway heading
            if (start_hover) {
                // Wheels 50 ft above the runway, 300 m in from the threshold.
                constexpr double kHoverWheelsAgl = 15.24;
                const double x = 450.0;
                s.pos_ned = math::Vector3(x, 0.0, environment::GroundCollision::get_ground_z(x, 0.0)
                                                      + compute_ground_stance_z(selected_aircraft) - kHoverWheelsAgl);
                s.vel_b   = math::Vector3::zero();
                s.q_att   = math::Quaternion::from_euler(0.0, 3.0 * M_PI / 180.0, 0.0); // Hover trim attitude
            } else {
                // Finals: 3 nm short of the threshold, 1,000 ft, 120 kt.
                const double x = 150.0 - 3.0 * 1852.0;
                s.pos_ned = math::Vector3(x, 0.0, environment::GroundCollision::get_ground_z(x, 0.0) - 1000.0 * 0.3048);
                s.vel_b   = math::Vector3(120.0 * 0.514444, 0.0, 0.0);
            }
            return s;
        }
        if (start_canyon) {
            // Two kilometres short of the west mouth, on the river's line,
            // 250 m above the water at 230 m/s (450 kt).
            using Canyon = graphics::TerrainField::Canyon;
            const float s0 = -2000.0f;
            float x = 0.0f, y = 0.0f;
            Canyon::from_axis(s0, Canyon::centre(s0), x, y);
            const double heading = std::atan2(Canyon::AXIS_Y, Canyon::AXIS_X);
            s.pos_ned = math::Vector3(x, y, -(Canyon::floor_elevation(s0) + 250.0));
            s.vel_b   = math::Vector3(230.0, 0.0, 0.0);
            s.omega_b = math::Vector3::zero();
            s.q_att   = math::Quaternion::from_euler(0.0, 0.0, heading);
            return s;
        }
        if (start_airborne) {
            s.pos_ned = math::Vector3(0.0, 0.0, -1500.0); // 1,500 m altitude
            s.vel_b   = math::Vector3(220.0, 0.0, 0.0);
            s.omega_b = math::Vector3::zero();
            s.q_att   = math::Quaternion::identity();
            return s;
        }
        const double stance_z = compute_ground_stance_z(selected_aircraft);
        s.pos_ned = math::Vector3(150.0, 0.0, stance_z); // Runway 09 lineup
        s.vel_b   = math::Vector3(0.0, 0.0, 0.0);         // Stationary on runway
        s.omega_b = math::Vector3::zero();
        s.q_att   = math::Quaternion::identity();        // Heading 000 deg down the runway
        return s;
    };

    // How long the HUD holds the airframe change confirmation banner.
    constexpr double AIRCRAFT_SWITCH_BANNER_SEC = 2.5;
    double aircraft_switch_timer = 0.0;

    // Every jet lives in a sim::World and flies the same per-aircraft
    // pipeline. Free flight is a world of one; a dogfight is an Engagement
    // (a world with an AI bandit, weapons and rules). The ownship is always
    // aircraft[0], reached through `own`.
    // Lever position for a new flight: ground idle on the runway, cruise
    // power in the air, the hold detent on the lift system.
    auto initial_throttle = [&]() noexcept {
        return stovl_start() ? propulsion::kStovlThrottleDetent : start_airborne ? 0.65 : 0.08;
    };
    double throttle = initial_throttle();
    auto free_world = std::make_unique<sim::World>();
    auto dogfight = std::make_unique<sim::Engagement>();
    bool dogfight_active = false;
    int player_selected_weapon = 0; // 0: Gun, 1: AIM-9X, 2: AIM-120C
    bool gear_down = true;
    free_world->clear();
    free_world->add(selected_aircraft, reset_flight_state());
    sim::Aircraft* own = &free_world->aircraft[0];
    own->landing_gear.deployed = gear_down;

    // F-35B STOVL/HOOK button, latched as the pilot last pressed it. A STOVL
    // start comes up converted at vertical-landing weight: 17,200 kg (37,900
    // lb), under the ~38,500 lb maximum VL weight and the 40,650 lbf of hover
    // thrust (T/W ~1.07). With the air-to-air load that is about 3,300 lb of
    // fuel; at full internal fuel the jet is too heavy to hover.
    constexpr double STOVL_START_GROSS_KG = 17200.0;
    bool stovl_selected = false;
    auto apply_stovl_start = [&]() {
        stovl_selected = stovl_start();
        if (!stovl_selected) return;
        const auto& m = aircraft::AircraftConfig::get(own->type).mass;
        own->engine.fuel_kg = std::clamp(STOVL_START_GROSS_KG - m.empty_mass_kg - m.combat_payload_kg, 0.0,
                                         m.internal_fuel_capacity_kg);
        own->refresh_mass();
        own->start_converted_stovl();
    };
    apply_stovl_start();

    // The pilot's fire-control radar (FCR page on the left MFD) follows
    // whichever world the ownship is in.
    sim::RadarScope radar_scope;
    sim::SweepScope sweep_scope;        // Right MFD in a fight: 360 deg sweep display
    double sweep_world_time = 0.0;      // World time it was last turned to
    unsigned maws_logged = 0;           // MAWS launch declarations already put on the console
    bool left_mfd_fcr = true;
    bool radar_lock_failed_shown = false;
    auto active_world = [&]() -> sim::World& { return dogfight_active ? dogfight->world : *free_world; };

    // Dogfight HUD cue timers and the event read cursor.
    double hit_cue_timer = 0.0;
    int dogfight_events_read = 0;

    auto start_dogfight_now = [&]() {
        dogfight_setup.own_type = selected_aircraft;
        if (!bandit_type_set) dogfight_setup.bandit_type = selected_aircraft;
        dogfight_setup.seed = static_cast<uint32_t>(SDL_GetTicks()) | 1u;
        dogfight->setup(dogfight_setup);
        dogfight_active = true;
        own = &dogfight->world.aircraft[0];
        stovl_selected = false;
        gear_down = false;
        throttle = 0.9;
        hit_cue_timer = 0.0;
        dogfight_events_read = 0;
        renderer.camera_rig().reset();
        // Dogfight switch: a visual fight starts with the radar in ACM, which
        // finds and locks the bandit by itself; RWS search cannot hold a jet
        // a mile away in a turning fight. Evade starts in CRM, the bandit
        // miles back.
        radar_scope.sync(dogfight->world, 0);
        radar_scope.set_master_mode(dogfight->world, !dogfight->evade_mode());
        sweep_scope.reset();
        sweep_world_time = dogfight->world.time;
        if (dogfight->evade_mode()) {
            // The evade set-up screens reopen on the level last flown (F11 included).
            main_menu.set_evade_difficulty(static_cast<int>(dogfight_setup.evade_difficulty));
            const auto profile = sim::EvadeProfile::of(dogfight_setup.evade_difficulty);
            std::cout << "[EVADE] " << sim::to_string(dogfight_setup.evade_difficulty) << ": "
                      << aircraft::to_string(dogfight_setup.bandit_type) << " (" << profile.summary << ") "
                      << static_cast<int>(profile.start_range_m / 1852.0 + 0.5)
                      << " nm in your six. Survive " << static_cast<int>(profile.survive_s)
                      << " s. C dispenses chaff.\n";
        } else {
            std::cout << "[DOGFIGHT] " << sim::to_string(dogfight_setup.geometry) << " vs "
                      << sim::to_string(dogfight_setup.skill) << " " << aircraft::to_string(dogfight_setup.bandit_type)
                      << ". Fight's on! Radar in ACM: put the bandit in the HUD to lock (O: scan, K: CRM).\n";
        }
    };
    auto leave_dogfight = [&]() {
        if (!dogfight_active) return;
        dogfight_active = false;
        free_world->clear();
        free_world->add(selected_aircraft, reset_flight_state());
        own = &free_world->aircraft[0];
        gear_down = true;
        own->landing_gear.deployed = gear_down;
        apply_stovl_start();
        radar_scope.sync(*free_world, 0);
        radar_scope.set_master_mode(*free_world, false);
    };

    // Ejection. The handle has to be held for EJECT_HOLD_S before it fires,
    // so a brushed key cannot throw the pilot out of a good jet.
    constexpr double EJECT_HOLD_S = 0.5;
    double eject_hold_s = 0.0;
    bool eject_pending = false;        // Handle pulled: goes out with the next physics step
    double eject_throttle = 0.0;       // Lever where the pilot left it
    bool camera_was_cockpit = false;   // Restored when a new flight starts
    bool was_ejected = false;
    math::Vector3 own_seat_prev{};     // Seat position before the last physics step (interpolation)
    double eject_cam_yaw = 0.0;        // Heading the follow camera looks along after ejecting
    ui::DrawList eject_overlay;

    // A fresh jet for the pilot: the seat is re-armed with it.
    auto reset_own = [&]() {
        own->reset(reset_flight_state());
        apply_stovl_start();
        active_world().seats[0].reset();
        eject_pending = false;
        eject_hold_s = 0.0;
    };

    auto switch_aircraft = [&](aircraft::AircraftType new_type) {
        // Re-selecting the active airframe would otherwise reset every actuator
        // and re-seat the gear for no visible reason, UNLESS currently crashed.
        if (new_type == selected_aircraft && !own->crashed) {
            std::cout << "[AIRCRAFT] " << aircraft::to_string(new_type)
                      << " already selected.\n";
            return;
        }

        selected_aircraft = new_type;
        stovl_selected = false; // A new airframe comes up in CTOL
        // A dogfight is set up for its airframes: changing jet means leaving it.
        leave_dogfight();
        // Carry the fuel state across (an in-flight airframe change must not
        // hand the pilot a full tank); the OFC re-plans on the new limits.
        own->configure(new_type);
        own->landing_gear.deployed = gear_down;

        // If switching while crashed or stationary on the runway, reset stance cleanly
        if (own->crashed || (!start_airborne && own->state.airspeed() < 5.0 && own->landing_gear.weight_on_wheels())) {
            reset_own();
            gear_down = true;
            own->landing_gear.deployed = gear_down;
            input_mgr.trim_hat.reset();
            throttle = initial_throttle();
        }

        // Raise the on-screen confirmation. The console banner below is
        // invisible in fullscreen, so the HUD is the signal that actually
        // reaches the pilot.
        aircraft_switch_timer = AIRCRAFT_SWITCH_BANNER_SEC;

        std::cout << "\n=========================================================\n";
        std::cout << "  [AIRCRAFT SWITCHED] -> " << aircraft::to_string(new_type) << "\n";
        std::cout << "  " << aircraft::signature_spec(new_type) << "\n";
        std::cout << "=========================================================\n";
        if (window) {
            std::string title = std::string(aircraft::to_string(new_type)) + " - Cockpit View";
            SDL_SetWindowTitle(window, title.c_str());
        }
    };

    // STOVL/HOOK button. Each press flips the request; the flight model
    // decides whether (250 kt LiftFan limit) and how quickly (~15 s) the
    // lift system converts.
    constexpr double STOVL_CUE_SEC = 3.0;
    double stovl_cue_timer = 0.0;          // On-screen mode cue after a press or a mode change
    propulsion::StovlMode prev_stovl_mode = own->stovl.mode;
    auto press_stovl_button = [&]() {
        if (own->type != aircraft::AircraftType::F35B_LIGHTNING_II) {
            std::cout << "[STOVL] Only the F-35B has a lift system.\n";
            return;
        }
        if (active_world().seats[0].initiated()) return;
        stovl_selected = !stovl_selected;
        stovl_cue_timer = STOVL_CUE_SEC;
        std::cout << "[STOVL] " << (stovl_selected ? "STOVL selected: doors open, LiftFan engages, nozzle down"
                                                   : "CTOL selected: nozzle aft, LiftFan declutches, doors close")
                  << "\n";
    };

    bool flight_started = no_menu;          // False until the pilot leaves the boot menu
    bool mission_from_main_menu = false;    // Tactical menu opened from the main menu returns there
    if (!no_menu) {
        main_menu.open(boot_screen, /*in_flight=*/false);
        main_menu.update(0.0f, static_cast<float>(pixel_w), static_cast<float>(pixel_h));
        if (boot_tab) main_menu.show_settings_tab(*boot_tab);
    }
    SDL_Gamepad* gamepad = nullptr;
    auto upper = [](std::string text) {
        for (char& c : text) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        return text;
    };
    if (start_chase) {
        renderer.camera_rig().set_mode(graphics::CameraMode::CHASE);
    }
    if (start_dogfight) {
        start_dogfight_now();
        if (start_chase) renderer.camera_rig().set_mode(graphics::CameraMode::CHASE);
    }
    if (look_yaw_deg != 0.0f || look_pitch_deg != 0.0f) {
        constexpr float kDeg = 3.14159265f / 180.0f;
        renderer.camera_rig().add_head_look(look_yaw_deg * kDeg, look_pitch_deg * kDeg);
    }

    // Keyboard stick emulation state
    double target_pitch = 0.0;
    double target_roll  = 0.0;
    double target_yaw   = 0.0;
    double stick_pitch  = 0.0;
    double stick_roll   = 0.0;
    double pedal_yaw    = 0.0;
    double throttle_dir = 0.0;  // Commanded lever direction this frame [-1, +1]
    bool throttle_in_detent = false; // Keyboard lever stopped on the STOVL hold detent until the key is released

    // Keyboard pull: a tap or a short hold is a firm pull (~6 G on a 9 G jet);
    // keep holding and the stick walks on to the stop, so the last few G take
    // a deliberate, sustained pull rather than arriving with every key press.
    constexpr double KBD_PULL_BASE      = 0.65;  // Stick on first press
    constexpr double KBD_PULL_DELAY_S   = 0.6;   // Held this long before it walks on
    constexpr double KBD_PULL_RAMP_PER_S = 0.35; // ...at this rate, to the stop
    double pitch_up_held_s = 0.0;
    bool pitch_up_down = false;
    auto keyboard_pull = [&](double held_s) {
        return std::min(1.0, KBD_PULL_BASE + KBD_PULL_RAMP_PER_S * std::max(0.0, held_s - KBD_PULL_DELAY_S));
    };

    // Physical throttle lever slew: a pilot takes roughly 1.5 s to run the
    // lever from idle to full reheat, so the full travel is covered in that
    // time regardless of frame rate.
    constexpr double THROTTLE_SLEW_PER_SEC = 0.67;

    // A hardware throttle takes over only once the pilot actually moves it,
    // so an idle or uncalibrated axis never silently overrides the keyboard.
    bool hardware_throttle_active = false;
    bool hardware_throttle_disabled = false;
    double hardware_throttle_ref = -1.0;

    // A hardware lever must move by this much before it takes over, so that
    // axis noise on an idle stick cannot capture the throttle.
    constexpr double HARDWARE_THROTTLE_DEADBAND = 0.25;

    bool speedbrake_out = false; // Commanded speedbrake switch position
    bool wheel_braking = false;  // Toe brakes (keyboard)

    bool running = true;
    int frame_count = 0;
    FrameStats frame_stats;
    if (profile) renderer.gpu_profiler().set_enabled(true);
    auto prev_time = std::chrono::high_resolution_clock::now();
    double accumulator = 0.0;
    // Keeps the loop at most two frames ahead of the GPU (see FramePacer).
    graphics::FramePacer frame_pacer;

    while (running) {
        // Before input is read, so what is sampled here is what gets drawn.
        frame_pacer.wait();

        // Event handling
        const ui::ControlSettings& controls_cfg = settings.committed().controls;
        auto bound = [&controls_cfg](ui::InputAction a, SDL_Scancode sc) { return controls_cfg.binding(a).matches(static_cast<int32_t>(sc)); };

        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_EVENT_QUIT) {
                running = false;
                continue;
            }
            if (ev.type == SDL_EVENT_WINDOW_RESIZED || ev.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
                int pw = 0, ph = 0;
                SDL_GetWindowSizeInPixels(window, &pw, &ph);
                if (pw > 0 && ph > 0) {
                    renderer.set_viewport(pw, ph);
                    pixel_w = pw;
                    pixel_h = ph;
                }
                continue;
            }
            if (ev.type == SDL_EVENT_GAMEPAD_ADDED && !gamepad) {
                gamepad = SDL_OpenGamepad(ev.gdevice.which);
                continue;
            }
            if (ev.type == SDL_EVENT_GAMEPAD_REMOVED && gamepad && SDL_GetGamepadID(gamepad) == ev.gdevice.which) {
                SDL_CloseGamepad(gamepad);
                gamepad = nullptr;
                continue;
            }

            // The main/pause menu owns all input while it is open.
            if (main_menu.is_open()) {
                if (auto menu_event = menu_input.translate(ev, SDL_GetWindowPixelDensity(window), main_menu.is_capturing_key())) {
                    main_menu.handle(*menu_event);
                }
                continue;
            }
            const bool pause_pressed =
                (ev.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN && ev.gbutton.button == SDL_GAMEPAD_BUTTON_START) ||
                (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_ESCAPE && !ev.key.repeat && !renderer.menu().is_open());
            if (pause_pressed && !renderer.menu().is_open()) {
                main_menu.open(ui::ScreenId::MAIN, flight_started);
                continue;
            }

            if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                if (renderer.menu().is_open() && ev.button.button == SDL_BUTTON_LEFT) {
                    bool launched = renderer.menu().handle_click();
                    if (launched || renderer.menu().selected_index() != static_cast<int>(selected_aircraft)) {
                        switch_aircraft(renderer.menu().get_selected_type());
                    }
                    if (launched) {
                        leave_dogfight();
                        reset_own();
                        own->landing_gear.deployed = gear_down;
                        throttle = initial_throttle();
                        flight_started = true;
                        mission_from_main_menu = false;
                    }
                } else if (ev.button.button == SDL_BUTTON_RIGHT) {
                    right_mouse_down = true;
                } else if (ev.button.button == SDL_BUTTON_MIDDLE) {
                    renderer.camera_rig().reset_head_look();
                }
            } else if (ev.type == SDL_EVENT_MOUSE_BUTTON_UP) {
                if (ev.button.button == SDL_BUTTON_RIGHT) {
                    right_mouse_down = false;
                }
            } else if (ev.type == SDL_EVENT_MOUSE_MOTION) {
                if (renderer.menu().is_open()) {
                    int win_w = 0, win_h = 0;
                    SDL_GetWindowSize(window, &win_w, &win_h);
                    const float aspect = (win_h > 0) ? static_cast<float>(win_w) / static_cast<float>(win_h) : 16.0f / 9.0f;
                    renderer.menu().update_mouse(static_cast<int>(ev.motion.x), static_cast<int>(ev.motion.y), win_w, win_h, aspect);
                } else if (right_mouse_down || mouse_look_toggle) {
                    // 0.0032 rad/px at 100% sensitivity: smooth pilot cervical rotation
                    constexpr float MOUSE_LOOK_RAD_PER_PX = 0.0032f;
                    const float gain = MOUSE_LOOK_RAD_PER_PX * static_cast<float>(controls_cfg.mouse_sensitivity) / 100.0f;
                    const float y_sign = controls_cfg.invert_mouse_y ? 1.0f : -1.0f;
                    renderer.camera_rig().add_head_look(
                        static_cast<float>(ev.motion.xrel) * gain,
                        static_cast<float>(ev.motion.yrel) * gain * y_sign
                    );
                }
            } else if (ev.type == SDL_EVENT_MOUSE_WHEEL) {
                renderer.camera_rig().zoom_chase(static_cast<float>(ev.wheel.y) * 1.5f);
            } else if (ev.type == SDL_EVENT_KEY_DOWN) {
                if (ev.key.key == SDLK_M) {
                    renderer.menu().toggle(selected_aircraft);
                    std::cout << "[MENU] Aircraft Selection Menu " << (renderer.menu().is_open() ? "OPEN" : "CLOSED") << "\n";
                }

                // If menu is open, handle menu navigation and airframe selection
                if (renderer.menu().is_open()) {
                    if (ev.key.key == SDLK_ESCAPE) {
                        renderer.menu().close();
                        if (mission_from_main_menu) {
                            mission_from_main_menu = false;
                            main_menu.open(ui::ScreenId::MAIN, flight_started);
                        }
                    } else if (ev.key.key == SDLK_TAB) {
                        renderer.menu().next_tab();
                    } else if (ev.key.key == SDLK_F1) {
                        renderer.menu().set_tab(fastjet::ui::MenuTab::AIRFRAME_SELECT);
                    } else if (ev.key.key == SDLK_F2) {
                        renderer.menu().set_tab(fastjet::ui::MenuTab::FLIGHT_ENVELOPE);
                    } else if (ev.key.key == SDLK_F3) {
                        renderer.menu().set_tab(fastjet::ui::MenuTab::HARDWARE_CALIBRATION);
                    } else if (ev.key.key == SDLK_F4) {
                        renderer.menu().set_tab(fastjet::ui::MenuTab::SORTIE_DISPATCH);
                    } else if (ev.key.key == SDLK_UP || ev.key.key == SDLK_W) {
                        renderer.menu().move_up();
                    } else if (ev.key.key == SDLK_DOWN || ev.key.key == SDLK_S) {
                        renderer.menu().move_down();
                    } else if (ev.key.key == SDLK_RETURN || ev.key.key == SDLK_KP_ENTER || ev.key.key == SDLK_SPACE) {
                        switch_aircraft(renderer.menu().get_selected_type());
                        leave_dogfight();
                        reset_own();
                        own->landing_gear.deployed = gear_down;
                        throttle = initial_throttle();
                        renderer.menu().close();
                        flight_started = true;
                        mission_from_main_menu = false;
                    } else if (ev.key.key == SDLK_1) {
                        switch_aircraft(aircraft::AircraftType::F16_FIGHTING_FALCON);
                        renderer.menu().select_index(0);
                    } else if (ev.key.key == SDLK_2) {
                        switch_aircraft(aircraft::AircraftType::F15EX_EAGLE_II);
                        renderer.menu().select_index(1);
                    } else if (ev.key.key == SDLK_3) {
                        switch_aircraft(aircraft::AircraftType::EUROFIGHTER_TYPHOON);
                        renderer.menu().select_index(2);
                    } else if (ev.key.key == SDLK_4) {
                        switch_aircraft(aircraft::AircraftType::F22_RAPTOR);
                        renderer.menu().select_index(3);
                    } else if (ev.key.key == SDLK_5) {
                        switch_aircraft(aircraft::AircraftType::A10_THUNDERBOLT);
                        renderer.menu().select_index(4);
                    } else if (ev.key.key == SDLK_6) {
                        switch_aircraft(aircraft::AircraftType::F35B_LIGHTNING_II);
                        renderer.menu().select_index(5);
                    }
                    continue;
                }

                if (ev.key.key == SDLK_F1) {
                    renderer.camera_rig().set_mode(fastjet::graphics::CameraMode::COCKPIT);
                    std::cout << "[CAMERA] Cockpit View (First-Person)\n";
                }
                if (ev.key.key == SDLK_F2) {
                    renderer.camera_rig().set_mode(fastjet::graphics::CameraMode::CHASE);
                    std::cout << "[CAMERA] Chase View (3D External Airframe)\n";
                }
                if (bound(ui::InputAction::TOGGLE_VIEW, ev.key.scancode) && !ev.key.repeat) {
                    renderer.camera_rig().toggle_mode();
                    std::cout << "[CAMERA] Toggled: "
                              << (renderer.camera_rig().mode() == fastjet::graphics::CameraMode::CHASE ? "Chase View" : "Cockpit View") << "\n";
                }
                if (bound(ui::InputAction::MOUSE_LOOK, ev.key.scancode) && !ev.key.repeat) {
                    mouse_look_toggle = !mouse_look_toggle;
                    SDL_SetWindowRelativeMouseMode(window, mouse_look_toggle);
                }
                if (bound(ui::InputAction::WHEEL_BRAKES, ev.key.scancode) && environment::GroundCollision::get_agl(own->state) > 20.0) {
                    renderer.camera_rig().reset_head_look();
                }
                if (bound(ui::InputAction::TRIM_RESET, ev.key.scancode)) input_mgr.trim_hat.reset();
                if (ev.key.key == SDLK_H && !ev.key.repeat) {
                    // Routed through settings so the hotkey and the Screen
                    // Shake option stay in sync and the choice persists.
                    settings.edit([](ui::UserSettings& u) { u.graphics.screen_shake = !u.graphics.screen_shake; });
                    settings.apply();
                    std::cout << "[CAMERA] Cockpit G-Head Motion: "
                              << (renderer.camera_rig().g_head_motion() ? "ENABLED" : "DISABLED (Fixed DEP Screen-Lock mode like MSFS/Real Jets)") << "\n";
                }
                if (ev.key.key == SDLK_Y && !ev.key.repeat) {
                    renderer.camera_rig().toggle_padlock();
                    std::cout << "[CAMERA] Padlock View: "
                              << (renderer.camera_rig().mode() == fastjet::graphics::CameraMode::PADLOCK ? "ENABLED" : "DISABLED") << "\n";
                }
                if (bound(ui::InputAction::WEAPON_SELECT, ev.key.scancode) && !ev.key.repeat) {
                    player_selected_weapon = (player_selected_weapon + 1) % 3;
                    const char* w_names[] = {"M61A1 Vulcan Cannon", "AIM-9X Sidewinder (IR)", "AIM-120C AMRAAM (Radar BVR)"};
                    std::cout << "[WEAPON] Selected: " << w_names[player_selected_weapon] << "\n";
                }
                if (bound(ui::InputAction::RESET_FLIGHT, ev.key.scancode) && !ev.key.repeat && dogfight_active) {
                    // In a dogfight, reset means a fresh fight from the same set-up.
                    start_dogfight_now();
                } else if (bound(ui::InputAction::RESET_FLIGHT, ev.key.scancode) && !ev.key.repeat) {
                    // Reset from crash
                    gear_down = true;
                    reset_own();
                    own->landing_gear.deployed = gear_down;
                    input_mgr.trim_hat.reset();
                    speedbrake_out = false;
                    wheel_braking = false;
                    input_mgr.speedbrake.position = 0.0;
                    target_pitch = 0.0;
                    target_roll  = 0.0;
                    target_yaw   = 0.0;
                    stick_pitch  = 0.0;
                    stick_roll   = 0.0;
                    pedal_yaw    = 0.0;
                    throttle     = initial_throttle();
                    renderer.camera_rig().reset();
                    std::cout << "[SIM] Flight state reset ("
                              << (start_airborne ? "level flight" : "runway lineup") << ") ("
                              << aircraft::to_string(selected_aircraft) << ").\n";
                }
                if (bound(ui::InputAction::SPEEDBRAKE, ev.key.scancode) && !ev.key.repeat) {
                    // Toggle the switch; the surface itself slews at the
                    // modelled hydraulic rate rather than snapping open.
                    speedbrake_out = !speedbrake_out;
                    std::cout << "[SPEEDBRAKE] " << (speedbrake_out ? "EXTEND" : "RETRACT") << "\n";
                }
                // Radar: TMS forward / aft on one key, range bumps, left MFD page.
                if (bound(ui::InputAction::RADAR_LOCK, ev.key.scancode) && !ev.key.repeat) {
                    sim::World& w = active_world();
                    radar_scope.sync(w, 0);
                    if (radar_scope.mode == sim::RadarScope::Mode::STT) {
                        radar_scope.undesignate(w);
                        std::cout << "[RADAR] Lock dropped - " << (radar_scope.acm ? "ACM" : "RWS") << "\n";
                    } else if (radar_scope.mode == sim::RadarScope::Mode::ACM) {
                        std::cout << "[RADAR] ACM locks by itself - put the bandit in the scan (O cycles it)\n";
                    } else if (!radar_scope.designate(w)) {
                        std::cout << "[RADAR] Nothing on the scope to lock\n";
                    }
                }
                if (bound(ui::InputAction::RADAR_MODE, ev.key.scancode) && !ev.key.repeat) {
                    sim::World& w = active_world();
                    radar_scope.sync(w, 0);
                    radar_scope.toggle_master_mode(w);
                    if (radar_scope.fitted()) {
                        std::cout << "[RADAR] " << (radar_scope.acm ? "ACM - auto-acquisition inside 10 nm" : "CRM - RWS")
                                  << "\n";
                    }
                }
                if (bound(ui::InputAction::RADAR_SCAN, ev.key.scancode) && !ev.key.repeat) {
                    radar_scope.sync(active_world(), 0);
                    radar_scope.cycle_scan();
                }
                if (bound(ui::InputAction::RADAR_RANGE_UP, ev.key.scancode)) radar_scope.range_up();
                if (bound(ui::InputAction::RADAR_RANGE_DOWN, ev.key.scancode)) radar_scope.range_down();
                if (bound(ui::InputAction::MFD_PAGE, ev.key.scancode) && !ev.key.repeat) left_mfd_fcr = !left_mfd_fcr;
                if (bound(ui::InputAction::STOVL_MODE, ev.key.scancode) && !ev.key.repeat) press_stovl_button();
                if (bound(ui::InputAction::LANDING_GEAR, ev.key.scancode) && !ev.key.repeat) {
                    gear_down = !gear_down;
                    own->landing_gear.deployed = gear_down;
                    std::cout << "[GEAR] " << (gear_down ? "DOWN and locked" : "UP")
                              << "\n";
                }

                // Aircraft selection hotkeys
                if (ev.key.key == SDLK_F3) switch_aircraft(aircraft::AircraftType::F16_FIGHTING_FALCON);
                if (ev.key.key == SDLK_F4) switch_aircraft(aircraft::AircraftType::F15EX_EAGLE_II);
                if (ev.key.key == SDLK_F5) switch_aircraft(aircraft::AircraftType::EUROFIGHTER_TYPHOON);
                if (ev.key.key == SDLK_F6) switch_aircraft(aircraft::AircraftType::F22_RAPTOR);
                if (ev.key.key == SDLK_F7) switch_aircraft(aircraft::AircraftType::A10_THUNDERBOLT);
                if (ev.key.key == SDLK_F12) switch_aircraft(aircraft::AircraftType::F35B_LIGHTNING_II);
                if (ev.key.key == SDLK_F8) {
                    constexpr int kAircraftTypes = static_cast<int>(aircraft::AircraftType::F35B_LIGHTNING_II) + 1;
                    const int next_idx = (static_cast<int>(selected_aircraft) + 1) % kAircraftTypes;
                    switch_aircraft(static_cast<aircraft::AircraftType>(next_idx));
                }
                // Combat: new fight, cycle the set-up (the four dogfight set-ups,
                // then evade), cycle the bandit's skill or the evade difficulty.
                if (ev.key.key == SDLK_F9 && !ev.key.repeat) start_dogfight_now();
                if (ev.key.key == SDLK_F10 && !ev.key.repeat) {
                    if (dogfight_setup.mode == sim::EngagementMode::EVADE) {
                        dogfight_setup.mode = sim::EngagementMode::DOGFIGHT;
                        dogfight_setup.geometry = sim::StartGeometry::HEAD_ON_MERGE;
                    } else if (dogfight_setup.geometry == sim::StartGeometry::GUNNERY_RANGE) {
                        dogfight_setup.mode = sim::EngagementMode::EVADE;
                    } else {
                        dogfight_setup.geometry = static_cast<sim::StartGeometry>(
                            (static_cast<int>(dogfight_setup.geometry) + 1) % 4);
                    }
                    start_dogfight_now();
                }
                if (ev.key.key == SDLK_F11 && !ev.key.repeat) {
                    if (dogfight_setup.mode == sim::EngagementMode::EVADE) {
                        dogfight_setup.evade_difficulty = static_cast<sim::EvadeDifficulty>(
                            (static_cast<int>(dogfight_setup.evade_difficulty) + 1) % 4);
                    } else {
                        dogfight_setup.skill =
                            static_cast<sim::AiSkill>((static_cast<int>(dogfight_setup.skill) + 1) % 3);
                    }
                    start_dogfight_now();
                }
                // Quick throttle positions, so the pilot can slam to a detent
                // without holding a key. These override a hardware lever only
                // until it is next moved.
                if (ev.key.key == SDLK_1) { throttle = 0.0;  hardware_throttle_active = false; }
                if (ev.key.key == SDLK_2) { throttle = 0.08; hardware_throttle_active = false; }
                if (ev.key.key == SDLK_3) { throttle = 0.85; hardware_throttle_active = false; }
                if (ev.key.key == SDLK_4) { throttle = 1.0;  hardware_throttle_active = false; }

                // Invert throttle axis hotkey (for joysticks with inverted sliders)
                if (ev.key.key == SDLK_I) {
                    input_mgr.config.throttle_cal.inverted = !input_mgr.config.throttle_cal.inverted;
                    std::cout << "[THROTTLE] Axis Inversion: "
                              << (input_mgr.config.throttle_cal.inverted ? "INVERTED (forward = max power)"
                                                                         : "NORMAL (forward = cutoff)")
                              << "\n";
                    // Reset latching reference so change takes effect smoothly
                    hardware_throttle_ref = -1.0;
                }

                // Hard lockout for a noisy or uncalibrated hardware lever, so
                // the keyboard throttle can always be recovered in flight.
                if (ev.key.key == SDLK_J) {
                    hardware_throttle_disabled = !hardware_throttle_disabled;
                    if (hardware_throttle_disabled) {
                        hardware_throttle_active = false;
                    }
                    hardware_throttle_ref = -1.0;
                    std::cout << "[THROTTLE] Hardware lever "
                              << (hardware_throttle_disabled ? "IGNORED - keyboard only"
                                                             : "enabled")
                              << "\n";
                }
            }
        }

        // Requests raised by the main/pause menu
        while (const auto action = main_menu.poll_action()) {
            switch (*action) {
                case ui::MenuAction::START_FLIGHT:
                    main_menu.close();
                    flight_started = true;
                    break;
                case ui::MenuAction::RESUME_FLIGHT:
                    main_menu.close();
                    break;
                case ui::MenuAction::START_DOGFIGHT: {
                    main_menu.close();
                    flight_started = true;
                    const ui::DogfightChoice& pick = main_menu.dogfight_choice();
                    if (pick.own_jet != selected_aircraft) switch_aircraft(pick.own_jet);
                    dogfight_setup.mode = sim::EngagementMode::DOGFIGHT;
                    dogfight_setup.geometry = pick.start_geometry();
                    dogfight_setup.loadout = pick.weapons_loadout();
                    dogfight_setup.skill = pick.ai_skill();
                    dogfight_setup.own_type = pick.own_jet;
                    dogfight_setup.bandit_type = pick.bandit_jet;
                    start_dogfight_now();
                    break;
                }
                case ui::MenuAction::START_EVADE: {
                    main_menu.close();
                    flight_started = true;
                    const ui::EvadeChoice& pick = main_menu.evade_choice();
                    if (pick.jet != selected_aircraft) switch_aircraft(pick.jet);
                    dogfight_setup.mode = sim::EngagementMode::EVADE;
                    dogfight_setup.evade_difficulty = static_cast<sim::EvadeDifficulty>(pick.difficulty);
                    start_dogfight_now();
                    break;
                }
                case ui::MenuAction::OPEN_MISSION_SELECT:
                    main_menu.close();
                    if (flight_started) renderer.menu().open(selected_aircraft);
                    else renderer.menu().open_as_dispatch(selected_aircraft);
                    mission_from_main_menu = true;
                    break;
                case ui::MenuAction::QUIT:
                    running = false;
                    break;
            }
        }

        // Poll keyboard state for inceptors
        const bool* keys = SDL_GetKeyboardState(nullptr);
        const bool any_menu_open = renderer.menu().is_open() || main_menu.is_open();
        auto held = [&](ui::InputAction a) { return ui::sdl::is_down(keys, controls_cfg.binding(a)); };
        if (any_menu_open) {
            throttle_dir = 0.0; // A lever held while opening a menu must not keep moving
        }
        bool trigger_held = false;
        bool pickle_held = false;
        bool dispense_held = false;
        bool flare_held = false;
        bool eject_held = false;
        double antenna_dir = 0.0;
        if (keys && !own->crashed && !any_menu_open) {
            target_pitch = 0.0;
            target_roll  = 0.0;
            target_yaw   = 0.0;

            if (held(ui::InputAction::PITCH_DOWN))   target_pitch -= 0.75; // Nose down
            pitch_up_down = held(ui::InputAction::PITCH_UP);
            if (pitch_up_down) target_pitch += keyboard_pull(pitch_up_held_s); // Nose up (pull Gs)
            if (held(ui::InputAction::ROLL_LEFT))    target_roll  -= 0.85; // Roll left
            if (held(ui::InputAction::ROLL_RIGHT))   target_roll  += 0.85; // Roll right
            if (held(ui::InputAction::RUDDER_LEFT))  target_yaw   -= 0.70; // Rudder left
            if (held(ui::InputAction::RUDDER_RIGHT)) target_yaw   += 0.70; // Rudder right

            // Throttle lever direction. The lever itself is moved below, once
            // frame_dt is known, so the slew rate is in units per second rather
            // than per rendered frame.
            throttle_dir = 0.0;
            if (held(ui::InputAction::THROTTLE_UP))   throttle_dir += 1.0;
            if (held(ui::InputAction::THROTTLE_DOWN)) throttle_dir -= 1.0;

            // Wheel brakes for the rollout.
            wheel_braking = held(ui::InputAction::WHEEL_BRAKES);
            trigger_held = held(ui::InputAction::FIRE_GUN);
            pickle_held = held(ui::InputAction::FIRE_MISSILE);
            eject_held = held(ui::InputAction::EJECT);
            dispense_held = held(ui::InputAction::DISPENSE_CHAFF);
            flare_held = held(ui::InputAction::DISPENSE_FLARE);
            antenna_dir = (held(ui::InputAction::ANTENNA_UP) ? 1.0 : 0.0) - (held(ui::InputAction::ANTENNA_DOWN) ? 1.0 : 0.0);
        }

        // Compute delta time
        auto cur_time = std::chrono::high_resolution_clock::now();
        double frame_dt = std::chrono::duration<double>(cur_time - prev_time).count();
        prev_time = cur_time;
        if (frame_dt > 0.1) frame_dt = 0.1; // Clamp max frame time
        pitch_up_held_s = (pitch_up_down && !any_menu_open && keys && !own->crashed) ? pitch_up_held_s + frame_dt : 0.0;
        {
            const bool armed = !active_world().seats[0].initiated() && !own->damage.pilot_incapacitated;
            eject_hold_s = (eject_held && armed) ? eject_hold_s + frame_dt : 0.0;
            if (auto_eject_s >= 0.0 && active_world().time >= auto_eject_s) eject_hold_s = EJECT_HOLD_S;
            if (eject_hold_s >= EJECT_HOLD_S && armed && !eject_pending) {
                eject_pending = true;
                eject_throttle = hardware_throttle_active ? input_mgr.throttle_in : throttle;
            }
        }

        // Pause simulation physics while tactical menu is open
        const bool menu_open = renderer.menu().is_open() || main_menu.is_open();
        if (!menu_open) {
            accumulator += frame_dt;
        } else {
            accumulator = 0.0;
        }

        // Move the throttle lever at a fixed rate per second. Doing this here
        // (rather than in the key poll above) keeps the response identical at
        // 60 FPS and at 8,000 FPS.
        if (antenna_dir != 0.0) radar_scope.slew_elevation(antenna_dir, frame_dt);
        if (throttle_dir != 0.0) {
            const double next = std::clamp(throttle + throttle_dir * THROTTLE_SLEW_PER_SEC * frame_dt, 0.0, 1.0);
            // On the lift system the lever has a hold detent at mid travel. A
            // held key stops on it with a click; release and press again to
            // go through, so a keyboard pilot can always find "hold speed".
            constexpr double D = propulsion::kStovlThrottleDetent;
            const bool lift_system = own->stovl.equipped && own->stovl.mode != propulsion::StovlMode::CTOL;
            if (throttle_in_detent) {
                // Held against the detent: stay put.
            } else if (lift_system && throttle != D && (throttle - D) * (next - D) <= 0.0) {
                throttle = D;
                throttle_in_detent = true;
                audio_engine.cue_detent();
            } else {
                throttle = next;
            }
        } else {
            throttle_in_detent = false;
        }

        // 200 Hz fixed-step physics integration
        using ProfileClock = std::chrono::steady_clock;
        const auto t_physics = ProfileClock::now();
        constexpr double SIM_DT = 0.005;
        while (accumulator >= SIM_DT) {
            sim::AircraftControls controls{};
            if (!own->crashed) {
                flcs::PilotCommands pilot_cmd = input_mgr.update(SIM_DT);
                if (input_mgr.stovl_button_pressed) press_stovl_button();

                // Latch onto a hardware throttle only on a large, deliberate
                // movement. A resting axis dithers by a few percent (and an
                // uncalibrated one can sit anywhere), so a small threshold
                // would hand control to a lever the pilot is not touching and
                // make the keyboard throttle appear dead.
                if (joy_driver->get_device_count() > 0 && !hardware_throttle_active &&
                    !hardware_throttle_disabled) {
                    if (hardware_throttle_ref < 0.0) {
                        hardware_throttle_ref = input_mgr.throttle_in;
                    } else if (std::abs(input_mgr.throttle_in - hardware_throttle_ref) >
                               HARDWARE_THROTTLE_DEADBAND) {
                        hardware_throttle_active = true;
                        std::cout << "[THROTTLE] Hardware lever moved - now driving the engine"
                                     " (press J to return to keyboard).\n";
                    }
                }

                // Smooth keyboard inceptor transitions (tau ~ 0.08s for responsive yet smooth feel)
                const double filter_alpha = 1.0 - std::exp(-SIM_DT / 0.08);
                stick_pitch += (target_pitch - stick_pitch) * filter_alpha;
                stick_roll  += (target_roll  - stick_roll)  * filter_alpha;
                pedal_yaw   += (target_yaw   - pedal_yaw)   * filter_alpha;

                // Blend hardware joystick with keyboard controls
                if (std::abs(stick_pitch) > 0.01) pilot_cmd.pitch_stick = stick_pitch;
                if (std::abs(stick_roll)  > 0.01) pilot_cmd.roll_stick  = stick_roll;
                if (std::abs(pedal_yaw)   > 0.01) pilot_cmd.rudder_pedal = pedal_yaw;

                // Pilot preferences: stick gain and pitch inversion apply to
                // the blended command, so keyboard and hardware behave alike.
                const double stick_gain = static_cast<double>(controls_cfg.stick_sensitivity) / 100.0;
                pilot_cmd.pitch_stick = std::clamp(pilot_cmd.pitch_stick * stick_gain, -1.0, 1.0);
                pilot_cmd.roll_stick  = std::clamp(pilot_cmd.roll_stick * stick_gain, -1.0, 1.0);
                if (controls_cfg.invert_pitch) pilot_cmd.pitch_stick = -pilot_cmd.pitch_stick;

                controls.stick = pilot_cmd;
                // Prefer a physical throttle lever, but only once it has
                // actually been moved off its resting position. (Auto-GCAS
                // throttle authority is applied inside the aircraft.)
                controls.throttle = hardware_throttle_active ? input_mgr.throttle_in : throttle;
                controls.speedbrake_out = speedbrake_out;
                // Toe brakes: hardware pedals if present, otherwise the
                // keyboard brake key applies both mains evenly.
                const double kbd_brake = wheel_braking ? 1.0 : 0.0;
                controls.brake_left  = (std::max)(input_mgr.brakes.effective_left(), kbd_brake);
                controls.brake_right = (std::max)(input_mgr.brakes.effective_right(), kbd_brake);
                controls.trigger = trigger_held;
                controls.pickle = pickle_held;
                controls.dispense = dispense_held;
                controls.dispense_flare = flare_held;
                controls.eject = eject_pending;
                controls.stovl_mode = stovl_selected;
                // Nobody is flying the jet once the handle is pulled: the
                // stick is free and the throttle stays where it was left.
                if (active_world().seats[0].initiated()) {
                    controls.stick = {};
                    controls.trigger = false;
                    controls.pickle = false;
                    controls.dispense = false;
                    controls.dispense_flare = false;
                    controls.throttle = eject_throttle;
                }
            }
            own_seat_prev = active_world().seats[0].pos;

            // One fixed step for every jet, round and rule in the sky.
            if (dogfight_active) {
                dogfight->step(SIM_DT, controls);
            } else {
                std::array<sim::AircraftControls, sim::World::kMaxAircraft> all{};
                all[0] = controls;
                free_world->step(SIM_DT, all);
            }
            if (active_world().seats[0].initiated() && eject_pending) {
                eject_pending = false;
                const sim::EjectionSeat& seat = active_world().seats[0];
                std::cout << "[EJECT] " << seat.spec.name << " mode " << seat.mode << " at "
                          << static_cast<int>(seat.eject_keas) << " KEAS, "
                          << static_cast<int>(seat.eject_alt_m / 0.3048) << " ft\n";
            }
            radar_scope.step(SIM_DT, active_world(), 0);
            if (radar_scope.lock_failed_cue && !radar_lock_failed_shown) {
                std::cout << "[RADAR] Lock failed - target no longer in the radar's view\n";
            }
            radar_lock_failed_shown = radar_scope.lock_failed_cue;

            accumulator -= SIM_DT;
        }

        const auto t_physics_end = ProfileClock::now();

        // Lift-system mode changes: cue the pilot, and on the way into STOVL
        // put the keyboard lever on the hold detent, so the jet keeps its
        // speed instead of accelerating on whatever thrust was set.
        if (own->stovl.mode != prev_stovl_mode) {
            const propulsion::StovlMode m = own->stovl.mode;
            if (prev_stovl_mode == propulsion::StovlMode::CTOL &&
                (m == propulsion::StovlMode::CONVERTING_TO_STOVL || m == propulsion::StovlMode::STOVL)) {
                if (hardware_throttle_active) {
                    std::cout << "[STOVL] Converting: bring the throttle to mid travel to hold speed.\n";
                } else {
                    throttle = propulsion::kStovlThrottleDetent;
                }
            }
            if (own->stovl.equipped) stovl_cue_timer = STOVL_CUE_SEC;
            prev_stovl_mode = m;
        }
        stovl_cue_timer = (std::max)(0.0, stovl_cue_timer - frame_dt);

        // =========================================================================
        // Build Real-Time Cockpit Avionics & Propulsion Telemetry
        // All flight data is displayed inside the cockpit on the MFDs & HUD.
        // =========================================================================
        // Retire the airframe change banner in wall-clock time, so it holds for
        // the same duration regardless of frame rate.
        if (aircraft_switch_timer > 0.0) {
            aircraft_switch_timer = (std::max)(0.0, aircraft_switch_timer - frame_dt);
        }

        graphics::AvionicsTelemetry tel{};
        tel.aircraft_type  = selected_aircraft;
        tel.aircraft_switch_timer = aircraft_switch_timer;
        const double shown_throttle = hardware_throttle_active ? input_mgr.throttle_in : throttle;
        tel.throttle_input = shown_throttle;
        tel.net_thrust_n   = own->engine.net_thrust_n;

        // Propulsion spool rotor speed & core temperatures
        tel.engine_rpm_pct = 65.0 + 35.0 * std::clamp(shown_throttle, 0.0, 1.0)
                           + (shown_throttle > 0.85 ? 4.0 * (shown_throttle - 0.85) / 0.15 : 0.0);
        tel.engine_ftit_deg_c = 420.0 + 380.0 * std::clamp(shown_throttle, 0.0, 0.85) / 0.85
                              + (shown_throttle > 0.85 ? 180.0 * (shown_throttle - 0.85) / 0.15 : 0.0);
        tel.nozzle_pos_pct = (shown_throttle > 0.80) ? 90.0 * (shown_throttle - 0.80) / 0.20 : 12.0;
        tel.oil_pressure_psi = 45.0 + 8.0 * (tel.engine_rpm_pct / 100.0);
        tel.hyd_press_a_psi  = 3000.0;
        tel.hyd_press_b_psi  = 3000.0;

        switch (own->engine.detent_state) {
            case input::ThrottleController::DetentState::CUTOFF:      tel.detent_str = "CUTOFF"; break;
            case input::ThrottleController::DetentState::IDLE:        tel.detent_str = "IDLE"; break;
            case input::ThrottleController::DetentState::MIL_POWER:   tel.detent_str = "MIL POWER"; break;
            case input::ThrottleController::DetentState::AFTERBURNER: tel.detent_str = "AFTERBURNER"; break;
        }

        tel.fuel_remaining_kg = own->engine.fuel_kg;
        tel.fuel_fraction     = own->engine.fuel_fraction();
        tel.fuel_flow_kg_hr   = (shown_throttle > 0.85)
            ? (3200.0 + 8500.0 * (shown_throttle - 0.85) / 0.15)
            : (800.0 + 1600.0 * shown_throttle);

        const auto& gear = own->landing_gear;
        tel.gear_deployed     = gear.deployed;
        tel.gear_collapsed    = gear.collapsed;
        tel.gear_transit_pos  = gear.deployed ? 1.0 : 0.0;
        tel.brake_left        = gear.brake_left;
        tel.brake_right       = gear.brake_right;
        tel.on_ground         = gear.weight_on_wheels() || (environment::GroundCollision::get_agl(own->state) <= 2.2);

        tel.speedbrake_pos    = own->speedbrake.position;
        tel.is_crashed        = own->crashed;
        tel.over_g_alert      = own->imu.Nz > 0.97 * own->damage.g_limit_pos
                             || own->imu.Nz < 0.95 * own->damage.g_limit_neg;
        tel.airframe_yielded  = own->damage.yielded;
        tel.structure_failed  = own->damage.structural_failure;
        tel.high_aoa_alert    = (own->state.alpha() * (180.0 / M_PI) > 20.0);
        tel.bingo_fuel_alert  = (own->engine.fuel_kg < 800.0);
        tel.master_caution    = own->crashed || gear.collapsed || tel.bingo_fuel_alert || tel.over_g_alert
                             || tel.airframe_yielded || tel.structure_failed;

        tel.is_hardware_hotas = hardware_throttle_active || (joy_driver->get_device_count() > 0);
        tel.input_name        = tel.is_hardware_hotas ? "HOTAS / JOYSTICK" : "KEYBOARD [SLEW]";

        // On-Board Flight Computer telemetry
        const auto& ofc = own->ofc;
        tel.ofc_gloc_active        = ofc.is_gloc_active();
        tel.ofc_gcas_active        = ofc.is_gcas_active();
        tel.ofc_tumble_active      = ofc.is_tumble_active();
        tel.ofc_gloc_blackout_frac = ofc.blackout_fraction_value();
        tel.ofc_gcas_tti_sec       = ofc.gcas_tti_seconds();
        tel.ofc_g_exposure         = ofc.g_exposure_value();

        // Lift system, and the ground velocity the hover cues need, in the
        // heading frame (forward along the nose's track, + drifting right).
        tel.stovl = own->stovl;
        {
            const math::Vector3 v = own->velocity();
            const double psi = own->state.yaw();
            tel.ground_fwd_mps   =  v.x * std::cos(psi) + v.y * std::sin(psi);
            tel.ground_right_mps = -v.x * std::sin(psi) + v.y * std::cos(psi);
        }

        // Render interpolation across sub-steps (no temporal aliasing at any refresh rate).
        const double render_alpha = std::clamp(accumulator / SIM_DT, 0.0, 1.0);
        auto render_state_of = [render_alpha](const sim::Aircraft& a) {
            return a.crashed ? a.state : fdm::FlightState::interpolate(a.prev_state, a.state, render_alpha);
        };

        // Dogfight: the bandit, tracers and the combat HUD.
        renderer.clear_traffic();
        renderer.clear_tracers();
        if (dogfight_active) {
            const sim::World& w = dogfight->world;
            const sim::Aircraft& bandit = dogfight->bandit();
            renderer.add_traffic(render_state_of(bandit), bandit.landing_gear.deployed, bandit.type, &bandit.stovl);
            // One round in five is a tracer, as in a real belt.
            for (int r = 0; r < sim::World::kMaxRounds; r += 5) {
                const sim::Projectile& p = w.rounds[static_cast<size_t>(r)];
                if (p.active) renderer.add_tracer(p.pos, p.pos - p.vel * 0.02);
            }
            // A missile is seen by its motor: a bright streak while it burns,
            // then next to nothing once it coasts.
            for (const sim::Missile& m : w.missiles) {
                if (m.active && m.body.motor_burning(m.spec)) {
                    renderer.add_tracer(m.body.pos, m.body.pos - m.body.vel * 0.03);
                }
            }

            // Hits scored since the last frame light the HIT cue.
            for (; dogfight_events_read < w.event_count(); ++dogfight_events_read) {
                const sim::CombatEvent& e = w.event(dogfight_events_read);
                if (e.kind == sim::CombatEvent::Kind::HIT && e.shooter == sim::Engagement::OWN) hit_cue_timer = 0.6;
                if (e.kind == sim::CombatEvent::Kind::HIT && e.victim == sim::Engagement::OWN) {
                    std::cout << "[DOGFIGHT] Hit taken: " << sim::to_string(e.zone) << "\n";
                }
                if (e.kind == sim::CombatEvent::Kind::EJECTION && e.victim != sim::Engagement::OWN) {
                    std::cout << "[DOGFIGHT] Bandit ejected - good chute.\n";
                }
                if (e.victim != sim::Engagement::OWN) continue;
                switch (e.kind) {
                    case sim::CombatEvent::Kind::MISSILE_LAUNCH:
                        std::cout << "[EVADE] Missile launch! Bandit fired an AIM-120.\n";
                        break;
                    case sim::CombatEvent::Kind::MISSILE_DEFEATED:
                        std::cout << "[EVADE] Missile defeated.\n";
                        break;
                    case sim::CombatEvent::Kind::MISSILE_DETONATION:
                        std::cout << "[EVADE] Missile detonated " << static_cast<int>(e.miss_m) << " m away.\n";
                        break;
                    case sim::CombatEvent::Kind::LOCK_BROKEN_CHAFF:
                        std::cout << "[EVADE] Chaff broke a lock.\n";
                        break;
                    case sim::CombatEvent::Kind::STRUCTURAL_FAILURE:
                        std::cout << "[DOGFIGHT] Over-G: the airframe failed.\n";
                        break;
                    default: break;
                }
            }
            hit_cue_timer = (std::max)(0.0, hit_cue_timer - frame_dt);

            auto& c = tel.combat;
            c.active = true;
            const auto geo = sim::AirCombatGeometry::compute(own->state, bandit.state);
            c.target_valid = bandit.alive() || !bandit.crashed;
            if (geo.range > 1.0) {
                const math::Vector3 d = geo.los_b / geo.range;
                c.target_dir_b[0] = d.x; c.target_dir_b[1] = d.y; c.target_dir_b[2] = d.z;
            }
            c.target_range_m = geo.range;
            c.closure_mps = geo.closure;
            c.aspect_deg = geo.aspect * 180.0 / M_PI;
            c.ata_deg = geo.ata * 180.0 / M_PI;
            const auto sol = sim::GunSolution::compute(own->state, own->gun.spec, bandit.position(),
                                                       bandit.velocity(), bandit.acceleration());
            c.pipper_valid = sol.valid && bandit.alive();
            if (sol.valid) {
                const math::Vector3 p = own->state.q_att.rotate_ned_to_body(sol.pipper_ned - own->state.pos_ned);
                const double n = (std::max)(1.0, p.norm());
                c.pipper_dir_b[0] = p.x / n; c.pipper_dir_b[1] = p.y / n; c.pipper_dir_b[2] = p.z / n;
            }
            c.gun_max_range_m = 1200.0;
            c.in_gun_range = geo.range < c.gun_max_range_m;
            c.ammo = own->gun.ammo;
            c.gun_firing = own->gun.firing();
            std::snprintf(c.gun_name, sizeof(c.gun_name), "GUN");
            const auto energy = sim::EnergyState::compute(own->state, own->forces, own->mass);
            c.turn_rate_dps = energy.turn_rate * 180.0 / M_PI;
            c.ps_mps = energy.ps;
            c.hits_scored = dogfight->stats.hits_scored;
            c.hits_taken = dogfight->stats.hits_taken;
            c.hit_cue_s = hit_cue_timer;
            const auto& dmg = own->damage;
            c.engine_damage = dmg.damaged() && dmg.thrust_factor() < 0.999;
            c.engine_fire = dmg.engine_fire;
            c.fuel_leak = dmg.fuel_leak_kgs > 0.0;
            c.control_damage = dmg.wing_left < 1.0 || dmg.wing_right < 1.0 || dmg.tail < 1.0;
            c.selected_weapon = player_selected_weapon;
            if (player_selected_weapon == 0) {
                std::snprintf(c.weapon_name, sizeof(c.weapon_name), "%s", "GUN");
                c.ir_seeker_active = false;
            } else if (player_selected_weapon == 1) {
                std::snprintf(c.weapon_name, sizeof(c.weapon_name), "%s", "AIM-9X");
                c.ir_seeker_active = true;
            } else {
                std::snprintf(c.weapon_name, sizeof(c.weapon_name), "%s", "AIM-120C");
                c.ir_seeker_active = false;
            }
            c.missiles_remaining = w.missile_load[sim::Engagement::OWN];
            c.flares = w.flare_load[sim::Engagement::OWN];

            if (w.count > 1 && bandit.alive()) {
                const math::Vector3 rel = bandit.position() - own->position();
                const double target_range = rel.norm();
                c.target_range_m = target_range;
                const auto spec = (player_selected_weapon == 2) ? sim::missile_specs::aim120c() : sim::missile_specs::aim9x();
                const auto dlz = sim::DynamicLaunchZone::compute(own->state, bandit.state, spec);
                c.dlz_rmax_m = dlz.rmax_m;
                c.dlz_rne_m = dlz.rne_m;
                c.dlz_rmin_m = dlz.rmin_m;
                c.shoot_cue = dlz.in_envelope(target_range);

                if (player_selected_weapon == 1) {
                    const math::Vector3 los_b = own->state.q_att.rotate_ned_to_body(rel.normalized());
                    const double off_bore = std::acos(std::clamp(los_b.x, -1.0, 1.0));
                    c.ir_seeker_locked = (off_bore < 40.0 * M_PI / 180.0) && (target_range < dlz.rmax_m);
                    c.ir_seeker_dir_b[0] = los_b.x;
                    c.ir_seeker_dir_b[1] = los_b.y;
                    c.ir_seeker_dir_b[2] = los_b.z;
                }
            }

            for (; dogfight_events_read < w.event_count(); ++dogfight_events_read) {
                const auto& ev = w.event(dogfight_events_read);
                if (ev.kind == sim::CombatEvent::Kind::MISSILE_LAUNCH) {
                    audio_engine.cue_missile_launch();
                } else if (ev.kind == sim::CombatEvent::Kind::MISSILE_DETONATION) {
                    audio_engine.cue_explosion(ev.miss_m);
                    if (ev.victim >= 0 && ev.victim < w.count) {
                        renderer.trigger_explosion(w.aircraft[ev.victim].position());
                    }
                }
            }

            const auto bandit_name = aircraft::to_short_string(dogfight_setup.bandit_type);
            const bool evade = dogfight->evade_mode();
            if (evade) {
                // The RWR, the chaff count and the time left until the bandit is bingo.
                const sim::RwrStatus rwr = dogfight->own_rwr();
                c.rwr_active = true;
                c.rwr_level = static_cast<int>(rwr.level);
                c.rwr_emitter_valid = rwr.emitter_valid;
                c.rwr_emitter_bearing_deg = rwr.emitter_bearing * 180.0 / M_PI;
                std::snprintf(c.rwr_symbol, sizeof(c.rwr_symbol), "%s", rwr.emitter_symbol);
                c.rwr_missile_valid = rwr.missile_valid;
                c.rwr_missile_bearing_deg = rwr.missile_bearing * 180.0 / M_PI;
                c.rwr_missile_range_m = rwr.missile_range_m;
                c.chaff = w.chaff_load[sim::Engagement::OWN];
                c.blink = (SDL_GetTicks() / 160) % 2 == 0;
                const int left = static_cast<int>(
                    std::max(0.0, dogfight->setup_data.time_limit_s - dogfight->stats.duration_s));
                std::snprintf(c.status, sizeof(c.status), "EVADE %s  %.*s  BINGO %d:%02d",
                              sim::to_string(dogfight_setup.evade_difficulty), static_cast<int>(bandit_name.size()),
                              bandit_name.data(), left / 60, left % 60);
            } else {
                std::snprintf(c.status, sizeof(c.status), "%s %.*s  %s", sim::to_string(dogfight_setup.skill),
                              static_cast<int>(bandit_name.size()), bandit_name.data(),
                              sim::to_string(dogfight_setup.geometry));
            }
            {
                // Missile approach warning, in every fight: it sees the heat
                // seekers the RWR cannot hear.
                const sim::MissileWarningSystem& mws = w.maws[sim::Engagement::OWN];
                const sim::MawsStatus& ms = mws.status;
                c.maws_fitted = mws.spec.fitted();
                c.maws_threat = ms.threat;
                c.maws_threat_count = ms.threat_count;
                c.maws_clock = ms.clock;
                c.maws_bearing_deg = ms.bearing_rad * 180.0 / M_PI;
                c.maws_elevation_deg = ms.elevation_rad * 180.0 / M_PI;
                c.maws_time_to_go_s = ms.time_to_go_s;
                c.maws_range_valid = ms.range_valid;
                c.maws_range_m = ms.range_m;
                c.maws_launch_count = ms.launch_count;
                if (ms.launch_count < maws_logged) maws_logged = 0; // A new world
                if (ms.launch_count > maws_logged) {
                    maws_logged = ms.launch_count;
                    std::cout << "[MAWS] " << mws.spec.name << ": missile " << ms.clock << " o'clock, "
                              << static_cast<int>(std::lround(ms.time_to_go_s)) << " s to go.\n";
                }
            }
            if (dogfight->finished()) {
                // Name a self-inflicted loss: pulling the wings off is not being shot down.
                const char* cause = own->damage.pilot_ejected ? "  (EJECTED)"
                                  : own->damage.structural_failure ? "  (OVER-G)"
                                  : bandit.damage.structural_failure ? "  (BANDIT OVER-G)" : "";
                std::snprintf(c.banner, sizeof(c.banner), "%.20s%.18s", sim::to_string(dogfight->outcome), cause);
                const auto& st = dogfight->stats;
                std::snprintf(c.debrief, sizeof(c.debrief), "HITS %d  ROUNDS %d  TAKEN %d  TIME %d SEC  PEAK %.1fG",
                              st.hits_scored, st.rounds_fired, st.hits_taken, static_cast<int>(st.duration_s),
                              st.peak_g);
                if (evade) {
                    std::snprintf(c.debrief2, sizeof(c.debrief2),
                                  "MISSILES %d  DEFEATED %d  CHAFF %d  LOCKS BROKEN %d  LOCKED %d SEC",
                                  st.missiles_at_you, st.missiles_defeated, st.chaff_used, st.locks_broken_by_chaff,
                                  static_cast<int>(st.time_locked_s));
                }
            }
        }

        // Ejected pilots: theirs and ours. Once our seat is clear of the jet
        // the camera follows the pilot and the jet flies on as traffic.
        renderer.escapes().clear();
        {
            const sim::World& w = active_world();
            for (int i = 0; i < w.count; ++i) {
                const sim::EjectionSeat& seat = w.seats[static_cast<size_t>(i)];
                if (!seat.clear_of_jet()) continue;
                graphics::EscapeView ev;
                ev.pilot_pos = i == 0 ? own_seat_prev + (seat.pos - own_seat_prev) * render_alpha : seat.pos;
                ev.attitude = seat.attitude;
                ev.seat_attached = seat.seat_attached();
                ev.drogue = seat.drogue_out();
                ev.chute = static_cast<float>(seat.chute_fraction);
                ev.seat_free = seat.seat_free;
                ev.seat_pos = seat.seat_pos;
                ev.seat_attitude = seat.attitude;
                renderer.escapes().add(ev);
            }
        }
        const bool own_ejected = active_world().seats[0].clear_of_jet();
        if (own_ejected && !was_ejected) {
            camera_was_cockpit = renderer.camera_rig().mode() == graphics::CameraMode::COCKPIT;
            eject_cam_yaw = own->state.yaw();
            renderer.camera_rig().set_mode(graphics::CameraMode::CHASE);
        } else if (!own_ejected && was_ejected && camera_was_cockpit) {
            renderer.camera_rig().set_mode(graphics::CameraMode::COCKPIT);
        }
        was_ejected = own_ejected;
        renderer.set_ownship_visible(!own_ejected);
        if (own_ejected) {
            renderer.add_traffic(render_state_of(*own), own->landing_gear.deployed, own->type, &own->stovl);
            // F1 has no cockpit to go back to.
            renderer.camera_rig().set_mode(graphics::CameraMode::CHASE);
        }
        tel.pilot_ejected = own_ejected;

        // Fire-control radar page.
        {
            constexpr double R2D = 180.0 / M_PI;
            constexpr double KT = 1.0 / 0.514444;
            const sim::World& w = active_world();
            radar_scope.sync(w, 0);
            auto& rt = tel.radar;
            rt.page_fcr = left_mfd_fcr && radar_scope.fitted(); // The A-10 has no FCR page
            rt.fitted = radar_scope.fitted();
            rt.mode = static_cast<int>(radar_scope.mode);
            rt.acm = radar_scope.acm;
            rt.acm_scan = static_cast<int>(radar_scope.acm_scan);
            rt.range_scale_nm = radar_scope.range_scale_m() / 1852.0;
            rt.az_limit_deg = sim::RadarScope::AZ_LIMIT_RAD * R2D;
            {
                // What the search covers, to the edges of the beam.
                const sim::RadarScope::Pattern p = radar_scope.pattern();
                const double h = sim::RadarScope::BEAM_HALF_RAD;
                rt.scan_az_deg = (p.az_hi + h) * R2D;
                rt.scan_el_lo_deg = (p.el_lo - h) * R2D;
                rt.scan_el_hi_deg = (p.el_hi + h) * R2D;
            }
            rt.bars = sim::RadarScope::BARS;
            rt.ant_az_deg = radar_scope.ant_az * R2D;
            rt.ant_el_deg = radar_scope.ant_el * R2D;
            rt.pitch_deg = own->state.pitch() * R2D;
            rt.roll_deg = own->state.roll() * R2D;
            rt.blink = (SDL_GetTicks() / 160) % 2 == 0;
            rt.hit_count = 0;
            for (int i = 0; i < radar_scope.hit_count && rt.hit_count < graphics::RadarTelemetry::kMaxHits; ++i) {
                const sim::RadarHit& h = radar_scope.hits[static_cast<size_t>(i)];
                auto& o = rt.hits[rt.hit_count++];
                o.az_deg = static_cast<float>(h.az_rad * R2D);
                o.range_nm = static_cast<float>(h.range_m / 1852.0);
                o.age = radar_scope.frame - h.frame;
            }
            // The cursor rests on the brick it would lock, else mid-scope.
            const sim::RadarHit* ch = radar_scope.cursor_hit();
            rt.cursor_az_deg = ch ? ch->az_rad * R2D : 0.0;
            rt.cursor_range_nm = ch ? ch->range_m / 1852.0 : 0.5 * rt.range_scale_nm;
            const double cur_m = rt.cursor_range_nm * 1852.0;
            const double own_alt_ft = own->state.altitude() / 0.3048;
            rt.cov_top_kft = static_cast<int>(std::lround(
                (own_alt_ft + cur_m * std::sin(radar_scope.coverage_top_rad()) / 0.3048) / 1000.0));
            rt.cov_bottom_kft = static_cast<int>(std::lround(
                (own_alt_ft + cur_m * std::sin(radar_scope.coverage_bottom_rad()) / 0.3048) / 1000.0));
            if (radar_scope.mode == sim::RadarScope::Mode::STT) {
                const sim::FireControlRadar& fcr = w.radars[0];
                const math::Vector3 tp = fcr.track_pos, tv = fcr.track_vel;
                double az = 0.0, el = 0.0;
                sim::RadarScope::stabilised_angles(own->state, tp, az, el);
                const math::Vector3 los = tp - own->position();
                const double range = std::max(1.0, los.norm());
                const double tgt_track = std::atan2(tv.y, tv.x);
                // Aspect: where the ownship sits off the target's tail (180: nose-on).
                const double beta = sim::RadarScope::wrap_pi(std::atan2(-los.y, -los.x) - tgt_track);
                rt.track_memory = fcr.coasting;
                rt.tgt_az_deg = az * R2D;
                rt.tgt_range_nm = range / 1852.0;
                rt.tgt_alt_kft = -tp.z / 0.3048 / 1000.0;
                rt.tgt_rel_heading_deg = sim::RadarScope::wrap_pi(tgt_track - sim::RadarScope::scan_heading(own->state)) * R2D;
                rt.tgt_heading_deg = std::fmod(tgt_track * R2D + 360.0, 360.0);
                rt.tgt_gs_kt = std::hypot(tv.x, tv.y) * KT;
                rt.closure_kt = -(tv - own->velocity()).dot(los / range) * KT;
                rt.aspect_tens = static_cast<int>(std::lround((180.0 - std::abs(beta) * R2D) / 10.0));
                rt.aspect_side = beta >= 0.0 ? 'R' : 'L';
            }
            // Datalink picture: every other jet still flying, radar contact or not.
            rt.track_count = 0;
            const int locked = radar_scope.locked_target(w);
            for (int i = 1; i < w.count && rt.track_count < graphics::RadarTelemetry::kMaxTracks; ++i) {
                const sim::Aircraft& a = w.aircraft[static_cast<size_t>(i)];
                if (!a.alive()) continue;
                double az = 0.0, el = 0.0;
                sim::RadarScope::stabilised_angles(own->state, a.position(), az, el);
                const math::Vector3 v = a.velocity();
                auto& t = rt.tracks[rt.track_count++];
                t.az_deg = static_cast<float>(az * R2D);
                t.range_nm = static_cast<float>((a.position() - own->position()).norm() / 1852.0);
                t.alt_kft = static_cast<float>(a.state.altitude() / 0.3048 / 1000.0);
                t.rel_heading_deg = static_cast<float>(
                    sim::RadarScope::wrap_pi(std::atan2(v.y, v.x) - sim::RadarScope::scan_heading(own->state)) * R2D);
                t.locked = i == locked;
            }
        }

        // Sweep display: the beam turns with the fight's clock, so it stops
        // when the sim is paused.
        tel.sweep.active = dogfight_active;
        if (dogfight_active) {
            constexpr double R2D = 180.0 / M_PI;
            const sim::World& w = dogfight->world;
            const double dt = w.time - sweep_world_time;
            if (dt > 0.0) sweep_scope.update(w, 0, dt);
            sweep_world_time = w.time;
            auto& st = tel.sweep;
            st.sweep_deg = static_cast<float>(sweep_scope.sweep_rad * R2D);
            st.range_scale_nm = static_cast<float>(sweep_scope.range_scale_m() / 1852.0);
            st.fcr_az_limit_deg = static_cast<float>(sim::RadarScope::AZ_LIMIT_RAD * R2D);
            st.blip_count = 0;
            const double own_heading = sim::RadarScope::scan_heading(own->state);
            for (int i = 0; i < sweep_scope.blip_count && st.blip_count < graphics::SweepTelemetry::kMaxBlips; ++i) {
                const sim::SweepScope::Blip& b = sweep_scope.blips[static_cast<size_t>(i)];
                auto& o = st.blips[st.blip_count++];
                o.bearing_deg = static_cast<float>(sim::SweepScope::relative_bearing(own->state, b.pos) * R2D);
                o.range_nm = static_cast<float>((b.pos - own->position()).norm() / 1852.0);
                o.alt_kft = static_cast<float>(-b.pos.z / 0.3048 / 1000.0);
                o.rel_heading_deg = static_cast<float>(
                    sim::RadarScope::wrap_pi(std::atan2(b.vel.y, b.vel.x) - own_heading) * R2D);
                o.age = static_cast<float>(sweep_scope.age(b));
            }
        }

        // No threat tones over a paused sim.
        if (main_menu.is_visible() || renderer.menu().is_open()) tel.combat.rwr_level = 0;
        if (main_menu.is_visible() || renderer.menu().is_open()) tel.combat.maws_threat = false;

        // Update procedural audio engine with real-time acoustics
        fdm::FlightState render_state = render_state_of(*own);
        double heard_airspeed = own->state.airspeed();
        double heard_altitude = own->state.altitude();
        if (own_ejected) {
            const sim::EjectionSeat& seat = active_world().seats[0];
            render_state = fdm::FlightState{};
            render_state.pos_ned = own_seat_prev + (seat.pos - own_seat_prev) * render_alpha;
            // The camera rides level, behind the pilot and facing his way:
            // his own body axes tumble and swing with the seat and canopy.
            const math::Vector3 fwd = seat.attitude.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
            if (std::hypot(fwd.x, fwd.y) > 0.2) eject_cam_yaw = std::atan2(fwd.y, fwd.x);
            render_state.q_att = math::Quaternion::from_euler(0.0, 0.0, eject_cam_yaw);
            render_state.vel_b = render_state.q_att.rotate_ned_to_body(seat.vel);
            heard_airspeed = seat.vel.norm();
            heard_altitude = -seat.pos.z;
        }
        const auto air_now = environment::Atmosphere1976::compute(heard_altitude, heard_airspeed);
        audio_engine.update(tel, air_now.dynamic_pressure, heard_airspeed, frame_dt);

        // 3D combat visual effects: missiles, expanding rocket trails, flares, chaff
        renderer.set_combat_fx(active_world().missiles.data(),
                               static_cast<int>(active_world().missiles.size()),
                               active_world().flares.data(),
                               static_cast<int>(active_world().flares.size()),
                               active_world().chaff.data(),
                               static_cast<int>(active_world().chaff.size()));

        if (dogfight_active && dogfight->world.count > 1 && dogfight->world.aircraft[1].alive()) {
            renderer.set_padlock_target(dogfight->world.aircraft[1].position());
        }

        // Render Frame with in-cockpit telemetry and live hardware calibration state
        const auto t_render = ProfileClock::now();
        renderer.render_frame(frame_dt, render_state, own->imu, own->crashed, tel, &input_mgr.config, &input_mgr);
        const auto t_render_end = ProfileClock::now();

        // Ejection status, over the scene.
        if (active_world().seats[0].initiated() && !main_menu.is_visible()) {
            const sim::EjectionSeat& seat = active_world().seats[0];
            const float k = static_cast<float>(pixel_h) / 1080.0f;
            const float w = 760.0f * k, h = 150.0f * k;
            const ui::Rect panel{0.5f * static_cast<float>(pixel_w) - 0.5f * w, static_cast<float>(pixel_h) - h - 40.0f * k, w, h};
            const ui::Rgba amber{1.0f, 0.72f, 0.18f, 1.0f};
            const ui::Rgba white{0.92f, 0.94f, 0.96f, 1.0f};
            eject_overlay.clear();
            eject_overlay.fill(panel, ui::Rgba{0.02f, 0.03f, 0.04f, 0.72f}, 6.0f * k);
            char line[128];
            std::snprintf(line, sizeof(line), "EJECTION  -  %s  MODE %d  -  %d KEAS", seat.spec.name, seat.mode,
                          static_cast<int>(seat.eject_keas));
            eject_overlay.text_centered(line, panel.cx(), panel.y + 30.0f * k, 26.0f * k, amber);
            const double alt_ft = std::max(0.0, -seat.pos.z) / 0.3048;
            const double agl_ft = std::max(0.0, environment::GroundCollision::get_ground_z(seat.pos.x, seat.pos.y) - seat.pos.z) / 0.3048;
            std::snprintf(line, sizeof(line), "%s   ALT %d FT   AGL %d FT   VS %+d FT/S", sim::to_string(seat.phase),
                          static_cast<int>(alt_ft), static_cast<int>(agl_ft), static_cast<int>(-seat.vel.z / 0.3048));
            eject_overlay.text_centered(line, panel.cx(), panel.y + 72.0f * k, 22.0f * k, white);
            if (seat.phase == sim::EjectionSeat::Phase::LANDED) {
                std::snprintf(line, sizeof(line), "%s%s%s   -   %s", sim::to_string(seat.outcome), seat.cause[0] ? "  -  " : "",
                              seat.cause, dogfight_active ? "F9 NEW FIGHT" : "R NEW FLIGHT");
            } else {
                std::snprintf(line, sizeof(line), "CHUTE %d%%   -   %s", static_cast<int>(seat.chute_fraction * 100.0),
                              dogfight_active ? "F9 NEW FIGHT" : "R NEW FLIGHT");
            }
            eject_overlay.text_centered(line, panel.cx(), panel.y + 112.0f * k, 22.0f * k,
                                        seat.outcome == sim::EjectionSeat::Outcome::KILLED ? ui::Rgba{1.0f, 0.3f, 0.25f, 1.0f} : amber);
            renderer.render_ui(eject_overlay);
        }

        // STOVL mode cue, for a few seconds after a press or a mode change.
        if (stovl_cue_timer > 0.0 && own->stovl.equipped && !active_world().seats[0].initiated() && !main_menu.is_visible()) {
            const propulsion::StovlStatus& st = own->stovl;
            const float k = static_cast<float>(pixel_h) / 1080.0f;
            const float w = 620.0f * k, h = 92.0f * k;
            const ui::Rect panel{0.5f * static_cast<float>(pixel_w) - 0.5f * w, static_cast<float>(pixel_h) - h - 40.0f * k, w, h};
            const ui::Rgba green{0.35f, 1.0f, 0.55f, 1.0f};
            const ui::Rgba white{0.92f, 0.94f, 0.96f, 1.0f};
            eject_overlay.clear();
            eject_overlay.fill(panel, ui::Rgba{0.02f, 0.03f, 0.04f, 0.72f}, 6.0f * k);
            char line[128];
            std::snprintf(line, sizeof(line), "%s  -  %d%% CONVERTED", propulsion::to_string(st.mode),
                          static_cast<int>(st.conversion * 100.0 + 0.5));
            eject_overlay.text_centered(line, panel.cx(), panel.y + 30.0f * k, 26.0f * k, green);
            const bool pending = stovl_selected != (st.mode == propulsion::StovlMode::STOVL ||
                                                    st.mode == propulsion::StovlMode::CONVERTING_TO_STOVL);
            const char* hint = pending ? (stovl_selected ? "STOVL REQUESTED  -  LIFTFAN LIMIT 250 KT" : "CTOL REQUESTED")
                             : st.mode == propulsion::StovlMode::CTOL ? "THROTTLE SETS THRUST"
                             : "THROTTLE DETENT HOLDS SPEED  -  STICK AFT CLIMBS";
            eject_overlay.text_centered(hint, panel.cx(), panel.y + 66.0f * k, 20.0f * k, white);
            renderer.render_ui(eject_overlay);
        }

        // Main / pause menu composited over the (paused) scene
        if (main_menu.is_visible()) {
            main_menu.set_current_aircraft(selected_aircraft);
            main_menu.set_status_text(upper(std::string(aircraft::to_string(selected_aircraft))) +
                                      (dogfight_active ? (dogfight->evade_mode() ? "  /  EVADE PAUSED" : "  /  DOGFIGHT PAUSED")
                                       : flight_started ? "  /  FLIGHT PAUSED"
                                       : stovl_start() ? (start_hover ? "  /  HOVER OVER RUNWAY 09" : "  /  STOVL APPROACH")
                                                       : "  /  RUNWAY 09 LINE-UP"));
        }
        main_menu.update(static_cast<float>(frame_dt), static_cast<float>(pixel_w), static_cast<float>(pixel_h));
        if (main_menu.is_visible()) {
            ui_draw_list.clear();
            main_menu.render(ui_draw_list);
            renderer.render_ui(ui_draw_list);
        }

        if (!screenshot_path.empty() && max_frames > 0 && frame_count + 1 >= max_frames) {
            std::vector<uint8_t> pixels(pixel_w * pixel_h * 4);
            glReadPixels(0, 0, pixel_w, pixel_h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
            std::ofstream out(screenshot_path, std::ios::binary);
            if (out) {
                out << "P6\n" << pixel_w << " " << pixel_h << "\n255\n";
                for (int y = pixel_h - 1; y >= 0; --y) {
                    for (int x = 0; x < pixel_w; ++x) {
                        int idx = (y * pixel_w + x) * 4;
                        out.put(pixels[idx + 0]);
                        out.put(pixels[idx + 1]);
                        out.put(pixels[idx + 2]);
                    }
                }
            }
        }

        const auto t_present = ProfileClock::now();
        SDL_GL_SwapWindow(window);
        frame_pacer.frame_submitted();
        if (!headless) frame_limiter.wait(ui::frame_rate_cap_hz(settings.committed().graphics.frame_rate_cap));

        if (profile) {
            auto ms = [](ProfileClock::time_point a, ProfileClock::time_point b) {
                return std::chrono::duration<double, std::milli>(b - a).count();
            };
            frame_stats.add(frame_dt * 1000.0, ms(t_physics, t_physics_end), ms(t_render, t_render_end),
                            ms(t_present, ProfileClock::now()));
            if (frame_stats.elapsed_s >= 2.0) {
                std::cout << frame_stats.report() << renderer.gpu_profiler().report() << std::flush;
            }
        }

        ++frame_count;
        if (max_frames > 0 && frame_count >= max_frames) {
            running = false;
        }
    }

    std::cout << "Simulation finished. Total frames rendered: " << frame_count << "\n";

    if (gamepad) SDL_CloseGamepad(gamepad);
    frame_pacer.destroy();
    renderer.destroy();
    audio_engine.destroy();
    SDL_GL_DestroyContext(gl_ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
