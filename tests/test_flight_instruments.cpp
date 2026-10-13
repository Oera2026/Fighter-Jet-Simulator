#include "fastjet/graphics/flight_instruments.hpp"
#include "fastjet/graphics/cockpit_gauges.hpp"
#include <SDL3/SDL.h>
#include <iostream>
#include <cassert>
#include <array>
#include <cmath>

using namespace fastjet::graphics;
using namespace fastjet::math;
using namespace fastjet::fdm;

void test_cockpit_gauges_gl();

void test_fbo_and_instruments_gl() {
    std::cout << "[Test] Flight Instruments: FBO Creation & Render-to-Texture... ";

    SDL_Init(SDL_INIT_VIDEO);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_Window* window = SDL_CreateWindow("Instruments Test", 640, 480, SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
    assert(window != nullptr);
    SDL_GLContext ctx = SDL_GL_CreateContext(window);
    assert(ctx != nullptr);

    FlightInstruments inst;
    bool ok = inst.init();
    assert(ok && "FlightInstruments failed to initialize FBO or shaders!");

    assert(inst.fbo_id() != 0);
    assert(inst.texture_id() != 0);

    // 1. Test Low AoA condition (alpha = 8 deg) -> bottom chevron
    FlightState state{};
    state.pos_ned = Vector3(0.0, 0.0, -3000.0);
    const double speed = 250.0;
    const double alpha_low = 8.0 * (3.14159265 / 180.0);
    state.vel_b = Vector3(speed * std::cos(alpha_low), 0.0, speed * std::sin(alpha_low));
    inst.update_and_render(state);
    assert(glGetError() == GL_NO_ERROR);

    // 2. Test On-Speed condition (alpha = 13 deg) -> center donut
    const double alpha_onspeed = 13.0 * (3.14159265 / 180.0);
    state.vel_b = Vector3(speed * std::cos(alpha_onspeed), 0.0, speed * std::sin(alpha_onspeed));
    inst.update_and_render(state);
    assert(glGetError() == GL_NO_ERROR);

    // 3. Test High AoA condition (alpha = 18 deg) -> top chevron
    const double alpha_high = 18.0 * (3.14159265 / 180.0);
    state.vel_b = Vector3(speed * std::cos(alpha_high), 0.0, speed * std::sin(alpha_high));
    inst.update_and_render(state);
    assert(glGetError() == GL_NO_ERROR);

    // 4. Test texture binding
    inst.bind_texture(GL_TEXTURE0);
    assert(glGetError() == GL_NO_ERROR);

    // 5. F-35B: the PCD's STOVL portal colours the LiftFan door on its plan
    // view grey while shut and green once open.
    {
        AvionicsTelemetry tel;
        tel.aircraft_type = fastjet::aircraft::AircraftType::F35B_LIGHTNING_II;
        tel.stovl.equipped = true;
        auto door_pixel = [&]() {
            inst.update_and_render(state, tel);
            // Portal centre (0.75, 0.5) in atlas NDC at 0.25 per portal
            // unit; the door symbol at portal (-0.35, 0.77).
            const int x = static_cast<int>((0.75f - 0.35f * 0.25f + 1.0f) * 0.5f * FlightInstruments::TEX_WIDTH);
            const int y = static_cast<int>((0.5f + 0.77f * 0.25f + 1.0f) * 0.5f * FlightInstruments::TEX_HEIGHT);
            unsigned char px[4] = {0, 0, 0, 0};
            glBindFramebuffer(GL_FRAMEBUFFER, inst.fbo_id());
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, px);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            return std::array<int, 4>{px[0], px[1], px[2], px[3]};
        };
        const auto shut = door_pixel();
        tel.stovl.mode = fastjet::propulsion::StovlMode::STOVL;
        tel.stovl.conversion = 1.0;
        tel.stovl.lift_fan_door = 1.0;
        const auto open = door_pixel();
        assert(glGetError() == GL_NO_ERROR);
        assert(shut[1] > 60 && std::abs(shut[0] - shut[1]) < 40);  // grey
        assert(open[1] > 150 && open[1] > open[0] + 60);           // green
    }

    inst.destroy();
    std::cout << "PASSED (FBO complete, off-screen rendering and texture binding verified)\n";

    test_cockpit_gauges_gl();
    SDL_GL_DestroyContext(ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

/// RGBA of the gauge atlas at a point inside a cell (fractions of its size).
static std::array<int, 4> gauge_pixel(const CockpitGauges& g, Gauge id, float fx, float fy) {
    const CockpitGauges::Cell c = CockpitGauges::cell(id);
    const int x = c.x + static_cast<int>(fx * static_cast<float>(c.w));
    const int y = c.y + static_cast<int>(fy * static_cast<float>(c.h));
    unsigned char px[4] = {0, 0, 0, 0};
    GLint prev = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev);
    glBindFramebuffer(GL_FRAMEBUFFER, g.fbo_id());
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, px);
    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(prev));
    return {px[0], px[1], px[2], px[3]};
}

void test_cockpit_gauges_gl() {
    std::cout << "[Test] Cockpit Gauges: live faces, lamps and pointer lag... ";

    // Atlas cells must not overlap and must fit the texture.
    for (int i = 0; i < static_cast<int>(Gauge::COUNT); ++i) {
        const auto a = CockpitGauges::cell(static_cast<Gauge>(i));
        assert(a.x >= 0 && a.y >= 0 && a.x + a.w <= CockpitGauges::TEX_WIDTH && a.y + a.h <= CockpitGauges::TEX_HEIGHT);
        for (int j = i + 1; j < static_cast<int>(Gauge::COUNT); ++j) {
            const auto b = CockpitGauges::cell(static_cast<Gauge>(j));
            const bool apart = a.x + a.w <= b.x || b.x + b.w <= a.x || a.y + a.h <= b.y || b.y + b.h <= a.y;
            assert(apart && "gauge atlas cells overlap");
        }
    }

    CockpitGauges gauges;
    assert(gauges.init());
    assert(gauges.texture_id() != 0);

    FlightState state{};
    state.pos_ned = Vector3(0.0, 0.0, -3000.0);
    state.vel_b = Vector3(200.0, 0.0, 0.0);
    AvionicsTelemetry tel{};
    tel.gear_deployed = true;
    tel.gear_transit_pos = 1.0;
    tel.engine_rpm_pct = 70.0;

    // First frame snaps every pointer to the sim.
    gauges.update_and_render(0.016, state, tel);
    assert(glGetError() == GL_NO_ERROR);
    const auto& r = gauges.readings();
    assert(std::abs(r.alt_ft - 3000.0 * 3.28084) < 1.0);
    assert(std::abs(r.rpm_pct - 70.0) < 1e-6);
    // 200 m/s TAS at 3,000 m is ~332 kt calibrated.
    assert(r.kcas > 320.0 && r.kcas < 345.0);
    // Pressurisation holds the cabin at 8,000 ft between 8,000 and 23,000 ft.
    assert(std::abs(r.cabin_alt_ft - 8000.0) < 1.0);

    // Gear down and locked: the gear lamp is green. Round dial corners are
    // transparent so the cockpit's own bezel shows through.
    const auto lamp_on = gauge_pixel(gauges, Gauge::GEAR_LAMP, 0.5f, 0.5f);
    assert(lamp_on[1] > 200 && lamp_on[1] > lamp_on[0] + 80);
    assert(gauge_pixel(gauges, Gauge::RPM, 0.02f, 0.02f)[3] == 0);
    assert(gauge_pixel(gauges, Gauge::RPM, 0.5f, 0.15f)[3] == 255);

    // Engine spools up: the RPM pointer lags rather than jumping.
    tel.engine_rpm_pct = 100.0;
    gauges.update_and_render(0.1, state, tel);
    assert(r.rpm_pct > 70.5 && r.rpm_pct < 90.0);
    for (int i = 0; i < 100; ++i) gauges.update_and_render(0.1, state, tel);
    assert(std::abs(r.rpm_pct - 100.0) < 0.1);

    // Gear up: lamp dark. Crash: ENG FIRE lights its upper legend only.
    tel.gear_deployed = false;
    tel.gear_transit_pos = 0.0;
    tel.is_crashed = true;
    gauges.update_and_render(0.016, state, tel);
    const auto lamp_off = gauge_pixel(gauges, Gauge::GEAR_LAMP, 0.5f, 0.5f);
    assert(lamp_off[1] < 60);
    const auto fire = gauge_pixel(gauges, Gauge::WARN_1, 0.06f, 0.73f);
    const auto engine = gauge_pixel(gauges, Gauge::WARN_1, 0.06f, 0.27f);
    assert(fire[0] > 80 && engine[0] < 30);

    // Low, slow and descending with the gear up: TO/LDG CONFIG.
    state.pos_ned = Vector3(0.0, 0.0, -600.0);
    state.vel_b = Vector3(80.0, 0.0, 5.0);  // ~155 kt, nose level: sinking at ~1,000 fpm
    gauges.update_and_render(0.016, state, tel);
    assert(gauges.readings().config);
    tel.gear_deployed = true;
    tel.gear_transit_pos = 1.0;
    gauges.update_and_render(0.016, state, tel);
    assert(!gauges.readings().config);

    // Chase-view return: reset() re-primes instead of sweeping pointers.
    tel.engine_rpm_pct = 65.0;
    gauges.reset();
    gauges.update_and_render(0.016, state, tel);
    assert(std::abs(r.rpm_pct - 65.0) < 1e-6);
    assert(glGetError() == GL_NO_ERROR);

    gauges.destroy();
    std::cout << "PASSED\n";
}

int main() {
    std::cout << "=== Flight Instruments (Render-to-Texture) Verification ===\n";
    test_fbo_and_instruments_gl();
    std::cout << "All Flight Instruments tests passed successfully!\n\n";
    return 0;
}
