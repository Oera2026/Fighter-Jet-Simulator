#pragma once

#include "fastjet/graphics/instrument_canvas.hpp"
#include "fastjet/graphics/cockpit_geometry.hpp"
#include "fastjet/graphics/cockpit_telemetry.hpp"
#include "fastjet/fdm/flight_state.hpp"
#include "fastjet/environment/atmosphere1976.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <tuple>
#include <utility>
#include <vector>

namespace fastjet::graphics {

/// @brief The panel instruments and lamps driven by CockpitGauges.
enum class Gauge : int {
    ASI,        ///< Airspeed / Mach indicator
    ALT,        ///< Counter-pointer altimeter
    ADI,        ///< Attitude director indicator
    HSI,        ///< Horizontal situation indicator
    VVI,        ///< Vertical velocity tape
    AOA,        ///< Angle-of-attack tape
    FUEL_QTY,   ///< Fuel quantity pointers and totalizer
    FUEL_FLOW,  ///< Fuel flow counter
    OIL,        ///< Engine oil pressure
    NOZ,        ///< Exhaust nozzle position
    RPM,        ///< Core RPM
    FTIT,       ///< Fan turbine inlet temperature
    CABIN,      ///< Cabin pressure altitude
    HYD_A,      ///< Hydraulic system A pressure
    HYD_B,      ///< Hydraulic system B pressure
    CLOCK,      ///< Panel clock
    DED,        ///< Data entry display (CNI page)
    WARN_1,     ///< ENG FIRE / ENGINE
    WARN_2,     ///< HYD/OIL PRESS
    WARN_3,     ///< FLCS / DBU ON
    WARN_4,     ///< TO/LDG CONFIG
    WARN_5,     ///< CANOPY / OXY LOW
    GEAR_LAMP,  ///< Gear-down lamp (shared by all three)
    RWR,        ///< Radar warning receiver azimuth indicator
    MDEF,       ///< Typhoon manual data entry facility: subsystem keys, display, keypad
    DWP,        ///< Typhoon dedicated warnings panel and attention getter
    GUH,        ///< Typhoon get-u-home (reversionary) flight display
    ISFD,       ///< F-35 integrated standby flight display
    COUNT
};

/// @brief Live faces for the painted cockpit instruments.
///
/// The airframe asset's cockpit has its dials, tapes, counters and lamps
/// painted into its texture, so they read the same whatever the jet is
/// doing. This draws a working face for each into a cell of its own atlas,
/// and ModelGLB::cockpit_gauges() lays each cell over the painted artwork.
///
/// Mechanical pointers are driven through first-order lags, as the real
/// instruments are: engine gauges follow the spool, the VVI trails the
/// climb rate, and nothing snaps between frames.
class CockpitGauges : public InstrumentCanvas {
public:
    static constexpr int TEX_WIDTH = 2048;
    static constexpr int TEX_HEIGHT = 1024;

    /// Brightness of the mechanical dials relative to the self-lit displays
    /// and lamps: they are lit by the cockpit, not by themselves.
    static constexpr float kDialBrightness = 0.78f;

    /// Runway centreline and TACAN station (runway midpoint), NED [m].
    static constexpr double kTacanNorth = 1500.0;
    static constexpr double kTacanEast = 0.0;
    static constexpr double kRunwayCourseDeg = 360.0;
    /// HSI course deviation scale [m of lateral offset per dot].
    static constexpr double kMetresPerDot = 150.0;

    /// @brief An atlas cell in texture pixels, origin bottom-left.
    struct Cell {
        int x, y, w, h;
        bool emissive;
    };

    [[nodiscard]] static constexpr Cell cell(Gauge g) noexcept {
        constexpr std::array<Cell, static_cast<size_t>(Gauge::COUNT)> kCells{{
            {0, 0, 256, 256, false},       // ASI
            {256, 0, 256, 256, false},     // ALT
            {512, 0, 256, 256, false},     // ADI
            {768, 0, 256, 256, false},     // HSI
            {768, 512, 110, 256, false},   // VVI   (2.2 x 5.1 cm)
            {896, 512, 110, 256, false},   // AOA
            {1024, 0, 256, 256, false},    // FUEL_QTY
            {1280, 256, 330, 256, false},  // FUEL_FLOW (3.1 x 2.4 cm)
            {0, 256, 256, 256, false},     // OIL
            {1792, 0, 256, 256, false},    // NOZ
            {1280, 0, 256, 256, false},    // RPM
            {1536, 0, 256, 256, false},    // FTIT
            {256, 256, 256, 256, false},   // CABIN
            {768, 256, 256, 256, false},   // HYD_A
            {1024, 256, 256, 256, false},  // HYD_B
            {512, 256, 256, 256, false},   // CLOCK
            {0, 512, 750, 256, true},      // DED  (8.2 x 2.8 cm)
            {1024, 512, 192, 192, true},   // WARN_1
            {1216, 512, 192, 192, true},   // WARN_2
            {1408, 512, 192, 192, true},   // WARN_3
            {1600, 512, 192, 192, true},   // WARN_4
            {1792, 512, 192, 192, true},   // WARN_5
            {0, 768, 128, 128, true},      // GEAR_LAMP
            {256, 768, 256, 256, true},    // RWR
            // Typhoon glareshield faces, each with a 16 px gutter round it so
            // mipmaps never bleed into the cells above.
            {544, 784, 480, 240, true},    // MDEF (22 x 11 cm)
            {1040, 784, 480, 240, true},   // DWP  (22 x 11 cm)
            {1536, 784, 240, 240, true},   // GUH  (8 x 8 cm)
            {1792, 784, 240, 240, true},   // ISFD (7 x 7 cm)
        }};
        return kCells[static_cast<size_t>(g)];
    }

    /// @brief A display quad showing gauge `g`, from its corners in body axes
    /// (top-left, top-right, bottom-right, bottom-left as the pilot sees it).
    [[nodiscard]] static ScreenQuad screen(Gauge g, const std::array<std::array<float, 3>, 4>& corners) noexcept {
        const Cell c = cell(g);
        ScreenQuad q;
        q.corners = corners;
        q.u0 = static_cast<float>(c.x) / TEX_WIDTH;
        q.u1 = static_cast<float>(c.x + c.w) / TEX_WIDTH;
        q.v0 = static_cast<float>(c.y) / TEX_HEIGHT;
        q.v1 = static_cast<float>(c.y + c.h) / TEX_HEIGHT;
        q.brightness = c.emissive ? 1.0f : kDialBrightness;
        return q;
    }

    /// @brief Values the lamps and pointers are showing (after lag).
    struct Readings {
        double kcas = 0.0;          ///< [kt]
        double mach = 0.0;
        double alt_ft = 0.0;
        double vvi_fpm = 0.0;
        double aoa_deg = 0.0;
        double heading_deg = 0.0;
        double rpm_pct = 0.0;
        double ftit_c = 0.0;
        double nozzle_pct = 0.0;
        double oil_psi = 0.0;
        double hyd_a_psi = 0.0;
        double hyd_b_psi = 0.0;
        double fuel_lb = 0.0;
        double fuel_flow_pph = 0.0;
        double cabin_alt_ft = 0.0;
        double dme_nm = 0.0;
        double tacan_bearing_deg = 0.0;
        double cdi_dots = 0.0;
        bool eng_fire = false;
        bool engine = false;
        bool hyd_oil = false;
        bool flcs = false;
        bool config = false;
        bool gear_green = false;
        // Typhoon warnings panel captions
        bool bingo = false;
        bool over_g = false;
        bool high_aoa = false;
        bool low_alt = false;
        bool gcas = false;
        bool structure = false;
        bool caution = false;
        bool gear_down = false;
        double speedbrake = 0.0;
        // RWR: bearings clockwise from the nose [deg]
        bool rwr_on = false;
        int rwr_level = 0;
        bool rwr_emitter = false;
        double rwr_emitter_deg = 0.0;
        char rwr_symbol[4] = "";
        bool rwr_missile = false;
        double rwr_missile_deg = 0.0;
        bool blink = false;
    };

    CockpitGauges() = default;
    ~CockpitGauges() { destroy(); }

    bool init() { return init_canvas(TEX_WIDTH, TEX_HEIGHT); }
    void destroy() noexcept { destroy_canvas(); }

    [[nodiscard]] const Readings& readings() const noexcept { return r_; }

    /// @brief Snaps every pointer to its target on the next update.
    void reset() noexcept { primed_ = false; }

    /// @brief Advances the pointers by `dt` and redraws every face.
    void update_and_render(double dt, const fdm::FlightState& state,
                           const AvionicsTelemetry& tel = AvionicsTelemetry{}) {
        if (!initialized_) return;
        update_readings(dt, state, tel);
        flash_t_ = std::fmod(flash_t_ + std::max(dt, 0.0), 60.0);

        begin_canvas();
        in_cell(Gauge::ASI);       draw_asi();
        in_cell(Gauge::ALT);       draw_alt();
        in_cell(Gauge::ADI);       draw_adi(state);
        in_cell(Gauge::HSI);       draw_hsi();
        in_cell(Gauge::VVI);       draw_vvi();
        in_cell(Gauge::AOA);       draw_aoa();
        in_cell(Gauge::FUEL_QTY);  draw_fuel_qty();
        in_cell(Gauge::FUEL_FLOW); draw_fuel_flow();
        in_cell(Gauge::OIL);       draw_dial(oil_dial(), static_cast<float>(r_.oil_psi));
        in_cell(Gauge::NOZ);       draw_dial(noz_dial(), static_cast<float>(r_.nozzle_pct));
        in_cell(Gauge::RPM);       draw_dial(rpm_dial(), static_cast<float>(r_.rpm_pct));
        in_cell(Gauge::FTIT);      draw_dial(ftit_dial(), static_cast<float>(r_.ftit_c));
        in_cell(Gauge::CABIN);     draw_dial(cabin_dial(), static_cast<float>(r_.cabin_alt_ft / 1000.0));
        in_cell(Gauge::HYD_A);     draw_dial(hyd_dial("A"), static_cast<float>(r_.hyd_a_psi / 1000.0));
        in_cell(Gauge::HYD_B);     draw_dial(hyd_dial("B"), static_cast<float>(r_.hyd_b_psi / 1000.0));
        in_cell(Gauge::CLOCK);     draw_clock();
        in_cell(Gauge::DED);       draw_ded();
        in_cell(Gauge::WARN_1);    draw_lamp("ENG FIRE", r_.eng_fire, "ENGINE", r_.engine);
        in_cell(Gauge::WARN_2);    draw_lamp("HYD/OIL", r_.hyd_oil, "PRESS", r_.hyd_oil);
        in_cell(Gauge::WARN_3);    draw_lamp("FLCS", r_.flcs, "DBU ON", false);
        in_cell(Gauge::WARN_4);    draw_lamp("TO/LDG", r_.config, "CONFIG", r_.config);
        in_cell(Gauge::WARN_5);    draw_lamp("CANOPY", false, "OXY LOW", false);
        in_cell(Gauge::GEAR_LAMP); draw_gear_lamp();
        in_cell(Gauge::RWR);       draw_rwr();
        in_cell(Gauge::MDEF);      draw_mdef();
        in_cell(Gauge::DWP);       draw_dwp();
        in_cell(Gauge::GUH);       draw_guh(state, "GUH");
        in_cell(Gauge::ISFD);      draw_guh(state, "ISFD");
        // Cells are cleared to transparent: the cockpit shows through
        // wherever a face does not paint (round dials' corners, bezels).
        render_canvas(Color4{0.0f, 0.0f, 0.0f, 0.0f});
    }

private:
    Readings r_{};
    bool primed_ = false;

    // ---- Palette -----------------------------------------------------------
    static constexpr Color4 kFace{0.020f, 0.020f, 0.022f, 1.0f};
    static constexpr Color4 kRim{0.055f, 0.055f, 0.060f, 1.0f};
    static constexpr Color4 kInk{0.90f, 0.90f, 0.86f, 1.0f};
    static constexpr Color4 kInkDim{0.60f, 0.60f, 0.58f, 1.0f};
    static constexpr Color4 kPointer{0.97f, 0.97f, 0.94f, 1.0f};
    static constexpr Color4 kHub{0.12f, 0.12f, 0.12f, 1.0f};
    static constexpr Color4 kSymbol{1.0f, 0.55f, 0.08f, 1.0f};   // fluorescent orange
    static constexpr Color4 kGreenBand{0.10f, 0.70f, 0.25f, 1.0f};
    static constexpr Color4 kAmberBand{0.95f, 0.70f, 0.10f, 1.0f};
    static constexpr Color4 kRedBand{0.90f, 0.12f, 0.10f, 1.0f};

    // ---- Readings ----------------------------------------------------------
    static void lag(double& v, double target, double tau, double dt) noexcept {
        v += (target - v) * (1.0 - std::exp(-dt / tau));
    }

    static double wrap360(double deg) noexcept {
        deg = std::fmod(deg, 360.0);
        return deg < 0.0 ? deg + 360.0 : deg;
    }

    /// Cabin pressure schedule: unpressurised to 8,000 ft, held at 8,000 ft
    /// to 23,000 ft, then a 5 psi differential above that.
    static double cabin_altitude_ft(double alt_m) noexcept {
        const double alt_ft = alt_m * 3.28084;
        if (alt_ft < 8000.0) return std::max(alt_ft, 0.0);
        const double p_amb = environment::Atmosphere1976::compute(alt_m, 0.0).pressure;
        const double p_cab = p_amb + 34474.0;
        const double h_m = 44330.8 * (1.0 - std::pow(p_cab / 101325.0, 0.190263));
        return std::max(8000.0, h_m * 3.28084);
    }

    void update_readings(double dt, const fdm::FlightState& state, const AvionicsTelemetry& tel) {
        const double alt_m = state.altitude();
        const auto air = environment::Atmosphere1976::compute(std::max(alt_m, 0.0), state.airspeed());
        // Calibrated airspeed, taken as equivalent airspeed (within 1-2% of
        // CAS through the subsonic envelope).
        const double kcas = state.airspeed() * std::sqrt(air.density / 1.225) * 1.94384;
        const double fpm = -state.velocity_ned().z * 196.8504;

        const double dn = kTacanNorth - state.pos_ned.x;
        const double de = kTacanEast - state.pos_ned.y;

        // Target values straight from the sim, before instrument lag.
        Readings t = r_;
        t.kcas = kcas;
        t.mach = air.mach_number;
        t.alt_ft = alt_m * 3.28084;
        t.vvi_fpm = fpm;
        t.aoa_deg = state.alpha() * 180.0 / 3.14159265358979;
        t.heading_deg = wrap360(state.yaw() * 180.0 / 3.14159265358979);
        t.rpm_pct = tel.engine_rpm_pct;
        t.ftit_c = tel.engine_ftit_deg_c;
        t.nozzle_pct = tel.nozzle_pos_pct;
        t.oil_psi = tel.oil_pressure_psi;
        t.hyd_a_psi = tel.hyd_press_a_psi;
        t.hyd_b_psi = tel.hyd_press_b_psi;
        t.fuel_lb = tel.fuel_remaining_kg * 2.20462;
        t.fuel_flow_pph = tel.fuel_flow_kg_hr * 2.20462;
        t.cabin_alt_ft = cabin_altitude_ft(alt_m);
        t.dme_nm = std::sqrt(dn * dn + de * de + alt_m * alt_m) / 1852.0;
        t.tacan_bearing_deg = wrap360(std::atan2(de, dn) * 180.0 / 3.14159265358979);
        t.cdi_dots = std::clamp(-state.pos_ned.y / kMetresPerDot, -2.5, 2.5);

        if (!primed_ || dt <= 0.0) {
            r_ = t;
            primed_ = true;
        } else {
            // Time constants [s] of each pointer.
            lag(r_.kcas, t.kcas, 0.15, dt);
            lag(r_.mach, t.mach, 0.15, dt);
            lag(r_.alt_ft, t.alt_ft, 0.10, dt);
            lag(r_.vvi_fpm, t.vvi_fpm, 0.60, dt);    // VVIs trail the climb rate
            lag(r_.aoa_deg, t.aoa_deg, 0.10, dt);
            lag(r_.rpm_pct, t.rpm_pct, 0.90, dt);    // spool
            lag(r_.ftit_c, t.ftit_c, 1.60, dt);      // turbine heat soak
            lag(r_.nozzle_pct, t.nozzle_pct, 0.80, dt);
            lag(r_.oil_psi, t.oil_psi, 0.50, dt);
            lag(r_.hyd_a_psi, t.hyd_a_psi, 0.30, dt);
            lag(r_.hyd_b_psi, t.hyd_b_psi, 0.30, dt);
            lag(r_.fuel_lb, t.fuel_lb, 0.50, dt);
            lag(r_.fuel_flow_pph, t.fuel_flow_pph, 0.60, dt);
            lag(r_.cabin_alt_ft, t.cabin_alt_ft, 2.00, dt);
            lag(r_.cdi_dots, t.cdi_dots, 0.30, dt);
            r_.heading_deg = t.heading_deg;          // compass card is slaved, not damped
            r_.dme_nm = t.dme_nm;
            r_.tacan_bearing_deg = t.tacan_bearing_deg;
        }

        // Lamps are discrete: no lag.
        const bool cutoff = std::strcmp(tel.detent_str, "CUTOFF") == 0;
        r_.eng_fire = tel.is_crashed;
        r_.engine = cutoff || tel.engine_rpm_pct < 60.0;
        r_.hyd_oil = tel.oil_pressure_psi < 15.0 || tel.hyd_press_a_psi < 1000.0 || tel.hyd_press_b_psi < 1000.0;
        r_.flcs = tel.ofc_tumble_active;
        // TO/LDG CONFIG: gear not down while slow, low and descending.
        r_.config = tel.gear_collapsed ||
                    (!tel.gear_deployed && !tel.on_ground && t.alt_ft < 10000.0 && kcas < 190.0 && fpm < -250.0);
        r_.gear_green = tel.gear_deployed && !tel.gear_collapsed && tel.gear_transit_pos >= 0.98;
        r_.bingo = tel.bingo_fuel_alert;
        r_.over_g = tel.over_g_alert || tel.airframe_yielded;
        r_.high_aoa = tel.high_aoa_alert;
        r_.low_alt = tel.low_altitude_alert;
        r_.gcas = tel.ofc_gcas_active;
        r_.structure = tel.structure_failed;
        r_.caution = tel.master_caution;
        r_.gear_down = tel.gear_deployed;
        r_.speedbrake = tel.speedbrake_pos;

        const CombatTelemetry& c = tel.combat;
        r_.rwr_on = c.active && c.rwr_active;
        r_.rwr_level = c.rwr_level;
        r_.rwr_emitter = c.rwr_emitter_valid;
        r_.rwr_emitter_deg = c.rwr_emitter_bearing_deg;
        std::snprintf(r_.rwr_symbol, sizeof(r_.rwr_symbol), "%s", c.rwr_symbol);
        r_.rwr_missile = c.rwr_missile_valid;
        r_.rwr_missile_deg = c.rwr_missile_bearing_deg;
        r_.blink = c.blink;
    }

    // ---- Cell and geometry helpers -----------------------------------------
    /// Local space of a cell: y in -1..1 over its height, x isotropic, so a
    /// cell of aspect a spans x in -a..a.
    void in_cell(Gauge g) noexcept {
        const Cell c = cell(g);
        const float cx = (static_cast<float>(c.x) + static_cast<float>(c.w) * 0.5f) / TEX_WIDTH * 2.0f - 1.0f;
        const float cy = (static_cast<float>(c.y) + static_cast<float>(c.h) * 0.5f) / TEX_HEIGHT * 2.0f - 1.0f;
        set_panel_viewport(cx, cy, static_cast<float>(c.h) / TEX_WIDTH, static_cast<float>(c.h) / TEX_HEIGHT);
    }

    static float aspect(Gauge g) noexcept {
        const Cell c = cell(g);
        return static_cast<float>(c.w) / static_cast<float>(c.h);
    }

    /// Point at radius `r` on dial bearing `deg` (clockwise from 12 o'clock).
    static std::pair<float, float> pol(float r, float deg) noexcept {
        const float a = deg * kPi / 180.0f;
        return {r * std::sin(a), r * std::cos(a)};
    }

    /// (x, y) turned clockwise by `deg`.
    static std::pair<float, float> rot(float x, float y, float deg) noexcept {
        const float a = deg * kPi / 180.0f;
        const float c = std::cos(a), s = std::sin(a);
        return {x * c + y * s, -x * s + y * c};
    }

    void tick(float deg, float r_out, float len, float width, const Color4& col) {
        const auto [x0, y0] = pol(r_out - len, deg);
        const auto [x1, y1] = pol(r_out, deg);
        add_bar(x0, y0, x1, y1, width, col);
    }

    /// Tapered pointer from a short tail through the hub to its tip.
    void needle(float deg, float len, float width, float tail, const Color4& col) {
        const auto [dx, dy] = pol(1.0f, deg);
        const float px = dy, py = -dx;  // perpendicular
        const float hw = width * 0.5f;
        add_convex({-dx * tail + px * hw, -dy * tail + py * hw,
                    dx * len * 0.78f + px * hw, dy * len * 0.78f + py * hw,
                    dx * len, dy * len,
                    dx * len * 0.78f - px * hw, dy * len * 0.78f - py * hw,
                    -dx * tail - px * hw, -dy * tail - py * hw},
                   col);
    }

    void hub(float r) {
        add_circle(0.0f, 0.0f, r, kHub, 20);
        add_circle(0.0f, 0.0f, r * 0.45f, kInkDim, 12);
    }

    /// Round instrument face: dark rim and matt black dial.
    void dial_face() {
        add_circle(0.0f, 0.0f, 1.0f, kRim, 64);
        add_circle(0.0f, 0.0f, 0.95f, kFace, 64);
    }

    /// Text centred on a point, turned clockwise by `deg` about it.
    void text_at(const char* s, float x, float y, float size, const Color4& col, float deg = 0.0f) {
        const auto [ox, oy] = rot(0.0f, -0.36f * size, deg);
        draw_text(s, x + ox, y + oy, size, col, 0, deg);
    }

    /// Counter window: digits on black drums with thin separators.
    void counter(const char* digits, float cx, float cy, float cell_w, float h, float size,
                 const Color4& ink = kInk) {
        const int n = static_cast<int>(std::strlen(digits));
        const float w = cell_w * static_cast<float>(n);
        add_rect(cx, cy, w + 0.04f, h + 0.04f, kRim);
        add_rect(cx, cy, w, h, Color4{0.0f, 0.0f, 0.0f, 1.0f});
        for (int i = 0; i < n; ++i) {
            const float x = cx - w * 0.5f + cell_w * (static_cast<float>(i) + 0.5f);
            if (i > 0) add_bar(x - cell_w * 0.5f, cy - h * 0.5f, x - cell_w * 0.5f, cy + h * 0.5f, 0.012f, kRim);
            const char d[2] = {digits[i], '\0'};
            text_at(d, x, cy, size, ink);
        }
    }

    // ---- Generic round dial ------------------------------------------------
    struct ScalePoint { float value; float deg; };
    struct Band { float v0, v1; Color4 col; };
    struct DialSpec {
        std::vector<ScalePoint> scale;  ///< value -> bearing, ascending
        float minor = 1.0f;             ///< minor tick step
        int major_every = 5;            ///< minor ticks per major
        std::vector<std::pair<float, const char*>> labels;
        const char* title = "";
        const char* units = "";
        float label_size = 0.2f;
        std::vector<Band> bands;
    };

    static float scale_deg(const std::vector<ScalePoint>& s, float v) noexcept {
        if (v <= s.front().value) return s.front().deg;
        for (size_t i = 1; i < s.size(); ++i) {
            if (v <= s[i].value) {
                const float f = (v - s[i - 1].value) / (s[i].value - s[i - 1].value);
                return s[i - 1].deg + f * (s[i].deg - s[i - 1].deg);
            }
        }
        return s.back().deg;
    }

    void draw_dial(const DialSpec& d, float value) {
        dial_face();
        for (const Band& b : d.bands) {
            add_ring_sector(0.0f, 0.0f, 0.80f, 0.90f, scale_deg(d.scale, b.v0), scale_deg(d.scale, b.v1), b.col, 16);
        }
        const float vmin = d.scale.front().value;
        const float vmax = d.scale.back().value;
        const int n = static_cast<int>(std::lround((vmax - vmin) / d.minor));
        for (int i = 0; i <= n; ++i) {
            const float v = vmin + d.minor * static_cast<float>(i);
            const bool major = (i % d.major_every) == 0;
            tick(scale_deg(d.scale, v), 0.92f, major ? 0.16f : 0.08f, major ? 0.040f : 0.022f, kInk);
        }
        for (const auto& [v, s] : d.labels) {
            const auto [x, y] = pol(0.64f, scale_deg(d.scale, v));
            text_at(s, x, y, d.label_size, kInk);
        }
        text_at(d.title, 0.0f, 0.30f, 0.15f, kInk);
        text_at(d.units, 0.0f, -0.36f, 0.12f, kInkDim);
        needle(scale_deg(d.scale, value), 0.86f, 0.075f, 0.18f, kPointer);
        hub(0.09f);
    }

    static const DialSpec& oil_dial() {
        static const DialSpec d{{{0.0f, -135.0f}, {100.0f, 135.0f}}, 5.0f, 4,
                                {{0.0f, "0"}, {20.0f, "2"}, {40.0f, "4"}, {60.0f, "6"}, {80.0f, "8"}, {100.0f, "10"}},
                                "OIL", "PSI x10", 0.24f, {}};
        return d;
    }

    static const DialSpec& noz_dial() {
        static const DialSpec d{{{0.0f, -135.0f}, {100.0f, 135.0f}}, 5.0f, 4,
                                {{0.0f, "0"}, {20.0f, "20"}, {40.0f, "40"}, {60.0f, "60"}, {80.0f, "80"}, {100.0f, "100"}},
                                "NOZ POS", "%", 0.19f, {}};
        return d;
    }

    static const DialSpec& rpm_dial() {
        static const DialSpec d{{{0.0f, -150.0f}, {110.0f, 150.0f}}, 2.0f, 5,
                                {{0.0f, "0"}, {20.0f, "2"}, {40.0f, "4"}, {60.0f, "6"}, {80.0f, "8"}, {100.0f, "10"}},
                                "RPM", "% x10", 0.22f,
                                {{100.0f, 110.0f, kRedBand}}};
        return d;
    }

    static const DialSpec& ftit_dial() {
        // Expanded between 700 and 1000 C, where the engine is operated.
        static const DialSpec d{{{200.0f, -150.0f}, {400.0f, -112.0f}, {600.0f, -66.0f}, {700.0f, -30.0f},
                                 {800.0f, 18.0f}, {900.0f, 66.0f}, {1000.0f, 108.0f}, {1100.0f, 132.0f},
                                 {1200.0f, 150.0f}},
                                20.0f, 5,
                                {{200.0f, "2"}, {400.0f, "4"}, {600.0f, "6"}, {700.0f, "7"}, {800.0f, "8"},
                                 {900.0f, "9"}, {1000.0f, "10"}, {1200.0f, "12"}},
                                "FTIT", "\xB0" "C x100", 0.20f,
                                {{1000.0f, 1200.0f, kRedBand}}};
        return d;
    }

    static const DialSpec& cabin_dial() {
        static const DialSpec d{{{0.0f, -150.0f}, {50.0f, 150.0f}}, 1.0f, 5,
                                {{0.0f, "0"}, {10.0f, "10"}, {20.0f, "20"}, {30.0f, "30"}, {40.0f, "40"}, {50.0f, "50"}},
                                "CABIN ALT", "FT x1000", 0.20f, {}};
        return d;
    }

    static const DialSpec& hyd_dial(const char* sys) {
        static const DialSpec a{{{0.0f, -135.0f}, {4000.0f / 1000.0f, 135.0f}}, 0.2f, 5,
                                {{0.0f, "0"}, {1.0f, "1"}, {2.0f, "2"}, {3.0f, "3"}, {4.0f, "4"}},
                                "HYD A", "PSI x1000", 0.26f, {}};
        static const DialSpec b{{{0.0f, -135.0f}, {4000.0f / 1000.0f, 135.0f}}, 0.2f, 5,
                                {{0.0f, "0"}, {1.0f, "1"}, {2.0f, "2"}, {3.0f, "3"}, {4.0f, "4"}},
                                "HYD B", "PSI x1000", 0.26f, {}};
        return sys[0] == 'A' ? a : b;
    }

    // ---- Airspeed / Mach ---------------------------------------------------
    void draw_asi() {
        // Expanded low-speed arc, compressed above 300 kt, as on the
        // airspeed/Mach indicator.
        static const std::vector<ScalePoint> s{{0.0f, 0.0f},     {80.0f, 22.0f},   {100.0f, 36.0f},  {150.0f, 80.0f},
                                               {200.0f, 120.0f}, {250.0f, 150.0f}, {300.0f, 175.0f}, {400.0f, 214.0f},
                                               {500.0f, 247.0f}, {600.0f, 274.0f}, {700.0f, 298.0f}, {800.0f, 320.0f},
                                               {850.0f, 330.0f}};
        dial_face();
        for (int v = 80; v <= 850; v += (v < 300 ? 10 : 50)) {
            const bool major = (v % 50) == 0;
            tick(scale_deg(s, static_cast<float>(v)), 0.92f, major ? 0.15f : 0.08f, major ? 0.036f : 0.020f, kInk);
        }
        for (int h = 1; h <= 8; ++h) {
            char b[4];
            std::snprintf(b, sizeof(b), "%d", h);
            const auto [x, y] = pol(0.64f, scale_deg(s, static_cast<float>(h * 100)));
            text_at(b, x, y, 0.21f, kInk);
        }
        text_at("KNOTS", 0.0f, -0.30f, 0.11f, kInkDim);

        // Mach readout in its window above the hub, clear of the pointer
        // through the cruise band.
        char m[8];
        std::snprintf(m, sizeof(m), "%.2f", std::clamp(r_.mach, 0.0, 2.99));
        add_rect(0.0f, 0.30f, 0.50f, 0.20f, kRim);
        add_rect(0.0f, 0.30f, 0.46f, 0.16f, Color4{0.0f, 0.0f, 0.0f, 1.0f});
        text_at(m, 0.0f, 0.30f, 0.15f, kInk);
        text_at("MACH", 0.0f, 0.48f, 0.09f, kInkDim);

        needle(scale_deg(s, static_cast<float>(std::clamp(r_.kcas, 0.0, 850.0))), 0.86f, 0.075f, 0.18f, kPointer);
        hub(0.09f);
    }

    // ---- Altimeter ---------------------------------------------------------
    void draw_alt() {
        dial_face();
        for (int i = 0; i < 50; ++i) {
            const bool major = (i % 5) == 0;
            tick(static_cast<float>(i) * 7.2f, 0.92f, major ? 0.16f : 0.08f, major ? 0.040f : 0.020f, kInk);
        }
        for (int d = 0; d < 10; ++d) {
            const char b[2] = {static_cast<char>('0' + d), '\0'};
            const auto [x, y] = pol(0.66f, static_cast<float>(d) * 36.0f);
            text_at(b, x, y, 0.23f, kInk);
        }
        text_at("ALT", -0.30f, -0.22f, 0.11f, kInkDim);

        // Drum counter: ten-thousands, thousands and hundreds, with the
        // fixed "00". The ten-thousands drum shows a crosshatch below 10,000 ft.
        const double alt = std::clamp(r_.alt_ft, 0.0, 99999.0);
        const int tens = static_cast<int>(alt / 10000.0) % 10;
        const int thou = static_cast<int>(alt / 1000.0) % 10;
        const int hund = static_cast<int>(alt / 100.0) % 10;
        char drums[4] = {static_cast<char>('0' + tens), static_cast<char>('0' + thou), static_cast<char>('0' + hund), '\0'};
        // Above the hub, so the pointer's tail clears it in level flight.
        const float cw = 0.16f, cy = 0.30f, h = 0.24f, cx = -0.12f;
        counter(drums, cx, cy, cw, h, 0.21f);
        text_at("00", cx + cw * 1.5f + 0.13f, cy - 0.01f, 0.15f, kInk);
        if (alt < 10000.0) {
            const float x0 = cx - cw * 1.5f;
            add_rect(x0 + cw * 0.5f, cy, cw, h, Color4{0.0f, 0.0f, 0.0f, 1.0f});
            for (int k = 0; k < 4; ++k) {
                const float yb = cy - h * 0.5f + static_cast<float>(k) * h / 3.0f;
                add_bar(x0, yb - 0.02f, x0 + cw, yb + 0.07f, 0.03f, kInk);
            }
        }

        // Barometric setting window.
        add_rect(0.28f, -0.42f, 0.44f, 0.16f, kRim);
        add_rect(0.28f, -0.42f, 0.40f, 0.12f, Color4{0.0f, 0.0f, 0.0f, 1.0f});
        text_at("29.92", 0.28f, -0.42f, 0.12f, kInk);
        text_at("IN HG", -0.24f, -0.42f, 0.09f, kInkDim);

        needle(static_cast<float>(std::fmod(alt, 1000.0) * 0.36), 0.86f, 0.075f, 0.18f, kPointer);
        hub(0.09f);
    }

    // ---- ADI ---------------------------------------------------------------
    void draw_adi(const fdm::FlightState& state) {
        constexpr float R = 0.87f;           // ball radius in the case
        constexpr float k = R / 45.0f;       // units per degree of pitch
        const float pitch = static_cast<float>(state.pitch() * 180.0 / 3.14159265358979);
        const float roll = static_cast<float>(state.roll() * 180.0 / 3.14159265358979);
        // The ball turns the opposite way to the jet: a right bank tilts the
        // horizon anticlockwise, i.e. clockwise by -roll.
        const float turn = -roll;
        auto P = [&](float x, float y) { return rot(x, y, turn); };

        const Color4 sky{0.40f, 0.58f, 0.74f, 1.0f};
        const Color4 ground{0.34f, 0.24f, 0.15f, 1.0f};
        const Color4 sky_ink{0.08f, 0.10f, 0.14f, 1.0f};

        add_circle(0.0f, 0.0f, R, ground, 72);
        const float yh = -pitch * k;
        if (yh < R) {
            std::vector<float> poly;
            constexpr int N = 96;
            for (int i = 0; i < N; ++i) {
                const float a0 = 2.0f * kPi * static_cast<float>(i) / N;
                const float a1 = 2.0f * kPi * static_cast<float>(i + 1) / N;
                const float x0 = R * std::cos(a0), y0 = R * std::sin(a0);
                const float x1 = R * std::cos(a1), y1 = R * std::sin(a1);
                const bool in0 = y0 > yh, in1 = y1 > yh;
                if (in0) {
                    const auto [px, py] = P(x0, y0);
                    poly.push_back(px); poly.push_back(py);
                }
                if (in0 != in1) {
                    const float t = (yh - y0) / (y1 - y0);
                    const auto [px, py] = P(x0 + t * (x1 - x0), yh);
                    poly.push_back(px); poly.push_back(py);
                }
            }
            if (poly.size() >= 6) add_convex(poly, sky);
        }
        // Horizon line.
        if (std::abs(yh) < R) {
            const float hx = std::sqrt(R * R - yh * yh);
            const auto [ax, ay] = P(-hx, yh);
            const auto [bx, by] = P(hx, yh);
            add_bar(ax, ay, bx, by, 0.03f, kInk);
        }
        // Pitch ladder on the ball.
        for (int p = -85; p <= 85; p += 5) {
            if (p == 0) continue;
            const float y = (static_cast<float>(p) - pitch) * k;
            if (std::abs(y) > R * 0.86f) continue;
            const bool ten = (p % 10) == 0;
            const float half = ten ? 0.22f : 0.10f;
            const Color4& ink = p > 0 ? sky_ink : kInk;
            const auto [ax, ay] = P(-half, y);
            const auto [bx, by] = P(half, y);
            add_bar(ax, ay, bx, by, ten ? 0.022f : 0.016f, ink);
            if (ten) {
                char b[4];
                std::snprintf(b, sizeof(b), "%d", std::abs(p));
                for (float side : {-1.0f, 1.0f}) {
                    const auto [tx, ty] = P(side * (half + 0.12f), y);
                    text_at(b, tx, ty, 0.10f, ink, turn);
                }
            }
        }
        // Roll pointer on the ball's edge.
        {
            const auto [ax, ay] = P(0.0f, R * 0.98f);
            const auto [bx, by] = P(-0.05f, R * 0.86f);
            const auto [cx, cy] = P(0.05f, R * 0.86f);
            add_tri(ax, ay, bx, by, cx, cy, kInk);
        }

        // Fixed case: rim and bank scale.
        add_ring(0.0f, 0.0f, R, 1.0f, kRim, 72);
        for (float b : {-90.0f, -60.0f, -45.0f, -30.0f, -20.0f, -10.0f, 10.0f, 20.0f, 30.0f, 45.0f, 60.0f, 90.0f}) {
            const bool big = std::fmod(std::abs(b), 30.0f) < 1e-3f;
            tick(b, 0.99f, big ? 0.11f : 0.07f, big ? 0.035f : 0.025f, kInk);
        }
        add_tri(0.0f, R + 0.01f, -0.05f, 0.99f, 0.05f, 0.99f, kInk);

        // Miniature aircraft.
        add_bar(-0.48f, 0.0f, -0.16f, 0.0f, 0.05f, kSymbol);
        add_bar(0.16f, 0.0f, 0.48f, 0.0f, 0.05f, kSymbol);
        add_bar(-0.16f, 0.0f, -0.16f, -0.08f, 0.05f, kSymbol);
        add_bar(0.16f, 0.0f, 0.16f, -0.08f, 0.05f, kSymbol);
        add_circle(0.0f, 0.0f, 0.04f, kSymbol, 16);
    }

    // ---- HSI ---------------------------------------------------------------
    void draw_hsi() {
        constexpr float C = 0.80f;  // compass card radius
        const float hdg = static_cast<float>(r_.heading_deg);

        add_ring(0.0f, 0.0f, C, 0.86f, kRim, 72);
        add_circle(0.0f, 0.0f, C, kFace, 72);
        for (int d = 0; d < 360; d += 5) {
            const bool ten = (d % 10) == 0;
            tick(static_cast<float>(d) - hdg, C, ten ? 0.10f : 0.05f, ten ? 0.022f : 0.014f, kInk);
        }
        static const char* kCard[] = {"N", "3", "6", "E", "12", "15", "S", "21", "24", "W", "30", "33"};
        for (int i = 0; i < 12; ++i) {
            const float b = static_cast<float>(i * 30) - hdg;
            const auto [x, y] = pol(0.60f, b);
            text_at(kCard[i], x, y, 0.15f, kInk, b);
        }
        // Fixed index marks every 45 degrees and the lubber line.
        for (int i = 1; i < 8; ++i) {
            const auto [x0, y0] = pol(C + 0.005f, static_cast<float>(i) * 45.0f);
            const auto [x1, y1] = pol(C + 0.055f, static_cast<float>(i) * 45.0f);
            add_bar(x0, y0, x1, y1, 0.03f, kInk);
        }
        add_bar(0.0f, C + 0.06f, 0.0f, C - 0.12f, 0.035f, kSymbol);

        // TACAN bearing pointer: to the field.
        {
            const float b = static_cast<float>(r_.tacan_bearing_deg) - hdg;
            const Color4 ptr{0.70f, 0.72f, 0.70f, 1.0f};
            auto seg = [&](float r0, float r1) {
                const auto [x0, y0] = pol(r0, b);
                const auto [x1, y1] = pol(r1, b);
                add_bar(x0, y0, x1, y1, 0.025f, ptr);
            };
            seg(0.46f, 0.70f);
            seg(-0.46f, -0.74f);
            const auto [tx, ty] = pol(0.78f, b);
            const auto [lx, ly] = pol(0.66f, b - 6.0f);
            const auto [rx, ry] = pol(0.66f, b + 6.0f);
            add_tri(tx, ty, lx, ly, rx, ry, ptr);
        }

        // Course arrow and deviation bar, set to the runway course.
        {
            const float c = static_cast<float>(kRunwayCourseDeg) - hdg;
            auto bar = [&](float x0, float y0, float x1, float y1, float w, const Color4& col) {
                const auto [ax, ay] = rot(x0, y0, c);
                const auto [bx, by] = rot(x1, y1, c);
                add_bar(ax, ay, bx, by, w, col);
            };
            bar(0.0f, 0.30f, 0.0f, 0.58f, 0.04f, kPointer);
            bar(0.0f, -0.30f, 0.0f, -0.70f, 0.04f, kPointer);
            const auto [hx, hy] = rot(0.0f, 0.74f, c);
            const auto [lx, ly] = rot(-0.07f, 0.56f, c);
            const auto [rx, ry] = rot(0.07f, 0.56f, c);
            add_tri(hx, hy, lx, ly, rx, ry, kPointer);
            for (float dot : {-2.0f, -1.0f, 1.0f, 2.0f}) {
                const auto [dx, dy] = rot(dot * 0.13f, 0.0f, c);
                add_circle_filled_ring(dx, dy, 0.03f);
            }
            const float dev = static_cast<float>(r_.cdi_dots) * 0.13f;
            bar(dev, -0.27f, dev, 0.27f, 0.04f, kPointer);
        }

        // Ownship.
        add_bar(0.0f, 0.13f, 0.0f, -0.13f, 0.035f, kSymbol);
        add_bar(-0.11f, 0.03f, 0.11f, 0.03f, 0.035f, kSymbol);
        add_bar(-0.05f, -0.11f, 0.05f, -0.11f, 0.03f, kSymbol);

        // Range and course windows in the upper corners of the case.
        char dme[8];
        if (r_.dme_nm < 99.95) std::snprintf(dme, sizeof(dme), "%4.1f", r_.dme_nm);
        else std::snprintf(dme, sizeof(dme), "%4.0f", std::min(r_.dme_nm, 999.0));
        char crs[8];
        std::snprintf(crs, sizeof(crs), "%03d", static_cast<int>(std::lround(kRunwayCourseDeg)) % 1000);
        for (int side : {-1, 1}) {
            const float x = static_cast<float>(side) * 0.73f;
            add_rect(x, 0.805f, 0.38f, 0.13f, kRim);
            add_rect(x, 0.805f, 0.34f, 0.10f, Color4{0.0f, 0.0f, 0.0f, 1.0f});
            text_at(side < 0 ? dme : crs, x, 0.805f, 0.10f, kInk);
        }
    }

    void add_circle_filled_ring(float x, float y, float r) {
        add_circle(x, y, r, kInk, 12);
        add_circle(x, y, r * 0.55f, kFace, 12);
    }

    // ---- Tapes -------------------------------------------------------------
    /// Vertical moving-tape frame shared by the AOA and VVI indicators.
    void tape_frame(float a) {
        add_rect(0.0f, 0.0f, a * 2.0f, 2.0f, kRim);
        add_rect(0.0f, 0.0f, a * 2.0f - 0.10f, 1.94f, kFace);
    }

    void tape_lubber(float a) {
        add_bar(-a + 0.05f, 0.0f, a - 0.05f, 0.0f, 0.03f, kSymbol);
        add_tri(-a, 0.07f, -a, -0.07f, -a + 0.12f, 0.0f, kInk);
        add_tri(a, 0.07f, a, -0.07f, a - 0.12f, 0.0f, kInk);
    }

    void draw_aoa() {
        const float a = aspect(Gauge::AOA);
        constexpr float k = 0.11f;  // units per degree
        constexpr float lim = 0.93f;
        const float aoa = static_cast<float>(r_.aoa_deg);
        tape_frame(a);
        auto band = [&](float d0, float d1, const Color4& col) {
            const float y0 = std::clamp((d0 - aoa) * k, -lim, lim);
            const float y1 = std::clamp((d1 - aoa) * k, -lim, lim);
            if (y1 - y0 > 1e-3f) add_rect(-a + 0.12f, (y0 + y1) * 0.5f, 0.10f, y1 - y0, col);
        };
        band(11.0f, 15.0f, kGreenBand);   // on-speed approach
        band(15.0f, 25.0f, kAmberBand);
        band(25.0f, 35.0f, kRedBand);
        for (int d = -10; d <= 35; ++d) {
            const float y = (static_cast<float>(d) - aoa) * k;
            if (std::abs(y) > lim - 0.02f) continue;
            const bool five = (d % 5) == 0;
            add_bar(a - (five ? 0.30f : 0.17f), y, a - 0.06f, y, five ? 0.025f : 0.015f, kInk);
            if (five && std::abs(y) < lim - 0.08f) {
                char b[4];
                std::snprintf(b, sizeof(b), "%d", std::abs(d));
                text_at(b, -0.04f, y, 0.17f, kInk);
            }
        }
        tape_lubber(a);
    }

    void draw_vvi() {
        const float a = aspect(Gauge::VVI);
        constexpr float k = 0.30f;  // units per 1,000 fpm
        constexpr float lim = 0.93f;
        const float vs = static_cast<float>(std::clamp(r_.vvi_fpm, -6000.0, 6000.0) / 1000.0);
        tape_frame(a);
        // Climb side of the tape is white, descent side black.
        const float y0 = std::clamp((0.0f - vs) * k, -lim, lim);
        if (lim - y0 > 1e-3f) add_rect(0.0f, (y0 + lim) * 0.5f, a * 2.0f - 0.10f, lim - y0, kInk);
        for (int i = -12; i <= 12; ++i) {
            const float v = static_cast<float>(i) * 0.5f;
            const float y = (v - vs) * k;
            if (std::abs(y) > lim - 0.02f) continue;
            const bool whole = (i % 2) == 0;
            const Color4& ink = v > 0.0f ? kFace : kInk;
            add_bar(a - (whole ? 0.30f : 0.17f), y, a - 0.06f, y, whole ? 0.025f : 0.015f, ink);
            if (whole && std::abs(y) < lim - 0.08f) {
                char b[4];
                std::snprintf(b, sizeof(b), "%d", std::abs(i / 2));
                text_at(b, -0.04f, y, 0.17f, ink);
            }
        }
        tape_lubber(a);
    }

    // ---- Fuel --------------------------------------------------------------
    void draw_fuel_qty() {
        // Two pointers, AL (aft and left) and FR (forward and right), each
        // on 0-4,200 lb, plus the totalizer.
        static const std::vector<ScalePoint> s{{0.0f, -140.0f}, {4200.0f, 140.0f}};
        dial_face();
        for (int v = 0; v <= 4200; v += 100) {
            const bool major = (v % 500) == 0;
            tick(scale_deg(s, static_cast<float>(v)), 0.92f, major ? 0.15f : 0.07f, major ? 0.038f : 0.020f, kInk);
        }
        for (int v = 0; v <= 40; v += 10) {
            char b[4];
            std::snprintf(b, sizeof(b), "%d", v);
            const auto [x, y] = pol(0.66f, scale_deg(s, static_cast<float>(v * 100)));
            text_at(b, x, y, 0.20f, kInk);
        }
        text_at("FUEL", 0.0f, 0.36f, 0.13f, kInkDim);

        const double total = std::clamp(r_.fuel_lb, 0.0, 99990.0);
        char tot[16];
        std::snprintf(tot, sizeof(tot), "%05d", static_cast<int>(std::lround(total / 10.0) * 10));
        counter(tot, 0.0f, -0.42f, 0.13f, 0.20f, 0.17f);
        text_at("TOTAL LBS", 0.0f, -0.66f, 0.09f, kInkDim);

        // The fuel model tracks total internal fuel only, so the two tank
        // groups are shown evenly split and the pointers ride together.
        const float half = scale_deg(s, static_cast<float>(total * 0.5));
        needle(half, 0.84f, 0.08f, 0.16f, kPointer);
        needle(half, 0.70f, 0.06f, 0.14f, kInkDim);
        text_at("AL", -0.26f, 0.08f, 0.10f, kInkDim);
        text_at("FR", 0.26f, 0.08f, 0.10f, kInkDim);
        hub(0.09f);
    }

    void draw_fuel_flow() {
        const float a = aspect(Gauge::FUEL_FLOW);
        add_rect(0.0f, 0.0f, a * 2.0f, 2.0f, kRim);
        add_rect(0.0f, 0.0f, a * 2.0f - 0.10f, 1.90f, kFace);
        text_at("FUEL FLOW", 0.0f, 0.62f, 0.26f, kInk);
        char ff[16];
        const int pph = static_cast<int>(std::lround(std::clamp(r_.fuel_flow_pph, 0.0, 99990.0) / 10.0) * 10);
        std::snprintf(ff, sizeof(ff), "%05d", pph);
        counter(ff, 0.0f, -0.02f, 0.40f, 0.62f, 0.48f);
        text_at("PPH", 0.0f, -0.68f, 0.22f, kInkDim);
    }

    // ---- Clock -------------------------------------------------------------
    void draw_clock() {
        dial_face();
        for (int i = 0; i < 60; ++i) {
            const bool hour = (i % 5) == 0;
            tick(static_cast<float>(i) * 6.0f, 0.92f, hour ? 0.14f : 0.06f, hour ? 0.040f : 0.018f, kInk);
        }
        for (int h = 1; h <= 12; ++h) {
            char b[4];
            std::snprintf(b, sizeof(b), "%d", h);
            const auto [x, y] = pol(0.66f, static_cast<float>(h) * 30.0f);
            text_at(b, x, y, 0.20f, kInk);
        }
        const auto [hh, mm, ss] = local_hms();
        const float hour_deg = (static_cast<float>(hh % 12) + static_cast<float>(mm) / 60.0f) * 30.0f;
        const float min_deg = (static_cast<float>(mm) + ss / 60.0f) * 6.0f;
        needle(hour_deg, 0.50f, 0.085f, 0.10f, kPointer);
        needle(min_deg, 0.80f, 0.060f, 0.12f, kPointer);
        needle(static_cast<float>(std::floor(ss)) * 6.0f, 0.86f, 0.020f, 0.20f, kSymbol);
        hub(0.07f);
    }

    /// Wall-clock hours, minutes and (fractional) seconds, local time.
    static std::tuple<int, int, float> local_hms() {
        const auto now = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
        return {tm.tm_hour, tm.tm_min, static_cast<float>(tm.tm_sec) + static_cast<float>(ms) / 1000.0f};
    }

    // ---- DED ---------------------------------------------------------------
    /// Fixed-pitch line of the DED's 24-column display.
    void ded_line(const char* s, float y, float a) {
        constexpr int kCols = 24;
        const float pitch = (a * 2.0f - 0.24f) / kCols;
        const float x0 = -a + 0.12f + pitch * 0.5f;
        const Color4 ink{0.30f, 1.0f, 0.40f, 1.0f};
        for (int i = 0; i < kCols && s[i]; ++i) {
            if (s[i] == ' ') continue;
            const char c[2] = {s[i], '\0'};
            text_at(c, x0 + pitch * static_cast<float>(i), y, 0.34f, ink);
        }
    }

    void draw_ded() {
        const float a = aspect(Gauge::DED);
        add_rect(0.0f, 0.0f, a * 2.0f, 2.0f, Color4{0.005f, 0.035f, 0.010f, 1.0f});
        // CNI page: comm, steerpoint, the clock and the IFF / TACAN line.
        const auto [hh, mm, ss] = local_hms();
        char l3[32];
        std::snprintf(l3, sizeof(l3), "VHF  1        %02d:%02d:%02d", hh, mm, static_cast<int>(ss));
        ded_line("UHF 292.30      STPT   1", 0.70f, a);
        ded_line(l3, 0.0f, a);
        ded_line("M1 3 C 3400  MAN   T 75X", -0.70f, a);
    }

    // ---- Lamps -------------------------------------------------------------
    /// Split-legend warning lamp: each half lights on its own.
    void draw_lamp(const char* top, bool top_on, const char* bottom, bool bottom_on) {
        add_rect(0.0f, 0.0f, 2.0f, 2.0f, Color4{0.010f, 0.010f, 0.010f, 1.0f});
        const Color4 lens_off{0.022f, 0.018f, 0.018f, 1.0f};
        const Color4 lens_on{0.42f, 0.05f, 0.03f, 1.0f};
        const Color4 ink_off{0.085f, 0.050f, 0.045f, 1.0f};
        const Color4 ink_on{1.0f, 0.62f, 0.50f, 1.0f};
        add_rect(0.0f, 0.46f, 1.84f, 0.84f, top_on ? lens_on : lens_off);
        add_rect(0.0f, -0.46f, 1.84f, 0.84f, bottom_on ? lens_on : lens_off);
        // Legends shrink to fit the lens width.
        const float size_top = std::min(0.40f, 1.70f / std::max(0.1f, text_width(top, 1.0f)));
        const float size_b = std::min(0.40f, 1.70f / std::max(0.1f, text_width(bottom, 1.0f)));
        text_at(top, 0.0f, 0.46f, size_top, top_on ? ink_on : ink_off);
        text_at(bottom, 0.0f, -0.46f, size_b, bottom_on ? ink_on : ink_off);
    }

    // ---- RWR ---------------------------------------------------------------
    /// Threat warning azimuth indicator: a green CRT seen from above, own jet
    /// at the centre, nose up. Emitters sit at their bearing; the more lethal
    /// a threat, the nearer the centre it is drawn, as on the real display.
    /// A locking radar is boxed in the priority diamond. An inbound missile
    /// is an "M" in a flashing diamond. The scope is blank unless the combat
    /// systems are running.
    void draw_rwr() {
        const Color4 glass{0.004f, 0.020f, 0.008f, 1.0f};
        const Color4 grid{0.08f, 0.32f, 0.13f, 1.0f};
        const Color4 ink{0.40f, 1.0f, 0.50f, 1.0f};
        const Color4 red{1.0f, 0.30f, 0.20f, 1.0f};
        add_circle(0.0f, 0.0f, 1.0f, kRim, 64);
        add_circle(0.0f, 0.0f, 0.94f, glass, 64);
        if (!r_.rwr_on) return;
        add_ring(0.0f, 0.0f, 0.86f, 0.89f, grid, 64);
        add_ring(0.0f, 0.0f, 0.44f, 0.46f, grid, 48);
        for (int q = 0; q < 12; ++q) tick(static_cast<float>(q) * 30.0f, 0.89f, q % 3 == 0 ? 0.12f : 0.06f, 0.025f, grid);
        // Own jet
        add_bar(0.0f, -0.10f, 0.0f, 0.12f, 0.035f, ink);
        add_bar(-0.10f, 0.02f, 0.10f, 0.02f, 0.035f, ink);
        add_bar(-0.05f, -0.09f, 0.05f, -0.09f, 0.03f, ink);

        auto diamond = [&](float x, float y, float h, const Color4& col) {
            add_bar(x, y + h, x + h, y, 0.03f, col);
            add_bar(x + h, y, x, y - h, 0.03f, col);
            add_bar(x, y - h, x - h, y, 0.03f, col);
            add_bar(x - h, y, x, y + h, 0.03f, col);
        };
        if (r_.rwr_emitter) {
            const float rr = r_.rwr_level >= 2 ? 0.55f : 0.72f;
            const auto [x, y] = pol(rr, static_cast<float>(r_.rwr_emitter_deg));
            text_at(r_.rwr_symbol, x, y, 0.26f, ink);
            if (r_.rwr_level >= 2 && (r_.rwr_level < 3 || r_.blink)) diamond(x, y, 0.20f, ink);
        }
        if (r_.rwr_missile) {
            const auto [x, y] = pol(0.30f, static_cast<float>(r_.rwr_missile_deg));
            text_at("M", x, y, 0.24f, red);
            if (r_.blink) diamond(x, y, 0.17f, red);
        }
    }

    void draw_gear_lamp() {
        if (r_.gear_green) {
            add_circle(0.0f, 0.0f, 0.98f, Color4{0.10f, 0.65f, 0.20f, 1.0f}, 32);
            add_circle(0.0f, 0.0f, 0.62f, Color4{0.45f, 1.0f, 0.50f, 1.0f}, 24);
        } else {
            add_circle(0.0f, 0.0f, 0.98f, Color4{0.07f, 0.09f, 0.075f, 1.0f}, 32);
            add_circle(0.0f, 0.0f, 0.62f, Color4{0.10f, 0.13f, 0.11f, 1.0f}, 24);
        }
    }

    // ---- Typhoon glareshield -----------------------------------------------
    // Key and caption legends are lit in NVG-compatible pale green; the
    // keys themselves are matt black with a lighter rim.
    static constexpr Color4 kKeyBody{0.018f, 0.019f, 0.020f, 1.0f};
    static constexpr Color4 kKeyRim{0.060f, 0.062f, 0.066f, 1.0f};
    static constexpr Color4 kKeyInk{0.70f, 0.92f, 0.70f, 1.0f};
    static constexpr Color4 kPlate{0.010f, 0.010f, 0.011f, 1.0f};

    /// A push key: rim, body and a legend shrunk to fit.
    void key(float x, float y, float w, float h, const char* legend, const Color4& ink = kKeyInk) {
        add_rect(x, y, w, h, kKeyRim);
        add_rect(x, y, w - 0.035f, h - 0.035f, kKeyBody);
        if (legend && *legend) {
            const float size = std::min(h * 0.50f, (w - 0.07f) / std::max(0.1f, text_width(legend, 1.0f)));
            text_at(legend, x, y, size, ink);
        }
    }

    /// Fixed-pitch line in a display window from x0 to x1.
    void mono_line(const char* s, float x0, float x1, int cols, float y, float size, const Color4& ink) {
        const float pitch = (x1 - x0) / static_cast<float>(cols);
        for (int i = 0; i < cols && s[i]; ++i) {
            if (s[i] == ' ') continue;
            const char c[2] = {s[i], '\0'};
            text_at(c, x0 + pitch * (static_cast<float>(i) + 0.5f), y, size, ink);
        }
    }

    /// An attention getter: a large push-to-cancel lamp that flashes while
    /// its warning or caution is new.
    void attention_getter(float x, float y, float w, float h, const char* legend, bool on, const Color4& lit) {
        add_rect(x, y, w, h, kKeyRim);
        const bool show = on && std::fmod(flash_t_, 0.5) < 0.30;
        const Color4 lens = show ? Color4{lit.r * 0.55f, lit.g * 0.55f, lit.b * 0.55f, 1.0f}
                                 : Color4{0.030f, 0.022f, 0.020f, 1.0f};
        add_rect(x, y, w - 0.05f, h - 0.05f, lens);
        const Color4 ink = show ? Color4{1.0f, 0.95f, 0.85f, 1.0f} : Color4{0.16f, 0.12f, 0.10f, 1.0f};
        const float size = std::min(h * 0.42f, (w - 0.12f) / std::max(0.1f, text_width(legend, 1.0f)));
        text_at(legend, x, y, size, ink);
    }

    /// Manual Data Entry Facility, left glareshield. Outboard, the
    /// subsystem keys that call up a subsystem's page on the MDEF display
    /// (SWP set apart); in the middle the display with its line keys; inboard
    /// the data entry keypad, under the master caution attention getter.
    void draw_mdef() {
        const float a = aspect(Gauge::MDEF);
        add_rect(0.0f, 0.0f, a * 2.0f, 2.0f, kPlate);

        // Subsystem keys, two columns of six, then SWP on its own.
        static constexpr const char* kSub[12] = {"AP",  "NAV",  "AIDS", "INT", "XPDR", "XMIT",
                                                 "RAD1", "RAD2", "DAS", "MISC", "MIDS", "NIS"};
        for (int i = 0; i < 12; ++i) {
            const float x = -a + 0.22f + 0.40f * static_cast<float>(i / 6);
            const float y = 0.80f - 0.27f * static_cast<float>(i % 6);
            key(x, y, 0.36f, 0.22f, kSub[i]);
        }
        key(-a + 0.42f, -0.84f, 0.46f, 0.22f, "SWP", Color4{0.95f, 0.80f, 0.40f, 1.0f});

        // MDEF display with four line keys under it.
        const float dx0 = -a + 0.92f, dx1 = 0.66f;
        add_rect(0.5f * (dx0 + dx1), 0.30f, dx1 - dx0 + 0.06f, 1.20f, kKeyRim);
        add_rect(0.5f * (dx0 + dx1), 0.30f, dx1 - dx0, 1.14f, Color4{0.004f, 0.020f, 0.008f, 1.0f});
        const Color4 ink{0.35f, 1.0f, 0.45f, 1.0f};
        const auto [hh, mm, ss] = local_hms();
        char l4[32];
        std::snprintf(l4, sizeof(l4), "XPDR 3400  %02d:%02d:%02d", hh, mm, static_cast<int>(ss));
        mono_line("RAD1 251.300  PRE 04", dx0 + 0.05f, dx1 - 0.05f, 20, 0.73f, 0.15f, ink);
        mono_line("RAD2 243.000  GRD   ", dx0 + 0.05f, dx1 - 0.05f, 20, 0.45f, 0.15f, ink);
        mono_line("NAV  WP 01  TCN 075X", dx0 + 0.05f, dx1 - 0.05f, 20, 0.17f, 0.15f, ink);
        mono_line(l4, dx0 + 0.05f, dx1 - 0.05f, 20, -0.11f, 0.15f, ink);
        for (int i = 0; i < 4; ++i) key(dx0 + 0.20f + 0.36f * static_cast<float>(i) * (dx1 - dx0 - 0.40f) / 1.08f,
                                        -0.55f, 0.30f, 0.20f, "");
        key(0.5f * (dx0 + dx1), -0.84f, dx1 - dx0, 0.20f, "SCRATCH", Color4{0.55f, 0.65f, 0.55f, 1.0f});

        // Data entry keypad, under the master caution.
        attention_getter(1.32f, 0.78f, 1.18f, 0.36f, "CAUTION", r_.caution, Color4{1.0f, 0.62f, 0.05f, 1.0f});
        static constexpr const char* kPad[12] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "CLR", "0", "ENT"};
        for (int i = 0; i < 12; ++i) {
            const float x = 0.94f + 0.38f * static_cast<float>(i % 3);
            const float y = 0.38f - 0.37f * static_cast<float>(i / 3);
            key(x, y, 0.34f, 0.31f, kPad[i]);
        }
    }

    /// Dedicated Warnings Panel, right glareshield: the master warning
    /// attention getter inboard, then the caption block, red warnings on
    /// top, amber cautions below, green advisories at the bottom. The
    /// captions are representative; the jet's full list is longer.
    void draw_dwp() {
        const float a = aspect(Gauge::DWP);
        add_rect(0.0f, 0.0f, a * 2.0f, 2.0f, kPlate);

        const bool red_any = r_.eng_fire || r_.engine || r_.flcs || r_.structure || r_.hyd_oil || r_.gcas;
        attention_getter(-a + 0.50f, 0.66f, 0.86f, 0.60f, "WARNING", red_any, Color4{1.0f, 0.10f, 0.06f, 1.0f});
        key(-a + 0.50f, 0.04f, 0.86f, 0.36f, "RESET");
        key(-a + 0.50f, -0.40f, 0.86f, 0.36f, "LAMP TEST");
        key(-a + 0.50f, -0.82f, 0.86f, 0.30f, "DIM");

        struct Caption {
            const char* text;
            bool on;
            int level; // 0 red, 1 amber, 2 green
        };
        const Caption cap[24] = {
            {"L FIRE", r_.eng_fire, 0}, {"R FIRE", r_.eng_fire, 0}, {"FCS", r_.flcs || r_.structure, 0}, {"HYD", r_.hyd_oil, 0},
            {"L ENG", r_.engine, 0},    {"R ENG", r_.engine, 0},    {"GCAS", r_.gcas, 0},             {"OXY", false, 0},
            {"BINGO", r_.bingo, 1},     {"GEAR", r_.config, 1},     {"OVER G", r_.over_g, 1},         {"AOA", r_.high_aoa, 1},
            {"L GEN", false, 1},        {"R GEN", false, 1},        {"ECS", false, 1},                {"LOW ALT", r_.low_alt, 1},
            {"L PUMP", false, 1},       {"R PUMP", false, 1},       {"PITOT", false, 1},              {"CANOPY", false, 1},
            {"AIRBRK", r_.speedbrake > 0.02, 2}, {"A/P", false, 2}, {"MIDS", false, 2},               {"DASS", false, 2},
        };
        const Color4 lit_ink[3] = {{1.0f, 0.22f, 0.15f, 1.0f}, {1.0f, 0.70f, 0.12f, 1.0f}, {0.35f, 1.0f, 0.42f, 1.0f}};
        const Color4 lit_lens[3] = {{0.20f, 0.025f, 0.015f, 1.0f}, {0.20f, 0.12f, 0.010f, 1.0f}, {0.02f, 0.14f, 0.04f, 1.0f}};
        const float x0 = -a + 1.06f, cw = (a - 0.04f - x0) / 4.0f, ch = 1.92f / 6.0f;
        for (int i = 0; i < 24; ++i) {
            const float x = x0 + cw * (static_cast<float>(i % 4) + 0.5f);
            const float y = 0.96f - ch * (static_cast<float>(i / 4) + 0.5f);
            const Caption& c = cap[i];
            add_rect(x, y, cw - 0.03f, ch - 0.03f, kKeyRim);
            add_rect(x, y, cw - 0.07f, ch - 0.07f, c.on ? lit_lens[c.level] : Color4{0.016f, 0.016f, 0.017f, 1.0f});
            const float size = std::min(0.15f, (cw - 0.14f) / std::max(0.1f, text_width(c.text, 1.0f)));
            text_at(c.text, x, y, size, c.on ? lit_ink[c.level] : Color4{0.13f, 0.13f, 0.12f, 1.0f});
        }
    }

    /// Get-U-Home display, under the right glareshield: the reversionary
    /// flight instrument. A small attitude sphere, airspeed left, altitude
    /// right, Mach and AoA along the top, heading and climb rate below. The
    /// F-35's integrated standby flight display, under its PCD, shows the
    /// same; @p label names which it is.
    void draw_guh(const fdm::FlightState& state, const char* label) {
        add_rect(0.0f, 0.0f, 2.0f, 2.0f, Color4{0.012f, 0.012f, 0.013f, 1.0f});
        add_rect(0.0f, 0.0f, 1.92f, 1.92f, Color4{0.0f, 0.0f, 0.0f, 1.0f});
        const Color4 ink{0.92f, 0.94f, 0.92f, 1.0f};
        const Color4 dim{0.55f, 0.58f, 0.55f, 1.0f};

        // Attitude sphere, clipped to a circle.
        constexpr float R = 0.48f, CY = 0.04f;
        const float pitch = static_cast<float>(state.pitch() * 180.0 / 3.14159265358979);
        const float roll = static_cast<float>(state.roll() * 180.0 / 3.14159265358979);
        const float k = R / 40.0f;
        auto P = [&](float x, float y) {
            const auto [rx, ry] = rot(x, y, -roll);
            return std::pair<float, float>{rx, ry + CY};
        };
        add_circle(0.0f, CY, R, Color4{0.36f, 0.24f, 0.13f, 1.0f}, 48);
        const float yh = -pitch * k;
        if (yh < R) {
            std::vector<float> poly;
            constexpr int N = 64;
            for (int i = 0; i < N; ++i) {
                const float a0 = 2.0f * kPi * static_cast<float>(i) / N, a1 = 2.0f * kPi * static_cast<float>(i + 1) / N;
                const float x0 = R * std::cos(a0), y0 = R * std::sin(a0), x1 = R * std::cos(a1), y1 = R * std::sin(a1);
                const bool in0 = y0 > yh, in1 = y1 > yh;
                if (in0) {
                    const auto [px, py] = P(x0, y0);
                    poly.push_back(px); poly.push_back(py);
                }
                if (in0 != in1) {
                    const float t = (yh - y0) / (y1 - y0);
                    const auto [px, py] = P(x0 + t * (x1 - x0), yh);
                    poly.push_back(px); poly.push_back(py);
                }
            }
            if (poly.size() >= 6) add_convex(poly, Color4{0.18f, 0.45f, 0.80f, 1.0f});
        }
        if (std::abs(yh) < R) {
            const float hx = std::sqrt(R * R - yh * yh);
            const auto [ax, ay] = P(-hx, yh);
            const auto [bx, by] = P(hx, yh);
            add_bar(ax, ay, bx, by, 0.022f, ink);
        }
        for (int p = -30; p <= 30; p += 10) {
            if (p == 0) continue;
            const float y = (static_cast<float>(p) - pitch) * k;
            if (std::abs(y) > R * 0.85f) continue;
            const auto [ax, ay] = P(-0.12f, y);
            const auto [bx, by] = P(0.12f, y);
            add_bar(ax, ay, bx, by, 0.014f, ink);
        }
        add_ring(0.0f, CY, R, R + 0.03f, dim, 48);
        // Fixed aircraft symbol.
        const Color4 sym{1.0f, 0.85f, 0.10f, 1.0f};
        add_bar(-0.30f, CY, -0.10f, CY, 0.035f, sym);
        add_bar(0.10f, CY, 0.30f, CY, 0.035f, sym);
        add_rect(0.0f, CY, 0.05f, 0.05f, sym);

        // Airspeed and altitude windows.
        char buf[24];
        auto window = [&](float x, const char* label, const char* value) {
            add_rect(x, CY, 0.40f, 0.22f, dim);
            add_rect(x, CY, 0.37f, 0.19f, Color4{0.0f, 0.0f, 0.0f, 1.0f});
            text_at(value, x, CY, std::min(0.15f, 0.34f / std::max(0.1f, text_width(value, 1.0f))), ink);
            text_at(label, x, CY + 0.21f, 0.13f, dim);
        };
        std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(std::clamp(r_.kcas, 0.0, 999.0))));
        window(-0.75f, "IAS", buf);
        std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(std::clamp(r_.alt_ft, -999.0, 99999.0) / 10.0) * 10));
        window(0.75f, "ALT", buf);

        std::snprintf(buf, sizeof(buf), "M%.2f", std::clamp(r_.mach, 0.0, 9.99));
        text_at(buf, -0.55f, 0.82f, 0.13f, ink);
        std::snprintf(buf, sizeof(buf), "AOA %.0f", std::clamp(r_.aoa_deg, -9.0, 99.0));
        text_at(buf, 0.55f, 0.82f, 0.13f, ink);
        std::snprintf(buf, sizeof(buf), "%03d", static_cast<int>(std::lround(r_.heading_deg)) % 360);
        add_rect(0.0f, -0.72f, 0.34f, 0.20f, dim);
        add_rect(0.0f, -0.72f, 0.31f, 0.17f, Color4{0.0f, 0.0f, 0.0f, 1.0f});
        text_at(buf, 0.0f, -0.72f, 0.14f, ink);
        std::snprintf(buf, sizeof(buf), "%+.1f", std::clamp(r_.vvi_fpm / 1000.0, -99.0, 99.0));
        text_at("VS", -0.70f, -0.60f, 0.12f, dim);
        text_at(buf, -0.70f, -0.80f, 0.13f, ink);
        text_at(label, 0.70f, -0.72f, 0.12f, dim);
    }

    double flash_t_ = 0.0; ///< Attention getter flash clock [s]
};

} // namespace fastjet::graphics
