/**
 * @file test_aircraft_specs.cpp
 * @brief Rigorous verification of aeronautical engineering statistics and properties
 * for all five supported aircraft: F-16C, F-15EX, Eurofighter Typhoon, F-22A, and A-10C.
 */
#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/fdm/mass_properties.hpp"
#include "fastjet/fdm/fuel_system.hpp"
#include "fastjet/propulsion/multi_engine.hpp"
#include "fastjet/aero/aircraft_aero_model.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace fastjet;
using namespace fastjet::aircraft;

namespace {

void test_f16c_specifications() {
    std::cout << "[TEST] Validating F-16C Block 50 specifications...\n";
    const auto cfg = AircraftConfig::get(AircraftType::F16_FIGHTING_FALCON);

    // Empty mass 19,700 lbs (USAF fact sheet)
    assert(std::abs(cfg.mass.empty_mass_kg - 8935.8) < 1.0);
    // Fuel capacity ~ 7,000 lbs
    assert(std::abs(cfg.mass.internal_fuel_capacity_kg - 3175.1) < 1.0);
    // Single F110 engine
    assert(cfg.propulsion.engine_count == 1);
    assert(cfg.propulsion.has_afterburner == true);
    assert(cfg.propulsion.static_dry_thrust_n == 75600.0);
    assert(cfg.propulsion.static_ab_thrust_n == 129000.0);

    // Geometry: S_ref = 300 sq ft, b = 30 ft, c_bar = 11.32 ft
    assert(std::abs(cfg.aero.s_ref - 27.87) < 0.1);
    assert(std::abs(cfg.aero.b_span - 9.144) < 0.01);

    // MassProperties factory test
    const auto mass = fdm::MassProperties::create(AircraftType::F16_FIGHTING_FALCON);
    assert(std::abs(mass.mass_kg - 8935.8) < 1.0);
    assert(mass.Ixx > 10000.0 && mass.Ixx < 15000.0);
    assert(mass.Iyy > 70000.0 && mass.Iyy < 80000.0);

    // Fuel system scaling test
    const auto gross_mass = fdm::FuelSystem::compute(3175.1, AircraftType::F16_FIGHTING_FALCON);
    assert(std::abs(gross_mass.mass_kg - (8935.8 + 3175.1)) < 1.0);
    assert(gross_mass.Ixx > mass.Ixx); // Inertia scales with gross mass
    std::cout << "  -> F-16C specs passed.\n";
}

void test_f15ex_specifications() {
    std::cout << "[TEST] Validating F-15EX Eagle II specifications...\n";
    const auto cfg = AircraftConfig::get(AircraftType::F15EX_EAGLE_II);

    // 31,700 lb empty mass (~14,379 kg), 81,000 lb MTOW (~36,741 kg)
    assert(std::abs(cfg.mass.empty_mass_kg - 14379.0) < 1.0);
    assert(std::abs(cfg.mass.internal_fuel_capacity_kg - 6146.0) < 1.0);
    assert(std::abs(cfg.mass.mtow_kg - 36741.0) < 1.0);

    // Twin F110 engines: 34,000 lbf dry (151.2 kN), 59,000 lbf AB (262.4 kN)
    assert(cfg.propulsion.engine_count == 2);
    assert(cfg.propulsion.static_dry_thrust_n == 151200.0);
    assert(cfg.propulsion.static_ab_thrust_n == 262400.0);

    // S_ref = 608 sq ft (~56.49 m^2), b = 42.8 ft (~13.05 m), c_bar = 15.94 ft (~4.86 m)
    assert(std::abs(cfg.aero.s_ref - 56.49) < 0.1);
    assert(std::abs(cfg.aero.b_span - 13.045) < 0.01);
    assert(std::abs(cfg.aero.c_bar - 4.858) < 0.01);

    // Digital Fly-By-Wire system
    assert(cfg.flcs.law_type == FLCSConfig::LawType::DIGITAL_FBW_G_ALPHA);
    assert(cfg.flcs.max_g_positive == 9.0);
    assert(cfg.flcs.alpha_limit_deg == 30.0);

    const auto mass = fdm::MassProperties::create(AircraftType::F15EX_EAGLE_II);
    assert(std::abs(mass.mass_kg - 14379.0) < 1.0);
    assert(mass.Iyy > 250000.0); // F-15 is longer and has substantially higher pitch inertia
    std::cout << "  -> F-15EX specs passed.\n";
}

void test_typhoon_specifications() {
    std::cout << "[TEST] Validating Eurofighter Typhoon specifications...\n";
    const auto cfg = AircraftConfig::get(AircraftType::EUROFIGHTER_TYPHOON);

    // 11,000 kg empty, 4,998 kg internal fuel
    assert(std::abs(cfg.mass.empty_mass_kg - 11000.0) < 1.0);
    assert(std::abs(cfg.mass.internal_fuel_capacity_kg - 4998.0) < 1.0);

    // Twin EJ200: 120 kN dry, 180 kN AB. Dry supercruise at Mach 1.50
    assert(cfg.propulsion.engine_count == 2);
    assert(cfg.propulsion.static_dry_thrust_n == 120000.0);
    assert(cfg.propulsion.static_ab_thrust_n == 180000.0);
    assert(cfg.propulsion.has_supercruise == true);
    assert(cfg.propulsion.supercruise_mach == 1.50);

    // Delta-canard geometry: S_ref = 51.2 m^2, b = 10.95 m
    assert(std::abs(cfg.aero.s_ref - 51.20) < 0.1);
    assert(std::abs(cfg.aero.b_span - 10.95) < 0.01);
    assert(cfg.aero.sweep_le_deg == 53.0);

    // Carefree Handling FBW
    assert(cfg.flcs.law_type == FLCSConfig::LawType::CAREFREE_DELTA_CANARD);
    assert(cfg.flcs.alpha_limit_deg == 35.0);
    std::cout << "  -> Eurofighter Typhoon specs passed.\n";
}

void test_f22_specifications() {
    std::cout << "[TEST] Validating F-22A Raptor specifications...\n";
    const auto cfg = AircraftConfig::get(AircraftType::F22_RAPTOR);

    // 43,340 lb empty (~19,659 kg), 18,000 lb fuel (~8,165 kg)
    assert(std::abs(cfg.mass.empty_mass_kg - 19659.0) < 1.0);
    assert(std::abs(cfg.mass.internal_fuel_capacity_kg - 8165.0) < 1.0);

    // Twin F119: 232 kN dry, 312 kN AB. Supercruise at Mach 1.82
    assert(cfg.propulsion.engine_count == 2);
    assert(cfg.propulsion.static_dry_thrust_n == 232000.0);
    assert(cfg.propulsion.static_ab_thrust_n == 312000.0);
    assert(cfg.propulsion.has_supercruise == true);
    assert(cfg.propulsion.supercruise_mach == 1.82);

    // 2D Thrust Vectoring
    assert(cfg.propulsion.has_thrust_vectoring == true);
    assert(cfg.propulsion.tvc_max_deflection_deg == 20.0);
    assert(cfg.propulsion.tvc_moment_arm_m == 6.0);

    // S_ref = 840 sq ft (78.04 m^2), wingspan = 44.5 ft (13.56 m)
    assert(std::abs(cfg.aero.s_ref - 78.04) < 0.1);
    assert(std::abs(cfg.aero.b_span - 13.564) < 0.01);

    // Post-stall AoA capability
    assert(cfg.flcs.law_type == FLCSConfig::LawType::FBW_TVC_ALLOCATED);
    assert(cfg.flcs.alpha_limit_deg == 65.0);
    assert(cfg.flcs.max_g_positive == 9.0);
    std::cout << "  -> F-22A Raptor specs passed.\n";
}

void test_a10_specifications() {
    std::cout << "[TEST] Validating A-10C Thunderbolt II specifications...\n";
    const auto cfg = AircraftConfig::get(AircraftType::A10_THUNDERBOLT);

    // 24,959 lb empty (~11,321 kg), 11,000 lb fuel (~4,990 kg)
    assert(std::abs(cfg.mass.empty_mass_kg - 11321.0) < 1.0);
    assert(std::abs(cfg.mass.internal_fuel_capacity_kg - 4990.0) < 1.0);

    // Twin TF34-GE-100A: 80.6 kN total dry, NO AFTERBURNER!
    assert(cfg.propulsion.engine_count == 2);
    assert(cfg.propulsion.has_afterburner == false);
    assert(cfg.propulsion.static_dry_thrust_n == 80600.0);
    assert(cfg.propulsion.static_ab_thrust_n == 80600.0);
    // High-mount engine pitch-thrust coupling: Z = -1.2m
    assert(cfg.propulsion.thrust_z_offset_m == -1.2);
    // High-bypass TSFC
    assert(cfg.propulsion.tsfc_dry < 1.5e-5);

    // Straight wing: AR = 6.54, unswept (0.0 deg)
    assert(cfg.aero.aspect_ratio == 6.54);
    assert(cfg.aero.sweep_le_deg == 0.0);
    assert(cfg.aero.mach_crit == 0.65); // Subsonic speed barrier

    // Hydromechanical controls + SAS
    assert(cfg.flcs.law_type == FLCSConfig::LawType::HYDRO_SAS_AUGMENTED);
    assert(cfg.flcs.max_g_positive == 7.33);

    // Wide gear stance with 0.3m offset nosewheel for GAU-8 Avenger
    assert(cfg.gear.nose_pos_b.y == 0.30);
    assert(std::abs(cfg.gear.main_r_pos_b.y - 2.62) < 0.01);
    std::cout << "  -> A-10C specs passed.\n";
}

void test_f35b_specifications() {
    std::cout << "[TEST] Validating F-35B Lightning II specifications...\n";
    const auto cfg = AircraftConfig::get(AircraftType::F35B_LIGHTNING_II);

    // 32,472 lb empty (14,729 kg), 13,100 lb internal fuel (5,942 kg)
    assert(std::abs(cfg.mass.empty_mass_kg - 14729.0) < 1.0);
    assert(std::abs(cfg.mass.internal_fuel_capacity_kg - 5942.0) < 1.0);

    // F135-PW-600: 27,000 lbf mil, 41,000 lbf class reheat
    assert(cfg.propulsion.engine_count == 1);
    assert(cfg.propulsion.has_afterburner);
    assert(std::abs(cfg.propulsion.static_dry_thrust_n / 4.44822 - 27000.0) < 100.0);
    assert(std::abs(cfg.propulsion.static_ab_thrust_n / 4.44822 - 41000.0) < 100.0);

    // 460 sq ft, 35 ft span
    assert(std::abs(cfg.aero.s_ref - 42.7) < 0.1);
    assert(std::abs(cfg.aero.b_span - 10.67) < 0.01);

    // 7 G, 50 deg AoA, unified STOVL law
    assert(cfg.flcs.law_type == FLCSConfig::LawType::UNIFIED_STOVL);
    assert(cfg.flcs.max_g_positive == 7.0);
    assert(cfg.flcs.alpha_limit_deg == 50.0);

    // Lift system: 40,650 lbf = 18,680 + 18,680 + 2 x 1,645 (P&W)
    assert(cfg.stovl.equipped);
    assert(std::abs(cfg.stovl.lift_fan_thrust_n / 4.44822 - 18680.0) < 10.0);
    assert(std::abs(cfg.stovl.nozzle_thrust_n / 4.44822 - 18680.0) < 10.0);
    assert(std::abs(cfg.stovl.roll_post_thrust_n / 4.44822 - 3290.0) < 10.0);
    assert(cfg.stovl.nozzle_max_deg == 95.0);
    assert(cfg.stovl.max_conversion_kcas == 250.0);
    // Fan forward of the CG, 3BSM aft, roll posts out under the wings.
    assert(cfg.stovl.lift_fan_pos_b.x > 3.0 && cfg.stovl.nozzle_pos_b.x < -3.0);
    assert(cfg.stovl.roll_post_pos_b.y > 2.0);
    // Only the F-35B carries one.
    assert(!AircraftConfig::get(AircraftType::F22_RAPTOR).stovl.equipped);
    std::cout << "  -> F-35B specs passed.\n";
}

} // namespace

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Fast Jet Multi-Aircraft Specification Verification Test \n";
    std::cout << "=========================================================\n";

    test_f16c_specifications();
    test_f15ex_specifications();
    test_typhoon_specifications();
    test_f22_specifications();
    test_a10_specifications();
    test_f35b_specifications();

    std::cout << "\nAll aircraft specifications successfully verified!\n";
    return 0;
}
