CXX ?= g++
CXXFLAGS ?= -std=c++20 -O3 -Wall -Wextra -Wpedantic -march=native -pthread -Iinclude
LDFLAGS ?= -lSDL3 -lepoxy -lGL -pthread
BIN_DIR = bin
SRC_TEST_DIR = tests

TESTS = test_atmosphere test_ballistic test_gyroscopic test_performance \
        test_interpolator test_aero_coefficients test_pitch_divergence \
        test_actuators test_flcs_level_flight test_flcs_pullup test_flcs_review \
        test_signal_conditioning test_force_curves test_avionics_controls \
        test_config_persistence test_input_pipeline \
        test_camera_rig test_hud_collimation test_hud_clipping test_flight_instruments \
        test_render_pipeline test_propulsion test_landing_gear test_compressibility test_integrated_flight test_throttle_control test_performance_envelope \
        test_terrain_field test_terrain_render test_ground_collision test_ofc_handover test_ofc_closed_loop \
        test_aircraft_specs test_multi_aircraft_performance test_model_glb \
        test_user_settings test_menu_navigation test_render_fidelity test_thread_pool \
        test_aircraft_entity test_guns test_air_combat_geometry test_bfm_ai \
        test_missiles test_evade_mode test_radar_scope test_flight_realism \
        test_ejection test_jet_cockpits test_stovl test_missile_warning test_air_combat

TARGETS = $(addprefix $(BIN_DIR)/, $(TESTS))

.PHONY: all clean run_all viewer $(TESTS)

all: $(BIN_DIR) $(TARGETS) $(BIN_DIR)/f16_sim_viewer

HEADERS = $(shell find include -name '*.hpp')

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(BIN_DIR)/%: $(SRC_TEST_DIR)/%.cpp $(HEADERS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

$(BIN_DIR)/f16_sim_viewer: src/f16_sim_viewer.cpp $(HEADERS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

viewer: $(BIN_DIR)/f16_sim_viewer

test_atmosphere: $(BIN_DIR)/test_atmosphere
	./$(BIN_DIR)/test_atmosphere

test_ballistic: $(BIN_DIR)/test_ballistic
	./$(BIN_DIR)/test_ballistic

test_gyroscopic: $(BIN_DIR)/test_gyroscopic
	./$(BIN_DIR)/test_gyroscopic

test_performance: $(BIN_DIR)/test_performance
	./$(BIN_DIR)/test_performance

test_interpolator: $(BIN_DIR)/test_interpolator
	./$(BIN_DIR)/test_interpolator

test_aero_coefficients: $(BIN_DIR)/test_aero_coefficients
	./$(BIN_DIR)/test_aero_coefficients

test_pitch_divergence: $(BIN_DIR)/test_pitch_divergence
	./$(BIN_DIR)/test_pitch_divergence

test_actuators: $(BIN_DIR)/test_actuators
	./$(BIN_DIR)/test_actuators

test_flcs_level_flight: $(BIN_DIR)/test_flcs_level_flight
	./$(BIN_DIR)/test_flcs_level_flight

test_flcs_pullup: $(BIN_DIR)/test_flcs_pullup
	./$(BIN_DIR)/test_flcs_pullup

test_flcs_review: $(BIN_DIR)/test_flcs_review
	./$(BIN_DIR)/test_flcs_review

test_signal_conditioning: $(BIN_DIR)/test_signal_conditioning
	./$(BIN_DIR)/test_signal_conditioning

test_force_curves: $(BIN_DIR)/test_force_curves
	./$(BIN_DIR)/test_force_curves

test_avionics_controls: $(BIN_DIR)/test_avionics_controls
	./$(BIN_DIR)/test_avionics_controls

test_config_persistence: $(BIN_DIR)/test_config_persistence
	./$(BIN_DIR)/test_config_persistence

test_input_pipeline: $(BIN_DIR)/test_input_pipeline
	./$(BIN_DIR)/test_input_pipeline

test_camera_rig: $(BIN_DIR)/test_camera_rig
	./$(BIN_DIR)/test_camera_rig

test_hud_collimation: $(BIN_DIR)/test_hud_collimation
	./$(BIN_DIR)/test_hud_collimation

test_hud_clipping: $(BIN_DIR)/test_hud_clipping
	./$(BIN_DIR)/test_hud_clipping

test_flight_instruments: $(BIN_DIR)/test_flight_instruments
	./$(BIN_DIR)/test_flight_instruments

test_render_pipeline: $(BIN_DIR)/test_render_pipeline
	./$(BIN_DIR)/test_render_pipeline

test_terrain_field: $(BIN_DIR)/test_terrain_field
	./$(BIN_DIR)/test_terrain_field

test_terrain_render: $(BIN_DIR)/test_terrain_render
	./$(BIN_DIR)/test_terrain_render

test_ground_collision: $(BIN_DIR)/test_ground_collision
	./$(BIN_DIR)/test_ground_collision

test_ofc_handover: $(BIN_DIR)/test_ofc_handover
	./$(BIN_DIR)/test_ofc_handover

test_ofc_closed_loop: $(BIN_DIR)/test_ofc_closed_loop
	./$(BIN_DIR)/test_ofc_closed_loop

test_aircraft_specs: $(BIN_DIR)/test_aircraft_specs
	./$(BIN_DIR)/test_aircraft_specs

test_multi_aircraft_performance: $(BIN_DIR)/test_multi_aircraft_performance
	./$(BIN_DIR)/test_multi_aircraft_performance

test_model_glb: $(BIN_DIR)/test_model_glb
	./$(BIN_DIR)/test_model_glb

test_user_settings: $(BIN_DIR)/test_user_settings
	./$(BIN_DIR)/test_user_settings

test_menu_navigation: $(BIN_DIR)/test_menu_navigation
	./$(BIN_DIR)/test_menu_navigation

test_render_fidelity: $(BIN_DIR)/test_render_fidelity
	./$(BIN_DIR)/test_render_fidelity

test_thread_pool: $(BIN_DIR)/test_thread_pool
	./$(BIN_DIR)/test_thread_pool

test_aircraft_entity: $(BIN_DIR)/test_aircraft_entity
	./$(BIN_DIR)/test_aircraft_entity

test_guns: $(BIN_DIR)/test_guns
	./$(BIN_DIR)/test_guns

test_air_combat_geometry: $(BIN_DIR)/test_air_combat_geometry
	./$(BIN_DIR)/test_air_combat_geometry

test_bfm_ai: $(BIN_DIR)/test_bfm_ai
	./$(BIN_DIR)/test_bfm_ai

test_missiles: $(BIN_DIR)/test_missiles
	./$(BIN_DIR)/test_missiles

test_evade_mode: $(BIN_DIR)/test_evade_mode
	./$(BIN_DIR)/test_evade_mode

test_radar_scope: $(BIN_DIR)/test_radar_scope
	./$(BIN_DIR)/test_radar_scope

test_flight_realism: $(BIN_DIR)/test_flight_realism
	./$(BIN_DIR)/test_flight_realism

test_ejection: $(BIN_DIR)/test_ejection
	./$(BIN_DIR)/test_ejection

test_air_combat: $(BIN_DIR)/test_air_combat
	./$(BIN_DIR)/test_air_combat

test_missile_warning: $(BIN_DIR)/test_missile_warning
	./$(BIN_DIR)/test_missile_warning

test_stovl: $(BIN_DIR)/test_stovl
	./$(BIN_DIR)/test_stovl

test_jet_cockpits: $(BIN_DIR)/test_jet_cockpits
	./$(BIN_DIR)/test_jet_cockpits

test_propulsion: $(BIN_DIR)/test_propulsion
	./$(BIN_DIR)/test_propulsion

test_landing_gear: $(BIN_DIR)/test_landing_gear
	./$(BIN_DIR)/test_landing_gear

test_compressibility: $(BIN_DIR)/test_compressibility
	./$(BIN_DIR)/test_compressibility

test_integrated_flight: $(BIN_DIR)/test_integrated_flight
	./$(BIN_DIR)/test_integrated_flight

test_throttle_control: $(BIN_DIR)/test_throttle_control
	./$(BIN_DIR)/test_throttle_control

test_performance_envelope: $(BIN_DIR)/test_performance_envelope
	./$(BIN_DIR)/test_performance_envelope

run_all: all
	@echo "================ Running All Verification Tests ================"
	@./$(BIN_DIR)/test_atmosphere
	@./$(BIN_DIR)/test_ballistic
	@./$(BIN_DIR)/test_gyroscopic
	@./$(BIN_DIR)/test_performance
	@./$(BIN_DIR)/test_interpolator
	@./$(BIN_DIR)/test_aero_coefficients
	@./$(BIN_DIR)/test_pitch_divergence
	@./$(BIN_DIR)/test_actuators
	@./$(BIN_DIR)/test_flcs_level_flight
	@./$(BIN_DIR)/test_flcs_pullup
	@./$(BIN_DIR)/test_flcs_review
	@./$(BIN_DIR)/test_signal_conditioning
	@./$(BIN_DIR)/test_force_curves
	@./$(BIN_DIR)/test_avionics_controls
	@./$(BIN_DIR)/test_config_persistence
	@./$(BIN_DIR)/test_input_pipeline
	@./$(BIN_DIR)/test_camera_rig
	@./$(BIN_DIR)/test_hud_collimation
	@./$(BIN_DIR)/test_hud_clipping
	@./$(BIN_DIR)/test_flight_instruments
	@./$(BIN_DIR)/test_render_pipeline
	@./$(BIN_DIR)/test_terrain_field
	@./$(BIN_DIR)/test_terrain_render
	@./$(BIN_DIR)/test_ground_collision
	@./$(BIN_DIR)/test_ofc_handover
	@./$(BIN_DIR)/test_ofc_closed_loop
	@./$(BIN_DIR)/test_propulsion
	@./$(BIN_DIR)/test_landing_gear
	@./$(BIN_DIR)/test_compressibility
	@./$(BIN_DIR)/test_integrated_flight
	@./$(BIN_DIR)/test_throttle_control
	@./$(BIN_DIR)/test_performance_envelope
	@./$(BIN_DIR)/test_aircraft_specs
	@./$(BIN_DIR)/test_multi_aircraft_performance
	@./$(BIN_DIR)/test_model_glb
	@./$(BIN_DIR)/test_user_settings
	@./$(BIN_DIR)/test_menu_navigation
	@./$(BIN_DIR)/test_render_fidelity
	@./$(BIN_DIR)/test_thread_pool
	@./$(BIN_DIR)/test_aircraft_entity
	@./$(BIN_DIR)/test_guns
	@./$(BIN_DIR)/test_air_combat_geometry
	@./$(BIN_DIR)/test_bfm_ai
	@./$(BIN_DIR)/test_missiles
	@./$(BIN_DIR)/test_evade_mode
	@./$(BIN_DIR)/test_radar_scope
	@./$(BIN_DIR)/test_flight_realism
	@./$(BIN_DIR)/test_ejection
	@./$(BIN_DIR)/test_jet_cockpits
	@./$(BIN_DIR)/test_stovl
	@./$(BIN_DIR)/test_missile_warning
	@./$(BIN_DIR)/test_air_combat
	@echo "================ All Tests Completed Successfully! ================"

clean:
	rm -rf $(BIN_DIR)
