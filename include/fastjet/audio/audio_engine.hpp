#pragma once

#include <SDL3/SDL.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "fastjet/graphics/cockpit_telemetry.hpp"

namespace fastjet::audio {

/// @brief Represents an authentic audio sample loaded from a .wav file
struct AudioSample {
    std::vector<float> pcm;    // Interleaved stereo 44.1kHz float32 samples
    bool loaded        = false;
    double playhead    = 0.0;
    bool is_playing    = false;
    bool loop          = true;
    float current_gain = 0.0f;
    float current_rate = 1.0f; // Resampling playback speed ratio (pitch)

    size_t frame_count() const noexcept {
        return pcm.size() / 2;
    }
};

/// @brief Authentic sample-based cockpit audio engine using SDL3
/// Replaces procedural synthetic math with real recorded .wav audio samples:
/// - F-16 interior turbofan spool loop (engine_spool.wav)
/// - F110 afterburner combustion rumble (afterburner.wav)
/// - High-speed canopy boundary-layer wind rush (wind_rush.wav)
/// - Touchdown tire screech chirp (touchdown.wav)
/// - Landing gear hydraulic transit clunk (gear_transit.wav)
/// - Cockpit aural warning tones / Betty (over_g.wav, stall.wav, pull_up.wav)
///
/// Radar warning receiver tones are synthesised, because in the jet they are
/// synthesised too: the RWR drives the headset with electronic tones, not
/// recordings. New emitter: three short chirps. Lock: a steady pulsed beep.
/// Missile launch: a fast two-tone warble that holds while the missile is guided.
///
/// The missile approach warner (MAWS) has its own sound, so the pilot can tell
/// "a radar missile is guiding on me" (RWR) from "I can see a missile coming"
/// (MAWS, which also sees heat seekers): falling sweeps, which no RWR tone
/// makes. Three fast high sweeps once for each new missile declared, then a
/// slower sweep that repeats faster as the time to go runs out.
///
/// The F-35B lift system is synthesised as well (no LiftFan recording ships
/// with the sim): broadband jet roar plus the fan's blade-passing whine, both
/// following LiftFan thrust, a hydraulic clunk as the doors start or finish
/// moving, and a click when the keyboard throttle drops into the STOVL detent.
///
/// NOTE: If sound files are not installed in assets/sounds/, the recorded
/// channels stay silent (no procedural substitutes for engine or airflow).
///
/// Threading: mixing runs on SDL's audio thread, in the stream's get
/// callback, so playback never depends on the frame rate: a long frame
/// (loading, a dragged window) cannot starve the device and crackle. The
/// main thread only sets playback parameters, holding the stream lock that
/// SDL also holds around the callback.
class AudioEngine {
public:
    static constexpr int SAMPLE_RATE = 44100;
    static constexpr int CHANNELS    = 2;

private:
    SDL_AudioDeviceID device_id_ = 0;
    SDL_AudioStream* stream_     = nullptr;
    bool initialized_            = false;
    bool enabled_                = true;

    // Authentic audio sample slots
    AudioSample sfx_spool_;        // Looped engine spool
    AudioSample sfx_afterburner_;  // Looped afterburner roar
    AudioSample sfx_wind_;         // Looped canopy wind rush
    AudioSample sfx_touchdown_;    // One-shot tire touchdown chirp
    AudioSample sfx_gear_;         // One-shot gear transit clunk
    AudioSample sfx_over_g_;       // Over-G aural warning
    AudioSample sfx_stall_;        // Stall warning horn
    AudioSample sfx_pullup_;       // Betty "Pull Up" alert

    // Mix bus gains: continuous engine/airflow loops vs. alerts and one-shots
    float engine_bus_gain_ = 1.0f;
    float alerts_bus_gain_ = 1.0f;

    // Mix buffer for the audio thread, allocated once at init.
    std::vector<float> mix_buffer_;
    static constexpr int kMixChunkFrames = 1024;

    // RWR tone generator (audio thread reads, main thread sets under the stream lock)
    enum class RwrTone : uint8_t { NONE, LOCK, LAUNCH };
    RwrTone rwr_tone_ = RwrTone::NONE;
    int rwr_chirps_left_ = 0;     // New-emitter chirps still to play
    double rwr_clock_ = 0.0;      // Seconds into the current pattern
    double rwr_phase_ = 0.0;      // Oscillator phase [cycles]
    float rwr_env_ = 0.0f;        // Click-free on/off envelope
    int prev_rwr_level_ = 0;

    // MAWS tone generator (audio thread reads, main thread sets under the stream lock)
    static constexpr double kMawsLaunchS = 0.45; // Launch alert: three 0.15 s sweeps
    bool maws_on_ = false;               // A declared missile is closing
    double maws_period_s_ = 0.6;         // Inbound sweep repetition [s]
    double maws_launch_left_s_ = 0.0;    // Launch alert still to play [s]
    double maws_clock_ = 0.0;            // Seconds into the inbound pattern
    double maws_phase_ = 0.0;            // Oscillator phase [cycles]
    float maws_env_ = 0.0f;              // Click-free on/off envelope
    unsigned prev_maws_launches_ = 0;

    // STOVL lift system synthesis (audio thread reads, main thread sets under the stream lock)
    float lift_fan_level_ = 0.0f;  // Target loudness 0..1 from LiftFan thrust
    float lift_fan_env_ = 0.0f;    // ...smoothed, as the fan spools
    double lift_fan_phase_ = 0.0;  // Blade-passing oscillator [cycles]
    float roar_lp1_ = 0.0f;        // Two one-pole low-passes shaping the roar
    float roar_lp2_ = 0.0f;
    uint32_t noise_state_ = 0x9E3779B9u;
    float clunk_env_ = 0.0f;       // Door clunk one-shot
    double clunk_phase_ = 0.0;
    float click_env_ = 0.0f;       // Throttle detent click one-shot
    double click_phase_ = 0.0;
    propulsion::StovlMode prev_stovl_mode_ = propulsion::StovlMode::CTOL;

    // Combat procedural audio synthesis (audio thread reads, main thread sets under stream lock)
    bool gun_firing_ = false;
    double gun_phase_ = 0.0;
    float gun_env_ = 0.0f;
    float gun_rumble_lp_ = 0.0f;

    float ir_growl_level_ = 0.0f;
    bool ir_locked_ = false;
    double ir_tone_phase_ = 0.0;
    double ir_am_phase_ = 0.0;
    float ir_env_ = 0.0f;

    float launch_whoosh_env_ = 0.0f;
    float launch_whoosh_lp_ = 0.0f;

    float blast_env_ = 0.0f;
    float blast_lp1_ = 0.0f;
    float blast_lp2_ = 0.0f;
    double blast_phase_ = 0.0;

    // State transition tracking
    bool prev_on_ground_     = false;
    bool prev_gear_deployed_ = true;
    bool prev_over_g_        = false;
    bool prev_stall_         = false;

public:
    AudioEngine() = default;

    ~AudioEngine() {
        destroy();
    }

    /// @brief Initialize SDL3 audio playback device and load authentic .wav assets
    bool init() {
        if (initialized_) return true;

        if (!SDL_InitSubSystem(SDL_INIT_AUDIO)) {
            return false;
        }

        SDL_AudioSpec spec{};
        spec.format   = SDL_AUDIO_F32;
        spec.channels = CHANNELS;
        spec.freq     = SAMPLE_RATE;

        device_id_ = SDL_OpenAudioDevice(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec);
        if (!device_id_) {
            return false;
        }

        stream_ = SDL_CreateAudioStream(&spec, &spec);
        if (!stream_) {
            SDL_CloseAudioDevice(device_id_);
            device_id_ = 0;
            return false;
        }

        if (!SDL_BindAudioStream(device_id_, stream_)) {
            SDL_DestroyAudioStream(stream_);
            SDL_CloseAudioDevice(device_id_);
            stream_ = nullptr;
            device_id_ = 0;
            return false;
        }

        // Load authentic audio samples from disk if present. Done before the
        // mixer starts, so the audio thread never sees a half-loaded sample.
        load_sample("assets/sounds/engine_spool.wav", sfx_spool_, true);
        load_sample("assets/sounds/afterburner.wav", sfx_afterburner_, true);
        load_sample("assets/sounds/wind_rush.wav", sfx_wind_, true);
        load_sample("assets/sounds/touchdown.wav", sfx_touchdown_, false);
        load_sample("assets/sounds/gear_transit.wav", sfx_gear_, false);
        load_sample("assets/sounds/over_g.wav", sfx_over_g_, false);
        load_sample("assets/sounds/stall.wav", sfx_stall_, false);
        load_sample("assets/sounds/pull_up.wav", sfx_pullup_, false);

        int loaded_count = (sfx_spool_.loaded ? 1 : 0) +
                           (sfx_afterburner_.loaded ? 1 : 0) +
                           (sfx_wind_.loaded ? 1 : 0) +
                           (sfx_touchdown_.loaded ? 1 : 0) +
                           (sfx_gear_.loaded ? 1 : 0) +
                           (sfx_over_g_.loaded ? 1 : 0) +
                           (sfx_stall_.loaded ? 1 : 0) +
                           (sfx_pullup_.loaded ? 1 : 0);

        if (loaded_count > 0) {
            std::cout << "[AUDIO] Loaded " << loaded_count << " authentic .wav sample assets.\n";
        } else {
            std::cout << "[AUDIO] No .wav audio samples in assets/sounds/ - Audio muted cleanly.\n";
        }

        mix_buffer_.assign(static_cast<size_t>(kMixChunkFrames) * CHANNELS, 0.0f);
        initialized_ = true;
        // The mixer always runs: the RWR tones need no sample files. With
        // nothing playing it only writes silence.
        SDL_SetAudioStreamGetCallback(stream_, &AudioEngine::feed, this);
        SDL_ResumeAudioDevice(device_id_);
        return true;
    }

    void destroy() noexcept {
        if (stream_) {
            SDL_DestroyAudioStream(stream_);
            stream_ = nullptr;
        }
        if (device_id_) {
            SDL_CloseAudioDevice(device_id_);
            device_id_ = 0;
        }
        initialized_ = false;
    }

    void set_enabled(bool enabled) noexcept {
        StreamLock lock(stream_);
        enabled_ = enabled;
    }

    /// @brief Final linear gains per mix bus (master volume and mute already
    /// folded in by the caller). Takes effect on the next mixed chunk.
    void set_bus_gains(float engine_bus, float alerts_bus) noexcept {
        StreamLock lock(stream_);
        engine_bus_gain_ = std::clamp(engine_bus, 0.0f, 1.0f);
        alerts_bus_gain_ = std::clamp(alerts_bus, 0.0f, 1.0f);
    }

    bool is_initialized() const noexcept { return initialized_; }

    /// @brief Throttle lever dropped into the STOVL hold detent.
    void cue_detent() noexcept {
        if (!initialized_) return;
        StreamLock lock(stream_);
        click_env_ = 1.0f;
    }

    /// @brief Missile launch whoosh SFX cue
    void cue_missile_launch() noexcept {
        if (!initialized_) return;
        StreamLock lock(stream_);
        launch_whoosh_env_ = 1.0f;
    }

    /// @brief Detonation explosion blast SFX cue
    void cue_explosion(float intensity = 1.0f) noexcept {
        if (!initialized_) return;
        StreamLock lock(stream_);
        blast_env_ = std::clamp(intensity, 0.0f, 1.0f);
    }

    /// @brief Update audio playback and mix active sample channels into stream
    void update(const graphics::AvionicsTelemetry& tel,
                double dynamic_pressure,
                double airspeed,
                [[maybe_unused]] double frame_dt) {
        if (!initialized_ || !enabled_) return;

        update_rwr(tel);
        update_maws(tel);
        update_stovl(tel);
        update_combat(tel);

        // If no authentic samples are installed on disk, run completely silent without queuing buffer
        const bool any_sample_loaded = (sfx_spool_.loaded || sfx_afterburner_.loaded ||
                                        sfx_wind_.loaded || sfx_touchdown_.loaded ||
                                        sfx_gear_.loaded || sfx_over_g_.loaded ||
                                        sfx_stall_.loaded || sfx_pullup_.loaded);
        if (!any_sample_loaded) {
            return;
        }

        // The audio thread reads these parameters while mixing.
        StreamLock lock(stream_);

        // If aircraft is crashed, silence propulsion and aerodynamic rush loops
        if (tel.is_crashed && !tel.pilot_ejected) {
            sfx_spool_.is_playing = false;
            sfx_afterburner_.is_playing = false;
            sfx_wind_.is_playing = false;
        } else if (tel.pilot_ejected) {
            // Out in the airflow: no engine, just the wind until the canopy opens.
            sfx_spool_.is_playing = false;
            sfx_afterburner_.is_playing = false;
            if (sfx_wind_.loaded) {
                const double q_norm = std::clamp(dynamic_pressure / 60000.0, 0.0, 1.0);
                sfx_wind_.is_playing = q_norm > 0.0005;
                sfx_wind_.current_gain = static_cast<float>(std::sqrt(q_norm) * 0.5);
                sfx_wind_.current_rate = static_cast<float>(0.85 + 0.35 * q_norm);
            }
        } else {
            // 1. Spool sample parameters
            if (sfx_spool_.loaded) {
                sfx_spool_.is_playing = true;
                const double rpm_norm = std::clamp((tel.engine_rpm_pct - 60.0) / 40.0, 0.0, 1.0);
                sfx_spool_.current_rate = static_cast<float>(0.75 + 0.50 * rpm_norm); // Pitch tracks RPM
                sfx_spool_.current_gain = static_cast<float>(0.30 + 0.45 * rpm_norm); // Volume tracks RPM
            }

            // 2. Afterburner sample parameters
            if (sfx_afterburner_.loaded) {
                const double ab_frac = std::clamp((tel.throttle_input - 0.85) / 0.15, 0.0, 1.0);
                sfx_afterburner_.is_playing = (ab_frac > 0.01);
                sfx_afterburner_.current_gain = static_cast<float>(ab_frac * 0.75);
                sfx_afterburner_.current_rate = 1.0f;
            }

            // 3. Canopy aerodynamic wind rush sample parameters
            if (sfx_wind_.loaded) {
                const double q_norm = std::clamp(dynamic_pressure / 60000.0, 0.0, 1.0);
                sfx_wind_.is_playing = (q_norm > 0.02);
                sfx_wind_.current_gain = static_cast<float>(q_norm * q_norm * 0.60);
                sfx_wind_.current_rate = static_cast<float>(0.85 + 0.35 * q_norm);
            }
        }

        // 4. One-shot transitions
        // Touchdown tire screech
        if (tel.on_ground && !prev_on_ground_ && airspeed > 25.0) {
            play_one_shot(sfx_touchdown_, 0.70f);
        }
        prev_on_ground_ = tel.on_ground;

        // Gear handle transition
        if (tel.gear_deployed != prev_gear_deployed_) {
            play_one_shot(sfx_gear_, 0.60f);
            prev_gear_deployed_ = tel.gear_deployed;
        }

        // Over-G alert tone
        if (tel.over_g_alert && !prev_over_g_) {
            play_one_shot(sfx_over_g_, 0.80f);
        }
        prev_over_g_ = tel.over_g_alert;

        // Stall / High-AoA alert horn
        if (tel.high_aoa_alert && !prev_stall_) {
            play_one_shot(sfx_stall_, 0.75f);
        }
        prev_stall_ = tel.high_aoa_alert;
    }

private:
    /// @brief Pick the RWR tone from the combat telemetry. The highest threat
    /// wins; a new emitter (clear -> anything) starts the three-chirp alert.
    void update_rwr(const graphics::AvionicsTelemetry& tel) noexcept {
        const int level = (tel.combat.active && tel.combat.rwr_active && !tel.is_crashed) ? tel.combat.rwr_level : 0;
        StreamLock lock(stream_);
        const RwrTone want = level >= 3 ? RwrTone::LAUNCH : level == 2 ? RwrTone::LOCK : RwrTone::NONE;
        if (want != rwr_tone_) {
            rwr_tone_ = want;
            rwr_clock_ = 0.0;
        }
        if (level > 0 && prev_rwr_level_ == 0) {
            rwr_chirps_left_ = 3;
            if (rwr_tone_ == RwrTone::NONE) rwr_clock_ = 0.0;
        }
        if (level == 0) rwr_chirps_left_ = 0;
        prev_rwr_level_ = level;
    }

    /// @brief MAWS tones from the combat telemetry: the launch alert once
    /// whenever the count of declared missiles rises, and the inbound sweep
    /// while one is closing, repeating every 0.6 s far out and every 0.25 s
    /// in the last few seconds.
    void update_maws(const graphics::AvionicsTelemetry& tel) noexcept {
        const graphics::CombatTelemetry& c = tel.combat;
        const bool live = c.active && c.maws_fitted && !tel.is_crashed && !tel.pilot_ejected;
        StreamLock lock(stream_);
        const bool on = live && c.maws_threat;
        if (on && !maws_on_) maws_clock_ = 0.0;
        maws_on_ = on;
        maws_period_s_ = std::clamp(0.2 + 0.04 * c.maws_time_to_go_s, 0.25, 0.6);
        if (live && c.maws_launch_count > prev_maws_launches_) maws_launch_left_s_ = kMawsLaunchS;
        prev_maws_launches_ = c.maws_launch_count;
    }

    /// @brief Synthesise the MAWS tones into @p out (audio thread).
    void mix_maws(float* out, int num_samples) noexcept {
        constexpr double DT = 1.0 / SAMPLE_RATE;
        constexpr float GAIN = 0.20f;
        if (!maws_on_ && maws_launch_left_s_ <= 0.0 && maws_env_ <= 0.0f) return;
        for (int i = 0; i < num_samples; ++i) {
            double freq = 0.0;
            bool on = false;
            if (maws_launch_left_s_ > 0.0) {
                // Launch: three fast sweeps, 2.6 down to 1.2 kHz in 0.11 s.
                const double cyc = std::fmod(kMawsLaunchS - maws_launch_left_s_, 0.15);
                on = cyc < 0.11;
                freq = 2600.0 - 1400.0 * std::min(cyc / 0.11, 1.0);
                maws_launch_left_s_ -= DT;
            } else if (maws_on_) {
                // Inbound: 1.5 down to 0.8 kHz in 0.16 s, once a period.
                const double cyc = std::fmod(maws_clock_, maws_period_s_);
                on = cyc < 0.16;
                freq = 1500.0 - 700.0 * std::min(cyc / 0.16, 1.0);
                maws_clock_ += DT;
            }
            // 3 ms attack/release, as the RWR's.
            const float target = on ? 1.0f : 0.0f;
            maws_env_ += (target - maws_env_) * 0.0075f;
            if (!on && maws_env_ < 1e-4f) maws_env_ = 0.0f;
            if (freq > 0.0) maws_phase_ = std::fmod(maws_phase_ + freq * DT, 1.0);
            const double ph = 2.0 * M_PI * maws_phase_;
            const float v = static_cast<float>(std::sin(ph) + 0.2 * std::sin(2.0 * ph)) * maws_env_ * GAIN *
                            alerts_bus_gain_;
            out[i * 2 + 0] = std::clamp(out[i * 2 + 0] + v, -1.0f, 1.0f);
            out[i * 2 + 1] = std::clamp(out[i * 2 + 1] + v, -1.0f, 1.0f);
        }
    }

    /// @brief LiftFan loudness from its thrust; a clunk on every lift-system
    /// mode change (doors starting to move, or reaching their stops).
    void update_stovl(const graphics::AvionicsTelemetry& tel) noexcept {
        constexpr double LIFT_FAN_RATED_N = 83100.0; // 18,680 lbf in a full-power hover (P&W)
        const propulsion::StovlStatus& st = tel.stovl;
        const bool live = st.equipped && !tel.is_crashed && !tel.pilot_ejected;
        StreamLock lock(stream_);
        lift_fan_level_ = live ? static_cast<float>(std::clamp(st.lift_fan_thrust_n / LIFT_FAN_RATED_N, 0.0, 1.0)) : 0.0f;
        if (live && st.mode != prev_stovl_mode_) clunk_env_ = 1.0f;
        prev_stovl_mode_ = st.mode;
    }

    /// @brief Synthesise the lift system into @p out (audio thread).
    void mix_stovl(float* out, int num_samples) noexcept {
        constexpr double DT = 1.0 / SAMPLE_RATE;
        if (lift_fan_level_ <= 0.0f && lift_fan_env_ < 1e-4f && clunk_env_ < 1e-4f && click_env_ < 1e-4f) return;
        for (int i = 0; i < num_samples; ++i) {
            // ~0.5 s spool lag, so the roar swells as the fan comes up.
            lift_fan_env_ += (lift_fan_level_ - lift_fan_env_) * 4.5e-5f;
            noise_state_ ^= noise_state_ << 13;
            noise_state_ ^= noise_state_ >> 17;
            noise_state_ ^= noise_state_ << 5;
            const float white = static_cast<float>(noise_state_) * (2.0f / 4294967296.0f) - 1.0f;

            // Roar: white noise low-passed at ~900 Hz plus a heavier ~280 Hz rumble.
            roar_lp1_ += (white - roar_lp1_) * 0.13f;
            roar_lp2_ += (roar_lp1_ - roar_lp2_) * 0.04f;
            const float roar = roar_lp1_ * 0.6f + roar_lp2_ * 1.6f;
            // Blade-passing whine, rising in pitch as the fan spools up.
            lift_fan_phase_ = std::fmod(lift_fan_phase_ + (900.0 + 900.0 * lift_fan_env_) * DT, 1.0);
            const double ph = 2.0 * M_PI * lift_fan_phase_;
            const float whine = static_cast<float>(0.25 * std::sin(ph) + 0.08 * std::sin(2.0 * ph)) * lift_fan_env_;
            const float fan = (roar + whine) * lift_fan_env_ * 0.35f * engine_bus_gain_;

            // Door clunk: a 55 Hz thump with a noise transient, ~0.15 s decay.
            clunk_phase_ = std::fmod(clunk_phase_ + 55.0 * DT, 1.0);
            const float clunk = (static_cast<float>(std::sin(2.0 * M_PI * clunk_phase_)) + white * clunk_env_ * 0.3f) *
                                clunk_env_ * 0.45f * alerts_bus_gain_;
            clunk_env_ *= 0.99985f;
            // Detent click: a 2.2 kHz tick, ~10 ms decay.
            click_phase_ = std::fmod(click_phase_ + 2200.0 * DT, 1.0);
            const float click = static_cast<float>(std::sin(2.0 * M_PI * click_phase_)) * click_env_ * 0.2f * alerts_bus_gain_;
            click_env_ *= 0.9977f;

            const float v = fan + clunk + click;
            out[i * 2 + 0] = std::clamp(out[i * 2 + 0] + v, -1.0f, 1.0f);
            out[i * 2 + 1] = std::clamp(out[i * 2 + 1] + v, -1.0f, 1.0f);
        }
    }

    /// @brief Synthesise the RWR tone into @p out (audio thread).
    void mix_rwr(float* out, int num_samples) noexcept {
        constexpr double DT = 1.0 / SAMPLE_RATE;
        constexpr float GAIN = 0.22f;
        const bool chirping = rwr_tone_ == RwrTone::NONE && rwr_chirps_left_ > 0;
        if (rwr_tone_ == RwrTone::NONE && !chirping && rwr_env_ <= 0.0f) return;
        for (int i = 0; i < num_samples; ++i) {
            double freq = 0.0;
            bool on = false;
            switch (rwr_tone_) {
                case RwrTone::LAUNCH:
                    // Two-tone warble, 12 alternations a second, continuous.
                    freq = std::fmod(rwr_clock_ * 12.0, 1.0) < 0.5 ? 1350.0 : 1800.0;
                    on = true;
                    break;
                case RwrTone::LOCK:
                    // Steady pulsed tone: beep-beep-beep, 5 a second.
                    freq = 1000.0;
                    on = std::fmod(rwr_clock_ * 5.0, 1.0) < 0.5;
                    break;
                case RwrTone::NONE:
                    if (rwr_chirps_left_ > 0) {
                        // New emitter: three 70 ms chirps, 50 ms apart.
                        const double cycle = std::fmod(rwr_clock_, 0.12);
                        freq = 1200.0;
                        on = cycle < 0.07;
                        if (rwr_clock_ >= 0.12 * 3) rwr_chirps_left_ = 0;
                    }
                    break;
            }
            // 3 ms attack/release: no clicks at the pulse edges.
            const float target = on ? 1.0f : 0.0f;
            rwr_env_ += (target - rwr_env_) * 0.0075f;
            if (!on && rwr_env_ < 1e-4f) rwr_env_ = 0.0f;
            if (freq > 0.0) rwr_phase_ = std::fmod(rwr_phase_ + freq * DT, 1.0);
            const double ph = 2.0 * M_PI * rwr_phase_;
            // A little third harmonic gives the hard edge of an electronic tone.
            const float v = static_cast<float>(std::sin(ph) + 0.3 * std::sin(3.0 * ph)) * rwr_env_ * GAIN *
                            alerts_bus_gain_;
            out[i * 2 + 0] = std::clamp(out[i * 2 + 0] + v, -1.0f, 1.0f);
            out[i * 2 + 1] = std::clamp(out[i * 2 + 1] + v, -1.0f, 1.0f);
            rwr_clock_ += DT;
        }
    }

    /// @brief Update combat sound triggers from telemetry
    void update_combat(const graphics::AvionicsTelemetry& tel) noexcept {
        const bool live = tel.combat.active && !tel.is_crashed && !tel.pilot_ejected;
        StreamLock lock(stream_);
        gun_firing_ = live && tel.combat.gun_firing;
        ir_growl_level_ = (live && tel.combat.ir_seeker_active) ? 1.0f : 0.0f;
        ir_locked_ = live && tel.combat.ir_seeker_locked;
    }

    /// @brief Synthesise gun, Sidewinder growl/screech, launch whoosh and detonation blasts into @p out (audio thread)
    void mix_combat(float* out, int num_samples) noexcept {
        constexpr double DT = 1.0 / SAMPLE_RATE;
        if (!gun_firing_ && gun_env_ < 1e-4f &&
            ir_growl_level_ <= 0.0f && ir_env_ < 1e-4f &&
            launch_whoosh_env_ < 1e-4f && blast_env_ < 1e-4f) {
            return;
        }

        for (int i = 0; i < num_samples; ++i) {
            noise_state_ ^= noise_state_ << 13;
            noise_state_ ^= noise_state_ >> 17;
            noise_state_ ^= noise_state_ << 5;
            const float white = static_cast<float>(noise_state_) * (2.0f / 4294967296.0f) - 1.0f;

            // 1. M61A1 Vulcan Cannon: 100 Hz (6,000 RPM) pulse train + low rumble
            const float gun_target = gun_firing_ ? 1.0f : 0.0f;
            gun_env_ += (gun_target - gun_env_) * 0.015f;
            float gun_val = 0.0f;
            if (gun_env_ > 1e-4f) {
                gun_phase_ = std::fmod(gun_phase_ + 100.0 * DT, 1.0);
                const float pulse = (gun_phase_ < 0.22) ? (std::sin(static_cast<float>(2.0 * M_PI * gun_phase_ * 4.5)) + white * 0.45f) : 0.0f;
                gun_rumble_lp_ += (white - gun_rumble_lp_) * 0.035f;
                gun_val = (pulse * 0.65f + gun_rumble_lp_ * 0.75f) * gun_env_ * 0.50f * alerts_bus_gain_;
            }

            // 2. Sidewinder IR Growl/Screech
            const float ir_target = ir_growl_level_;
            ir_env_ += (ir_target - ir_env_) * 0.005f;
            float ir_val = 0.0f;
            if (ir_env_ > 1e-4f) {
                if (ir_locked_) {
                    // High-pitched lock screech (420 Hz + 840 Hz harmonic)
                    ir_tone_phase_ = std::fmod(ir_tone_phase_ + 420.0 * DT, 1.0);
                    ir_am_phase_   = std::fmod(ir_am_phase_ + 840.0 * DT, 1.0);
                    const float s1 = std::sin(static_cast<float>(2.0 * M_PI * ir_tone_phase_));
                    const float s2 = std::sin(static_cast<float>(2.0 * M_PI * ir_am_phase_));
                    ir_val = (s1 * 0.65f + s2 * 0.35f) * ir_env_ * 0.28f * alerts_bus_gain_;
                } else {
                    // Low growl buzz (85 Hz AM tone with rasp)
                    ir_am_phase_   = std::fmod(ir_am_phase_ + 85.0 * DT, 1.0);
                    ir_tone_phase_ = std::fmod(ir_tone_phase_ + 170.0 * DT, 1.0);
                    const float mod = 0.55f + 0.45f * std::sin(static_cast<float>(2.0 * M_PI * ir_am_phase_));
                    const float tone = std::sin(static_cast<float>(2.0 * M_PI * ir_tone_phase_));
                    ir_val = (tone * mod + white * 0.18f) * ir_env_ * 0.24f * alerts_bus_gain_;
                }
            }

            // 3. Missile launch whoosh
            float whoosh_val = 0.0f;
            if (launch_whoosh_env_ > 1e-4f) {
                launch_whoosh_lp_ += (white - launch_whoosh_lp_) * (0.05f + 0.15f * (1.0f - launch_whoosh_env_));
                whoosh_val = launch_whoosh_lp_ * launch_whoosh_env_ * 0.45f * alerts_bus_gain_;
                launch_whoosh_env_ *= 0.99988f; // ~0.8s tail
            }

            // 4. Detonation blast (45 Hz shockwave thump + heavy rumble)
            float blast_val = 0.0f;
            if (blast_env_ > 1e-4f) {
                blast_phase_ = std::fmod(blast_phase_ + 45.0 * DT, 1.0);
                blast_lp1_ += (white - blast_lp1_) * 0.04f;
                blast_lp2_ += (blast_lp1_ - blast_lp2_) * 0.02f;
                const float thump = std::sin(static_cast<float>(2.0 * M_PI * blast_phase_));
                blast_val = (thump * 0.55f + blast_lp2_ * 1.4f + blast_lp1_ * 0.4f) * blast_env_ * 0.65f * alerts_bus_gain_;
                blast_env_ *= 0.99993f; // ~1.5s decay
            }

            const float v = gun_val + ir_val + whoosh_val + blast_val;
            out[i * 2 + 0] = std::clamp(out[i * 2 + 0] + v, -1.0f, 1.0f);
            out[i * 2 + 1] = std::clamp(out[i * 2 + 1] + v, -1.0f, 1.0f);
        }
    }

    /// Scoped SDL stream lock (no-op without a stream).
    struct StreamLock {
        SDL_AudioStream* s;
        explicit StreamLock(SDL_AudioStream* stream) : s(stream) { if (s) SDL_LockAudioStream(s); }
        ~StreamLock() { if (s) SDL_UnlockAudioStream(s); }
        StreamLock(const StreamLock&) = delete;
        StreamLock& operator=(const StreamLock&) = delete;
    };

    /// Audio-thread callback: the device wants `additional` more bytes.
    /// SDL holds the stream lock for the duration.
    static void SDLCALL feed(void* userdata, SDL_AudioStream* stream, int additional, int /*total*/) {
        auto* self = static_cast<AudioEngine*>(userdata);
        constexpr int kFrameBytes = CHANNELS * static_cast<int>(sizeof(float));
        int frames = (additional + kFrameBytes - 1) / kFrameBytes;
        while (frames > 0) {
            const int n = std::min(frames, kMixChunkFrames);
            float* buf = self->mix_buffer_.data();
            std::fill(buf, buf + static_cast<size_t>(n) * CHANNELS, 0.0f);
            if (self->enabled_) self->mix_samples(buf, n);
            SDL_PutAudioStreamData(stream, buf, n * kFrameBytes);
            frames -= n;
        }
    }

    void play_one_shot(AudioSample& s, float gain) noexcept {
        if (!s.loaded) return;
        s.playhead = 0.0;
        s.current_gain = gain;
        s.current_rate = 1.0f;
        s.is_playing = true;
    }

    /// @brief Mix active audio sample channels into output buffer with linear interpolation
    void mix_samples(float* out, int num_samples) noexcept {
        struct Channel { AudioSample* sample; float bus_gain; };
        const Channel channels[] = {
            {&sfx_spool_, engine_bus_gain_}, {&sfx_afterburner_, engine_bus_gain_}, {&sfx_wind_, engine_bus_gain_},
            {&sfx_touchdown_, alerts_bus_gain_}, {&sfx_gear_, alerts_bus_gain_}, {&sfx_over_g_, alerts_bus_gain_},
            {&sfx_stall_, alerts_bus_gain_}, {&sfx_pullup_, alerts_bus_gain_}
        };

        for (const Channel& ch : channels) {
            AudioSample* s = ch.sample;
            if (!s->loaded || !s->is_playing || s->pcm.empty()) continue;

            const size_t total_frames = s->frame_count();
            if (total_frames < 2) continue;

            const float gain = s->current_gain * ch.bus_gain;
            const double rate = s->current_rate;

            for (int i = 0; i < num_samples; ++i) {
                const size_t f0 = static_cast<size_t>(s->playhead);
                const size_t f1 = (f0 + 1 < total_frames) ? (f0 + 1) : (s->loop ? 0 : f0);
                const float frac = static_cast<float>(s->playhead - static_cast<double>(f0));

                const float l0 = s->pcm[f0 * 2 + 0];
                const float r0 = s->pcm[f0 * 2 + 1];
                const float l1 = s->pcm[f1 * 2 + 0];
                const float r1 = s->pcm[f1 * 2 + 1];

                const float l_sample = (l0 + (l1 - l0) * frac) * gain;
                const float r_sample = (r0 + (r1 - r0) * frac) * gain;

                out[i * 2 + 0] = std::clamp(out[i * 2 + 0] + l_sample, -1.0f, 1.0f);
                out[i * 2 + 1] = std::clamp(out[i * 2 + 1] + r_sample, -1.0f, 1.0f);

                s->playhead += rate;
                if (s->playhead >= static_cast<double>(total_frames)) {
                    if (s->loop) {
                        s->playhead = std::fmod(s->playhead, static_cast<double>(total_frames));
                    } else {
                        s->is_playing = false;
                        s->playhead = 0.0;
                        break;
                    }
                }
            }
        }
        mix_rwr(out, num_samples);
        mix_maws(out, num_samples);
        mix_stovl(out, num_samples);
        mix_combat(out, num_samples);
    }

    /// @brief Load a .wav file from disk and convert to target 44.1kHz stereo Float32 format
    bool load_sample(const std::string& filepath, AudioSample& sample, bool loop) {
        sample.loaded = false;
        sample.loop = loop;
        sample.pcm.clear();

        SDL_AudioSpec src_spec{};
        Uint8* wav_buf = nullptr;
        Uint32 wav_len = 0;

        if (!SDL_LoadWAV(filepath.c_str(), &src_spec, &wav_buf, &wav_len)) {
            return false; // File not present on disk
        }

        SDL_AudioSpec dst_spec{};
        dst_spec.format   = SDL_AUDIO_F32;
        dst_spec.channels = CHANNELS;
        dst_spec.freq     = SAMPLE_RATE;

        // Use SDL3 AudioStream for automatic resampling, channel remixing, and format conversion
        SDL_AudioStream* conv_stream = SDL_CreateAudioStream(&src_spec, &dst_spec);
        if (!conv_stream) {
            SDL_free(wav_buf);
            return false;
        }

        if (!SDL_PutAudioStreamData(conv_stream, wav_buf, static_cast<int>(wav_len)) ||
            !SDL_FlushAudioStream(conv_stream)) {
            SDL_DestroyAudioStream(conv_stream);
            SDL_free(wav_buf);
            return false;
        }
        SDL_free(wav_buf);

        const int available_bytes = SDL_GetAudioStreamAvailable(conv_stream);
        if (available_bytes > 0) {
            const size_t num_floats = static_cast<size_t>(available_bytes) / sizeof(float);
            sample.pcm.resize(num_floats);
            SDL_GetAudioStreamData(conv_stream, sample.pcm.data(), available_bytes);
            sample.loaded = true;
        }

        SDL_DestroyAudioStream(conv_stream);
        return sample.loaded;
    }
};

} // namespace fastjet::audio
