#include "audio/audio_output.h"
#include "log.h"

#include <SDL.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace ap::audio {
namespace {

// Starting an empty push-mode device makes every small network scheduling
// delay audible.  Keep a modest cushion, as UxPlay does with its RAOP jitter
// buffer, while still staying well below perceptible lip-sync latency.
constexpr uint32_t kStartBufferMs = 80;
constexpr uint32_t kMaxBufferMs   = 500;

} // namespace

SdlAudioOutput::SdlAudioOutput()  = default;
SdlAudioOutput::~SdlAudioOutput() { stop(); }

bool SdlAudioOutput::start(int sample_rate, int channels) {
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        LOG_ERROR << "SDL_InitSubSystem(AUDIO) failed: " << SDL_GetError();
        return false;
    }

    SDL_AudioSpec want{};
    want.freq     = sample_rate;
    want.format   = AUDIO_S16SYS;     // interleaved int16, host byte order
    want.channels = static_cast<Uint8>(channels);
    want.samples  = 1024;             // buffer granularity (~23 ms @ 44.1 kHz)
    want.callback = nullptr;          // push mode via SDL_QueueAudio

    SDL_AudioSpec got{};
    // Do not accept a different callback rate/channel count here.  The PCM
    // producer remains at the AirPlay rate, so accepting (for example) a
    // native 48 kHz device while continuing to enqueue 44.1 kHz samples makes
    // the consumer drain 8.8% faster and guarantees periodic underruns.  With
    // allowed_changes=0 SDL performs the device conversion internally and the
    // queue exposed to us retains the requested format/rate.
    device_ = SDL_OpenAudioDevice(nullptr, 0, &want, &got, 0);
    if (device_ == 0) {
        LOG_ERROR << "SDL_OpenAudioDevice failed: " << SDL_GetError();
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        return false;
    }
    if (got.freq != want.freq || got.channels != want.channels ||
        got.format != want.format) {
        LOG_ERROR << "SDL returned an unexpected audio format: requested "
                  << want.freq << " Hz x " << static_cast<int>(want.channels)
                  << ", got " << got.freq << " Hz x "
                  << static_cast<int>(got.channels);
        SDL_CloseAudioDevice(device_);
        device_ = 0;
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        return false;
    }

    sample_rate_ = got.freq;
    channels_    = got.channels;
    bytes_per_second_ = static_cast<uint32_t>(sample_rate_ * channels_ *
                                               sizeof(int16_t));
    playing_ = false;
    // Leave the device paused until push() has accumulated a jitter cushion.
    // SDL otherwise consumes the first frame before the next UDP packet has
    // arrived and a momentary underrun can turn into a conspicuous dropout.
    SDL_PauseAudioDevice(device_, 1);
    LOG_INFO << "SdlAudioOutput ready at " << sample_rate_ << " Hz x "
             << channels_ << "ch (int16, " << kStartBufferMs
             << " ms startup jitter buffer)";
    return true;
}

void SdlAudioOutput::stop() {
    if (device_) {
        SDL_PauseAudioDevice(device_, 1);
        SDL_ClearQueuedAudio(device_);
        SDL_CloseAudioDevice(device_);
        device_ = 0;
        playing_ = false;
        bytes_per_second_ = 0;
        // Deliberately NOT calling SDL_QuitSubSystem(SDL_INIT_AUDIO):
        // stop() runs on the RTSP server thread during session
        // teardown, and on Windows shutting down the SDL audio
        // subsystem cross-thread can knock out the video event pump
        // (spontaneous SDL_QUIT / window close). SDL_Quit() in the
        // renderer's shutdown path will clean up the subsystem when
        // the app actually exits.
    }
}

void SdlAudioOutput::flush() {
    if (!device_) return;
    SDL_PauseAudioDevice(device_, 1);
    SDL_ClearQueuedAudio(device_);
    playing_ = false;
}

void SdlAudioOutput::push(const int16_t* samples, int count) {
    if (!device_ || !samples || count <= 0) return;

    const uint32_t queued_before = SDL_GetQueuedAudioSize(device_);
    if (playing_ && queued_before == 0) {
        // Rebuffer after an underrun instead of repeatedly playing one frame
        // followed by silence while the network/decoder catches up.
        SDL_PauseAudioDevice(device_, 1);
        playing_ = false;
        LOG_WARN << "audio output underrun; rebuilding jitter buffer";
    }

    const uint32_t max_bytes = bytes_per_second_ * kMaxBufferMs / 1000;
    if (max_bytes && queued_before > max_bytes) {
        // A suspended/default-device transition can stop SDL consuming data.
        // Do not retain seconds of stale PCM and then appear to recover late.
        SDL_ClearQueuedAudio(device_);
        SDL_PauseAudioDevice(device_, 1);
        playing_ = false;
        LOG_WARN << "audio output backlog exceeded " << kMaxBufferMs
                 << " ms; dropping stale PCM and resynchronizing";
    }

    const float gain = gain_.load(std::memory_order_relaxed);

    // Fast path: unity gain → send as-is.
    if (gain >= 0.9999f && gain <= 1.0001f) {
        const uint32_t bytes = static_cast<uint32_t>(count) * sizeof(int16_t);
        if (SDL_QueueAudio(device_, samples, bytes) != 0) {
            LOG_WARN << "SDL_QueueAudio failed: " << SDL_GetError();
        }
    } else {
        // Apply gain into a scratch buffer, saturate to int16 range.
        static thread_local std::vector<int16_t> scratch;
        scratch.resize(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i) {
            float s = static_cast<float>(samples[i]) * gain;
            if (s >  32767.f) s =  32767.f;
            if (s < -32768.f) s = -32768.f;
            scratch[i] = static_cast<int16_t>(s);
        }
        const uint32_t bytes = static_cast<uint32_t>(count) * sizeof(int16_t);
        if (SDL_QueueAudio(device_, scratch.data(), bytes) != 0) {
            LOG_WARN << "SDL_QueueAudio failed: " << SDL_GetError();
        }
    }

    const uint32_t start_bytes = bytes_per_second_ * kStartBufferMs / 1000;
    if (!playing_ && SDL_GetQueuedAudioSize(device_) >= start_bytes) {
        SDL_PauseAudioDevice(device_, 0);
        playing_ = true;
    }
}

void SdlAudioOutput::set_volume_db(float db) {
    // AirPlay: 0 dB = full, -144 dB = mute. Anything below -100 treat as 0.
    float lin = (db <= -100.f) ? 0.f
              : (db >=    0.f) ? 1.f
              : std::pow(10.f, db / 20.f);
    gain_.store(lin, std::memory_order_relaxed);
    LOG_INFO << "audio volume: " << db << " dB (gain=" << lin << ')';
}

uint32_t SdlAudioOutput::queued_bytes() const {
    return device_ ? SDL_GetQueuedAudioSize(device_) : 0u;
}

} // namespace ap::audio
