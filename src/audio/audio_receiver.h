#pragma once

#include "audio/aac_decoder.h"
#include "audio/audio_output.h"
#include "net/socket.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Forward-declare OpenSSL type to keep this header self-contained.
struct evp_cipher_ctx_st;
typedef struct evp_cipher_ctx_st EVP_CIPHER_CTX;

namespace ap::video { class VideoRenderer; }

namespace ap::audio {

// UDP audio receiver for the AirPlay RAOP stream (type 96).
//
// Ported from UxPlay's raop_buffer.c (GPL-3.0). Packets arrive on the
// UDP data port negotiated during SETUP. Each packet is:
//
//   [12 bytes RTP header][AES-128-CBC encrypted payload]
//
// The AES key/IV come from SETUP (the same `aes_key` post-hashed against
// the ECDH secret, and `aes_iv` as-is). For every packet, the AES context
// is re-initialised with the SAME iv (UxPlay calls this "aes_cbc_reset"
// after each decrypt) so packets stay independent — trailing bytes that
// don't fit into a full 16-byte block are copied unchanged.
//
// Valid codec frames are reordered, deduplicated and decoded to SDL PCM.
class AudioReceiver {
public:
    struct Config {
        socket_t                    data_sock = INVALID_SOCK; // owned after successful start
        socket_t                    control_sock = INVALID_SOCK; // same ownership
        std::string                 remote_ip;
        uint16_t                    remote_control_port = 0;
        int                         spf = 480;
        std::vector<unsigned char>  aes_key;                   // 16 B
        std::vector<unsigned char>  aes_iv;                    // 16 B
        int                         ct          = 0;           // compression type
        int                         sample_rate = 44100;
        // Non-owning. When set, the receiver acts as a play/pause
        // watchdog: it pushes rate 1 on decoded audio and rate 0 after
        // ~500 ms of silence. Apple Music and many iOS apps signal
        // pause solely by stopping the RTP flow — no RTSP verb or
        // text/parameters rate: update is sent — so this is the only
        // reliable way to drive the UI pause state.
        ap::video::VideoRenderer*   renderer    = nullptr;
    };

    AudioReceiver();
    ~AudioReceiver();

    AudioReceiver(const AudioReceiver&)            = delete;
    AudioReceiver& operator=(const AudioReceiver&) = delete;

    bool start(Config cfg);
    void stop();
    // Queue a reset on the receiver thread. RTP-Info seq is the next expected
    // sequence; -1 accepts a fresh timeline from the first arriving packet.
    void flush(int next_sequence = -1);

    // Thread-safe: hand to SdlAudioOutput when it exists.
    void set_volume_db(float db);

private:
    void thread_fn();

    std::mutex                  lifecycle_mutex_;
    std::mutex                  flush_mutex_;
    std::atomic<bool>           flush_pending_{false};
    int                         flush_sequence_ = -1;
    Config                      cfg_;
    std::atomic<bool>           running_{false};
    std::thread                 thread_;
    EVP_CIPHER_CTX*                 aes_ctx_{nullptr};
    std::unique_ptr<AacDecoder>     decoder_;
    std::unique_ptr<SdlAudioOutput> output_;
};

// Human-readable label for a RAOP "ct" (compression type) value.
const char* ct_name(int ct);

} // namespace ap::audio
