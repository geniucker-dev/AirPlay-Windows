#include "audio/audio_receiver.h"
#include "log.h"
#include "audio/rtp_audio_buffer.h"
#include <algorithm>
#include <array>
#include "video/video_renderer.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <sstream>

#include <openssl/evp.h>

#if defined(_WIN32)
    #include <winsock2.h>
#else
    #include <sys/socket.h>
    #include <sys/time.h>
    #include <errno.h>
#endif

namespace ap::audio {
namespace {

// Normalise IPv4 and IPv4-mapped IPv6 to the same 16-byte address.
std::array<uint8_t, 16> peer_address(const sockaddr_storage& peer) {
    std::array<uint8_t, 16> address{};
    if (peer.ss_family == AF_INET) {
        address[10] = address[11] = 0xff;
        std::memcpy(address.data() + 12,
                    &reinterpret_cast<const sockaddr_in*>(&peer)->sin_addr, 4);
    } else if (peer.ss_family == AF_INET6) {
        std::memcpy(address.data(),
                    &reinterpret_cast<const sockaddr_in6*>(&peer)->sin6_addr, 16);
    }
    return address;
}

bool control_endpoint(socket_t socket, const std::string& ip, uint16_t port,
                      sockaddr_storage& peer) {
    sockaddr_storage local{};
#if defined(_WIN32)
    int length = sizeof(local);
#else
    socklen_t length = sizeof(local);
#endif
    if (socket == INVALID_SOCK ||
        getsockname(socket, reinterpret_cast<sockaddr*>(&local), &length) != 0)
        return false;
    in_addr v4{};
    if (inet_pton(AF_INET, ip.c_str(), &v4) == 1) {
        if (local.ss_family == AF_INET6) {
            auto* v6 = reinterpret_cast<sockaddr_in6*>(&peer);
            v6->sin6_family = AF_INET6;
            v6->sin6_port = htons(port);
            auto* bytes = reinterpret_cast<uint8_t*>(&v6->sin6_addr);
            bytes[10] = bytes[11] = 0xff;
            std::memcpy(bytes + 12, &v4, 4);
        } else {
            auto* address = reinterpret_cast<sockaddr_in*>(&peer);
            address->sin_family = AF_INET;
            address->sin_port = htons(port);
            address->sin_addr = v4;
        }
        return true;
    }
    auto* v6 = reinterpret_cast<sockaddr_in6*>(&peer);
    if (inet_pton(AF_INET6, ip.c_str(), &v6->sin6_addr) != 1) return false;
    v6->sin6_family = AF_INET6;
    v6->sin6_port = htons(port);
    return true;
}

// A one-byte datagram wakes select for FLUSH/stop. It cannot be parsed as
// RTP or learned as a control endpoint, and needs no extra polling thread.
void wake_socket(socket_t socket) {
    if (socket == INVALID_SOCK) return;
    sockaddr_storage local{};
#if defined(_WIN32)
    int length = sizeof(local);
#else
    socklen_t length = sizeof(local);
#endif
    if (getsockname(socket, reinterpret_cast<sockaddr*>(&local), &length) != 0) return;
    if (local.ss_family == AF_INET6) {
        auto* address = reinterpret_cast<sockaddr_in6*>(&local);
        if (IN6_IS_ADDR_UNSPECIFIED(&address->sin6_addr))
            inet_pton(AF_INET6, "::1", &address->sin6_addr);
    } else if (local.ss_family == AF_INET) {
        auto* address = reinterpret_cast<sockaddr_in*>(&local);
        if (address->sin_addr.s_addr == htonl(INADDR_ANY))
            address->sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    } else return;
    const char byte = 0;
    ::sendto(socket, &byte, 1, 0, reinterpret_cast<const sockaddr*>(&local), length);
}

std::string hex_dump(const unsigned char* b, std::size_t n, std::size_t max = 32) {
    std::ostringstream os;
    for (std::size_t i = 0; i < n && i < max; ++i) {
        char tmp[4];
        std::snprintf(tmp, sizeof(tmp), "%02x ", b[i]);
        os << tmp;
    }
    if (n > max) os << "...(" << n << "B)";
    return os.str();
}

} // namespace

// RAOP compression-type values, from UxPlay/global.h and observed sessions.
const char* ct_name(int ct) {
    switch (ct) {
        case 0: return "unspecified";
        case 1: return "PCM";
        case 2: return "ALAC";
        case 3: return "AAC-LC";
        case 4: return "AAC-ELD";      // common on AirPlay 2 (Apple Music)
        case 8: return "AAC-ELD 44.1k";
        default: return "unknown";
    }
}

AudioReceiver::AudioReceiver()  = default;
AudioReceiver::~AudioReceiver() { stop(); }

bool AudioReceiver::start(Config cfg) {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    if (running_.load()) return false;
    if (cfg.data_sock == INVALID_SOCK) return false;
    if (cfg.aes_key.size() != 16 || cfg.aes_iv.size() != 16) {
        LOG_ERROR << "AudioReceiver: aes_key/aes_iv must be 16 B "
                  << "(got " << cfg.aes_key.size() << '/' << cfg.aes_iv.size() << ')';
        return false;
    }

    cfg_ = std::move(cfg);
    {
        std::lock_guard<std::mutex> lock(flush_mutex_);
        flush_pending_.store(false, std::memory_order_relaxed);
    }

    aes_ctx_ = EVP_CIPHER_CTX_new();
    if (!aes_ctx_ ||
        EVP_DecryptInit_ex(aes_ctx_, EVP_aes_128_cbc(), nullptr,
                           cfg_.aes_key.data(), cfg_.aes_iv.data()) != 1) {
        LOG_ERROR << "AudioReceiver: EVP_DecryptInit_ex(aes-128-cbc) failed";
        if (aes_ctx_) { EVP_CIPHER_CTX_free(aes_ctx_); aes_ctx_ = nullptr; }
        return false;
    }
    EVP_CIPHER_CTX_set_padding(aes_ctx_, 0);

    // Select ALAC or AAC using the negotiated compression type.
    decoder_ = std::make_unique<AacDecoder>();
    AacDecoder::Config dc;
    dc.ct          = cfg_.ct;
    dc.sample_rate = cfg_.sample_rate;
    dc.channels    = 2;
    dc.spf         = cfg_.spf;
    if (!decoder_->init(dc)) {
        LOG_WARN << "AudioReceiver: AAC decoder init failed — running in "
                    "decrypt-only mode";
        decoder_.reset();
    }

    // SDL-backed audio sink. Goes live only if the OS gave us a device;
    // a headless Linux VM will fall back to silent decode-only mode.
    if (decoder_) {
        output_ = std::make_unique<SdlAudioOutput>();
        if (!output_->start(cfg_.sample_rate, 2)) {
            LOG_WARN << "AudioReceiver: SDL audio output unavailable — "
                        "PCM will be decoded but not played";
            output_.reset();
        }
    }

    running_ = true;
    thread_  = std::thread(&AudioReceiver::thread_fn, this);
    LOG_INFO << "AudioReceiver listening (ct=" << cfg_.ct
             << ' ' << ct_name(cfg_.ct)
             << ", sample_rate=" << cfg_.sample_rate << ')';
    return true;
}

void AudioReceiver::set_volume_db(float db) {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    if (output_) output_->set_volume_db(db);
}

void AudioReceiver::flush(int next_sequence) {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    if (!running_.load()) return;
    {
        std::lock_guard<std::mutex> lock(flush_mutex_);
        flush_sequence_ = next_sequence >= 0 && next_sequence <= 65535 ? next_sequence : -1;
        flush_pending_.store(true, std::memory_order_release);
    }
    wake_socket(cfg_.control_sock != INVALID_SOCK ? cfg_.control_sock : cfg_.data_sock);
}

void AudioReceiver::stop() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    if (!running_.exchange(false)) return;

    // Wake the receiver, then join before changing or closing sockets.
    // This avoids descriptor reuse races and polling while no audio arrives.
    wake_socket(cfg_.control_sock != INVALID_SOCK ? cfg_.control_sock : cfg_.data_sock);
    if (thread_.joinable()) thread_.join();
    for (auto* socket : {&cfg_.data_sock, &cfg_.control_sock}) {
        if (*socket != INVALID_SOCK) ap::net::close_socket(*socket);
        *socket = INVALID_SOCK;
    }

    if (output_)  { output_->stop();  output_.reset();  }
    if (decoder_) {                    decoder_.reset(); }

    if (aes_ctx_) {
        EVP_CIPHER_CTX_free(aes_ctx_);
        aes_ctx_ = nullptr;
    }
}

void AudioReceiver::thread_fn() {
    RtpAudioBuffer buffer;
    uint64_t received = 0, duplicates = 0;
    uint16_t request_id = 0;
    int64_t last_request = -20;
    bool audio_ever_seen = false;
    auto last_audio = std::chrono::steady_clock::now();
    sockaddr_storage control_peer{};
    const socket_t peer_socket = cfg_.control_sock != INVALID_SOCK
        ? cfg_.control_sock : cfg_.data_sock;
    const bool have_address = control_endpoint(peer_socket, cfg_.remote_ip,
                                               cfg_.remote_control_port, control_peer);
    bool have_peer = have_address && cfg_.remote_control_port;
    bool have_timestamp = false;
    uint32_t next_timestamp = 0;
    auto now_ms = [] {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };
    auto emit_pcm = [&](const int16_t* pcm, int count) {
        if (cfg_.renderer)
            cfg_.renderer->push_audio_pcm(pcm, count, cfg_.sample_rate, 2);
        if (output_) output_->push(pcm, count);
    };

    auto apply_flush = [&] {
        if (!flush_pending_.load(std::memory_order_acquire)) return;
        int next_sequence = -1;
        {
            std::lock_guard<std::mutex> lock(flush_mutex_);
            if (!flush_pending_.load(std::memory_order_relaxed)) return;
            next_sequence = flush_sequence_;
            flush_pending_.store(false, std::memory_order_relaxed);
        }
        buffer.reset(next_sequence, now_ms());
        if (decoder_) decoder_->flush();
        if (output_) output_->flush();
        have_timestamp = false;
        audio_ever_seen = false;
        last_request = -20;
    };

    std::vector<uint8_t> cleartext; // reuse storage on the packet hot path
    while (running_.load()) {
        apply_flush();
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(cfg_.data_sock, &readable);
        socket_t largest = cfg_.data_sock;
        if (cfg_.control_sock != INVALID_SOCK) {
            FD_SET(cfg_.control_sock, &readable);
            largest = std::max(largest, cfg_.control_sock);
        }
        // Normal packets wake select immediately. Poll quickly only while
        // recovering a gap; FLUSH and stop have an explicit socket wakeup.
        timeval timeout{0, buffer.waiting_for_gap() ? 10000 : 200000};
#if defined(_WIN32)
        const int nfds = 0; // Winsock ignores nfds; SOCKET need not fit in int.
#else
        const int nfds = largest + 1;
#endif
        const int ready = ::select(nfds, &readable,
                                   nullptr, nullptr, &timeout);
        if (ready < 0) {
#if defined(_WIN32)
            if (ap::net::last_error() == WSAEINTR) continue;
#else
            if (ap::net::last_error() == EINTR) continue;
#endif
            LOG_WARN << "audio socket select failed: " << ap::net::last_error_string();
            break;
        }
        // FLUSH may arrive while select waits; apply it before reading the
        // next packet so a queued old sequence cannot re-enter the timeline.
        apply_flush();
        if (ready > 0) {
            for (const auto socket : {cfg_.data_sock, cfg_.control_sock}) {
                if (socket == INVALID_SOCK || !FD_ISSET(socket, &readable)) continue;
                uint8_t bytes[4096];
                sockaddr_storage peer{};
#if defined(_WIN32)
                int peer_len = sizeof(peer);
#else
                socklen_t peer_len = sizeof(peer);
#endif
                const int size = ::recvfrom(socket, reinterpret_cast<char*>(bytes),
                    sizeof(bytes), 0, reinterpret_cast<sockaddr*>(&peer), &peer_len);
                if (size < 0) continue;
                if (have_address && peer_address(peer) != peer_address(control_peer))
                    continue;
                // Link-local IPv6 needs the incoming interface scope before
                // any control traffic arrives; the TCP peer string has no scope.
                if (have_address && peer.ss_family == AF_INET6 && control_peer.ss_family == AF_INET6) {
                    auto* endpoint = reinterpret_cast<sockaddr_in6*>(&control_peer);
                    if (!endpoint->sin6_scope_id)
                        endpoint->sin6_scope_id = reinterpret_cast<const sockaddr_in6*>(&peer)->sin6_scope_id;
                }
                // Learn the actual control endpoint (including NAT mappings)
                // only from the negotiated sender's address.
                if (socket == cfg_.control_sock && size >= 16 &&
                    (bytes[0] >> 6) == 2 &&
                    (((bytes[1] & 0x7f) == 0x54 && size >= 20) || (bytes[1] & 0x7f) == 0x56) &&
                    have_address && peer_address(peer) == peer_address(control_peer)) {
                    control_peer = peer;
                    have_peer = true;
                }
                RtpAudioPacket packet;
                if (!parse_audio_rtp(bytes, static_cast<std::size_t>(size), packet)) continue;
                ++received;
                const uint32_t elapsed_samples = packet.timestamp - next_timestamp;
                const bool retransmission = (bytes[1] & 0x7f) == 0x56;
                if (!retransmission && have_timestamp && elapsed_samples < 0x80000000u &&
                    elapsed_samples >= static_cast<uint32_t>(cfg_.sample_rate / 2) &&
                    std::chrono::steady_clock::now() - last_audio >= std::chrono::milliseconds(500)) {
                    // Empty keepalives can advance sequence numbers during a
                    // long pause. Once more than half the 16-bit range passed,
                    // modular sequence comparison alone calls new audio late.
                    // A forward RTP timestamp after inactivity establishes a
                    // new timeline; old retransmissions cannot trigger this.
                    buffer.reset();
                    if (decoder_) decoder_->flush();
                    if (output_) output_->flush();
                    have_timestamp = false;
                }
                if (!buffer.enqueue(std::move(packet), now_ms())) ++duplicates;
            }
        }

        const int64_t now = now_ms();
        uint16_t missing_seq = 0, missing_count = 0;
        if (have_peer && cfg_.control_sock != INVALID_SOCK &&
            buffer.missing(missing_seq, missing_count) && now - last_request >= 20) {
            const uint8_t request[] = {0x80, 0xd5,
                static_cast<uint8_t>(request_id >> 8), static_cast<uint8_t>(request_id),
                static_cast<uint8_t>(missing_seq >> 8), static_cast<uint8_t>(missing_seq),
                static_cast<uint8_t>(missing_count >> 8), static_cast<uint8_t>(missing_count)};
            ++request_id;
            ::sendto(cfg_.control_sock, reinterpret_cast<const char*>(request),
                     sizeof(request), 0, reinterpret_cast<const sockaddr*>(&control_peer),
                     control_peer.ss_family == AF_INET6 ? sizeof(sockaddr_in6) : sizeof(sockaddr_in));
            last_request = now;
        }

        RtpAudioPacket packet;
        while (buffer.pop(now, packet)) {
            // CBC applies only to complete blocks; the remainder is plaintext.
            cleartext.resize(packet.payload.size());
            const int encrypted = static_cast<int>(packet.payload.size() / 16 * 16);
            int length = 0;
            if (EVP_DecryptInit_ex(aes_ctx_, nullptr, nullptr, nullptr,
                                  cfg_.aes_iv.data()) != 1 ||
                (encrypted && EVP_DecryptUpdate(aes_ctx_, cleartext.data(), &length,
                                               packet.payload.data(), encrypted) != 1))
                continue;
            const std::size_t tail = packet.payload.size() - encrypted;
            std::memcpy(cleartext.data() + length, packet.payload.data() + encrypted, tail);
            length += static_cast<int>(tail);
            if (!decoder_) continue;
            const int decoded = decoder_->decode(cleartext.data(), length);
            if (decoded <= 0) continue;

            // Preserve short lost intervals in the RTP timeline after the
            // resend deadline. Large discontinuities start a new timeline.
            const uint32_t gap = packet.timestamp - next_timestamp;
            if (have_timestamp && gap > 0 && gap <= static_cast<uint32_t>(cfg_.sample_rate / 2)) {
                int16_t silence[2048]{};
                uint32_t remaining = gap * 2;
                while (remaining) {
                    const int count = static_cast<int>(std::min<uint32_t>(remaining, 2048));
                    emit_pcm(silence, count);
                    remaining -= count;
                }
            }
            next_timestamp = packet.timestamp + static_cast<uint32_t>(decoded / 2);
            have_timestamp = true;
            int16_t pcm[8192];
            int count = 0;
            bool activity = packet.payload.size() >= 100;
            bool reported_activity = false;
            while ((count = decoder_->pull_pcm_s16(pcm, 8192)) > 0) {
                // iOS emits tiny all-zero codec frames while paused. Keep
                // their sample time, without using them to undo a pause.
                // Quiet nonzero audio counts regardless of compressed size.
                if (!activity) {
                    for (int i = 0; i < count; ++i) {
                        if (pcm[i] != 0) { activity = true; break; }
                    }
                }
                if (activity && !reported_activity) {
                    last_audio = std::chrono::steady_clock::now();
                    audio_ever_seen = true;
                    if (cfg_.renderer && !cfg_.renderer->in_flush_grace())
                        cfg_.renderer->push_playback_rate(1.0f);
                    reported_activity = true;
                }
                emit_pcm(pcm, count);
            }
            if (decoder_->frames_decoded() <= 3)
                LOG_INFO << "audio seq=" << packet.sequence << " decoded=" << decoded
                         << " payload=" << length << " clear="
                         << hex_dump(cleartext.data(), cleartext.size());
        }
        if (cfg_.renderer && audio_ever_seen &&
            std::chrono::steady_clock::now() - last_audio >= std::chrono::milliseconds(500))
            cfg_.renderer->push_playback_rate(0.0f);
    }
    LOG_INFO << "AudioReceiver stopped (" << received << " audio packets, "
             << duplicates << " duplicate/late packets, "
             << (decoder_ ? decoder_->frames_decoded() : 0) << " PCM frames decoded)";
}

} // namespace ap::audio
