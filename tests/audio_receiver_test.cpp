#include "audio/audio_receiver.h"
#include "video/video_renderer.h"
#include "video/video_recorder.h"
#define SDL_MAIN_HANDLED
#include <SDL.h>

#include <chrono>
#include <algorithm>
#include <openssl/evp.h>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>
extern "C" {
#include <libavcodec/avcodec.h>
}

static std::mutex pcm_mutex;
static std::vector<int16_t> received_pcm;
static std::atomic<bool> playback_active{true};
// The test replaces only the UI sink. UDP polling, parsing, CBC, ordering,
// FFmpeg decoding and SDL output all use the production implementations.
namespace ap::video {
VideoRenderer::VideoRenderer() = default;
VideoRenderer::~VideoRenderer() = default;
DecodedFrame::~DecodedFrame() { if (frame) av_frame_free(&frame); }
void VideoRenderer::push_audio_pcm(const int16_t* pcm, int count, int, int) {
    std::lock_guard<std::mutex> lock(pcm_mutex);
    received_pcm.insert(received_pcm.end(), pcm, pcm + count);
}
void VideoRenderer::push_playback_rate(float rate) { playback_active.store(rate > 0.5f); }
bool VideoRenderer::in_flush_grace() const { return false; }
}
static void check(bool ok, const char* what) {
    if (!ok) { std::fprintf(stderr, "%s\n", what); std::exit(1); }
}
static socket_t bind_local(sockaddr_in& address, bool dual = false) {
    socket_t sock = ::socket(dual ? AF_INET6 : AF_INET, SOCK_DGRAM, 0);
    check(sock != INVALID_SOCK, "create UDP socket");
    address = {}; address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sockaddr_in6 address6{};
    address6.sin6_family = AF_INET6;
    if (dual) {
        int only = 0;
        check(setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY,
              reinterpret_cast<const char*>(&only), sizeof(only)) == 0, "dual stack");
        check(::bind(sock, reinterpret_cast<sockaddr*>(&address6), sizeof(address6)) == 0, "bind dual UDP");
    } else {
        check(::bind(sock, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0, "bind UDP");
    }
#if defined(_WIN32)
    int size = sizeof(address);
#else
    socklen_t size = sizeof(address);
#endif
    if (dual) {
        size = sizeof(address6);
        check(getsockname(sock, reinterpret_cast<sockaddr*>(&address6), &size) == 0, "dual UDP endpoint");
        address.sin_port = address6.sin6_port;
    } else {
        check(getsockname(sock, reinterpret_cast<sockaddr*>(&address), &size) == 0, "UDP endpoint");
    }
    return sock;
}
static std::vector<uint8_t> encode(int value) {
    auto codec = avcodec_find_encoder(AV_CODEC_ID_ALAC);
    auto ctx = avcodec_alloc_context3(codec);
    check(ctx != nullptr, "ALAC encoder available");
    ctx->sample_rate = 44100; ctx->sample_fmt = AV_SAMPLE_FMT_S16P;
    av_channel_layout_default(&ctx->ch_layout, 2);
    check(avcodec_open2(ctx, codec, nullptr) == 0, "open ALAC encoder");
    auto frame = av_frame_alloc();
    frame->format = ctx->sample_fmt; frame->sample_rate = 44100; frame->nb_samples = 352;
    av_channel_layout_copy(&frame->ch_layout, &ctx->ch_layout);
    check(av_frame_get_buffer(frame, 0) == 0, "allocate frame");
    for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 352; ++i)
        reinterpret_cast<int16_t*>(frame->data[ch])[i] = static_cast<int16_t>(value);
    check(avcodec_send_frame(ctx, frame) == 0, "encode ALAC");
    auto pkt = av_packet_alloc();
    check(avcodec_receive_packet(ctx, pkt) == 0 && pkt->size < 100, "small ALAC packet");
    std::vector<uint8_t> result(pkt->data, pkt->data + pkt->size);
    av_packet_free(&pkt); av_frame_free(&frame); avcodec_free_context(&ctx);
    return result;
}
static std::vector<uint8_t> rtp(uint16_t seq, uint32_t ts, int value) {
    auto payload = encode(value);
    // Encrypt complete blocks, with the same independent zero IV for each packet.
    const int encrypted = static_cast<int>(payload.size() / 16 * 16);
    uint8_t key[16]{};
    auto aes = EVP_CIPHER_CTX_new();
    check(EVP_EncryptInit_ex(aes, EVP_aes_128_cbc(), nullptr, key, key) == 1, "CBC init");
    EVP_CIPHER_CTX_set_padding(aes, 0);
    std::vector<uint8_t> ciphertext(payload.size());
    int length = 0;
    check(EVP_EncryptUpdate(aes, ciphertext.data(), &length, payload.data(), encrypted) == 1,
          "CBC encrypt");
    std::copy(payload.begin() + encrypted, payload.end(), ciphertext.begin() + length);
    EVP_CIPHER_CTX_free(aes);
    std::vector<uint8_t> bytes = {0x80,0xe0,static_cast<uint8_t>(seq >> 8),static_cast<uint8_t>(seq),
        static_cast<uint8_t>(ts >> 24),static_cast<uint8_t>(ts >> 16),
        static_cast<uint8_t>(ts >> 8),static_cast<uint8_t>(ts),0,0,0,1};
    bytes.insert(bytes.end(), ciphertext.begin(), ciphertext.end());
    return bytes;
}
static bool use_ipv6 = false;
static void send_packet(socket_t sock, const sockaddr_in& destination, const std::vector<uint8_t>& data) {
    sockaddr_in6 destination6{};
    destination6.sin6_family = AF_INET6;
    destination6.sin6_port = destination.sin_port;
    inet_pton(AF_INET6, "::1", &destination6.sin6_addr);
    const auto* address = use_ipv6 ? reinterpret_cast<const sockaddr*>(&destination6)
                                  : reinterpret_cast<const sockaddr*>(&destination);
    const int length = use_ipv6 ? sizeof(destination6) : sizeof(destination);
    check(sendto(sock, reinterpret_cast<const char*>(data.data()), static_cast<int>(data.size()), 0,
          address, length) == static_cast<int>(data.size()),
          "send packet");
}
static void wait_samples(std::size_t count, int timeout_ms = 2000) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        { std::lock_guard<std::mutex> lock(pcm_mutex); if (received_pcm.size() >= count) return; }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(false, "PCM receive deadline");
}
int main(int argc, char** argv) {
    const bool dual = argc > 1;
    const bool v6 = argc > 1 && std::string(argv[1]) == "--ipv6";
    SDL_SetMainReady();
    use_ipv6 = v6;
    check(ap::net::global_init(), "Winsock init");
    sockaddr_in data_address{}, control_address{}, sender_address{};
    auto data = bind_local(data_address, dual);
    auto control = bind_local(control_address, dual);
    auto sender = bind_local(sender_address, v6);
    ap::video::VideoRenderer sink;
    ap::audio::AudioReceiver receiver;
    ap::audio::AudioReceiver::Config config;
    config.data_sock = data; config.control_sock = control;
    config.remote_ip = v6 ? "::1" : "127.0.0.1";
    config.remote_control_port = ntohs(sender_address.sin_port);
    config.aes_key.assign(16, 0); config.aes_iv.assign(16, 0);
    config.ct = 2; config.renderer = &sink;
    check(receiver.start(std::move(config)), "start receiver");
    auto first = rtp(65534, 0xfffffe00u, 10);
    auto second = rtp(65535, 0xfffffe00u + 352u, 20);
    auto third = rtp(0, 0xfffffe00u + 704u, 30);
    send_packet(sender, data_address, first);
    send_packet(sender, data_address, first);
    wait_samples(704);
    send_packet(sender, data_address, third);
    fd_set readable; FD_ZERO(&readable); FD_SET(sender, &readable);
    timeval timeout{2, 0};
    check(select(static_cast<int>(sender + 1), &readable, nullptr, nullptr, &timeout) == 1,
          "missing packet resend request not received");
    uint8_t request[8];
    check(recvfrom(sender, reinterpret_cast<char*>(request), sizeof(request), 0, nullptr, nullptr) == 8,
          "RAOP resend request size");
    check(request[0] == 0x80 && request[1] == 0xd5 && request[4] == 0xff &&
          request[5] == 0xff && request[6] == 0 && request[7] == 1, "RAOP resend sequence/count");
    std::vector<uint8_t> retransmit = {0x80,0xd6,0,1};
    retransmit.insert(retransmit.end(), second.begin(), second.end());
    send_packet(sender, control_address, retransmit);
    send_packet(sender, control_address, retransmit);
    wait_samples(704 * 3);
    // Lose seq 1 permanently; seq 2 must resume after the resend deadline,
    // with exactly one frame of silence preserving the RTP timeline.
    send_packet(sender, data_address, rtp(2, 0xfffffe00u + 352u * 4, 50));
    wait_samples(704 * 5);
    // Simulate a long keepalive-only pause: the RTP clock advances and the
    // audio sequence has moved past half the modular range without a FLUSH.
    const auto resumed = rtp(40000, 2000000, 55);
    std::this_thread::sleep_for(std::chrono::milliseconds(520));
    send_packet(sender, data_address, resumed);
    wait_samples(704 * 6);

    // A track change can move the sequence base by more than half its range.
    // Previously all resumed packets would be rejected as late until wrap.
    receiver.flush(20000);
    send_packet(sender, data_address, rtp(2, 0xfffffe00u + 352u * 4, 50)); // stale
    send_packet(sender, data_address, rtp(20000, 10000, 60));
    wait_samples(704 * 7);
    // Consecutive losses must not accumulate 60 ms per missing sequence.
    // The old per-gap clock took at least 660 ms to drain these 12 frames.
    constexpr int burst_frames = 12;
    std::vector<std::vector<uint8_t>> burst;
    for (int i = 0; i < burst_frames; ++i)
        burst.push_back(rtp(static_cast<uint16_t>(20002 + i * 2),
                            10000 + 352 * (2 + i * 2), 70 + i));
    for (const auto& bytes : burst) send_packet(sender, data_address, bytes);
    wait_samples(704 * (7 + burst_frames * 2), 400);
    // Valid small silence must be decoded, but must not undo an explicit
    // pause; a quiet, nonzero small frame still indicates resumed playback.
    sink.push_playback_rate(0.0f);
    receiver.flush(21000);
    send_packet(sender, data_address, rtp(21000, 0, 0));
    wait_samples(704 * (8 + burst_frames * 2));
    check(!playback_active.load(), "small silent frame undid pause state");
    send_packet(sender, data_address, rtp(21001, 352, 1));
    wait_samples(704 * (9 + burst_frames * 2));
    check(playback_active.load(), "small quiet frame did not resume playback");
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const auto stopping = std::chrono::steady_clock::now();
    receiver.stop();
    check(std::chrono::steady_clock::now() - stopping < std::chrono::milliseconds(150),
          "idle stop did not wake the receiver");
    receiver.stop(); // repeated stop must not close recycled descriptors
    {
        std::lock_guard<std::mutex> lock(pcm_mutex);
        check(received_pcm.size() == 704 * (9 + burst_frames * 2), "duplicate replay or incorrect loss duration");
        const int expected[] = {10, 20, 30, 0, 50, 55, 60};
        for (int block = 0; block < 7; ++block) for (int i = 0; i < 704; ++i)
            check(received_pcm[block * 704 + i] == expected[block], "PCM order/content after resend");
        for (int block = 0; block < burst_frames * 2; ++block)
            for (int i = 0; i < 704; ++i)
                check(received_pcm[(7 + block) * 704 + i] == (block % 2 ? 70 + block / 2 : 0),
                      "consecutive-loss PCM order or timeline mismatch");
        for (int block = 0; block < 2; ++block) for (int i = 0; i < 704; ++i)
            check(received_pcm[(7 + burst_frames * 2 + block) * 704 + i] == block,
                  "small silence/quiet PCM was dropped");
    }
    ap::net::close_socket(sender);
    ap::net::global_shutdown();
    SDL_Quit();
    std::puts("UDP/CBC/small ALAC/retransmission/loss recovery and teardown passed");
}
