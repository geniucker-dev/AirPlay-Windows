#include "audio/rtp_audio_buffer.h"

#include <cstdio>
#include <cstdlib>

using namespace ap::audio;
static void check(bool ok, const char* what) {
    if (!ok) { std::fprintf(stderr, "%s\n", what); std::exit(1); }
}
static RtpAudioPacket packet(uint16_t seq) {
    return {seq, uint32_t(seq) * 480, {1, 2, 3}};
}
int main() {
    // Several sequence wraps, including the two-copies pattern that previously
    // permanently blacklisted every new frame after the first wrap.
    for (int copies : {1, 2, 3}) {
        RtpAudioBuffer buffer;
        RtpAudioPacket out;
        for (uint32_t i = 0; i < 65536 * 4; ++i) {
            for (int j = 0; j < copies; ++j)
                check(buffer.enqueue(packet(static_cast<uint16_t>(i))) == (j == 0),
                      "duplicate handling during sequence wrap");
            check(buffer.pop(i, out) && out.sequence == static_cast<uint16_t>(i),
                  "new frame lost during sequence wrap");
            check(!buffer.enqueue(packet(static_cast<uint16_t>(i))), "late replay accepted");
        }
    }
    RtpAudioBuffer buffer;
    RtpAudioPacket out;
    check(buffer.enqueue(packet(65534)) && buffer.pop(0, out), "start near wrap");
    check(buffer.enqueue(packet(0), 10), "enqueue beyond missing wrap packet");
    check(!buffer.pop(10, out), "gap must wait for resend");
    uint16_t seq = 0, count = 0;
    check(buffer.missing(seq, count) && seq == 65535 && count == 1, "wrap resend range");
    check(buffer.enqueue(packet(65535)), "accept retransmission");
    check(buffer.pop(20, out) && out.sequence == 65535, "retransmission order");
    check(buffer.pop(20, out) && out.sequence == 0, "wrapped buffered packet order");
    check(buffer.enqueue(packet(3), 30) && buffer.enqueue(packet(2), 30), "out of order packets");
    check(!buffer.pop(30, out) && !buffer.pop(89, out), "gap deadline too early");
    check(buffer.pop(90, out) && out.sequence == 2, "unrecoverable gap must unblock");
    check(buffer.pop(90, out) && out.sequence == 3, "continue after gap");
    check(!buffer.enqueue(packet(1)), "late missing packet must not replay");
    check(buffer.enqueue(packet(500)) && buffer.pop(100, out) && out.sequence == 500,
          "bounded window resynchronisation");

    buffer.reset(20000);
    check(!buffer.enqueue(packet(500)), "pre-FLUSH packet accepted");
    check(!buffer.enqueue(packet(60000)), "distant pre-FLUSH packet defeated fence");
    check(buffer.enqueue(packet(20000)), "FLUSH first sequence rejected");
    check(!buffer.enqueue(packet(500)), "stale packet overwrote buffered post-FLUSH audio");
    check(buffer.pop(110, out) && out.sequence == 20000,
          "FLUSH sequence base did not recover");
    buffer.reset();
    check(buffer.enqueue(packet(1)) && buffer.pop(120, out) && out.sequence == 1,
          "FLUSH without sequence did not recover");

    // Several distinct missing packets must share the original jitter deadline,
    // rather than each adding another 60 ms while older packets accumulate.
    RtpAudioBuffer burst;
    check(burst.enqueue(packet(0)) && burst.pop(0, out), "burst start");
    check(burst.enqueue(packet(2), 10), "burst seq2");
    check(!burst.pop(10, out), "burst first gap must wait");
    check(burst.enqueue(packet(4), 20) && burst.enqueue(packet(6), 30), "burst arrival");
    check(burst.pop(70, out) && out.sequence == 2, "burst first gap deadline");
    check(burst.pop(80, out) && out.sequence == 4,
          "consecutive gaps accumulated an extra jitter wait");

    check(burst.pop(90, out) && out.sequence == 6, "burst latency grew across gaps");
    buffer.reset(20000, 100);
    check(!buffer.enqueue(packet(21000), 200), "FLUSH fence released too early");
    check(buffer.enqueue(packet(21000), 600) && buffer.pop(600, out),
          "FLUSH fence left mismatched sequence permanently blocked");

    buffer.reset(20000, 100);
    check(!buffer.enqueue(packet(19000), 200), "backward FLUSH fence released too early");
    check(buffer.enqueue(packet(19000), 600) && buffer.pop(600, out),
          "FLUSH fence left a backward sender sequence permanently blocked");

    buffer.reset(20000, 100);
    check(buffer.enqueue(packet(20000), 101) && buffer.pop(101, out), "post-FLUSH first frame");
    check(!buffer.enqueue(packet(50000), 150), "late pre-FLUSH packet replaced resumed timeline");
    check(buffer.enqueue(packet(20001), 151) && buffer.pop(151, out),
          "late pre-FLUSH packet blocked resumed audio");

    RtpAudioBuffer delayed;
    check(delayed.enqueue(packet(0)) && delayed.pop(0, out), "delayed resend start");
    check(delayed.enqueue(packet(6), 10) && !delayed.pop(10, out), "delayed resend first gap");
    check(delayed.enqueue(packet(4), 50) && delayed.enqueue(packet(2), 65), "late closer resends");
    check(delayed.pop(70, out) && out.sequence == 2, "late resend extended jitter deadline");
    check(delayed.pop(70, out) && out.sequence == 4, "second late resend extended deadline");
    check(delayed.pop(70, out) && out.sequence == 6, "oldest waiting frame was stalled");

    std::vector<uint8_t> bytes(12 + 36, 0);
    bytes[0] = 0x80; bytes[1] = 0xe0; bytes[2] = 0xff; bytes[3] = 0xff;
    bytes[7] = 42;
    check(parse_audio_rtp(bytes.data(), bytes.size(), out) && out.payload.size() == 36 &&
          out.sequence == 65535 && out.timestamp == 42, "small compressed audio discarded");
    std::vector<uint8_t> resend = {0x80, 0xd6, 0, 1};
    resend.insert(resend.end(), bytes.begin(), bytes.end());
    check(parse_audio_rtp(resend.data(), resend.size(), out) && out.payload.size() == 36,
          "RAOP retransmit wrapper");
    bytes.resize(16); bytes[12] = 0; bytes[13] = 0x68; bytes[14] = 0x34; bytes[15] = 0;
    check(!parse_audio_rtp(bytes.data(), bytes.size(), out), "empty marker accepted");
    bytes[13] = 0x69;
    check(parse_audio_rtp(bytes.data(), bytes.size(), out), "tiny non-marker frame discarded");
    bytes[1] = 0xd4;
    check(!parse_audio_rtp(bytes.data(), bytes.size(), out), "sync packet treated as audio");
    bytes[1] = 0x60; bytes[0] = 0x81;
    check(!parse_audio_rtp(bytes.data(), bytes.size(), out), "empty CSRC payload");
    bytes.resize(20); bytes[16] = 7;
    check(parse_audio_rtp(bytes.data(), bytes.size(), out) && out.payload.size() == 4,
          "CSRC parsing");
    bytes[0] = 0x90; bytes[14] = 0xff; bytes[15] = 0xff;
    check(!parse_audio_rtp(bytes.data(), bytes.size(), out), "truncated RTP extension accepted");
    bytes[0] = 0xa0; bytes.back() = 2;
    check(parse_audio_rtp(bytes.data(), bytes.size(), out) && out.payload.size() == 6,
          "RTP padding removal");
    bytes.back() = 99;
    check(!parse_audio_rtp(bytes.data(), bytes.size(), out), "invalid padding accepted");
    check(!parse_audio_rtp(nullptr, 20, out), "null input accepted");
    std::puts("RTP wrap, dedup, reorder, resend deadlines and small packets passed");
}
