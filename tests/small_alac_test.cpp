#include "audio/aac_decoder.h"
#include "audio/rtp_audio_buffer.h"

#include <cstdio>
#include <cstdlib>
extern "C" {
#include <libavcodec/avcodec.h>
}
static void check(bool ok, const char* what) {
    if (!ok) { std::fprintf(stderr, "%s\n", what); std::exit(1); }
}
int main() {
    auto encoder = avcodec_alloc_context3(avcodec_find_encoder(AV_CODEC_ID_ALAC));
    check(encoder != nullptr, "ALAC encoder unavailable");
    encoder->sample_rate = 44100;
    encoder->sample_fmt = AV_SAMPLE_FMT_S16P;
    av_channel_layout_default(&encoder->ch_layout, 2);
    check(avcodec_open2(encoder, encoder->codec, nullptr) == 0, "open encoder");
    auto frame = av_frame_alloc();
    frame->format = encoder->sample_fmt;
    frame->sample_rate = encoder->sample_rate;
    frame->nb_samples = 352;
    av_channel_layout_copy(&frame->ch_layout, &encoder->ch_layout);
    check(av_frame_get_buffer(frame, 0) == 0, "allocate audio frame");
    // Quiet but nonzero audio, legitimately much smaller than 100 bytes.
    for (int ch = 0; ch < 2; ++ch)
        for (int i = 0; i < 352; ++i)
            reinterpret_cast<int16_t*>(frame->data[ch])[i] = 1;
    check(avcodec_send_frame(encoder, frame) == 0, "encode frame");
    auto encoded = av_packet_alloc();
    check(avcodec_receive_packet(encoder, encoded) == 0, "receive ALAC frame");
    check(encoded->size < 100, "expected highly compressed small ALAC frame");
    std::vector<uint8_t> bytes(12, 0);
    bytes[0] = 0x80; bytes[1] = 0x60;
    bytes.insert(bytes.end(), encoded->data, encoded->data + encoded->size);
    ap::audio::RtpAudioPacket rtp;
    check(ap::audio::parse_audio_rtp(bytes.data(), bytes.size(), rtp), "small RTP frame rejected");
    ap::audio::AacDecoder decoder;
    ap::audio::AacDecoder::Config cfg;
    cfg.ct = 2; cfg.sample_rate = 44100; cfg.channels = 2;
    check(decoder.init(cfg), "open ALAC decoder");
    check(decoder.decode(rtp.payload.data(), static_cast<int>(rtp.payload.size())) == 704,
          "small frame did not preserve sample count");
    int16_t pcm[704];
    check(decoder.pull_pcm_s16(pcm, 704) == 704, "PCM sample count");
    for (int16_t sample : pcm) check(sample == 1, "small frame PCM mismatch");
    std::printf("Decoded %d-byte ALAC frame: 352 stereo sample frames preserved\n", encoded->size);
    av_packet_free(&encoded);
    av_frame_free(&frame);
    avcodec_free_context(&encoder);
}
