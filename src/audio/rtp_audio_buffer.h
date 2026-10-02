#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <utility>
#include <vector>

namespace ap::audio {

struct RtpAudioPacket {
    uint16_t sequence = 0;
    uint32_t timestamp = 0;
    std::vector<uint8_t> payload;
    int64_t arrival_ms = 0;
};

// Parse regular audio RTP, including a RAOP retransmission's four-byte wrapper.
// Only the documented empty marker is ignored; compressed quiet audio can be tiny.
inline bool parse_audio_rtp(const uint8_t* data, std::size_t size,
                            RtpAudioPacket& packet) {
    if (!data || size < 12 || (data[0] >> 6) != 2) return false;
    if ((data[1] & 0x7f) == 0x56) {
        data += 4;
        size -= 4;
        if (size < 12 || (data[0] >> 6) != 2) return false;
    }
    if ((data[1] & 0x7f) != 0x60) return false;
    std::size_t offset = 12 + 4 * (data[0] & 0x0f);
    if (offset > size) return false;
    if (data[0] & 0x10) {
        if (size - offset < 4) return false;
        const std::size_t words = (data[offset + 2] << 8) | data[offset + 3];
        offset += 4 + words * 4;
        if (offset > size) return false;
    }
    if (data[0] & 0x20) {
        const std::size_t padding = data[size - 1];
        if (!padding || padding > size - offset) return false;
        size -= padding;
    }
    const uint8_t empty[] = {0x00, 0x68, 0x34, 0x00};
    if (size == offset || (size - offset == sizeof(empty) &&
                          std::memcmp(data + offset, empty, sizeof(empty)) == 0))
        return false;
    packet.sequence = static_cast<uint16_t>((data[2] << 8) | data[3]);
    packet.timestamp = (uint32_t(data[4]) << 24) | (uint32_t(data[5]) << 16) |
                       (uint32_t(data[6]) << 8) | data[7];
    packet.payload.assign(data + offset, data + size);
    return true;
}

// A sequence-based bounded window. Duplicates never advance its lifetime;
// modular distances work across 65535 -> 0 without a session-long bitset.
class RtpAudioBuffer {
public:
    static constexpr int kCapacity = 256;
    static constexpr int64_t kGapWaitMs = 60;

    void reset(int next_sequence = -1, int64_t now_ms = 0) {
        packets_.clear();
        started_ = next_sequence >= 0;
        reset_fence_ = started_;
        flush_fence_ = started_;
        fence_until_ms_ = now_ms + 500;
        next_ = static_cast<uint16_t>(next_sequence);
    }

    bool enqueue(RtpAudioPacket packet, int64_t now_ms = 0) {
        if (!started_) { started_ = true; next_ = packet.sequence; }
        const int distance = delta(packet.sequence, next_);
        if (flush_fence_ && now_ms >= fence_until_ms_) flush_fence_ = false;
        if (distance < 0 && (!reset_fence_ || flush_fence_)) return false;
        if (distance < 0 || distance >= kCapacity) {
            // Fence delayed pre-FLUSH packets briefly, but a mismatched
            // sender sequence must not leave playback permanently silent.
            if (flush_fence_) return false;
            packets_.clear();
            next_ = packet.sequence;
            reset_fence_ = false;
        }
        packet.arrival_ms = now_ms;
        return packets_.emplace(packet.sequence, std::move(packet)).second;
    }

    bool pop(int64_t now_ms, RtpAudioPacket& packet) {
        if (packets_.empty()) return false;
        auto it = packets_.find(next_);
        if (it == packets_.end()) {
            // The oldest waiting packet establishes the deadline. Late resends
            // and consecutive losses must not repeatedly extend that deadline.
            it = closest();
            int64_t oldest_arrival = it->second.arrival_ms;
            for (const auto& entry : packets_)
                if (entry.second.arrival_ms < oldest_arrival)
                    oldest_arrival = entry.second.arrival_ms;
            if (now_ms - oldest_arrival < kGapWaitMs) return false;
            next_ = it->first;
        }
        packet = std::move(it->second);
        packets_.erase(it);
        ++next_;
        reset_fence_ = false;
        return true;
    }

    bool waiting_for_gap() const {
        return !packets_.empty() && !packets_.count(next_);
    }

    bool missing(uint16_t& sequence, uint16_t& count) const {
        if (packets_.empty() || packets_.count(next_)) return false;
        sequence = next_;
        count = static_cast<uint16_t>(delta(closest()->first, next_));
        return count != 0;
    }

private:
    static int delta(uint16_t a, uint16_t b) {
        const unsigned d = static_cast<uint16_t>(a - b);
        return d < 32768 ? static_cast<int>(d) : static_cast<int>(d) - 65536;
    }
    std::map<uint16_t, RtpAudioPacket>::const_iterator closest() const {
        auto best = packets_.begin();
        for (auto it = packets_.begin(); it != packets_.end(); ++it)
            if (delta(it->first, next_) < delta(best->first, next_)) best = it;
        return best;
    }
    std::map<uint16_t, RtpAudioPacket>::iterator closest() {
        auto best = packets_.begin();
        for (auto it = packets_.begin(); it != packets_.end(); ++it)
            if (delta(it->first, next_) < delta(best->first, next_)) best = it;
        return best;
    }
    std::map<uint16_t, RtpAudioPacket> packets_;
    uint16_t next_ = 0;
    bool started_ = false;
    bool reset_fence_ = false;
    bool flush_fence_ = false;
    int64_t fence_until_ms_ = 0;
};

} // namespace ap::audio
