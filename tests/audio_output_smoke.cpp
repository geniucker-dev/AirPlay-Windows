#include "audio/audio_output.h"
#define SDL_MAIN_HANDLED
#include <SDL.h>

#include <array>
#include <cstdint>

int main() {
    SDL_SetMainReady();
    ap::audio::SdlAudioOutput output;
    if (!output.start(44100, 2)) return 1;
    std::array<int16_t, 960> silence{};
    const auto push = [&] { output.push(silence.data(), static_cast<int>(silence.size())); };

    // Less than 80 ms must remain buffered even while the dummy device runs.
    push(); push();
    SDL_Delay(60);
    if (output.queued_bytes() != silence.size() * sizeof(int16_t) * 2) return 2;
    for (int i = 0; i < 8; ++i) push();
    SDL_Delay(300);
    if (output.queued_bytes() != 0) return 3; // startup actually resumed consumption

    // An underrun must pause and rebuild the cushion rather than repeatedly
    // consuming one frame followed by silence. Then playback must resume.
    push(); push();
    SDL_Delay(60);
    if (output.queued_bytes() != silence.size() * sizeof(int16_t) * 2) return 4;
    for (int i = 0; i < 8; ++i) push();
    SDL_Delay(300);
    if (output.queued_bytes() != 0) return 5;
    push(); push();
    output.flush();
    if (output.queued_bytes() != 0) return 6;
    push(); push();
    SDL_Delay(60);
    if (output.queued_bytes() != silence.size() * sizeof(int16_t) * 2) return 7;
    output.stop();
    output.stop();
    SDL_Quit();
    return 0;
}
