//******************************************************************************
// @addtogroup MPL
// @file       mpl_speaker.h
// @brief      スピーカー制御
//******************************************************************************
#pragma once

#include "mpl_conf.h"

namespace mpl {

enum class MusicTone : uint16_t {
    C3 = 131,
    D3 = 147,
    E3 = 165,
    F3 = 175,
    G3 = 196,
    A3 = 220,
    B3 = 247,
    C4 = 262,
    D4 = 294,
    E4 = 330,
    F4 = 349,
    G4 = 392,
    A4 = 440,
    B4 = 494,
    C5 = 523,
    D5 = 587,
    E5 = 659,
    F5 = 698,
    G5 = 784,
    A5 = 880,
    B5 = 988,
    C6 = 1047,
    D6 = 1175,
    E6 = 1319,
    F6 = 1397,
    G6 = 1568,
    A6 = 1760,
    B6 = 1976,
    C7 = 2093,
    D7 = 2349,
    E7 = 2637,
    F7 = 2794,
    G7 = 3136,
    A7 = 3520,
    B7 = 3951,
    C8 = 4186,
};

class Speaker {
   private:
    Speaker();

    uint16_t freq;
    uint32_t end_time;  // [us], 0 means no-playing

   public:
    mpl::MplStatus initPort();
    void deinitPort();

    bool isPlaying();

    mpl::MplStatus playFrequencySync(uint16_t freq, uint16_t millisec);
    mpl::MplStatus playFrequencyAsync(uint16_t freq, uint16_t millisec);
    mpl::MplStatus playToneSync(MusicTone tone, uint16_t millisec);
    mpl::MplStatus playToneAsync(MusicTone tone, uint16_t millisec);

    void stop();

    // mpl::MplStatus playMusicSync(MusicTitle title);
    // mpl::MplStatus playMusicAsync(MusicTitle title);

    void interruptPeriodic();

    static Speaker* getInstance();
};

}  // namespace mpl
