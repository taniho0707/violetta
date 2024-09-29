//******************************************************************************
// @addtogroup MPL
// @file       mpl_speaker.cpp
// @brief      スピーカー制御
//******************************************************************************
#include "mpl_speaker.h"

#include "hal_speaker.h"
#include "mpl_timer.h"

using namespace mpl;

mpl::Speaker::Speaker() {}

mpl::MplStatus mpl::Speaker::initPort() {
    initMusicToneItem();
    auto status = hal::initSpeakerPort();
    if (status != hal::HalStatus::SUCCESS) {
        return mpl::MplStatus::ERROR;
    } else {
        return mpl::MplStatus::SUCCESS;
    }
}

void mpl::Speaker::deinitPort() {
    hal::deinitSpeakerPort();
}

void Speaker::initMusicToneItem() {
    // clang-format off
#if defined(MOUSE_LAZULI)
    music_tone[static_cast<uint8_t>(MusicTitle::NONE)][0] = MusicToneItem({MusicTone::NONE, 0});
    music_tone[static_cast<uint8_t>(MusicTitle::STARTUP)][0] = MusicToneItem({MusicTone::A5, 100});
    music_tone[static_cast<uint8_t>(MusicTitle::STARTUP)][1] = MusicToneItem({MusicTone::D6, 200});
    music_tone[static_cast<uint8_t>(MusicTitle::STARTUP)][2] = MusicToneItem({MusicTone::NONE, 0});

    music_tone[static_cast<uint8_t>(MusicTitle::SELECT1)][0] = MusicToneItem({MusicTone::D6, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT1)][1] = MusicToneItem({MusicTone::G6S, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT1)][2] = MusicToneItem({MusicTone::C7, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT1)][3] = MusicToneItem({MusicTone::NONE, 0});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT2)][0] = MusicToneItem({MusicTone::D6, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT2)][1] = MusicToneItem({MusicTone::NONE, 100});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT2)][2] = MusicToneItem({MusicTone::D6, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT2)][3] = MusicToneItem({MusicTone::G6S, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT2)][4] = MusicToneItem({MusicTone::C7, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT2)][5] = MusicToneItem({MusicTone::NONE, 0});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT3)][0] = MusicToneItem({MusicTone::D6, 90});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT3)][1] = MusicToneItem({MusicTone::G6S, 90});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT3)][2] = MusicToneItem({MusicTone::C7, 90});
    music_tone[static_cast<uint8_t>(MusicTitle::SELECT3)][3] = MusicToneItem({MusicTone::NONE, 0});
    
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT1)][0] = MusicToneItem({MusicTone::C7, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT1)][1] = MusicToneItem({MusicTone::G6S, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT1)][2] = MusicToneItem({MusicTone::D6, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT1)][3] = MusicToneItem({MusicTone::NONE, 0});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT2)][0] = MusicToneItem({MusicTone::C7, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT2)][1] = MusicToneItem({MusicTone::NONE, 100});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT2)][2] = MusicToneItem({MusicTone::C7, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT2)][3] = MusicToneItem({MusicTone::G6S, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT2)][4] = MusicToneItem({MusicTone::D6, 30});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT2)][5] = MusicToneItem({MusicTone::NONE, 0});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT3)][0] = MusicToneItem({MusicTone::C7, 90});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT3)][1] = MusicToneItem({MusicTone::G6S, 90});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT3)][2] = MusicToneItem({MusicTone::D6, 90});
    music_tone[static_cast<uint8_t>(MusicTitle::DESELECT3)][3] = MusicToneItem({MusicTone::NONE, 0});

    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][0] = MusicToneItem({MusicTone::C6, 100});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][1] = MusicToneItem({MusicTone::E6, 100});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][2] = MusicToneItem({MusicTone::F6, 100});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][3] = MusicToneItem({MusicTone::G6, 50});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][4] = MusicToneItem({MusicTone::NONE, 100});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][5] = MusicToneItem({MusicTone::C7, 100});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][6] = MusicToneItem({MusicTone::NONE, 100});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][7] = MusicToneItem({MusicTone::G6, 100});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][8] = MusicToneItem({MusicTone::NONE, 0});

    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][0] = MusicToneItem({MusicTone::A5, 500});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER1)][1] = MusicToneItem({MusicTone::NONE, 0});

    music_tone[static_cast<uint8_t>(MusicTitle::ENTER2)][0] = MusicToneItem({MusicTone::D6, 500});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER2)][1] = MusicToneItem({MusicTone::NONE, 0});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER3)][0] = MusicToneItem({MusicTone::D7, 500});
    music_tone[static_cast<uint8_t>(MusicTitle::ENTER3)][1] = MusicToneItem({MusicTone::NONE, 0});
    
    music_tone[static_cast<uint8_t>(MusicTitle::RUNSTART)][0] = MusicToneItem({MusicTone::A5, 500});
    music_tone[static_cast<uint8_t>(MusicTitle::RUNSTART)][1] = MusicToneItem({MusicTone::NONE, 0});
    music_tone[static_cast<uint8_t>(MusicTitle::SEARCHCOMPLETE)][0] = MusicToneItem({MusicTone::D6, 500});
    music_tone[static_cast<uint8_t>(MusicTitle::SEARCHCOMPLETE)][1] = MusicToneItem({MusicTone::NONE, 0});

    music_tone[static_cast<uint8_t>(MusicTitle::PULSE_LOW)][0] = MusicToneItem({MusicTone::A5, 200});
    music_tone[static_cast<uint8_t>(MusicTitle::PULSE_LOW)][1] = MusicToneItem({MusicTone::NONE, 0});
    music_tone[static_cast<uint8_t>(MusicTitle::PULSE_HIGH)][0] = MusicToneItem({MusicTone::D6, 200});
    music_tone[static_cast<uint8_t>(MusicTitle::PULSE_HIGH)][1] = MusicToneItem({MusicTone::NONE, 0});

    music_tone[static_cast<uint8_t>(MusicTitle::ERROR)][0] = MusicToneItem({MusicTone::D7, 500});
    music_tone[static_cast<uint8_t>(MusicTitle::ERROR)][1] = MusicToneItem({MusicTone::NONE, 0});
#endif  // MOUSE_LAZULI
    // clang-format on
}

bool mpl::Speaker::isPlaying() {
    return end_time != 0;
}

mpl::MplStatus mpl::Speaker::playFrequencySync(uint16_t freq, uint16_t millisec) {
    hal::setSpeakerFrequency(freq);
    Timer::sleepMs(millisec);
    stop();
    return mpl::MplStatus::SUCCESS;
}

mpl::MplStatus mpl::Speaker::playFrequencyAsync(uint16_t freq, uint16_t millisec) {
    hal::setSpeakerFrequency(freq);
    end_time = Timer::getMicroTime() + millisec * 1000;
    return mpl::MplStatus::SUCCESS;
}

mpl::MplStatus mpl::Speaker::playToneSync(MusicTone tone, uint16_t millisec) {
    return playFrequencySync(static_cast<uint16_t>(tone), millisec);
}

mpl::MplStatus mpl::Speaker::playToneAsync(MusicTone tone, uint16_t millisec) {
    return playFrequencyAsync(static_cast<uint16_t>(tone), millisec);
}

// TODO: implement
mpl::MplStatus Speaker::playMusicSync(MusicTitle title) {
    playMusicAsync(title);
    while (isPlaying());
    return mpl::MplStatus::SUCCESS;
}

mpl::MplStatus Speaker::playMusicAsync(MusicTitle title) {
    music_tone_index = 0;
    if (title == MusicTitle::NONE) {
        stop();
        music_title = MusicTitle::NONE;
        end_time = 0;
        return mpl::MplStatus::INVALID_PARAMS;
    }
    music_title = title;
    end_time = Timer::getMicroTime() + music_tone[static_cast<uint16_t>(title)][0].duration * 1000;
    return mpl::MplStatus::SUCCESS;
}

void mpl::Speaker::stop() {
    hal::offSpeaker();
    music_title = MusicTitle::NONE;
    music_tone_index = 0;
    end_time = 0;
}

void mpl::Speaker::interruptPeriodic() {
    // TODO: command を確認し、あれば処理する
    if (end_time == 0) return;  // no playing
    if (Timer::getMicroTime() >= end_time) {
        if (music_title != MusicTitle::NONE) {
            if (music_tone_index < SPEAKER_MUSIC_TONE_LENGTH) {
                auto tone = music_tone[static_cast<uint16_t>(music_title)][music_tone_index++];
                if (tone.duration == 0) {
                    stop();
                    music_title = MusicTitle::NONE;
                    music_tone_index = 0;
                    end_time = 0;
                } else if (tone.tone == MusicTone::NONE) {
                    hal::offSpeaker();
                    end_time = Timer::getMicroTime() + tone.duration * 1000;
                } else {
                    hal::setSpeakerFrequency(static_cast<uint16_t>(tone.tone));
                    end_time = Timer::getMicroTime() + tone.duration * 1000;
                }
            } else {
                stop();
                music_title = MusicTitle::NONE;
                music_tone_index = 0;
                end_time = 0;
            }
        } else {
            stop();
            end_time = 0;
        }
    }
}

mpl::Speaker* mpl::Speaker::getInstance() {
    static mpl::Speaker instance;
    return &instance;
}
