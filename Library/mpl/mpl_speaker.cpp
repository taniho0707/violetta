//******************************************************************************
// @addtogroup MPL
// @file       mpl_speaker.cpp
// @brief      スピーカー制御
//******************************************************************************
#include "mpl_speaker.h"

#include "hal_speaker.h"
#include "mpl_timer.h"

mpl::Speaker::Speaker() {}

mpl::MplStatus mpl::Speaker::initPort() {
    auto status = hal::initSpeakerPort();
    if (status != hal::HalStatus::SUCCESS) {
        return mpl::MplStatus::ERROR;
    } else {
        return mpl::MplStatus::SUCCESS;
    }
}

void mpl::Speaker::deinitPort() { hal::deinitSpeakerPort(); }

bool mpl::Speaker::isPlaying() { return end_time != 0; }

mpl::MplStatus mpl::Speaker::playFrequencySync(uint16_t freq,
                                               uint16_t millisec) {
    hal::setSpeakerFrequency(freq);
    Timer::sleepMs(millisec);
    stop();
    return mpl::MplStatus::SUCCESS;
}

mpl::MplStatus mpl::Speaker::playFrequencyAsync(uint16_t freq,
                                                uint16_t millisec) {
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

void mpl::Speaker::stop() { hal::offSpeaker(); }

void mpl::Speaker::interruptPeriodic() {
    // TODO: command を確認し、あれば処理する
    if (end_time == 0) return;  // no playing
    if (Timer::getMicroTime() >= end_time) {
        stop();
        end_time = 0;
    }
}

mpl::Speaker* mpl::Speaker::getInstance() {
    static mpl::Speaker instance;
    return &instance;
}
