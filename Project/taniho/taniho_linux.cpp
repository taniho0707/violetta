#include <cstdio>

#include "mpl_led.h"
#include "mpl_timer.h"
#include "taniho.h"

// githash.cpp
void getGithash(char* addr);

int main(void) {
    // 各モジュールの初期化

    // 各モジュールをタイマーイベントに登録

    // 起動音再生

    // アクティビティの開始

    char hash[14];
    getGithash(hash);
    printf("%s\n", hash);

    auto led = mpl::Led::getInstance();
    led->initPort(hal::LedNumbers::ALL);

    mpl::Timer::init();

    for (int i = 0; i < 1000; ++i) {
    }

    return 0;
}
