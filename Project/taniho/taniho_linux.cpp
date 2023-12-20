#include <cstdio>

#include "mpl_imu.h"
#include "mpl_led.h"
#include "mpl_timer.h"
#include "observer.h"
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

    // auto observer = plt::Observer::getInstance();
    // hal::ImuData imudata;
    // observer->getImuData(imudata);
    // printf("TEMP:%x GX:%x GY:%x GZ:%x AX:%x AY:%x AZ:%x\n", imudata.OUT_TEMP,
    //        imudata.OUT_X_G, imudata.OUT_Y_G, imudata.OUT_Z_G,
    //        imudata.OUT_X_A, imudata.OUT_Y_A, imudata.OUT_Z_A);

    // auto led = mpl::Led::getInstance();
    // led->initPort(hal::LedNumbers::ALL);

    // mpl::Timer::init();

    auto imu = mpl::Imu::getInstance();
    hal::ImuData imudata;
    imu->scanAllSync(imudata);
    printf("TEMP:%x GX:%x GY:%x GZ:%x AX:%x AY:%x AZ:%x\n", imudata.OUT_TEMP,
           imudata.OUT_X_G, imudata.OUT_Y_G, imudata.OUT_Z_G, imudata.OUT_X_A,
           imudata.OUT_Y_A, imudata.OUT_Z_A);

    // for (int i = 0; i < 1000; ++i) {
    // }

    return 0;
}
