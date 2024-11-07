//******************************************************************************
// @addtogroup PLT
// @file       observer.h
// @brief      各コンポーネントの内部状況をモニタリングするクラス
//******************************************************************************
#pragma once

#include "udp_client.h"

namespace plt {

class Observer {
   private:
    plt::UdpClient* udp;

   public:
    Observer();
    ~Observer();

    bool getImuData(hal::ImuData& data);

    bool getBatteryVoltage(float& voltage);

    bool getWallSensorData(uint16_t* data);

    bool getEncoder(hal::EncoderData& data);

    bool sendUartDebug(uint8_t* data, const int len);

    static Observer* getInstance();
};

}  // namespace plt
