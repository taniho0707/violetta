//******************************************************************************
// @addtogroup PLT
// @file       observer.h
// @brief      各コンポーネントの内部状況をモニタリングするクラス
//******************************************************************************
#pragma once

#include "tcp_client.h"

namespace plt {

class Observer {
   private:
    plt::TcpClient* tcp;

   public:
    Observer();
    ~Observer();

    bool getImuData(hal::ImuData& data);

    bool getBatteryVoltage(float& voltage);

    static Observer* getInstance();
};

}  // namespace plt
