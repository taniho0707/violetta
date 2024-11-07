//******************************************************************************
// @addtogroup PLT
// @file       udp_client.h
// @brief      シミュレータ HagoniwaMouse と通信するクライアントライブラリ
//******************************************************************************
#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "hal_conf.h"

namespace plt {

class UdpClient {
   private:
    int client_socket;
    sockaddr_in server_address;

   public:
    UdpClient();
    ~UdpClient();

    bool connectServer();
    bool disconnectServer();

    // IMU
    bool getImuData(hal::ImuData& data);

    // Battery
    bool getBatteryVoltage(float& voltage);

    // WallSensor
    bool getWallSensorData(uint16_t* data);

    // Encoder
    bool getEncoder(hal::EncoderData& data);

    // Uart
    bool sendUartDebug(uint8_t* data, const int len);

    static UdpClient* getInstance();
};

}  // namespace plt
