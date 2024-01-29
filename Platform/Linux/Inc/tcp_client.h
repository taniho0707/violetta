//******************************************************************************
// @addtogroup PLT
// @file       tcp_client.h
// @brief      シミュレータと通信するクライアントライブラリ
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

class TcpClient {
   private:
    int client_socket;
    sockaddr_in server_address;

   public:
    TcpClient();
    ~TcpClient();

    bool connectServer();
    bool disconnectServer();

    // IMU
    bool getImuData(hal::ImuData& data);

    // Battery
    bool getBatteryVoltage(float& voltage);

    static TcpClient* getInstance();
};

}  // namespace plt
