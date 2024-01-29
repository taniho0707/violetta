//******************************************************************************
// @addtogroup PLT
// @file       tcp_client.cpp
// @brief      シミュレータと通信するクライアントライブラリ
//******************************************************************************

#include "tcp_client.h"

using namespace std;

plt::TcpClient::TcpClient() {}

plt::TcpClient::~TcpClient() {}

bool plt::TcpClient::connectServer() {
    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket < 0) {
        perror("Error at opening new client socket\n");
        return false;
    }

    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(3000);
    server_address.sin_addr.s_addr = inet_addr("172.27.224.1");

    if (connect(client_socket, (struct sockaddr*)&server_address,
                sizeof(server_address)) < 0) {
        perror("Error at connecting to server\n");
        return false;
    }
    return true;
}

bool plt::TcpClient::disconnectServer() {
    if (close(client_socket) == -1) {
        return false;
    } else {
        return true;
    }
}

bool plt::TcpClient::getImuData(hal::ImuData& data) {
    char send_data[] = {0x00, 0x00, 0x00, 0x01, 0x41};
    ssize_t bytes_sent = send(client_socket, send_data, sizeof(send_data), 0);
    if (bytes_sent == -1) {
        perror("Error at sending IMU data request\n");
        close(client_socket);
        return false;
    } else if (bytes_sent != sizeof(send_data)) {
        perror("Error at sending IMU data request in the middle\n");
        close(client_socket);
        return false;
    }

    char recv_data[128] = {0};
    ssize_t bytes_recv = recv(client_socket, recv_data, sizeof(recv_data), 0);
    if (bytes_recv == -1) {
        perror("Error at receiving IMU data\n");
        close(client_socket);
        return false;
    }

    printf("%ld bytes received from IMU\n", bytes_recv);

    if (bytes_recv == 14) {
        data.OUT_TEMP = static_cast<int16_t>(
            (static_cast<uint16_t>(recv_data[0]) << 8) | recv_data[1]);
        data.OUT_X_G = static_cast<int16_t>(
            (static_cast<uint16_t>(recv_data[2]) << 8) | recv_data[3]);
        data.OUT_Y_G = static_cast<int16_t>(
            (static_cast<uint16_t>(recv_data[4]) << 8) | recv_data[5]);
        data.OUT_Z_G = static_cast<int16_t>(
            (static_cast<uint16_t>(recv_data[6]) << 8) | recv_data[7]);
        data.OUT_X_A = static_cast<int16_t>(
            (static_cast<uint16_t>(recv_data[8]) << 8) | recv_data[9]);
        data.OUT_Y_A = static_cast<int16_t>(
            (static_cast<uint16_t>(recv_data[10]) << 8) | recv_data[11]);
        data.OUT_Z_A = static_cast<int16_t>(
            (static_cast<uint16_t>(recv_data[12]) << 8) | recv_data[13]);
        return true;
    } else {
        return false;
    }
}

bool plt::TcpClient::getBatteryVoltage(float& voltage) {
    char send_data[] = {0x00, 0x00, 0x00, 0x01, 0x42};
    ssize_t bytes_sent = send(client_socket, send_data, sizeof(send_data), 0);
    if (bytes_sent == -1) {
        perror("Error at sending IMU data request\n");
        close(client_socket);
        return false;
    } else if (bytes_sent != sizeof(send_data)) {
        perror("Error at sending IMU data request in the middle\n");
        close(client_socket);
        return false;
    }

    char recv_data[128] = {0};
    ssize_t bytes_recv = recv(client_socket, recv_data, sizeof(recv_data), 0);
    if (bytes_recv == -1) {
        perror("Error at receiving IMU data\n");
        close(client_socket);
        return false;
    }

    printf("%ld bytes received from Battery\n", bytes_recv);

    if (bytes_recv == 5) {
        voltage = static_cast<float>(static_cast<uint32_t>(recv_data[0] << 24) |
                                     (recv_data[1] << 16) |
                                     (recv_data[2] << 8) | recv_data[3]);
        return true;
    } else {
        return false;
    }
}

plt::TcpClient* plt::TcpClient::getInstance() {
    static plt::TcpClient instance;
    return &instance;
}