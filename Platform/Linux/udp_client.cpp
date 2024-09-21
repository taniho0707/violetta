//******************************************************************************
// @addtogroup PLT
// @file       udp_client.cpp
// @brief      シミュレータ HagoniwaMouse と通信するクライアントライブラリ
//******************************************************************************

#include "udp_client.h"

using namespace std;

plt::UdpClient::UdpClient() {}

plt::UdpClient::~UdpClient() {}

bool plt::UdpClient::connectServer() {
    client_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (client_socket < 0) {
        perror("Error at opening new client socket\n");
        return false;
    }

    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(3000);
    server_address.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (connect(client_socket, (struct sockaddr*)&server_address, sizeof(server_address)) < 0) {
        perror("Error at connecting to server\n");
        return false;
    }
    return true;
}

bool plt::UdpClient::disconnectServer() {
    if (close(client_socket) == -1) {
        return false;
    } else {
        return true;
    }
}

bool plt::UdpClient::getImuData(hal::ImuData& data) {
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
        data.OUT_TEMP = static_cast<int16_t>((static_cast<uint16_t>(recv_data[1]) << 8) | recv_data[2]);
        data.OUT_X_G = static_cast<int16_t>((static_cast<uint16_t>(recv_data[3]) << 8) | recv_data[4]);
        data.OUT_Y_G = static_cast<int16_t>((static_cast<uint16_t>(recv_data[5]) << 8) | recv_data[6]);
        data.OUT_Z_G = static_cast<int16_t>((static_cast<uint16_t>(recv_data[7]) << 8) | recv_data[8]);
        data.OUT_X_A = static_cast<int16_t>((static_cast<uint16_t>(recv_data[9]) << 8) | recv_data[10]);
        data.OUT_Y_A = static_cast<int16_t>((static_cast<uint16_t>(recv_data[11]) << 8) | recv_data[12]);
        data.OUT_Z_A = static_cast<int16_t>((static_cast<uint16_t>(recv_data[13]) << 8) | recv_data[14]);
        return true;
    } else {
        return false;
    }
}

bool plt::UdpClient::getBatteryVoltage(float& voltage) {
    char send_data[] = {0x00, 0x01, 0x42};
    ssize_t bytes_sent = send(client_socket, send_data, sizeof(send_data), 0);
    if (bytes_sent == -1) {
        perror("Error at sending Battery voltage request\n");
        close(client_socket);
        return false;
    } else if (bytes_sent != sizeof(send_data)) {
        perror("Error at sending Battery voltage request in the middle\n");
        close(client_socket);
        return false;
    }

    char recv_data[128] = {0};
    ssize_t bytes_recv = recv(client_socket, recv_data, sizeof(recv_data), 0);
    if (bytes_recv == -1) {
        perror("Error at receiving Battery voltage\n");
        close(client_socket);
        return false;
    }

    printf("%ld bytes received from Battery\n", bytes_recv);

    if (bytes_recv == 5 + 2) {
        voltage = static_cast<float>(static_cast<uint32_t>(recv_data[3] << 24) | (recv_data[4] << 16) | (recv_data[5] << 8) | recv_data[6]);
        return true;
    } else {
        return false;
    }
}

bool plt::UdpClient::getWallSensorData(uint16_t* data) {
    char send_data[] = {0x00, 0x00, 0x00, 0x01, 0x40};
    ssize_t bytes_sent = send(client_socket, send_data, sizeof(send_data), 0);
    if (bytes_sent == -1) {
        perror("Error at sending WallSensor data request\n");
        close(client_socket);
        return false;
    } else if (bytes_sent != sizeof(send_data)) {
        perror("Error at sending WallSensor data request in the middle\n");
        close(client_socket);
        return false;
    }

    char recv_data[128] = {0};
    ssize_t bytes_recv = recv(client_socket, recv_data, sizeof(recv_data), 0);
    if (bytes_recv == -1) {
        perror("Error at receiving WallSensor data\n");
        close(client_socket);
        return false;
    }

    printf("%ld bytes received from WallSensor\n", bytes_recv);

    if (bytes_recv == (2 * WALLSENSOR_NUMS + 1)) {
        if (recv_data[0] != 0x40) return false;
        for (int i = 0; i < WALLSENSOR_NUMS; ++i) {
            data[i] = static_cast<uint16_t>(static_cast<uint16_t>(recv_data[2 * i + 1] << 8) | recv_data[2 * i + 2]);
        }
        return true;
    } else {
        return false;
    }
}

bool plt::UdpClient::getEncoder(hal::EncoderData& data) {
    char send_data[] = {0x00, 0x00, 0x00, 0x01, 0x43};
    ssize_t bytes_sent = send(client_socket, send_data, sizeof(send_data), 0);
    if (bytes_sent == -1) {
        perror("Error at sending Encoder data request\n");
        close(client_socket);
        return false;
    } else if (bytes_sent != sizeof(send_data)) {
        perror("Error at sending Encoder data request in the middle\n");
        close(client_socket);
        return false;
    }

    char recv_data[128] = {0};
    ssize_t bytes_recv = recv(client_socket, recv_data, sizeof(recv_data), 0);
    if (bytes_recv == -1) {
        perror("Error at receiving Encoder data\n");
        close(client_socket);
        return false;
    }

    printf("%ld bytes received from Encoder\n", bytes_recv);

    if (bytes_recv == 5) {
        if (recv_data[0] != 0x43) return false;
        data.LEFT = static_cast<uint16_t>(static_cast<uint16_t>(recv_data[1] << 8) | recv_data[2]);
        data.RIGHT = static_cast<uint16_t>(static_cast<uint16_t>(recv_data[3] << 8) | recv_data[4]);
        return true;
    } else {
        return false;
    }
}

plt::UdpClient* plt::UdpClient::getInstance() {
    static plt::UdpClient instance;
    return &instance;
}
