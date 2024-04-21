#pragma once

// TODO: どこかでパラメータを定義し、どこかでデフォルトパラメータを読み込む
struct HardwareParams {
    int voltage;
    float temperature;
    bool isPoweredOn;
    // Add more parameters as needed

    // Constructor
    HardwareParams(int v, float t, bool poweredOn)
        : voltage(v), temperature(t), isPoweredOn(poweredOn) {}
};