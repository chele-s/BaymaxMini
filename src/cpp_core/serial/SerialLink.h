#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <vector>
#include <cstdint>

#include "ipc/SharedData.h"

class SerialLink {
public:
    struct Stats {
        uint32_t bytesReceived = 0;
        uint32_t bytesSent     = 0;
        uint32_t packetsRx     = 0;
        uint32_t packetsTx     = 0;
        uint32_t checksumFails = 0;
        uint32_t readErrors    = 0;
    };

    SerialLink();
    ~SerialLink();

    bool openPort(const std::string& portName, uint32_t baudRate = 921600);
    void closePort();
    bool isOpen() const;

    bool getLatestTelemetry(ipc::TelemetryFrame& outFrame);
    bool sendCommand(ipc::CommandType type, const ipc::CommandPayload& payload);

    bool sendExpression(ipc::ExpressionType expr, float transitionSec = 0.3f);
    bool sendEyelid(float openness, float durationSec = 0.2f);
    bool sendGaze(float x, float y, float speed = 1.0f);
    bool sendBreath(float intensity);
    bool sendEmergencyStop();
    bool sendShutdown();

    Stats getStats() const;
    void resetStats();

private:
    void readLoop();
    bool writeRaw(const uint8_t* data, std::size_t len);

    void* m_handle;
    std::string m_portName;
    uint32_t m_baudRate;

    std::atomic<bool> m_running;
    std::thread m_readThread;

    mutable std::mutex m_telemetryMutex;
    ipc::TelemetryFrame m_latestTelemetry;
    bool m_hasNewTelemetry;

    std::atomic<uint32_t> m_txSequence;

    mutable std::mutex m_statsMutex;
    Stats m_stats;
};
