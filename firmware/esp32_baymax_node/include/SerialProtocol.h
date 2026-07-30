#pragma once

#include <cstdint>
#include <cstring>
#include <Arduino.h>
#include "SharedData.h"
#include "Config.h"

class SerialProtocol {
public:
    static constexpr std::size_t RX_BUF_SIZE = 512;
    static constexpr std::size_t TX_BUF_SIZE = 256;

    enum class LinkState : uint8_t {
        DISCONNECTED,
        SYNCING,
        CONNECTED
    };

    struct Stats {
        uint32_t txFrames       = 0;
        uint32_t rxFrames       = 0;
        uint32_t rxErrors       = 0;
        uint32_t checksumFails  = 0;
        uint32_t overruns       = 0;
        float    lastRoundtripMs = 0.0f;
    };

    explicit SerialProtocol(HardwareSerial& port)
        : m_port(port)
        , m_linkState(LinkState::DISCONNECTED)
        , m_txSequence(0)
        , m_rxWriteIdx(0)
        , m_lastHeartbeatMs(0)
        , m_lastRxMs(0)
        , m_brainTimeoutMs(static_cast<uint32_t>(cfg::BRAIN_TIMEOUT_SEC * 1000.0))
    {
        std::memset(m_rxBuf, 0, RX_BUF_SIZE);
    }

    void begin() {
        m_port.begin(cfg::SERIAL_BAUD);
        m_port.setRxBufferSize(RX_BUF_SIZE);
        m_lastHeartbeatMs = millis();
        m_lastRxMs = millis();
        m_linkState = LinkState::SYNCING;
    }

    void sendTelemetry(ipc::TelemetryFrame& frame) {
        frame.stamp(m_txSequence++, micros64());
        uint8_t buf[sizeof(ipc::TelemetryFrame)];
        std::size_t len = ipc::toBytes(frame, buf, sizeof(buf));
        if (len > 0) {
            m_port.write(buf, len);
            ++m_stats.txFrames;
        }
    }

    void sendStatus(ipc::SystemStatus& status) {
        status.uptime_us = micros64();
        status.stamp();
        uint8_t buf[sizeof(ipc::SystemStatus)];
        std::size_t len = ipc::toBytes(status, buf, sizeof(buf));
        if (len > 0) {
            m_port.write(buf, len);
        }
    }

    void sendHeartbeat(ipc::SystemStateId state) {
        ipc::Heartbeat hb;
        hb.timestamp_us = micros64();
        hb.source = state;
        uint8_t buf[sizeof(ipc::Heartbeat)];
        std::memcpy(buf, &hb, sizeof(hb));
        m_port.write(buf, sizeof(hb));
    }

    bool receiveCommand(ipc::CommandFrame& out) {
        while (m_port.available() > 0) {
            uint8_t byte = m_port.read();

            m_rxBuf[m_rxWriteIdx++] = byte;

            if (m_rxWriteIdx >= RX_BUF_SIZE) {
                m_rxWriteIdx = 0;
                ++m_stats.overruns;
                continue;
            }

            if (m_rxWriteIdx >= sizeof(uint32_t)) {
                uint32_t magic = 0;
                std::memcpy(&magic, m_rxBuf, sizeof(uint32_t));

                if (magic != ipc::COMMAND_MAGIC) {
                    shiftBuffer();
                    continue;
                }
            }

            if (m_rxWriteIdx >= sizeof(ipc::CommandFrame)) {
                ipc::CommandFrame candidate;
                if (ipc::fromBytes(m_rxBuf, m_rxWriteIdx, candidate)) {
                    if (candidate.verify()) {
                        out = candidate;
                        m_rxWriteIdx = 0;
                        m_lastRxMs = millis();
                        ++m_stats.rxFrames;

                        if (m_linkState != LinkState::CONNECTED) {
                            m_linkState = LinkState::CONNECTED;
                        }
                        return true;
                    } else {
                        ++m_stats.checksumFails;
                        m_rxWriteIdx = 0;
                    }
                }
            }
        }
        return false;
    }

    void updateHeartbeat(ipc::SystemStateId state) {
        uint32_t now = millis();
        if (now - m_lastHeartbeatMs >= cfg::HEARTBEAT_INTERVAL_MS) {
            sendHeartbeat(state);
            m_lastHeartbeatMs = now;
        }
    }

    void checkTimeout() {
        uint32_t now = millis();
        if (m_linkState == LinkState::CONNECTED) {
            if (now - m_lastRxMs > m_brainTimeoutMs) {
                m_linkState = LinkState::DISCONNECTED;
            }
        }
    }

    bool isConnected() const {
        return m_linkState == LinkState::CONNECTED;
    }

    LinkState linkState() const {
        return m_linkState;
    }

    const Stats& stats() const {
        return m_stats;
    }

    uint64_t micros64() const {
        return static_cast<uint64_t>(esp_timer_get_time());
    }

private:
    void shiftBuffer() {
        if (m_rxWriteIdx <= 1) {
            m_rxWriteIdx = 0;
            return;
        }

        bool found = false;
        for (std::size_t i = 1; i < m_rxWriteIdx; ++i) {
            uint32_t probe = 0;
            if (i + sizeof(uint32_t) <= m_rxWriteIdx) {
                std::memcpy(&probe, m_rxBuf + i, sizeof(uint32_t));
                if (probe == ipc::COMMAND_MAGIC) {
                    std::size_t remaining = m_rxWriteIdx - i;
                    std::memmove(m_rxBuf, m_rxBuf + i, remaining);
                    m_rxWriteIdx = remaining;
                    found = true;
                    break;
                }
            }
        }

        if (!found) {
            m_rxWriteIdx = 0;
            ++m_stats.rxErrors;
        }
    }

    HardwareSerial& m_port;
    LinkState       m_linkState;
    uint32_t        m_txSequence;

    uint8_t         m_rxBuf[RX_BUF_SIZE];
    std::size_t     m_rxWriteIdx;

    uint32_t        m_lastHeartbeatMs;
    uint32_t        m_lastRxMs;
    uint32_t        m_brainTimeoutMs;

    Stats           m_stats;
};
