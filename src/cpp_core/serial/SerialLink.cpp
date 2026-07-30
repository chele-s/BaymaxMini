#include "SerialLink.h"

#include <chrono>
#include <cstring>

#if defined(_WIN32) || defined(_WIN64)
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>
#else
    #include <fcntl.h>
    #include <termios.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
#endif

SerialLink::SerialLink()
    : m_handle(nullptr)
    , m_baudRate(921600)
    , m_running(false)
    , m_hasNewTelemetry(false)
    , m_txSequence(0)
{}

SerialLink::~SerialLink() {
    closePort();
}

bool SerialLink::openPort(const std::string& portName, uint32_t baudRate) {
    if (m_running.load()) {
        closePort();
    }

    m_portName = portName;
    m_baudRate = baudRate;

#if defined(_WIN32) || defined(_WIN64)
    std::string formattedName = portName;
    if (portName.rfind("COM", 0) == 0 && portName.length() > 4) {
        formattedName = "\\\\.\\" + portName;
    }

    HANDLE hSerial = CreateFileA(
        formattedName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    if (hSerial == INVALID_HANDLE_VALUE) {
        return false;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        CloseHandle(hSerial);
        return false;
    }

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;
    dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_ENABLE;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        CloseHandle(hSerial);
        return false;
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 10;
    timeouts.ReadTotalTimeoutConstant     = 50;
    timeouts.ReadTotalTimeoutMultiplier   = 1;
    timeouts.WriteTotalTimeoutConstant    = 50;
    timeouts.WriteTotalTimeoutMultiplier  = 1;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        CloseHandle(hSerial);
        return false;
    }

    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    m_handle = static_cast<void*>(hSerial);
#else
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        return false;
    }

    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);

    speed_t speed = B921600;
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 1;

    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIFLUSH);

    m_handle = reinterpret_cast<void*>(static_cast<intptr_t>(fd));
#endif

    m_running.store(true);
    m_readThread = std::thread(&SerialLink::readLoop, this);

    return true;
}

void SerialLink::closePort() {
    m_running.store(false);

    if (m_readThread.joinable()) {
        m_readThread.join();
    }

    if (m_handle != nullptr) {
#if defined(_WIN32) || defined(_WIN64)
        HANDLE hSerial = static_cast<HANDLE>(m_handle);
        CloseHandle(hSerial);
#else
        int fd = static_cast<int>(reinterpret_cast<intptr_t>(m_handle));
        close(fd);
#endif
        m_handle = nullptr;
    }
}

bool SerialLink::isOpen() const {
    return m_running.load() && m_handle != nullptr;
}

bool SerialLink::writeRaw(const uint8_t* data, std::size_t len) {
    if (!isOpen()) return false;

#if defined(_WIN32) || defined(_WIN64)
    HANDLE hSerial = static_cast<HANDLE>(m_handle);
    DWORD bytesWritten = 0;
    BOOL result = WriteFile(hSerial, data, static_cast<DWORD>(len), &bytesWritten, NULL);
    if (result && bytesWritten == len) {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.bytesSent += bytesWritten;
        return true;
    }
    return false;
#else
    int fd = static_cast<int>(reinterpret_cast<intptr_t>(m_handle));
    ssize_t res = write(fd, data, len);
    if (res > 0) {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.bytesSent += static_cast<uint32_t>(res);
        return static_cast<std::size_t>(res) == len;
    }
    return false;
#endif
}

void SerialLink::readLoop() {
    std::vector<uint8_t> rxBuffer;
    rxBuffer.reserve(1024);

    uint8_t tempBuf[256];

    while (m_running.load()) {
        std::size_t bytesRead = 0;

#if defined(_WIN32) || defined(_WIN64)
        HANDLE hSerial = static_cast<HANDLE>(m_handle);
        DWORD readCount = 0;
        if (ReadFile(hSerial, tempBuf, sizeof(tempBuf), &readCount, NULL) && readCount > 0) {
            bytesRead = static_cast<std::size_t>(readCount);
        }
#else
        int fd = static_cast<int>(reinterpret_cast<intptr_t>(m_handle));
        ssize_t res = read(fd, tempBuf, sizeof(tempBuf));
        if (res > 0) {
            bytesRead = static_cast<std::size_t>(res);
        }
#endif

        if (bytesRead > 0) {
            {
                std::lock_guard<std::mutex> lock(m_statsMutex);
                m_stats.bytesReceived += static_cast<uint32_t>(bytesRead);
            }

            rxBuffer.insert(rxBuffer.end(), tempBuf, tempBuf + bytesRead);

            while (rxBuffer.size() >= sizeof(ipc::TelemetryFrame)) {
                uint32_t magic = 0;
                std::memcpy(&magic, rxBuffer.data(), sizeof(uint32_t));

                if (magic != ipc::TELEMETRY_MAGIC) {
                    rxBuffer.erase(rxBuffer.begin());
                    continue;
                }

                ipc::TelemetryFrame frame;
                if (ipc::fromBytes(rxBuffer.data(), rxBuffer.size(), frame)) {
                    if (frame.verify()) {
                        {
                            std::lock_guard<std::mutex> lock(m_telemetryMutex);
                            m_latestTelemetry = frame;
                            m_hasNewTelemetry = true;
                        }
                        {
                            std::lock_guard<std::mutex> lock(m_statsMutex);
                            ++m_stats.packetsRx;
                        }
                        rxBuffer.erase(rxBuffer.begin(), rxBuffer.begin() + sizeof(ipc::TelemetryFrame));
                    } else {
                        {
                            std::lock_guard<std::mutex> lock(m_statsMutex);
                            ++m_stats.checksumFails;
                        }
                        rxBuffer.erase(rxBuffer.begin());
                    }
                } else {
                    break;
                }
            }

            if (rxBuffer.size() > 2048) {
                rxBuffer.clear();
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

bool SerialLink::getLatestTelemetry(ipc::TelemetryFrame& outFrame) {
    std::lock_guard<std::mutex> lock(m_telemetryMutex);
    if (!m_hasNewTelemetry) return false;
    outFrame = m_latestTelemetry;
    m_hasNewTelemetry = false;
    return true;
}

bool SerialLink::sendCommand(ipc::CommandType type, const ipc::CommandPayload& payload) {
    ipc::CommandFrame cmd;
    cmd.type = type;
    cmd.payload = payload;

    auto now = std::chrono::steady_clock::now();
    uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    cmd.stamp(m_txSequence.fetch_add(1), ts);

    uint8_t buf[sizeof(ipc::CommandFrame)];
    std::size_t len = ipc::toBytes(cmd, buf, sizeof(buf));

    if (len > 0 && writeRaw(buf, len)) {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        ++m_stats.packetsTx;
        return true;
    }
    return false;
}

bool SerialLink::sendExpression(ipc::ExpressionType expr, float transitionSec) {
    ipc::CommandPayload payload;
    payload.expression.type = expr;
    payload.expression.transition = transitionSec;
    return sendCommand(ipc::CommandType::SET_EXPRESSION, payload);
}

bool SerialLink::sendEyelid(float openness, float durationSec) {
    ipc::CommandPayload payload;
    payload.eyelid.openness = openness;
    payload.eyelid.duration = durationSec;
    return sendCommand(ipc::CommandType::SET_EYELID, payload);
}

bool SerialLink::sendGaze(float x, float y, float speed) {
    ipc::CommandPayload payload;
    payload.gaze.x = x;
    payload.gaze.y = y;
    payload.gaze.speed = speed;
    return sendCommand(ipc::CommandType::SET_GAZE, payload);
}

bool SerialLink::sendBreath(float intensity) {
    ipc::CommandPayload payload;
    payload.breath.intensity = intensity;
    return sendCommand(ipc::CommandType::SET_BREATH, payload);
}

bool SerialLink::sendEmergencyStop() {
    ipc::CommandPayload payload;
    return sendCommand(ipc::CommandType::EMERGENCY_STOP, payload);
}

bool SerialLink::sendShutdown() {
    ipc::CommandPayload payload;
    return sendCommand(ipc::CommandType::SHUTDOWN, payload);
}

SerialLink::Stats SerialLink::getStats() const {
    std::lock_guard<std::mutex> lock(m_statsMutex);
    return m_stats;
}

void SerialLink::resetStats() {
    std::lock_guard<std::mutex> lock(m_statsMutex);
    m_stats = Stats{};
}
