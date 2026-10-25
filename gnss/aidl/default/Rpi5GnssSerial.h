#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

class Rpi5GnssSerial {
public:
    using NmeaCallback = std::function<void(const std::string&)>;

    Rpi5GnssSerial();
    ~Rpi5GnssSerial();

    bool start(NmeaCallback callback);
    void stop();

    bool isRunning() const;

private:
    void readerLoop();

    bool openDevice();
    void closeDevice();

    int mFd = -1;
    std::atomic<bool> mRunning{false};
    std::thread mThread;

    NmeaCallback mCallback;
    std::mutex mMutex;

    std::string mLineBuffer;

    // The receiver is a USB device: after a re-plug it comes back as ttyACM1, ttyACM2, ... so the
    // device nodes are probed instead of relying on a fixed name.
    static constexpr const char* kDevicePrefix = "/dev/ttyACM";
    static constexpr int kMaxDevices = 10;
    static constexpr int kReopenIntervalMs = 1000;

    std::string mDevice;
    bool mReportedMissing = false;
};
