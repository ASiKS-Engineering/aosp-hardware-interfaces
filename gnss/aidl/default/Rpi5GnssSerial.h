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

    static constexpr const char* kDevice = "/dev/ttyACM0";
};
