#define LOG_TAG "Rpi5GnssSerial"

#include "Rpi5GnssSerial.h"

#include <android-base/logging.h>

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

Rpi5GnssSerial::Rpi5GnssSerial() = default;

Rpi5GnssSerial::~Rpi5GnssSerial() {
    stop();
}

bool Rpi5GnssSerial::openDevice() {
    mFd = open(kDevice, O_RDONLY | O_NOCTTY | O_NONBLOCK);

    if (mFd < 0) {
        LOG(ERROR) << "Failed to open " << kDevice
                   << ": " << strerror(errno);
        return false;
    }

    termios tty{};

    if (tcgetattr(mFd, &tty) != 0) {
        LOG(ERROR) << "tcgetattr failed: " << strerror(errno);
        closeDevice();
        return false;
    }

    cfmakeraw(&tty);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    if (tcsetattr(mFd, TCSANOW, &tty) != 0) {
        LOG(ERROR) << "tcsetattr failed: " << strerror(errno);
        closeDevice();
        return false;
    }

    LOG(INFO) << "Opened GNSS device " << kDevice;

    return true;
}

void Rpi5GnssSerial::closeDevice() {
    if (mFd >= 0) {
        close(mFd);
        mFd = -1;
    }
}

bool Rpi5GnssSerial::start(NmeaCallback callback) {
    if (mRunning) {
        return true;
    }

    if (!callback) {
        LOG(ERROR) << "No NMEA callback supplied";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mMutex);
        mCallback = std::move(callback);
    }

    if (!openDevice()) {
        return false;
    }

    mRunning = true;
    mLineBuffer.clear();

    mThread = std::thread(&Rpi5GnssSerial::readerLoop, this);

    LOG(INFO) << "GNSS serial reader started";

    return true;
}

void Rpi5GnssSerial::stop() {
    if (!mRunning && !mThread.joinable()) {
        return;
    }

    mRunning = false;

    if (mThread.joinable()) {
        mThread.join();
    }

    closeDevice();

    {
        std::lock_guard<std::mutex> lock(mMutex);
        mCallback = nullptr;
    }

    LOG(INFO) << "GNSS serial reader stopped";
}

bool Rpi5GnssSerial::isRunning() const {
    return mRunning;
}

void Rpi5GnssSerial::readerLoop() {
    char buffer[512];

    while (mRunning) {
        if (mFd < 0) {
            break;
        }

        const ssize_t bytesRead = read(mFd, buffer, sizeof(buffer));

        if (bytesRead > 0) {
            mLineBuffer.append(buffer, bytesRead);

            size_t newlinePos;

            while ((newlinePos = mLineBuffer.find('\n')) != std::string::npos) {
                std::string line =
                        mLineBuffer.substr(0, newlinePos);

                mLineBuffer.erase(0, newlinePos + 1);

                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }

                if (line.empty()) {
                    continue;
                }

                if (line[0] != '$') {
                    continue;
                }

                NmeaCallback callback;

                {
                    std::lock_guard<std::mutex> lock(mMutex);
                    callback = mCallback;
                }

                if (callback) {
                    callback(line + "\n");
                }
            }

            continue;
        }

        if (bytesRead < 0 &&
            (errno == EAGAIN || errno == EWOULDBLOCK)) {
            usleep(10000);
            continue;
        }

        if (bytesRead < 0 && errno == EINTR) {
            continue;
        }

        LOG(ERROR) << "GNSS serial read failed: "
                   << strerror(errno);
        break;
    }

    mRunning = false;
}
