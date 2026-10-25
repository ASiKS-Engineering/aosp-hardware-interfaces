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
    bool anyNode = false;

    for (int i = 0; i < kMaxDevices; ++i) {
        const std::string path = std::string(kDevicePrefix) + std::to_string(i);

        const int fd = open(path.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            if (errno != ENOENT) {
                anyNode = true;
                LOG(WARNING) << "Failed to open " << path << ": " << strerror(errno);
            }
            continue;
        }
        anyNode = true;

        termios tty{};

        if (tcgetattr(fd, &tty) != 0) {
            LOG(WARNING) << "tcgetattr failed on " << path << ": " << strerror(errno);
            close(fd);
            continue;
        }

        cfmakeraw(&tty);

        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            LOG(WARNING) << "tcsetattr failed on " << path << ": " << strerror(errno);
            close(fd);
            continue;
        }

        mFd = fd;
        mDevice = path;
        mReportedMissing = false;
        LOG(INFO) << "Opened GNSS device " << mDevice;
        return true;
    }

    // Log once per outage, not once per retry.
    if (!mReportedMissing) {
        LOG(WARNING) << (anyNode ? "No usable GNSS serial device" : "No GNSS serial device present")
                     << ", waiting for it to appear";
        mReportedMissing = true;
    }
    return false;
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

    // A missing receiver is not fatal: the reader keeps looking for it, so the receiver can be
    // plugged in (or re-plugged) at any time.
    mReportedMissing = false;
    openDevice();

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
            if (!openDevice()) {
                // Sleep in short steps so stop() is not delayed.
                for (int waited = 0; mRunning && waited < kReopenIntervalMs; waited += 50) {
                    usleep(50000);
                }
                continue;
            }
            mLineBuffer.clear();
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

        // read() == 0 means the device went away (USB unplugged); errors like ENODEV/EIO mean the same.
        LOG(WARNING) << "GNSS serial " << mDevice << " lost ("
                     << (bytesRead == 0 ? "end of file" : strerror(errno)) << "), reopening";
        closeDevice();
    }

    mRunning = false;
}
