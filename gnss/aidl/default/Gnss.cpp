/*
 * Copyright (C) 2020 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "GnssAidl"

#include "Gnss.h"
#include "NmeaFixInfo.h"
#include <inttypes.h>
#include <log/log.h>
#include <utils/SystemClock.h>
#include <utils/Timers.h>
#include <cstdlib>
#include <sstream>
#include "AGnss.h"
#include "AGnssRil.h"
#include "GnssAntennaInfo.h"
#include "GnssAssistanceInterface.h"
#include "GnssBatching.h"
#include "GnssConfiguration.h"
#include "GnssDebug.h"
#include "GnssGeofence.h"
#include "GnssNavigationMessageInterface.h"
#include "GnssPsds.h"
#include "GnssVisibilityControl.h"
#include "MeasurementCorrectionsInterface.h"
#include "Utils.h"

namespace aidl::android::hardware::gnss {
using ::android::hardware::gnss::common::Utils;
using ::android::hardware::gnss::common::NmeaFixInfo;

using ndk::ScopedAStatus;
using GnssSvInfo = IGnssCallback::GnssSvInfo;

std::shared_ptr<IGnssCallback> Gnss::sGnssCallback = nullptr;

namespace {

// Satellite status older than this is not reported any more (receiver unplugged or silent).
constexpr std::chrono::seconds kSvStatusMaxAge(5);

// Splits an NMEA sentence into its fields (without '$' and checksum). Returns an empty vector for
// malformed sentences or a wrong checksum.
std::vector<std::string> splitNmea(const std::string& sentence) {
    const size_t start = sentence.find('$');
    if (start == std::string::npos) {
        return {};
    }
    const size_t star = sentence.find('*', start);
    std::string body = sentence.substr(
            start + 1, (star == std::string::npos ? sentence.size() : star) - start - 1);
    while (!body.empty() && (body.back() == '\n' || body.back() == '\r')) {
        body.pop_back();
    }
    if (star != std::string::npos) {
        const std::string hex = sentence.substr(star + 1, 2);
        char* end = nullptr;
        const long expected = strtol(hex.c_str(), &end, 16);
        if (end == hex.c_str()) {
            return {};
        }
        uint8_t checksum = 0;
        for (char c : body) {
            checksum ^= static_cast<uint8_t>(c);
        }
        if (checksum != expected) {
            return {};
        }
    }
    std::vector<std::string> fields;
    size_t pos = 0;
    while (true) {
        const size_t comma = body.find(',', pos);
        if (comma == std::string::npos) {
            fields.push_back(body.substr(pos));
            break;
        }
        fields.push_back(body.substr(pos, comma - pos));
        pos = comma + 1;
    }
    return fields;
}

int toInt(const std::string& s, int fallback) {
    return s.empty() ? fallback : static_cast<int>(strtol(s.c_str(), nullptr, 10));
}

float toFloat(const std::string& s, float fallback) {
    return s.empty() ? fallback : strtof(s.c_str(), nullptr);
}

struct SvId {
    GnssConstellationType type;
    int svid;
};

// Maps an NMEA satellite number to the constellation/svid used by Android. systemId is the NMEA
// 4.1 GSA system id (1 GPS, 2 GLONASS, 3 Galileo, 4 BeiDou, 5 QZSS) or 0 when unknown.
SvId mapNmeaSv(const std::string& talker, int prn, int systemId) {
    GnssConstellationType type = GnssConstellationType::UNKNOWN;
    if (talker == "GN" && systemId > 0) {
        switch (systemId) {
            case 1: type = GnssConstellationType::GPS; break;
            case 2: type = GnssConstellationType::GLONASS; break;
            case 3: type = GnssConstellationType::GALILEO; break;
            case 4: type = GnssConstellationType::BEIDOU; break;
            case 5: type = GnssConstellationType::QZSS; break;
            default: break;
        }
    } else if (talker == "GL") {
        type = GnssConstellationType::GLONASS;
    } else if (talker == "GA") {
        type = GnssConstellationType::GALILEO;
    } else if (talker == "GB" || talker == "BD") {
        type = GnssConstellationType::BEIDOU;
    } else if (talker == "GQ") {
        type = GnssConstellationType::QZSS;
    } else if (prn >= 1 && prn <= 32) {
        type = GnssConstellationType::GPS;
    } else if (prn >= 33 && prn <= 64) {
        type = GnssConstellationType::SBAS;
    } else if (prn >= 65 && prn <= 96) {
        type = GnssConstellationType::GLONASS;
    } else if (prn >= 193 && prn <= 202) {
        type = GnssConstellationType::QZSS;
    }

    int svid = prn;
    switch (type) {
        case GnssConstellationType::SBAS: svid = prn + 87; break;
        case GnssConstellationType::GLONASS: svid = prn > 64 ? prn - 64 : prn; break;
        case GnssConstellationType::BEIDOU: svid = prn > 200 ? prn - 200 : prn; break;
        case GnssConstellationType::QZSS: svid = prn < 193 ? prn + 192 : prn; break;
        default: break;
    }
    return {type, svid};
}

int svKey(const SvId& id) {
    return static_cast<int>(id.type) * 1000 + id.svid;
}

int64_t carrierFrequencyHz(GnssConstellationType type) {
    switch (type) {
        case GnssConstellationType::GLONASS: return 1602000000;
        case GnssConstellationType::BEIDOU: return 1561098000;
        default: return 1575420000;
    }
}

}  // namespace

Gnss::Gnss() : mMinIntervalMs(1000), mFirstFixReceived(false) {}

ScopedAStatus Gnss::setCallback(const std::shared_ptr<IGnssCallback>& callback) {
    ALOGD("setCallback");
    if (callback == nullptr) {
        ALOGE("%s: Null callback ignored", __func__);
        return ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }
    {
        std::unique_lock<std::mutex> lock(mMutex);
        sGnssCallback = callback;
    }

    int capabilities =
            (int)(IGnssCallback::CAPABILITY_MEASUREMENTS | IGnssCallback::CAPABILITY_SCHEDULING |
                  IGnssCallback::CAPABILITY_SATELLITE_BLOCKLIST |
                  IGnssCallback::CAPABILITY_SATELLITE_PVT |
                  IGnssCallback::CAPABILITY_CORRELATION_VECTOR |
                  IGnssCallback::CAPABILITY_ANTENNA_INFO |
                  IGnssCallback::CAPABILITY_ACCUMULATED_DELTA_RANGE);
    auto status = callback->gnssSetCapabilitiesCb(capabilities);
    if (!status.isOk()) {
        ALOGE("%s: Unable to invoke callback.gnssSetCapabilitiesCb", __func__);
    }

    IGnssCallback::GnssSystemInfo systemInfo = {
            .yearOfHw = 2022,
            .name = "Google, Cuttlefish, AIDL v3",
    };
    status = callback->gnssSetSystemInfoCb(systemInfo);
    if (!status.isOk()) {
        ALOGE("%s: Unable to invoke callback.gnssSetSystemInfoCb", __func__);
    }
    GnssSignalType signalType1 = {
            .constellation = GnssConstellationType::GPS,
            .carrierFrequencyHz = 1.57542e+09,
            .codeType = GnssSignalType::CODE_TYPE_C,
    };
    GnssSignalType signalType2 = {
            .constellation = GnssConstellationType::GLONASS,
            .carrierFrequencyHz = 1.5980625e+09,
            .codeType = GnssSignalType::CODE_TYPE_C,
    };
    status = callback->gnssSetSignalTypeCapabilitiesCb(
            std::vector<GnssSignalType>({signalType1, signalType2}));
    if (!status.isOk()) {
        ALOGE("%s: Unable to invoke callback.gnssSetSignalTypeCapabilitiesCb", __func__);
    }

    // In case when the setCallback() and close() calls are not balanced
    mIsInitialized = false;
    mIsActive = false;
    mThreadBlocker.notify();
    if (mThread.joinable()) {
        mThread.join();
    }

    mIsInitialized = true;
    mThreadBlocker.reset();
    mThread = std::thread([this]() {
        while (mIsInitialized) {
            if (mIsActive) {
                if (!mGnssMeasurementEnabled || mMinIntervalMs <= mGnssMeasurementIntervalMs) {
                    this->reportSvStatus();
                }

                mGnssPowerIndication->notePowerConsumption();
                mThreadBlocker.wait_for(std::chrono::milliseconds(mMinIntervalMs));
            } else {
                // Wait indefinitely until start() or close() is called
                mThreadBlocker.wait();
            }
        }
    });

    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::start() {
    ALOGD("start");
    if (mIsActive) {
        ALOGW("Gnss has started. Restarting...");
        stop();
    }

    {
        std::lock_guard<std::mutex> lock(mNmeaMutex);
        mGprmcLine.clear();
        mGpggaLine.clear();
        mSvPending.clear();
        mSvComplete.clear();
        mUsedSvKeys.clear();
    }

    mIsActive = true;

    if (!mGnssSerial.start([this](const std::string& nmea) {
            handleNmea(nmea);
        })) {
        ALOGE("Failed to start RPi5 GNSS serial reader");
    }

    // notify measurement engine to update measurement interval
    mGnssMeasurementInterface->setLocationEnabled(true);
    this->reportGnssStatusValue(IGnssCallback::GnssStatusValue::SESSION_BEGIN);
    mThreadBlocker.notify();
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::stop() {
    ALOGD("stop");
    mIsActive = false;
    mGnssSerial.stop();
    mGnssMeasurementInterface->setLocationEnabled(false);
    this->reportGnssStatusValue(IGnssCallback::GnssStatusValue::SESSION_END);

    int reportedLocationCount = mReportedLocationCount;
    ALOGD("reportedLocationCount: %d", reportedLocationCount);
    mReportedLocationCount = 0;

    {
        std::lock_guard<std::mutex> lock(mNmeaMutex);
        mGprmcLine.clear();
        mGpggaLine.clear();
        mSvPending.clear();
        mSvComplete.clear();
        mUsedSvKeys.clear();
    }

    mThreadBlocker.notify();
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::close() {
    ALOGD("close");

    mIsActive = false;
    mGnssSerial.stop();

    {
        std::unique_lock<std::mutex> lock(mMutex);
        sGnssCallback = nullptr;
    }
    mIsInitialized = false;
    mThreadBlocker.notify();
    if (mThread.joinable()) {
        mThread.join();
    }
    return ScopedAStatus::ok();
}

void Gnss::handleNmea(const std::string& nmea) {
    if (!mIsActive) {
        return;
    }
    
    ALOGI("NMEA RX: %s", nmea.c_str());

    updateSvFromNmea(nmea);
    
    if (mIsNmeaActive) {
        reportNmea(nmea);
    }

    std::string fixInput;

    {
        std::lock_guard<std::mutex> lock(mNmeaMutex);

        if (nmea.rfind("$GPRMC", 0) == 0) {
            mGprmcLine = nmea;
        } else if (nmea.rfind("$GPGGA", 0) == 0) {
            mGpggaLine = nmea;
        } else {
            return;
        }

        if (mGprmcLine.empty() || mGpggaLine.empty()) {
            return;
        }

        fixInput = mGprmcLine + "\n" + mGpggaLine;
        mGprmcLine.clear();
        mGpggaLine.clear();
    }

    ALOGI("NMEA FIX INPUT: %s", fixInput.c_str());
    
    auto location = NmeaFixInfo::getAidlLocationFromInputStr(fixInput);

    if (location == nullptr) {
        ALOGW("Could not parse GNSS fix from NMEA");
        return;
    }

ALOGD("GNSS fix: lat=%lf lon=%lf alt=%lf speed=%f bearing=%f",
    location->latitudeDegrees,
    location->longitudeDegrees,
    location->altitudeMeters,
    location->speedMetersPerSec,
    location->bearingDegrees);

    mFirstFixReceived = true;
    reportLocation(*location);
    mReportedLocationCount += 1;
}

void Gnss::reportLocation(const GnssLocation& location) {
    std::shared_ptr<IGnssCallback> callback;
    {
        std::unique_lock<std::mutex> lock(mMutex);
        if (sGnssCallback == nullptr) {
            ALOGE("%s: GnssCallback is null.", __func__);
            return;
        }
        callback = sGnssCallback;
    }
    mLastLocation = std::make_shared<GnssLocation>(location);
    auto status = callback->gnssLocationCb(location);
    if (!status.isOk()) {
        ALOGE("%s: Unable to invoke gnssLocationCb", __func__);
    }
    return;
}

void Gnss::updateSvFromNmea(const std::string& nmea) {
    const std::vector<std::string> fields = splitNmea(nmea);
    if (fields.empty() || fields[0].size() != 5) {
        return;
    }
    const std::string talker = fields[0].substr(0, 2);
    const std::string sentence = fields[0].substr(2);

    if (sentence == "GSV") {
        // $--GSV,<messages>,<message>,<in view>,{<prn>,<elevation>,<azimuth>,<snr>}*4[,<signal>]
        if (fields.size() < 4) {
            return;
        }
        const int total = toInt(fields[1], 0);
        const int number = toInt(fields[2], 0);

        std::lock_guard<std::mutex> lock(mNmeaMutex);
        auto& pending = mSvPending[talker];
        if (number <= 1) {
            pending.clear();
        }
        for (size_t i = 4; i + 3 < fields.size(); i += 4) {
            const int prn = toInt(fields[i], 0);
            if (prn <= 0) {
                continue;
            }
            const SvId id = mapNmeaSv(talker, prn, 0);
            if (id.type == GnssConstellationType::UNKNOWN) {
                continue;
            }
            GnssSvInfo sv;
            sv.svid = id.svid;
            sv.constellation = id.type;
            // Empty SNR means the satellite is known but not tracked.
            sv.cN0Dbhz = toFloat(fields[i + 3], 0.0f);
            sv.basebandCN0DbHz = 0.0f;
            sv.elevationDegrees = toFloat(fields[i + 1], 0.0f);
            sv.azimuthDegrees = toFloat(fields[i + 2], 0.0f);
            sv.carrierFrequencyHz = carrierFrequencyHz(id.type);
            sv.svFlag = static_cast<int>(IGnssCallback::GnssSvFlags::HAS_CARRIER_FREQUENCY);
            GnssSignalType signalType;
            signalType.constellation = id.type;
            signalType.carrierFrequencyHz = static_cast<double>(carrierFrequencyHz(id.type));
            signalType.codeType = GnssSignalType::CODE_TYPE_C;
            sv.signalType = signalType;
            pending.push_back(sv);
        }
        if (total > 0 && number >= total) {
            SvCycle& cycle = mSvComplete[talker];
            cycle.sv = std::move(pending);
            cycle.time = std::chrono::steady_clock::now();
            pending.clear();
        }
    } else if (sentence == "GSA") {
        // $--GSA,<mode>,<fix type>,<prn>*12,<pdop>,<hdop>,<vdop>[,<system id>]
        if (fields.size() < 15) {
            return;
        }
        const int fixType = toInt(fields[2], 1);
        const int systemId = fields.size() > 18 ? toInt(fields[18], 0) : 0;
        std::set<int> used;
        if (fixType >= 2) {
            for (size_t i = 3; i <= 14; ++i) {
                const int prn = toInt(fields[i], 0);
                if (prn > 0) {
                    used.insert(svKey(mapNmeaSv(talker, prn, systemId)));
                }
            }
        }
        std::lock_guard<std::mutex> lock(mNmeaMutex);
        mUsedSvKeys[talker + std::to_string(systemId)] = std::move(used);
    }
}

std::vector<GnssSvInfo> Gnss::buildSvList() const {
    std::vector<GnssSvInfo> result;
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mNmeaMutex);
        for (const auto& [talker, cycle] : mSvComplete) {
            if (now - cycle.time > kSvStatusMaxAge) {
                continue;
            }
            for (GnssSvInfo sv : cycle.sv) {
                const int key = svKey({sv.signalType->constellation, sv.svid});
                for (const auto& [source, usedKeys] : mUsedSvKeys) {
                    if (usedKeys.count(key) > 0) {
                        sv.svFlag |= static_cast<int>(IGnssCallback::GnssSvFlags::USED_IN_FIX);
                        break;
                    }
                }
                result.push_back(sv);
            }
        }
    }

    ElapsedRealtime elapsedRealtime = {
            .flags = ElapsedRealtime::HAS_TIMESTAMP_NS | ElapsedRealtime::HAS_TIME_UNCERTAINTY_NS,
            .timestampNs = static_cast<int64_t>(::android::elapsedRealtimeNano()),
            .timeUncertaintyNs = 1000000};
    for (GnssSvInfo& sv : result) {
        sv.elapsedRealtime = elapsedRealtime;
    }
    return result;
}

void Gnss::reportSvStatus() const {
    if (mIsSvStatusActive) {
        auto svStatus = filterBlocklistedSatellites(buildSvList());
        reportSvStatus(svStatus);
    }
}

void Gnss::reportSvStatus(const std::vector<GnssSvInfo>& svInfoList) const {
    std::shared_ptr<IGnssCallback> callback;
    {
        std::unique_lock<std::mutex> lock(mMutex);
        if (sGnssCallback == nullptr) {
            ALOGE("%s: GnssCallback is null.", __func__);
            return;
        }
        callback = sGnssCallback;
    }
    auto status = callback->gnssSvStatusCb(svInfoList);
    if (!status.isOk()) {
        ALOGE("%s: Unable to invoke callback", __func__);
    }
}

std::vector<GnssSvInfo> Gnss::filterBlocklistedSatellites(
        std::vector<GnssSvInfo> gnssSvInfoList) const {
    for (uint32_t i = 0; i < gnssSvInfoList.size(); i++) {
        if (mGnssConfiguration->isBlocklisted(gnssSvInfoList[i])) {
            gnssSvInfoList[i].svFlag &= ~(uint32_t)IGnssCallback::GnssSvFlags::USED_IN_FIX;
        }
    }
    return gnssSvInfoList;
}

void Gnss::reportGnssStatusValue(const IGnssCallback::GnssStatusValue gnssStatusValue) const {
    std::shared_ptr<IGnssCallback> callback;
    {
        std::unique_lock<std::mutex> lock(mMutex);
        if (sGnssCallback == nullptr) {
            ALOGE("%s: GnssCallback is null.", __func__);
            return;
        }
        callback = sGnssCallback;
    }
    auto status = callback->gnssStatusCb(gnssStatusValue);
    if (!status.isOk()) {
        ALOGE("%s: Unable to invoke gnssStatusCb", __func__);
    }
}

void Gnss::reportNmea(const std::string& nmea) const {
    std::shared_ptr<IGnssCallback> callback;

    {
        std::unique_lock<std::mutex> lock(mMutex);
        if (sGnssCallback == nullptr) {
            return;
        }
        callback = sGnssCallback;
    }

    nsecs_t now = systemTime(SYSTEM_TIME_MONOTONIC);
    auto status = callback->gnssNmeaCb(now, nmea);
    if (!status.isOk()) {
        ALOGE("%s: Unable to invoke callback", __func__);
    }
}

ScopedAStatus Gnss::startSvStatus() {
    ALOGD("startSvStatus");
    mIsSvStatusActive = true;
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::stopSvStatus() {
    ALOGD("stopSvStatus");
    mIsSvStatusActive = false;
    return ScopedAStatus::ok();
}
ScopedAStatus Gnss::startNmea() {
    ALOGD("startNmea");
    mIsNmeaActive = true;
    return ScopedAStatus::ok();
}
ScopedAStatus Gnss::stopNmea() {
    ALOGD("stopNmea");
    mIsNmeaActive = false;
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::getExtensionAGnss(std::shared_ptr<IAGnss>* iAGnss) {
    ALOGD("Gnss::getExtensionAGnss");
    *iAGnss = SharedRefBase::make<AGnss>();
    return ndk::ScopedAStatus::ok();
}

ScopedAStatus Gnss::injectTime(int64_t timeMs, int64_t timeReferenceMs, int uncertaintyMs) {
    ALOGD("injectTime. timeMs:%" PRId64 ", timeReferenceMs:%" PRId64 ", uncertaintyMs:%d", timeMs,
          timeReferenceMs, uncertaintyMs);
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::getExtensionAGnssRil(std::shared_ptr<IAGnssRil>* iAGnssRil) {
    ALOGD("Gnss::getExtensionAGnssRil");
    *iAGnssRil = SharedRefBase::make<AGnssRil>();
    return ndk::ScopedAStatus::ok();
}

ScopedAStatus Gnss::injectLocation(const GnssLocation& location) {
    ALOGD("injectLocation. lat:%lf, lng:%lf, acc:%f", location.latitudeDegrees,
          location.longitudeDegrees, location.horizontalAccuracyMeters);
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::injectBestLocation(const GnssLocation& location) {
    ALOGD("injectBestLocation. lat:%lf, lng:%lf, acc:%f", location.latitudeDegrees,
          location.longitudeDegrees, location.horizontalAccuracyMeters);
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::deleteAidingData(GnssAidingData aidingDataFlags) {
    ALOGD("deleteAidingData. flags:%d", (int)aidingDataFlags);
    mFirstFixReceived = false;
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::setPositionMode(const PositionModeOptions& options) {
    ALOGD("setPositionMode. minIntervalMs:%d, lowPowerMode:%d", options.minIntervalMs,
          (int)options.lowPowerMode);
    if (std::max(1000, options.minIntervalMs) != mMinIntervalMs) {
        mMinIntervalMs = std::max(1000, options.minIntervalMs);
        mGnssMeasurementInterface->setLocationInterval(mMinIntervalMs);
        if (mIsActive) {
            stop();
            start();
        }
    }
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::getExtensionPsds(std::shared_ptr<IGnssPsds>* iGnssPsds) {
    ALOGD("getExtensionPsds");
    *iGnssPsds = SharedRefBase::make<GnssPsds>();
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::getExtensionGnssConfiguration(
        std::shared_ptr<IGnssConfiguration>* iGnssConfiguration) {
    ALOGD("getExtensionGnssConfiguration");
    if (mGnssConfiguration == nullptr) {
        mGnssConfiguration = SharedRefBase::make<GnssConfiguration>();
    }
    *iGnssConfiguration = mGnssConfiguration;
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::getExtensionGnssPowerIndication(
        std::shared_ptr<IGnssPowerIndication>* iGnssPowerIndication) {
    ALOGD("getExtensionGnssPowerIndication");
    if (mGnssPowerIndication == nullptr) {
        mGnssPowerIndication = SharedRefBase::make<GnssPowerIndication>();
    }

    *iGnssPowerIndication = mGnssPowerIndication;
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::getExtensionGnssMeasurement(
        std::shared_ptr<IGnssMeasurementInterface>* iGnssMeasurement) {
    ALOGD("getExtensionGnssMeasurement");
    if (mGnssMeasurementInterface == nullptr) {
        mGnssMeasurementInterface = SharedRefBase::make<GnssMeasurementInterface>();
        mGnssMeasurementInterface->setGnssInterface(static_cast<std::shared_ptr<Gnss>>(this));
    }
    *iGnssMeasurement = mGnssMeasurementInterface;
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::getExtensionGnssBatching(std::shared_ptr<IGnssBatching>* iGnssBatching) {
    ALOGD("getExtensionGnssBatching");

    *iGnssBatching = SharedRefBase::make<GnssBatching>();
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::getExtensionGnssGeofence(std::shared_ptr<IGnssGeofence>* iGnssGeofence) {
    ALOGD("getExtensionGnssGeofence");

    *iGnssGeofence = SharedRefBase::make<GnssGeofence>();
    return ScopedAStatus::ok();
}

ScopedAStatus Gnss::getExtensionGnssNavigationMessage(
        std::shared_ptr<IGnssNavigationMessageInterface>* iGnssNavigationMessage) {
    ALOGD("getExtensionGnssNavigationMessage");

    *iGnssNavigationMessage = SharedRefBase::make<GnssNavigationMessageInterface>();
    return ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::getExtensionGnssDebug(std::shared_ptr<IGnssDebug>* iGnssDebug) {
    ALOGD("Gnss::getExtensionGnssDebug");
    *iGnssDebug = SharedRefBase::make<GnssDebug>();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::getExtensionGnssVisibilityControl(
        std::shared_ptr<visibility_control::IGnssVisibilityControl>* iGnssVisibilityControl) {
    ALOGD("Gnss::getExtensionGnssVisibilityControl");

    *iGnssVisibilityControl = SharedRefBase::make<visibility_control::GnssVisibilityControl>();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::getExtensionGnssAntennaInfo(
        std::shared_ptr<IGnssAntennaInfo>* iGnssAntennaInfo) {
    ALOGD("Gnss::getExtensionGnssAntennaInfo");

    *iGnssAntennaInfo = SharedRefBase::make<GnssAntennaInfo>();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::getExtensionMeasurementCorrections(
        std::shared_ptr<measurement_corrections::IMeasurementCorrectionsInterface>*
                iMeasurementCorrections) {
    ALOGD("Gnss::getExtensionMeasurementCorrections");

    *iMeasurementCorrections =
            SharedRefBase::make<measurement_corrections::MeasurementCorrectionsInterface>();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Gnss::getExtensionGnssAssistanceInterface(
        std::shared_ptr<gnss_assistance::IGnssAssistanceInterface>* iGnssAssistanceInterface) {
    ALOGD("Gnss::getExtensionGnssAssistanceInterface");

    *iGnssAssistanceInterface = SharedRefBase::make<gnss_assistance::GnssAssistanceInterface>();
    return ndk::ScopedAStatus::ok();
}

void Gnss::setGnssMeasurementEnabled(const bool enabled) {
    mGnssMeasurementEnabled = enabled;
}

void Gnss::setGnssMeasurementInterval(const long intervalMs) {
    mGnssMeasurementIntervalMs = intervalMs;
}

std::shared_ptr<GnssLocation> Gnss::getLastLocation() const {
    return mLastLocation;
}

}  // namespace aidl::android::hardware::gnss
