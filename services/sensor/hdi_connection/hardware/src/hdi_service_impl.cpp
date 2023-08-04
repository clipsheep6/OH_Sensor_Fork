/*
 * Copyright (c) 2021 Huawei Device Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "hdi_service_impl.h"

#include <thread>
#include <unistd.h>

#include "sensors_errors.h"

namespace OHOS {
namespace Sensors {
using namespace OHOS::HiviewDFX;

namespace {
constexpr HiLogLabel LABEL = { LOG_CORE, SENSOR_LOG_DOMAIN, "HdiServiceImpl" };
constexpr int64_t SAMPLING_INTERVAL_NS = 200000000;
constexpr int32_t CONVERT_MULTIPLES = 1000;
std::vector<SensorInfo> g_sensorInfos = {
    {"sensor_test", "default", "1.0.0", "1.0.0", 0, 1, 9999.0, 0.000001, 23.0, 100000000, 1000000000},
};
std::vector<int32_t> g_supportSensors = {
    SENSOR_TYPE_ID_ACCELEROMETER,
    SENSOR_TYPE_ID_COLOR,
    SENSOR_TYPE_ID_SAR,
    SENSOR_TYPE_ID_POSTURE
};
float g_testData[] = { 9.8, 0.0, 0.0 };
float g_colorData[] = { 2.2, 3.3 };
float g_sarData[] = { 8.8 };
float g_postureData[] = { 9.8, 0.0, 0.0, 9.8, 0.0, 0.0, 180.0 };
SensorEvent g_testEvent = {
    .sensorTypeId = SENSOR_TYPE_ID_ACCELEROMETER,
    .data = reinterpret_cast<uint8_t *>(g_testData),
    .dataLen = 12
};
SensorEvent g_colorEvent = {
    .sensorTypeId = SENSOR_TYPE_ID_COLOR,
    .data = reinterpret_cast<uint8_t *>(g_colorData),
    .dataLen = 8
};
SensorEvent g_sarEvent = {
    .sensorTypeId = SENSOR_TYPE_ID_SAR,
    .data = reinterpret_cast<uint8_t *>(g_sarData),
    .dataLen = 4
};
SensorEvent g_postureEvent = {
    .sensorTypeId = SENSOR_TYPE_ID_POSTURE,
    .data = reinterpret_cast<uint8_t *>(g_postureData),
    .dataLen = 28
};
}
std::vector<int32_t> HdiServiceImpl::enableSensors_;
std::vector<RecordSensorCallback> HdiServiceImpl::callbacks_;
int64_t HdiServiceImpl::samplingInterval_ = -1;
int64_t HdiServiceImpl::reportInterval_ = -1;
std::atomic_bool HdiServiceImpl::isStop_ = false;

int32_t HdiServiceImpl::GetSensorList(std::vector<SensorInfo>& sensorList)
{
    CALL_LOG_ENTER;
    sensorList.assign(g_sensorInfos.begin(), g_sensorInfos.end());
    return ERR_OK;
}

void HdiServiceImpl::DataReportThread()
{
    CALL_LOG_ENTER;
    while (true) {
        usleep(samplingInterval_ / CONVERT_MULTIPLES);
        for (const auto &it : callbacks_) {
            if (it == nullptr) {
                SEN_HILOGW("RecordSensorCallback is null");
                continue;
            }
            for (const auto &iter : enableSensors_) {
                if (iter == SENSOR_TYPE_ID_COLOR) {
                    it(&g_colorEvent);
                } else if (iter == SENSOR_TYPE_ID_SAR) {
                    it(&g_sarEvent);
                } else if (iter == SENSOR_TYPE_ID_POSTURE) {
                    it(&g_postureEvent);
                } else {
                    it(&g_testEvent);
                }
            }
        }
        if (isStop_) {
            break;
        }
    }
    SEN_HILOGI("Thread stop");
    return;
}

int32_t HdiServiceImpl::EnableSensor(int32_t sensorId)
{
    CALL_LOG_ENTER;
    if (std::find(g_supportSensors.begin(), g_supportSensors.end(), sensorId) == g_supportSensors.end()) {
        SEN_HILOGE("Not support enable sensorId:%{public}d", sensorId);
        return ERR_NO_INIT;
    }
    if (std::find(enableSensors_.begin(), enableSensors_.end(), sensorId) != enableSensors_.end()) {
        SEN_HILOGI("sensorId:%{public}d has been enabled", sensorId);
        return ERR_OK;
    }
    enableSensors_.push_back(sensorId);
    if (!dataReportThread_.joinable() || isStop_) {
        if (dataReportThread_.joinable()) {
            dataReportThread_.join();
        }
        std::thread senocdDataThread(HdiServiceImpl::DataReportThread);
        dataReportThread_ = std::move(senocdDataThread);
        isStop_ = false;
    }
    return ERR_OK;
};

int32_t HdiServiceImpl::DisableSensor(int32_t sensorId)
{
    CALL_LOG_ENTER;
    if (std::find(g_supportSensors.begin(), g_supportSensors.end(), sensorId) == g_supportSensors.end()) {
        SEN_HILOGE("Not support disable sensorId:%{public}d", sensorId);
        return ERR_NO_INIT;
    }
    if (std::find(enableSensors_.begin(), enableSensors_.end(), sensorId) == enableSensors_.end()) {
        SEN_HILOGE("sensorId:%{public}d should be enable first", sensorId);
        return ERR_NO_INIT;
    }
    std::vector<int32_t>::iterator iter;
    for (iter = enableSensors_.begin(); iter != enableSensors_.end();) {
        if (*iter == sensorId) {
            iter = enableSensors_.erase(iter);
            break;
        } else {
            ++iter;
        }
    }
    if (enableSensors_.empty()) {
        isStop_ = true;
    }
    return ERR_OK;
}

int32_t HdiServiceImpl::SetBatch(int32_t sensorId, int64_t samplingInterval, int64_t reportInterval)
{
    CALL_LOG_ENTER;
    if (samplingInterval < 0 || reportInterval < 0) {
        samplingInterval = SAMPLING_INTERVAL_NS;
        reportInterval = 0;
    }
    samplingInterval_ = samplingInterval;
    reportInterval_ = reportInterval;
    return ERR_OK;
}

int32_t HdiServiceImpl::SetMode(int32_t sensorId, int32_t mode)
{
    return ERR_OK;
}

int32_t HdiServiceImpl::Register(RecordSensorCallback cb)
{
    CHKPR(cb, ERROR);
    callbacks_.push_back(cb);
    return ERR_OK;
}

int32_t HdiServiceImpl::Unregister()
{
    isStop_ = true;
    return ERR_OK;
}
}  // namespace Sensors
}  // namespace OHOS