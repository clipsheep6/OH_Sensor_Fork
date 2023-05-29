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
std::vector<int32_t> supportSensors = {1, 14, 15};
float testData[] = {9.8, 0.0, 0.0};
float colorData[] = {2.2, 3.3};
float sarData[] = {8.8};
SensorEvent testEvent = {
    .sensorTypeId = 1,
    .data = (uint8_t *)testData,
    .dataLen = 12
};
SensorEvent colorEvent = {
    .sensorTypeId = 14,
    .data = (uint8_t *)colorData,
    .dataLen = 8
};
SensorEvent sarEvent = {
    .sensorTypeId = 15,
    .data = (uint8_t *)sarData,
    .dataLen = 4
};
}
std::vector<int32_t> HdiServiceImpl::g_enableSensors;
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
                continue;
            }
            for (const auto &iter : g_enableSensors) {
                if (iter == 14) {
                    it(&colorEvent);
                } else if (iter == 15) {
                    it(&sarEvent);
                } else {
                    it(&testEvent);
                }
            }
        }
        if (isStop_) {
            break;
        }
    }
    SEN_HILOGI("thread stop");
    return;
}

int32_t HdiServiceImpl::EnableSensor(int32_t sensorId)
{
    CALL_LOG_ENTER;
    if (std::find(supportSensors.begin(), supportSensors.end(), sensorId) == supportSensors.end()) {
        SEN_HILOGE("not support enable sensorId:%{public}d", sensorId);
        return ERR_NO_INIT;
    }
    if (std::find(g_enableSensors.begin(), g_enableSensors.end(), sensorId) != g_enableSensors.end()) {
        SEN_HILOGI("sensorId:%{public}d has been enabled", sensorId);
        return ERR_OK;
    }
    g_enableSensors.push_back(sensorId);
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
    if (std::find(supportSensors.begin(), supportSensors.end(), sensorId) == supportSensors.end()) {
        SEN_HILOGE("not support disable sensorId:%{public}d", sensorId);
        return ERR_NO_INIT;
    }
    if (std::find(g_enableSensors.begin(), g_enableSensors.end(), sensorId) == g_enableSensors.end()) {
        SEN_HILOGE("sensorId:%{public}d should be enable first", sensorId);
        return ERR_NO_INIT;
    }
    std::vector<int32_t>::iterator iter;
    for (iter = g_enableSensors.begin(); iter != g_enableSensors.end(); ++iter) {
        if (*iter == sensorId) {
            g_enableSensors.erase(iter++);
            break;
        }
    }
    if (g_enableSensors.empty()) {
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