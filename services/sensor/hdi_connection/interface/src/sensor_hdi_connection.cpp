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
#include "sensor_hdi_connection.h"

#include "compatible_connection.h"
#include "hdi_connection.h"
#include "hitrace_meter.h"
#include "sensor_errors.h"

namespace OHOS {
namespace Sensors {
using namespace OHOS::HiviewDFX;
namespace {
constexpr HiLogLabel LABEL = { LOG_CORE, SENSOR_LOG_DOMAIN, "SensorHdiConnection" };
constexpr float MAX_RANGE = 9999.0;
constexpr float POWER = 20.0;
constexpr float RESOLITION = 0.000001;
constexpr float MIN_SAMPLE_PERIOD_NS = 100000000;
constexpr float MAX_SAMPLE_PERIOD_NS = 1000000000;
const std::string VERSION_NAME = "1.0.1";
std::unordered_set<int32_t> g_supportMockSensors = { SENSOR_TYPE_ID_COLOR, SENSOR_TYPE_ID_SAR, SENSOR_TYPE_ID_POSTURE };
}

int32_t SensorHdiConnection::ConnectHdi()
{
    iSensorHdiConnection_ = std::make_unique<HdiConnection>();
    int32_t ret = ConnectHdiService();
    if (ret != ERR_OK) {
        SEN_HILOGE("Connect hdi service failed, try to connect compatible connection, ret:%{public}d", ret);
        iSensorHdiConnection_ = std::make_unique<CompatibleConnection>();
        ret = ConnectHdiService();
        if (ret != ERR_OK) {
            SEN_HILOGE("Connect compatible connection failed, ret:%{public}d", ret);
            return ret;
        }
    }
    if (!CheckAllInSensorSet(g_supportMockSensors)) {
        SEN_HILOGD("SensorList not contain all mock sensors, connect compatible connection");
        ret = ConnectCompatibleHdi();
        if (ret != ERR_OK) {
            SEN_HILOGE("Connect mock sensors compatible connection failed, ret:%{public}d", ret);
        }
        return ret;
    }
    return ERR_OK;
}

int32_t SensorHdiConnection::ConnectHdiService()
{
    int32_t ret = iSensorHdiConnection_->ConnectHdi();
    if (ret != 0) {
        SEN_HILOGE("Connect hdi service failed");
        return CONNECT_SENSOR_HDF_ERR;
    }
    std::lock_guard<std::mutex> sensorLock(sensorMutex_);
    ret = iSensorHdiConnection_->GetSensorList(sensorList_);
    if (ret != 0) {
        SEN_HILOGE("Get sensor list failed");
        return GET_SENSOR_LIST_ERR;
    }
    for (const auto &sensor : sensorList_) {
        sensorSet_.insert(sensor.GetSensorId());
    }
    return ERR_OK;
}

int32_t SensorHdiConnection::ConnectCompatibleHdi()
{
    if (iSensorCompatibleHdiConnection_ == nullptr) {
        iSensorCompatibleHdiConnection_ = std::make_unique<CompatibleConnection>();
    }
    int32_t ret = iSensorCompatibleHdiConnection_->ConnectHdi();
    if (ret != ERR_OK) {
        SEN_HILOGE("Connect hdi compatible service failed");
        return CONNECT_SENSOR_HDF_ERR;
    }
    return ERR_OK;
}

bool SensorHdiConnection::CheckAllInSensorSet(const std::unordered_set<int32_t> &sensors)
{
    std::lock_guard<std::mutex> sensorLock(sensorMutex_);
    for (const auto &sensorId : sensors) {
        if (sensorSet_.find(sensorId) == sensorSet_.end()) {
            return false;
        }
    }
    return true;
}

bool SensorHdiConnection::CheckOneInSensorSet(int32_t sensorId)
{
    std::lock_guard<std::mutex> sensorLock(sensorMutex_);
    if (sensorSet_.find(sensorId) == sensorSet_.end()) {
        return false;
    }
    return true;
}

bool SensorHdiConnection::CheckOneInMockSet(int32_t sensorId)
{
    std::lock_guard<std::mutex> mockLock(mockMutex_);
    if (mockSet_.find(sensorId) == mockSet_.end()) {
        return false;
    }
    return true;
}

int32_t SensorHdiConnection::InjectMockSensor(int32_t sensorId)
{
    if (CheckOneInSensorSet(sensorId)) {
        SEN_HILOGE("sensorId exist in sensorList, no need inject");
        return ERROR;
    }
    if (g_supportMockSensors.find(sensorId) == g_supportMockSensors.end()) {
        SEN_HILOGE("no support cur sensor inject");
        return ERROR;
    }
    std::lock_guard<std::mutex> mockLock(mockMutex_);
    auto pairRet = mockSet_.insert(sensorId); // 保存注入的打桩传感器
    if (!pairRet.second) {
        SEN_HILOGE("mockSet_ insert sensorId fail, sensorId:%{public}d", sensorId);
        return ERROR;
    }
    return ERR_OK;
}

int32_t SensorHdiConnection::UninjectMockSensor(int32_t sensorId)
{
    std::lock_guard<std::mutex> mockLock(mockMutex_);
    auto it = mockSet_.find(sensorId);
    if (it == mockSet_.end()) {
        SEN_HILOGE("mockSet_ not find sensorId, sensorId:%{public}d", sensorId);
        return ERROR;
    }
    mockSet_.erase(it); // 删除注入的打桩传感器
    return ERR_OK;
}

Sensor SensorHdiConnection::GenerateColorSensor()
{
    Sensor sensorColor;
    sensorColor.SetSensorId(SENSOR_TYPE_ID_COLOR);
    sensorColor.SetSensorTypeId(SENSOR_TYPE_ID_COLOR);
    sensorColor.SetFirmwareVersion(VERSION_NAME);
    sensorColor.SetHardwareVersion(VERSION_NAME);
    sensorColor.SetMaxRange(MAX_RANGE);
    sensorColor.SetSensorName("sensor_color");
    sensorColor.SetVendorName("default_color");
    sensorColor.SetResolution(RESOLITION);
    sensorColor.SetPower(POWER);
    sensorColor.SetMinSamplePeriodNs(MIN_SAMPLE_PERIOD_NS);
    sensorColor.SetMaxSamplePeriodNs(MAX_SAMPLE_PERIOD_NS);
    return sensorColor;
}

Sensor SensorHdiConnection::GenerateSarSensor()
{
    Sensor sensorSar;
    sensorSar.SetSensorId(SENSOR_TYPE_ID_SAR);
    sensorSar.SetSensorTypeId(SENSOR_TYPE_ID_SAR);
    sensorSar.SetFirmwareVersion(VERSION_NAME);
    sensorSar.SetHardwareVersion(VERSION_NAME);
    sensorSar.SetMaxRange(MAX_RANGE);
    sensorSar.SetSensorName("sensor_sar");
    sensorSar.SetVendorName("default_sar");
    sensorSar.SetResolution(RESOLITION);
    sensorSar.SetPower(POWER);
    sensorSar.SetMinSamplePeriodNs(MIN_SAMPLE_PERIOD_NS);
    sensorSar.SetMaxSamplePeriodNs(MAX_SAMPLE_PERIOD_NS);
    return sensorSar;
}

Sensor SensorHdiConnection::GeneratePostureSensor()
{
    Sensor sensorPosture;
    sensorPosture.SetSensorId(SENSOR_TYPE_ID_POSTURE);
    sensorPosture.SetSensorTypeId(SENSOR_TYPE_ID_POSTURE);
    sensorPosture.SetFirmwareVersion(VERSION_NAME);
    sensorPosture.SetHardwareVersion(VERSION_NAME);
    sensorPosture.SetMaxRange(MAX_RANGE);
    sensorPosture.SetSensorName("sensor_posture");
    sensorPosture.SetVendorName("default_posture");
    sensorPosture.SetResolution(RESOLITION);
    sensorPosture.SetPower(POWER);
    sensorPosture.SetMinSamplePeriodNs(MIN_SAMPLE_PERIOD_NS);
    sensorPosture.SetMaxSamplePeriodNs(MAX_SAMPLE_PERIOD_NS);
    return sensorPosture;
}

int32_t SensorHdiConnection::GetSensorList(std::vector<Sensor> &sensorList)
{
    {
        std::lock_guard<std::mutex> sensorLock(sensorMutex_);
        sensorList.assign(sensorList_.begin(), sensorList_.end());
    }
    std::lock_guard<std::mutex> mockLock(mockMutex_);
    for (const auto &sensorId : mockSet_) {
        switch (sensorId) {
            case SENSOR_TYPE_ID_COLOR:
                sensorList.push_back(GenerateColorSensor());
                break;
            case SENSOR_TYPE_ID_SAR:
                sensorList.push_back(GenerateSarSensor());
                break;
            case SENSOR_TYPE_ID_POSTURE:
                sensorList.push_back(GeneratePostureSensor());
                break;
            default:
                break;
        }
    }
    return ERR_OK;
}

int32_t SensorHdiConnection::EnableSensor(int32_t sensorId)
{
    StartTrace(HITRACE_TAG_SENSORS, "EnableSensor");
    int32_t ret = ENABLE_SENSOR_ERR;
    if (CheckOneInMockSet(sensorId)) {
        // 当前sensor在注入打桩的set中，走打桩
        CHKPR(iSensorCompatibleHdiConnection_, ENABLE_SENSOR_ERR);
        ret = iSensorCompatibleHdiConnection_->EnableSensor(sensorId);
        FinishTrace(HITRACE_TAG_SENSORS);
        if (ret != ERR_OK) {
            SEN_HILOGE("Enable sensor failed in compatible, sensorId:%{public}d", sensorId);
            return ENABLE_SENSOR_ERR;
        }
        return ret;
    }
    CHKPR(iSensorHdiConnection_, ENABLE_SENSOR_ERR);
    ret = iSensorHdiConnection_->EnableSensor(sensorId);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != ERR_OK) {
        SEN_HILOGI("Enable sensor failed, sensorId:%{public}d", sensorId);
        return ENABLE_SENSOR_ERR;
    }
    return ret;
};

int32_t SensorHdiConnection::DisableSensor(int32_t sensorId)
{
    StartTrace(HITRACE_TAG_SENSORS, "DisableSensor");
    int32_t ret = DISABLE_SENSOR_ERR;
    if (CheckOneInMockSet(sensorId)) {
        // 当前sensor在注入打桩的set中，走打桩
        CHKPR(iSensorCompatibleHdiConnection_, DISABLE_SENSOR_ERR);
        ret = iSensorCompatibleHdiConnection_->DisableSensor(sensorId);
        FinishTrace(HITRACE_TAG_SENSORS);
        if (ret != ERR_OK) {
            SEN_HILOGE("Disable sensor failed in compatible, sensorId:%{public}d", sensorId);
            return DISABLE_SENSOR_ERR;
        }
        return ret;
    }
    CHKPR(iSensorHdiConnection_, DISABLE_SENSOR_ERR);
    ret = iSensorHdiConnection_->DisableSensor(sensorId);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != ERR_OK) {
        SEN_HILOGI("Disable sensor failed, sensorId:%{public}d", sensorId);
        return DISABLE_SENSOR_ERR;
    }
    return ret;
}

int32_t SensorHdiConnection::SetBatch(int32_t sensorId, int64_t samplingInterval, int64_t reportInterval)
{
    StartTrace(HITRACE_TAG_SENSORS, "SetBatch");
    int32_t ret = SET_SENSOR_CONFIG_ERR;
    if (CheckOneInMockSet(sensorId)) {
        // 当前sensor在注入打桩的set中，走打桩
        CHKPR(iSensorCompatibleHdiConnection_, SET_SENSOR_CONFIG_ERR);
        ret = iSensorCompatibleHdiConnection_->SetBatch(sensorId, samplingInterval, reportInterval);
        FinishTrace(HITRACE_TAG_SENSORS);
        if (ret != ERR_OK) {
            SEN_HILOGI("Set batch failed in compatible, sensorId:%{public}d", sensorId);
            return SET_SENSOR_CONFIG_ERR;
        }
        return ret;
    }
    CHKPR(iSensorHdiConnection_, SET_SENSOR_CONFIG_ERR);
    ret = iSensorHdiConnection_->SetBatch(sensorId, samplingInterval, reportInterval);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != ERR_OK) {
        SEN_HILOGI("Set batch failed, sensorId:%{public}d", sensorId);
        return SET_SENSOR_CONFIG_ERR;
    }
    return ret;
}

int32_t SensorHdiConnection::SetMode(int32_t sensorId, int32_t mode)
{
    StartTrace(HITRACE_TAG_SENSORS, "SetMode");
    int32_t ret = SET_SENSOR_MODE_ERR;
    if (CheckOneInMockSet(sensorId)) {
        // 当前sensor在注入打桩的set中，走打桩
        CHKPR(iSensorCompatibleHdiConnection_, SET_SENSOR_MODE_ERR);
        ret = iSensorCompatibleHdiConnection_->SetMode(sensorId, mode);
        FinishTrace(HITRACE_TAG_SENSORS);
        if (ret != ERR_OK) {
            SEN_HILOGI("Set mode failed, sensorId:%{public}d", sensorId);
            return SET_SENSOR_MODE_ERR;
        }
        return ret;
    }
    CHKPR(iSensorHdiConnection_, SET_SENSOR_MODE_ERR);
    ret = iSensorHdiConnection_->SetMode(sensorId, mode);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != ERR_OK) {
        SEN_HILOGI("Set mode failed, sensorId:%{public}d", sensorId);
        return SET_SENSOR_MODE_ERR;
    }
    return ret;
}

int32_t SensorHdiConnection::RegisterDataReport(ReportDataCb cb, sptr<ReportDataCallback> reportDataCallback)
{
    StartTrace(HITRACE_TAG_SENSORS, "RegisterDataReport");
    CHKPR(iSensorHdiConnection_, REGIST_CALLBACK_ERR);
    int32_t ret = iSensorHdiConnection_->RegisterDataReport(cb, reportDataCallback);
    CHKPR(iSensorCompatibleHdiConnection_, REGIST_CALLBACK_ERR);
    int32_t compatibleRet = iSensorCompatibleHdiConnection_->RegisterDataReport(cb, reportDataCallback);
    if (compatibleRet != ERR_OK) {
        SEN_HILOGE("Registe dataReport failed in compatible");
    }
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != 0) {
        SEN_HILOGI("Registe dataReport failed");
        return REGIST_CALLBACK_ERR;
    }
    return ret;
}

int32_t SensorHdiConnection::DestroyHdiConnection()
{
    CHKPR(iSensorHdiConnection_, DEVICE_ERR);
    int32_t ret = iSensorHdiConnection_->DestroyHdiConnection();
    if (ret != 0) {
        SEN_HILOGI("Destroy hdi connection failed");
        return DEVICE_ERR;
    }
    CHKPR(iSensorCompatibleHdiConnection_, DEVICE_ERR);
    int32_t compatibleRet = iSensorCompatibleHdiConnection_->DestroyHdiConnection();
    if (compatibleRet != ERR_OK) {
        SEN_HILOGE("Destroy hdi connection failed in compatible");
    }
    return ret;
}
}  // namespace Sensors
}  // namespace OHOS
