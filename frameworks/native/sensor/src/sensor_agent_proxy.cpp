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

#include "sensor_agent_proxy.h"

#include <cstring>

#include "securec.h"
#include "sensor_catalog.h"
#include "sensor_service_client.h"
#include "sensors_errors.h"
#include "sensors_log_domain.h"

using namespace OHOS::HiviewDFX;
namespace OHOS {
namespace Sensors {
namespace {
constexpr HiLogLabel LABEL = { LOG_CORE, OHOS::SensorsLogDomain::SENSORS_IMPLEMENT, "SensorAgentProxy" };

using OHOS::ERR_OK;
using OHOS::Sensors::BODY;
using OHOS::Sensors::DEVICE_MOTION;
using OHOS::Sensors::ENVIRONMENT;
using OHOS::Sensors::INVALID_POINTER;
using OHOS::Sensors::LIGHT;
using OHOS::Sensors::ORIENTATION;
using OHOS::Sensors::OTHER;
using OHOS::Sensors::SENSOR_TYPE_6DOF_ATTITUDE;
using OHOS::Sensors::SENSOR_TYPE_ACCELEROMETER;
using OHOS::Sensors::SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
using OHOS::Sensors::SENSOR_TYPE_AMBIENT_LIGHT;
using OHOS::Sensors::SENSOR_TYPE_AMBIENT_TEMPERATURE;
using OHOS::Sensors::SENSOR_TYPE_BAROMETER;
using OHOS::Sensors::SENSOR_TYPE_DEVICE_ORIENTATION;
using OHOS::Sensors::SENSOR_TYPE_GAME_ROTATION_VECTOR;
using OHOS::Sensors::SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR;
using OHOS::Sensors::SENSOR_TYPE_GRAVITY;
using OHOS::Sensors::SENSOR_TYPE_GYROSCOPE;
using OHOS::Sensors::SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
using OHOS::Sensors::SENSOR_TYPE_HEART_RATE_DETECTOR;
using OHOS::Sensors::SENSOR_TYPE_HUMIDITY;
using OHOS::Sensors::SENSOR_TYPE_LINEAR_ACCELERATION;
using OHOS::Sensors::SENSOR_TYPE_MAGNETIC_FIELD;
using OHOS::Sensors::SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
using OHOS::Sensors::SENSOR_TYPE_ORIENTATION;
using OHOS::Sensors::SENSOR_TYPE_PRESSURE_DETECTOR;
using OHOS::Sensors::SENSOR_TYPE_PROXIMITY;
using OHOS::Sensors::SENSOR_TYPE_ROTATION_VECTOR;
using OHOS::Sensors::SENSOR_TYPE_SIGNIFICANT_MOTION;
using OHOS::Sensors::SENSOR_TYPE_STEP_COUNTER;
using OHOS::Sensors::SENSOR_TYPE_STEP_DETECTOR;
using OHOS::Sensors::SENSOR_TYPE_WEAR_DETECTOR;
using OHOS::Sensors::SensorDataChannel;
using OHOS::Sensors::SensorServiceClient;
}  // namespace

OHOS::sptr<SensorAgentProxy> SensorAgentProxy::sensorObj_ = nullptr;
bool SensorAgentProxy::g_isChannelCreated;
int64_t SensorAgentProxy::g_samplingInterval;
int64_t SensorAgentProxy::g_reportInterval;
std::mutex SensorAgentProxy::subscribeMutex_;
std::mutex SensorAgentProxy::chanelMutex_;
std::map<int32_t, const SensorUser *> SensorAgentProxy::g_subscribeMap;
std::map<int32_t, const SensorUser *> SensorAgentProxy::g_unsubscribeMap;

SensorAgentProxy::SensorAgentProxy()
    : dataChannel_(new (std::nothrow) SensorDataChannel())
{}

const SensorAgentProxy *SensorAgentProxy::GetSensorsObj()
{
    HiLog::Debug(LABEL, "%{public}s", __func__);

    if (sensorObj_ == nullptr) {
        HiLog::Debug(LABEL, "%{public}s sensorObj_ new object", __func__);
        sensorObj_ = new (std::nothrow) SensorAgentProxy();
    }
    return sensorObj_;
}

void SensorAgentProxy::HandleSensorData(struct SensorEvent *events, int32_t num, void *data)
{
    if (events == nullptr || num <= 0) {
        SEN_HILOGE("events is null or num is invalid");
        return;
    }
    struct SensorEvent eventStream;
    for (int32_t i = 0; i < num; ++i) {
        eventStream = events[i];
        CHKPV(eventStream.data);
        if (g_subscribeMap.find(eventStream.sensorTypeId) == g_subscribeMap.end()) {
            SEN_HILOGE("sensorTypeId not in g_subscribeMap");
            return;
        }
        if (g_subscribeMap[eventStream.sensorTypeId] == nullptr) {
            SEN_HILOGE("sensorTypeId in g_subscribeMap is nullptr");
            return;
        }
        g_subscribeMap[eventStream.sensorTypeId]->callback(&eventStream);
    }
}

int32_t SensorAgentProxy::CreateSensorDataChannel() const
{
    HiLog::Debug(LABEL, "%{public}s", __func__);
    std::lock_guard<std::mutex> chanelLock(chanelMutex_);
    if (g_isChannelCreated) {
        HiLog::Info(LABEL, "%{public}s the channel has already been created", __func__);
        return ERR_OK;
    }
    CHKPR(dataChannel_, INVALID_POINTER);
    auto ret = dataChannel_->CreateSensorDataChannel(HandleSensorData, nullptr);
    if (ret != ERR_OK) {
        SEN_HILOGE("create data channel failed, ret: %{public}d", ret);
        return ret;
    }
    auto &client = SensorServiceClient::GetInstance();
    ret = client.TransferDataChannel(dataChannel_);
    if (ret != ERR_OK) {
        auto destoryRet = dataChannel_->DestroySensorDataChannel();
        HiLog::Error(LABEL, "%{public}s transfer data channel failed, ret : %{public}d, destoryRet : %{public}d",
                     __func__, ret, destoryRet);
        return ret;
    }
    g_isChannelCreated = true;
    return ERR_OK;
}

int32_t SensorAgentProxy::DestroySensorDataChannel() const
{
    HiLog::Debug(LABEL, "%{public}s", __func__);
    std::lock_guard<std::mutex> chanelLock(chanelMutex_);
    if (!g_isChannelCreated) {
        HiLog::Info(LABEL, "%{public}s channel has been destroyed", __func__);
        return ERR_OK;
    }
    CHKPR(dataChannel_, INVALID_POINTER);
    int32_t ret = dataChannel_->DestroySensorDataChannel();
    if (ret != ERR_OK) {
        HiLog::Error(LABEL, "%{public}s destory data channel failed, ret : %{public}d", __func__, ret);
        return ret;
    }
    SensorServiceClient &client = SensorServiceClient::GetInstance();
    ret = client.DestroyDataChannel();
    if (ret != ERR_OK) {
        HiLog::Error(LABEL, "%{public}s destory service data channel fail, ret : %{public}d", __func__, ret);
        return ret;
    }
    g_isChannelCreated = false;
    return ERR_OK;
}

int32_t SensorAgentProxy::ActivateSensor(int32_t sensorId, const SensorUser *user) const
{
    if (user == nullptr || sensorId < 0 || user->callback == nullptr) {
        HiLog::Error(LABEL, "%{public}s user is null or sensorId is invalid", __func__);
        return OHOS::Sensors::ERROR;
    }
    if (g_samplingInterval < 0 || g_reportInterval < 0) {
        HiLog::Error(LABEL, "%{public}s samplingPeroid or g_reportInterval is invalid", __func__);
        return OHOS::Sensors::ERROR;
    }
    std::lock_guard<std::mutex> subscribeLock(subscribeMutex_);
    if ((g_subscribeMap.find(sensorId) == g_subscribeMap.end()) || (g_subscribeMap[sensorId] != user)) {
        HiLog::Error(LABEL, "%{public}s subscribe sensorId first", __func__);
        return OHOS::Sensors::ERROR;
    }
    SensorServiceClient &client = SensorServiceClient::GetInstance();
    int32_t ret = client.EnableSensor(sensorId, g_samplingInterval, g_reportInterval);
    g_samplingInterval = -1;
    g_reportInterval = -1;
    if (ret != 0) {
        HiLog::Error(LABEL, "%{public}s enable sensor failed, ret: %{public}d", __func__, ret);
        g_subscribeMap.erase(sensorId);

        return OHOS::Sensors::ERROR;
    }
    return OHOS::Sensors::SUCCESS;
}

int32_t SensorAgentProxy::DeactivateSensor(int32_t sensorId, const SensorUser *user) const
{
    if (user == nullptr || sensorId < 0 || user->callback == nullptr) {
        HiLog::Error(LABEL, "%{public}s user is null or sensorId is invalid", __func__);
        return OHOS::Sensors::ERROR;
    }
    std::lock_guard<std::mutex> subscribeLock(subscribeMutex_);
    if ((g_subscribeMap.find(sensorId) == g_subscribeMap.end()) || (g_subscribeMap[sensorId] != user)) {
        HiLog::Error(LABEL, "%{public}s subscribe sensorId first", __func__);
        return OHOS::Sensors::ERROR;
    }
    g_subscribeMap.erase(sensorId);
    g_unsubscribeMap[sensorId] = user;
    SensorServiceClient &client = SensorServiceClient::GetInstance();
    int32_t ret = client.DisableSensor(sensorId);
    if (ret != 0) {
        HiLog::Error(LABEL, "%{public}s disable sensor failed, ret: %{public}d", __func__, ret);
        return OHOS::Sensors::ERROR;
    }
    return OHOS::Sensors::SUCCESS;
}

int32_t SensorAgentProxy::SetBatch(int32_t sensorId, const SensorUser *user, int64_t samplingInterval,
                                   int64_t reportInterval) const
{
    if (user == nullptr || sensorId < 0) {
        HiLog::Error(LABEL, "%{public}s user is null or sensorId is invalid", __func__);
        return OHOS::Sensors::ERROR;
    }
    if (samplingInterval < 0 || reportInterval < 0) {
        HiLog::Error(LABEL, "%{public}s samplingInterval or reportInterval is invalid", __func__);
        return OHOS::Sensors::ERROR;
    }
    std::lock_guard<std::mutex> subscribeLock(subscribeMutex_);
    if ((g_subscribeMap.find(sensorId) == g_subscribeMap.end()) || (g_subscribeMap.at(sensorId) != user)) {
        HiLog::Error(LABEL, "%{public}s subscribe sensorId first", __func__);
        return OHOS::Sensors::ERROR;
    }
    g_samplingInterval = samplingInterval;
    g_reportInterval = reportInterval;
    return OHOS::Sensors::SUCCESS;
}

int32_t SensorAgentProxy::SubscribeSensor(int32_t sensorId, const SensorUser *user) const
{
    HiLog::Info(LABEL, "%{public}s in, sensorId: %{public}d", __func__, sensorId);
    if (user == nullptr || sensorId < 0 || user->callback == nullptr) {
        HiLog::Error(LABEL, "%{public}s user or sensorId is invalid", __func__);
        return OHOS::Sensors::ERROR;
    }
    int32_t ret = CreateSensorDataChannel();
    if (ret != ERR_OK) {
        HiLog::Error(LABEL, "%{public}s create sensor data chanel failed", __func__);
        return OHOS::Sensors::ERROR;
    }
    std::lock_guard<std::mutex> subscribeLock(subscribeMutex_);
    g_subscribeMap[sensorId] = user;
    return OHOS::Sensors::SUCCESS;
}

int32_t SensorAgentProxy::UnsubscribeSensor(int32_t sensorId, const SensorUser *user) const
{
    HiLog::Info(LABEL, "%{public}s in, sensorId: %{public}d", __func__, sensorId);
    if (user == nullptr || sensorId < 0  || user->callback == nullptr) {
        HiLog::Error(LABEL, "%{public}s user is null or sensorId is invalid", __func__);
        return OHOS::Sensors::ERROR;
    }
    std::lock_guard<std::mutex> subscribeLock(subscribeMutex_);
    if (g_unsubscribeMap.find(sensorId) == g_unsubscribeMap.end() || g_unsubscribeMap[sensorId] != user) {
        HiLog::Error(LABEL, "%{public}s deactivate sensorId first", __func__);
        return OHOS::Sensors::ERROR;
    }
    if (g_subscribeMap.empty()) {
        int32_t ret = DestroySensorDataChannel();
        if (ret != ERR_OK) {
            HiLog::Error(LABEL, "%{public}s destory data channel fail, ret : %{public}d", __func__, ret);
            return ret;
        }
    }
    g_unsubscribeMap.erase(sensorId);
    return OHOS::Sensors::SUCCESS;
}

int32_t SensorAgentProxy::SetMode(int32_t sensorId, const SensorUser *user, int32_t mode) const
{
    if (user == nullptr || sensorId < 0 || user->callback == nullptr) {
        HiLog::Error(LABEL, "%{public}s user is null or sensorId is invalid", __func__);
        return OHOS::Sensors::ERROR;
    }
    std::lock_guard<std::mutex> subscribeLock(subscribeMutex_);
    if ((g_subscribeMap.find(sensorId) == g_subscribeMap.end()) || (g_subscribeMap.at(sensorId) != user)) {
        HiLog::Error(LABEL, "%{public}s subscribe sensorId first", __func__);
        return OHOS::Sensors::ERROR;
    }
    return OHOS::Sensors::SUCCESS;
}

int32_t SensorAgentProxy::SetOption(int32_t sensorId, const SensorUser *user, int32_t option) const
{
    if (user == nullptr || sensorId < 0 || user->callback == nullptr) {
        HiLog::Error(LABEL, "%{public}s user is null or sensorId is invalid", __func__);
        return OHOS::Sensors::ERROR;
    }
    std::lock_guard<std::mutex> subscribeLock(subscribeMutex_);
    if ((g_subscribeMap.find(sensorId) == g_subscribeMap.end()) || (g_subscribeMap.at(sensorId) != user)) {
        HiLog::Error(LABEL, "%{public}s subscribe sensorId first", __func__);
        return OHOS::Sensors::ERROR;
    }
    return OHOS::Sensors::SUCCESS;
}

int32_t SensorAgentProxy::GetAllSensors(SensorInfo **sensorInfo, int32_t *count) const
{
    if (sensorInfo == nullptr || count == nullptr) {
        HiLog::Error(LABEL, "%{public}s sensorInfo or count is null", __func__);
        return OHOS::Sensors::ERROR;
    }
    SensorServiceClient &client = SensorServiceClient::GetInstance();
    std::vector<OHOS::Sensors::Sensor> sensorList_ = client.GetSensorList();
    if (sensorList_.empty()) {
        HiLog::Error(LABEL, "%{public}s get sensor lists failed", __func__);
        return OHOS::Sensors::ERROR;
    }
    *count = sensorList_.size();
    *sensorInfo = (SensorInfo *)malloc(sizeof(SensorInfo) * (*count));
    CHKPR(*sensorInfo, ERROR);
    for (int32_t index = 0; index < *count; ++index) {
        errno_t ret = strcpy_s((*sensorInfo + index)->sensorName, NAME_MAX_LEN,
            sensorList_[index].GetSensorName().c_str());
        if (ret != EOK) {
            HiLog::Error(LABEL, "%{public}s strcpy sensorName failed", __func__);
            return OHOS::Sensors::ERROR;
        }
        ret = strcpy_s((*sensorInfo + index)->vendorName, NAME_MAX_LEN,
            sensorList_[index].GetVendorName().c_str());
        if (ret != EOK) {
            HiLog::Error(LABEL, "%{public}s strcpy vendorName failed", __func__);
            return OHOS::Sensors::ERROR;
        }
        ret = strcpy_s((*sensorInfo + index)->hardwareVersion, VERSION_MAX_LEN,
            sensorList_[index].GetHardwareVersion().c_str());
        if (ret != EOK) {
            HiLog::Error(LABEL, "%{public}s strcpy hardwareVersion failed", __func__);
            return OHOS::Sensors::ERROR;
        }
        ret = strcpy_s((*sensorInfo + index)->firmwareVersion, VERSION_MAX_LEN,
            sensorList_[index].GetFirmwareVersion().c_str());
        if (ret != EOK) {
            HiLog::Error(LABEL, "%{public}s strcpy hardwareVersion failed", __func__);
            return OHOS::Sensors::ERROR;
        }
        (*sensorInfo + index)->sensorId = static_cast<int32_t>(sensorList_[index].GetSensorId());
        (*sensorInfo + index)->sensorTypeId = static_cast<int32_t>(sensorList_[index].GetSensorTypeId());
        (*sensorInfo + index)->maxRange = sensorList_[index].GetMaxRange();
        (*sensorInfo + index)->precision = sensorList_[index].GetResolution();
        (*sensorInfo + index)->power = sensorList_[index].GetPower();
    }
    return OHOS::Sensors::SUCCESS;
}
}  // namespace Sensors
}  // namespace OHOS