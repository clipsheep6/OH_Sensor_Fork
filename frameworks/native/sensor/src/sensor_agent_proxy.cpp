/*
 * Copyright (c) 2021-2023 Huawei Device Co., Ltd.
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

#include "sensor_service_client.h"
#include "sensors_errors.h"

using namespace OHOS::HiviewDFX;
namespace OHOS {
namespace Sensors {
namespace {
constexpr HiLogLabel LABEL = { LOG_CORE, SENSOR_LOG_DOMAIN, "SensorAgentProxy" };
constexpr uint32_t MAX_SENSOR_LIST_SIZE = 0Xffff;
}  // namespace

#define SenClient SensorServiceClient::GetInstance()
OHOS::sptr<SensorAgentProxy> SensorAgentProxy::sensorObj_ = nullptr;
bool SensorAgentProxy::g_isChannelCreated;
int64_t SensorAgentProxy::g_samplingInterval;
int64_t SensorAgentProxy::g_reportInterval;
std::recursive_mutex SensorAgentProxy::subscribeMutex_;
std::mutex SensorAgentProxy::chanelMutex_;
std::mutex sensorInfoMutex_;
SensorInfo *sensorInfos_ = nullptr;
std::mutex subscribeSensorInfoMutex_;
SubscribeSensorInfo *subscribeSensorInfos_ = nullptr;
int32_t sensorInfoCount_ = 0;
std::map<int32_t, const SensorUser *> SensorAgentProxy::g_subscribeMap;
std::map<int32_t, const SensorUser *> SensorAgentProxy::g_unsubscribeMap;

SensorAgentProxy::SensorAgentProxy()
    : dataChannel_(new (std::nothrow) SensorDataChannel())
{}

SensorAgentProxy::~SensorAgentProxy()
{
    CALL_LOG_ENTER;
    ClearSensorInfos();
}

const SensorAgentProxy *SensorAgentProxy::GetSensorsObj()
{
    CALL_LOG_ENTER;
    if (sensorObj_ == nullptr) {
        SEN_HILOGD("sensorObj_ new object");
        sensorObj_ = new (std::nothrow) SensorAgentProxy();
    }
    return sensorObj_;
}

void SensorAgentProxy::HandleSensorData(SensorEvent *events, int32_t num, void *data)
{
    CHKPV(events);
    if (num <= 0) {
        SEN_HILOGE("events is null or num is invalid");
        return;
    }
    SensorEvent eventStream;
    for (int32_t i = 0; i < num; ++i) {
        eventStream = events[i];
        std::lock_guard<std::recursive_mutex> subscribeLock(subscribeMutex_);
        auto iter = g_subscribeMap.find(eventStream.sensorTypeId);
        if (iter == g_subscribeMap.end()) {
            SEN_HILOGE("sensor is not subscribed");
            return;
        }
        const SensorUser *user = iter->second;
        CHKPV(user);
        user->callback(&eventStream);
    }
}

int32_t SensorAgentProxy::CreateSensorDataChannel() const
{
    CALL_LOG_ENTER;
    std::lock_guard<std::mutex> chanelLock(chanelMutex_);
    if (g_isChannelCreated) {
        SEN_HILOGI("the channel has already been created");
        return ERR_OK;
    }
    CHKPR(dataChannel_, INVALID_POINTER);
    auto ret = dataChannel_->CreateSensorDataChannel(HandleSensorData, nullptr);
    if (ret != ERR_OK) {
        SEN_HILOGE("create data channel failed, ret:%{public}d", ret);
        return ret;
    }
    ret = SenClient.TransferDataChannel(dataChannel_);
    if (ret != ERR_OK) {
        auto destroyRet = dataChannel_->DestroySensorDataChannel();
        SEN_HILOGE("transfer data channel failed, ret:%{public}d,destroyRet:%{public}d", ret, destroyRet);
        return ret;
    }
    g_isChannelCreated = true;
    return ERR_OK;
}

int32_t SensorAgentProxy::DestroySensorDataChannel() const
{
    CALL_LOG_ENTER;
    std::lock_guard<std::mutex> chanelLock(chanelMutex_);
    if (!g_isChannelCreated) {
        SEN_HILOGI("channel has been destroyed");
        return ERR_OK;
    }
    CHKPR(dataChannel_, INVALID_POINTER);
    int32_t ret = dataChannel_->DestroySensorDataChannel();
    if (ret != ERR_OK) {
        SEN_HILOGE("destroy data channel failed, ret:%{public}d", ret);
        return ret;
    }
    ret = SenClient.DestroyDataChannel();
    if (ret != ERR_OK) {
        SEN_HILOGE("destroy service data channel fail, ret:%{public}d", ret);
        return ret;
    }
    g_isChannelCreated = false;
    return ERR_OK;
}

int32_t SensorAgentProxy::ActivateSensor(int32_t sensorId, const SensorUser *user) const
{
    CHKPR(user, OHOS::Sensors::ERROR);
    CHKPR(user->callback, OHOS::Sensors::ERROR);
    if (g_samplingInterval < 0 || g_reportInterval < 0) {
        SEN_HILOGE("samplingPeriod or g_reportInterval is invalid");
        return ERROR;
    }
    if (!SenClient.IsValid(sensorId)) {
        SEN_HILOGE("sensorId is invalid, %{public}d", sensorId);
        return PARAMETER_ERROR;
    }
    std::lock_guard<std::recursive_mutex> subscribeLock(subscribeMutex_);
    if ((g_subscribeMap.find(sensorId) == g_subscribeMap.end()) || (g_subscribeMap[sensorId] != user)) {
        SEN_HILOGE("subscribe sensorId first");
        return ERROR;
    }
    int32_t ret = SenClient.EnableSensor(sensorId, g_samplingInterval, g_reportInterval);
    g_samplingInterval = -1;
    g_reportInterval = -1;
    if (ret != 0) {
        SEN_HILOGE("enable sensor failed, ret:%{public}d", ret);
        g_subscribeMap.erase(sensorId);
        return ret;
    }
    return ret;
}

int32_t SensorAgentProxy::DeactivateSensor(int32_t sensorId, const SensorUser *user) const
{
    CHKPR(user, OHOS::Sensors::ERROR);
    CHKPR(user->callback, OHOS::Sensors::ERROR);
    if (!SenClient.IsValid(sensorId)) {
        SEN_HILOGE("sensorId is invalid, %{public}d", sensorId);
        return PARAMETER_ERROR;
    }
    std::lock_guard<std::recursive_mutex> subscribeLock(subscribeMutex_);
    if ((g_subscribeMap.find(sensorId) == g_subscribeMap.end()) || (g_subscribeMap[sensorId] != user)) {
        SEN_HILOGE("subscribe sensorId first");
        return OHOS::Sensors::ERROR;
    }
    g_subscribeMap.erase(sensorId);
    g_unsubscribeMap[sensorId] = user;
    int32_t ret = SenClient.DisableSensor(sensorId);
    if (ret != 0) {
        SEN_HILOGE("DisableSensor failed, ret:%{public}d", ret);
        return ret;
    }
    return ret;
}

int32_t SensorAgentProxy::SetBatch(int32_t sensorId, const SensorUser *user, int64_t samplingInterval,
                                   int64_t reportInterval) const
{
    CHKPR(user, OHOS::Sensors::ERROR);
    if (!SenClient.IsValid(sensorId)) {
        SEN_HILOGE("sensorId is invalid, %{public}d", sensorId);
        return PARAMETER_ERROR;
    }
    if (samplingInterval < 0 || reportInterval < 0) {
        SEN_HILOGE("samplingInterval or reportInterval is invalid");
        return OHOS::Sensors::ERROR;
    }
    std::lock_guard<std::recursive_mutex> subscribeLock(subscribeMutex_);
    if ((g_subscribeMap.find(sensorId) == g_subscribeMap.end()) || (g_subscribeMap.at(sensorId) != user)) {
        SEN_HILOGE("subscribe sensorId first");
        return OHOS::Sensors::ERROR;
    }
    g_samplingInterval = samplingInterval;
    g_reportInterval = reportInterval;
    return OHOS::Sensors::SUCCESS;
}

int32_t SensorAgentProxy::SubscribeSensor(int32_t sensorId, const SensorUser *user) const
{
    SEN_HILOGI("in, sensorId:%{public}d", sensorId);
    CHKPR(user, OHOS::Sensors::ERROR);
    CHKPR(user->callback, OHOS::Sensors::ERROR);
    if (!SenClient.IsValid(sensorId)) {
        SEN_HILOGE("sensorId is invalid, %{public}d", sensorId);
        return PARAMETER_ERROR;
    }
    int32_t ret = CreateSensorDataChannel();
    if (ret != ERR_OK) {
        SEN_HILOGE("create sensor data chanel failed");
        return OHOS::Sensors::ERROR;
    }
    std::lock_guard<std::recursive_mutex> subscribeLock(subscribeMutex_);
    g_subscribeMap[sensorId] = user;
    return OHOS::Sensors::SUCCESS;
}

int32_t SensorAgentProxy::UnsubscribeSensor(int32_t sensorId, const SensorUser *user) const
{
    SEN_HILOGI("in, sensorId: %{public}d", sensorId);
    CHKPR(user, OHOS::Sensors::ERROR);
    CHKPR(user->callback, OHOS::Sensors::ERROR);
    if (!SenClient.IsValid(sensorId)) {
        SEN_HILOGE("sensorId is invalid, %{public}d", sensorId);
        return PARAMETER_ERROR;
    }
    std::lock_guard<std::recursive_mutex> subscribeLock(subscribeMutex_);
    if (g_unsubscribeMap.find(sensorId) == g_unsubscribeMap.end() || g_unsubscribeMap[sensorId] != user) {
        SEN_HILOGE("deactivate sensorId first");
        return OHOS::Sensors::ERROR;
    }
    if (g_subscribeMap.empty()) {
        int32_t ret = DestroySensorDataChannel();
        if (ret != ERR_OK) {
            SEN_HILOGE("destroy data channel fail, ret:%{public}d", ret);
            return ret;
        }
    }
    g_unsubscribeMap.erase(sensorId);
    return OHOS::Sensors::SUCCESS;
}

int32_t SensorAgentProxy::SetMode(int32_t sensorId, const SensorUser *user, int32_t mode) const
{
    CHKPR(user, OHOS::Sensors::ERROR);
    CHKPR(user->callback, OHOS::Sensors::ERROR);
    if (!SenClient.IsValid(sensorId)) {
        SEN_HILOGE("sensorId is invalid, %{public}d", sensorId);
        return ERROR;
    }
    std::lock_guard<std::recursive_mutex> subscribeLock(subscribeMutex_);
    if ((g_subscribeMap.find(sensorId) == g_subscribeMap.end()) || (g_subscribeMap.at(sensorId) != user)) {
        SEN_HILOGE("subscribe sensorId first");
        return OHOS::Sensors::ERROR;
    }
    return OHOS::Sensors::SUCCESS;
}

void SensorAgentProxy::ClearSensorInfos() const
{
    if (subscribeSensorInfos_ != nullptr) {
        free(subscribeSensorInfos_);
        subscribeSensorInfos_ = nullptr;
    }
    CHKPV(sensorInfos_);
    free(sensorInfos_);
    sensorInfos_ = nullptr;
}

int32_t SensorAgentProxy::ConvertSensorInfos() const
{
    CALL_LOG_ENTER;
    std::vector<Sensor> sensorList = SenClient.GetSensorList();
    if (sensorList.empty()) {
        SEN_HILOGE("get sensor lists failed");
        return ERROR;
    }
    size_t count = sensorList.size();
    if (count > MAX_SENSOR_LIST_SIZE) {
        SEN_HILOGE("The number of sensors exceeds the maximum value");
        return ERROR;
    }
    sensorInfos_ = (SensorInfo *)malloc(sizeof(SensorInfo) * count);
    CHKPR(sensorInfos_, ERROR);
    for (size_t i = 0; i < count; ++i) {
        SensorInfo *sensorInfo = sensorInfos_ + i;
        errno_t ret = strcpy_s(sensorInfo->sensorName, NAME_MAX_LEN,
            sensorList[i].GetSensorName().c_str());
        CHKCR(ret == EOK, ERROR);
        ret = strcpy_s(sensorInfo->vendorName, NAME_MAX_LEN,
            sensorList[i].GetVendorName().c_str());
        CHKCR(ret == EOK, ERROR);
        ret = strcpy_s(sensorInfo->hardwareVersion, VERSION_MAX_LEN,
            sensorList[i].GetHardwareVersion().c_str());
        CHKCR(ret == EOK, ERROR);
        ret = strcpy_s(sensorInfo->firmwareVersion, VERSION_MAX_LEN,
            sensorList[i].GetFirmwareVersion().c_str());
        CHKCR(ret == EOK, ERROR);
        sensorInfo->sensorId = static_cast<int32_t>(sensorList[i].GetSensorId());
        sensorInfo->sensorTypeId = static_cast<int32_t>(sensorList[i].GetSensorTypeId());
        sensorInfo->maxRange = sensorList[i].GetMaxRange();
        sensorInfo->precision = sensorList[i].GetResolution();
        sensorInfo->power = sensorList[i].GetPower();
        sensorInfo->minSamplePeriod = sensorList[i].GetMinSamplePeriodNs();
        sensorInfo->maxSamplePeriod = sensorList[i].GetMaxSamplePeriodNs();
    }
    sensorInfoCount_ = static_cast<int32_t>(count);
    return SUCCESS;
}

int32_t SensorAgentProxy::GetAllSensors(SensorInfo **sensorInfo, int32_t *count) const
{
    CALL_LOG_ENTER;
    CHKPR(sensorInfo, OHOS::Sensors::ERROR);
    CHKPR(count, OHOS::Sensors::ERROR);
    std::lock_guard<std::mutex> listLock(sensorInfoMutex_);
    if (sensorInfos_ == nullptr) {
        int32_t ret = ConvertSensorInfos();
        if (ret != SUCCESS) {
            SEN_HILOGE("convert sensor lists failed");
            ClearSensorInfos();
            return ERROR;
        }
    }
    *sensorInfo = sensorInfos_;
    *count = sensorInfoCount_;
    return SUCCESS;
}

int32_t SensorAgentProxy::SuspendSensors(int32_t pid) const
{
    CALL_LOG_ENTER;
    if (pid < 0) {
        SEN_HILOGE("Pid is invalid, %{public}d", pid);
        return PARAMETER_ERROR;
    }
    int32_t ret = SenClient.SuspendSensors(pid);
    if (ret != 0) {
        SEN_HILOGE("Suspend pid sensors failed, ret:%{public}d", ret);
    }
    return ret;
}

int32_t SensorAgentProxy::ResumeSensors(int32_t pid) const
{
    CALL_LOG_ENTER;
    if (pid < 0) {
        SEN_HILOGE("Pid is invalid, %{public}d", pid);
        return PARAMETER_ERROR;
    }
    int32_t ret = SenClient.ResumeSensors(pid);
    if (ret != 0) {
        SEN_HILOGE("Resume pid sensors failed, ret:%{public}d", ret);
    }
    return ret;
}

int32_t SensorAgentProxy::GetSubscribeInfos(int32_t pid, SubscribeSensorInfo **subscribeSensorInfos, int32_t *count) const
{
    CALL_LOG_ENTER;
    if (pid < 0) {
        SEN_HILOGE("Pid is invalid, %{public}d", pid);
        return PARAMETER_ERROR;
    }
    CHKPR(subscribeSensorInfos, OHOS::Sensors::ERROR);
    CHKPR(count, OHOS::Sensors::ERROR);
    std::lock_guard<std::mutex> subscribeSensorInfoLock(subscribeSensorInfoMutex_);
    if (subscribeSensorInfos_ != nullptr) {
        free(subscribeSensorInfos_);
        subscribeSensorInfos_ = nullptr;
    }
    std::vector<SubscribeInfo> subscribeInfoList;
    int32_t ret = SenClient.GetSubscribeInfoList(pid, subscribeInfoList);
    if (ret != 0) {
        SEN_HILOGE("Get subscribe info list failed, ret:%{public}d", ret);
        return ERROR;
    }
    if (subscribeInfoList.empty()) {
        SEN_HILOGE("Subscribe info list is empty, pid:%{public}d", pid);
        return ERROR;
    }
    size_t subscribeInfoCount = subscribeInfoList.size();
    if (subscribeInfoCount > MAX_SENSOR_LIST_SIZE) {
        SEN_HILOGE("The number of subscribe info exceeds the maximum value, count:%{public}d", subscribeInfoCount);
        return ERROR;
    }
    subscribeSensorInfos_ = (SubscribeSensorInfo *)malloc(sizeof(SubscribeSensorInfo) * subscribeInfoCount);
    CHKPR(subscribeSensorInfos_, ERROR);
    for (size_t i = 0; i < subscribeInfoCount; ++i) {
        SubscribeSensorInfo *curSubscribeInfo= subscribeSensorInfos_ + i;
        curSubscribeInfo->pid = subscribeInfoList[i].GetPid();
        curSubscribeInfo->sensorId = subscribeInfoList[i].GetSensorId();
        curSubscribeInfo->isActive = subscribeInfoList[i].IsActive();
        curSubscribeInfo->samplingPeriodNs = subscribeInfoList[i].GetSamplingPeriodNs();
        curSubscribeInfo->maxReportDelayNs = subscribeInfoList[i].GetMaxReportDelayNs();
    }
    *subscribeSensorInfos = subscribeSensorInfos_;
    *count = static_cast<int32_t>(subscribeInfoCount);
    return SUCCESS;
}

int32_t SensorAgentProxy::RegisterClientInfoCallback(ClientInfoCallback callback) const
{
    CHKPR(callback, OHOS::Sensors::ERROR);
    CHKPR(dataChannel_, INVALID_POINTER);
    int32_t ret = SenClient.RegisterClientInfoCallback(callback, dataChannel_);
    if (ret != ERR_OK) {
        SEN_HILOGE("RegisterClientInfoCallback failed");
    }
    return SUCCESS;
}

int32_t SensorAgentProxy::UnregisterClientInfoCallback(ClientInfoCallback callback) const
{
    CHKPR(callback, OHOS::Sensors::ERROR);
    int32_t ret = SenClient.UnregisterClientInfoCallback(callback);
    if (ret != ERR_OK) {
        SEN_HILOGE("UnregisterClientInfoCallback failed");
    }
    return SUCCESS;
}
}  // namespace Sensors
}  // namespace OHOS