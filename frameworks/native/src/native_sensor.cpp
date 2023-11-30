/*
 * Copyright (c) 2023 Huawei Device Co., Ltd.
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

#include "native_sensor.h"

#include "i_sensor_service.h"
#include "native_sensor_impl.h"
#include "securec.h"
#include "sensor_agent.h"
#include "sensor_errors.h"

using OHOS::HiviewDFX::HiLog;
using OHOS::HiviewDFX::HiLogLabel;
namespace {
const HiLogLabel LABEL = {LOG_CORE, OHOS::Sensors::SENSOR_LOG_DOMAIN, "SensorCapiAPI"};
const uint32_t FLOAT_SIZE = 4;
}

Sensor_Result OH_Sensor_GetAllSensors(Sensor_Sensor **sensors, uint32_t *count)
{
    if (count == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    SensorInfo *sensorInfo = nullptr;
    int32_t sensorCount = -1;
    int32_t ret = GetAllSensors(&sensorInfo, &sensorCount);
    if (ret != SENSOR_SUCCESS || sensorCount < 0) {
        SEN_HILOGE("GetAllSensors fail");
        return SENSOR_SERVICE_EXCEPTION;
    }
    if (sensors == nullptr) {
        *count = static_cast<uint32_t>(sensorCount);
        SEN_HILOGD("Sensor count: %{public}d", *count);
        return SENSOR_SUCCESS;
    }
    if (static_cast<uint32_t>(sensorCount) != *count) {
        SEN_HILOGE("Count:%{public}d is invalid, should be:%{public}d", *count, sensorCount);
        return SENSOR_PARAMETER_ERROR;
    }
    for (int32_t i = 0; i < sensorCount; ++i) {
        if (sensors[i] == nullptr) {
            SEN_HILOGE("Sensor is null, i:%{public}d", i);
            return SENSOR_PARAMETER_ERROR;
        }
        errno_t result = strcpy_s(sensors[i]->sensorName, NAME_MAX_LEN, sensorInfo[i].sensorName);
        CHKCR(result == EOK, SENSOR_SERVICE_EXCEPTION);
        result = strcpy_s(sensors[i]->vendorName, NAME_MAX_LEN, sensorInfo[i].vendorName);
        CHKCR(result == EOK, SENSOR_SERVICE_EXCEPTION);
        result = strcpy_s(sensors[i]->firmwareVersion, VERSION_MAX_LEN, sensorInfo[i].firmwareVersion);
        CHKCR(result == EOK, SENSOR_SERVICE_EXCEPTION);
        result = strcpy_s(sensors[i]->hardwareVersion, VERSION_MAX_LEN, sensorInfo[i].hardwareVersion);
        CHKCR(result == EOK, SENSOR_SERVICE_EXCEPTION);
        sensors[i]->sensorTypeId = sensorInfo[i].sensorTypeId;
        sensors[i]->sensorId = sensorInfo[i].sensorId;
        sensors[i]->maxRange = sensorInfo[i].maxRange;
        sensors[i]->precision = sensorInfo[i].precision;
        sensors[i]->power = sensorInfo[i].power;
        sensors[i]->minSamplePeriod = sensorInfo[i].minSamplePeriod;
        sensors[i]->maxSamplePeriod = sensorInfo[i].maxSamplePeriod;
    }
    return SENSOR_SUCCESS;
}

Sensor_Sensor **OH_Sensor_CreateSensors(uint32_t count)
{
    auto sensors = new Sensor_Sensor *[count];
    for (uint32_t i = 0; i < count; ++i) {
        sensors[i] = new Sensor_Sensor();
    }
    return sensors;
}

int32_t OH_Sensor_DestroySensors(Sensor_Sensor **sensors, uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i) {
        delete sensors[i];
    }
    delete[] sensors;
    sensors = nullptr;
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetSensorName(Sensor_Sensor* sensor, char *sensorName, uint32_t *length)
{
    if (sensor == nullptr || sensorName == nullptr || length == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    uint32_t nameLen = strlen(sensor->sensorName);
    if (nameLen == 0 || *length <= nameLen) {
        SEN_HILOGE("Parameter error, length:%{public}d is small, should big than:%{public}d", *length, nameLen);
        return SENSOR_PARAMETER_ERROR;
    }
    errno_t result = strcpy_s(sensorName, *length, sensor->sensorName);
    if (result != EOK) {
        SEN_HILOGE("strcpy_s failed, result is %{public}d", result);
        return SENSOR_SERVICE_EXCEPTION;
    }
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetVendorName(Sensor_Sensor* sensor, char *vendorName, uint32_t *length)
{
    if (sensor == nullptr || vendorName == nullptr || length == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    uint32_t nameLen = strlen(sensor->vendorName);
    if (nameLen == 0 || *length <= nameLen) {
        SEN_HILOGE("Parameter error, length:%{public}d is small, should big than:%{public}d", *length, nameLen);
        return SENSOR_PARAMETER_ERROR;
    }
    errno_t result = strcpy_s(vendorName, *length, sensor->vendorName);
    if (result != EOK) {
        SEN_HILOGE("strcpy_s failed, result is %{public}d", result);
        return SENSOR_SERVICE_EXCEPTION;
    }
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetSensorType(Sensor_Sensor* sensor, Sensor_SensorType *sensorType)
{
    if (sensor == nullptr || sensorType == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *sensorType = static_cast<Sensor_SensorType>(sensor->sensorTypeId);
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetSensorResolution(Sensor_Sensor* sensor, float *resolution)
{
    if (sensor == nullptr || resolution == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *resolution = sensor->precision;
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetMinSamplingInterval(Sensor_Sensor* sensor, int64_t *minSamplePeriod)
{
    if (sensor == nullptr || minSamplePeriod == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *minSamplePeriod = sensor->minSamplePeriod;
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetMaxSamplingInterval(Sensor_Sensor* sensor, int64_t *maxSamplePeriod)
{
    if (sensor == nullptr || maxSamplePeriod == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *maxSamplePeriod = sensor->maxSamplePeriod;
    return SENSOR_SUCCESS;
}

Sensor_Result OH_Sensor_SubscribeSensor(const Sensor_SensorSubscriptionId *id,
    const Sensor_SubscriptionAttribute *attribute, const Sensor_Subscriber *user)
{
    if (id == nullptr || attribute == nullptr || user == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    const SensorUser *sensorUser = reinterpret_cast<const SensorUser *>(user);
    int32_t sensorType = id->sensorType;
    int32_t ret = SubscribeSensor(sensorType, sensorUser);
    if (ret != SENSOR_SUCCESS) {
        SEN_HILOGE("SubscribeSensor failed, %{public}d", ret);
        return SENSOR_SERVICE_EXCEPTION;
    }
    int64_t samplingInterval = attribute->samplingInterval;
    ret = SetBatch(sensorType, sensorUser, samplingInterval, samplingInterval);
    if (ret != SENSOR_SUCCESS) {
        SEN_HILOGE("SetBatch failed, %{public}d", ret);
        return SENSOR_SERVICE_EXCEPTION;
    }
    return static_cast<Sensor_Result>(ActivateSensor(sensorType, sensorUser));
}

Sensor_Result OH_Sensor_UnsubscribeSensor(const Sensor_SensorSubscriptionId *id,
    const Sensor_Subscriber *user)
{
    if (id == nullptr || user == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    const SensorUser *sensorUser = reinterpret_cast<const SensorUser *>(user);
    int32_t sensorType = id->sensorType;
    int32_t ret = DeactivateSensor(sensorType, sensorUser);
    if (ret != SENSOR_SUCCESS) {
        SEN_HILOGE("SetBatch failed, %{public}d", ret);
        return SENSOR_SERVICE_EXCEPTION;
    }
    return static_cast<Sensor_Result>(UnsubscribeSensor(sensorType, sensorUser));
}

int32_t OH_Sensor_GetEventSensorType(Sensor_SensorEvent* sensorEvent, Sensor_SensorType *sensorType)
{
    if (sensorEvent == nullptr || sensorType == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *sensorType = static_cast<Sensor_SensorType>(sensorEvent->sensorTypeId);
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetEventTimestamp(Sensor_SensorEvent* sensorEvent, int64_t *timestamp)
{
    if (sensorEvent == nullptr || timestamp == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *timestamp = sensorEvent->timestamp;
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetEventAccuracy(Sensor_SensorEvent* sensorEvent, Sensor_SensorAccuracy *accuracy)
{
    if (sensorEvent == nullptr || accuracy == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *accuracy = static_cast<Sensor_SensorAccuracy>(sensorEvent->option);
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetSensorData(Sensor_SensorEvent* sensorEvent, float **data, uint32_t *length)
{
    if (sensorEvent == nullptr || data == nullptr || length == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *data = reinterpret_cast<float *>(sensorEvent->data);
    *length = sensorEvent->dataLen / FLOAT_SIZE;
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetSubscriptionSensorType(Sensor_SensorSubscriptionId* id, Sensor_SensorType *sensorType)
{
    if (id == nullptr || sensorType == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *sensorType = static_cast<Sensor_SensorType>(id->sensorType);
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_SetSubscriptionSensorType(Sensor_SensorSubscriptionId* id, const Sensor_SensorType sensorType)
{
    if (id == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    id->sensorType = sensorType;
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_SetSamplingInterval(Sensor_SubscriptionAttribute* attribute, const int64_t samplingInterval)
{
    if (attribute == nullptr || samplingInterval < 0) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    attribute->samplingInterval = samplingInterval;
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetSamplingInterval(Sensor_SubscriptionAttribute* attribute, int64_t *samplingInterval)
{
    if (attribute == nullptr || samplingInterval == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *samplingInterval = attribute->samplingInterval;
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_SetSensorCallback(Sensor_Subscriber* user, const Sensor_SensorCallback callback)
{
    if (user == nullptr || callback == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    user->callback = callback;
    return SENSOR_SUCCESS;
}

int32_t OH_Sensor_GetSensorCallback(Sensor_Subscriber* user, Sensor_SensorCallback *callback)
{
    if (user == nullptr || callback == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    *callback = user->callback;
    return SENSOR_SUCCESS;
}

Sensor_SensorSubscriptionId *OH_Sensor_CreateSubscriptionId()
{
    return new (std::nothrow) Sensor_SensorSubscriptionId();
}

int32_t OH_Sensor_DestroySubscriptionId(Sensor_SensorSubscriptionId *id)
{
    if (id == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    delete id;
    id = nullptr;
    return SENSOR_SUCCESS;
}

Sensor_SubscriptionAttribute *OH_Sensor_CreateAttribute()
{
    return new (std::nothrow) Sensor_SubscriptionAttribute();
}

int32_t OH_Sensor_DestroyAttribute(Sensor_SubscriptionAttribute *attribute)
{
    if (attribute == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    delete attribute;
    attribute = nullptr;
    return SENSOR_SUCCESS;
}

Sensor_Subscriber *OH_Sensor_CreateSubscriber()
{
    return new (std::nothrow) Sensor_Subscriber();
}

int32_t OH_Sensor_DestroySubscriber(Sensor_Subscriber *user)
{
    if (user == nullptr) {
        SEN_HILOGE("Parameter error");
        return SENSOR_PARAMETER_ERROR;
    }
    delete user;
    user = nullptr;
    return SENSOR_SUCCESS;
}