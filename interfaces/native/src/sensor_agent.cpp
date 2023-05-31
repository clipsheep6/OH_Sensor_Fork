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

#include "sensor_agent.h"

#include "sensor_agent_proxy.h"
#include "sensors_errors.h"

using OHOS::HiviewDFX::HiLog;
using OHOS::HiviewDFX::HiLogLabel;
using OHOS::Sensors::SensorAgentProxy;
using OHOS::Sensors::SERVICE_EXCEPTION;
using OHOS::Sensors::PARAMETER_ERROR;
using OHOS::Sensors::PERMISSION_DENIED;

static const HiLogLabel LABEL = {LOG_CORE, OHOS::Sensors::SENSOR_LOG_DOMAIN, "SensorNativeAPI"};

static int32_t NormalizeErrCode(int32_t code)
{
    switch (code) {
        case PERMISSION_DENIED: {
            return PERMISSION_DENIED;
        }
        case PARAMETER_ERROR: {
            return PARAMETER_ERROR;
        }
        default: {
            return SERVICE_EXCEPTION;
        }
    }
}

int32_t GetAllSensors(SensorInfo **sensorInfo, int32_t *count)
{
    int32_t ret = SensorAgentImpl->GetAllSensors(sensorInfo, count);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("GetAllSensors failed");
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t ActivateSensor(int32_t sensorId, const SensorUser *user)
{
    int32_t ret = SensorAgentImpl->ActivateSensor(sensorId, user);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("ActivateSensor failed");
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t DeactivateSensor(int32_t sensorId, const SensorUser *user)
{
    int32_t ret = SensorAgentImpl->DeactivateSensor(sensorId, user);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("DeactivateSensor failed");
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t SetBatch(int32_t sensorId, const SensorUser *user, int64_t samplingInterval, int64_t reportInterval)
{
    int32_t ret = SensorAgentImpl->SetBatch(sensorId, user, samplingInterval, reportInterval);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("SetBatch failed");
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t SubscribeSensor(int32_t sensorId, const SensorUser *user)
{
    int32_t ret = SensorAgentImpl->SubscribeSensor(sensorId, user);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("SubscribeSensor failed");
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t UnsubscribeSensor(int32_t sensorId, const SensorUser *user)
{
    int32_t ret = SensorAgentImpl->UnsubscribeSensor(sensorId, user);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("UnsubscribeSensor failed");
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t SetMode(int32_t sensorId, const SensorUser *user, int32_t mode)
{
    return SensorAgentImpl->SetMode(sensorId, user, mode);
}

int32_t SuspendSensors(int32_t pid)
{
    int32_t ret = SensorAgentImpl->SuspendSensors(pid);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("Suspend sensors failed, ret:%{public}d", ret);
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t ResumeSensors(int32_t pid)
{
    int32_t ret = SensorAgentImpl->ResumeSensors(pid);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("Resume sensors failed, ret:%{public}d", ret);
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t GetActiveSensorInfos(int32_t pid, SensorActiveInfo **sensorActiveInfos, int32_t *count)
{
    CHKPR(sensorActiveInfos, OHOS::Sensors::ERROR);
    CHKPR(count, OHOS::Sensors::ERROR);
    int32_t ret = SensorAgentImpl->GetSensorActiveInfos(pid, sensorActiveInfos, count);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("Get active sensor infos failed, ret:%{public}d", ret);
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t Register(SensorActiveInfoCB callback)
{
    int32_t ret = SensorAgentImpl->Register(callback);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("Register active sensor infos callback failed, ret:%{public}d", ret);
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t Unregister(SensorActiveInfoCB callback)
{
    int32_t ret = SensorAgentImpl->Unregister(callback);
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("Unregister active sensor infos callback failed, ret:%{public}d", ret);
        return NormalizeErrCode(ret);
    }
    return ret;
}

int32_t ResetSensors()
{
    int32_t ret = SensorAgentImpl->ResetSensors();
    if (ret != OHOS::ERR_OK) {
        SEN_HILOGE("Reset sensors failed, ret:%{public}d", ret);
        return NormalizeErrCode(ret);
    }
    return ret;
}