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
#include "sensors_errors.h"

namespace OHOS {
namespace Sensors {
using namespace OHOS::HiviewDFX;
namespace {
constexpr HiLogLabel LABEL = { LOG_CORE, SENSOR_LOG_DOMAIN, "SensorHdiConnection" };
}

int32_t SensorHdiConnection::ConnectHdi()
{
    iSensorHdiConnection_ = std::make_unique<HdiConnection>();
    int32_t ret = ConnectHdiService();
    if (ret != ERR_OK) {
        SEN_HILOGE("connect hdi service failed, try to connect compatible connection");
        iSensorHdiConnection_ = std::make_unique<CompatibleConnection>();
        ret = ConnectHdiService();
    }
    if (ret != ERR_OK) {
        SEN_HILOGE("connect hdi failed");
    }
    return ERR_OK;
}

int32_t SensorHdiConnection::ConnectHdiService()
{
    int32_t ret = iSensorHdiConnection_->ConnectHdi();
    if (ret != 0) {
        SEN_HILOGE("connect hdi service failed");
        return CONNECT_SENSOR_HDF_ERR;
    }
    ret = iSensorHdiConnection_->GetSensorList(sensorList_);
    if (ret != 0) {
        SEN_HILOGE("get sensor list failed");
        return GET_SENSOR_LIST_ERR;
    }
    return ERR_OK;
}

int32_t SensorHdiConnection::GetSensorList(std::vector<Sensor>& sensorList)
{
    sensorList.assign(sensorList_.begin(), sensorList_.end());
    return ERR_OK;
}

int32_t SensorHdiConnection::EnableSensor(int32_t sensorId)
{
    StartTrace(HITRACE_TAG_SENSORS, "EnableSensor");
    int32_t ret = iSensorHdiConnection_->EnableSensor(sensorId);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != 0) {
        SEN_HILOGE("enable sensor failed, sensorId:%{public}d", sensorId);
        return ENABLE_SENSOR_ERR;
    }
    return ret;
};

int32_t SensorHdiConnection::DisableSensor(int32_t sensorId)
{
    StartTrace(HITRACE_TAG_SENSORS, "DisableSensor");
    int32_t ret = iSensorHdiConnection_->DisableSensor(sensorId);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != 0) {
        SEN_HILOGE("disable sensor failed, sensorId:%{public}d", sensorId);
        return DISABLE_SENSOR_ERR;
    }
    return ret;
}

int32_t SensorHdiConnection::SetBatch(int32_t sensorId, int64_t samplingInterval, int64_t reportInterval)
{
    StartTrace(HITRACE_TAG_SENSORS, "SetBatch");
    int32_t ret = iSensorHdiConnection_->SetBatch(sensorId, samplingInterval, reportInterval);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != 0) {
        SEN_HILOGE("set batch failed, sensorId:%{public}d", sensorId);
        return SET_SENSOR_CONFIG_ERR;
    }
    return ret;
}

int32_t SensorHdiConnection::SetMode(int32_t sensorId, int32_t mode)
{
    StartTrace(HITRACE_TAG_SENSORS, "SetMode");
    int32_t ret = iSensorHdiConnection_->SetMode(sensorId, mode);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != 0) {
        SEN_HILOGE("set mode failed, sensorId:%{public}d", sensorId);
        return SET_SENSOR_MODE_ERR;
    }
    return ret;
}

int32_t SensorHdiConnection::SetOption(int32_t sensorId, int32_t option)
{
    StartTrace(HITRACE_TAG_SENSORS, "SetOption");
    int32_t ret = iSensorHdiConnection_->SetOption(sensorId, option);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != 0) {
        SEN_HILOGE("set option failed, sensorId:%{public}d", sensorId);
        return SET_SENSOR_OPTION_ERR;
    }
    return ret;
}

int32_t SensorHdiConnection::RunCommand(int32_t sensorId, int32_t cmd, int32_t params)
{
    StartTrace(HITRACE_TAG_SENSORS, "RunCommand");
    int32_t ret = iSensorHdiConnection_->RunCommand(sensorId, cmd, params);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != 0) {
        SEN_HILOGE("run command failed, sensorId:%{public}d", sensorId);
        return RUN_COMMAND_ERR;
    }
    return ret;
}

int32_t SensorHdiConnection::RegisteDataReport(ReportDataCb cb, sptr<ReportDataCallback> reportDataCallback)
{
    StartTrace(HITRACE_TAG_SENSORS, "RegisteDataReport");
    int32_t ret = iSensorHdiConnection_->RegisteDataReport(cb, reportDataCallback);
    FinishTrace(HITRACE_TAG_SENSORS);
    if (ret != 0) {
        SEN_HILOGE("registe dataReport failed");
        return REGIST_CALLBACK_ERR;
    }
    return ret;
}

int32_t SensorHdiConnection::DestroyHdiConnection()
{
    int32_t ret = iSensorHdiConnection_->DestroyHdiConnection();
    if (ret != 0) {
        SEN_HILOGE("destroy hdi connection failed");
        return DEVICE_ERR;
    }
    return ret;
}
}  // namespace Sensors
}  // namespace OHOS
