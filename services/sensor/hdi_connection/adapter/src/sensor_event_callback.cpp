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

#include "sensor_event_callback.h"
#include "hdi_connection.h"
#include "sensors_errors.h"
#include "sensors_log_domain.h"

namespace OHOS {
namespace Sensors {
using namespace OHOS::HiviewDFX;
namespace {
constexpr HiLogLabel LABEL = { LOG_CORE, SensorsLogDomain::SENSOR_SERVICE, "HdiConnection" };
std::unique_ptr<HdiConnection> HdiConnection_ = std::make_unique<HdiConnection>();
}
int32_t SensorEventCallback::OnDataEvent(const HdfSensorEvents& event)
{
    ZReportDataCb reportDataCb_ = HdiConnection_->getReportDataCb();
    sptr<ReportDataCallback> reportDataCallback_ = HdiConnection_->getReportDataCallback();
    if (reportDataCb_ == nullptr || reportDataCallback_ == nullptr) {
        HiLog::Error(LABEL, "%{public}s reportDataCb_ or reportDataCallback_ cannot be null", __func__);
        return ERR_NO_INIT;
    }
    int32_t dataSize = static_cast<int32_t>(event.data.size());
    if (dataSize == 0) {
        HiLog::Error(LABEL, "%{public}s data is empty", __func__);
        return ERR_INVALID_VALUE;
    }
    struct SensorEvent sensorEvent = {
        .sensorTypeId = event.sensorId,
        .version = event.version,
        .timestamp = event.timestamp,
        .option = event.option,
        .mode = event.mode,
        .dataLen = event.dataLen
    };
    sensorEvent.data = new uint8_t[SENSOR_DATA_LENGHT];
    for (int32_t i = 0; i < static_cast<int32_t>(dataSize); i++) {
        sensorEvent.data[i] = event.data[i];
    }
    
    if (sensorEvent.sensorTypeId == SENSOR_TYPE_ID_PROXIMITY) {
        ProximityData* data = (ProximityData*)sensorEvent.data;
        HiLog::Debug(LABEL, "%{public}s ProximityData = %{public}f",__func__ ,data->scalar);
    }
    
    (void)(reportDataCallback_->*(reportDataCb_))(&sensorEvent, reportDataCallback_);
    ISensorHdiConnection::dataCondition_.notify_one();
    return ERR_OK;
}
}  // namespace Sensors
}  // namespace OHOS