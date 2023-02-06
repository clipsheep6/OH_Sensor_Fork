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

#ifndef I_SENSOR_STATUS_CALLBACK_H
#define I_SENSOR_STATUS_CALLBACK_H

#include "iremote_broker.h"

#include "sensor_agent_type.h"

namespace OHOS {
namespace Sensors {
class ISensorStatusCallback : public IRemoteBroker {
public:
    DECLARE_INTERFACE_DESCRIPTOR(u"ohos.sensors.ISensorStatusCallback");
    virtual void OnSensorChanged(const AppSensorInfo &appSensorInfo) = 0;
    enum : uint32_t {
        SENSOR_CHANGE = 0,
    };
};
}  // namespace Sensors
}  // namespace OHOS
#endif  // I_SENSOR_STATUS_CALLBACK_H
