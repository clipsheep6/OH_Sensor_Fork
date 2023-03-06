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

#ifndef SENSOR_SERVICE_CLIENT_H
#define SENSOR_SERVICE_CLIENT_H

#include <map>
#include <set>
#include <vector>

#include "iservice_registry.h"
#include "singleton.h"

#include "sensor_agent_type.h"
#include "sensor_basic_data_channel.h"
#include "sensor_basic_info.h"
#include "sensor_client_stub.h"
#include "sensor_data_channel.h"
#include "sensor.h"
#include "sensor_service_proxy.h"
#include "stream_socket.h"
#include "subscribe_info.h"

namespace OHOS {
namespace Sensors {
class SensorServiceClient : public StreamSocket, public Singleton<SensorServiceClient> {
public:
    ~SensorServiceClient();
    std::vector<Sensor> GetSensorList();
    int32_t EnableSensor(int32_t sensorId, int64_t samplingPeriod, int64_t maxReportDelay);
    int32_t DisableSensor(int32_t sensorId);
    int32_t TransferDataChannel(sptr<SensorDataChannel> sensorDataChannel);
    int32_t DestroyDataChannel();
    void ProcessDeathObserver(const wptr<IRemoteObject> &object);
    bool IsValid(int32_t sensorId);
    int32_t SuspendSensors(int32_t pid);
    int32_t ResumeSensors(int32_t pid);
    int32_t GetSubscribeInfoList(int32_t pid, std::vector<SubscribeInfo> &subscribeInfoList);

    int32_t RegisterClientInfoCallback(ClientInfoCallback callback, sptr<SensorDataChannel> sensorDataChannel);
    int32_t UnregisterClientInfoCallback(ClientInfoCallback callback);

    void ReceiveMessage(const char *buf, size_t size);
    void Disconnect();

private:
    int32_t InitServiceClient();
    void UpdateSensorInfoMap(int32_t sensorId, int64_t samplingPeriod, int64_t maxReportDelay);
    void DeleteSensorInfoItem(int32_t sensorId);
    std::mutex clientMutex_;
    sptr<IRemoteObject::DeathRecipient> serviceDeathObserver_;
    sptr<ISensorService> sensorServer_;
    std::vector<Sensor> sensorList_;
    sptr<SensorDataChannel> dataChannel_;
    sptr<SensorClientStub> sensorClientStub_;
    std::mutex mapMutex_;
    std::map<int32_t, SensorBasicInfo> sensorInfoMap_;

    int32_t CreateSocketChannel();
    void HandleNetPacke(NetPacket &pkt);
    std::atomic_bool isConnected_ = false;
    CircleStreamBuffer circBuf_;

    std::mutex clientInfoCallbackMutex_;
    std::set<ClientInfoCallback> clientInfoCallbackSet_;
};
}  // namespace Sensors
}  // namespace OHOS
#endif  // SENSOR_SERVICE_CLIENT_H
