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

#include "sensor_data_processer.h"

#include <cinttypes>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <thread>

#include "hisysevent.h"
#include "permission_util.h"
#include "securec.h"
#include "sensor_basic_data_channel.h"
#include "sensor_errors.h"
#include "system_ability_definition.h"

#undef LOG_TAG
#define LOG_TAG "SensorDataProcesser"

namespace OHOS {
namespace Sensors {
using namespace OHOS::HiviewDFX;

namespace {
const std::string SENSOR_REPORT_THREAD_NAME = "OS_SenProducer";
} // namespace

SensorDataProcesser::SensorDataProcesser(const std::unordered_map<int32_t, Sensor> &sensorMap)
{
    sensorMap_.insert(sensorMap.begin(), sensorMap.end());
    SEN_HILOGD("sensorMap_.size:%{public}d", int32_t { sensorMap_.size() });
}

SensorDataProcesser::~SensorDataProcesser()
{
    dataCountMap_.clear();
    sensorMap_.clear();
}

void SensorDataProcesser::SendNoneFifoCacheData(std::unordered_map<int32_t, SensorData> &cacheBuf,
                                                sptr<SensorBasicDataChannel> &channel, SensorData &data,
                                                uint64_t periodCount)
{
    std::vector<SensorData> sendEvents;
    std::lock_guard<std::mutex> dataCountLock(dataCountMutex_);
    sendEvents.push_back(data);
    auto dataCountIt = dataCountMap_.find(data.sensorTypeId);
    if (dataCountIt == dataCountMap_.end()) {
        std::vector<sptr<FifoCacheData>> channelFifoList;
        sptr<FifoCacheData> fifoCacheData = new (std::nothrow) FifoCacheData();
        CHKPV(fifoCacheData);
        fifoCacheData->SetChannel(channel);
        channelFifoList.push_back(fifoCacheData);
        dataCountMap_.insert(std::make_pair(data.sensorTypeId, channelFifoList));
        SendRawData(cacheBuf, channel, sendEvents);
        return;
    }
    bool channelExist = false;
    for (auto fifoIt = dataCountIt->second.begin(); fifoIt != dataCountIt->second.end();) {
        auto fifoCacheData = *fifoIt;
        CHKPC(fifoCacheData);
        auto fifoChannel = fifoCacheData->GetChannel();
        if (fifoChannel == nullptr) {
            fifoIt = dataCountIt->second.erase(fifoIt);
            continue;
        }
        ++fifoIt;
        if (fifoChannel != channel) {
            continue;
        }
        channelExist = true;
        uint64_t curCount = fifoCacheData->GetPeriodCount();
        curCount++;
        fifoCacheData->SetPeriodCount(curCount);
        if (periodCount != 0 && fifoCacheData->GetPeriodCount() % periodCount != 0UL) {
            continue;
        }
        SendRawData(cacheBuf, channel, sendEvents);
        fifoCacheData->SetPeriodCount(0);
        return;
    }
    if (!channelExist) {
        sptr<FifoCacheData> fifoCacheData = new (std::nothrow) FifoCacheData();
        CHKPV(fifoCacheData);
        fifoCacheData->SetChannel(channel);
        dataCountIt->second.push_back(fifoCacheData);
        SendRawData(cacheBuf, channel, sendEvents);
    }
}

void SensorDataProcesser::SendFifoCacheData(std::unordered_map<int32_t, SensorData> &cacheBuf,
                                            sptr<SensorBasicDataChannel> &channel, SensorData &data,
                                            uint64_t periodCount, uint64_t fifoCount)
{
    std::lock_guard<std::mutex> dataCountLock(dataCountMutex_);
    auto dataCountIt = dataCountMap_.find(data.sensorTypeId);
    // there is no channelFifoList
    if (dataCountIt == dataCountMap_.end()) {
        std::vector<sptr<FifoCacheData>> channelFifoList;
        sptr<FifoCacheData> fifoCacheData = new (std::nothrow) FifoCacheData();
        CHKPV(fifoCacheData);
        fifoCacheData->SetChannel(channel);
        channelFifoList.push_back(fifoCacheData);
        dataCountMap_.insert(std::make_pair(data.sensorTypeId, channelFifoList));
        return;
    }
    // find channel in channelFifoList
    bool channelExist = false;
    for (auto fifoIt = dataCountIt->second.begin(); fifoIt != dataCountIt->second.end();) {
        auto fifoData = *fifoIt;
        CHKPC(fifoData);
        auto fifoChannel = fifoData->GetChannel();
        if (fifoChannel == nullptr) {
            fifoIt = dataCountIt->second.erase(fifoIt);
            continue;
        }
        ++fifoIt;
        if (fifoChannel != channel) {
            continue;
        }
        channelExist = true;
        uint64_t curCount = fifoData->GetPeriodCount();
        curCount++;
        fifoData->SetPeriodCount(curCount);
        if (fifoData->GetPeriodCount() % periodCount != 0UL) {
            continue;
        }
        fifoData->SetPeriodCount(0);
        std::vector<SensorData> fifoDataList = fifoData->GetFifoCacheData();
        fifoDataList.push_back(data);
        fifoData->SetFifoCacheData(fifoDataList);
        if ((fifoData->GetFifoCacheData()).size() != fifoCount) {
            continue;
        }
        SendRawData(cacheBuf, channel, fifoData->GetFifoCacheData());
        fifoData->InitFifoCache();
        return;
    }
    // cannot find channel in channelFifoList
    if (!channelExist) {
        sptr<FifoCacheData> fifoCacheData = new (std::nothrow) FifoCacheData();
        CHKPV(fifoCacheData);
        fifoCacheData->SetChannel(channel);
        dataCountIt->second.push_back(fifoCacheData);
    }
}

void SensorDataProcesser::ReportData(sptr<SensorBasicDataChannel> &channel, SensorData &data)
{
    CHKPV(channel);
    int32_t sensorId = data.sensorTypeId;
    auto &cacheBuf = const_cast<std::unordered_map<int32_t, SensorData> &>(channel->GetDataCacheBuf());
    if (ReportNotContinuousData(cacheBuf, channel, data)) {
        return;
    }
    uint64_t periodCount = clientInfo_.ComputeBestPeriodCount(sensorId, channel);
    if (periodCount == 0UL) {
        return;
    }
    auto fifoCount = clientInfo_.ComputeBestFifoCount(sensorId, channel);
    if (fifoCount <= 1) {
        SendNoneFifoCacheData(cacheBuf, channel, data, periodCount);
        return;
    }
    SendFifoCacheData(cacheBuf, channel, data, periodCount, fifoCount);
}

bool SensorDataProcesser::ReportNotContinuousData(std::unordered_map<int32_t, SensorData> &cacheBuf,
                                                  sptr<SensorBasicDataChannel> &channel, SensorData &data)
{
    int32_t sensorId = data.sensorTypeId;
    std::lock_guard<std::mutex> sensorLock(sensorMutex_);
    auto sensor = sensorMap_.find(sensorId);
    if (sensor == sensorMap_.end()) {
        SEN_HILOGE("Data's sensorId is not supported");
        return false;
    }
    sensor->second.SetFlags(data.mode);
    if (((SENSOR_ON_CHANGE & sensor->second.GetFlags()) == SENSOR_ON_CHANGE) ||
        ((SENSOR_ONE_SHOT & sensor->second.GetFlags()) == SENSOR_ONE_SHOT)) {
        std::vector<SensorData> sendEvents;
        sendEvents.push_back(data);
        SendRawData(cacheBuf, channel, sendEvents);
        return true;
    }
    return false;
}

void SensorDataProcesser::SendRawData(std::unordered_map<int32_t, SensorData> &cacheBuf,
                                      sptr<SensorBasicDataChannel> channel, std::vector<SensorData> events)
{
    CHKPV(channel);
    if (events.empty()) {
        return;
    }
    int32_t eventSize = static_cast<int32_t>(events.size());
    auto ret = channel->SendData(events.data(), eventSize * sizeof(SensorData));
    if (ret != ERR_OK) {
        SEN_HILOGE("Send data failed, ret:%{public}d, sensorId:%{public}d, timestamp:%{public}" PRId64,
            ret, events[eventSize - 1].sensorTypeId, events[eventSize - 1].timestamp);
        int32_t sensorId = events[eventSize - 1].sensorTypeId;
        cacheBuf[sensorId] = events[eventSize - 1];
    }
}

int32_t SensorDataProcesser::CacheSensorEvent(const SensorData &data, sptr<SensorBasicDataChannel> &channel)
{
    CHKPR(channel, INVALID_POINTER);
    int32_t ret = ERR_OK;
    auto &cacheBuf = const_cast<std::unordered_map<int32_t, SensorData> &>(channel->GetDataCacheBuf());
    int32_t sensorId = data.sensorTypeId;
    auto cacheEvent = cacheBuf.find(sensorId);
    if (cacheEvent != cacheBuf.end()) {
        // Try to send the last failed value, if it still fails, replace the previous cache directly
        const SensorData &cacheData = cacheEvent->second;
        ret = channel->SendData(&cacheData, sizeof(SensorData));
        if (ret != ERR_OK) {
            SEN_HILOGE("retry send cache data failed, ret:%{public}d, sensorId:%{public}d, timestamp:%{public}" PRId64,
                ret, cacheData.sensorTypeId, cacheData.timestamp);
        }
        ret = channel->SendData(&data, sizeof(SensorData));
        if (ret != ERR_OK) {
            SEN_HILOGE("retry send data failed, ret:%{public}d, sensorId:%{public}d, timestamp:%{public}" PRId64,
                ret, data.sensorTypeId, data.timestamp);
            cacheBuf[sensorId] = data;
        } else {
            cacheBuf.erase(cacheEvent);
        }
    } else {
        ret = channel->SendData(&data, sizeof(SensorData));
        if (ret != ERR_OK) {
            SEN_HILOGE("directly retry failed, ret:%{public}d, sensorId:%{public}d, timestamp:%{public}" PRId64,
                ret, data.sensorTypeId, data.timestamp);
            cacheBuf[sensorId] = data;
        }
    }
    return ret;
}

void SensorDataProcesser::EventFilter(CircularEventBuf &eventsBuf)
{
    int32_t sensorId = eventsBuf.circularBuf[eventsBuf.readPos].sensorTypeId;
    std::vector<sptr<SensorBasicDataChannel>> channelList = clientInfo_.GetSensorChannel(sensorId);
    for (auto &channel : channelList) {
        if (channel->GetSensorStatus()) {
            SendEvents(channel, eventsBuf.circularBuf[eventsBuf.readPos]);
        }
    }
}

int32_t SensorDataProcesser::ProcessEvents(sptr<ReportDataCallback> dataCallback)
{
    CHKPR(dataCallback, INVALID_POINTER);
    std::unique_lock<std::mutex> lk(ISensorHdiConnection::dataMutex_);
    ISensorHdiConnection::dataCondition_.wait(lk);
    auto &eventsBuf = dataCallback->GetEventData();
    if (eventsBuf.eventNum <= 0) {
        SEN_HILOGE("Data cannot be empty");
        return NO_EVENT;
    }
    int32_t eventNum = eventsBuf.eventNum;
    for (int32_t i = 0; i < eventNum; i++) {
        EventFilter(eventsBuf);

        eventsBuf.readPos++;
        if (eventsBuf.readPos == CIRCULAR_BUF_LEN) {
            eventsBuf.readPos = 0;
        }
        eventsBuf.eventNum--;
    }
    return SUCCESS;
}

int32_t SensorDataProcesser::SendEvents(sptr<SensorBasicDataChannel> &channel, SensorData &data)
{
    CHKPR(channel, INVALID_POINTER);
    clientInfo_.UpdateDataQueue(data.sensorTypeId, data);
    auto &cacheBuf = channel->GetDataCacheBuf();
    if (cacheBuf.empty()) {
        ReportData(channel, data);
    } else {
        CacheSensorEvent(data, channel);
    }
    clientInfo_.StoreEvent(data);
    return SUCCESS;
}

int32_t SensorDataProcesser::DataThread(sptr<SensorDataProcesser> dataProcesser, sptr<ReportDataCallback> dataCallback)
{
    CALL_LOG_ENTER;
    prctl(PR_SET_NAME, SENSOR_REPORT_THREAD_NAME.c_str());
    do {
        if (dataProcesser == nullptr || dataCallback == nullptr) {
            SEN_HILOGE("dataProcesser or dataCallback is nullptr");
            return INVALID_POINTER;
        }
        if (dataProcesser->ProcessEvents(dataCallback) == INVALID_POINTER) {
            SEN_HILOGE("Callback cannot be null");
            return INVALID_POINTER;
        }
    } while (1);
}
} // namespace Sensors
} // namespace OHOS
