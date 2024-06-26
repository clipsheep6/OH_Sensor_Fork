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

#include "fifo_cache_data.h"
namespace OHOS {
namespace Sensors {

FifoCacheData::FifoCacheData() : periodCount_(0), channel_(nullptr)
{}

FifoCacheData::~FifoCacheData()
{
    fifoCacheData_.clear();
}

void FifoCacheData::InitFifoCache()
{
    periodCount_ = 0;
    fifoCacheData_.clear();
}

void FifoCacheData::SetPeriodCount(uint64_t periodCount)
{
    periodCount_ = periodCount;
}

uint64_t FifoCacheData::GetPeriodCount() const
{
    return periodCount_;
}

void FifoCacheData::SetFifoCacheData(const std::vector<SensorData> &fifoCacheData)
{
    fifoCacheData_ = fifoCacheData;
}

std::vector<SensorData> FifoCacheData::GetFifoCacheData() const
{
    return fifoCacheData_;
}

void FifoCacheData::SetChannel(const sptr<SensorBasicDataChannel> &channel)
{
    channel_ = channel;
}

sptr<SensorBasicDataChannel> FifoCacheData::GetChannel() const
{
    return channel_.promote();
}
} // namespace Sensors
} // namespace OHOS
