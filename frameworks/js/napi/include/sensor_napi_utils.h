/*
 * Copyright (c) 2022 Huawei Device Co., Ltd.
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
#ifndef SENSOR_NAPI_UTILS_H
#define SENSOR_NAPI_UTILS_H

#include <iostream>

#include "refbase.h"

#include "async_callback_info.h"

namespace OHOS {
namespace Sensors {
using std::vector;
using std::string;
using ConvertDataFunc = bool(*)(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo,
    napi_value result[2]);

bool IsSameValue(const napi_env &env, const napi_value &lhs, const napi_value &rhs);
bool IsMatchType(const napi_env &env, const napi_value &value, const napi_valuetype &type);
bool IsMatchArrayType(const napi_env &env, const napi_value &value);
bool GetNativeInt32(const napi_env &env, const napi_value &value, int32_t &number);
bool GetNativeDouble(const napi_env &env, const napi_value &value, double &number);
bool GetFloatArray(const napi_env &env, const napi_value &value, vector<float> &array);
bool GetNativeInt64(const napi_env &env, const napi_value &value, int64_t &number);
bool RegisterNapiCallback(const napi_env &env, const napi_value &value, napi_ref &callback);
napi_value GetNamedProperty(const napi_env &env, const napi_value &object, string name);
bool GetNativeFloat(const napi_env &env, const napi_value &value, float &number);
napi_value GetNapiInt32(const napi_env &env, int32_t number);
bool GetStringValue(const napi_env &env, const napi_value &value, string &result);
void EmitAsyncCallbackWork(sptr<AsyncCallbackInfo> asyncCallbackInfo);
void EmitUvEventLoop(sptr<AsyncCallbackInfo> asyncCallbackInfo);
void EmitPromiseWork(sptr<AsyncCallbackInfo> asyncCallbackInfo);
bool ConvertToFailData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToGeomagneticData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToNumber(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToArray(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToRotationMatrix(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToSensorData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool CreateNapiArray(const napi_env &env, float *data, int32_t dataLength, napi_value &result);
bool ConvertToSensorInfos(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToSingleSensor(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToSensorInfo(const napi_env &env, const SensorInfo &sensorInfo, napi_value &result);
bool ConvertToBodyData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool CreateFailMessage(CallbackDataType type, int32_t code, string message,
    sptr<AsyncCallbackInfo> &asyncCallbackInfo);
bool ConvertToBodyData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToCompass(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
void ReleaseCallback(sptr<AsyncCallbackInfo> asyncCallbackInfo);


#define CHKNCF(env, cond, message) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("(%{public}s)", #message); \
            return false; \
        } \
    } while (0)

#define CHKNRP(env, state, message) \
    do { \
        if ((state) != napi_ok) { \
            SEN_HILOGE("(%{public}s) fail", #message); \
            return nullptr; \
        } \
    } while (0)

#define CHKNRF(env, state, message) \
    do { \
        if ((state) != napi_ok) { \
            SEN_HILOGE("(%{public}s) fail", #message); \
            return false; \
        } \
    } while (0)

#define CHKNRF(env, state, message) \
    do { \
        if ((state) != napi_ok) { \
            SEN_HILOGE("(%{public}s) fail", #message); \
            return false; \
        } \
    } while (0)
}  // namespace Sensors
}  // namespace OHOS
#endif // SENSOR_NAPI_UTILS_H
