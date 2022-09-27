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
#include <map>
#include <optional>

#include "async_callback_info.h"
#include "refbase.h"
#include "sensors_errors.h"

namespace OHOS {
namespace Sensors {
using std::vector;
using std::string;
using ConvertDataFunc = bool(*)(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo,
    napi_value result[2]);

bool IsSameValue(const napi_env &env, const napi_value &lhs, const napi_value &rhs);
bool IsMatchType(const napi_env &env, const napi_value &value, const napi_valuetype &type);
bool IsMatchArrayType(const napi_env &env, const napi_value &value);
bool GetCppInt32(const napi_env &env, const napi_value &value, int32_t &number);
bool GetCppDouble(const napi_env &env, const napi_value &value, double &number);
bool GetCppBool(const napi_env &env, const napi_value &value);
bool GetFloatArray(const napi_env &env, const napi_value &value, vector<float> &array);
bool GetCppInt64(const napi_env &env, const napi_value &value, int64_t &number);
bool RegisterNapiCallback(const napi_env &env, const napi_value &value, napi_ref &callback);
napi_value GetNamedProperty(const napi_env &env, const napi_value &object, string name);
bool GetCppFloat(const napi_env &env, const napi_value &value, float &number);
napi_value GetNapiInt32(const napi_env &env, int32_t number);
bool GetStringValue(const napi_env &env, const napi_value &value, string &result);
void EmitAsyncCallbackWork(sptr<AsyncCallbackInfo> asyncCallbackInfo);
void EmitUvEventLoop(sptr<AsyncCallbackInfo> asyncCallbackInfo);
void EmitPromiseWork(sptr<AsyncCallbackInfo> asyncCallbackInfo);
napi_value GreateBusinessError(const napi_env &env, int32_t errCode, string errMessage,
    string errName, string errStack);
bool ConvertToFailData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToGeomagneticData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToNumber(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToArray(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToRotationMatrix(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToSensorData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool CreateNapiArray(const napi_env &env, float *data, int32_t dataLength, napi_value &result);
bool ConvertToSensorInfos(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToSingleSensor(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToBodyData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool CreateFailMessage(CallbackDataType type, int32_t code, string message,
    sptr<AsyncCallbackInfo> &asyncCallbackInfo);
bool ConvertToBodyData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);
bool ConvertToCompass(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2]);

struct NapiError {
    std::string errorCode;
    std::string message;
};

const std::map<int32_t, NapiError> ERROR_MESSAGES = {
    {SERVICE_EXCEPTION,  {"14500101", "Service exception."}},
    {PERMISSION_DENIED,  {"201", "Permission denied."}},
    {PARAMETER_ERROR,  {"401", "The parameter invalid."}},
};

inline const std::optional<NapiError> GetNapiError(int32_t errorCode) {
    auto iter = ERROR_MESSAGES.find(errorCode);
    if (iter != ERROR_MESSAGES.end()) {
        return iter->second;
    }
    return std::nullopt;
}

#define THROWERR(env, code, message) \
    do { \
        MISC_HILOGE("message: %{public}s, code: %{public}s", #message, (#code)); \
        auto error = GetNapiError(code); \
        if (error.has_value()) { \
            auto napiError = error.value(); \
            napi_throw_error(env, napiError.codes.c_str(), napiError.message.c_str()); \
        }\
    } while (0)

#define CHKNCR(env, cond, message, retVal) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("(%{public}s)", #message); \
            return retVal; \
        } \
    } while (0)

#define CHKNCP(env, cond, message) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("(%{public}s)", #message); \
            return nullptr; \
        } \
    } while (0)

#define CHKNCF(env, cond, message) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("(%{public}s)", #message); \
            return false; \
        } \
    } while (0)

#define CHKNCV(env, cond, message) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("(%{public}s)", #message); \
            return; \
        } \
    } while (0)

#define CHKNCC(env, cond, message) \
    { \
        if (!(cond)) { \
            SEN_HILOGW("(%{public}s)", #message); \
            continue; \
        } \
    }

#define CHKNRR(env, state, message, retVal) \
    do { \
        if ((state) != napi_ok) { \
            SEN_HILOGE("(%{public}s) fail", #message); \
            return retVal; \
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

#define CHKNRV(env, state, message) \
    do { \
        if ((state) != napi_ok) { \
            SEN_HILOGE("(%{public}s) fail", #message); \
            return; \
        } \
    } while (0)

#define CHKNRC(env, state, message) \
    { \
        if ((state) != napi_ok) { \
            SEN_HILOGW("(%{public}s) fail", #message); \
            continue; \
        } \
    }
}  // namespace Sensors
}  // namespace OHOS
#endif // SENSOR_NAPI_UTILS_H
