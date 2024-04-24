/*
 * Copyright (c) 2021-2024 Huawei Device Co., Ltd.
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

#include "sensor_napi_utils.h"

#include <map>
#include <string>
#include <vector>

#include "sensor_napi_error.h"

namespace OHOS {
namespace Sensors {
namespace {
    constexpr int32_t STRING_LENGTH_MAX = 64;
} // namespace
bool IsSameValue(const napi_env &env, const napi_value &lhs, const napi_value &rhs)
{
    CALL_LOG_ENTER;
    bool result = false;
    CHKNRF(env, napi_strict_equals(env, lhs, rhs, &result), "napi_strict_equals");
    return result;
}

bool IsMatchType(const napi_env &env, const napi_value &value, const napi_valuetype &type)
{
    CALL_LOG_ENTER;
    napi_valuetype paramType = napi_undefined;
    CHKNRF(env, napi_typeof(env, value, &paramType), "napi_typeof");
    return paramType == type;
}

bool IsMatchArrayType(const napi_env &env, const napi_value &value)
{
    CALL_LOG_ENTER;
    bool result = false;
    CHKNRF(env, napi_is_array(env, value, &result), "napi_is_array");
    return result;
}

bool GetFloatArray(const napi_env &env, const napi_value &value, vector<float> &array)
{
    CALL_LOG_ENTER;
    uint32_t arrayLength = 0;
    CHKNRF(env, napi_get_array_length(env, value, &arrayLength), "napi_get_array_length");
    for (size_t i = 0; i < arrayLength; ++i) {
        napi_value element = nullptr;
        CHKNRF(env, napi_get_element(env, value, i, &element), "napi_get_element");
        CHKNCF(env, IsMatchType(env, element, napi_number), "Wrong argument type. Number or function expected");
        double number = 0;
        CHKNCF(env, GetNativeDouble(env, element, number), "Wrong argument type. get double fail");
        array.push_back(static_cast<float>(number));
    }
    return true;
}

napi_value GetNamedProperty(const napi_env &env, const napi_value &object, string name)
{
    CALL_LOG_ENTER;
    bool status = false;
    CHKNRP(env, napi_has_named_property(env, object, name.c_str(), &status), "napi_has_named_property");
    if (!status) {
        SEN_HILOGW("%{public}s not exists on the object", name.c_str());
        return nullptr;
    }
    napi_value value = nullptr;
    CHKNRP(env, napi_get_named_property(env, object, name.c_str(), &value),
        "napi_get_named_property");
    return value;
}

bool GetNativeDouble(const napi_env &env, const napi_value &value, double &number)
{
    CALL_LOG_ENTER;
    CHKNRF(env, napi_get_value_double(env, value, &number), "napi_get_value_double");
    return true;
}

bool GetNativeFloat(const napi_env &env, const napi_value &value, float &number)
{
    CALL_LOG_ENTER;
    double result = 0;
    CHKNCF(env, GetNativeDouble(env, value, result), "Get cpp double fail");
    number = static_cast<float>(result);
    return true;
}

bool GetNativeInt32(const napi_env &env, const napi_value &value, int32_t &number)
{
    CALL_LOG_ENTER;
    CHKNRF(env, napi_get_value_int32(env, value, &number), "napi_get_value_int32");
    return true;
}

bool GetNativeInt64(const napi_env &env, const napi_value &value, int64_t &number)
{
    CALL_LOG_ENTER;
    CHKNRF(env, napi_get_value_int64(env, value, &number), "napi_get_value_int64");
    return true;
}

bool GetNativeBool(const napi_env &env, const napi_value &value)
{
    CALL_LOG_ENTER;
    bool number = false;
    CHKNRF(env, napi_get_value_bool(env, value, &number), "napi_get_value_bool");
    return number;
}

napi_value GetNapiInt32(const napi_env &env, int32_t number)
{
    napi_value value = nullptr;
    CHKNRP(env, napi_create_int32(env, number, &value), "napi_create_int32");
    return value;
}

bool GetStringValue(const napi_env &env, const napi_value &value, string &result)
{
    CALL_LOG_ENTER;
    CHKNCF(env, IsMatchType(env, value, napi_string), "Wrong argument type. String or function expected");
    char buf[STRING_LENGTH_MAX] = { 0 };
    size_t copyLength = 0;
    CHKNRF(env, napi_get_value_string_utf8(env, value, buf, STRING_LENGTH_MAX, &copyLength),
        "napi_get_value_string_utf8");
    result = std::string(buf);
    return true;
}

bool RegisterNapiCallback(const napi_env &env, const napi_value &value,
    napi_ref &callback)
{
    CHKNCF(env, IsMatchType(env, value, napi_function), "Wrong argument type, should be function");
    CHKNRF(env, napi_create_reference(env, value, 1, &callback), "napi_create_reference");
    return true;
}

bool CreateFailMessage(CallbackDataType type, int32_t code, string message,
    sptr<AsyncCallbackInfo> &asyncCallbackInfo)
{
    CHKPF(asyncCallbackInfo);
    asyncCallbackInfo->type = type;
    asyncCallbackInfo->error.code = code;
    asyncCallbackInfo->error.message = message;
    return true;
}

std::map<int32_t, vector<string>> g_sensorAttributeList = {
    { 0, { "x" } },
    { SENSOR_TYPE_ID_ACCELEROMETER, { "x", "y", "z" } },
    { SENSOR_TYPE_ID_GYROSCOPE, { "x", "y", "z" } },
    { SENSOR_TYPE_ID_AMBIENT_LIGHT, { "intensity" } },
    { SENSOR_TYPE_ID_MAGNETIC_FIELD, { "x", "y", "z" } },
    { SENSOR_TYPE_ID_BAROMETER, { "pressure" } },
    { SENSOR_TYPE_ID_HALL, { "status" } },
    { SENSOR_TYPE_ID_TEMPERATURE, { "temperature" } },
    { SENSOR_TYPE_ID_PROXIMITY, { "distance" } },
    { SENSOR_TYPE_ID_HUMIDITY, { "humidity" } },
    { SENSOR_TYPE_ID_ORIENTATION, { "alpha", "beta", "gamma" } },
    { SENSOR_TYPE_ID_GRAVITY, { "x", "y", "z" } },
    { SENSOR_TYPE_ID_LINEAR_ACCELERATION, { "x", "y", "z" } },
    { SENSOR_TYPE_ID_ROTATION_VECTOR, { "x", "y", "z", "w" } },
    { SENSOR_TYPE_ID_AMBIENT_TEMPERATURE, { "temperature" } },
    { SENSOR_TYPE_ID_MAGNETIC_FIELD_UNCALIBRATED, { "x", "y", "z", "biasX", "biasY", "biasZ" } },
    { SENSOR_TYPE_ID_GYROSCOPE_UNCALIBRATED, { "x", "y", "z", "biasX", "biasY", "biasZ" } },
    { SENSOR_TYPE_ID_SIGNIFICANT_MOTION, { "scalar" } },
    { SENSOR_TYPE_ID_PEDOMETER_DETECTION, { "scalar" } },
    { SENSOR_TYPE_ID_PEDOMETER, { "steps" } },
    { SENSOR_TYPE_ID_HEART_RATE, { "heartRate" } },
    { SENSOR_TYPE_ID_WEAR_DETECTION, { "value" } },
    { SENSOR_TYPE_ID_ACCELEROMETER_UNCALIBRATED, { "x", "y", "z", "biasX", "biasY", "biasZ" } },
    { SENSOR_TYPE_ID_COLOR, { "lightIntensity", "colorTemperature" } },
    { SENSOR_TYPE_ID_SAR, { "absorptionRatio" } }
};

std::map<int32_t, ConvertDataFunc> g_convertfuncList = {
    {FAIL, ConvertToFailData},
    {ON_CALLBACK, ConvertToSensorData},
    {ONCE_CALLBACK, ConvertToSensorData},
    {GET_GEOMAGNETIC_FIELD, ConvertToGeomagneticData},
    {GET_ALTITUDE, ConvertToNumber},
    {GET_GEOMAGNETIC_DIP, ConvertToNumber},
    {GET_ANGLE_MODIFY, ConvertToArray},
    {CREATE_ROTATION_MATRIX, ConvertToArray},
    {TRANSFORM_COORDINATE_SYSTEM, ConvertToArray},
    {CREATE_QUATERNION, ConvertToArray},
    {GET_DIRECTION, ConvertToArray},
    {ROTATION_INCLINATION_MATRIX, ConvertToRotationMatrix},
    {GET_SENSOR_LIST, ConvertToSensorInfos},
    {GET_SINGLE_SENSOR, ConvertToSingleSensor},
    {GET_BODY_STATE, ConvertToBodyData},
    {SUBSCRIBE_CALLBACK, ConvertToSensorData},
    {SUBSCRIBE_COMPASS, ConvertToCompass},
};

bool getJsonObject(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value &result)
{
    CHKPF(asyncCallbackInfo);
    CHKNRF(env, napi_create_object(env, &result), "napi_create_object");
    napi_value value = nullptr;
    CHKNRF(env, napi_create_double(env, asyncCallbackInfo->data.geomagneticData.x, &value), "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "x", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_double(env, asyncCallbackInfo->data.geomagneticData.y, &value), "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "y", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_double(env, asyncCallbackInfo->data.geomagneticData.z, &value), "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "z", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_double(env, asyncCallbackInfo->data.geomagneticData.geomagneticDip, &value),
        "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "geomagneticDip", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_double(env, asyncCallbackInfo->data.geomagneticData.deflectionAngle, &value),
        "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "deflectionAngle", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_double(env, asyncCallbackInfo->data.geomagneticData.levelIntensity, &value),
        "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "levelIntensity", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_double(env, asyncCallbackInfo->data.geomagneticData.totalIntensity, &value),
        "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "totalIntensity", value), "napi_set_named_property");
    return true;
}

bool ConvertToSensorInfo(const napi_env &env, const SensorInfo &sensorInfo, napi_value &result)
{
    CALL_LOG_ENTER;
    CHKNRF(env, napi_create_object(env, &result), "napi_create_object");
    napi_value value = nullptr;
    CHKNRF(env, napi_create_string_latin1(env, sensorInfo.sensorName, NAPI_AUTO_LENGTH, &value),
        "napi_create_string_latin1");
    CHKNRF(env, napi_set_named_property(env, result, "sensorName", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_string_latin1(env, sensorInfo.vendorName, NAPI_AUTO_LENGTH, &value),
        "napi_create_string_latin1");
    CHKNRF(env, napi_set_named_property(env, result, "vendorName", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_string_latin1(env, sensorInfo.firmwareVersion, NAPI_AUTO_LENGTH, &value),
        "napi_create_string_latin1");
    CHKNRF(env, napi_set_named_property(env, result, "firmwareVersion", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_string_latin1(env, sensorInfo.hardwareVersion, NAPI_AUTO_LENGTH, &value),
        "napi_create_string_latin1");
    CHKNRF(env, napi_set_named_property(env, result, "hardwareVersion", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_double(env, sensorInfo.sensorId, &value), "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "sensorId", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_double(env, sensorInfo.maxRange, &value), "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "maxRange", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_double(env, sensorInfo.precision, &value), "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "precision", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_double(env, sensorInfo.power, &value), "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result, "power", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_int64(env, sensorInfo.minSamplePeriod, &value), "napi_create_int64");
    CHKNRF(env, napi_set_named_property(env, result, "minSamplePeriod", value), "napi_set_named_property");
    value = nullptr;
    CHKNRF(env, napi_create_int64(env, sensorInfo.maxSamplePeriod, &value), "napi_create_int64");
    CHKNRF(env, napi_set_named_property(env, result, "maxSamplePeriod", value), "napi_set_named_property");
    return true;
}

bool ConvertToSingleSensor(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2])
{
    CALL_LOG_ENTER;
    CHKPF(asyncCallbackInfo);
    return ConvertToSensorInfo(env, asyncCallbackInfo->sensorInfos[0], result[1]);
}

bool ConvertToSensorInfos(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2])
{
    CALL_LOG_ENTER;
    CHKPF(asyncCallbackInfo);
    CHKNRF(env, napi_create_array(env, &result[1]), "napi_create_array");
    auto sensorInfos = asyncCallbackInfo->sensorInfos;
    for (uint32_t i = 0; i < sensorInfos.size(); ++i) {
        napi_value value = nullptr;
        CHKNCF(env, ConvertToSensorInfo(env, sensorInfos[i], value), "Convert sensor info fail");
        CHKNRF(env, napi_set_element(env, result[1], i, value), "napi_set_element");
    }
    return true;
}

bool ConvertToFailData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2])
{
    CALL_LOG_ENTER;
    CHKPF(asyncCallbackInfo);
    int32_t code = asyncCallbackInfo->error.code;
    auto msg = GetNapiError(code);
    if (!msg) {
        SEN_HILOGE("ErrCode:%{public}d is invalid", code);
        return false;
    }
    result[0] = CreateBusinessError(env, code, msg.value());
    return (result[0] != nullptr);
}

bool ConvertToSensorData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2])
{
    CHKPF(asyncCallbackInfo);
    int32_t sensorTypeId = asyncCallbackInfo->data.sensorData.sensorTypeId;
    CHKNCF(env, (g_sensorAttributeList.find(sensorTypeId) != g_sensorAttributeList.end()), "Invalid sensor type");
    if (sensorTypeId == SENSOR_TYPE_ID_WEAR_DETECTION && asyncCallbackInfo->type == SUBSCRIBE_CALLBACK) {
        return ConvertToBodyData(env, asyncCallbackInfo, result);
    }
    size_t size = g_sensorAttributeList[sensorTypeId].size();
    uint32_t dataLength = asyncCallbackInfo->data.sensorData.dataLength / sizeof(float);
    CHKNCF(env, (size <= dataLength), "Data length mismatch");

    CHKNRF(env, napi_create_object(env, &result[1]), "napi_create_object");
    napi_value message = nullptr;
    auto sensorAttributes = g_sensorAttributeList[sensorTypeId];
    for (uint32_t i = 0; i < size; ++i) {
        CHKNRF(env, napi_create_double(env, asyncCallbackInfo->data.sensorData.data[i], &message),
            "napi_create_double");
        CHKNRF(env, napi_set_named_property(env, result[1], sensorAttributes[i].c_str(), message),
            "napi_set_named_property");
        message = nullptr;
    }
    CHKNRF(env, napi_create_int64(env, asyncCallbackInfo->data.sensorData.timestamp, &message),
        "napi_create_int64");
    CHKNRF(env, napi_set_named_property(env, result[1], "timestamp", message), "napi_set_named_property");
    message = nullptr;
    CHKNRF(env, napi_create_int32(env, asyncCallbackInfo->data.sensorData.sensorAccuracy, &message),
        "napi_create_int32");
    CHKNRF(env, napi_set_named_property(env, result[1], "accuracy", message), "napi_set_named_property");
    return true;
}

bool ConvertToGeomagneticData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2])
{
    CALL_LOG_ENTER;
    return getJsonObject(env, asyncCallbackInfo, result[1]);
}

bool ConvertToBodyData(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2])
{
    CALL_LOG_ENTER;
    CHKPF(asyncCallbackInfo);
    CHKNRF(env, napi_create_object(env, &result[1]), "napi_create_object");
    napi_value status = nullptr;
    CHKNRF(env, napi_get_boolean(env, asyncCallbackInfo->data.sensorData.data[0], &status),
        "napi_get_boolean");
    CHKNRF(env, napi_set_named_property(env, result[1], "value", status), "napi_set_named_property");
    return true;
}

bool ConvertToCompass(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2])
{
    CALL_LOG_ENTER;
    CHKPF(asyncCallbackInfo);
    CHKNRF(env, napi_create_object(env, &result[1]), "napi_create_object");
    napi_value message = nullptr;
    CHKNRF(env, napi_create_double(env, asyncCallbackInfo->data.sensorData.data[0], &message),
        "napi_create_double");
    CHKNRF(env, napi_set_named_property(env, result[1], "direction", message), "napi_set_named_property");
    return true;
}

bool ConvertToNumber(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2])
{
    CALL_LOG_ENTER;
    CHKPF(asyncCallbackInfo);
    napi_status status =
        napi_create_double(env, static_cast<double>(asyncCallbackInfo->data.reserveData.reserve[0]), &result[1]);
    CHKNRF(env, status, "napi_create_double");
    return true;
}

bool ConvertToArray(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2])
{
    CALL_LOG_ENTER;
    CHKPF(asyncCallbackInfo);
    bool ret = CreateNapiArray(env, asyncCallbackInfo->data.reserveData.reserve,
        asyncCallbackInfo->data.reserveData.length, result[1]);
    CHKNCF(env, ret, "Create napi array fail");
    return true;
}

bool ConvertToRotationMatrix(const napi_env &env, sptr<AsyncCallbackInfo> asyncCallbackInfo, napi_value result[2])
{
    CALL_LOG_ENTER;
    CHKPF(asyncCallbackInfo);
    napi_value rotation = nullptr;
    bool ret = CreateNapiArray(env, asyncCallbackInfo->data.rationMatrixData.rotationMatrix,
        THREE_DIMENSIONAL_MATRIX_LENGTH, rotation);
    CHKNCF(env, ret, "Create napi array rotation fail");
    napi_value inclination = nullptr;
    ret = CreateNapiArray(env, asyncCallbackInfo->data.rationMatrixData.inclinationMatrix,
        THREE_DIMENSIONAL_MATRIX_LENGTH, inclination);
    CHKNCF(env, ret, "Create napi array inclination fail");
    CHKNRF(env, napi_create_object(env, &result[1]), "napi_create_object");
    CHKNRF(env, napi_set_named_property(env, result[1], "rotation", rotation),
        "napi_set_named_property");
    CHKNRF(env, napi_set_named_property(env, result[1], "inclination", inclination),
        "napi_set_named_property");
    return true;
}

bool CreateNapiArray(const napi_env &env, float data[], int32_t dataLength, napi_value &result)
{
    CHKNRF(env, napi_create_array(env, &result), "napi_create_array");
    for (int32_t i = 0; i < dataLength; ++i) {
        napi_value message = nullptr;
        CHKNRF(env, napi_create_double(env, data[i], &message), "napi_create_double");
        CHKNRF(env, napi_set_element(env, result, i, message), "napi_set_element");
    }
    return true;
}

void ReleaseCallback(sptr<AsyncCallbackInfo> asyncCallbackInfo)
{
    CHKPV(asyncCallbackInfo);
    if (asyncCallbackInfo->type == ONCE_CALLBACK) {
        napi_env env = asyncCallbackInfo->env;
        CHKPV(env);
        napi_ref callback = asyncCallbackInfo->callback[0];
        if (callback != nullptr) {
            napi_delete_reference(env, callback);
        }
    }
}

void EmitAsyncCallbackWork(sptr<AsyncCallbackInfo> asyncCallbackInfo)
{
    CALL_LOG_ENTER;
    CHKPV(asyncCallbackInfo);
    napi_value resourceName = nullptr;
    napi_env env = asyncCallbackInfo->env;
    napi_status ret = napi_create_string_latin1(env, "AsyncCallback", NAPI_AUTO_LENGTH, &resourceName);
    CHKCV((ret == napi_ok), "napi_create_string_latin1 fail");
    asyncCallbackInfo->IncStrongRef(nullptr);
    napi_status status = napi_create_async_work(env, nullptr, resourceName,
        [](napi_env env, void *data) {},
        [](napi_env env, napi_status status, void *data) {
            CALL_LOG_ENTER;
            sptr<AsyncCallbackInfo> asyncCallbackInfo(static_cast<AsyncCallbackInfo *>(data));
            /**
             * After the asynchronous task is created, the asyncCallbackInfo reference count is reduced
             * to 0 destruction, so you need to add 1 to the asyncCallbackInfo reference count when the
             * asynchronous task is created, and subtract 1 from the reference count after the naked
             * pointer is converted to a pointer when the asynchronous task is executed, the reference
             * count of the smart pointer is guaranteed to be 1.
             */
            asyncCallbackInfo->DecStrongRef(nullptr);
            napi_value callback = nullptr;
            napi_value callResult = nullptr;
            napi_value result[2] = {0};
            if (asyncCallbackInfo->type == SUBSCRIBE_FAIL) {
                CHKCV((napi_get_reference_value(env, asyncCallbackInfo->callback[1], &callback) == napi_ok),
                    "napi_get_reference_value fail");
                CHKCV((napi_create_string_utf8(env, asyncCallbackInfo->error.message.c_str(),
                    NAPI_AUTO_LENGTH, &result[0]) == napi_ok), "napi_create_string_utf8 fail");
                CHKCV((napi_create_int32(env, asyncCallbackInfo->error.code, &result[1]) == napi_ok),
                    "napi_create_int32 fail");
                CHKCV((napi_call_function(env, nullptr, callback, 2, result, &callResult) == napi_ok),
                    "napi_call_function fail");
                return;
            }
            CHKCV((napi_get_reference_value(env, asyncCallbackInfo->callback[0], &callback) == napi_ok),
                "napi_get_reference_value fail");
            CHKCV((g_convertfuncList.find(asyncCallbackInfo->type) != g_convertfuncList.end()),
                "Callback type invalid in async work");
            bool ret = g_convertfuncList[asyncCallbackInfo->type](env, asyncCallbackInfo, result);
            CHKCV(ret, "Create napi data fail in async work");
            CHKCV((napi_call_function(env, nullptr, callback, 2, result, &callResult) == napi_ok),
                "napi_call_function fail");
        },
        asyncCallbackInfo.GetRefPtr(), &asyncCallbackInfo->asyncWork);
    if (status != napi_ok
        || napi_queue_async_work_with_qos(
            asyncCallbackInfo->env, asyncCallbackInfo->asyncWork, napi_qos_default) != napi_ok) {
        SEN_HILOGE("Create async work fail");
        asyncCallbackInfo->DecStrongRef(nullptr);
    }
}

void DeleteWork(uv_work_t *work)
{
    CHKPV(work);
    delete work;
    work = nullptr;
}

void EmitUvEventLoop(sptr<AsyncCallbackInfo> asyncCallbackInfo)
{
    CHKPV(asyncCallbackInfo);
    uv_loop_s *loop(nullptr);
    CHKCV((napi_get_uv_event_loop(asyncCallbackInfo->env, &loop) == napi_ok), "napi_get_uv_event_loop fail");
    CHKPV(loop);
    uv_work_t *work = new(std::nothrow) uv_work_t;
    CHKPV(work);
    asyncCallbackInfo->IncStrongRef(nullptr);
    work->data = asyncCallbackInfo.GetRefPtr();
    int32_t ret = uv_queue_work_with_qos(loop, work, [] (uv_work_t *work) { }, [] (uv_work_t *work, int status) {
        CHKPV(work);
        sptr<AsyncCallbackInfo> asyncCallbackInfo(static_cast<AsyncCallbackInfo *>(work->data));
        DeleteWork(work);
        /**
         * After the asynchronous task is created, the asyncCallbackInfo reference count is reduced
         * to 0 destruction, so you need to add 1 to the asyncCallbackInfo reference count when the
         * asynchronous task is created, and subtract 1 from the reference count after the naked
         * pointer is converted to a pointer when the asynchronous task is executed, the reference
         * count of the smart pointer is guaranteed to be 1.
         */
        asyncCallbackInfo->DecStrongRef(nullptr);
        napi_handle_scope scope = nullptr;
        napi_open_handle_scope(asyncCallbackInfo->env, &scope);
        if (scope == nullptr) {
            SEN_HILOGE("napi_handle_scope is nullptr");
            ReleaseCallback(asyncCallbackInfo);
            return;
        }
        napi_env env = asyncCallbackInfo->env;
        napi_value callback = nullptr;
        if (napi_get_reference_value(env, asyncCallbackInfo->callback[0], &callback) != napi_ok) {
            SEN_HILOGE("napi_get_reference_value fail");
            napi_throw_error(env, nullptr, "napi_get_reference_value fail");
            ReleaseCallback(asyncCallbackInfo);
            napi_close_handle_scope(asyncCallbackInfo->env, scope);
            return;
        }
        napi_value callResult = nullptr;
        napi_value result[2] = {0};
        if (!(g_convertfuncList.find(asyncCallbackInfo->type) != g_convertfuncList.end())) {
            SEN_HILOGE("asyncCallbackInfo type is invalid");
            napi_throw_error(env, nullptr, "asyncCallbackInfo type is invalid");
            ReleaseCallback(asyncCallbackInfo);
            napi_close_handle_scope(asyncCallbackInfo->env, scope);
            return;
        }
        g_convertfuncList[asyncCallbackInfo->type](env, asyncCallbackInfo, result);
        if (napi_call_function(env, nullptr, callback, 1, &result[1], &callResult) != napi_ok) {
            SEN_HILOGE("napi_call_function callback fail");
            napi_throw_error(env, nullptr, "napi_call_function callback fail");
            ReleaseCallback(asyncCallbackInfo);
            napi_close_handle_scope(asyncCallbackInfo->env, scope);
            return;
        }
        ReleaseCallback(asyncCallbackInfo);
        napi_close_handle_scope(asyncCallbackInfo->env, scope);
    }, uv_qos_default);
    if (ret != 0) {
        SEN_HILOGE("uv_queue_work_with_qos fail");
        asyncCallbackInfo->DecStrongRef(nullptr);
        DeleteWork(work);
    }
}

void EmitPromiseWork(sptr<AsyncCallbackInfo> asyncCallbackInfo)
{
    CALL_LOG_ENTER;
    CHKPV(asyncCallbackInfo);
    napi_value resourceName = nullptr;
    napi_env env = asyncCallbackInfo->env;
    napi_status ret = napi_create_string_latin1(env, "Promise", NAPI_AUTO_LENGTH, &resourceName);
    CHKCV((ret == napi_ok), "napi_create_string_latin1 fail");
    asyncCallbackInfo->IncStrongRef(nullptr);
    napi_status status = napi_create_async_work(env, nullptr, resourceName,
        [](napi_env env, void *data) {},
        [](napi_env env, napi_status status, void *data) {
            CALL_LOG_ENTER;
            sptr<AsyncCallbackInfo> asyncCallbackInfo(static_cast<AsyncCallbackInfo *>(data));
            /**
             * After the asynchronous task is created, the asyncCallbackInfo reference count is reduced
             * to 0 destruction, so you need to add 1 to the asyncCallbackInfo reference count when the
             * asynchronous task is created, and subtract 1 from the reference count after the naked
             * pointer is converted to a pointer when the asynchronous task is executed, the reference
             * count of the smart pointer is guaranteed to be 1.
             */
            asyncCallbackInfo->DecStrongRef(nullptr);
            napi_value result[2] = {0};
            CHKCV((g_convertfuncList.find(asyncCallbackInfo->type) != g_convertfuncList.end()),
                "Callback type invalid in promise");
            bool ret = g_convertfuncList[asyncCallbackInfo->type](env, asyncCallbackInfo, result);
            CHKCV(ret, "Callback type invalid in promise");
            if (asyncCallbackInfo->type == FAIL) {
                CHKCV((napi_reject_deferred(env, asyncCallbackInfo->deferred, result[0]) == napi_ok),
                    "napi_reject_deferred fail");
            } else {
                CHKCV((napi_resolve_deferred(env, asyncCallbackInfo->deferred, result[1]) == napi_ok),
                    "napi_resolve_deferred fail");
            }
        },
        asyncCallbackInfo.GetRefPtr(), &asyncCallbackInfo->asyncWork);
    if (status != napi_ok
        || napi_queue_async_work_with_qos(
            asyncCallbackInfo->env, asyncCallbackInfo->asyncWork, napi_qos_default) != napi_ok) {
        SEN_HILOGE("Create async work fail");
        asyncCallbackInfo->DecStrongRef(nullptr);
    }
}
}  // namespace Sensors
}  // namespace OHOS