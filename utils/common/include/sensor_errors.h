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

#ifndef SENSORS_ERRORS_H
#define SENSORS_ERRORS_H

#include <errors.h>
#include <sensor_log.h>

#include "hilog/log.h"

namespace OHOS {
namespace Sensors {
constexpr uint32_t MAX_SENSOR_COUNT = 200;
// Error code for user
enum ErrorCode : int32_t {
    PERMISSION_DENIED = 201, // Use this error code when permission is denied.
    PARAMETER_ERROR = 401, // Use this error code when the input parameter type or range does not match.
    SERVICE_EXCEPTION = 14500101, // Use this error code when the service is exception.
    SENSOR_NO_SUPPORT = 14500102, // Use this error code when the sensor is not supported by the device.
    NON_SYSTEM_API = 202 // Permission check failed. A non-system application uses the system API.
};

enum {
    MODULE_COMMON = 0x00,
    MODULE_SENSORS_DEVICE = 0x01,
    MODULE_SENSOR_SERVICE = 0x02,
    MODULE_SENSORS_UTILS = 0x03,
    MODULE_SENSORS_NATIVE = 0X04,
};

// Error code for common
constexpr ErrCode COMMON_ERR_OFFSET = ErrCodeOffset(SUBSYS_SENSORS, MODULE_COMMON);

enum {
    ERROR = -1,
    SUCCESS = 0,
    COMMON_ERR = COMMON_ERR_OFFSET,
    FILE_OPEN_FAIL = COMMON_ERR_OFFSET + 1,
    STREAM_BUF_READ_FAIL = FILE_OPEN_FAIL + 1,
};

// Error code for device
constexpr ErrCode DEVICE_ERR_OFFSET = ErrCodeOffset(SUBSYS_SENSORS, MODULE_SENSORS_DEVICE);

enum {
    DEVICE_ERR = DEVICE_ERR_OFFSET,
    DEVICE_ENABLE_SENSOR_ERR = DEVICE_ERR_OFFSET + 1,
    DEVICE_DISABLE_SENSOR_ERR = DEVICE_ENABLE_SENSOR_ERR + 1,
    DEVICE_RUN_COMMAND_ERR = DEVICE_DISABLE_SENSOR_ERR + 1,
    DEVICE_SET_SENSOR_CONFIG_ERR = DEVICE_RUN_COMMAND_ERR + 1,
    DEVICE_NOT_SUPPORT_CMD_ERR = DEVICE_SET_SENSOR_CONFIG_ERR + 1,
};

// Error code for sensor service
constexpr ErrCode SENSOR_SERVICE_ERR_OFFSET = ErrCodeOffset(SUBSYS_SENSORS, MODULE_SENSOR_SERVICE);

enum {
    SENSOR_SERVICE_ERR = SENSOR_SERVICE_ERR_OFFSET,
    CMD_TYPE_ERR = SENSOR_SERVICE_ERR + 1,
    SENSOR_DEVICE_INIT_ERR = CMD_TYPE_ERR + 1,
    CONNECT_SENSOR_HDF_ERR = SENSOR_DEVICE_INIT_ERR + 1,
    SET_SENSOR_CONFIG_ERR = CONNECT_SENSOR_HDF_ERR + 1,
    ENABLE_SENSOR_ERR = SET_SENSOR_CONFIG_ERR + 1,
    DISABLE_SENSOR_ERR = ENABLE_SENSOR_ERR + 1,
    RUN_COMMAND_ERR = DISABLE_SENSOR_ERR + 1,
    GET_SENSOR_LIST_ERR = RUN_COMMAND_ERR + 1,
    SENSOR_ENABLED_ERR = GET_SENSOR_LIST_ERR + 1,
    UPDATE_SENSOR_CHANNEL_ERR = SENSOR_ENABLED_ERR + 1,
    CLEAR_SENSOR_CHANNEL_ERR = UPDATE_SENSOR_CHANNEL_ERR + 1,
    CLEAR_SENSOR_INFO_ERR = CLEAR_SENSOR_CHANNEL_ERR + 1,
    UPDATE_SENSOR_INFO_ERR = CLEAR_SENSOR_INFO_ERR + 1,
    CLIENT_PID_INVALID_ERR = UPDATE_SENSOR_INFO_ERR + 1,
    DESTROY_SENSOR_CHANNEL_ERR = CLIENT_PID_INVALID_ERR + 1,
    UPDATE_UID_ERR = DESTROY_SENSOR_CHANNEL_ERR + 1,
    INVALID_POINTER = UPDATE_UID_ERR + 1,
    NO_EVENT = INVALID_POINTER + 1,
    COPY_ERR = NO_EVENT + 1,
    REGIST_PERMISSION_CHANGED_ERR = COPY_ERR + 1,
    DUMP_PARAM_ERR = REGIST_PERMISSION_CHANGED_ERR + 1,
    WRITE_PARCEL_ERR = DUMP_PARAM_ERR + 1,
    READ_PARCEL_ERR = WRITE_PARCEL_ERR + 1,
    SET_SENSOR_MODE_ERR = READ_PARCEL_ERR + 1,
    SET_SENSOR_OPTION_ERR = SET_SENSOR_MODE_ERR + 1,
    REGIST_CALLBACK_ERR = SET_SENSOR_OPTION_ERR + 1,
    SUSPEND_ERR = REGIST_CALLBACK_ERR + 1,
    RESUME_ERR = SUSPEND_ERR + 1,
    RESET_ERR = RESUME_ERR + 1,
};

// Error code for Sensor utils
constexpr ErrCode SENSOR_UTILS_ERR_OFFSET = ErrCodeOffset(SUBSYS_SENSORS, MODULE_SENSORS_UTILS);
enum {
    SENSOR_CHANNEL_SOCKET_CREATE_ERR = SENSOR_UTILS_ERR_OFFSET,
    SENSOR_CHANNEL_SENDFD_ERR = SENSOR_CHANNEL_SOCKET_CREATE_ERR + 1,
    SENSOR_CHANNEL_WRITE_DESCRIPTOR_ERR = SENSOR_CHANNEL_SENDFD_ERR + 1,
    SENSOR_CHANNEL_READ_DESCRIPTOR_ERR = SENSOR_CHANNEL_WRITE_DESCRIPTOR_ERR + 1,
    SENSOR_CHANNEL_BASIC_CHANNEL_NOT_INIT = SENSOR_CHANNEL_READ_DESCRIPTOR_ERR + 1,
    SENSOR_CHANNEL_SEND_ADDR_ERR = SENSOR_CHANNEL_BASIC_CHANNEL_NOT_INIT + 1,
    SENSOR_CHANNEL_SEND_DATA_ERR = SENSOR_CHANNEL_SEND_ADDR_ERR + 1,
    SENSOR_CHANNEL_RECEIVE_DATA_ERR = SENSOR_CHANNEL_SEND_DATA_ERR + 1,
    SENSOR_CHANNEL_RECEIVE_ADDR_ERR = SENSOR_CHANNEL_RECEIVE_DATA_ERR + 1,
    SENSOR_CHANNEL_RESTORE_CB_ERR = SENSOR_CHANNEL_RECEIVE_ADDR_ERR + 1,
    SENSOR_CHANNEL_RESTORE_FD_ERR = SENSOR_CHANNEL_RESTORE_CB_ERR + 1,
    SENSOR_CHANNEL_RESTORE_THREAD_ERR = SENSOR_CHANNEL_RESTORE_FD_ERR + 1,
};
// Error code for Sensor native
constexpr ErrCode SENSOR_NATIVE_ERR_OFFSET = ErrCodeOffset(SUBSYS_SENSORS, MODULE_SENSORS_NATIVE);

enum {
    SENSOR_NATIVE_SAM_ERR = SENSOR_NATIVE_ERR_OFFSET,
    SENSOR_NATIVE_GET_SERVICE_ERR = SENSOR_NATIVE_SAM_ERR + 1,
    SENSOR_NATIVE_REGSITER_CB_ERR = SENSOR_NATIVE_GET_SERVICE_ERR + 1,
};

class InnerFunctionTracer {
public:
    InnerFunctionTracer(const char *func, const char *tag)
        : func_ { func }, tag_ { tag }
    {
        if (HiLogIsLoggable(LOG_DOMAIN, tag_, LOG_DEBUG)) {
            if (func_  != nullptr && tag_ != nullptr) {
                HILOG_IMPL(LOG_CORE, LOG_DEBUG, LOG_DOMAIN, tag_, "in %{public}s, enter", func_);
            }
        }
    }
    ~InnerFunctionTracer()
    {
        if (HiLogIsLoggable(LOG_DOMAIN, tag_, LOG_DEBUG)) {
            if (func_  != nullptr && tag_ != nullptr) {
                HILOG_IMPL(LOG_CORE, LOG_DEBUG, LOG_DOMAIN, tag_, "in %{public}s, leave", func_);
            }
        }
    }
private:
    const char *func_ { nullptr };
    const char *tag_ { nullptr };
};

#define CALL_LOG_ENTER InnerFunctionTracer ___innerFuncTracer___ { __FUNCTION__, LOG_TAG }

#ifdef DEBUG_CODE_TEST
#define CHKPL(cond, ...) \
    do { \
        if ((cond) == nullptr) { \
            SEN_HILOGW("%{public}s, (%{public}d), CHKPL(%{public}s) is null, do nothing", \
                __FILE__, __LINE__, #cond); \
        } \
    } while (0)

#define CHKPV(cond) \
    do { \
        if ((cond) == nullptr) { \
            SEN_HILOGE("%{public}s, (%{public}d), CHKPV(%{public}s) is null", \
                __FILE__, __LINE__, #cond); \
            return; \
        } \
    } while (0)

#define CHKPF(cond) \
    do { \
        if ((cond) == nullptr) { \
            SEN_HILOGE("%{public}s, (%{public}d), CHKPF(%{public}s) is null", \
                __FILE__, __LINE__, #cond); \
            return false; \
        } \
    } while (0)

#define CHKPC(cond) \
    { \
        if ((cond) == nullptr) { \
            SEN_HILOGW("%{public}s, (%{public}d), CHKPC(%{public}s) is null, skip then continue", \
                __FILE__, __LINE__, #cond); \
            continue; \
        } \
    }

#define CHKPB(cond) \
    { \
        if ((cond) == nullptr) { \
            SEN_HILOGW("%{public}s, (%{public}d), CHKPC(%{public}s) is null, skip then break", \
                __FILE__, __LINE__, #cond); \
            break; \
        } \
    }

#define CHKPR(cond, r) \
    do { \
        if ((cond) == nullptr) { \
            SEN_HILOGE("%{public}s, (%{public}d), CHKPR(%{public}s) is null, return value is %{public}d", \
                __FILE__, __LINE__, #cond, r); \
            return r; \
        } \
    } while (0)

#define CHKPP(cond) \
    do { \
        if ((cond) == nullptr) { \
            SEN_HILOGE("%{public}s, (%{public}d), CHKPP(%{public}s) is null, return value is null", \
                __FILE__, __LINE__, #cond); \
            return nullptr; \
        } \
    } while (0)

#define CK(cond, ec) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("%{public}s, (%{public}d), CK(%{public}s), errCode:%{public}d", \
                __FILE__, __LINE__, #cond, ec); \
        } \
    } while (0)

#else // DEBUG_CODE_TEST
#define CHKPL(cond) \
    do { \
        if ((cond) == nullptr) { \
            SEN_HILOGW("CHKPL(%{public}s) is null, do nothing", #cond); \
        } \
    } while (0)

#define CHKPV(cond) \
    do { \
        if ((cond) == nullptr) { \
            SEN_HILOGE("CHKPV(%{public}s) is null", #cond); \
            return; \
        } \
    } while (0)

#define CHKPF(cond) \
    do { \
        if ((cond) == nullptr) { \
            SEN_HILOGE("CHKPF(%{public}s) is null", #cond); \
            return false; \
        } \
    } while (0)

#define CHKPC(cond) \
    { \
        if ((cond) == nullptr) { \
            SEN_HILOGW("CHKPC(%{public}s) is null, skip then continue", #cond); \
            continue; \
        } \
    }

#define CHKPB(cond) \
    { \
        if ((cond) == nullptr) { \
            SEN_HILOGW("CHKPC(%{public}s) is null, skip then break", #cond); \
            break; \
        } \
    }

#define CHKPR(cond, r) \
    do { \
        if ((cond) == nullptr) { \
            SEN_HILOGE("CHKPR(%{public}s) is null, return value is %{public}d", #cond, r); \
            return r; \
        } \
    } while (0)

#define CHKCF(cond, message) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("CK(%{public}s), %{public}s", #cond, message); \
            return false; \
        } \
    } while (0)

#define CHKCP(cond, message) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("(%{public}s)", #message); \
            return nullptr; \
        } \
    } while (0)

#define CHKCV(cond, message) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("CK(%{public}s), %{public}s", #cond, message); \
            return; \
        } \
    } while (0)

#define CHKPP(cond) \
    do { \
        if ((cond) == nullptr) { \
            SEN_HILOGE("CHKPP(%{public}s) is null, return value is null", #cond); \
            return nullptr; \
        } \
    } while (0)

#define CK(cond, ec) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("CK(%{public}s), errCode:%{public}d", #cond, ec); \
        } \
    } while (0)

#define CHKCR(cond, r) \
    do { \
        if (!(cond)) { \
            SEN_HILOGE("CK(%{public}s), errCode:%{public}d", #cond, r); \
            return r; \
        } \
    } while (0)

#endif
} // namespace Sensors
} // namespace OHOS
#endif // SENSORS_ERRORS_H
