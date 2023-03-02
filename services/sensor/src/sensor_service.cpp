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

#include "sensor_service.h"

#include <cinttypes>
#include <string_ex.h>
#include <sys/socket.h>
#include <unistd.h>

#include "accesstoken_kit.h"
#include "hisysevent.h"
#include "iservice_registry.h"
#include "permission_util.h"
#include "securec.h"
#include "sensor.h"
#include "sensor_dump.h"
#include "sensors_errors.h"
#include "system_ability_definition.h"

namespace OHOS {
namespace Sensors {
using namespace OHOS::HiviewDFX;
using namespace Security::AccessToken;
namespace {
constexpr HiLogLabel LABEL = { LOG_CORE, SENSOR_LOG_DOMAIN, "SensorService" };
constexpr int32_t INVALID_SENSOR_ID = -1;
constexpr int32_t INVALID_PID = -1;
constexpr int64_t MAX_EVENT_COUNT = 1000;
enum {
    FLUSH = 0,
    SET_MODE,
    RESERVED,
};
}  // namespace

REGISTER_SYSTEM_ABILITY_BY_ID(SensorService, SENSOR_SERVICE_ABILITY_ID, true);

SensorService::SensorService(int32_t systemAbilityId, bool runOnCreate)
    : SystemAbility(systemAbilityId, runOnCreate), state_(SensorServiceState::STATE_STOPPED)
{}

void SensorService::OnDump()
{
    SEN_HILOGI("OnDump");
}

void SensorService::OnStart()
{
    CALL_LOG_ENTER;
    if (state_ == SensorServiceState::STATE_RUNNING) {
        SEN_HILOGW("SensorService has already started");
        return;
    }
    if (!InitInterface()) {
        SEN_HILOGE("Init interface error");
        return;
    }
    if (!InitDataCallback()) {
        SEN_HILOGE("Init data callback error");
        return;
    }
    if (!InitSensorList()) {
        SEN_HILOGE("Init sensor list error");
        return;
    }
    sensorDataProcesser_ = new (std::nothrow) SensorDataProcesser(sensorMap_);
    CHKPV(sensorDataProcesser_);
    if (!InitSensorPolicy()) {
        SEN_HILOGE("Init sensor policy error");
    }

    if (!SystemAbility::Publish(this)) {
        SEN_HILOGE("publish SensorService error");
        return;
    }
    sensorManager_.InitSensorMap(sensorMap_, sensorDataProcesser_, reportDataCallback_);
    state_ = SensorServiceState::STATE_RUNNING;
}

bool SensorService::InitInterface()
{
    auto ret = sensorHdiConnection_.ConnectHdi();
    if (ret != ERR_OK) {
        SEN_HILOGE("connect hdi failed");
        return false;
    }
    return true;
}

bool SensorService::InitDataCallback()
{
    reportDataCallback_ = new (std::nothrow) ReportDataCallback();
    CHKPF(reportDataCallback_);
    ReportDataCb cb = &ReportDataCallback::ReportEventCallback;
    auto ret = sensorHdiConnection_.RegisteDataReport(cb, reportDataCallback_);
    if (ret != ERR_OK) {
        SEN_HILOGE("RegisterDataReport failed");
        return false;
    }
    return true;
}

bool SensorService::InitSensorList()
{
    std::lock_guard<std::mutex> sensorLock(sensorsMutex_);
    int32_t ret = sensorHdiConnection_.GetSensorList(sensors_);
    if (ret != 0) {
        SEN_HILOGE("GetSensorList is failed");
        return false;
    }
    {
        std::lock_guard<std::mutex> sensorMapLock(sensorMapMutex_);
        for (const auto &it : sensors_) {
            if (!(sensorMap_.insert(std::make_pair(it.GetSensorId(), it)).second)) {
                SEN_HILOGW("sensorMap_ Insert failed");
            }
        }
    }
    return true;
}

bool SensorService::InitSensorPolicy()
{
    return true;
}

void SensorService::OnStop()
{
    CALL_LOG_ENTER;
    if (state_ == SensorServiceState::STATE_STOPPED) {
        SEN_HILOGW("already stopped");
        return;
    }
    state_ = SensorServiceState::STATE_STOPPED;
    int32_t ret = sensorHdiConnection_.DestroyHdiConnection();
    if (ret != ERR_OK) {
        SEN_HILOGE("destroy hdi connect fail");
    }
}

void SensorService::ReportSensorSysEvent(int32_t sensorId, bool enable, int32_t pid)
{
    std::string packageName("");
    AccessTokenID tokenId = clientInfo_.GetTokenIdByPid(pid);
    sensorManager_.GetPackageName(tokenId, packageName);
    const int logLevel = 4;
    int32_t uid = clientInfo_.GetUidByPid(pid);
    if (enable) {
        HiSysEventWrite(HiSysEvent::Domain::SENSOR, "ENABLE_SENSOR", HiSysEvent::EventType::STATISTIC,
            "LEVEL", logLevel, "UID", uid, "PKG_NAME", packageName, "TYPE", sensorId);
    } else {
        HiSysEventWrite(HiSysEvent::Domain::SENSOR, "DISABLE_SENSOR", HiSysEvent::EventType::STATISTIC,
            "LEVEL", logLevel, "UID", uid, "PKG_NAME", packageName, "TYPE", sensorId);
    }
}

void SensorService::ReportOnChangeData(int32_t sensorId)
{
    std::lock_guard<std::mutex> sensorMapLock(sensorMapMutex_);
    auto it = sensorMap_.find(sensorId);
    if (it == sensorMap_.end()) {
        SEN_HILOGE("sensorId is invalid");
        return;
    }
    if ((SENSOR_ON_CHANGE & it->second.GetFlags()) != SENSOR_ON_CHANGE) {
        SEN_HILOGW("it is not onchange data, no need to report");
        return;
    }
    SensorData sensorData;
    auto ret = clientInfo_.GetStoreEvent(sensorId, sensorData);
    if (ret != ERR_OK) {
        SEN_HILOGE("there is no data to be reported");
        return;
    }
    sptr<SensorBasicDataChannel> channel = clientInfo_.GetSensorChannelByPid(GetCallingPid());
    CHKPV(channel);
    auto sendRet = channel->SendData(&sensorData, sizeof(sensorData));
    if (sendRet != ERR_OK) {
        SEN_HILOGE("send data failed");
        return;
    }
}

ErrCode SensorService::SaveSubscriber(int32_t sensorId, int64_t samplingPeriodNs, int64_t maxReportDelayNs)
{
    auto ret = sensorManager_.SaveSubscriber(sensorId, GetCallingPid(), samplingPeriodNs, maxReportDelayNs);
    if (ret != ERR_OK) {
        SEN_HILOGE("SaveSubscriber failed");
        return ret;
    }
    sensorManager_.StartDataReportThread();
    if (!sensorManager_.SetBestSensorParams(sensorId, samplingPeriodNs, maxReportDelayNs)) {
        SEN_HILOGE("SetBestSensorParams failed");
        clientInfo_.RemoveSubscriber(sensorId, GetCallingPid());
        return ENABLE_SENSOR_ERR;
    }
    return ret;
}

ErrCode SensorService::EnableSensor(int32_t sensorId, int64_t samplingPeriodNs, int64_t maxReportDelayNs)
{
    CALL_LOG_ENTER;
    if ((sensorId == INVALID_SENSOR_ID) ||
        ((samplingPeriodNs != 0L) && ((maxReportDelayNs / samplingPeriodNs) > MAX_EVENT_COUNT))) {
        SEN_HILOGE("sensorId is 0 or maxReportDelayNs exceeded the maximum value");
        return ERR_NO_INIT;
    }
    int32_t pid = GetCallingPid();
    std::lock_guard<std::mutex> serviceLock(serviceLock_);
    if (clientInfo_.GetSensorState(sensorId)) {
        SEN_HILOGW("sensor has been enabled already");
        auto ret = SaveSubscriber(sensorId, samplingPeriodNs, maxReportDelayNs);
        if (ret != ERR_OK) {
            SEN_HILOGE("SaveSubscriber failed");
            return ret;
        }
        ReportSensorSysEvent(sensorId, true, pid);
        uint32_t flag = sensorManager_.GetSensorFlag(sensorId);
        ret = flushInfo_.FlushProcess(sensorId, flag, pid, true);
        if (ret != ERR_OK) {
            SEN_HILOGE("ret : %{public}d", ret);
        }
        ReportOnChangeData(sensorId);
        if (IsReportClientInfo_) {
            ReportClientInfo(sensorId, true, pid);
        }
        return ERR_OK;
    }
    auto ret = SaveSubscriber(sensorId, samplingPeriodNs, maxReportDelayNs);
    if (ret != ERR_OK) {
        SEN_HILOGE("SaveSubscriber failed");
        clientInfo_.RemoveSubscriber(sensorId, GetCallingPid());
        return ret;
    }
    ret = sensorHdiConnection_.EnableSensor(sensorId);
    if (ret != ERR_OK) {
        SEN_HILOGE("EnableSensor failed");
        clientInfo_.RemoveSubscriber(sensorId, GetCallingPid());
        return ENABLE_SENSOR_ERR;
    }
    ReportSensorSysEvent(sensorId, true, pid);
    if (IsReportClientInfo_) {
        ReportClientInfo(sensorId, true, pid);
    }
    return ret;
}

ErrCode SensorService::DisableSensor(int32_t sensorId, int32_t pid)
{
    CALL_LOG_ENTER;
    if (sensorId == INVALID_SENSOR_ID) {
        SEN_HILOGE("sensorId is invalid");
        return ERR_NO_INIT;
    }
    if (pid < 0) {
        SEN_HILOGE("pid is invalid, pid:%{public}d", pid);
        return CLIENT_PID_INVALID_ERR;
    }
    ReportSensorSysEvent(sensorId, false, pid);
    std::lock_guard<std::mutex> serviceLock(serviceLock_);
    if (sensorManager_.IsOtherClientUsingSensor(sensorId, pid)) {
        SEN_HILOGW("other client is using this sensor now, cannot disable");
        if (IsReportClientInfo_) {
            ReportClientInfo(sensorId, false, pid);
        }
        return ERR_OK;
    }
    if (sensorHdiConnection_.DisableSensor(sensorId) != ERR_OK) {
        SEN_HILOGE("DisableSensor is failed");
        return DISABLE_SENSOR_ERR;
    }
    int32_t uid = clientInfo_.GetUidByPid(pid);
    clientInfo_.DestroyCmd(uid);
    clientInfo_.ClearDataQueue(sensorId);
    if (IsReportClientInfo_) {
        ReportClientInfo(sensorId, false, pid);
    }
    return sensorManager_.AfterDisableSensor(sensorId);
}

ErrCode SensorService::DisableSensor(int32_t sensorId)
{
    CALL_LOG_ENTER;
    return DisableSensor(sensorId, GetCallingPid());
}

std::vector<Sensor> SensorService::GetSensorList()
{
    std::lock_guard<std::mutex> sensorLock(sensorsMutex_);
    int32_t ret = sensorHdiConnection_.GetSensorList(sensors_);
    if (ret != 0) {
        SEN_HILOGE("GetSensorList is failed");
        return sensors_;
    }
    for (const auto &it : sensors_) {
        std::lock_guard<std::mutex> sensorMapLock(sensorMapMutex_);
        sensorMap_.insert(std::make_pair(it.GetSensorId(), it));
    }
    return sensors_;
}

ErrCode SensorService::TransferDataChannel(const sptr<SensorBasicDataChannel> &sensorBasicDataChannel,
                                           const sptr<IRemoteObject> &sensorClient)
{
    CHKPR(sensorBasicDataChannel, ERR_NO_INIT);
    auto pid = GetCallingPid();
    auto uid = GetCallingUid();
    auto callerToken = GetCallingTokenID();
    if (!clientInfo_.UpdateAppThreadInfo(pid, uid, callerToken)) {
        SEN_HILOGE("UpdateUid is failed");
        return UPDATE_UID_ERR;
    }
    if (!clientInfo_.UpdateSensorChannel(pid, sensorBasicDataChannel)) {
        SEN_HILOGE("UpdateSensorChannel is failed");
        return UPDATE_SENSOR_CHANNEL_ERR;
    }
    sensorBasicDataChannel->SetSensorStatus(true);
    RegisterClientDeathRecipient(sensorClient, pid);
    return ERR_OK;
}

ErrCode SensorService::DestroySensorChannel(sptr<IRemoteObject> sensorClient)
{
    CALL_LOG_ENTER;
    const int32_t clientPid = GetCallingPid();
    if (clientPid < 0) {
        SEN_HILOGE("clientPid is invalid, clientPid:%{public}d", clientPid);
        
        return CLIENT_PID_INVALID_ERR;
    }
    std::lock_guard<std::mutex> serviceLock(serviceLock_);
    bool destroyRet = clientInfo_.DestroySensorChannel(clientPid);
    if (!destroyRet) {
        SEN_HILOGE("DestroySensorChannel is failed");
        return DESTROY_SENSOR_CHANNEL_ERR;
    }
    clientInfo_.DestroyCmd(GetCallingUid());
    UnregisterClientDeathRecipient(sensorClient);
    return ERR_OK;
}

void SensorService::ProcessDeathObserver(const wptr<IRemoteObject> &object)
{
    CALL_LOG_ENTER;
    sptr<IRemoteObject> client = object.promote();
    CHKPV(client);
    int32_t pid = clientInfo_.FindClientPid(client);
    if (pid == INVALID_PID) {
        SEN_HILOGE("pid is -1");
        return;
    }
    SEN_HILOGI("pid is %{public}d", pid);
    std::vector<int32_t> activeSensors = clientInfo_.GetSensorIdByPid(pid);
    for (size_t i = 0; i < activeSensors.size(); ++i) {
        int32_t ret = DisableSensor(activeSensors[i], pid);
        if (ret != ERR_OK) {
            SEN_HILOGE("disablesensor failed, ret:%{public}d", ret);
        }
    }
    DelSession(pid);
    clientInfo_.DelClientInfoCallbackPid(pid);
    clientInfo_.DestroySensorChannel(pid);
    clientInfo_.DestroyClientPid(client);
    clientInfo_.DestroyCmd(clientInfo_.GetUidByPid(pid));
}

void SensorService::RegisterClientDeathRecipient(sptr<IRemoteObject> sensorClient, int32_t pid)
{
    CALL_LOG_ENTER;
    sptr<ISensorClient> client = iface_cast<ISensorClient>(sensorClient);
    clientDeathObserver_ = new (std::nothrow) DeathRecipientTemplate(*const_cast<SensorService *>(this));
    CHKPV(clientDeathObserver_);
    client->AsObject()->AddDeathRecipient(clientDeathObserver_);
    clientInfo_.SaveClientPid(sensorClient, pid);
}

void SensorService::UnregisterClientDeathRecipient(sptr<IRemoteObject> sensorClient)
{
    CALL_LOG_ENTER;
    int32_t pid = clientInfo_.FindClientPid(sensorClient);
    if (pid == INVALID_PID) {
        SEN_HILOGE("pid is -1");
        return;
    }
    if (!clientInfo_.IsUnregisterClientDeathRecipient(pid)) {
        SEN_HILOGI("Client call other service, not need unregister client death recipient, pid:%{public}d", pid);
        return;
    }
    sptr<ISensorClient> client = iface_cast<ISensorClient>(sensorClient);
    clientDeathObserver_ = new (std::nothrow) DeathRecipientTemplate(*const_cast<SensorService *>(this));
    CHKPV(clientDeathObserver_);
    client->AsObject()->RemoveDeathRecipient(clientDeathObserver_);
    clientInfo_.DestroyClientPid(sensorClient);
}

int32_t SensorService::Dump(int32_t fd, const std::vector<std::u16string> &args)
{
    CALL_LOG_ENTER;
    if (fd < 0) {
        SEN_HILOGE("Invalid fd");
        return DUMP_PARAM_ERR;
    }
    SensorDump &sensorDump = SensorDump::GetInstance();
    if (args.empty()) {
        SEN_HILOGE("param cannot be empty");
        dprintf(fd, "param cannot be empty\n");
        sensorDump.DumpHelp(fd);
        return DUMP_PARAM_ERR;
    }
    std::vector<std::string> argList = { "" };
    std::transform(args.begin(), args.end(), std::back_inserter(argList),
        [](const std::u16string &arg) {
        return Str16ToStr8(arg);
    });
    sensorDump.ParseCommand(fd, argList, sensors_, clientInfo_);
    return ERR_OK;
}

ErrCode SensorService::SuspendSensors(int32_t pid)
{
    CALL_LOG_ENTER;
    if (pid < 0) {
        SEN_HILOGE("Pid is invalid, pid:%{public}d", pid);
        return CLIENT_PID_INVALID_ERR;
    }
    int32_t ret = suspendPolicy_.DoSuspend(pid);
    if (ret != ERR_OK) {
        SEN_HILOGE("Suspend pid sensors failed, pid:%{public}d", pid);
        return ERROR;
    }
    return ERR_OK;
}

ErrCode SensorService::ResumeSensors(int32_t pid)
{
    CALL_LOG_ENTER;
    if (pid < 0) {
        SEN_HILOGE("Pid is invalid, pid:%{public}d", pid);
        return CLIENT_PID_INVALID_ERR;
    }
    int32_t ret = suspendPolicy_.DoResume(pid);
    if (ret != ERR_OK) {
        SEN_HILOGE("Resume pid sensors failed, pid:%{public}d", pid);
        return ERROR;
    }
    return ERR_OK;
}

ErrCode SensorService::GetAppSensorList(int32_t pid, std::vector<AppSensor> &appSensorList)
{
    CALL_LOG_ENTER;
    if (pid < 0) {
        SEN_HILOGE("Pid is invalid, pid:%{public}d", pid);
        return CLIENT_PID_INVALID_ERR;
    }
    int32_t ret = suspendPolicy_.GetAppSensorList(pid, appSensorList);
    if (ret != ERR_OK) {
        SEN_HILOGE("Get pid sensor list failed, pid:%{public}d", pid);
        return ERROR;
    }
    return ERR_OK;
}

ErrCode SensorService::CreateSocketChannel(int32_t &clientFd, const sptr<IRemoteObject> &sensorClient)
{
    CALL_LOG_ENTER;
    int32_t uid = GetCallingUid();
    int32_t pid = GetCallingPid();
    int32_t tokenType = AccessTokenKit::GetTokenTypeFlag(GetCallingTokenID());
    int32_t serverFd = -1;
    clientFd = -1;
    int32_t ret = AddSocketPairInfo(uid, pid, tokenType, serverFd, std::ref(clientFd));
    if (ret != ERR_OK) {
        SEN_HILOGE("AddSocketPairInfo failed");
        return ERROR;
    }
    RegisterClientDeathRecipient(sensorClient, pid);
    return ERR_OK;
}

ErrCode SensorService::DestroySocketChannel(const sptr<IRemoteObject> &sensorClient)
{
    CALL_LOG_ENTER;
    int32_t pid = GetCallingPid();
    DelSession(pid);
    UnregisterClientDeathRecipient(sensorClient);
    return ERR_OK;
}

ErrCode SensorService::EnableClientInfoCallback()
{
    CALL_LOG_ENTER;
    IsReportClientInfo_ = true;
    int32_t pid = GetCallingPid();
    return clientInfo_.AddClientInfoCallbackPid(pid);
}

ErrCode SensorService::DisableClientInfoCallback()
{
    CALL_LOG_ENTER;
    IsReportClientInfo_ = false;
    int32_t pid = GetCallingPid();
    return clientInfo_.DelClientInfoCallbackPid(pid);
}

void SensorService::ReportClientInfo(int32_t sensorId, bool isActive, int32_t pid)
{
    CALL_LOG_ENTER;
    std::vector<SessionPtr> sessionList;
    auto pidSet = clientInfo_.GetClientInfoCallbackPidSet();
    for (auto pid : pidSet) {
        auto sess = GetSessionByPid(pid);
        if (sess != nullptr) {
            sessionList.push_back(sess);
        }
    }
    SensorBasicInfo sensorInfo = clientInfo_.GetCurPidSensorInfo(sensorId, pid);
    SensorStatus sensorStatus(sensorId, isActive, sensorInfo.GetSamplingPeriodNs(),
                              sensorInfo.GetMaxReportDelayNs());
    suspendPolicy_.ReportClientInfo(pid, sensorStatus, sessionList);
}
}  // namespace Sensors
}  // namespace OHOS
