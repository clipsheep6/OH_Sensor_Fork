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

/**
 * @addtogroup PanSensor
 * @{
 *
 * @brief Provides standard open APIs for you to use common capabilities of sensors.
 *
 * For example, you can call these APIs to obtain sensor information,
 * subscribe to or unsubscribe from sensor data, enable or disable a sensor,
 * and set the sensor data reporting mode.
 *
 * @since 5
 */

/**
 * @file sensor_agent.h
 *
 * @brief Declares common APIs for sensor management, such as APIs for subscribing to
 * and obtaining sensor data, enabling a sensor, and setting the sensor data reporting mode.
 *
 * @since 5
 */

#ifndef SENSOR_AGENT_H
#define SENSOR_AGENT_H

#include "sensor_agent_type.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/**
 * @brief Obtains information about all sensors in the system.
 *
 * @param sensorInfo Indicates the double pointer to the information about all sensors in the system.
 * For details, see {@link SensorInfo}.
 * @param count Indicates the pointer to the total number of sensors in the system.
 * @return Returns <b>0</b> if the information is obtained; returns a non-zero value otherwise.
 *
 * @since 5
 */
int32_t GetAllSensors(SensorInfo **sensorInfo, int32_t *count);
/**
 * @brief Subscribes to sensor data. The system will report the obtained sensor data to the subscriber.
 *
 * @param sensorTypeId Indicates the ID of a sensor type. For details, see {@link SensorTypeId}.
 * @param user Indicates the pointer to the sensor subscriber that requests sensor data. For details,
 * see {@link SensorUser}. A subscriber can obtain data from only one sensor.
 * @return Returns <b>0</b> if the subscription is successful; returns a non-zero value otherwise.
 *
 * @since 5
 */
int32_t SubscribeSensor(int32_t sensorTypeId, const SensorUser *user);
/**
 * @brief Unsubscribes from sensor data.
 *
 * @param sensorTypeId Indicates the ID of a sensor type. For details, see {@link SensorTypeId}.
 * @param user Indicates the pointer to the sensor subscriber that requests sensor data.
 * For details, see {@link SensorUser}. A subscriber can obtain data from only one sensor.
 * @return Returns <b>0</b> if the unsubscription is successful; returns a non-zero value otherwise.
 *
 * @since 5
 */
int32_t UnsubscribeSensor(int32_t sensorTypeId, const SensorUser *user);
/**
 * @brief Sets the data sampling interval and data reporting interval for the specified sensor.
 *
 * @param sensorTypeId Indicates the ID of a sensor type. For details, see {@link SensorTypeId}.
 * @param user Indicates the pointer to the sensor subscriber that requests sensor data.
 * For details, see {@link SensorUser}. A subscriber can obtain data from only one sensor.
 * @param samplingInterval Indicates the sensor data sampling interval to set, in nanoseconds.
 * @param reportInterval Indicates the sensor data reporting interval, in nanoseconds.
 * @return Returns <b>0</b> if the setting is successful; returns a non-zero value otherwise.
 *
 * @since 5
 */
int32_t SetBatch(int32_t sensorTypeId, const SensorUser *user, int64_t samplingInterval, int64_t reportInterval);
/**
 * @brief Enables the sensor that has been subscribed to. The subscriber can obtain the sensor data
 * only after the sensor is enabled.
 *
 * @param sensorTypeId Indicates the ID of a sensor type. For details, see {@link SensorTypeId}.
 * @param user Indicates the pointer to the sensor subscriber that requests sensor data.
 * For details, see {@link SensorUser}. A subscriber can obtain data from only one sensor.
 * @return Returns <b>0</b> if the sensor is successfully enabled; returns a non-zero value otherwise.
 *
 * @since 5
 */
int32_t ActivateSensor(int32_t sensorTypeId, const SensorUser *user);
/**
 * @brief Disables an enabled sensor.
 *
 * @param sensorTypeId Indicates the ID of a sensor type. For details, see {@link SensorTypeId}.
 * @param user Indicates the pointer to the sensor subscriber that requests sensor data.
 * For details, see {@link SensorUser}. A subscriber can obtain data from only one sensor.
 * @return Returns <b>0</b> if the sensor is successfully disabled; returns a non-zero value otherwise.
 *
 * @since 5
 */
int32_t DeactivateSensor(int32_t sensorTypeId, const SensorUser *user);
/**
 * @brief Sets the data reporting mode for the specified sensor.
 *
 * @param sensorTypeId Indicates the ID of a sensor type. For details, see {@link SensorTypeId}.
 * @param user Indicates the pointer to the sensor subscriber that requests sensor data.
 * For details, see {@link SensorUser}. A subscriber can obtain data from only one sensor.
 * @param mode Indicates the data reporting mode to set. For details, see {@link SensorMode}.
 * @return Returns <b>0</b> if the sensor data reporting mode is successfully set; returns a non-zero value otherwise.
 *
 * @since 5
 */
int32_t SetMode(int32_t sensorTypeId, const SensorUser *user, int32_t mode);

/**
 * @brief 挂起一个进程订阅的所有传感器
 *
 * @param pid 将被挂起的进程的进程号
 * @return 返回0表示成功，否则表示失败
 *
 * @since 10
 */
int32_t SuspendSensors(int32_t pid);

/**
 * @brief 唤醒一个进程订阅的所有传感器
 *
 * @param pid 将被唤醒的进程的进程号
 * @return 返回0表示成功，否则表示失败
 *
 * @since 10
 */
int32_t ResumeSensors(int32_t pid);

/**
 * @brief 查询一个进程打开的所有传感器的信息
 *
 * @param pid 将被查询的进程的进程号
 * @param sensorActiveInfos 返回进程打开的所有传感器信息
 * @param count 返回进程打开的传感器数量
 * @return 返回0表示成功，否则表示失败
 *
 * @since 10
 */
int32_t GetActiveSensorInfos(int32_t pid, SensorActiveInfo **sensorActiveInfos, int32_t *count);

/**
 * @brief 订阅激活的传感器的信息
 *
 * @param callback 回调函数，返回激活的传感器的信息
 * @return 返回0表示成功，否则表示失败
 *
 * @since 10
 */
int32_t Register(SensorActiveInfoCB callback);

/**
 * @brief 取消订阅激活的传感器的信息
 *
 * @param callback 取消对此回调函数的订阅
 * @return 返回0表示成功，否则表示失败
 *
 * @since 10
 */
int32_t Unregister(SensorActiveInfoCB callback);

/**
 * @brief 重置休眠的所有传感器
 *
 * @return 返回0表示成功，否则表示失败
 *
 * @since 10
 */
int32_t ResetSensors();

int32_t InjectMockSensor(int32_t sensorTypeId); // 注入打桩sensor

int32_t UninjectMockSensor(int32_t sensorTypeId); // 删除打桩sensor

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif
#endif /* SENSOR_AGENT_H */
/** @} */