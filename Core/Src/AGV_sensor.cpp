/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
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
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert */
/* Maintainer: Indruino */

#include "AGV_sensor.h"

AGVSensor::AGVSensor()
{
}

AGVSensor::~AGVSensor()
{
}


bool AGVSensor::init(void)
{
  uint8_t get_error_code = 0x00;
  get_error_code = imu_.begin();

  return get_error_code;
}

void AGVSensor::initIMU(void)
{
  imu_.begin();
}

void AGVSensor::updateIMU(void)
{
  imu_.update();
}

void AGVSensor::calibrationGyro()
{
  uint32_t pre_time;
  uint32_t t_time;

  imu_.SEN.gyro_cali_start();

  t_time = millis();
  pre_time = millis();

  while (!imu_.SEN.gyro_cali_get_done())
  {
    imu_.update();

    if (millis() - pre_time > 5000)
    {
      break;
    }
    if (millis() - t_time > 100)
    {
      t_time = millis();
    }
  }
}

sensor_msgs::Imu AGVSensor::getIMU(void)
{
  imu_msg_.angular_velocity.x = (imu_.SEN.gyroADC[0] * GYRO_FACTOR);
  imu_msg_.angular_velocity.y = (imu_.SEN.gyroADC[1] * GYRO_FACTOR);
  imu_msg_.angular_velocity.z = (imu_.SEN.gyroADC[2] * GYRO_FACTOR);
  imu_msg_.angular_velocity_covariance[0] = 0.02;
  imu_msg_.angular_velocity_covariance[1] = 0;
  imu_msg_.angular_velocity_covariance[2] = 0;
  imu_msg_.angular_velocity_covariance[3] = 0;
  imu_msg_.angular_velocity_covariance[4] = 0.02;
  imu_msg_.angular_velocity_covariance[5] = 0;
  imu_msg_.angular_velocity_covariance[6] = 0;
  imu_msg_.angular_velocity_covariance[7] = 0;
  imu_msg_.angular_velocity_covariance[8] = 0.02;

  imu_msg_.linear_acceleration.x = imu_.SEN.accADC[0] * ACCEL_FACTOR;
  imu_msg_.linear_acceleration.y = imu_.SEN.accADC[1] * ACCEL_FACTOR;
  imu_msg_.linear_acceleration.z = imu_.SEN.accADC[2] * ACCEL_FACTOR;

  imu_msg_.linear_acceleration_covariance[0] = 0.04;
  imu_msg_.linear_acceleration_covariance[1] = 0;
  imu_msg_.linear_acceleration_covariance[2] = 0;
  imu_msg_.linear_acceleration_covariance[3] = 0;
  imu_msg_.linear_acceleration_covariance[4] = 0.04;
  imu_msg_.linear_acceleration_covariance[5] = 0;
  imu_msg_.linear_acceleration_covariance[6] = 0;
  imu_msg_.linear_acceleration_covariance[7] = 0;
  imu_msg_.linear_acceleration_covariance[8] = 0.04;

  imu_msg_.orientation.w = imu_.quat[0];
  imu_msg_.orientation.x = imu_.quat[1];
  imu_msg_.orientation.y = imu_.quat[2];
  imu_msg_.orientation.z = imu_.quat[3];

  imu_msg_.orientation_covariance[0] = 0.0025;
  imu_msg_.orientation_covariance[1] = 0;
  imu_msg_.orientation_covariance[2] = 0;
  imu_msg_.orientation_covariance[3] = 0;
  imu_msg_.orientation_covariance[4] = 0.0025;
  imu_msg_.orientation_covariance[5] = 0;
  imu_msg_.orientation_covariance[6] = 0;
  imu_msg_.orientation_covariance[7] = 0;
  imu_msg_.orientation_covariance[8] = 0.0025;

  return imu_msg_;
}

float *AGVSensor::getOrientation(void)
{
  static float orientation[4];

  orientation[0] = imu_.quat[0];
  orientation[1] = imu_.quat[1];
  orientation[2] = imu_.quat[2];
  orientation[3] = imu_.quat[3];

  return orientation;
}





