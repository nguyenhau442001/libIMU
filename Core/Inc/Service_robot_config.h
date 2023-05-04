/*
 * Service_robot_config.h
 *
 *  Created on: Oct 16, 2022
 *      Author: Nguyen Ngoc Hau
 */

#ifndef Service_Robot_Config_H_
#define Service_Robot_Config_H_

#ifdef __cplusplus

 extern "C" {
#endif

void setup(void);
void loop(void);
#ifdef __cplusplus
}
#endif

// ROS LIBRARY
#include <ros.h>
#include <time.h>
#include "ros/duration.h"
#include <stdint.h>
#include "ros/time.h"
#include <std_msgs/String.h>  		 // library for /chatter topic
#include <sensor_msgs/Imu.h>  		 // library for /imu topic

// NON-ROS LIBRARY
#include "stm32f4xx_hal_i2c.h"
#include "math.h"
#include "stdio.h"
#include "AGV_sensor.h"
 /******************************************************************************************************************************************
  ************************************************************ DEFINE VARIABLE*********************************************************
  *****************************************************************************************************************************************/
#define FIRMWARE_VER "1.0.0"
#define IMU_PUBLISH_FREQUENCY                  34  //hz


// Function prototypes
void publishImuMsg(void);

ros::Time rosNow(void);

#endif /* Service_Robot_Config_H_ */
