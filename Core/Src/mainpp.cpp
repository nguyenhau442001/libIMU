/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 *  Added and Modified by: Nguyen Ngoc Hau ,cording start date: 10/10/2022, update:4/5/2023
 */

#include "Service_robot_config.h"

const float PI = 3.14159265359;

char imu_frame_id[30]="imu_link"; // name of imu frame

AGVSensor sensors;

ros::NodeHandle nh;

extern I2C_HandleTypeDef hi2c1;
uint32_t TxCpltCallback,RxCpltCallback;

 /*******************************************************************************
 * Publisher
 *******************************************************************************/
 // String variable
 std_msgs::String str_msg;
 ros::Publisher chatter("chatter", &str_msg);
 char hello[] = "Service robot";

 // IMU of Morning B
 sensor_msgs::Imu imu_msg;
 ros::Publisher imu_pub("imu", &imu_msg);


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
  TxCpltCallback++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
  RxCpltCallback++;
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}


/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation, type of messages sensor_msgs/Imu
*  Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
*******************************************************************************/
void publishImuMsg(void)
{
	  imu_msg = sensors.getIMU();

	  imu_msg.header.stamp    = rosNow();
	  imu_msg.header.frame_id = imu_frame_id;

	  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;

  if (isConnected)
  {
    if (variable_flag == false)
    {
      sensors.initIMU();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

void setup(void)
{
	// Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(imu_pub);   // name of publisher node

  // Setting for IMU
  sensors.init();
}

static uint32_t tTime[2];
uint32_t count;

void loop(void)
{

//	yaw=getYaw(0,0,0.366939217,0.945571899);
  count++;
  uint32_t t = HAL_GetTick();
  updateVariable(nh.connected());
  // Publish IMU data at 200Hz
  if ((t - tTime[0]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

    publishImuMsg();
//    chatter.publish(&str_msg);
    tTime[0] = t;
  }
  // Update the IMU unit
  sensors.updateIMU();
  // Update gyro calibration
  updateGyroCali(nh.connected());

  // Process incoming ROS messages
  nh.spinOnce();

}
