#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "IMU.h"

class AGVSensor
{
 public:
  AGVSensor();
  ~AGVSensor();

  bool init(void);

  // IMU
  void initIMU(void);
  sensor_msgs::Imu getIMU(void);
  void updateIMU(void);
  void calibrationGyro(void);

  float* getOrientation(void);

 private:
  sensor_msgs::Imu           imu_msg_;
  cIMU imu_;
};
