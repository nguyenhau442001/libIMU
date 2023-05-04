//----------------------------------------------------------------------------
//    프로그램명 	: IMU
//
//    만든이     	: Made by Baram ( chcbaram@paran.com )
//
//    날  짜     :
//
//    최종 수정  	:
//
//    MPU_Type	:
//
//    파일명     	: IMU.h
//----------------------------------------------------------------------------
#ifndef _IMU_H_
#define _IMU_H_

#include "MPU9250.h"
#include "MadgwickAHRS.h"

#define IMU_OK			  0x00
#define IMU_ERR_I2C		  0x01

#define ACCEL_FACTOR                      0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
                                                           //                                             Scale : +- 16384
#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
                                                           //                                             Scale : +- 16.4[deg/s]

const unsigned long MICROS_PER_MILLIS = 1000UL;
unsigned long micros(void);
unsigned long millis();

class cIMU
{
public:
  cMPU9250 SEN;
  
  int16_t angle[3];
  float   rpy[3];
  float   quat[4];
  int16_t gyroData[3];
  int16_t gyroRaw[3];
  int16_t accData[3];
  int16_t accRaw[3];

  float ax, ay, az;
  float gx, gy, gz;

	bool bConnected;

  float aRes;
  float gRes;

public:
	cIMU();

	uint8_t  begin( uint32_t hz = 34 );
	uint16_t update( uint32_t option = 0 );

private:
  Madgwick filter;
  uint32_t update_hz;
  uint32_t update_us;

	void computeIMU( void );

};


#endif
