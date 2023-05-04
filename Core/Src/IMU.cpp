//----------------------------------------------------------------------------
//    프로그램명 	:
//
//    만든이     	: Made by Baram ( chcbaram@paran.com )
//
//    날  짜     :
//
//    최종 수정  	:
//
//    MPU_Type	:
//
//    파일명     	: IMU.ino
//----------------------------------------------------------------------------
#include "IMU.h"

unsigned long micros(void)
{
    static unsigned long last_micros = 0;
    unsigned long current_micros = HAL_GetTick() * MICROS_PER_MILLIS + (MICROS_PER_MILLIS - SysTick->VAL / (SystemCoreClock / 1000000UL));
    if(current_micros < last_micros)
    {
        // Handle timer overflow
        current_micros += (0xFFFFFFFFUL / MICROS_PER_MILLIS);
    }
    last_micros = current_micros;
    return current_micros;
}

unsigned long millis() {
  return (unsigned long) HAL_GetTick();
}


/*---------------------------------------------------------------------------
     TITLE   : BLE
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cIMU::cIMU()
{
  uint8_t i;

  for( i=0; i<3; i++ )
  {
    rpy[i] = 0.;

  }
	bConnected = false;
}


/*---------------------------------------------------------------------------
     TITLE   : begin
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
uint8_t cIMU::begin( uint32_t hz )
{
  uint8_t err_code = IMU_OK;
  uint32_t i;
  uint32_t pre_time;

  update_hz = hz;
  update_us = 1000000/hz;

  aRes = 2.0/32768.0;      // 2g

  gRes = 250.0/32768.0;   // 2000dps

  bConnected = SEN.begin();
  
  
  if( bConnected == true )
  {
    filter.begin(update_hz);

    for (i=0; i<32; i++)
    {
      update();
    }

    pre_time = millis();
    while(!SEN.gyro_cali_get_done())
    {
      update();

      if (millis()-pre_time > 5000)
      {
        break;
      }
    }
  }
  
  

	return err_code;
}

/*---------------------------------------------------------------------------
     TITLE   : update
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
uint16_t cIMU::update( uint32_t option )
{
  UNUSED(option);

	uint16_t ret_time = 0;

	static uint32_t tTime;


	if( (micros()-tTime) >= update_us )
	{
		ret_time = micros()-tTime;
    tTime = micros();

    computeIMU();

    gyroData[0] = SEN.gyroADC[0];
    gyroData[1] = SEN.gyroADC[1];
    gyroData[2] = SEN.gyroADC[2];

    gyroRaw[0]  = SEN.gyroRAW[0];
    gyroRaw[1]  = SEN.gyroRAW[1];
    gyroRaw[2]  = SEN.gyroRAW[2];

    accData[0]  = SEN.accADC[0];
    accData[1]  = SEN.accADC[1];
    accData[2]  = SEN.accADC[2];

    accRaw[0]   = SEN.accRAW[0];
    accRaw[1]   = SEN.accRAW[1];
    accRaw[2]   = SEN.accRAW[2];
	}

	return ret_time;
}


#define FILTER_NUM    3

/*---------------------------------------------------------------------------
     TITLE   : compute
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cIMU::computeIMU( void )
{
  static uint32_t prev_process_time = micros();
  static uint32_t cur_process_time = 0;
  static uint32_t process_time = 0;
  uint32_t i;
  static int32_t gyroADC[3][FILTER_NUM] = {0,};
  int32_t gyroAdcSum;

  uint32_t axis;

	SEN.acc_get_adc();
	SEN.gyro_get_adc();

  for (axis = 0; axis < 3; axis++)
  {
    gyroADC[axis][0] = SEN.gyroADC[axis];


    gyroAdcSum = 0;
    for (i=0; i<FILTER_NUM; i++)
    {
      gyroAdcSum += gyroADC[axis][i];
    }
    SEN.gyroADC[axis] = gyroAdcSum/FILTER_NUM;
    for (i=FILTER_NUM-1; i>0; i--)
    {
      gyroADC[axis][i] = gyroADC[axis][i-1];
    }

    if (abs(SEN.gyroADC[axis]) <= 3)
    {
      SEN.gyroADC[axis] = 0;
    }
  }


  for( i=0; i<3; i++ )
  {
    accRaw[i]   = SEN.accRAW[i];
    accData[i]  = SEN.accADC[i];
    gyroRaw[i]  = SEN.gyroRAW[i];
    gyroData[i] = SEN.gyroADC[i];
  }

  ax = (float)SEN.accADC[0]*aRes;
  ay = (float)SEN.accADC[1]*aRes;
  az = (float)SEN.accADC[2]*aRes;

  gx = (float)SEN.gyroADC[0]*gRes;
  gy = (float)SEN.gyroADC[1]*gRes;
  gz = (float)SEN.gyroADC[2]*gRes;

  cur_process_time  = micros();
  process_time      = cur_process_time-prev_process_time;
  prev_process_time = cur_process_time;

  if (SEN.calibratingG == 0 && SEN.calibratingA == 0)
  {
    filter.invSampleFreq = (float)process_time/1000000.0f;
     filter.updateIMU(0, 0, gz, 0, 0, az);
  }


  rpy[0] = filter.getRoll();
  rpy[1] = filter.getPitch();
  rpy[2] = filter.getYaw()-180.;

  quat[0] = filter.q0;
  quat[1] = filter.q1;
  quat[2] = filter.q2;
  quat[3] = filter.q3;

  angle[0] = (int16_t)(rpy[0] * 10.);
  angle[1] = (int16_t)(rpy[1] * 10.);
  angle[2] = (int16_t)(rpy[1] * 1.);

}
