/** 
    @file  : MPU9250.c
		@author: Nguyen Ngoc hau
**/

#include "MPU9250.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_gpio.h"

#define MPU_CALI_COUNT      512

#define ROLL		0
#define PITCH		1
#define YAW			2

#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  = -X; accADC[ROLL]  =  Y; accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

//#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  =  Y; accADC[ROLL]  =  X; accADC[YAW]  =   Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}


#define MPU9250_SPI			hspi1
#define	MPU9250_CS_GPIO		GPIOC
#define	MPU9250_CS_PIN		GPIO_PIN_8

extern SPI_HandleTypeDef hspi1;

const uint8_t READWRITE_CMD = 0x80;
const uint8_t DUMMY_BYTE = 0x00;

void MPU9250_Activate()
{
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

void MPU9250_Deactivate()
{
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		return -1;
	}
	else
	{

	}
	return receivedbyte;
}
float checkWhoAMI;
void MPU_SPI_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	MPU9250_Activate();
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite>=0x01)
	{
		SPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	MPU9250_Deactivate();
}

void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	MPU9250_Activate();
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_SPI_Transmit(&MPU9250_SPI, &data, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&MPU9250_SPI, pBuffer, NumByteToRead, HAL_MAX_DELAY);
	MPU9250_Deactivate();
}

/* writes a byte to MPU9250 register given a register address and data */
void writeRegister(uint8_t subAddress, uint8_t data)
{
	MPU_SPI_Write(&data, subAddress, 1);
	HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
	MPU_SPI_Read(dest, subAddress, count);
}


uint8_t buff;
/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
uint8_t whoAmI(){
	// read the WHO AM I register
	readRegisters(WHO_AM_I,1,&buff);

	// return the register value
	return buff;
}

/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range)
{
	writeRegister(ACCEL_CONFIG, range);
}

/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range)
{
	writeRegister(GYRO_CONFIG, range);
}

/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth)
{
	writeRegister(ACCEL_CONFIG2,bandwidth);
	writeRegister(CONFIG,bandwidth);
}

/*---------------------------------------------------------------------------
     TITLE   : cMPU9250
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cMPU9250::cMPU9250()
{
	calibratingG = 0;
	calibratingA = 0;
	bConnected   = false;
}

/*---------------------------------------------------------------------------
     TITLE   : cMPU9250
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
uint8_t result;

bool cMPU9250::begin()
{
	  // select clock source to gyro
	  	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
	  	// enable I2C master mode
	  	writeRegister(USER_CTRL, I2C_MST_EN);
	  	// set the I2C bus speed to 400 kHz
	  	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);
	  	// reset the MPU9250
	  	writeRegister(PWR_MGMNT_1,PWR_RESET);
	  	// wait for MPU-9250 to come back up
	  	HAL_Delay(10);
	  	// select clock source to gyro
	  	writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

	  	// enable accelerometer and gyro
	  	writeRegister(PWR_MGMNT_2,SEN_ENABLE);

	  	// setting accel range to 16G as default
	  	writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G);

	  	// setting the gyro range to 2000DPS as default
	  	writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS);

	  	// setting bandwidth to 184Hz as default
	  	writeRegister(ACCEL_CONFIG2,DLPF_184);

	  	// setting gyro bandwidth to 184Hz
	  	writeRegister(CONFIG,DLPF_184);

	  	// setting the sample rate divider to 0 as default
	  	writeRegister(SMPDIV,0x00);

	  	// enable I2C master mode
	  	writeRegister(USER_CTRL,I2C_MST_EN);

	  	// set the I2C bus speed to 400 kHz
	  	writeRegister(I2C_MST_CTRL,I2C_MST_CLK);

	  	HAL_Delay(100); // long wait between AK8963 mode changes

	  	// select clock source to gyro
	  	writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

			// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
		  uint8_t who = whoAmI();
		  result = who;
		  if(who == 0x70 || who == 0x71)
		  {
			bConnected = true;
			init();
			gyro_init();
			acc_init();
		  }
		  return bConnected;
}

void cMPU9250::init( void )
{

}

/*---------------------------------------------------------------------------
     TITLE   : gyro_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_init( void )
{
  uint8_t i;

  for( i=0; i<3; i++ )
  {
    gyroADC[i]  = 0;
	gyroZero[i] = 0;
	gyroRAW[i]  = 0;
  }
  calibratingG = MPU_CALI_COUNT;
}

/*---------------------------------------------------------------------------
     TITLE   : gyro_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_get_adc( void )
{
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;

  uint8_t rawADC[6];

  if( bConnected == true )
  {
	readRegisters(GYRO_OUT, 6, rawADC);
 	x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
  	y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
  	z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

  	gyroRAW[0] = x;
  	gyroRAW[1] = y;
  	gyroRAW[2] = z;

  	GYRO_ORIENTATION( x, y,z );
  }

  gyro_common();
}


/*---------------------------------------------------------------------------
     TITLE   : gyro_cali_start
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_cali_start()
{
  calibratingG = MPU_CALI_COUNT;
  calibratingA = MPU_CALI_COUNT;
}

/*---------------------------------------------------------------------------
     TITLE   : gyro_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_common()
{
	static int16_t previousGyroADC[3];
	static int32_t g[3];
	uint8_t axis, tilt=0;

	memset(previousGyroADC, 0, 3 * sizeof(int));


	if (calibratingG>0)
	{
		for (axis = 0; axis < 3; axis++)
		{
			if (calibratingG == MPU_CALI_COUNT)
			{ // Reset g[axis] at start of calibration
				g[axis]=0;
				previousGyroADC[axis] = gyroADC[axis];
			}
			if (calibratingG % 10 == 0)
			{
				//if(abs(gyroADC[axis] - previousGyroADC[axis]) > 8) tilt=1;

				previousGyroADC[axis] = gyroADC[axis];
			}
			g[axis] += gyroADC[axis]; // Sum up 512 readings
			gyroZero[axis]=g[axis]>>9;

			if (calibratingG == 1)
			{
				//SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_ELSE);
			}
		}

		if(tilt)
		{
			calibratingG=1000;
		}
		else
		{
			calibratingG--;
		}
		return;
	}


  for (axis = 0; axis < 3; axis++)
  {
    gyroADC[axis] -= gyroZero[axis];

    //anti gyro glitch, limit the variation between two consecutive readings
    //gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
    previousGyroADC[axis] = gyroADC[axis];
  }
}

/*---------------------------------------------------------------------------
     TITLE   : acc_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_init( void )
{
  uint8_t i;

  for( i=0; i<3; i++ )
  {
    accADC[i]   = 0;
    accZero[i]  = 0;
    accRAW[i]   = 0;
  }
}

/*---------------------------------------------------------------------------
     TITLE   : acc_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_get_adc( void )
{
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  uint8_t rawADC[6];

  if( bConnected == true )
  {
	readRegisters(ACCEL_OUT, 6, rawADC);

    x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
    y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
    z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

    accRAW[0] = x;
    accRAW[1] = y;
    accRAW[2] = z;
	
	ACC_ORIENTATION( x,	y, z );
  }

	acc_common();
}

/*---------------------------------------------------------------------------
     TITLE   : acc_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_common()
{
	static int32_t a[3];

	if (calibratingA>0)
	{
		calibratingA--;
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			if (calibratingA ==(MPU_CALI_COUNT-1)) a[axis]=0;  // Reset a[axis] at start of calibration
			a[axis] += accADC[axis];             // Sum up 512 readings
			accZero[axis] = a[axis]>>9;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
		}
		if (calibratingA == 0)
		{
			//accZero[YAW] -= ACC_1G;
      accZero[YAW] = 0;
		}
	}

  accADC[ROLL]  -=  accZero[ROLL] ;
  accADC[PITCH] -=  accZero[PITCH];
  accADC[YAW]   -=  accZero[YAW] ;
}

/*---------------------------------------------------------------------------
     TITLE   : acc_cali_start
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_cali_start()
{
	calibratingA = MPU_CALI_COUNT;
}

/*---------------------------------------------------------------------------
     TITLE   : acc_cali_get_done
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU9250::acc_cali_get_done()
{
	if( calibratingA == 0 ) return true;
	else                    return false;
}

/*---------------------------------------------------------------------------
     TITLE   : gyro_cali_get_done
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU9250::gyro_cali_get_done()
{
	if( calibratingG == 0 ) return true;
	else                    return false;
}

