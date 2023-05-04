################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

CPP_SRCS += \
../Core/Src/AGV_sensor.cpp \
../Core/Src/IMU.cpp \
../Core/Src/MPU9250.cpp \
../Core/Src/MadgwickAHRS.cpp \
../Core/Src/duration.cpp \
../Core/Src/main.cpp \
../Core/Src/mainpp.cpp \
../Core/Src/time.cpp 

C_DEPS += \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/AGV_sensor.o \
./Core/Src/IMU.o \
./Core/Src/MPU9250.o \
./Core/Src/MadgwickAHRS.o \
./Core/Src/duration.o \
./Core/Src/main.o \
./Core/Src/mainpp.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/time.o 

CPP_DEPS += \
./Core/Src/AGV_sensor.d \
./Core/Src/IMU.d \
./Core/Src/MPU9250.d \
./Core/Src/MadgwickAHRS.d \
./Core/Src/duration.d \
./Core/Src/main.d \
./Core/Src/mainpp.d \
./Core/Src/time.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AGV_sensor.cyclo ./Core/Src/AGV_sensor.d ./Core/Src/AGV_sensor.o ./Core/Src/AGV_sensor.su ./Core/Src/IMU.cyclo ./Core/Src/IMU.d ./Core/Src/IMU.o ./Core/Src/IMU.su ./Core/Src/MPU9250.cyclo ./Core/Src/MPU9250.d ./Core/Src/MPU9250.o ./Core/Src/MPU9250.su ./Core/Src/MadgwickAHRS.cyclo ./Core/Src/MadgwickAHRS.d ./Core/Src/MadgwickAHRS.o ./Core/Src/MadgwickAHRS.su ./Core/Src/duration.cyclo ./Core/Src/duration.d ./Core/Src/duration.o ./Core/Src/duration.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mainpp.cyclo ./Core/Src/mainpp.d ./Core/Src/mainpp.o ./Core/Src/mainpp.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/time.cyclo ./Core/Src/time.d ./Core/Src/time.o ./Core/Src/time.su

.PHONY: clean-Core-2f-Src

