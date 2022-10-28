################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MPU9250/inv_mpu.c \
../Core/Src/MPU9250/inv_mpu_dmp_motion_driver.c 

CPP_SRCS += \
../Core/Src/MPU9250/MPU9250-DMP.cpp 

C_DEPS += \
./Core/Src/MPU9250/inv_mpu.d \
./Core/Src/MPU9250/inv_mpu_dmp_motion_driver.d 

OBJS += \
./Core/Src/MPU9250/MPU9250-DMP.o \
./Core/Src/MPU9250/inv_mpu.o \
./Core/Src/MPU9250/inv_mpu_dmp_motion_driver.o 

CPP_DEPS += \
./Core/Src/MPU9250/MPU9250-DMP.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/MPU9250/%.o: ../Core/Src/MPU9250/%.cpp Core/Src/MPU9250/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/MPU9250/%.o: ../Core/Src/MPU9250/%.c Core/Src/MPU9250/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-MPU9250

clean-Core-2f-Src-2f-MPU9250:
	-$(RM) ./Core/Src/MPU9250/MPU9250-DMP.d ./Core/Src/MPU9250/MPU9250-DMP.o ./Core/Src/MPU9250/inv_mpu.d ./Core/Src/MPU9250/inv_mpu.o ./Core/Src/MPU9250/inv_mpu_dmp_motion_driver.d ./Core/Src/MPU9250/inv_mpu_dmp_motion_driver.o

.PHONY: clean-Core-2f-Src-2f-MPU9250

