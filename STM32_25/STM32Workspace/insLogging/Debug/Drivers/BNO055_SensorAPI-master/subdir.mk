################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/STM32Workspace/BNO055_SensorAPI-master/bno055.c \
D:/STM32Workspace/BNO055_SensorAPI-master/bno055_support.c 

C_DEPS += \
./Drivers/BNO055_SensorAPI-master/bno055.d \
./Drivers/BNO055_SensorAPI-master/bno055_support.d 

OBJS += \
./Drivers/BNO055_SensorAPI-master/bno055.o \
./Drivers/BNO055_SensorAPI-master/bno055_support.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BNO055_SensorAPI-master/bno055.o: D:/STM32Workspace/BNO055_SensorAPI-master/bno055.c Drivers/BNO055_SensorAPI-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BNO055_SensorAPI-master/bno055_support.o: D:/STM32Workspace/BNO055_SensorAPI-master/bno055_support.c Drivers/BNO055_SensorAPI-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BNO055_SensorAPI-2d-master

clean-Drivers-2f-BNO055_SensorAPI-2d-master:
	-$(RM) ./Drivers/BNO055_SensorAPI-master/bno055.cyclo ./Drivers/BNO055_SensorAPI-master/bno055.d ./Drivers/BNO055_SensorAPI-master/bno055.o ./Drivers/BNO055_SensorAPI-master/bno055.su ./Drivers/BNO055_SensorAPI-master/bno055_support.cyclo ./Drivers/BNO055_SensorAPI-master/bno055_support.d ./Drivers/BNO055_SensorAPI-master/bno055_support.o ./Drivers/BNO055_SensorAPI-master/bno055_support.su

.PHONY: clean-Drivers-2f-BNO055_SensorAPI-2d-master

