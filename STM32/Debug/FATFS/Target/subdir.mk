################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FATFS/Target/user_diskio.c \
../FATFS/Target/user_diskio_spi.c 

OBJS += \
./FATFS/Target/user_diskio.o \
./FATFS/Target/user_diskio_spi.o 

C_DEPS += \
./FATFS/Target/user_diskio.d \
./FATFS/Target/user_diskio_spi.d 


# Each subdirectory must supply rules for building sources it contributes
FATFS/Target/%.o FATFS/Target/%.su FATFS/Target/%.cyclo: ../FATFS/Target/%.c FATFS/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I"D:/github/flyingrobots/Flying-Probe-Tester/STM32/Components/Inc" -IC:/Users/yazed/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.2/Drivers/STM32F7xx_HAL_Driver/Inc -IC:/Users/yazed/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.2/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -IC:/Users/yazed/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.2/Drivers/CMSIS/Device/ST/STM32F7xx/Include -IC:/Users/yazed/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.2/Drivers/CMSIS/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FATFS-2f-Target

clean-FATFS-2f-Target:
	-$(RM) ./FATFS/Target/user_diskio.cyclo ./FATFS/Target/user_diskio.d ./FATFS/Target/user_diskio.o ./FATFS/Target/user_diskio.su ./FATFS/Target/user_diskio_spi.cyclo ./FATFS/Target/user_diskio_spi.d ./FATFS/Target/user_diskio_spi.o ./FATFS/Target/user_diskio_spi.su

.PHONY: clean-FATFS-2f-Target

