################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/Src/LCD.c \
../Components/Src/encoder.c \
../Components/Src/encoder_config.c \
../Components/Src/extras.c \
../Components/Src/lcd_config.c \
../Components/Src/servo.c 

OBJS += \
./Components/Src/LCD.o \
./Components/Src/encoder.o \
./Components/Src/encoder_config.o \
./Components/Src/extras.o \
./Components/Src/lcd_config.o \
./Components/Src/servo.o 

C_DEPS += \
./Components/Src/LCD.d \
./Components/Src/encoder.d \
./Components/Src/encoder_config.d \
./Components/Src/extras.d \
./Components/Src/lcd_config.d \
./Components/Src/servo.d 


# Each subdirectory must supply rules for building sources it contributes
Components/Src/%.o Components/Src/%.su Components/Src/%.cyclo: ../Components/Src/%.c Components/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I"D:/github/flyingrobots/Flying-Probe-Tester/Flying-Probe_tester/Components/Inc" -IC:/Users/yazed/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.2/Drivers/STM32F7xx_HAL_Driver/Inc -IC:/Users/yazed/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.2/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -IC:/Users/yazed/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.2/Drivers/CMSIS/Device/ST/STM32F7xx/Include -IC:/Users/yazed/STM32Cube/Repository/STM32Cube_FW_F7_V1.17.2/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-Src

clean-Components-2f-Src:
	-$(RM) ./Components/Src/LCD.cyclo ./Components/Src/LCD.d ./Components/Src/LCD.o ./Components/Src/LCD.su ./Components/Src/encoder.cyclo ./Components/Src/encoder.d ./Components/Src/encoder.o ./Components/Src/encoder.su ./Components/Src/encoder_config.cyclo ./Components/Src/encoder_config.d ./Components/Src/encoder_config.o ./Components/Src/encoder_config.su ./Components/Src/extras.cyclo ./Components/Src/extras.d ./Components/Src/extras.o ./Components/Src/extras.su ./Components/Src/lcd_config.cyclo ./Components/Src/lcd_config.d ./Components/Src/lcd_config.o ./Components/Src/lcd_config.su ./Components/Src/servo.cyclo ./Components/Src/servo.d ./Components/Src/servo.o ./Components/Src/servo.su

.PHONY: clean-Components-2f-Src
