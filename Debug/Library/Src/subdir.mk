################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library/Src/Lcd.c \
../Library/Src/Lora.c \
../Library/Src/modbus.c 

OBJS += \
./Library/Src/Lcd.o \
./Library/Src/Lora.o \
./Library/Src/modbus.o 

C_DEPS += \
./Library/Src/Lcd.d \
./Library/Src/Lora.d \
./Library/Src/modbus.d 


# Each subdirectory must supply rules for building sources it contributes
Library/Src/%.o Library/Src/%.su: ../Library/Src/%.c Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"D:/CudeIDE/Doantotnghiep/Modbbus/Library/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Library-2f-Src

clean-Library-2f-Src:
	-$(RM) ./Library/Src/Lcd.d ./Library/Src/Lcd.o ./Library/Src/Lcd.su ./Library/Src/Lora.d ./Library/Src/Lora.o ./Library/Src/Lora.su ./Library/Src/modbus.d ./Library/Src/modbus.o ./Library/Src/modbus.su

.PHONY: clean-Library-2f-Src

