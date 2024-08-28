################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Robomaster/Robomaster.c 

OBJS += \
./Core/Src/Robomaster/Robomaster.o 

C_DEPS += \
./Core/Src/Robomaster/Robomaster.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Robomaster/%.o Core/Src/Robomaster/%.su Core/Src/Robomaster/%.cyclo: ../Core/Src/Robomaster/%.c Core/Src/Robomaster/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Robomaster

clean-Core-2f-Src-2f-Robomaster:
	-$(RM) ./Core/Src/Robomaster/Robomaster.cyclo ./Core/Src/Robomaster/Robomaster.d ./Core/Src/Robomaster/Robomaster.o ./Core/Src/Robomaster/Robomaster.su

.PHONY: clean-Core-2f-Src-2f-Robomaster

