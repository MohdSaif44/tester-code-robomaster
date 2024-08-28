################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Faulhaber/FH.c \
../Core/Src/Faulhaber/Faulhaber_Interface.c \
../Core/Src/Faulhaber/ObjectDictionary.c 

OBJS += \
./Core/Src/Faulhaber/FH.o \
./Core/Src/Faulhaber/Faulhaber_Interface.o \
./Core/Src/Faulhaber/ObjectDictionary.o 

C_DEPS += \
./Core/Src/Faulhaber/FH.d \
./Core/Src/Faulhaber/Faulhaber_Interface.d \
./Core/Src/Faulhaber/ObjectDictionary.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Faulhaber/%.o Core/Src/Faulhaber/%.su Core/Src/Faulhaber/%.cyclo: ../Core/Src/Faulhaber/%.c Core/Src/Faulhaber/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Faulhaber

clean-Core-2f-Src-2f-Faulhaber:
	-$(RM) ./Core/Src/Faulhaber/FH.cyclo ./Core/Src/Faulhaber/FH.d ./Core/Src/Faulhaber/FH.o ./Core/Src/Faulhaber/FH.su ./Core/Src/Faulhaber/Faulhaber_Interface.cyclo ./Core/Src/Faulhaber/Faulhaber_Interface.d ./Core/Src/Faulhaber/Faulhaber_Interface.o ./Core/Src/Faulhaber/Faulhaber_Interface.su ./Core/Src/Faulhaber/ObjectDictionary.cyclo ./Core/Src/Faulhaber/ObjectDictionary.d ./Core/Src/Faulhaber/ObjectDictionary.o ./Core/Src/Faulhaber/ObjectDictionary.su

.PHONY: clean-Core-2f-Src-2f-Faulhaber

