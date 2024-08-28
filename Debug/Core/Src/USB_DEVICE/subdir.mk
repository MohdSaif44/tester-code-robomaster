################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/USB_DEVICE/usb.c \
../Core/Src/USB_DEVICE/usb_device.c \
../Core/Src/USB_DEVICE/usbd_cdc_if.c \
../Core/Src/USB_DEVICE/usbd_conf.c \
../Core/Src/USB_DEVICE/usbd_desc.c 

OBJS += \
./Core/Src/USB_DEVICE/usb.o \
./Core/Src/USB_DEVICE/usb_device.o \
./Core/Src/USB_DEVICE/usbd_cdc_if.o \
./Core/Src/USB_DEVICE/usbd_conf.o \
./Core/Src/USB_DEVICE/usbd_desc.o 

C_DEPS += \
./Core/Src/USB_DEVICE/usb.d \
./Core/Src/USB_DEVICE/usb_device.d \
./Core/Src/USB_DEVICE/usbd_cdc_if.d \
./Core/Src/USB_DEVICE/usbd_conf.d \
./Core/Src/USB_DEVICE/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/USB_DEVICE/%.o Core/Src/USB_DEVICE/%.su Core/Src/USB_DEVICE/%.cyclo: ../Core/Src/USB_DEVICE/%.c Core/Src/USB_DEVICE/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-USB_DEVICE

clean-Core-2f-Src-2f-USB_DEVICE:
	-$(RM) ./Core/Src/USB_DEVICE/usb.cyclo ./Core/Src/USB_DEVICE/usb.d ./Core/Src/USB_DEVICE/usb.o ./Core/Src/USB_DEVICE/usb.su ./Core/Src/USB_DEVICE/usb_device.cyclo ./Core/Src/USB_DEVICE/usb_device.d ./Core/Src/USB_DEVICE/usb_device.o ./Core/Src/USB_DEVICE/usb_device.su ./Core/Src/USB_DEVICE/usbd_cdc_if.cyclo ./Core/Src/USB_DEVICE/usbd_cdc_if.d ./Core/Src/USB_DEVICE/usbd_cdc_if.o ./Core/Src/USB_DEVICE/usbd_cdc_if.su ./Core/Src/USB_DEVICE/usbd_conf.cyclo ./Core/Src/USB_DEVICE/usbd_conf.d ./Core/Src/USB_DEVICE/usbd_conf.o ./Core/Src/USB_DEVICE/usbd_conf.su ./Core/Src/USB_DEVICE/usbd_desc.cyclo ./Core/Src/USB_DEVICE/usbd_desc.d ./Core/Src/USB_DEVICE/usbd_desc.o ./Core/Src/USB_DEVICE/usbd_desc.su

.PHONY: clean-Core-2f-Src-2f-USB_DEVICE

