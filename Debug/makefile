################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/VESC_CAN/subdir.mk
-include Core/Src/USB_DEVICE/subdir.mk
-include Core/Src/SPI/subdir.mk
-include Core/Src/SERVO/subdir.mk
-include Core/Src/Robomaster/subdir.mk
-include Core/Src/RNS_interface/subdir.mk
-include Core/Src/PSx_Interface/subdir.mk
-include Core/Src/PID/subdir.mk
-include Core/Src/Odrive/subdir.mk
-include Core/Src/MODN/subdir.mk
-include Core/Src/MAIN/subdir.mk
-include Core/Src/LASER/subdir.mk
-include Core/Src/KF/subdir.mk
-include Core/Src/IMU/subdir.mk
-include Core/Src/I2C/subdir.mk
-include Core/Src/Faulhaber/subdir.mk
-include Core/Src/CAN/subdir.mk
-include Core/Src/BIOS/subdir.mk
-include Core/Src/ADC/subdir.mk
-include Core/Src/ABT/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := mainboard-usb2
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
mainboard-usb2.elf \

MAP_FILES += \
mainboard-usb2.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
mainboard-usb2.list \


# All Target
all: main-build

# Main-build Target
main-build: mainboard-usb2.elf secondary-outputs

# Tool invocations
mainboard-usb2.elf mainboard-usb2.map: $(OBJS) $(USER_OBJS) /home/saif/Desktop/STM32/tester_code-master/mainboard-usb\ (2)/mainboard-usb/STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "mainboard-usb2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/saif/Desktop/STM32/tester_code-master/mainboard-usb (2)/mainboard-usb/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="mainboard-usb2.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

mainboard-usb2.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "mainboard-usb2.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout mainboard-usb2.elf mainboard-usb2.list mainboard-usb2.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
