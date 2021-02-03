################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
PPM_standalone.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
PPM_standalone.list \

OBJCOPY_BIN += \
PPM_standalone.bin \


# All Target
all: PPM_standalone.elf secondary-outputs

# Tool invocations
PPM_standalone.elf: $(OBJS) $(USER_OBJS) C:\Users\OBU\OneDrive\Documents\CVUT\ING\Diplomka\ProtonPrecessionMagnetometer\SW\STM32_PPM\PPM_standalone\STM32F767VITX_FLASH.ld
	arm-none-eabi-gcc -o "PPM_standalone.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\OBU\OneDrive\Documents\CVUT\ING\Diplomka\ProtonPrecessionMagnetometer\SW\STM32_PPM\PPM_standalone\STM32F767VITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="PPM_standalone.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

PPM_standalone.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "PPM_standalone.list"
	@echo 'Finished building: $@'
	@echo ' '

PPM_standalone.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "PPM_standalone.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
