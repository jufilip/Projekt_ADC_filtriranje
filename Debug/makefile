################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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

BUILD_ARTIFACT_NAME := projekt_ADC_filtriranje
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
projekt_ADC_filtriranje.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
projekt_ADC_filtriranje.list \

OBJCOPY_BIN += \
projekt_ADC_filtriranje.bin \


# All Target
all: main-build

# Main-build Target
main-build: projekt_ADC_filtriranje.elf secondary-outputs

# Tool invocations
projekt_ADC_filtriranje.elf: $(OBJS) $(USER_OBJS) C:\Users\juric\STM32CubeIDE\workspace_1.5.0\projekt_ADC_filtriranje\STM32F429ZITX_FLASH.ld
	arm-none-eabi-gcc -o "projekt_ADC_filtriranje.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\juric\STM32CubeIDE\workspace_1.5.0\projekt_ADC_filtriranje\STM32F429ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="projekt_ADC_filtriranje.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

projekt_ADC_filtriranje.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "projekt_ADC_filtriranje.list"
	@echo 'Finished building: $@'
	@echo ' '

projekt_ADC_filtriranje.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "projekt_ADC_filtriranje.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
