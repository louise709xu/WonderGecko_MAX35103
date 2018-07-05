################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include CMSIS/EFM32WG/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: WonderGecko_MAX35103.axf

# Tool invocations
WonderGecko_MAX35103.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "WonderGecko_MAX35103.ld" -Xlinker --gc-sections -Xlinker -Map="WonderGecko_MAX35103.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o WonderGecko_MAX35103.axf "./src/main.o" "./emlib/dmactrl.o" "./emlib/dmadrv.o" "./emlib/em_cmu.o" "./emlib/em_core.o" "./emlib/em_dma.o" "./emlib/em_emu.o" "./emlib/em_gpio.o" "./emlib/em_rtc.o" "./emlib/em_system.o" "./emlib/em_usart.o" "./emlib/rtcdriver.o" "./emlib/spidrv.o" "./emlib/uartdrv.o" "./CMSIS/EFM32WG/startup_efm32wg.o" "./CMSIS/EFM32WG/system_efm32wg.o" "./emlib/em_leuart.o" "./emlib/gpiointerrupt.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: WonderGecko_MAX35103.hex'
	arm-none-eabi-objcopy -O ihex "WonderGecko_MAX35103.axf" "WonderGecko_MAX35103.hex"
	@echo ' '

	@echo 'Building bin file: WonderGecko_MAX35103.bin'
	arm-none-eabi-objcopy -O binary "WonderGecko_MAX35103.axf" "WonderGecko_MAX35103.bin"
	@echo ' '

	@echo 'Building s37 file: WonderGecko_MAX35103.s37'
	arm-none-eabi-objcopy -O srec "WonderGecko_MAX35103.axf" "WonderGecko_MAX35103.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "WonderGecko_MAX35103.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) WonderGecko_MAX35103.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
