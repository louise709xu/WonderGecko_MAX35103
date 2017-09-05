################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1/platform/Device/SiliconLabs/EFM32WG/Source/system_efm32wg.c 

S_UPPER_SRCS += \
C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1/platform/Device/SiliconLabs/EFM32WG/Source/GCC/startup_efm32wg.S 

OBJS += \
./CMSIS/EFM32WG/startup_efm32wg.o \
./CMSIS/EFM32WG/system_efm32wg.o 

C_DEPS += \
./CMSIS/EFM32WG/system_efm32wg.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/EFM32WG/startup_efm32wg.o: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1/platform/Device/SiliconLabs/EFM32WG/Source/GCC/startup_efm32wg.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Assembler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -c -x assembler-with-cpp -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//hardware/kit/common/drivers" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//platform/Device/SiliconLabs/EFM32WG/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//hardware/kit/EFM32WG_STK3800/config" '-DEFM32WG990F256=1' -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/EFM32WG/system_efm32wg.o: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1/platform/Device/SiliconLabs/EFM32WG/Source/system_efm32wg.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFM32WG990F256=1' '-DNDEBUG=1' -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//hardware/kit/common/drivers" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//platform/Device/SiliconLabs/EFM32WG/Include" -I"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//hardware/kit/EFM32WG_STK3800/config" -O3 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"CMSIS/EFM32WG/system_efm32wg.d" -MT"CMSIS/EFM32WG/system_efm32wg.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


