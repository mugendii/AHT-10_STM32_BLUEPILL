################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CPP/ONBOARDLED.cpp 

OBJS += \
./CPP/ONBOARDLED.o 

CPP_DEPS += \
./CPP/ONBOARDLED.d 


# Each subdirectory must supply rules for building sources it contributes
CPP/%.o CPP/%.su CPP/%.cyclo: ../CPP/%.cpp CPP/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-CPP

clean-CPP:
	-$(RM) ./CPP/ONBOARDLED.cyclo ./CPP/ONBOARDLED.d ./CPP/ONBOARDLED.o ./CPP/ONBOARDLED.su

.PHONY: clean-CPP

