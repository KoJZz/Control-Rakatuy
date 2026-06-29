################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/SimpleFOC_Math/foc_utils.cpp \
../Core/Src/SimpleFOC_Math/pid.cpp 

OBJS += \
./Core/Src/SimpleFOC_Math/foc_utils.o \
./Core/Src/SimpleFOC_Math/pid.o 

CPP_DEPS += \
./Core/Src/SimpleFOC_Math/foc_utils.d \
./Core/Src/SimpleFOC_Math/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/SimpleFOC_Math/%.o Core/Src/SimpleFOC_Math/%.su Core/Src/SimpleFOC_Math/%.cyclo: ../Core/Src/SimpleFOC_Math/%.cpp Core/Src/SimpleFOC_Math/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-SimpleFOC_Math

clean-Core-2f-Src-2f-SimpleFOC_Math:
	-$(RM) ./Core/Src/SimpleFOC_Math/foc_utils.cyclo ./Core/Src/SimpleFOC_Math/foc_utils.d ./Core/Src/SimpleFOC_Math/foc_utils.o ./Core/Src/SimpleFOC_Math/foc_utils.su ./Core/Src/SimpleFOC_Math/pid.cyclo ./Core/Src/SimpleFOC_Math/pid.d ./Core/Src/SimpleFOC_Math/pid.o ./Core/Src/SimpleFOC_Math/pid.su

.PHONY: clean-Core-2f-Src-2f-SimpleFOC_Math

