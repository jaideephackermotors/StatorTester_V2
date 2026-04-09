################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/communication.c \
../Src/freertos.c \
../Src/globalsw.c \
../Src/hlpfunctions.c \
../Src/main.c \
../Src/platform_config.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_hal_timebase_TIM.c \
../Src/stm32f4xx_it.c \
../Src/str2dbl.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/communication.o \
./Src/freertos.o \
./Src/globalsw.o \
./Src/hlpfunctions.o \
./Src/main.o \
./Src/platform_config.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_hal_timebase_TIM.o \
./Src/stm32f4xx_it.o \
./Src/str2dbl.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/communication.d \
./Src/freertos.d \
./Src/globalsw.d \
./Src/hlpfunctions.d \
./Src/main.d \
./Src/platform_config.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_hal_timebase_TIM.d \
./Src/stm32f4xx_it.d \
./Src/str2dbl.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/communication.cyclo ./Src/communication.d ./Src/communication.o ./Src/communication.su ./Src/freertos.cyclo ./Src/freertos.d ./Src/freertos.o ./Src/freertos.su ./Src/globalsw.cyclo ./Src/globalsw.d ./Src/globalsw.o ./Src/globalsw.su ./Src/hlpfunctions.cyclo ./Src/hlpfunctions.d ./Src/hlpfunctions.o ./Src/hlpfunctions.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/platform_config.cyclo ./Src/platform_config.d ./Src/platform_config.o ./Src/platform_config.su ./Src/stm32f4xx_hal_msp.cyclo ./Src/stm32f4xx_hal_msp.d ./Src/stm32f4xx_hal_msp.o ./Src/stm32f4xx_hal_msp.su ./Src/stm32f4xx_hal_timebase_TIM.cyclo ./Src/stm32f4xx_hal_timebase_TIM.d ./Src/stm32f4xx_hal_timebase_TIM.o ./Src/stm32f4xx_hal_timebase_TIM.su ./Src/stm32f4xx_it.cyclo ./Src/stm32f4xx_it.d ./Src/stm32f4xx_it.o ./Src/stm32f4xx_it.su ./Src/str2dbl.cyclo ./Src/str2dbl.d ./Src/str2dbl.o ./Src/str2dbl.su ./Src/system_stm32f4xx.cyclo ./Src/system_stm32f4xx.d ./Src/system_stm32f4xx.o ./Src/system_stm32f4xx.su

.PHONY: clean-Src

