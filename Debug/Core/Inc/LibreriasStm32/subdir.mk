################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/LibreriasStm32/ChipCap2.c \
../Core/Inc/LibreriasStm32/MDC100.c \
../Core/Inc/LibreriasStm32/R12DS.c \
../Core/Inc/LibreriasStm32/SparkFun_Qwiic_Relay.c \
../Core/Inc/LibreriasStm32/bluetooth.c \
../Core/Inc/LibreriasStm32/bno055.c \
../Core/Inc/LibreriasStm32/keya_driver.c \
../Core/Inc/LibreriasStm32/kinematics.c \
../Core/Inc/LibreriasStm32/tf02Pro.c 

OBJS += \
./Core/Inc/LibreriasStm32/ChipCap2.o \
./Core/Inc/LibreriasStm32/MDC100.o \
./Core/Inc/LibreriasStm32/R12DS.o \
./Core/Inc/LibreriasStm32/SparkFun_Qwiic_Relay.o \
./Core/Inc/LibreriasStm32/bluetooth.o \
./Core/Inc/LibreriasStm32/bno055.o \
./Core/Inc/LibreriasStm32/keya_driver.o \
./Core/Inc/LibreriasStm32/kinematics.o \
./Core/Inc/LibreriasStm32/tf02Pro.o 

C_DEPS += \
./Core/Inc/LibreriasStm32/ChipCap2.d \
./Core/Inc/LibreriasStm32/MDC100.d \
./Core/Inc/LibreriasStm32/R12DS.d \
./Core/Inc/LibreriasStm32/SparkFun_Qwiic_Relay.d \
./Core/Inc/LibreriasStm32/bluetooth.d \
./Core/Inc/LibreriasStm32/bno055.d \
./Core/Inc/LibreriasStm32/keya_driver.d \
./Core/Inc/LibreriasStm32/kinematics.d \
./Core/Inc/LibreriasStm32/tf02Pro.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/LibreriasStm32/%.o: ../Core/Inc/LibreriasStm32/%.c Core/Inc/LibreriasStm32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-LibreriasStm32

clean-Core-2f-Inc-2f-LibreriasStm32:
	-$(RM) ./Core/Inc/LibreriasStm32/ChipCap2.d ./Core/Inc/LibreriasStm32/ChipCap2.o ./Core/Inc/LibreriasStm32/MDC100.d ./Core/Inc/LibreriasStm32/MDC100.o ./Core/Inc/LibreriasStm32/R12DS.d ./Core/Inc/LibreriasStm32/R12DS.o ./Core/Inc/LibreriasStm32/SparkFun_Qwiic_Relay.d ./Core/Inc/LibreriasStm32/SparkFun_Qwiic_Relay.o ./Core/Inc/LibreriasStm32/bluetooth.d ./Core/Inc/LibreriasStm32/bluetooth.o ./Core/Inc/LibreriasStm32/bno055.d ./Core/Inc/LibreriasStm32/bno055.o ./Core/Inc/LibreriasStm32/keya_driver.d ./Core/Inc/LibreriasStm32/keya_driver.o ./Core/Inc/LibreriasStm32/kinematics.d ./Core/Inc/LibreriasStm32/kinematics.o ./Core/Inc/LibreriasStm32/tf02Pro.d ./Core/Inc/LibreriasStm32/tf02Pro.o

.PHONY: clean-Core-2f-Inc-2f-LibreriasStm32

