################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/Components/lsm303dlhc/lsm303dlhc.c 

OBJS += \
./Utilities/Components/lsm303dlhc/lsm303dlhc.o 

C_DEPS += \
./Utilities/Components/lsm303dlhc/lsm303dlhc.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/Components/lsm303dlhc/%.o: ../Utilities/Components/lsm303dlhc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ili9325" -I/home/nursyeha/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.externaltools.arm-none.linux64_1.17.0.201812190825/tools/compiler/lib/gcc/arm-none-eabi/7.3.1/include-fixed -I/home/nursyeha/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.externaltools.arm-none.linux64_1.17.0.201812190825/tools/compiler/lib/gcc/arm-none-eabi/7.3.1/include -I/home/nursyeha/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.externaltools.arm-none.linux64_1.17.0.201812190825/tools/compiler/arm-none-eabi/include -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/s25fl512s" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/cs43l22" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ili9341" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ampire480272" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/n25q512a" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/s5k5cag" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/mfxstm32l152" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/CMSIS/device" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/n25q128a" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ts3510" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/st7735" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/HAL_Driver/Inc/Legacy" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/lis302dl" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/otm8009a" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/stmpe1600" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ov2640" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/Common" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/l3gd20" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/HAL_Driver/Inc" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/stmpe811" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/lis3dsh" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/wm8994" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/n25q256a" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/inc" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ls016b8uy" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ft6x06" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/STM32F4-Discovery" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/exc7200" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/st7789h2" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ampire640480" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/lsm303dlhc" -I"/home/nursyeha/Documents/Stm32_parent/f4SwarmRobotInC_Forward/CMSIS/core" -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


