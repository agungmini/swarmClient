################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f407xx.s 

OBJS += \
./startup/startup_stm32f407xx.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ili9325" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/s25fl512s" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/cs43l22" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ili9341" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ampire480272" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/n25q512a" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/s5k5cag" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/mfxstm32l152" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/CMSIS/device" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/n25q128a" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ts3510" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/st7735" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/HAL_Driver/Inc/Legacy" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/lis302dl" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/otm8009a" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/stmpe1600" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ov2640" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/Common" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/l3gd20" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/HAL_Driver/Inc" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/stmpe811" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/lis3dsh" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/wm8994" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/n25q256a" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/inc" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ls016b8uy" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ft6x06" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/STM32F4-Discovery" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/exc7200" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/st7789h2" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/ampire640480" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/Utilities/Components/lsm303dlhc" -I"/home/nursyeha/Documents/projects/stm32_parent/f4SwarmRobotInC_Forward/CMSIS/core" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


