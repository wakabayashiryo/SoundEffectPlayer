################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/PWM/LPC11xx_PWM.c 

OBJS += \
./src/PWM/LPC11xx_PWM.o 

C_DEPS += \
./src/PWM/LPC11xx_PWM.d 


# Each subdirectory must supply rules for building sources it contributes
src/PWM/%.o: ../src/PWM/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M0 -D__USE_CMSIS=CMSISv1p30_LPC11xx -D__LPC11XX__ -D__REDLIB__ -I"/home/evaota/LPCXpresso/workspace/CMSISv1p30_LPC11xx/inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


