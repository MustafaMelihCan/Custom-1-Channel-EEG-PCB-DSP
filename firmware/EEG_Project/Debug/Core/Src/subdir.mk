################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/arm_biquad_cascade_df2T_f32.c \
../Core/Src/arm_biquad_cascade_df2T_init_f32.c \
../Core/Src/arm_bitreversal2.c \
../Core/Src/arm_cfft_f32.c \
../Core/Src/arm_cfft_init_f32.c \
../Core/Src/arm_cfft_radix8_f32.c \
../Core/Src/arm_cmplx_mag_squared_f32.c \
../Core/Src/arm_common_tables.c \
../Core/Src/arm_const_structs.c \
../Core/Src/arm_rfft_fast_f32.c \
../Core/Src/arm_rfft_fast_init_f32.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/arm_biquad_cascade_df2T_f32.o \
./Core/Src/arm_biquad_cascade_df2T_init_f32.o \
./Core/Src/arm_bitreversal2.o \
./Core/Src/arm_cfft_f32.o \
./Core/Src/arm_cfft_init_f32.o \
./Core/Src/arm_cfft_radix8_f32.o \
./Core/Src/arm_cmplx_mag_squared_f32.o \
./Core/Src/arm_common_tables.o \
./Core/Src/arm_const_structs.o \
./Core/Src/arm_rfft_fast_f32.o \
./Core/Src/arm_rfft_fast_init_f32.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/arm_biquad_cascade_df2T_f32.d \
./Core/Src/arm_biquad_cascade_df2T_init_f32.d \
./Core/Src/arm_bitreversal2.d \
./Core/Src/arm_cfft_f32.d \
./Core/Src/arm_cfft_init_f32.d \
./Core/Src/arm_cfft_radix8_f32.d \
./Core/Src/arm_cmplx_mag_squared_f32.d \
./Core/Src/arm_common_tables.d \
./Core/Src/arm_const_structs.d \
./Core/Src/arm_rfft_fast_f32.d \
./Core/Src/arm_rfft_fast_init_f32.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I/Users/mustafamelihcan/Documents/CMSIS-DSP-main/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/arm_biquad_cascade_df2T_f32.cyclo ./Core/Src/arm_biquad_cascade_df2T_f32.d ./Core/Src/arm_biquad_cascade_df2T_f32.o ./Core/Src/arm_biquad_cascade_df2T_f32.su ./Core/Src/arm_biquad_cascade_df2T_init_f32.cyclo ./Core/Src/arm_biquad_cascade_df2T_init_f32.d ./Core/Src/arm_biquad_cascade_df2T_init_f32.o ./Core/Src/arm_biquad_cascade_df2T_init_f32.su ./Core/Src/arm_bitreversal2.cyclo ./Core/Src/arm_bitreversal2.d ./Core/Src/arm_bitreversal2.o ./Core/Src/arm_bitreversal2.su ./Core/Src/arm_cfft_f32.cyclo ./Core/Src/arm_cfft_f32.d ./Core/Src/arm_cfft_f32.o ./Core/Src/arm_cfft_f32.su ./Core/Src/arm_cfft_init_f32.cyclo ./Core/Src/arm_cfft_init_f32.d ./Core/Src/arm_cfft_init_f32.o ./Core/Src/arm_cfft_init_f32.su ./Core/Src/arm_cfft_radix8_f32.cyclo ./Core/Src/arm_cfft_radix8_f32.d ./Core/Src/arm_cfft_radix8_f32.o ./Core/Src/arm_cfft_radix8_f32.su ./Core/Src/arm_cmplx_mag_squared_f32.cyclo ./Core/Src/arm_cmplx_mag_squared_f32.d ./Core/Src/arm_cmplx_mag_squared_f32.o ./Core/Src/arm_cmplx_mag_squared_f32.su ./Core/Src/arm_common_tables.cyclo ./Core/Src/arm_common_tables.d ./Core/Src/arm_common_tables.o ./Core/Src/arm_common_tables.su ./Core/Src/arm_const_structs.cyclo ./Core/Src/arm_const_structs.d ./Core/Src/arm_const_structs.o ./Core/Src/arm_const_structs.su ./Core/Src/arm_rfft_fast_f32.cyclo ./Core/Src/arm_rfft_fast_f32.d ./Core/Src/arm_rfft_fast_f32.o ./Core/Src/arm_rfft_fast_f32.su ./Core/Src/arm_rfft_fast_init_f32.cyclo ./Core/Src/arm_rfft_fast_init_f32.d ./Core/Src/arm_rfft_fast_init_f32.o ./Core/Src/arm_rfft_fast_init_f32.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

