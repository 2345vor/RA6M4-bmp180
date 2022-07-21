################################################################################
# �Զ����ɵ��ļ�����Ҫ�༭��
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../rt-thread/libcpu/arm/cortex-m4/cpuport.c 

S_UPPER_SRCS += \
../rt-thread/libcpu/arm/cortex-m4/context_gcc.S 

OBJS += \
./rt-thread/libcpu/arm/cortex-m4/context_gcc.o \
./rt-thread/libcpu/arm/cortex-m4/cpuport.o 

S_UPPER_DEPS += \
./rt-thread/libcpu/arm/cortex-m4/context_gcc.d 

C_DEPS += \
./rt-thread/libcpu/arm/cortex-m4/cpuport.d 


# Each subdirectory must supply rules for building sources it contributes
rt-thread/libcpu/arm/cortex-m4/%.o: ../rt-thread/libcpu/arm/cortex-m4/%.S
	arm-none-eabi-gcc -x assembler-with-cpp -Xassembler -mimplicit-it=thumb -c -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb  -gdwarf-2 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
rt-thread/libcpu/arm/cortex-m4/%.o: ../rt-thread/libcpu/arm/cortex-m4/%.c
	arm-none-eabi-gcc -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\board\ports" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\board" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\libraries\HAL_Drivers\config" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\libraries\HAL_Drivers" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\packages\bmp180-latest" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\ra\arm\CMSIS_5\CMSIS\Core\Include" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\ra\fsp\inc\api" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\ra\fsp\inc\instances" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\ra\fsp\inc" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\ra_cfg\fsp_cfg\bsp" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\ra_cfg\fsp_cfg" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\ra_gen" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\components\drivers\include" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\components\drivers\sensors" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\components\finsh" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\components\libc\compilers\common" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\components\libc\compilers\newlib" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\components\libc\posix\io\poll" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\components\libc\posix\io\stdio" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\components\libc\posix\ipc" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\include" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\libcpu\arm\common" -I"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rt-thread\libcpu\arm\cortex-m4" -include"D:\RT-ThreadStudio\workspace\RA6M4-bmp180\rtconfig_preinc.h" -std=gnu11 -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
