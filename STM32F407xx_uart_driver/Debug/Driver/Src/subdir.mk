################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver/Src/stm32f407xx_gpio_driver.c \
../Driver/Src/stm32f407xx_rcc_driver.c \
../Driver/Src/stm32f407xx_uart_driver.c 

OBJS += \
./Driver/Src/stm32f407xx_gpio_driver.o \
./Driver/Src/stm32f407xx_rcc_driver.o \
./Driver/Src/stm32f407xx_uart_driver.o 

C_DEPS += \
./Driver/Src/stm32f407xx_gpio_driver.d \
./Driver/Src/stm32f407xx_rcc_driver.d \
./Driver/Src/stm32f407xx_uart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Driver/Src/stm32f407xx_gpio_driver.o: ../Driver/Src/stm32f407xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/erenc/OneDrive/Desktop/STM32F4_git/STME32F407xx_uart_driver/STM32F407xx_uart_driver/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Driver/Src/stm32f407xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Driver/Src/stm32f407xx_rcc_driver.o: ../Driver/Src/stm32f407xx_rcc_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/erenc/OneDrive/Desktop/STM32F4_git/STME32F407xx_uart_driver/STM32F407xx_uart_driver/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Driver/Src/stm32f407xx_rcc_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Driver/Src/stm32f407xx_uart_driver.o: ../Driver/Src/stm32f407xx_uart_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/erenc/OneDrive/Desktop/STM32F4_git/STME32F407xx_uart_driver/STM32F407xx_uart_driver/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Driver/Src/stm32f407xx_uart_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

