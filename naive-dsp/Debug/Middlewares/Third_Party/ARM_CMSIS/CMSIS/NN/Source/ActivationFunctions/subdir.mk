################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

