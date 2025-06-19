################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/BasicMathFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/BayesFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/CommonTables" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/ComplexMathFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/ControllerFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/DistanceFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/FastMathFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/FilteringFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/InterpolationFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/MatrixFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/QuaternionMathFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/StatisticsFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/SupportFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/SVMFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/TransformFunctions" -I"/home/beamkeerati/Solar/transporter_ws/firmware/solar_transport/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-QuaternionMathFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-QuaternionMathFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-QuaternionMathFunctions

