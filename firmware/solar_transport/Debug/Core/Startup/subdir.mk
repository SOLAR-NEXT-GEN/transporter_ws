################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/BasicMathFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/BayesFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/CommonTables" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/ComplexMathFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/ControllerFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/DistanceFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/FastMathFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/FilteringFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/InterpolationFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/MatrixFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/QuaternionMathFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/StatisticsFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/SupportFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/SVMFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/TransformFunctions" -I"/home/natthaphxt/Documents/GitHub/transporter_ws/firmware/solar_transport/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

