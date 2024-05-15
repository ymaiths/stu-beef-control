################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
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
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/BasicMathFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/BayesFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/CommonTables" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/ComplexMathFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/ControllerFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/DistanceFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/FastMathFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/FilteringFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/InterpolationFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/MatrixFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/QuaternionMathFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/StatisticsFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/SupportFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/SVMFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/TransformFunctions" -I"C:/Users/ASUS/Desktop/My Folder/year 2/Term 2/Studio/modbus/ModbusDemo2024/MODBUS2024ALT/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

