################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv5/tools/compiler/tms470_4.9.5

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./drivers/buttons.obj" \
"./sensors/L3G4200D.obj" \
"./sensors/HMC5883.obj" \
"./sensors/HC_SR04.obj" \
"./sensors/BMP085.obj" \
"./sensors/ADXL345.obj" \
"./utils/uartstdio.obj" \
"./utils/spi_functions.obj" \
"./utils/linalg.obj" \
"./utils/i2c_functions.obj" \
"./utils/delay.obj" \
"./startup_ccs.obj" \
"./sensors.obj" \
"./rc.obj" \
"./motor.obj" \
"./main.obj" \
"./RF24.obj" \
-l"libc.a" \
"../lm4f120h5qr.cmd" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include utils/subdir_vars.mk
-include sensors/subdir_vars.mk
-include drivers/subdir_vars.mk
-include subdir_rules.mk
-include utils/subdir_rules.mk
-include sensors/subdir_rules.mk
-include drivers/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: quadcopter.out

dependents:
	-cd C:/StellarisWare/driverlib/ccs-cm4f/Debug && $(MAKE) clean all
	-cd C:/StellarisWare/usblib/ccs-cm4f/Debug && $(MAKE) clean all

# Tool invocations
quadcopter.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS) C:/StellarisWare/driverlib/ccs-cm4f/Debug/driverlib-cm4f.lib C:/StellarisWare/usblib/ccs-cm4f/Debug/usblib-cm4f.lib
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --diag_warning=225 --display_error_number -z --stack_size=256 -m"quadcopter.map" --heap_size=0 -i"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/lib" -i"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --reread_libs --warn_sections --display_error_number --rom_model -o "quadcopter.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(TMS470_EXECUTABLE_OUTPUTS__QUOTED) "quadcopter.out"
	-$(RM) "RF24.pp" "main.pp" "motor.pp" "rc.pp" "sensors.pp" "startup_ccs.pp" "utils\delay.pp" "utils\i2c_functions.pp" "utils\linalg.pp" "utils\spi_functions.pp" "utils\uartstdio.pp" "sensors\ADXL345.pp" "sensors\BMP085.pp" "sensors\HC_SR04.pp" "sensors\HMC5883.pp" "sensors\L3G4200D.pp" "drivers\buttons.pp" 
	-$(RM) "RF24.obj" "main.obj" "motor.obj" "rc.obj" "sensors.obj" "startup_ccs.obj" "utils\delay.obj" "utils\i2c_functions.obj" "utils\linalg.obj" "utils\spi_functions.obj" "utils\uartstdio.obj" "sensors\ADXL345.obj" "sensors\BMP085.obj" "sensors\HC_SR04.obj" "sensors\HMC5883.obj" "sensors\L3G4200D.obj" "drivers\buttons.obj" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
C:/StellarisWare/driverlib/ccs-cm4f/Debug/driverlib-cm4f.lib:
C:/StellarisWare/usblib/ccs-cm4f/Debug/usblib-cm4f.lib:

-include ../makefile.targets
