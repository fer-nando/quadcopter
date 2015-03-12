################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
utils/delay.obj: ../utils/delay.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="utils/delay.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/i2c_functions.obj: ../utils/i2c_functions.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="utils/i2c_functions.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/linalg.obj: ../utils/linalg.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="utils/linalg.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/spi_functions.obj: ../utils/spi_functions.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="utils/spi_functions.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/uartstdio.obj: C:/StellarisWare/utils/uartstdio.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="utils/uartstdio.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


