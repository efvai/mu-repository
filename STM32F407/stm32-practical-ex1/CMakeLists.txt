cmake_minimum_required(VERSION 3.20)

# ##################### VARIABLES ######################################
set(PROJECT_NAME "stm32-practical-ex1")
set(PROJECT_TYPE ${PROJECT_TYPE_EXECUTABLE})
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F407VETX_FLASH.ld")
set(MCPU ${MCPU_CORTEX_M4})
set(MFPU ${MFPU_FPV4_SP_D16})
set(MFLOAT_ABI ${MFLOAT_ABI_HARDWARE})
set(RUNTIME_LIBRARY ${RUNTIME_LIBRARY_REDUCED_C})
set(RUNTIME_LIBRARY_SYSCALLS ${RUNTIME_LIBRARY_SYSCALLS_MINIMAL})

set(PROJECT_SOURCES

	# LIST SOURCE FILES HERE
	Core/Startup/startup_stm32f407vetx.s
	Core/Src/main.c
	Core/Src/syscalls.c
	Core/Src/sysmem.c
	Core/Src/system_stm32f4xx.c
	Core/Src/stm32f4xx_it.c
	Core/Src/stm32f4xx_hal_msp.c

	Core/Src/virtual_motor.c
)

set(STM32_SOURCES

	# HAL
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
)

set(PROJECT_INCLUDES

	# LIST INCLUDE DIRECTORIES HERE

	# HAL
	Drivers/STM32F4xx_HAL_Driver/Inc

	# CMSIS
	Drivers/CMSIS/Include
	Drivers/CMSIS/Device/ST/STM32F4xx/Include

	# SRC
	Core/Inc
)

# ########### MODIFY ACCORDING TO REQUIREMENTS) ########################

# ######################################################################

# ################# PROJECT SETUP ######################################
project(${PROJECT_NAME} C ASM)

set(CMAKE_SYSTEM_PROCESSOR "arm" CACHE STRING "")
set(CMAKE_SYSTEM_NAME "Generic" CACHE STRING "")

# Specify toolchain.
set(TOOLCHAIN_PREFIX "arm-none-eabi-")
set(CMAKE_C_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++")
set(CMAKE_AR "${TOOLCHAIN_PREFIX}ar")
set(CMAKE_LINKER "${TOOLCHAIN_PREFIX}ld")
set(CMAKE_OBJCOPY "${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_RANLIB "${TOOLCHAIN_PREFIX}ranlib")
set(CMAKE_SIZE "${TOOLCHAIN_PREFIX}size")
set(CMAKE_STRIP "${TOOLCHAIN_PREFIX}ld")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Remove Windows-specific linker flags
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_FLAGS "")

# for vscode intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# elf
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>)

# bin
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O binary
	"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf"
	"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin"
	COMMENT "Converting ELF to BIN"
	VERBATIM
)

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_INCLUDES})

# Add HAL source files
target_sources(${PROJECT_NAME} PRIVATE ${STM32_SOURCES})

# Define STM32F405xx preprocessor macro
target_compile_definitions(${PROJECT_NAME} PRIVATE ${STM32_CHIP})
target_compile_definitions(${PROJECT_NAME} PRIVATE "USE_HAL_DRIVER")

set(CMAKE_EXECUTABLE_SUFFIX ".elf")
set(CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -ffunction-sections -fdata-sections -mthumb -Wall -Werror")
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -Wl,-Map=test.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
