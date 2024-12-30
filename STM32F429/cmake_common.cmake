###################### CONSTANTS ######################################
set (PROJECT_TYPE_EXECUTABLE          "exe")
set (MCPU_CORTEX_M4				      "-mcpu=cortex-m4")
set (MFPU_FPV4_SP_D16                 "-mfpu=fpv4-sp-d16")
set (RUNTIME_LIBRARY_REDUCED_C        "--specs=nano.specs")
set (RUNTIME_LIBRARY_STD_C            "")
set (RUNTIME_LIBRARY_SYSCALLS_MINIMAL "--specs=nosys.specs")
set (RUNTIME_LIBRARY_SYSCALLS_NONE    "")
set (MFLOAT_ABI_SOFTWARE              "-mfloat-abi=soft")
set (MFLOAT_ABI_HARDWARE              "-mfloat-abi=hard")
set (MFLOAT_ABI_MIX                   "-mfloat-abi=softfp")
set (MFLOAT_USE_SINGLE_PREC           "-fsingle-precision-constant")
#######################################################################

set(HAL_PATH "${CMAKE_SOURCE_DIR}/../CommonDrivers/Drivers/STM32F4xx_HAL_Driver")
set(CMSIS_PATH "${CMAKE_SOURCE_DIR}/../CommonDrivers/Drivers/CMSIS")

# STM32 Chip
set(STM32_CHIP "STM32F429xx")

