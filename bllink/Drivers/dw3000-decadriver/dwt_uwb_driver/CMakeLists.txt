cmake_minimum_required(VERSION 3.13)

# Define the library name
project(dw3000_driver)

# Create the library (Static Link)
add_library(dw3000_driver STATIC)

# -----------------------------------------------------------------------------
# 1. Driver Core Files (Inside dwt_uwb_driver folder)
# -----------------------------------------------------------------------------
target_sources(dw3000_driver PRIVATE
    "deca_interface.c"
    "deca_compat.c"
    "deca_rsl.c"
    "dw3000/dw3000_device.c"
    "lib/qmath/src/qmath.c"
)

# -----------------------------------------------------------------------------
# 2. Platform Generic Files (Inside ../platform folder)
# -----------------------------------------------------------------------------
# These are the top-level API files you mentioned
target_sources(dw3000_driver PRIVATE
    "../platform/deca_compat.c"
    "../platform/dw3000_spi_trace.c"
)

# -----------------------------------------------------------------------------
# 3. STM32-FreeRTOS Port Files (Inside ../platform/stm-freertos folder)
# -----------------------------------------------------------------------------
# These implement the low-level hardware access
target_sources(dw3000_driver PRIVATE
    "../platform/stm-freertos/deca_port.c"
    "../platform/stm-freertos/dw3000_spi.c"
    "../platform/stm-freertos/dw_3000_hw.c"
)

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------
# PUBLIC = Headers needed by your Main Application
# PRIVATE = Headers needed only inside the driver code
target_include_directories(dw3000_driver PUBLIC
    "."                           # For deca_device_api.h
    "dw3000"                      # For dw3000_regs.h
    "lib/qmath/include"           # For qmath.h
    "../platform"                 # For Top API headers (dw3000.h, etc.)
    "../platform/stm-freertos"    # For platform specific headers
    "${CMAKE_SOURCE_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc"
    "${CMAKE_SOURCE_DIR}/Core/Inc"
    "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32L4xx/Include"
    "${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include"
    "${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"
    "${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" 
)

# -----------------------------------------------------------------------------
# Compiler Definitions
# -----------------------------------------------------------------------------
target_compile_definitions(dw3000_driver PUBLIC
    -DDWT_DW3000          # Enable DW3000 driver code
    -DQMATH               # Enable QMath support
    -DUSE_FREERTOS        # Often needed by port files
    -DSTM32L4xx        
)

# Link against math library (standard C lib) if qmath needs it
target_link_libraries(dw3000_driver PRIVATE m)
