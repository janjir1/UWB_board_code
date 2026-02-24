cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM32L4)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/blink.cpp
    Core/Src/my_print.cpp
    
    #.h files
    Core/Inc/my_print.h

    #lsm6dsv files
    Core/Src/LSM6DSV/lsm6dsv_platform.c
    Core/Src/LSM6DSV/lsm6dsv_reg.c
    Core/Src/LSM6DSV/lsm6dsv_self_test.c
    Core/Src/LSM6DSV/read_data_polling.c
    Core/Src/LSM6DSV/lsm6dsv.c

    Core/Inc/LSM6DSV/lsm6dsv_platform.h
    Core/Inc/LSM6DSV/lsm6dsv_reg.h
    Core/Inc/LSM6DSV/lsm6dsv_ST_example.h



)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    "Drivers/dw3000-decadriver/dwt_uwb_driver/deca_interface.c"
    "Drivers/dw3000-decadriver/dwt_uwb_driver/deca_compat.c"
    "Drivers/dw3000-decadriver/dwt_uwb_driver/deca_rsl.c"
    "Drivers/dw3000-decadriver/dwt_uwb_driver/dw3000/dw3000_device.c"
    "Drivers/dw3000-decadriver/dwt_uwb_driver/lib/qmath/src/qmath.c"
    "Drivers/dw3000-decadriver/dwt_uwb_driver/dw3000/dw3000_device.c"
    "Drivers/dw3000-decadriver/platform/stm-freertos/dw_3000_hw.c"
    "Drivers/dw3000-decadriver/platform/stm-freertos/dw3000_spi.c"
    "Drivers/dw3000-decadriver/platform/stm-freertos/deca_port.c"

    

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "Drivers/dw3000-decadriver/dwt_uwb_driver"
    "Drivers/dw3000-decadriver/platform"
    "Drivers/dw3000-decadriver/dwt_uwb_driver/lib/qmath/include"
)
    # Add user defined include paths


# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE 
    -u _printf_float
)




