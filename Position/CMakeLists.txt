cmake_minimum_required(VERSION 3.22)

# ── BOARD SELECTION ──────────────────────────────────────────────────────────
if(NOT DEFINED BOARD)
    set(BOARD "Board_V1.1")
endif()
message("Board: ${BOARD}")
# ─────────────────────────────────────────────────────────────────────────────

# Compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME STM32L4)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(CMAKE_TOOLCHAIN_FILE 
    ${CMAKE_CURRENT_SOURCE_DIR}/${BOARD}/cmake/gcc-arm-none-eabi.cmake
    CACHE STRING "Toolchain file")

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})

# ── Board-specific CubeMX generated sources (HAL, CMSIS, Core, clock) ────────
add_subdirectory(${BOARD}/cmake/stm32cubemx)

# ── Link options ──────────────────────────────────────────────────────────────
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE 
    -u _printf_float
    -Wl,-Map=${CMAKE_BINARY_DIR}/Position.map)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ── Shared drivers and app (same for both boards) ────────────────────────────
add_library(hal_freertos INTERFACE)

add_subdirectory(Drivers/DW3000)
add_subdirectory(Drivers/lsm6dsv)
add_subdirectory(App)


# ── Link everything ───────────────────────────────────────────────────────────
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    dw3000
    lsm6dsv
    app
)