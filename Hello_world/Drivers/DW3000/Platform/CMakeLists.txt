add_library(dw3000_platform STATIC
    deca_compat.c                        # overrides dwt_uwb_driver version
    dw3000_spi_trace.c
    stm-freertos/deca_port.c
    stm-freertos/dw3000_spi.c
    stm-freertos/dw_3000_hw.c
)

target_include_directories(dw3000_platform PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}          # dw3000.h, dw3000_hw.h, dw3000_spi.h, etc.
)

# Needs driver core headers

target_link_libraries(dw3000_platform PUBLIC dw3000_core)


# Needs HAL + FreeRTOS — these come from the firmware target via INTERFACE
# firmware's CMakeLists must provide these, e.g. via a hal_freertos INTERFACE target
target_link_libraries(dw3000_platform PRIVATE hal_freertos)
