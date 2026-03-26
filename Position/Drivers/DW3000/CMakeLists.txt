# Public entry point — firmware links against this target only
add_subdirectory(dwt_uwb_driver)
add_subdirectory(Platform)

target_include_directories(dw3000_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform
)

target_link_libraries(dw3000_platform PRIVATE stm32cubemx)

add_library(dw3000 INTERFACE)
target_link_libraries(dw3000 INTERFACE
    dw3000_core
    dw3000_platform
)

target_include_directories(dw3000 INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform        # dw3000_hw.h, dw3000_spi.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver  # deca_device_api.h, deca_types.h
)
