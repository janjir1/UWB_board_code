add_library(lsm6dsv_platform STATIC
    lsm6dsv_platform.c
)

target_include_directories(lsm6dsv_platform PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}          # lsm6dsv_platform.h
)

target_link_libraries(lsm6dsv_platform PUBLIC
    lsm6dsv_core                         # platform.h likely includes lsm6dsv_reg.h
)

target_link_libraries(lsm6dsv_platform PRIVATE stm32cubemx)
