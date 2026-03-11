add_library(lsm6dsv_core STATIC
    lsm6dsv_reg.c
)

target_include_directories(lsm6dsv_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}          # lsm6dsv_reg.h
)
