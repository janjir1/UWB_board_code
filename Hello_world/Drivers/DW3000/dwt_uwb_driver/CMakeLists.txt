add_subdirectory(lib/qmath)

add_library(dw3000_core STATIC
    deca_compat.c
    deca_interface.c
    deca_rsl.c
    dw3000/dw3000_device.c
)

target_include_directories(dw3000_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}          # deca_device_api.h, deca_types.h, etc.
    ${CMAKE_CURRENT_SOURCE_DIR}/dw3000   # dw3000_deca_regs.h, dw3000_deca_vals.h
)

target_link_libraries(dw3000_core PUBLIC
    qmath
)
