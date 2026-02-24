add_subdirectory(lsm6dsv)
add_subdirectory(Platform)

add_library(lsm6dsv INTERFACE)
target_link_libraries(lsm6dsv INTERFACE
    lsm6dsv_core
    lsm6dsv_platform
)
