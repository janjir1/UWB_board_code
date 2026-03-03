add_library(app STATIC
    # Accelerometer
    Accelerometer/lsm6dsv.c
    Accelerometer/lsm6dsv_self_test.c
    Accelerometer/read_data_polling.c

    # Blink
    Blink/blink.cpp

    # Generic utilities
    Generic/my_print.cpp

    # DWM — add files here when ready
    DWM/DWM3000.c
)

target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Accelerometer
    ${CMAKE_CURRENT_SOURCE_DIR}/Blink
    ${CMAKE_CURRENT_SOURCE_DIR}/Generic
    ${CMAKE_CURRENT_SOURCE_DIR}/DWM
)

# Both drivers — headers and symbols
target_link_libraries(app PUBLIC
    dw3000
    lsm6dsv
)

# HAL + FreeRTOS + device defines
target_link_libraries(app PRIVATE stm32cubemx)

get_target_property(DIRS app INCLUDE_DIRECTORIES)
message(STATUS "App includes: ${DIRS}")
