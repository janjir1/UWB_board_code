add_library(app STATIC
    # Accelerometer
    Accelerometer/lsm6dsv.c
    Accelerometer/lsm6dsv_self_test.c
    Accelerometer/read_data_polling.c
    Accelerometer/imu.c

    # Blink
    Blink/blink.c

    # Generic utilities
    Generic/my_print.c

    # DWM — add files here when ready
    DWM/DWM3000.c
    DWM/DWM3000_driver.c

    UWB_app/messages.c
    UWB_app/uwb_network.c
    UWB_app/uwb_exchange.c
    Calculations/distance.c
    Calculations/calibrate.c
)

target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Accelerometer
    ${CMAKE_CURRENT_SOURCE_DIR}/Blink
    ${CMAKE_CURRENT_SOURCE_DIR}/Generic
    ${CMAKE_CURRENT_SOURCE_DIR}/DWM
    ${CMAKE_CURRENT_SOURCE_DIR}/Calculations
)

# Both drivers — headers and symbols
target_link_libraries(app PUBLIC
    dw3000
    lsm6dsv
    m
)

# HAL + FreeRTOS + device defines
target_link_libraries(app PRIVATE stm32cubemx)

get_target_property(DIRS app INCLUDE_DIRECTORIES)
message(STATUS "App includes: ${DIRS}")
