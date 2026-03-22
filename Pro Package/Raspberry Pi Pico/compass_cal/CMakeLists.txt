add_executable(${PROJECT_NAME}
    compass_cal.cpp
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
)

pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)