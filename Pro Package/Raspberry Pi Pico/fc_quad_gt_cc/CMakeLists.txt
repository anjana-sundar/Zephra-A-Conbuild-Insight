add_executable(${PROJECT_NAME}
    fc_quad_gt_cc.cpp
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_uart
    hardware_irq
    hardware_dma
    hardware_i2c
    hardware_pwm
)

pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)