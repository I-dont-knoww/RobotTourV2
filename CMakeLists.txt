# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)
set(PICO_BOARD_CMAKE_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)
set(PICO_BOARD robot_tour CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(main C CXX ASM)

pico_sdk_init()

add_executable(
    ${PROJECT_NAME}
    src/main.cpp

    src/drivers/battery/Battery.cpp
    src/drivers/button/Button.cpp
    src/drivers/compass/Compass.cpp
    src/drivers/encoders/Encoders.cpp
    src/drivers/gyroscope/Gyroscope.cpp
    src/drivers/ledRGB/LedRGB.cpp
    src/drivers/motors/Motors.cpp
    src/drivers/time/Time.cpp

    src/filters/LagFilter.cpp
    src/filters/RCFilter.cpp

    src/fusion/Fusion.cpp

    src/kinematics/ForwardKinematics.cpp
    src/kinematics/InverseKinematics.cpp

    src/managers/ExitCondition.cpp
    src/managers/Heading.cpp
    src/managers/Linear.cpp
    src/managers/Rotation.cpp

    src/regulators/CurrentRegulator.cpp
    src/regulators/VelocityRegulator.cpp
)

pico_generate_pio_header(
    ${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/drivers/compass/Compass.pio
    ${CMAKE_CURRENT_LIST_DIR}/src/drivers/encoders/Encoders.pio
    ${CMAKE_CURRENT_LIST_DIR}/src/drivers/gyroscope/Gyroscope.pio
)

target_link_libraries(
    ${PROJECT_NAME}

    hardware_adc
    hardware_clocks
    hardware_dma
    hardware_flash
    hardware_gpio
    hardware_i2c
    hardware_irq
    hardware_pwm
    hardware_pio
    hardware_spi
    hardware_timer

    pico_flash
    pico_multicore
    pico_stdlib
    pico_sync
    pico_time
)

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_set_program_name(${PROJECT_NAME} "main")
pico_set_program_version(${PROJECT_NAME} "0.1")

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})
