cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME ism330dhcx) # <-- Name your project/executable here!

include(pico_sdk_import.cmake)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(default_build_type "Release")

set(CMAKE_C_COMPILER /usr/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER /usr/bin/arm-none-eabi-gcc)


# Gooey boilerplate

project(${NAME} VERSION 0.0.1 LANGUAGES C CXX ASM)


# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(${NAME}
    src/ism330dhcx.cpp # <-- Add source files here!
)

# Don't forget to link the libraries you need!
target_link_libraries(${NAME}
    pico_stdlib            # Standard Pico library
    hardware_adc
    hardware_gpio
    hardware_i2c
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})


# Enable or Disable UART
pico_enable_stdio_uart(${NAME} 0)

# Enable or Disable USB CDC
pico_enable_stdio_usb(${NAME} 1)
