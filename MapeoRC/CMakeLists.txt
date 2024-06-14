cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD "pico_w")

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(Prueba_senal)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(myblink_w
	RCmapeo.c
	control_pid.c
	
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(myblink_w pico_stdlib pico_cyw43_arch_none hardware_i2c hardware_pwm)

# create map/bin/hex/uf2 file in addition to ELF.
pico_enable_stdio_usb(myblink_w 1) 
pico_enable_stdio_uart(myblink_w 0)
pico_add_extra_outputs(myblink_w)