cmake_minimum_required(VERSION 3.13...3.27)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(u2f-hax
    main.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(u2f-hax pico_stdlib hardware_resets hardware_irq)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(u2f-hax)
