# st25r3916b-driver — a small, MCU-independent C driver for the ST ST25R3916B NFC front end.
cmake_minimum_required(VERSION 3.13)
project(st25r3916b_driver C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# The library: pure C, no platform dependencies. Add it to your firmware and implement the
# st25r_bus_t transaction callback for your SPI peripheral (see README.md and examples/).
add_library(st25r3916 STATIC src/st25r3916.c)
target_include_directories(st25r3916 PUBLIC include)

# Host unit test (SPI framing, register access, the probe) against a fake chip. Opt out with
# -DST25R3916_BUILD_TESTS=OFF.
option(ST25R3916_BUILD_TESTS "Build the host unit test" ON)
if(ST25R3916_BUILD_TESTS)
    enable_testing()
    add_executable(test_st25r3916 test/test_st25r3916.c)
    target_link_libraries(test_st25r3916 PRIVATE st25r3916)
    target_include_directories(test_st25r3916 PRIVATE test)
    if(NOT MSVC)
        target_compile_options(test_st25r3916 PRIVATE -Wall -Wextra)
    endif()
    add_test(NAME test_st25r3916 COMMAND test_st25r3916)
endif()
