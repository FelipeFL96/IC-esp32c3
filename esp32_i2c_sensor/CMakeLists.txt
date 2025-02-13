cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})

project(esp32_i2c_sensor)

FILE(GLOB app_sources src/*.c)

target_sources(app PRIVATE ${app_sources})
