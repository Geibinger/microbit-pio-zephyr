cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(microbit-pio-zephyr)

file(GLOB app_sources ${CMAKE_CURRENT_LIST_DIR}/../src/*.[cS])
target_sources(app PRIVATE ${app_sources})
