cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ..)

set(TEST_COMPONENTS "ble_bidi_fast" CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_runner)
