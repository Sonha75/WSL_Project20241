# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
 cmake_minimum_required(VERSION 3.5)

# set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/button
#                          $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/example_init)

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/button
                         $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/example_init
                         $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/example_nvs)
# set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/common)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(BLE_MESH_Node)

target_compile_options(${lib} PRIVATE -Wno-error=format)