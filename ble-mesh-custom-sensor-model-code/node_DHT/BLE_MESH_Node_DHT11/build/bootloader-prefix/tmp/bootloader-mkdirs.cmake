# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/IDF/install/Espressif/frameworks/esp-idf-v5.2.3/components/bootloader/subproject"
  "D:/IDF/git/WSL_Project20241/ble-mesh-custom-sensor-model-code/node_DHT/BLE_MESH_Node_DHT11/build/bootloader"
  "D:/IDF/git/WSL_Project20241/ble-mesh-custom-sensor-model-code/node_DHT/BLE_MESH_Node_DHT11/build/bootloader-prefix"
  "D:/IDF/git/WSL_Project20241/ble-mesh-custom-sensor-model-code/node_DHT/BLE_MESH_Node_DHT11/build/bootloader-prefix/tmp"
  "D:/IDF/git/WSL_Project20241/ble-mesh-custom-sensor-model-code/node_DHT/BLE_MESH_Node_DHT11/build/bootloader-prefix/src/bootloader-stamp"
  "D:/IDF/git/WSL_Project20241/ble-mesh-custom-sensor-model-code/node_DHT/BLE_MESH_Node_DHT11/build/bootloader-prefix/src"
  "D:/IDF/git/WSL_Project20241/ble-mesh-custom-sensor-model-code/node_DHT/BLE_MESH_Node_DHT11/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/IDF/git/WSL_Project20241/ble-mesh-custom-sensor-model-code/node_DHT/BLE_MESH_Node_DHT11/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/IDF/git/WSL_Project20241/ble-mesh-custom-sensor-model-code/node_DHT/BLE_MESH_Node_DHT11/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
