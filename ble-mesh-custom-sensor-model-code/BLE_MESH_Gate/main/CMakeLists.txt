cmake_minimum_required(VERSION 3.5)

set(srcs "main.c" "board.c"  "mqtt.c" "wifi_config.c" "../../src/mesh_client.c" "../../src/mesh_device_app.c" )
        
# set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/button)

idf_component_register(SRCS "${srcs}" 
                    INCLUDE_DIRS  "." "../../include" "../../BLE_MESH_Node/main")
