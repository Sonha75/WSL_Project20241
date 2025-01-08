

#ifndef __CUSTOM_SENSOR_MODEL_DEFS_H__
#define __CUSTOM_SENSOR_MODEL_DEFS_H__

#include <stdio.h>

#include "sdkconfig.h"

#include "esp_ble_mesh_common_api.h"

#define BLE_MESH_DEVICE_NAME    "BLE Mesh" /*!< Device Advertising Name */ 
#define CID_ESP                 0x02E5                  /*!< Espressif Component ID */

//* define id for model (2 types)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_ID_SERVER      0x1414  /*!< Custom Server Model ID */
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_ID_CLIENT      0x1415  /*!< Custom Client Model ID */

//* define opcode message(3 types)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_OP_GET         ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_OP_SET         ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_OP_STATUS      ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_OP_ACK         ESP_BLE_MESH_MODEL_OP_3(0x03, CID_ESP)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_OP_NACK        ESP_BLE_MESH_MODEL_OP_3(0x04, CID_ESP)
//define group address for pub/sub
#define ESP_BLE_MESH_GROUP_PUB_ADDR                     0xC002
#define ESP_BLE_MESH_GROUP_GW_SUB_ADDR                  0xC001
#define ESP_BLE_MESH_ADDR_ALL_NODES                     0xFFFF          

/** 
 * @brief Device Main Data Structure
 */

typedef struct __attribute__((packed)) {
    uint32_t msg_id;
    char device_name[5];
    float temperature;      
    float humidity;          
    int index;
    bool status;
    uint32_t csum; 
} model_sensor_data_t;

typedef union{
    model_sensor_data_t message;
    uint8_t data[22];
    
}union_t;
typedef struct {
    uint32_t msg_ack_id;      // ID của message cần ACK
    uint8_t status;       // 0: Success, 1: Error, 2: Invalid data...
    uint32_t timestamp;   // Timestamp để track thời gian
} ack_message_t;

#endif   