

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
//define group address for pub/sub
#define ESP_BLE_MESH_GROUP_PUB_ADDR                     0xC002
#define ESP_BLE_MESH_GROUP_GW_SUB_ADDR                  0xC001
#define ESP_BLE_MESH_ADDR_ALL_NODES                     0xFFFF          

/** 
 * @brief Device Main Data Structure
 */
typedef struct __attribute__((packed)) {
    char device_name[6];
    
  
    float temperature;   
    int light;     
    float humidity;   
    double smoke;  

    
         
    float temp_ADC;          
    int index;
  

    /**< Feedback answers */
    uint8_t feedback;   
} model_sensor_data_t;


#endif   