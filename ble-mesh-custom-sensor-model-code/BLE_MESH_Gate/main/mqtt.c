#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"



#include "nvs_flash.h"

#include "esp_log.h"

#include "mqtt_client.h"



#include "mqtt.h"

esp_mqtt_client_handle_t client;
char Handler[50];
uint8_t flagmqtt;
static const char *TAG = "MQTT";

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
     int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
       // mode = NORMAL;
       //esp_mqtt_client_subscribe(client, "v1/devices/me/attributes", 1 );
       //esp_mqtt_client_subscribe(client, "v1/devices/me/rpc/request/+", 1 );
       ESP_LOGI(TAG, "MQTT event connected");
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        
        ESP_LOGI(TAG, "MQTT event disconnected");
        esp_mqtt_client_reconnect(client);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT event subcribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT event unsubcribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT event published, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG,"MQTT event_data");
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT event error");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// static void mqtt_task(void *param)
// {
//     char *mess_recv = NULL;
//     size_t mess_size = 0;
//     mqtt_obj_t mqtt_obj;
//     while (1)
//     {
//         mess_recv = (char *)xRingbufferReceive(mqtt_ring_buf, &mess_size, portMAX_DELAY);
//         if (mess_recv)
//         {
//             mess_recv[mess_size] = '\0';
//             ESP_LOGI(TAG, "Recv payload: %s", mess_recv);
//             memset(&mqtt_obj, 0, sizeof(mqtt_obj));
//             mqtt_parse_data(mess_recv, &mqtt_obj);

//             vRingbufferReturnItem(mqtt_ring_buf, (void *)mess_recv);
//         }
//     }
// }

void mqtt_client_sta(void)
{
    uint8_t broker[50] = {0};
    ESP_LOGI(TAG, "MQTT init");
 
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = Server,
        // .broker.address.port =1883,
        .credentials.username = USERNAME,
        
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
   // xTaskCreate(&mqtt_task, "mqtt_task", 4096, NULL, 9, NULL);
}
