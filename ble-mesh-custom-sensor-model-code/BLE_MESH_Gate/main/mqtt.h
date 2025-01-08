#ifndef _MQTT_H_
#define _MQTT_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include"mqtt_client.h"

#define MQTT_BROKER "broker.hivemq.com"
//#define MQTT_BROKER "https://mosquitto.org/"


// esp_mqtt_client_handle_t client;
// char Handler[50];
// uint8_t flagmqtt;



#define USERNAME "f0bsy2yvcw3l9d7x9v14"


#define TOPIC "v1/devices/me/telemetry"
#define Server "mqtt://demo.thingsboard.io"

void mqtt_client_sta(void);

#endif