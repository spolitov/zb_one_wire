#pragma once

#include "esp_zigbee_core.h"
#include "driver/gpio.h"

#define APP_ENDPOINT_MAIN 1
#define APP_ENDPOINT_GREEN 2
#define APP_ENDPOINT_BLUE 3

#define APP_ENDPOINT_TEMPERATURE_0 10
#define APP_MAX_TEMP_SENSORS 10

#define APP_CONTROL_CLUSTER_ID    0xFC00
#define APP_MANUFACTURER_CODE     0x1234
#define APP_COMMAND_RESCAN        0x80
#define APP_ATTR_ONE_WIRE_ADDRESS 0xF001
#define APP_NVS_NAMESPACE         "storage"
