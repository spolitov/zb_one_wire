/*
 * SPDX-FileCopyrightText: 2021-2025
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ds18x20.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "app.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl_utility.h"
#include "led.h"

#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"

#if !defined CONFIG_ZB_ZCZR
#error "Enable Router: set CONFIG_ZB_ZCZR=y (menuconfig)"
#endif

#define LOGI(...) ESP_LOGI("ZBOW", __VA_ARGS__)
#define LOGW(...) ESP_LOGW("ZBOW", __VA_ARGS__)
#define RETURN_ON_ERROR(expr, ...) ESP_RETURN_ON_ERROR(expr, "ZBOW", __VA_ARGS__)

static void zb_on_off_handler(uint8_t endpoint, bool state) {
  LOGI("Light %d: %s", (int)endpoint, state ? "ON" : "OFF");
  if (endpoint == APP_ENDPOINT_GREEN) {
    led_set_green(state ? 0xff : 0);
    return;
  }
  if (endpoint == APP_ENDPOINT_BLUE) {
    led_set_blue(state ? 0xff : 0);
    return;
  }
}

static void zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
  if (!message) {
    return;
  }

  if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
      message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
      message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
    bool state = *(bool *)message->attribute.data.value;
    zb_on_off_handler(message->info.dst_endpoint, state);
    return;
  }

  LOGW("Wrong attribute, endpoint %d, cluster 0x%04x, attribute 0x%04x, type 0x%02x",
       message->info.dst_endpoint, message->info.cluster, message->attribute.id,
       message->attribute.data.type);
}

static EventGroupHandle_t ow_event_group;
#define OW_RESCAN_BIT (1 << 0)

static void zb_custom_cluster_req(const esp_zb_zcl_custom_cluster_command_message_t* message) {
  if (message->info.dst_endpoint == APP_ENDPOINT_MAIN &&
      message->info.cluster == APP_CONTROL_CLUSTER_ID &&
      message->info.command.id == APP_COMMAND_RESCAN) {
    xEventGroupSetBits(ow_event_group, OW_RESCAN_BIT);
    return;
  }
  LOGW("Unexpected command, endpoint %d, cluster 0x%04x, command 0x%02x, data size %d",
       message->info.dst_endpoint, message->info.cluster, message->info.command.id,
       message->data.size);
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
  switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
      zb_attribute_handler((const esp_zb_zcl_set_attr_value_message_t *)message);
      break;
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
      zb_custom_cluster_req((const esp_zb_zcl_custom_cluster_command_message_t*)message);
      break;
    default:
      break;
  }
  return ESP_OK;
}

#define MS_TO_US(value) ((value)*1000)
#define SEC_TO_MS(value) ((value)*1000)
#define SEC_TO_US(value) MS_TO_US(SEC_TO_MS(value))

#define BUTTON_GPIO                 GPIO_NUM_9
#define ONE_WIRE_GPIO               GPIO_NUM_5

#define FACTORY_RESET_TIME_US       SEC_TO_US(6)

static volatile int64_t button_press_start = 0;

static void button_task(void *pv) {
  gpio_config_t io = {
    .pin_bit_mask = 1ULL << BUTTON_GPIO,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io);

  LOGI("Button GPIO%d", BUTTON_GPIO);

  for (;;) {
    if (gpio_get_level(BUTTON_GPIO) == 0) {
      if (button_press_start == 0) {
        button_press_start = esp_timer_get_time();
        LOGI("Button pressed.");
      } else if (esp_timer_get_time() - button_press_start >= FACTORY_RESET_TIME_US) {
        LOGW("Button factory reset.");
        esp_zb_factory_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
      }
    } else {
      button_press_start = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static int64_t led_switch_time = 0;
static bool led_state = true;
static volatile bool zb_init_done = false;

static int led_handle_blink(int64_t interval_us) {
  if (esp_timer_get_time() - led_switch_time >= interval_us) {
    return !led_state;
  }
  return led_state;
}

typedef enum {
  ZB_STATUS_IDLE,
  ZB_STATUS_JOIN_IN_PROGRESS,
  ZB_STATUS_JOINED,
} ZBStatus;

ZBStatus zb_status = ZB_STATUS_IDLE;

static uint16_t* identify_time_ptr = NULL;

static bool led_desired_state() {
  if (button_press_start != 0 && zb_status != ZB_STATUS_JOIN_IN_PROGRESS) {
    return true;
  }
  if (zb_status == ZB_STATUS_JOINED) {
    if (identify_time_ptr == NULL) {
      if (esp_zb_lock_acquire(portMAX_DELAY)) {
        esp_zb_zcl_attr_t* attribute = esp_zb_zcl_get_attribute(
            APP_ENDPOINT_MAIN, ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID);
        esp_zb_lock_release();
        if (!attribute) {
          LOGW("Identify time attribute not found");
        } else if (attribute->type != ESP_ZB_ZCL_ATTR_TYPE_U16) {
          LOGW("Identify time wrong attribute type %d", (int)attribute->type);
        } else {
          identify_time_ptr = (uint16_t*)attribute->data_p;
        }
      }
    }
    if (identify_time_ptr && *identify_time_ptr > 0) {
      return led_handle_blink(MS_TO_US(500));
    }

    return false;
  }
  return led_handle_blink(
      (zb_status == ZB_STATUS_JOIN_IN_PROGRESS || led_state) ? MS_TO_US(250) : MS_TO_US(2500));
}

static void led_task(void *pv) {
  led_switch_time = esp_timer_get_time();
  for (;;) {
    if (zb_init_done) {
      bool desired_state = led_desired_state();
      if (desired_state != led_state) {
        led_state = desired_state;
        led_switch_time = esp_timer_get_time();
        led_set_red(led_state ? 0xff : 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* ---------------- Commissioning helper ---------------- */
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  if (esp_zb_bdb_start_top_level_commissioning(mode_mask) != ESP_OK) {
    LOGW("Failed to start Zigbee commissioning");
  }
}

static void basic_publish_metadata_ep_main(void) {
  const char* date_code = "\x0A""2026-01-27";
  const char* sw_build  = "\x0B""ZB_OW-1.0.0";
  uint8_t power_src = ESP_ZB_ZCL_BASIC_POWER_SOURCE_MAINS_SINGLE_PHASE;

  if (esp_zb_lock_acquire(portMAX_DELAY)) {
    esp_zb_zcl_set_attribute_val(APP_ENDPOINT_MAIN,
                                 ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID,
                                 &power_src, false);

    esp_zb_zcl_set_attribute_val(APP_ENDPOINT_MAIN,
                                 ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID,
                                 (void *)date_code, false);
    esp_zb_zcl_set_attribute_val(APP_ENDPOINT_MAIN,
                                 ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID,
                                 (void *)sw_build, false);

    esp_zb_lock_release();
  }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct) {
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = *signal_struct->p_app_signal;

  switch (sig_type) {
  case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
    LOGI("Initialize Zigbee stack");
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
    break;

  case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
  case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    if (err_status == ESP_OK) {
      basic_publish_metadata_ep_main();

      LOGI("Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
      if (esp_zb_bdb_is_factory_new()) {
        LOGI("Start network steering");
        zb_status = ZB_STATUS_JOIN_IN_PROGRESS;
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      } else {
        LOGI("Device rebooted");
        zb_status = ZB_STATUS_JOINED;
      }
    } else {
      LOGW("Failed to initialize Zigbee stack (%s)", esp_err_to_name(err_status));
    }
    break;

  case ESP_ZB_BDB_SIGNAL_STEERING:
    if (err_status == ESP_OK) {
      zb_status = ZB_STATUS_JOINED;
      esp_zb_ieee_addr_t extended_pan_id;
      esp_zb_get_extended_pan_id(extended_pan_id);
      LOGI("Joined network successfully (ExtPAN:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN:0x%04hx, Ch:%d, Short:0x%04hx)",
               extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
               extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
               esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
    } else {
      LOGW("Network steering not successful (%s)", esp_err_to_name(err_status));
      esp_zb_scheduler_alarm(
          bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
    }
    break;

  default:
    LOGI("ZDO signal: %s (0x%x), status: %s",
         esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
    break;
  }
}

static esp_zb_endpoint_config_t make_ep_config(uint8_t endpoint, uint16_t device_id) {
  esp_zb_endpoint_config_t ep_cfg = {
    .endpoint = endpoint,
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
    .app_device_id = device_id,
    .app_device_version = 0
  };
  return ep_cfg;
}

static void setup_ep_main(esp_zb_ep_list_t* dev_ep_list) {
  esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

  esp_zb_cluster_list_add_basic_cluster(cl, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_identify_cluster(cl, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_attribute_list_t *control_attr_list = esp_zb_zcl_attr_list_create(APP_CONTROL_CLUSTER_ID);
  esp_zb_cluster_list_add_custom_cluster(cl, control_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_endpoint_config_t ep_cfg = make_ep_config(APP_ENDPOINT_MAIN, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID);
  esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);

  zcl_basic_manufacturer_info_t info = {
    .manufacturer_name = "\x0A""Mahtan-DIY",
    .model_identifier = "\x0D""Mahtan_OW_DIY",
  };

  esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, APP_ENDPOINT_MAIN, &info);
}

static void setup_ep_light(esp_zb_ep_list_t* dev_ep_list, uint8_t endpoint) {
  esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

  esp_zb_on_off_cluster_cfg_t on_off_cfg = { .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE };

  esp_zb_cluster_list_add_on_off_cluster(cl, esp_zb_on_off_cluster_create(&on_off_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_endpoint_config_t ep_cfg = make_ep_config(endpoint, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID);
  esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
}

static onewire_addr_t ow_addr_list[APP_MAX_TEMP_SENSORS];

static void complete_zb_string(char* buffer) {
  buffer[0] = strlen(buffer + 1);
}

static char* one_wire_addr_to_string(char* buffer, uint64_t value) {
  if (value == ONEWIRE_NONE) {
    strcpy(buffer, "NONE");
    return buffer;
  }

  for (int i = 0; i < 8; i++) {
    sprintf(buffer + i * 2, "%02X", (uint8_t)(value >> i * 8));
  }
  return buffer;
}

static char* one_wire_addr_to_zb_string(char* buffer, uint64_t value) {
  one_wire_addr_to_string(buffer + 1, value);
  complete_zb_string(buffer);
  return buffer;
}

static void setup_ep_temperature(esp_zb_ep_list_t* dev_ep_list, nvs_handle_t nvs_handle, size_t idx) {
  esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

  esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
    .measured_value = ESP_ZB_ZCL_VALUE_S16_NaS,
    .min_value = ESP_ZB_ZCL_VALUE_S16_NaS,
    .max_value = ESP_ZB_ZCL_VALUE_S16_NaS,
  };

  esp_zb_attribute_list_t *temp_attr_list =
    esp_zb_temperature_meas_cluster_create(&temp_cfg);

  ow_addr_list[idx] = ONEWIRE_NONE;
  char key[0x10];
  char buffer[0x20];
  sprintf(key, "ow_address_%u", idx);
  nvs_get_u64(nvs_handle, key, ow_addr_list + idx);

  ESP_ERROR_CHECK(esp_zb_cluster_add_manufacturer_attr(
      temp_attr_list,
      ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
      APP_ATTR_ONE_WIRE_ADDRESS,
      APP_MANUFACTURER_CODE,
      ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      one_wire_addr_to_zb_string(buffer, ow_addr_list[idx])
  ));

  esp_zb_cluster_list_add_temperature_meas_cluster(cl, temp_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_endpoint_config_t ep_cfg = make_ep_config(APP_ENDPOINT_TEMPERATURE_0 + idx, ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID);
  esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
}

static esp_err_t one_wire_rescan() {
  LOGI("Rescan");
  onewire_addr_t old_addr_list[APP_MAX_TEMP_SENSORS];
  memcpy(old_addr_list, ow_addr_list, sizeof(old_addr_list));
  onewire_addr_t new_addr_list[APP_MAX_TEMP_SENSORS];
  size_t found = 0;
  RETURN_ON_ERROR(
      ds18x20_scan_devices(ONE_WIRE_GPIO, new_addr_list, APP_MAX_TEMP_SENSORS, &found),
      "Scan failed");
  LOGI("Found devices: %u", found);
  if (found > APP_MAX_TEMP_SENSORS) {
    found = APP_MAX_TEMP_SENSORS;
  }

  for (size_t i = 0; i != APP_MAX_TEMP_SENSORS; ++i) {
    ow_addr_list[i] = ONEWIRE_NONE;
    if (old_addr_list[i] == ONEWIRE_NONE) {
      continue;
    }
    for (size_t j = 0; j != found; ++j) {
      if (old_addr_list[i] == new_addr_list[j]) {
        ow_addr_list[i] = old_addr_list[i];
        new_addr_list[j] = ONEWIRE_NONE;
      }
    }
  }
  for (size_t j = 0; j != found; ++j) {
    if (new_addr_list[j] == ONEWIRE_NONE) {
      continue;
    }
    for (size_t i = 0; i != APP_MAX_TEMP_SENSORS; ++i) {
      if (ow_addr_list[i] == ONEWIRE_NONE) {
        ow_addr_list[i] = new_addr_list[j];
        break;
      }
    }
  }

  nvs_handle_t nvs_handle;
  RETURN_ON_ERROR(nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle));
  char buffer[0x20];
  for (size_t i = 0; i != APP_MAX_TEMP_SENSORS; ++i) {
    LOGI("Device %d address: %s", i, one_wire_addr_to_string(buffer, ow_addr_list[i]));
    char key[0x10];
    sprintf(key, "ow_address_%u", i);
    nvs_set_u64(nvs_handle, key, ow_addr_list[i]);
  }
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);

  if (esp_zb_lock_acquire(portMAX_DELAY)) {
    for (size_t i = 0; i != APP_MAX_TEMP_SENSORS; ++i) {
      uint8_t endpoint = APP_ENDPOINT_TEMPERATURE_0 + i;
      ESP_ERROR_CHECK(esp_zb_zcl_set_manufacturer_attribute_val(
          endpoint, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, APP_MANUFACTURER_CODE, APP_ATTR_ONE_WIRE_ADDRESS,
          one_wire_addr_to_zb_string(buffer, ow_addr_list[i]),
          false));

      esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
      memset(&report_attr_cmd, 0, sizeof(report_attr_cmd));
      report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
      report_attr_cmd.attributeID = APP_ATTR_ONE_WIRE_ADDRESS;
      report_attr_cmd.manuf_code = APP_MANUFACTURER_CODE;
      report_attr_cmd.manuf_specific = 1;
      report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
      report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
      report_attr_cmd.zcl_basic_cmd.src_endpoint = endpoint;

      esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
      if (err != ESP_OK) {
        LOGW("Report device id in %d failed: %s", endpoint, esp_err_to_name(err));
      }
    }

    esp_zb_lock_release();
  }

  return ESP_OK;
}

static void one_wire_read() {
  onewire_addr_t addr_list[APP_MAX_TEMP_SENSORS];
  size_t indexes[APP_MAX_TEMP_SENSORS];
  float result[APP_MAX_TEMP_SENSORS];
  size_t count = 0;
  for (size_t i = 0; i != APP_MAX_TEMP_SENSORS; ++i) {
    if (ow_addr_list[i] != ONEWIRE_NONE) {
      addr_list[count] = ow_addr_list[i];
      indexes[count] = i;
      ++count;
    }
  }
  if (!count) {
    return;
  }
  esp_err_t err = ds18x20_measure_and_read_multi(ONE_WIRE_GPIO, addr_list, count, result);
  if (err != ESP_OK) {
    LOGW("Failed to read temperature: %s", esp_err_to_name(err));
    return;
  }
  if (esp_zb_lock_acquire(portMAX_DELAY)) {
    for (size_t i = 0; i != count; ++i) {
      uint8_t endpoint = APP_ENDPOINT_TEMPERATURE_0 + indexes[i];
      int16_t value = result[i] * 100;
      esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
          endpoint,
          ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
          ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
          &value,
          false);
      if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        LOGW("Failed to update temperature at %d to %d: %02x", endpoint, value, status);
      }

      esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
      memset(&report_attr_cmd, 0, sizeof(report_attr_cmd));
      report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
      report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
      report_attr_cmd.manuf_specific = 0;
      report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
      report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
      report_attr_cmd.zcl_basic_cmd.src_endpoint = endpoint;

      esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
      if (err != ESP_OK) {
        LOGW("Report temperature in %d failed: %s", endpoint, esp_err_to_name(err));
      }
    }
    esp_zb_lock_release();
  }
}

static void one_wire_task(void* args) {
  for (;;) {
    EventBits_t bits = xEventGroupWaitBits(ow_event_group, OW_RESCAN_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
    if (bits & OW_RESCAN_BIT) {
      one_wire_rescan();
    }
    if (!bits) {
      one_wire_read();
    }
  }
}

static void zigbee_task(void *args) {
  esp_zb_cfg_t zb_nwk_cfg = {
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
    .install_code_policy = false,
    .nwk_cfg.zczr_cfg = {
      .max_children = 10,
    },
  };

  esp_zb_init(&zb_nwk_cfg);

  esp_zb_ep_list_t* dev_ep_list = esp_zb_ep_list_create();

  nvs_handle_t nvs_handle;
  ESP_ERROR_CHECK(nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle));
  setup_ep_main(dev_ep_list);
  setup_ep_light(dev_ep_list, APP_ENDPOINT_GREEN);
  setup_ep_light(dev_ep_list, APP_ENDPOINT_BLUE);
  for (size_t i = 0; i != APP_MAX_TEMP_SENSORS; ++i) {
    setup_ep_temperature(dev_ep_list, nvs_handle, i);
  }
  nvs_close(nvs_handle);

  esp_zb_device_register(dev_ep_list);
  esp_zb_core_action_handler_register(zb_action_handler);

  esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_zcl_temp_measurement_init_server();

  zb_init_done = true;
  esp_zb_stack_main_loop();
}

void app_main() {
  esp_err_t nvs_rc = nvs_flash_init();
  if (nvs_rc == ESP_ERR_NVS_NO_FREE_PAGES || nvs_rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_rc = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_rc);

  led_init();

  led_set_red(0xff);

  ow_event_group = xEventGroupCreate();

  xTaskCreate(button_task, "button", 2048, NULL, 4, NULL);
  xTaskCreate(led_task, "led", 2048, NULL, 5, NULL);
  xTaskCreate(one_wire_task, "one_wire", 4096, NULL, 6, NULL);
  xTaskCreate(zigbee_task, "zigbee", 4096, NULL, 7, NULL);
}
