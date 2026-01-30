/*
 * SPDX-FileCopyrightText: 2021-2025
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"

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

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
  if (!message) {
    return ESP_OK;
  }

  if (message->info.dst_endpoint == APP_ENDPOINT_MAIN &&
    message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
    if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
      message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
      bool state = *(bool *)message->attribute.data.value;
      LOGI("Green %s", state ? "ON" : "OFF");
      led_set_green(state ? 0xff : 0);
      return ESP_OK;
    }
  } else if (message->info.dst_endpoint == APP_ENDPOINT_BLUE &&
    message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
    if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
      message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
      bool state = *(bool *)message->attribute.data.value;
      LOGI("Blue %s", state ? "ON" : "OFF");
      led_set_blue(state ? 0xff : 0);
      return ESP_OK;
    }
  }

  return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
  if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
    return zb_attribute_handler((const esp_zb_zcl_set_attr_value_message_t *)message);
  }
  return ESP_OK;
}

#define MS_TO_US(value) ((value)*1000)
#define SEC_TO_MS(value) ((value)*1000)
#define SEC_TO_US(value) MS_TO_US(SEC_TO_MS(value))

#define BUTTON_GPIO                 GPIO_NUM_9
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
  if (button_press_start != 0) {
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

static void setup_ep_main(esp_zb_ep_list_t* dev_ep_list) {
  esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

  esp_zb_on_off_cluster_cfg_t on_off_cfg = { .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE };

  esp_zb_cluster_list_add_basic_cluster(cl, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_identify_cluster(cl, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_on_off_cluster(cl, esp_zb_on_off_cluster_create(&on_off_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_endpoint_config_t ep_cfg = {
      .endpoint = APP_ENDPOINT_MAIN,
      .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
      .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
      .app_device_version = 0
  };
  esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);

  zcl_basic_manufacturer_info_t info = {
    .manufacturer_name = "\x0A""Mahtan-DIY",
    .model_identifier = "\x0D""Mahtan_OW_DIY",
  };

  esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, APP_ENDPOINT_MAIN, &info);
}

static void setup_ep_blue(esp_zb_ep_list_t* dev_ep_list) {
  esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

  esp_zb_on_off_cluster_cfg_t on_off_cfg = { .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE };

  esp_zb_cluster_list_add_on_off_cluster(cl, esp_zb_on_off_cluster_create(&on_off_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_endpoint_config_t ep_cfg = {
      .endpoint = APP_ENDPOINT_BLUE,
      .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
      .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
      .app_device_version = 0
  };
  esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
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

  setup_ep_main(dev_ep_list);
  setup_ep_blue(dev_ep_list);

  esp_zb_device_register(dev_ep_list);
  esp_zb_core_action_handler_register(zb_action_handler);
  esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

  ESP_ERROR_CHECK(esp_zb_start(false));

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
  xTaskCreate(button_task, "button", 2048, NULL, 4, NULL);
  xTaskCreate(led_task,       "led", 2048, NULL, 5, NULL);
  xTaskCreate(zigbee_task, "zigbee", 4096, NULL, 6, NULL);
}
