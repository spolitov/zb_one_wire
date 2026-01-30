#include "led.h"

#include "led_strip.h"

static led_strip_handle_t led_strip;
static uint8_t red = 0;
static uint8_t green = 0;
static uint8_t blue = 0;

static void led_refresh() {
  ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, red, green, blue));
  ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

void led_set_red(uint8_t value) {
  red = value;
  led_refresh();
}

void led_set_green(uint8_t value) {
  green = value;
  led_refresh();
}

void led_set_blue(uint8_t value) {
  blue = value;
  led_refresh();
}

#define LED_GPIO 8

void led_init() {
  led_strip_config_t led_strip_conf = {
    .max_leds = 1,
    .strip_gpio_num = LED_GPIO,
  };
  led_strip_rmt_config_t rmt_conf = {
    .resolution_hz = 10 * 1000 * 1000, // 10MHz
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_conf, &rmt_conf, &led_strip));
}
