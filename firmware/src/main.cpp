#include <cstdio>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

int main() {
  stdio_init_all();

  if (cyw43_arch_init() != 0) {
    while (true) {
      printf("Failed to start the wireless chip, which drives the LED\n");
      sleep_ms(1000);
    }
  }

  bool led_on = false;
  while (true) {
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    printf("LED %s\n", led_on ? "on" : "off");
    sleep_ms(500);
  }
}
