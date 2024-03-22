#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"


#define INPUT_PIN GPIO_NUM_4
#define OUTPUT_PIN GPIO_NUM_2
#define WHEEL_CIRCUMFERENCE_CM 90
#define Z_MAX_TICK_DISTANCE_CM 40

// Global variables
volatile int64_t last_tick_time = 0;
volatile int64_t tick_interval = 0;

// Forward declarations
void adjust_output_ticks(int64_t tick_interval);
float calculate_speed(int64_t tick_interval);
int determine_number_of_synthetic_ticks(float speed);

static void IRAM_ATTR on_tick_detected(void* arg) {
    int64_t current_time = esp_timer_get_time();
    if (last_tick_time != 0) {
        tick_interval = current_time - last_tick_time;
        adjust_output_ticks(tick_interval);
    }
    last_tick_time = current_time;
}

void adjust_output_ticks(int64_t tick_interval) {
    float speed = calculate_speed(tick_interval);
    int num_ticks = determine_number_of_synthetic_ticks(speed);

    int64_t interval_between_ticks = tick_interval / num_ticks;
    for (int i = 0; i < num_ticks; ++i) {
        gpio_set_level(OUTPUT_PIN, 1);
        esp_rom_delay_us(interval_between_ticks / 2);
        gpio_set_level(OUTPUT_PIN, 0);
        if (i < num_ticks - 1) {
            esp_rom_delay_us(interval_between_ticks / 2);
        }
    }
}

float calculate_speed(int64_t tick_interval) {
    float time_hours = (float)tick_interval / (3600 * 1e6);
    float distance_per_tick_km = (float)WHEEL_CIRCUMFERENCE_CM / 100000.0;
    return distance_per_tick_km / time_hours;
}

int determine_number_of_synthetic_ticks(float speed) {
    if (WHEEL_CIRCUMFERENCE_CM > Z_MAX_TICK_DISTANCE_CM) {
        return (int)ceil((float)WHEEL_CIRCUMFERENCE_CM / Z_MAX_TICK_DISTANCE_CM);
    }
    return 1;
}

void app_main() {
    gpio_config_t input_pin_config = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << INPUT_PIN),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&input_pin_config);

    gpio_config_t output_pin_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << OUTPUT_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&output_pin_config);

    // Install GPIO interrupt service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT_PIN, on_tick_detected, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Sleep for 1 second
    }
}
