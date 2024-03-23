#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

// GPIO Pin Definitions
#define INPUT_PIN GPIO_NUM_4          // GPIO pin for receiving the real wheel tick signal.
#define OUTPUT_PIN GPIO_NUM_2         // GPIO pin for outputting synthetic wheel tick signals.
#define LED_INPUT_PIN GPIO_NUM_5      // GPIO pin connected to an LED indicating input signal detection.
#define LED_OUTPUT_PIN GPIO_NUM_18    // GPIO pin connected to an LED indicating output signal generation.

// Wheel Tick and Distance Constants
#define WHEEL_CIRCUMFERENCE_CM 90     // Circumference of the wheel in centimeters.
#define Z_MAX_TICK_DISTANCE_CM 40     // Maximum distance per tick accepted by the receiver in centimeters.

// Global Variables for Timing
volatile int64_t last_tick_time = 0;  // Timestamp of the last detected input tick.
volatile int64_t tick_interval = 0;   // Time interval between consecutive input ticks.

// ESP Logging Tag
static const char *TAG = "WheelTickConverter";

// Task Handle for post ISR work
static TaskHandle_t post_isr_handle = NULL;

/**
 * @brief Task that handles actions post-ISR
 *
 * This task waits for notifications from the ISR and then performs actions that are not safe to do in ISR context,
 * such as logging and using vTaskDelay.
 */
static void post_isr_task(void *arg) {
    uint32_t ulNotifiedValue;
    while (1) {
        // Wait indefinitely for the notification from the ISR
        if (xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY) == pdTRUE) {
            // Log the tick detection and its interval
            ESP_LOGI(TAG, "Tick detected. Interval: %lld us", tick_interval);

            // Indicate input pulse detection by toggling the LED
            gpio_set_level(LED_INPUT_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));  // Delay is safe here
            gpio_set_level(LED_INPUT_PIN, 0);
        }
    }
}

/**
 * @brief ISR for input tick detection
 *
 * This ISR is triggered by a rising edge on the INPUT_PIN. It calculates the interval between
 * consecutive ticks and then notifies the post-ISR task to perform logging and other actions.
 *
 * @param arg Unused parameter, present to match ISR signature.
 */
static void IRAM_ATTR on_tick_detected(void* arg) {
    int64_t current_time = esp_timer_get_time(); // Get current time in microseconds
    if (last_tick_time != 0) {
        tick_interval = current_time - last_tick_time; // Calculate the interval between ticks

        // Notify the post-ISR task
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(post_isr_handle, 1, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    last_tick_time = current_time; // Update the timestamp of the last tick
}

/**
 * @brief Main application entry point
 *
 * Initializes the system, configures GPIO pins for input detection and output signal generation,
 * and sets up the interrupt service routine for the input tick detection.
 */
void app_main(void) {
    ESP_LOGI(TAG, "Initializing Wheel Tick Converter...");

    // Configure INPUT_PIN as input with an interrupt on the rising edge
    gpio_config_t input_pin_config = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << INPUT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&input_pin_config);

    // Configure OUTPUT_PIN as output
    gpio_config_t output_pin_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << OUTPUT_PIN),
    };
    gpio_config(&output_pin_config);

    // Initialize and configure LED pins as outputs
    gpio_set_direction(LED_INPUT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_OUTPUT_PIN, GPIO_MODE_OUTPUT);

    // Create the post-ISR task
    xTaskCreate(post_isr_task, "post_isr_task", 2048, NULL, 10, &post_isr_handle);

    // Install GPIO interrupt service and add the ISR handler for
    // input tick detection.
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM); // Use IRAM flag for ISR routines
    gpio_isr_handler_add(INPUT_PIN, on_tick_detected, NULL); // Add ISR handler for the designated input pin

    while (true) {
        // Main loop can perform other tasks or simply idle, waiting for ISR notifications.
        // In a more complex application, this loop might also check statuses,
        // manage power modes, or perform other periodic checks.
        vTaskDelay(pdMS_TO_TICKS(1000));  // Sleep to reduce load, waking periodically.
    }
}
