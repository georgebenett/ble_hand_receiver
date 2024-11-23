#include "adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <stdio.h>

#define ADC_TAG "ADC"

uint16_t current_adc_value = THROTTLE_NEUTRAL_VALUE;

TaskHandle_t adc_print_task_handle = NULL;
static TimerHandle_t adc_timeout_timer = NULL;
static bool timeout_monitoring_active = false;

esp_err_t adc_init(void)
{
    // Create the timeout timer
    adc_timeout_timer = xTimerCreate(
        "adc_timeout",
        pdMS_TO_TICKS(ADC_TIMEOUT_MS),
        pdTRUE,              // Auto reload
        NULL,
        adc_timeout_callback
    );

    if (adc_timeout_timer == NULL) {
        ESP_LOGE(ADC_TAG, "Failed to create ADC timeout timer");
        return ESP_FAIL;
    }

    // Create ADC print task
    if (xTaskCreate(adc_print_task, "adc_print", 2048, NULL, 5, &adc_print_task_handle) != pdPASS) {
        ESP_LOGE(ADC_TAG, "Failed to create ADC print task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void adc_update_value(uint16_t value)
{
    current_adc_value = value;
    ESP_LOGD(ADC_TAG, "%d", value);
}

void adc_reset_value(void)
{
    current_adc_value = THROTTLE_NEUTRAL_VALUE;
}

void adc_timeout_callback(TimerHandle_t xTimer)
{
    adc_reset_value();
}

void adc_print_task(void *pvParameters)
{
    while (1) {
        ESP_LOGI(ADC_TAG, "%d", current_adc_value);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void adc_reset_timeout(void)
{
    if (timeout_monitoring_active && adc_timeout_timer != NULL) {
        if (xTimerReset(adc_timeout_timer, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(ADC_TAG, "Failed to reset ADC timeout timer");
        }
    }
}

void adc_start_timeout_monitor(void)
{
    if (adc_timeout_timer != NULL) {
        if (xTimerStart(adc_timeout_timer, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(ADC_TAG, "Failed to start ADC timeout timer");
        } else {
            timeout_monitoring_active = true;
            ESP_LOGI(ADC_TAG, "ADC timeout monitoring started");
        }
    }
}

void adc_stop_timeout_monitor(void)
{
    if (adc_timeout_timer != NULL) {
        if (xTimerStop(adc_timeout_timer, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(ADC_TAG, "Failed to stop ADC timeout timer");
        } else {
            timeout_monitoring_active = false;
            ESP_LOGI(ADC_TAG, "ADC timeout monitoring stopped");
        }
    }
}