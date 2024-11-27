#include "adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include <stdio.h>
#include "bldc_interface_uart.h"
#include "bldc_interface.h"
#include "soc/gpio_num.h"


#define ADC_TAG "ADC"

uint16_t current_adc_value = THROTTLE_NEUTRAL_VALUE;

static TimerHandle_t adc_timeout_timer = NULL;
static bool timeout_monitoring_active = false;

static void adc_timeout_callback(TimerHandle_t xTimer);
static void uart_rx_task(void *pvParameters);
static void uart_send_function(unsigned char *data, unsigned int len);
static void configure_uart(void);
static void test_nunchuck_task(void *pvParameters);

esp_err_t adc_init(void)
{
    // Configure UART first
    configure_uart();
    
    // Initialize VESC UART interface
    bldc_interface_uart_init(uart_send_function);
    
    // Create task to read UART data with increased stack size
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);
    
    // Create task for nunchuck testing
    //xTaskCreate(test_nunchuck_task, "nunchuck_test", 2048, NULL, 5, NULL);

    ESP_LOGI(ADC_TAG, "ADC and VESC communication initialized");
    
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

static void uart_rx_task(void *pvParameters)
{
    uint8_t data[128];
    while (1) {
        int len = uart_read_bytes(UART_NUM_1, data, sizeof(data), pdMS_TO_TICKS(10));
        for (int i = 0; i < len; i++) {
            bldc_interface_uart_process_byte(data[i]);
        }
        bldc_interface_uart_run_timer();
    }
}

static void uart_send_function(unsigned char *data, unsigned int len)
{
    uart_write_bytes(UART_NUM_1, (const char*)data, len);
}

static void configure_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    
    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_10, GPIO_NUM_9, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0));
}

static void test_nunchuck_task(void *pvParameters) {
    uint8_t y_value = 0;
    bool increasing = true;
    
    while (1) {
        // Create packet for nunchuck data
        uint8_t buffer[5];
        int32_t ind = 0;
        
        buffer[ind++] = COMM_SET_CHUCK_DATA;  // Command ID
        buffer[ind++] = 128;                  // x-axis centered
        buffer[ind++] = y_value;              // y-axis variable
        buffer[ind++] = 0;                    // buttons released
        buffer[ind++] = 0;                    // extension data
        
        // Send the packet
        bldc_interface_send_packet(buffer, ind);
        
        // Update y_value
        if (increasing) {
            y_value++;
            if (y_value >= 255) {
                increasing = false;
            }
        } else {
            y_value--;
            if (y_value <= 0) {
                increasing = true;
            }
        }
        
        // Log the current value
        ESP_LOGI(ADC_TAG, "%d", y_value);
        
        // Delay before next update
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms delay for smooth ramping
    }
}