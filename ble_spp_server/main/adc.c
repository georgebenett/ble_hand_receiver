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
static void request_values_task(void *pvParameters);
static void uart_send_function(unsigned char *data, unsigned int len);
static void configure_uart(void);
static void test_nunchuck_task(void *pvParameters);

esp_err_t adc_init(void)
{
    // Configure UART first
    configure_uart();
    
    // Initialize VESC UART interface
    bldc_interface_uart_init(uart_send_function);
    
    // Set up callbacks
    bldc_interface_set_rx_value_func(values_callback);
    
    // Create task to read UART data with increased stack size
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);

    // Create task to periodically request values
    xTaskCreate(request_values_task, "request_values", 2048, NULL, 5, NULL);
    
    // Create task for nunchuck testing
    xTaskCreate(test_nunchuck_task, "nunchuck_test", 2048, NULL, 5, NULL);

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

void values_callback(mc_values *values) {
    ESP_LOGI(ADC_TAG, "Values received:");
    ESP_LOGI(ADC_TAG, "RPM: %.1f", values->rpm);
    ESP_LOGI(ADC_TAG, "Current: %.2f A", values->current_motor);
    ESP_LOGI(ADC_TAG, "Duty: %.1f %%", values->duty_now * 100.0f);
    ESP_LOGI(ADC_TAG, "V_IN: %.1f V", values->v_in);
    ESP_LOGI(ADC_TAG, "Temp MOS: %.1f C", values->temp_mos);
    ESP_LOGI(ADC_TAG, "Temp Motor: %.1f C", values->temp_motor);
    ESP_LOGI(ADC_TAG, "Amp Hours: %.2f Ah", values->amp_hours);
    ESP_LOGI(ADC_TAG, "Amp Hours Charged: %.2f Ah", values->amp_hours_charged);
}

static void uart_rx_task(void *pvParameters)
{
    uint8_t data[64];
    while (1) {
        int len = uart_read_bytes(UART_NUM_1, data, sizeof(data), pdMS_TO_TICKS(20));
        if (len > 0) {
            ESP_LOGI(ADC_TAG, "Received %d bytes from UART", len);
            for (int i = 0; i < len; i++) {
                bldc_interface_uart_process_byte(data[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        bldc_interface_uart_run_timer();
    }
}

static void request_values_task(void *pvParameters)
{
    // Add a delay before starting to request values
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (1) {
        ESP_LOGI(ADC_TAG, "Requesting VESC values...");
        bldc_interface_get_values();
        vTaskDelay(pdMS_TO_TICKS(2000));
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
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_18, GPIO_NUM_17, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
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