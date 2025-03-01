#include "ble_spp_server.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "adc.h"
#include "bldc_interface.h"
#include "bldc_interface_uart.h"
#include "driver/uart.h"
#include "soc/gpio_num.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"

static const char *TAG = "MAIN";



static mc_values stored_values;

// Add this function before app_main
static void send_packet(unsigned char *data, unsigned int len) {
    // Configure UART for VESC communication
    const uart_port_t uart_num = UART_NUM_1;  // Using UART1
    uart_write_bytes(uart_num, (const char*)data, len);
}

// Add this after the send_packet function and before app_main
static void bldc_values_received(mc_values *values) {
    // Store the values without logging
    stored_values = *values;
}

// Add this function before app_main
static void vesc_task(void *pvParameters) {
    while (1) {
        bldc_interface_get_values();
        vTaskDelay(pdMS_TO_TICKS(VESC_UPDATE_INTERVAL_MS));
        
    }
}


static void uart_rx_task(void *pvParameters) {
    uint8_t data[128];
    while (1) {
        int len = uart_read_bytes(UART_NUM_1, data, sizeof(data), pdMS_TO_TICKS(10));
        for (int i = 0; i < len; i++) {
            bldc_interface_uart_process_byte(data[i]);
        }
        bldc_interface_uart_run_timer();
    }
}

void print_stored_values(void) {
    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "VESC Data:");
    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "Input voltage: %.2f V", stored_values.v_in);
    ESP_LOGI(TAG, "Temperature MOS: %.2f °C", stored_values.temp_mos);
    ESP_LOGI(TAG, "Temperature Motor: %.2f °C", stored_values.temp_motor);
    ESP_LOGI(TAG, "Current Motor: %.2f A", stored_values.current_motor);
    ESP_LOGI(TAG, "Current Input: %.2f A", stored_values.current_in);
    ESP_LOGI(TAG, "ID: %.2f A", stored_values.id);
    ESP_LOGI(TAG, "IQ: %.2f A", stored_values.iq);
    ESP_LOGI(TAG, "RPM: %.1f RPM", stored_values.rpm);
    ESP_LOGI(TAG, "Duty cycle: %.1f %%", stored_values.duty_now * 100.0);
    ESP_LOGI(TAG, "Amp Hours Drawn: %.4f Ah", stored_values.amp_hours);
    ESP_LOGI(TAG, "Amp Hours Regen: %.4f Ah", stored_values.amp_hours_charged);
    ESP_LOGI(TAG, "Watt Hours Drawn: %.4f Wh", stored_values.watt_hours);
    ESP_LOGI(TAG, "Watt Hours Regen: %.4f Wh", stored_values.watt_hours_charged);
    ESP_LOGI(TAG, "Tachometer: %d counts", stored_values.tachometer);
    ESP_LOGI(TAG, "Tachometer Abs: %d counts", stored_values.tachometer_abs);
    ESP_LOGI(TAG, "PID Position: %.2f", stored_values.pid_pos);
    ESP_LOGI(TAG, "VESC ID: %d", stored_values.vesc_id);
    
    // Print fault code if any
    const char* fault_str = bldc_interface_fault_to_string(stored_values.fault_code);
    ESP_LOGI(TAG, "Fault Code: %s", fault_str);
    ESP_LOGI(TAG, "----------------------------------------");
}

// Add before app_main
mc_values* get_stored_vesc_values(void) {
    return &stored_values;
}

void app_main(void)
{
    esp_err_t ret;
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize LED
    ESP_ERROR_CHECK(led_init());

    // Initialize Bluetooth
    ret = ble_spp_server_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE SPP server: %s", esp_err_to_name(ret));
        return;
    }

    // Start the server
    ret = ble_spp_server_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE SPP server: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "BLE SPP server started successfully");

    adc_init();
    bldc_interface_uart_init(send_packet);
    bldc_interface_set_rx_value_func(bldc_values_received);
    
    // Create UART receive task
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);
    
    // Create task to periodically request VESC values
    xTaskCreate(vesc_task, "vesc_task", 2048, NULL, 5, NULL);
}
