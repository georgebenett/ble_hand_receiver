#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

// Pin definitions
#define UART1_TX_PIN         GPIO_NUM_5
#define UART1_RX_PIN         GPIO_NUM_18
#define LED_PIN              GPIO_NUM_12

// UART configurations
#define UART1_BAUD_RATE     115200
#define UART1_BUF_SIZE      256

