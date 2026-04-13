#ifndef MY_SPI_H_
#define MY_SPI_H_

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "my_wifi.h"

#define RCV_HOST    SPI2_HOST

#define GPIO_MOSI           13
#define GPIO_MISO           11
#define GPIO_SCLK           12
#define GPIO_CS             10

#define     LED1_PIN   3
#define     LED2_PIN   4
#define     V1_PIN     2
#define     V2_PIN     1
#define     TRIG_PIN   9

void spi_slave_init(void);
void spi_slave_send_csi(void *pvParameters);
void ant_set(int ant);
void gpio_init();
void IRAM_ATTR trig_isr_handler(void* arg);

#endif
