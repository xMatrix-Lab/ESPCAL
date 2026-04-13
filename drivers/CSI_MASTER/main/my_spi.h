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
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "my_wifi.h"

#define SENDER_HOST SPI2_HOST

#define GPIO_MOSI           13
#define GPIO_MISO           11
#define GPIO_SCLK           12

#define GPIO_A0           7
#define GPIO_A1           8
#define GPIO_A2           9
#define GPIO_E1           10

#define GPIO_TRIG      14

extern uint8_t sendbuf[8];
extern uint8_t recvbuf[8];
extern uint8_t cmd[128];
extern SemaphoreHandle_t cmd_mutex;
extern SemaphoreHandle_t data_ready_semaphore;  // 数据就绪信号量
extern SemaphoreHandle_t send_complete_semaphore; // 发送完成信号量

void spi_init(void);
void spi_set_cs(uint8_t slave);
void spi_master_recv_csi(void);
void spi_task(void *pvParameters);

#endif
