/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "my_eth.h"
#include "my_wifi.h"
#include "my_spi.h"

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    spi_init();
    wifi_init();
    eth_ch390h_driver_init();

    // 初始化信号量
    if(data_ready_semaphore == NULL) {
        data_ready_semaphore = xSemaphoreCreateBinary();
    }
    if(send_complete_semaphore == NULL) {
        send_complete_semaphore = xSemaphoreCreateBinary();
        xSemaphoreGive(send_complete_semaphore);
    }
    if(cmd_mutex == NULL) {
        cmd_mutex = xSemaphoreCreateMutex();
    }

    // spi_set_cs(7);

    xTaskCreate(
      tcp_server_task, 
      "tcp0_server", 
      8192, 
      NULL, 
      2, 
      NULL
    );

    xTaskCreate(
      spi_task,           // 任务函数
      "spi_task",         // 任务名称
      4096,              // 堆栈大小
      NULL,              // 参数
      1,                 // 优先级
      NULL               // 不需要任务句柄
    );

}
