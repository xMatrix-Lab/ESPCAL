/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"

#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "my_wifi.h"
#include "my_spi.h"

static const char *TAG = "csi_slave";


void app_main(void)
{
    //初始化FLASH
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    gpio_set_level(LED1_PIN, 0);
    gpio_set_level(LED2_PIN, 1);

    gpio_init();
    ant_set(1);

    wifi_init();
    csi_init();
    spi_slave_init();
    //wifi_ping_router_start();
    //wifi_scan_start();

    xTaskCreate(
        spi_slave_send_csi,           // 任务函数
        "spi_task",         // 任务名称
        4096,              // 堆栈大小
        NULL,              // 参数
        1,                 // 优先级
        NULL               // 不需要任务句柄
    );

}
