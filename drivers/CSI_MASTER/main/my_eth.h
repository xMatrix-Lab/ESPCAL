#ifndef __MY_ETH_H__
#define __MY_ETH_H__

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
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "sys/socket.h"

#include "esp_eth_phy_ch390.h"
#include "esp_eth_mac_ch390.h"

#include "my_wifi.h"


void eth_ch390h_driver_init(void);
void tcp_server_task(void *pvParameters);



#endif
