#include "my_spi.h"
#include "my_eth.h"

spi_device_handle_t spi;
uint8_t sendbuf[8];
uint8_t recvbuf[8];
uint8_t cmd[128];
uint8_t captrue_flag;
SemaphoreHandle_t cmd_mutex = NULL;
SemaphoreHandle_t data_ready_semaphore;  // 数据就绪信号量
SemaphoreHandle_t send_complete_semaphore; // 发送完成信号量

void spi_init(void)
{
    esp_err_t ret;
    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    
    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 10000000,
        .duty_cycle_pos = 128,      //50% duty cycle
        .mode = 0,
        .spics_io_num = -1,
        .cs_ena_posttrans = 0,      //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = 8,
    };

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret = spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(SENDER_HOST, &devcfg, &spi);
    assert(ret == ESP_OK);

    gpio_config_t A0_config = {
        .pin_bit_mask = (1ULL<<GPIO_A0),          //配置引脚
        .mode =GPIO_MODE_OUTPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_ENABLE,         //不使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_DISABLE            //不使能引脚中断
    };
    gpio_config_t A1_config = {
        .pin_bit_mask = (1ULL<<GPIO_A1),          //配置引脚
        .mode =GPIO_MODE_OUTPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_ENABLE,         //不使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_DISABLE            //不使能引脚中断
    };
    gpio_config_t A2_config = {
        .pin_bit_mask = (1ULL<<GPIO_A2),          //配置引脚
        .mode =GPIO_MODE_OUTPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_ENABLE,         //不使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_DISABLE            //不使能引脚中断
    };
    gpio_config_t E1_config = {
        .pin_bit_mask = (1ULL<<GPIO_E1),          //配置引脚
        .mode =GPIO_MODE_OUTPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_ENABLE,         //不使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_DISABLE            //不使能引脚中断
    };
    gpio_config_t TRIG_config = {
        .pin_bit_mask = (1ULL<<GPIO_TRIG),         //配置引脚
        .mode =GPIO_MODE_OUTPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_ENABLE,         //使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_DISABLE            //不使能引脚中断
    };
    gpio_config(&A0_config);
    gpio_config(&A1_config);
    gpio_config(&A2_config);
    gpio_config(&E1_config);
    gpio_config(&TRIG_config);
    gpio_set_level(GPIO_TRIG, 1);
    spi_set_cs(8);
}

void spi_set_cs(uint8_t slave)
{
    if(slave > 7){
        gpio_set_level(GPIO_E1, 1);  
        return;
    }
    gpio_set_level(GPIO_E1, 0);
    gpio_set_level(GPIO_A0, (slave >> 0) & 0x01);  
    gpio_set_level(GPIO_A1, (slave >> 1) & 0x01); 
    gpio_set_level(GPIO_A2, (slave >> 2) & 0x01);  
}

void spi_task(void *pvParameters)
{
    spi_transaction_t t = {0};
    bool waiting_for_tcp = false; // 标记是否在等待TCP发送完成
    // Debug开关
    #define SPI_DEBUG 1  // 1开启debug, 0关闭

    while(1) {
        uint8_t current_cmd = 0;
        // --- 高优先级：检查并处理CMD命令 ---
        // 使用非阻塞或短超时获取cmd_mutex，确保不会长时间阻塞数据流程
        if(xSemaphoreTake(cmd_mutex, 0) == pdTRUE) { 
            current_cmd = cmd[0];

            if(current_cmd == 0x01) { // 启用普通模式
                sendbuf[0] = 0x01;
                memcpy(&sendbuf[1], &cmd[1], 6); // MAC地址
                #if SPI_DEBUG
                printf("[SPI] CMD 0x01 - Set Normal mode, MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", MAC2STR(cmd+1));
                #endif
            }
            else if(current_cmd == 0x02) { // 启用校准模式
                sendbuf[0] = 0x02;
                memcpy(&sendbuf[1], my_mac, 6); // 自己的MAC
                #if SPI_DEBUG
                printf("[SPI] CMD 0x02 - Set Calibration mode, MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", MAC2STR(my_mac));
                #endif
            }
            else if(current_cmd == 0x03) { // 设置信道、带宽
                sendbuf[0] = 0x03;
                sendbuf[1] = cmd[1]; // 信道号
                sendbuf[2] = cmd[2]; // 带宽
                wifi_channel = cmd[1]; 
                wifi_bw = cmd[2];
                #if SPI_DEBUG
                printf("[SPI] CMD 0x03 - Set channel: %d, bandwidth: %d\n", cmd[1], cmd[2]);
                #endif
            }
            else if(current_cmd == 0xFF) { // 打开校准wifi
                #if SPI_DEBUG
                printf("[SPI] CMD 0xFF - Send calibration frames\n");
                #endif
                esp_wifi_start();
                esp_wifi_set_bandwidth(WIFI_IF_AP, wifi_bw);
                if(wifi_bw == 1)
                    esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE);
                else
                    esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_ABOVE);
                memset(cmd, 0, sizeof(cmd));
                xSemaphoreGive(cmd_mutex);
                continue; // 跳过本次循环剩余部分，重新检查CMD
            }
            else if(current_cmd == 0xFE){ //关闭校准wifi
                esp_wifi_stop();
                memset(cmd, 0, sizeof(cmd));
                xSemaphoreGive(cmd_mutex);
                continue; // 跳过本次循环剩余部分，重新检查CMD
            }

            // 清空cmd缓冲区，释放mutex
            memset(cmd, 0, sizeof(cmd));
            xSemaphoreGive(cmd_mutex);

            // --- 检查是否是需要重置系统的配置命令 ---
            if(current_cmd == 0x01 || current_cmd == 0x02 || current_cmd == 0x03) {
                #if SPI_DEBUG
                printf("[SPI] Processing config CMD %d, broadcasting to all slaves and resetting.\n", current_cmd);
                #endif
                // 1. 向所有从机发送配置命令
                for(uint8_t i = 0; i < 8; i++) {
                    memset(&t, 0, sizeof(t));
                    t.length = sizeof(sendbuf) * 8;
                    t.tx_buffer = sendbuf;
                    t.rx_buffer = NULL; // 配置命令通常不需要接收
                    spi_set_cs(i);
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                    spi_device_transmit(spi, &t);
                    spi_set_cs(8);
                }
                // 2. 发送TRIG信号重置所有从机
                gpio_set_level(GPIO_TRIG, 0);
                gpio_set_level(GPIO_TRIG, 1);

                // 3. 重置主机本地状态
                captrue_flag = 0; // 清零采集标志
                waiting_for_tcp = false; // 退出等待TCP状态

                #if SPI_DEBUG
                printf("[SPI] Config CMD %d completed, system reset.\n", current_cmd);
                #endif
                // 完成配置和重置，跳过本次循环剩余部分，下次循环继续等待CMD或开始新周期
                continue; 
            }
            // 如果不是配置命令，则sendbuf已准备好，可以用于后续的数据轮询
        } // END CMD Check

        // --- 数据采集/发送流程 (CMD处理完成后执行) ---
        // 如果正在等待TCP发送完成，则不进行数据采集
        if(waiting_for_tcp) {
            // 检查TCP是否发送完成
            if(xSemaphoreTake(send_complete_semaphore, 0) == pdTRUE) { // 非阻塞检查
                #if SPI_DEBUG
                printf("[SPI] TCP send complete. Sending TRIG to reset slaves.\n");
                #endif
                // TCP发送完成，发送TRIG重置从机
                gpio_set_level(GPIO_TRIG, 0);
                gpio_set_level(GPIO_TRIG, 1);
                
                // 重置本地状态
                captrue_flag = 0;
                waiting_for_tcp = false;

                #if SPI_DEBUG
                printf("[SPI] Reset complete, starting new cycle.\n");
                #endif
            }
            // 短暂延时，然后继续检查CMD
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue; // 跳过数据采集逻辑
        }

        // --- 正常数据采集流程 ---
        // 1. 轮询从机状态 (检查是否数据已满)
        memset(sendbuf, 0, sizeof(sendbuf));
        for(uint8_t i = 0; i < 8; i++) {
            if((captrue_flag & (1 << i)) == 0) { // 只检查尚未标记为完成的从机
                memset(&t, 0, sizeof(t));
                memset(recvbuf, 0, sizeof(recvbuf));
                t.length = sizeof(sendbuf) * 8; // 发送请求 (sendbuf[0] = 0x00)
                t.tx_buffer = sendbuf; // 使用之前设置好的sendbuf (通常是0x00)
                t.rx_buffer = recvbuf; // 接收从机的栈指针
                spi_set_cs(i);
                vTaskDelay(1 / portTICK_PERIOD_MS);
                spi_device_transmit(spi, &t);
                spi_set_cs(8);
                // 检查从机返回的栈指针
                if(recvbuf[0] == CSI_STACK_SIZE) { // 从机数据栈满了
                    captrue_flag |= (1 << i); // 标记该从机数据已准备好
                    #if SPI_DEBUG
                    printf("[SPI] Slave %d ready (ptr=%d), captrue_flag=0x%02X\n", i, recvbuf[0], captrue_flag);
                    #endif
                }
            }
        }

        // 2. 如果所有从机数据都准备好了
        if(captrue_flag == 0xFF) {
            #if SPI_DEBUG
            printf("[SPI] All slaves ready (0xFF). Collecting data now.\n");
            #endif

            // 3. 收集所有从机的CSI数据
            sendbuf[0] = 0x04; // 请求数据
            for(uint8_t i = 0; i < 8; i++) {
                // 发送请求数据命令 (0x04)
                memset(&t, 0, sizeof(t));
                t.length = sizeof(sendbuf) * 8;
                t.tx_buffer = sendbuf;
                t.rx_buffer = NULL; // 发送命令，不接收
                spi_set_cs(i);
                vTaskDelay(1 / portTICK_PERIOD_MS);
                spi_device_transmit(spi, &t);
                spi_set_cs(8);
                // 立即接收CSI数据块
                memset(&t, 0, sizeof(t));
                t.length = sizeof(csi_data[i]) * 8; // 接收整个数据块
                t.tx_buffer = NULL; // 只接收
                t.rx_buffer = csi_data[i];
                spi_set_cs(i);
                vTaskDelay(1 / portTICK_PERIOD_MS);
                spi_device_transmit(spi, &t);
                spi_set_cs(8);
            }
            memset(sendbuf, 0, sizeof(sendbuf));

            // 4. 数据收集完毕，通知TCP任务发送
            #if SPI_DEBUG
            printf("[SPI] All data collected. Notifying TCP task.\n");
            #endif
            xSemaphoreGive(data_ready_semaphore);
            waiting_for_tcp = true; // 设置等待TCP发送完成的标志
        }

        // 5. 短暂延时，然后进入下一次循环检查CMD
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
