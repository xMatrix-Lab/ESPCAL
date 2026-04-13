#include "my_eth.h"
#include "my_wifi.h"
#include "my_spi.h"




static const char *TAG = "CH390D";

esp_netif_t *eth_netif;
uint8_t eth_port_cnt = 0;
esp_eth_handle_t eth_handles;



static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == ETH_EVENT) {
        switch (event_id) {
            case ETHERNET_EVENT_CONNECTED:
                ESP_LOGI(TAG, "ETHERNET_EVENT_CONNECTED: Link up");
                vTaskDelay(pdMS_TO_TICKS(100)); 
                esp_err_t dhcp_err = esp_netif_dhcpc_stop(eth_netif);
                if (dhcp_err != ESP_OK) {
                    ESP_LOGW(TAG, "DHCP client stop failed (err: %d), try to force static IP", dhcp_err);
                }
                
                esp_netif_ip_info_t ip_t = {0};
                ip_t.ip.addr = ipaddr_addr("192.168.2.11");       // 新网段IP
                ip_t.netmask.addr = ipaddr_addr("255.255.255.0"); // 子网掩码
                ip_t.gw.addr = ipaddr_addr("192.168.2.1");        // 网关
                // 强制设置静态IP，覆盖DHCP
                esp_err_t ip_err = esp_netif_set_ip_info(eth_netif, &ip_t);
                if (ip_err == ESP_OK) {
                    ESP_LOGI(TAG, "Static IP set successfully: 192.168.2.10");
                } else {
                    ESP_LOGE(TAG, "Failed to set static IP (err: %d)", ip_err);
                }
                break;
            case ETHERNET_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "ETHERNET_EVENT_DISCONNECTED: Link down");
                break;   
            default:
                break;
        }
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
}

void eth_ch390h_driver_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = 40,
        .miso_io_num = 39,
        .sclk_io_num = 41,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = 20 * 1000 * 1000,
        .spics_io_num = 42,
        .queue_size = 20,
    };
    eth_ch390_config_t ch390_config = ETH_CH390_DEFAULT_CONFIG(SPI3_HOST, &spi_devcfg);
    ch390_config.int_gpio_num = 38;
    gpio_install_isr_service(0);

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.rx_task_stack_size = 4096;
    esp_eth_mac_t *mac = esp_eth_mac_new_ch390(&ch390_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.autonego_timeout_ms = 0; // ENC28J60 doesn't support auto-negotiation
    phy_config.reset_gpio_num = -1; // ENC28J60 doesn't have a pin to reset internal PHY
    esp_eth_phy_t *phy = esp_eth_phy_new_ch390(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handles));


    //ESP_ERROR_CHECK(ethernet_init_all(&eth_handles, &eth_port_cnt));
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handles)));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    ESP_ERROR_CHECK(esp_eth_start(eth_handles));
}
    
void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int ip_protocol = 0;
    struct sockaddr_storage dest_addr;

    // Debug开关
    #define MY_TCP_DEBUG 1  // 1开启debug, 0关闭

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(8000);
    ip_protocol = IPPROTO_IP;

    int listen_sock = socket(AF_INET, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }

    err = listen(listen_sock, 5);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    #if MY_TCP_DEBUG
    printf("[TCP] Server started on port 8000\n");
    #endif

    while (1) {
        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        
        if (sock < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        int send_buf_size = 64 * 1024;
        setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));

        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);

        #if MY_TCP_DEBUG
        printf("[TCP] Client connected: %s, sock=%d\n", addr_str, sock);
        #endif

        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
        uint8_t sequence_counter = 0;
        bool connection_ok = true;
        bool sending_data = false;  // 标记是否正在发送数据
        char rx_buffer[128];
        int len;

        while (connection_ok) {
            // --- 1. 高优先级：接收并处理控制命令 (非阻塞) ---
            len = recv(sock, rx_buffer, sizeof(rx_buffer), 0);
            if (len > 0) { // 确保收到了数据
                #if MY_TCP_DEBUG
                printf("[TCP] Received command: 0x%02X, len=%d\n", rx_buffer[0], len);
                #endif
                // 尝试获取CMD互斥锁，如果获取不到，说明SPI任务正在使用，CMD可能丢失或延迟
                // 为了不阻塞TCP接收，这里使用短超时或非阻塞
                if(xSemaphoreTake(cmd_mutex, 0) == pdTRUE) {
                    memcpy(cmd, rx_buffer, (len < sizeof(cmd)) ? len : sizeof(cmd));
                    xSemaphoreGive(cmd_mutex);
                    // CMD已放入缓冲区，SPI任务下次循环会处理
                } else {
                     #if MY_TCP_DEBUG
                     printf("[TCP] Failed to take cmd_mutex, CMD might be delayed.\n");
                     #endif
                     // 可以选择丢弃CMD或稍后重试，这里选择丢弃
                }
            }
            else if (len == 0) {
                #if MY_TCP_DEBUG
                printf("[TCP] Client disconnected.\n");
                #endif
                connection_ok = false;
                break;
            }
            else if (len < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    #if MY_TCP_DEBUG
                    printf("[TCP] Recv error: errno %d\n", errno);
                    #endif
                    connection_ok = false;
                    break;
                }
                // EAGAIN/EWOULDBLOCK: 没有数据可读，正常情况
            }

            // --- 2. 检查是否开始发送数据 ---
            if(!sending_data) {
                // 尝试获取数据就绪信号量，如果获取不到，说明数据未准备好
                if(xSemaphoreTake(data_ready_semaphore, 0) == pdTRUE) { // 非阻塞获取
                    sending_data = true;
                    sequence_counter = 0;
                    #if MY_TCP_DEBUG
                    printf("[TCP] Start sending CSI data, total packets: %d\n", 8 * CSI_STACK_SIZE);
                    #endif
                }
            }

            // --- 3. 如果正在发送数据，尝试发送下一个包 ---
            if(sending_data) {
                struct __attribute__((packed)) {
                    uint8_t sequence_num;
                    my_csi_data_t data;
                } packet_with_seq = {
                    .sequence_num = sequence_counter,
                    .data = csi_data[sequence_counter / CSI_STACK_SIZE][sequence_counter % CSI_STACK_SIZE]
                };

                int sent = send(sock, &packet_with_seq, sizeof(packet_with_seq), 0);
                if (sent < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        #if MY_TCP_DEBUG
                        printf("[TCP] Send error: errno %d\n", errno);
                        #endif
                        connection_ok = false;
                        sending_data = false;
                    }
                    // 发送阻塞，下次循环重试
                }
                else {
                    sequence_counter++;


                    if(sequence_counter == 8 * CSI_STACK_SIZE) {
                        // 所有数据发送完成
                        #if MY_TCP_DEBUG
                        printf("[TCP] All data sent. Notifying SPI task.\n");
                        #endif
                        xSemaphoreGive(send_complete_semaphore); // 通知SPI任务
                        sending_data = false; // 退出发送状态
                    }
                }
            }
            
            vTaskDelay(10 / portTICK_PERIOD_MS); // 小延迟避免忙等待
        }

        if (sock >= 0) {
            #if MY_TCP_DEBUG
            printf("[TCP] Closing client socket %d\n", sock);
            #endif
            close(sock);
        }
    }
    
CLEAN_UP:
    close(listen_sock);
    #if MY_TCP_DEBUG
    printf("[TCP] Server task ended\n");
    #endif
    vTaskDelete(NULL);
}