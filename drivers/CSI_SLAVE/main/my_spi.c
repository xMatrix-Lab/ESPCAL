#include "my_spi.h"
#include "esp_radar.h"
#include "esp_csi_gain_ctrl.h"

extern int s_count;

void spi_slave_init(void)
{
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
    };

    //Initialize SPI slave interface
    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

}

void gpio_init()
{
    gpio_config_t V1_config = {
        .pin_bit_mask = (1ULL<<V1_PIN),          //配置引脚
        .mode =GPIO_MODE_OUTPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE,        //不使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_DISABLE            //不使能引脚中断
    };
    gpio_config_t V2_config = {
        .pin_bit_mask = (1ULL<<V2_PIN),          //配置引脚
        .mode =GPIO_MODE_OUTPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE,        //不使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_DISABLE            //不使能引脚中断
    };
    gpio_config_t led1_config = {
        .pin_bit_mask = (1ULL<<LED1_PIN),          //配置引脚
        .mode =GPIO_MODE_OUTPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_ENABLE,        //不使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_DISABLE            //不使能引脚中断
    };
    gpio_config_t led2_config = {
        .pin_bit_mask = (1ULL<<LED2_PIN),          //配置引脚
        .mode =GPIO_MODE_OUTPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_ENABLE,        //使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_DISABLE            //不使能引脚中断
    };
    gpio_config_t trig_config = {
        .pin_bit_mask = (1ULL<<TRIG_PIN),          //配置引脚
        .mode = GPIO_MODE_INPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_ENABLE,        //使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_NEGEDGE            //引脚中断
    };
    gpio_install_isr_service(0);
    gpio_config(&led1_config);
    gpio_config(&led2_config);
    gpio_config(&V2_config);
    gpio_config(&V1_config);
    gpio_config(&trig_config);
    gpio_isr_handler_add(TRIG_PIN, trig_isr_handler, NULL);
}



void ant_set(int ant)
{
    if(ant){//使用板载天线
        gpio_set_level(V1_PIN, 1);
        gpio_set_level(V2_PIN, 0);
    }
    else{//校准通道
        gpio_set_level(V1_PIN, 0);
        gpio_set_level(V2_PIN, 1);
    }
}

// Debug开关
#define SPI_SLAVE_DEBUG 0  // 1开启debug, 0关闭

#if SPI_SLAVE_DEBUG
volatile bool trig_received = false;
#endif

void spi_slave_send_csi(void *pvParameters)
{
    spi_slave_transaction_t t = {0};
    
    while(1) {
        #if SPI_SLAVE_DEBUG
        // 检查TRIG事件
        if(trig_received) {
            printf("[SLAVE] TRIG received, reset stack pointer\n");
            trig_received = false;
        }
        #endif

        uint8_t recvbuf[8] = {0};
        uint8_t sendbuf[8] = {0};
        
        // 始终发送当前栈指针
        sendbuf[0] = csi_stack_pointer;
        memset(&t, 0, sizeof(t));
        t.length = sizeof(recvbuf) * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        // 处理配置命令
        if(recvbuf[0] == 0x01) {  // 普通模式
            memcpy(mac_filter, recvbuf + 1, 6);
            s_count = 0;
            esp_csi_gain_ctrl_reset_rx_gain_baseline();
            // esp_csi_gain_ctrl_set_rx_force_gain(0, 0);
            ant_set(1);
            gpio_set_level(LED1_PIN, 0);
            gpio_set_level(LED2_PIN, 1);
            #if SPI_SLAVE_DEBUG
            printf("[SLAVE] Normal mode, MAC filter: %02x:%02x:%02x:%02x:%02x:%02x\n", 
                   MAC2STR(mac_filter));
            #endif
        }
        else if(recvbuf[0] == 0x02) {  // 校准模式
            memcpy(mac_filter, recvbuf + 1, 6);
            ant_set(0);
            s_count = 0;
            esp_csi_gain_ctrl_reset_rx_gain_baseline();
            // esp_csi_gain_ctrl_set_rx_force_gain(0, 0);
            gpio_set_level(LED1_PIN, 1);
            gpio_set_level(LED2_PIN, 0);
            #if SPI_SLAVE_DEBUG
            printf("[SLAVE] Calibration mode, MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", 
                   MAC2STR(mac_filter));
            #endif
        }
        else if(recvbuf[0] == 0x03) {  // 设置信道、带宽
            esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, recvbuf[2]);
            s_count = 0;
            esp_csi_gain_ctrl_reset_rx_gain_baseline();
            // esp_csi_gain_ctrl_set_rx_force_gain(0, 0);
            if(recvbuf[2]==1)
                esp_wifi_set_channel(recvbuf[1], WIFI_SECOND_CHAN_NONE);
            else
                esp_wifi_set_channel(recvbuf[1], WIFI_SECOND_CHAN_BELOW);
            #if SPI_SLAVE_DEBUG
            printf("[SLAVE] Set channel to %d\n", recvbuf[1]);
            #endif
        }
        else if(recvbuf[0] == 0x04) {
            #if SPI_SLAVE_DEBUG
            printf("[SLAVE] Sending CSI data to host, size: %d bytes\n", sizeof(csi_data));
            #endif
            
            // 发送CSI数据
            memset(&t, 0, sizeof(t));
            t.length = sizeof(csi_data) * 8;
            t.tx_buffer = csi_data;
            t.rx_buffer = NULL;
            spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
            
            #if SPI_SLAVE_DEBUG
            printf("[SLAVE] CSI data sent, waiting for TRIG\n");
            #endif
        }

    }
}

// 中断处理函数 - 收到TRIG时重置栈
void IRAM_ATTR trig_isr_handler(void* arg) 
{
    #if SPI_SLAVE_DEBUG
    trig_received = true;
    #endif
    
    memset(csi_data, 0, sizeof(csi_data));
    csi_stack_pointer = 0;
}
