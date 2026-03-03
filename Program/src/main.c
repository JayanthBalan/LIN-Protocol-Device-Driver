
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"

#include "lin_master.h"
#include "lin_slave.h"

#define RMT_CLK_GPIO GPIO_NUM_18
#define HALF_PERIOD_US 26 // 2/(baud_rate MHz); baud_rate = 19200
#define RMT_RESOLUTION 1000000 // 1MHz -> 1 tick = 1 us

static const char *TAG = "MAIN";

static rmt_channel_handle_t tx_chan = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;

TaskHandle_t master_sim_task_handle, slave_sim_task_handle;

void master_sim_task(void*);
void slave_sim_task(void*);
static void rmt_clk_init(void);

void master_sim_task(void* pvParameters) {
    uint8_t data_send[8] = {0x13, 0x23, 0x33, 0x43, 0x53, 0x63, 0x73, 0x83}, data_get[3];

    lin_master_init(true, 19200, 17, UART_NUM_1);
    lin_slave_registry(0x12, 8, true, UART_NUM_1, 17);
    lin_slave_registry(0x2A, 3, false, UART_NUM_1, 17);
    lin_master_begin();

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    while(1) {
        esp_err_t err = lin_master_tx(0x12, data_send);
        if(err == ESP_OK) {
            ESP_LOGI(TAG, "Master: Transmitted Data for PID 0x12");
        }
        else {
            ESP_LOGE(TAG, "Master: Transmission Failed for PID 0x12 : %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(10));

        err = lin_master_rx(0x2A, data_get);
        if(err == ESP_OK) {
            ESP_LOGI(TAG, "Master: Received Data for PID 0x2A: %02X %02X %02X", data_get[0], data_get[1], data_get[2]);
        }
        else {
            ESP_LOGE(TAG, "Master: Reception Failed for PID 0x2A : %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void slave_sim_task(void* pvParameters) {
    uint8_t data_send[2][3] = {{0xAB, 0xCD, 0xEF}, {0xA1, 0xB2, 0xC3}}, data_get[8];
    lin_slave_context_t *rx_ctx, *tx_ctx;

    lin_slave_init(true, 19200, 26, UART_NUM_2);
    lin_slave_register(0x12, 8, true, UART_NUM_2, false, 26, default_buffer, &rx_ctx);
    lin_slave_register(0x2A, 3, false, UART_NUM_2, true, 26, default_buffer, &tx_ctx);
    lin_slave_begin();

    xTaskNotifyGive(master_sim_task_handle);

    while(1) {
        xSemaphoreTake(lin_slave_buffer_lock_2, portMAX_DELAY);
        memcpy(tx_ctx->buffer, data_send[1], 3);
        xSemaphoreGive(lin_slave_buffer_lock_2);
        ESP_LOGI(TAG, "Slave: Buffer Prepped for TX via 0x2A");
        vTaskDelay(pdMS_TO_TICKS(10));

        xSemaphoreTake(lin_slave_buffer_lock_2, portMAX_DELAY);
        memcpy(data_get, rx_ctx->buffer, 8);
        xSemaphoreGive(lin_slave_buffer_lock_2);
        ESP_LOGI(TAG, "Slave: Buffer Updated from RX via 0x12: %02X %02X %02X %02X %02X %02X %02X %02X", data_get[0], data_get[1], data_get[2], data_get[3], data_get[4], data_get[5], data_get[6], data_get[7]);
        vTaskDelay(pdMS_TO_TICKS(10));

        xSemaphoreTake(lin_slave_buffer_lock_2, portMAX_DELAY);
        memcpy(tx_ctx->buffer, data_send[0], 3);
        xSemaphoreGive(lin_slave_buffer_lock_2);
        ESP_LOGI(TAG, "Slave: Buffer Prepped for TX via 0x2A");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}

void rmt_clk_init(void)
{
    esp_err_t err;
    rmt_tx_channel_config_t tx_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = RMT_CLK_GPIO,
        .mem_block_symbols = 64, // 64 symbols (1 block equivalent)
        .resolution_hz = RMT_RESOLUTION,
        .trans_queue_depth = 4, // TX queue size
    };
    err = rmt_new_tx_channel(&tx_config, &tx_chan);
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(rmt_enable(tx_chan));

    rmt_copy_encoder_config_t copy_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_config, &copy_encoder));

    rmt_symbol_word_t clock_symbol = {
        .level0 = 1,
        .duration0 = HALF_PERIOD_US,
        .level1 = 0,
        .duration1 = HALF_PERIOD_US,
    };

    rmt_transmit_config_t transmit_config;
    transmit_config.loop_count = -1;

    ESP_ERROR_CHECK(rmt_transmit(tx_chan, copy_encoder, &clock_symbol, sizeof(clock_symbol), &transmit_config));
    ESP_LOGI(TAG, "RMT: 19200 baud clock started on GPIO %d", RMT_CLK_GPIO);
}

void app_main() {
    rmt_clk_init();
    xTaskCreate(master_sim_task, "master_sim_task", 4096, NULL, 2, &master_sim_task_handle);
    xTaskCreate(slave_sim_task, "slave_sim_task", 4096, NULL, 2, &slave_sim_task_handle);
    
    vTaskDelete(NULL);
}

