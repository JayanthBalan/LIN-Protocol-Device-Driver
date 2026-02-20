
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "lin_master.h"
#include "lin_slave.h"

static const char *TAG = "MAIN";

TaskHandle_t master_sim_task_handle, slave_sim_task_handle;

void master_sim_task(void*);
void slave_sim_task(void*);

void master_sim_task(void* pvParameters) {
    uint8_t data_send[8] = {0x13, 0x23, 0x33, 0x43, 0x53, 0x63, 0x73}, data_get[3];

    lin_master_init(true, 19200, 17, UART_NUM_1);
    lin_slave_registry(0x12, 7, true, UART_NUM_1, 17);
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
        vTaskDelay(pdMS_TO_TICKS(2000));

        err = lin_master_rx(0x2A, data_get);
        if(err == ESP_OK) {
            ESP_LOGI(TAG, "Master: Received Data for PID 0x2A: %02X %02X %02X", data_get[0], data_get[1], data_get[2]);
        }
        else {
            ESP_LOGE(TAG, "Master: Reception Failed for PID 0x2A : %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void slave_sim_task(void* pvParameters) {
    uint8_t data_send[2][8] = {{0xAB, 0xCD, 0xEF}, {0xA1, 0xB2, 0xC3}}, data_get[7];
    lin_slave_context_t *rx_ctx, *tx_ctx;

    lin_slave_init(true, 19200, 26, UART_NUM_2);
    lin_slave_register(0x12, 7, true, UART_NUM_2, false, 26, default_buffer, &rx_ctx);
    lin_slave_register(0x2A, 3, false, UART_NUM_2, true, 26, default_buffer, &tx_ctx);
    lin_slave_begin();

    xTaskNotifyGive(master_sim_task_handle);

    while(1) {
        xSemaphoreTake(lin_slave_buffer_lock_2, portMAX_DELAY);
        memcpy(tx_ctx->buffer, data_send[1], 3);
        xSemaphoreGive(lin_slave_buffer_lock_2);
        ESP_LOGI(TAG, "Slave: Buffer Prepped for TX via 0x2A");
        vTaskDelay(pdMS_TO_TICKS(1500));

        xSemaphoreTake(lin_slave_buffer_lock_2, portMAX_DELAY);
        memcpy(data_get, rx_ctx->buffer, 7);
        xSemaphoreGive(lin_slave_buffer_lock_2);
        ESP_LOGI(TAG, "Slave: Buffer Updated from RX via 0x12: %02X %02X %02X %02X %02X %02X %02X", data_get[0], data_get[1], data_get[2], data_get[3], data_get[4], data_get[5], data_get[6]);
        vTaskDelay(pdMS_TO_TICKS(1500));

        xSemaphoreTake(lin_slave_buffer_lock_2, portMAX_DELAY);
        memcpy(tx_ctx->buffer, data_send[0], 3);
        xSemaphoreGive(lin_slave_buffer_lock_2);
        ESP_LOGI(TAG, "Slave: Buffer Prepped for TX via 0x2A");
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

}

void app_main() {
    xTaskCreate(master_sim_task, "master_sim_task", 4096, NULL, 2, &master_sim_task_handle);
    xTaskCreate(slave_sim_task, "slave_sim_task", 4096, NULL, 2, &slave_sim_task_handle);
    
    vTaskDelete(NULL);
}

