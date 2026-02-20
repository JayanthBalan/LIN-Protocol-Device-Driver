
#include "lin_slave.h"

static const char *TAG = "LIN_SLAVE";

uint8_t default_buffer[LIN_MAX_DATA_BYTE_LEN];

static TaskHandle_t lin_slave_uart_1_handle;
static TaskHandle_t lin_slave_uart_2_handle;
SemaphoreHandle_t lin_slave_buffer_lock_1;
SemaphoreHandle_t lin_slave_buffer_lock_2;
static QueueHandle_t uart1_event_queue = NULL;
static QueueHandle_t uart2_event_queue = NULL;

static lin_slave_context_t slave_ctx[LIN_MAX_PERM_ENTS];
static uint8_t entry_cnt = 0;
static uint8_t idx_map[LIN_MAX_PERM_ENTS];
volatile static bool registration_freeze = false;
volatile static bool init_success = false;

//Task Prototypes
void lin_slave_uart_1(void*);
void lin_slave_uart_2(void*);

//Function Prototypes
static uint8_t parity_calc(uint8_t);
static esp_err_t uart_init(uint16_t, uint8_t, uart_port_t);
static esp_err_t uart_switch(bool, uint8_t, uart_port_t);
static uint8_t checksum_classic(uint8_t, uint8_t*);
static uint8_t checksum_enhanced(uint8_t, uint8_t*);
static inline uint8_t pid_lut(uint8_t);
static void lin_slave_fsm_2(uint8_t);
static void lin_slave_fsm_1(uint8_t);


static uint8_t parity_calc(uint8_t id) {
    uint8_t id0 = (id >> 0) & 1;
    uint8_t id1 = (id >> 1) & 1;
    uint8_t id2 = (id >> 2) & 1;
    uint8_t id3 = (id >> 3) & 1;
    uint8_t id4 = (id >> 4) & 1;
    uint8_t id5 = (id >> 5) & 1;
    uint8_t p0 = id0 ^ id1 ^ id2 ^ id4;
    uint8_t p1 = !(id1 ^ id3 ^ id4 ^ id5);
    return ((id & 0x3F) | (p0 << 6) | (p1 << 7));
}

esp_err_t lin_slave_register(uint8_t id, uint8_t len, bool type, uart_port_t uart_num, bool dir, uint8_t lin_pin, uint8_t buf[], lin_slave_context_t **ctx) {
    if (registration_freeze || !init_success) {
        ESP_LOGE(TAG, "Registration Frozen or Uninitialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (entry_cnt >= LIN_MAX_PERM_ENTS) {
        ESP_LOGE(TAG, "Slave Registry Overflow");
        return ESP_ERR_NO_MEM;
    }
    if (id > 0x3F) {
        ESP_LOGE(TAG, "Invalid PID: %d", id);
        return ESP_ERR_INVALID_ARG;
    }
    if (len == 0 || len > 8) {
        ESP_LOGE(TAG, "Invalid Frame Length: %d", len);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pid = parity_calc(id);
    
    slave_ctx[entry_cnt].uart_num = uart_num;
    slave_ctx[entry_cnt].pid = pid;
    slave_ctx[entry_cnt].data_length = len;
    slave_ctx[entry_cnt].checksum_type = type;
    slave_ctx[entry_cnt].pin = lin_pin;
    slave_ctx[entry_cnt].direction = dir;
    slave_ctx[entry_cnt].data_byte_cnt = 0;
    if (len <= LIN_MAX_DATA_BYTE_LEN) {
        memcpy(slave_ctx[entry_cnt].buffer, buf, len);
    }
    else {
        ESP_LOGE(TAG, "Buffer overflow prevented");
        return ESP_ERR_INVALID_SIZE;
    }

    *ctx = &slave_ctx[entry_cnt];
    idx_map[id] = entry_cnt;
    entry_cnt++;

    return ESP_OK;
}

static esp_err_t uart_init(uint16_t baud_rate, uint8_t lin_pin, uart_port_t uart_num)
{
    esp_err_t ret;
    uart_config_t config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ret = uart_param_config(uart_num, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_init: Param Config Failure: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_set_pin(uart_num, LIN_DUMMY_PIN_SLAVE, lin_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); //Initial Slave in RX mode
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_init: Set Pin Failure: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "uart_init: Configured GPIO %d for LIN", lin_pin);

    int rx_buf_size = 1024;
    int tx_buf_size = 1024;
    int queue_size = 50;
    if(uart_num == UART_NUM_1) {
        ret = uart_driver_install(uart_num, rx_buf_size, tx_buf_size, queue_size, &uart1_event_queue, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "uart_init: Driver Install Failure: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    else if(uart_num == UART_NUM_2) {
        ret = uart_driver_install(uart_num, rx_buf_size, tx_buf_size, queue_size, &uart2_event_queue, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "uart_init: Driver Install Failure: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    uart_flush_input(uart_num);

    uart_set_rx_timeout(uart_num, 2);
    uart_set_mode(uart_num, UART_MODE_UART);
    uart_set_rx_full_threshold(uart_num, 1);

    ESP_LOGI(TAG, "uart_init: Initialization Successful on UART%d", uart_num);
    return ESP_OK;
}

static esp_err_t uart_switch(bool dir, uint8_t lin_pin, uart_port_t uart_num)
{
    esp_err_t ret;
    if(dir)
    {
        ret = uart_set_pin(uart_num, lin_pin, LIN_DUMMY_PIN_SLAVE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    else {
        uart_wait_tx_done(uart_num, pdMS_TO_TICKS(50));
        ret = uart_set_pin(uart_num, LIN_DUMMY_PIN_SLAVE, lin_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_flush_input(uart_num);
    }

    ets_delay_us(200);

    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

esp_err_t lin_slave_init(bool uart_init_sel, uint16_t baud_rate, uint8_t lin_pin, uart_port_t uart_num)
{
    if(registration_freeze) {
        ESP_LOGE(TAG, "init: Registration Freeze Active");
        return ESP_ERR_INVALID_STATE;
    }

    if (uart_init_sel) { //UART Initialization Option
        esp_err_t ret = uart_init(baud_rate, lin_pin, uart_num);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "init: UART Initialization Failed");
            return ESP_FAIL;
        }

        if(uart_num == UART_NUM_1) {
            xTaskCreate(lin_slave_uart_1, "lin_slave_uart_1", 4096, NULL, 8, &lin_slave_uart_1_handle);
            lin_slave_buffer_lock_1 = xSemaphoreCreateMutex();
            if (lin_slave_buffer_lock_1 == NULL) {
                ESP_LOGE(TAG, "UART1 Mutex Failure");
                return ESP_ERR_NO_MEM;
            }
            ESP_LOGI(TAG, "UART1 FSM Created");
        }
        else if(uart_num == UART_NUM_2) {
            xTaskCreate(lin_slave_uart_2, "lin_slave_uart_2", 4096, NULL, 8, &lin_slave_uart_2_handle);
            lin_slave_buffer_lock_2 = xSemaphoreCreateMutex();
            if (lin_slave_buffer_lock_2 == NULL) {
                ESP_LOGE(TAG, "UART2 Mutex Failure");
                return ESP_ERR_NO_MEM;
            }
            ESP_LOGI(TAG, "UART2 FSM Created");
        }
        else {
            ESP_LOGE(TAG, "init: Invalid UART Number");
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    if (!init_success) {
        memset(default_buffer, 0xff, sizeof(default_buffer));
        memset(slave_ctx, 0, sizeof(slave_ctx));
        memset(idx_map, 0xff, sizeof(idx_map));
        entry_cnt = 0;
        init_success = true;
        registration_freeze = false;
    }

    return ESP_OK;
}

void lin_slave_begin(void) {
    if(lin_slave_uart_1_handle != NULL && eTaskGetState(lin_slave_uart_1_handle) != eDeleted) {
        vTaskResume(lin_slave_uart_1_handle);
        ESP_LOGI(TAG, "UART1 FSM Started");
    }
    if(lin_slave_uart_2_handle != NULL && eTaskGetState(lin_slave_uart_2_handle) != eDeleted) {
        vTaskResume(lin_slave_uart_2_handle);
        ESP_LOGI(TAG, "UART2 FSM Started");
    }
    registration_freeze = true;
    ESP_LOGI(TAG, "Operational");
}

static uint8_t checksum_classic(uint8_t ent, uint8_t* data) {
    uint16_t s = 0;
    for (uint8_t i = 0; i < slave_ctx[ent].data_length; i++) {
        s += data[i];
        if (s > 0xFF) {
            s = (s & 0xFF) + 1;
        }
    }
    return (uint8_t)(~s);
}

static uint8_t checksum_enhanced(uint8_t ent, uint8_t* data) {
    uint16_t s = slave_ctx[ent].pid;
    for (uint8_t i = 0; i < slave_ctx[ent].data_length; i++) {
        s += data[i];
        if (s > 0xFF) {
            s = (s & 0xFF) + 1;
        }
    }
    return (uint8_t)(~s);
}

static inline uint8_t pid_lut(uint8_t pid) {
    return idx_map[pid & 0x3f];
}

void lin_slave_uart_2(void* pvParameters) {    
    uart_event_t event;
    uint8_t data[128];
    uart_port_t uart_num = UART_NUM_2;

    vTaskSuspend(NULL);

    while (1) {
        if (xQueueReceive(uart2_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            switch (event.type) {
                case UART_DATA: {
                    int to_read = event.size;
                    while (to_read > 0) {
                        int chunk = (to_read > sizeof(data)) ? sizeof(data) : to_read;
                        int r = uart_read_bytes(uart_num, data, chunk, pdMS_TO_TICKS(50));
                        if (r > 0) {
                            for (int i = 0; i < r; ++i) {
                                lin_slave_fsm_2(data[i]);
                            }
                        }
                        to_read -= chunk;
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    uart_flush_input(uart_num);
                    uart_clear_intr_status(uart_num, UART_INTR_RXFIFO_OVF);
                    break;
                case UART_BUFFER_FULL:
                    uart_flush_input(uart_num);
                    break;
                case UART_BREAK:
                case UART_PARITY_ERR:
                case UART_FRAME_ERR:
                    break;
                default:
                    break;
            }
        }
        taskYIELD();
    }
}

void lin_slave_uart_1(void* pvParameters) {    
    uart_event_t event;
    uint8_t data[128];
    uart_port_t uart_num = UART_NUM_1;

    vTaskSuspend(NULL);

    while (1) {
        if (xQueueReceive(uart1_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            switch (event.type) {
                case UART_DATA: {
                    int to_read = event.size;
                    while (to_read > 0) {
                        int chunk = (to_read > sizeof(data)) ? sizeof(data) : to_read;
                        int r = uart_read_bytes(uart_num, data, chunk, pdMS_TO_TICKS(50));
                        if (r > 0) {
                            for (int i = 0; i < r; ++i) {
                                lin_slave_fsm_1(data[i]);
                            }
                        }
                        to_read -= chunk;
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    uart_flush_input(uart_num);
                    uart_clear_intr_status(uart_num, UART_INTR_RXFIFO_OVF);
                    break;
                case UART_BUFFER_FULL:
                    uart_flush_input(uart_num);
                    break;
                case UART_BREAK:
                case UART_PARITY_ERR:
                case UART_FRAME_ERR:
                    break;
                default:
                    break;
            }
        }
    }
    taskYIELD();
}

void lin_slave_fsm_2(uint8_t buffer_byte) 
{
    uart_port_t uart_num = UART_NUM_2;
    static uint8_t cnt = 0;
    static uint8_t curr_pid = 0xff;
    static lin_slave_state_t curr_state = LIN_STATE_IDLE;

            switch (curr_state) {
                case LIN_STATE_IDLE: {
                    if(buffer_byte == 0x00) {
                        cnt++;
                        if (cnt >= 2) {
                            curr_state = LIN_STATE_SYNC;
                            cnt = 0;
                        }
                    }
                    else {
                        if (cnt > 0) {
                            ESP_LOGW(TAG, "UART%d: Expected 0x00, got 0x%02X (reset cnt)", uart_num, buffer_byte);
                        }
                        cnt = 0;
                    }
                    break;
                }

                case LIN_STATE_SYNC: {
                    if(buffer_byte == 0x55) {
                        curr_state = LIN_STATE_PID;
                    }
                    else {
                        ESP_LOGE(TAG, "UART%d: Expected 0x55, got 0x%02X", uart_num, buffer_byte);
                        curr_state = LIN_STATE_ERROR;
                    }
                    break;
                }

                case LIN_STATE_PID: {
                    uint8_t ent = pid_lut(buffer_byte);
                    if (parity_calc((buffer_byte & 0x3f)) == buffer_byte && ent != 0xff) {
                        curr_pid = slave_ctx[ent].pid;
                        if (slave_ctx[ent].direction) {
                            uart_switch(true, slave_ctx[ent].pin, slave_ctx[ent].uart_num);
                            
                            //LIN_STATE_TX_DATA:
                            if (xSemaphoreTake(lin_slave_buffer_lock_2, pdMS_TO_TICKS(1000)) != pdTRUE) {
                                curr_state = LIN_STATE_ERROR;
                                break;
                            }

                            uart_write_bytes(uart_num, slave_ctx[ent].buffer, slave_ctx[ent].data_length);
                            uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
                                        
                            xSemaphoreGive(lin_slave_buffer_lock_2);
                            
                            //LIN_STATE_TX_CHECKSUM:
                            uint8_t checksum = ((slave_ctx[ent].checksum_type) && ((curr_pid & 0x3f) < 0x3c)) ? checksum_enhanced(ent, slave_ctx[ent].buffer) : checksum_classic(ent, slave_ctx[ent].buffer);
                            uart_write_bytes(uart_num, &checksum, 1);
                            uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
                            
                            uart_switch(false, slave_ctx[ent].pin, slave_ctx[ent].uart_num);
                            curr_state = LIN_STATE_IDLE;
                        }
                        else {
                            curr_state = LIN_STATE_RX_DATA;
                        }
                    }
                    else if (parity_calc((buffer_byte & 0x3f)) != buffer_byte) {
                        curr_state = LIN_STATE_ERROR;
                        ESP_LOGE(TAG, "UART%d: PID Parity Mismatch", uart_num);
                    }
                    else {
                        ESP_LOGI(TAG, "UART%d: Unregistered PID 0x%02X", uart_num, buffer_byte);
                        curr_state = LIN_STATE_IDLE;
                    }
                    break;
                }
                
                case LIN_STATE_RX_DATA: {
                    uint8_t ent = pid_lut(curr_pid);
                    if(ent == 0xff) {
                        curr_state = LIN_STATE_ERROR;
                    }
                    else {
                        if (xSemaphoreTake(lin_slave_buffer_lock_2, pdMS_TO_TICKS(1000)) != pdTRUE) {
                            curr_state = LIN_STATE_ERROR;
                            break;
                        }
                        
                        if (slave_ctx[ent].data_byte_cnt < slave_ctx[ent].data_length - 1) {
                            slave_ctx[ent].buffer[slave_ctx[ent].data_byte_cnt] = buffer_byte;
                            slave_ctx[ent].data_byte_cnt++;
                        }
                        else if(slave_ctx[ent].data_byte_cnt == slave_ctx[ent].data_length - 1) {
                            slave_ctx[ent].buffer[slave_ctx[ent].data_byte_cnt] = buffer_byte;
                            slave_ctx[ent].data_byte_cnt = 0;
                            curr_state = LIN_STATE_RX_CHECKSUM;
                        }
                        else {
                            curr_state = LIN_STATE_ERROR;
                        }
                        
                        xSemaphoreGive(lin_slave_buffer_lock_2);
                    }
                    break;
                }
                
                case LIN_STATE_RX_CHECKSUM: {
                    uint8_t ent = pid_lut(curr_pid);
                    if (ent == 0xff) {
                        curr_state = LIN_STATE_ERROR;
                    }
                    else {
                        uint8_t calc_checksum = ((slave_ctx[ent].checksum_type) && ((curr_pid & 0x3f) < 0x3c)) ? checksum_enhanced(ent, slave_ctx[ent].buffer) : checksum_classic(ent, slave_ctx[ent].buffer);
                        
                        if (calc_checksum == buffer_byte) {
                            curr_state = LIN_STATE_IDLE;
                        }
                        else {
                            ESP_LOGE(TAG, "UART%d: Checksum error: got 0x%02X, expected 0x%02X", uart_num, buffer_byte, calc_checksum);
                            curr_state = LIN_STATE_ERROR;
                        }
                    }
                    break;
                }

                case LIN_STATE_ERROR: {
                    ESP_LOGE(TAG, "UART%d: ERROR - resetting to IDLE", uart_num);
                    curr_state = LIN_STATE_IDLE;
                    cnt = 0;
                    curr_pid = 0xff;
                    break;
                }
            }
}

void lin_slave_fsm_1(uint8_t buffer_byte) 
{
    uart_port_t uart_num = UART_NUM_1;
    static uint8_t cnt = 0;
    static uint8_t curr_pid = 0xff;
    static lin_slave_state_t curr_state = LIN_STATE_IDLE;

            switch (curr_state) {
                case LIN_STATE_IDLE: {
                    if(buffer_byte == 0x00) {
                        cnt++;
                        if (cnt >= 2) {
                            curr_state = LIN_STATE_SYNC;
                            cnt = 0;
                        }
                    }
                    else {
                        if (cnt > 0) {
                            ESP_LOGW(TAG, "UART%d: Expected 0x00, got 0x%02X (reset cnt)", uart_num, buffer_byte);
                        }
                        cnt = 0;
                    }
                    break;
                }

                case LIN_STATE_SYNC: {
                    if(buffer_byte == 0x55) {
                        curr_state = LIN_STATE_PID;
                    }
                    else {
                        ESP_LOGE(TAG, "UART%d: Expected 0x55, got 0x%02X", uart_num, buffer_byte);
                        curr_state = LIN_STATE_ERROR;
                    }
                    break;
                }

                case LIN_STATE_PID: {
                    uint8_t ent = pid_lut(buffer_byte);
                    if (parity_calc((buffer_byte & 0x3f)) == buffer_byte && ent != 0xff) {
                        curr_pid = slave_ctx[ent].pid;
                        if (slave_ctx[ent].direction) {
                            uart_switch(true, slave_ctx[ent].pin, slave_ctx[ent].uart_num);
                            
                            //LIN_STATE_TX_DATA:
                            if (xSemaphoreTake(lin_slave_buffer_lock_1, pdMS_TO_TICKS(1000)) != pdTRUE) {
                                curr_state = LIN_STATE_ERROR;
                                break;
                            }
                            uart_write_bytes(uart_num, slave_ctx[ent].buffer, slave_ctx[ent].data_length);
                            uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
                                        
                            xSemaphoreGive(lin_slave_buffer_lock_1);

                            //LIN_STATE_TX_CHECKSUM:
                            uint8_t checksum = ((slave_ctx[ent].checksum_type) && ((curr_pid & 0x3f) < 0x3c)) ? checksum_enhanced(ent, slave_ctx[ent].buffer) : checksum_classic(ent, slave_ctx[ent].buffer);
                            uart_write_bytes(uart_num, &checksum, 1);
                            uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
                            
                            uart_switch(false, slave_ctx[ent].pin, slave_ctx[ent].uart_num);
                            curr_state = LIN_STATE_IDLE;
                        }
                        else {
                            curr_state = LIN_STATE_RX_DATA;
                        }
                    }
                    else if (parity_calc((buffer_byte & 0x3f)) != buffer_byte) {
                        curr_state = LIN_STATE_ERROR;
                        ESP_LOGE(TAG, "UART%d: PID Parity Mismatch", uart_num);
                    }
                    else {
                        ESP_LOGI(TAG, "UART%d: Unregistered PID 0x%02X", uart_num, buffer_byte);
                        curr_state = LIN_STATE_IDLE;
                    }
                    break;
                }
                
                case LIN_STATE_RX_DATA: {
                    uint8_t ent = pid_lut(curr_pid);
                    if(ent == 0xff) {
                        curr_state = LIN_STATE_ERROR;
                    }
                    else {
                        if (xSemaphoreTake(lin_slave_buffer_lock_1, pdMS_TO_TICKS(1000)) != pdTRUE) {
                            curr_state = LIN_STATE_ERROR;
                            break;
                        }
                        
                        if (slave_ctx[ent].data_byte_cnt < slave_ctx[ent].data_length - 1) {
                            slave_ctx[ent].buffer[slave_ctx[ent].data_byte_cnt] = buffer_byte;
                            slave_ctx[ent].data_byte_cnt++;
                        }
                        else if(slave_ctx[ent].data_byte_cnt == slave_ctx[ent].data_length - 1) {
                            slave_ctx[ent].buffer[slave_ctx[ent].data_byte_cnt] = buffer_byte;
                            slave_ctx[ent].data_byte_cnt = 0;
                            curr_state = LIN_STATE_RX_CHECKSUM;
                        }
                        else {
                            curr_state = LIN_STATE_ERROR;
                        }
                        
                        xSemaphoreGive(lin_slave_buffer_lock_1);
                    }
                    break;
                }
                
                case LIN_STATE_RX_CHECKSUM: {
                    uint8_t ent = pid_lut(curr_pid);
                    if (ent == 0xff) {
                        curr_state = LIN_STATE_ERROR;
                    }
                    else {
                        uint8_t calc_checksum = ((slave_ctx[ent].checksum_type) && ((curr_pid & 0x3f) < 0x3c)) ? checksum_enhanced(ent, slave_ctx[ent].buffer) : checksum_classic(ent, slave_ctx[ent].buffer);
                        
                        if (calc_checksum == buffer_byte) {
                            curr_state = LIN_STATE_IDLE;
                        }
                        else {
                            curr_state = LIN_STATE_ERROR;
                        }
                    }
                    break;
                }

                case LIN_STATE_ERROR: {
                    ESP_LOGE(TAG, "UART%d: ERROR - resetting to IDLE", uart_num);
                    curr_state = LIN_STATE_IDLE;
                    cnt = 0;
                    curr_pid = 0xff;
                    break;
                }
            }
}
