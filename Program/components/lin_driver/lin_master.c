
#include "lin_master.h"

SemaphoreHandle_t lin_mutex = NULL;

static const char *TAG = "LIN_MASTER";

volatile static bool init_success = false;
volatile static bool registration_freeze = false;

typedef struct {
    uart_port_t uart_num; //UART Channel Number
    uint8_t pid; //entry ID
    uint8_t data_length; //data length
    bool status; //true for active, false for inactive
    bool checksum_type; //true for enhanced, false for classic
    uint8_t pin; //GPIO PIN
} lin_slave_entry_t;
static lin_slave_entry_t slave_table[LIN_MAX_PERMIT_SLAVES];
static uint8_t entry_cnt = 0;
static uint8_t idx_map[LIN_MAX_PERMIT_SLAVES];

//Function Prototypes
static esp_err_t uart_init(uint16_t, uint8_t, uart_port_t);
static esp_err_t uart_switch(bool, uint8_t, uart_port_t);
static uint8_t checksum_classic(uint8_t, uint8_t*);
static uint8_t checksum_enhanced(uint8_t, uint8_t*);

esp_err_t lin_slave_registry(uint8_t id, uint8_t len, bool type, uart_port_t uart_num, uint8_t lin_pin) {
    if(registration_freeze || !init_success) {
        ESP_LOGE(TAG, "Registration Freeze Active or Uninitialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (entry_cnt >= LIN_MAX_PERMIT_SLAVES) {
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
    
    uint8_t id0 = (id >> 0) & 1;
    uint8_t id1 = (id >> 1) & 1;
    uint8_t id2 = (id >> 2) & 1;
    uint8_t id3 = (id >> 3) & 1;
    uint8_t id4 = (id >> 4) & 1;
    uint8_t id5 = (id >> 5) & 1;
    uint8_t p0 = id0 ^ id1 ^ id2 ^ id4;
    uint8_t p1 = !(id1 ^ id3 ^ id4 ^ id5);
    uint8_t pid = (id & 0x3F) | (p0 << 6) | (p1 << 7);
    
    slave_table[entry_cnt].uart_num = uart_num;
    slave_table[entry_cnt].pid = pid;
    slave_table[entry_cnt].data_length = len;
    slave_table[entry_cnt].status = true;
    slave_table[entry_cnt].checksum_type = type;
    slave_table[entry_cnt].pin = lin_pin;

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

    ret = uart_set_pin(uart_num, lin_pin, LIN_DUMMY_PIN_MASTER, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); //Initial Master in TX mode
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_init: Set Pin Failure: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "uart_init: Configured GPIO %d for LIN", lin_pin);

    ret = uart_driver_install(uart_num, 256, 256, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_init: Driver Install Failure: %s", esp_err_to_name(ret));
        return ret;
    }

    uart_set_rx_timeout(uart_num, 2);
    uart_set_mode(uart_num, UART_MODE_UART);
    
    ESP_LOGI(TAG, "uart_init: Initialization Successful on UART%d", uart_num);
    return ESP_OK;
}

static esp_err_t uart_switch(bool dir, uint8_t lin_pin, uart_port_t uart_num)
{
    esp_err_t ret;
    if(dir)
    {
        ret = uart_set_pin(uart_num, lin_pin, LIN_DUMMY_PIN_MASTER, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    else {
        uart_wait_tx_done(uart_num, pdMS_TO_TICKS(50));
        ret = uart_set_pin(uart_num, LIN_DUMMY_PIN_MASTER, lin_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_flush_input(uart_num);
    }

    ets_delay_us(200);

    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

esp_err_t lin_master_init(bool uart_init_sel, uint16_t baud_rate, uint8_t lin_pin, uart_port_t uart_num)
{
    if (uart_init_sel) { //UART Initialization Option
        esp_err_t ret = uart_init(baud_rate, lin_pin, uart_num);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "init: UART Initialization Failed");
            return ESP_FAIL;
        }
    }

    if (!init_success) {
        lin_mutex = xSemaphoreCreateMutex();
        if (lin_mutex == NULL) {
            ESP_LOGE(TAG, "LIN Mutex Not Initialized");
            return ESP_ERR_INVALID_STATE;
        }
        memset(slave_table, 0, sizeof(slave_table));
        memset(idx_map, 0xff, sizeof(idx_map));
        entry_cnt = 0;
        init_success = true;
    }

    return ESP_OK;
}

static uint8_t checksum_classic(uint8_t ent, uint8_t* data) {
    uint16_t s = 0;
    for (uint8_t i = 0; i < slave_table[ent].data_length; i++) {
        s += data[i];
        if (s > 0xFF) {
            s = (s & 0xFF) + 1;
        }
    }
    return (uint8_t)(~s);
}

static uint8_t checksum_enhanced(uint8_t ent, uint8_t* data) {
    uint16_t s = slave_table[ent].pid;
    for (uint8_t i = 0; i < slave_table[ent].data_length; i++) {
        s += data[i];
        if (s > 0xFF) {
            s = (s & 0xFF) + 1;
        }
    }
    return (uint8_t)(~s);
}

void lin_master_begin(void) {
    if (!init_success) {
        ESP_LOGE(TAG, "Uninitialized");
        return;
    }
    registration_freeze = true;
    ESP_LOGI(TAG, "Operational");
}

esp_err_t lin_master_tx(uint8_t id, uint8_t *data)
{
    if (!init_success || !registration_freeze) {
        ESP_LOGE(TAG, "Registration Freeze Inactive or Uninitialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ent = idx_map[id];
    if(id > 0x3f || ent == 0xff) {
        ESP_LOGE(TAG, "Invalid LIN ID");
        return ESP_ERR_INVALID_ARG;
    }
    if (ent >= entry_cnt || !slave_table[ent].status) {
        ESP_LOGE(TAG, "Invalid/Inactive Entry: %d", ent);
        return ESP_ERR_INVALID_ARG;
    }
    if (data == NULL) {
        ESP_LOGE(TAG, "NULL Pointer Error");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(lin_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = uart_switch(true, slave_table[ent].pin, slave_table[ent].uart_num);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART Switch Failed");
        xSemaphoreGive(lin_mutex);
        return err;
    }

    //Break Field
    uint8_t dummy[2] = {0x00, 0x00};
    uart_write_bytes(slave_table[ent].uart_num, dummy, 2);
    uart_wait_tx_done(slave_table[ent].uart_num, pdMS_TO_TICKS(100));

    //Sync Field
    uint8_t sync = 0x55;
    uart_write_bytes(slave_table[ent].uart_num, &sync, 1);

    //PID Field
    uart_write_bytes(slave_table[ent].uart_num, &slave_table[ent].pid, 1);

    uart_write_bytes(slave_table[ent].uart_num, data, slave_table[ent].data_length); //Transmit Field

    //Checksum Field
    uint8_t checksum = (slave_table[ent].checksum_type && id < 0x3c) ? checksum_enhanced(ent, data) : checksum_classic(ent, data);
    uart_write_bytes(slave_table[ent].uart_num, &checksum, 1);

    uart_wait_tx_done(slave_table[ent].uart_num, pdMS_TO_TICKS(100));

    xSemaphoreGive(lin_mutex);
    return ESP_OK;
}

esp_err_t lin_master_rx(uint8_t id, uint8_t *data)
{
    if (!init_success || !registration_freeze) {
        ESP_LOGE(TAG, "Registration Freeze Inactive or Uninitialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ent = idx_map[id];
    if(id > 0x3f || ent == 0xff) {
        ESP_LOGE(TAG, "Invalid LIN ID");
        return ESP_ERR_INVALID_ARG;
    }
    if (ent >= entry_cnt || !slave_table[ent].status) {
        ESP_LOGE(TAG, "Invalid/Inactive Entry: %d", ent);
        return ESP_ERR_INVALID_ARG;
    }
    if (data == NULL) {
        ESP_LOGE(TAG, "NULL Pointer Error");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(lin_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = uart_switch(true, slave_table[ent].pin, slave_table[ent].uart_num);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART Switch Failed");
        xSemaphoreGive(lin_mutex);
        return err;
    }

    //Break Field
    uint8_t dummy[2] = {0x00, 0x00};
    uart_write_bytes(slave_table[ent].uart_num, dummy, 2); 
    uart_wait_tx_done(slave_table[ent].uart_num, pdMS_TO_TICKS(100));

    //Sync Field
    uint8_t sync = 0x55;
    int writ = uart_write_bytes(slave_table[ent].uart_num, &sync, 1);
    if (writ < 0) {
        ESP_LOGE(TAG, "Sync Field Failure");
        xSemaphoreGive(lin_mutex);
        return ESP_FAIL;
    }

    //PID Field
    uart_write_bytes(slave_table[ent].uart_num, &slave_table[ent].pid, 1);
    uart_wait_tx_done(slave_table[ent].uart_num, pdMS_TO_TICKS(100));

    err = uart_switch(false, slave_table[ent].pin, slave_table[ent].uart_num);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART Switch Failed");
        xSemaphoreGive(lin_mutex);
        return err;
    }

    //Receive Field
    int bytes_read = uart_read_bytes(slave_table[ent].uart_num, data, slave_table[ent].data_length, pdMS_TO_TICKS(100));
    if (bytes_read != slave_table[ent].data_length) {
        ESP_LOGE(TAG, "Data reception failed: expected %d, got %d", slave_table[ent].data_length, bytes_read);
        xSemaphoreGive(lin_mutex);
        return ESP_FAIL;
    }

    //Checksum Field
    uint8_t checksum;
    int chk_read = uart_read_bytes(slave_table[ent].uart_num, &checksum, 1, pdMS_TO_TICKS(100));
    if (chk_read != 1) {
        ESP_LOGE(TAG, "Checksum read failed");
        xSemaphoreGive(lin_mutex);
        return ESP_ERR_TIMEOUT;
    }
    uint8_t ver_checksum = (slave_table[ent].checksum_type && id < 0x3c) ? checksum_enhanced(ent, data) : checksum_classic(ent, data);
    if(checksum != ver_checksum) {
        ESP_LOGE(TAG, "Checksum Mismatch: Received 0x%02X, Calculated 0x%02X", checksum, ver_checksum);
        xSemaphoreGive(lin_mutex);
        return ESP_ERR_INVALID_CRC;
    }

    xSemaphoreGive(lin_mutex);
    return ESP_OK;
}
