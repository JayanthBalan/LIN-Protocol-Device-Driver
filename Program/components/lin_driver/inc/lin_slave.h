
#ifndef LIN_SLAVE_H
#define LIN_SLAVE_H

#include "bridge.h"

#define LIN_DUMMY_PIN_SLAVE 25
#define LIN_MAX_PERM_ENTS 64 //Maximum Entry Number
#define LIN_MAX_DATA_BYTE_LEN 8 //Maximum Data Byte Size
#define INTER_BYTE_MS 64 //Inter-Byte Timeout

typedef enum {
    LIN_STATE_IDLE = 0,
    LIN_STATE_SYNC,
    LIN_STATE_PID,
    LIN_STATE_RX_DATA,
    LIN_STATE_RX_CHECKSUM,
    LIN_STATE_ERROR
} lin_slave_state_t;

typedef struct {
    uart_port_t uart_num; //UART Channel Number
    uint8_t pid; //entry ID
    uint8_t data_length; //data length
    uint8_t data_byte_cnt; //data byte counter for reception
    bool checksum_type; //true for enhanced, false for classic
    uint8_t pin; //GPIO pin
    bool direction; // true for tx, false for rx
    uint8_t buffer[LIN_MAX_DATA_BYTE_LEN]; //data buffer
} lin_slave_context_t;

extern uint8_t default_buffer[]; //8 * 0xff

extern SemaphoreHandle_t lin_slave_buffer_lock_1;
extern SemaphoreHandle_t lin_slave_buffer_lock_2;

esp_err_t lin_slave_init(bool, uint16_t, uint8_t, uart_port_t);
esp_err_t lin_slave_register(uint8_t, uint8_t, bool, uart_port_t, bool, uint8_t, uint8_t[], lin_slave_context_t**);
void lin_slave_begin(void);

#endif
