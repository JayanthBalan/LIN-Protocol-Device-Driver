
#ifndef LIN_MASTER_H
#define LIN_MASTER_H

#include "bridge.h"

#define LIN_DUMMY_PIN_MASTER 16
#define LIN_MAX_PERMIT_SLAVES 64 //Maximum Entry Number

esp_err_t lin_master_init(bool, uint16_t, uint8_t, uart_port_t);
esp_err_t lin_slave_registry(uint8_t, uint8_t, bool, uart_port_t, uint8_t);
esp_err_t lin_master_rx(uint8_t, uint8_t*);
esp_err_t lin_master_tx(uint8_t, uint8_t*);
void lin_master_begin(void);

#endif
