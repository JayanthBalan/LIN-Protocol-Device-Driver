
#ifndef BRIDGE_H
#define BRIDGE_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "soc/uart_struct.h"
#include "rom/ets_sys.h"
#include <string.h>

#define LIN_BRK_LEN 16 //Break Field Length
#define MAX_UARTS 2 //UART Channels -> ESP32

#endif
