#pragma once
#define UART_NUM_1 1
#define ESP_OK 0
#define pdMS_TO_TICKS(x) (x)
inline int uart_wait_tx_done(int, unsigned) { return 0; }
