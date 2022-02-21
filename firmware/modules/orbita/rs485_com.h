#ifndef ORBITA_RS485_H
#define ORBITA_RS485_H

#include "usart.h"

#include "message.h"

HAL_StatusTypeDef rs485_read_message(uint8_t my_id, instruction_packet_t *p);
HAL_StatusTypeDef rs485_send_message(uint8_t my_id, status_packet_t p);

void rs485_switch_to_tx();
void rs485_switch_to_rx();

#endif // ORBITA_RS485_H