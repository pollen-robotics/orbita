#ifndef ORBITA_RS485_H
#define ORBITA_RS485_H

#include "usart.h"

#include "message.h"

HAL_StatusTypeDef rs485_wait_for_message_IT();
HAL_StatusTypeDef rs485_get_instruction_packet(instruction_packet_t *dst, uint8_t *crc);

HAL_StatusTypeDef rs485_send_message_IT(uint8_t id, status_packet_t *status_packet);


#endif // ORBITA_RS485_H