#include "rs485_com.h"

#include "string.h"
#include "usart.h"

#define MAX_BUFF_SIZE 128

static uint8_t recv_buff[MAX_BUFF_SIZE];
static uint8_t send_buff[MAX_BUFF_SIZE];


HAL_StatusTypeDef rs485_read_message(uint8_t my_id, instruction_packet_t *p)
{
    rs485_switch_to_rx();

    HAL_StatusTypeDef ret = HAL_UART_Receive(&huart1, recv_buff, MSG_HEADER_SIZE, 1000);
    if (ret != HAL_OK)
    {
        return ret;
    }
    
    uint8_t id;
    uint8_t length;
    if (parse_message_header(recv_buff, &id, &length) == -1)
    {
        return HAL_ERROR;
    }
    p->id = id;

    ret = HAL_UART_Receive(&huart1, recv_buff, length, 1000);
    if (ret != HAL_OK)
    {
        return ret;
    }
    
    if (my_id != id)
    {
        return HAL_TIMEOUT;
    }

    if (parse_message_instruction(recv_buff, length, p) == -1)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef rs485_send_message(uint8_t my_id, status_packet_t p)
{
    rs485_switch_to_tx();

    send_buff[0] = 255;
    send_buff[1] = 255;
    send_buff[2] = my_id;
    send_buff[3] = p.size + 2;
    send_buff[4] = p.error;
    if (p.size > 0)
    {
        memcpy(send_buff + 5, p.payload, p.size);
    }
    // TODO: Checksum
    send_buff[5 + p.size] = 0x42;

    return HAL_UART_Transmit(&huart1, send_buff, p.size + 6, 1000);
}

typedef enum {
    RX,
    TX,
    UNKNOWN,
} communication_mode_t;

static communication_mode_t communication_mode = UNKNOWN;

void rs485_switch_to_tx()
{
    if (communication_mode != TX)
    {
        HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
        HAL_Delay(1);

        communication_mode = TX;
    }
}

void rs485_switch_to_rx()
{
    if (communication_mode != RX)
    {
        HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);   
        communication_mode = RX;
    }
}
