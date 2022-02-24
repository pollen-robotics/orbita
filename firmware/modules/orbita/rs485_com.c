#include "rs485_com.h"

#include "string.h"
#include "usart.h"
#include "message.h"
#include "utils.h"


typedef enum {
    RxUnknown,
    RxReadingHeader,
    RxReadingInstructionPacket,
    RxMessageReady,
} rx_state_t;

#define MAX_BUFF_SIZE 256 + MSG_HEADER_SIZE + 1

static volatile uint8_t msg_buff[MAX_BUFF_SIZE];
static volatile rx_state_t rx_state = RxUnknown;

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

        for (uint32_t i = 0; i < 1500; i++) {
            asm("NOP");
        }

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


HAL_StatusTypeDef reset_rx_buff()
{
    rx_state = RxUnknown;
    return HAL_UART_Receive_IT(&huart1, (uint8_t *)msg_buff, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (UartHandle != &huart1)
    {
        return;
    }

    if (rx_state == RxUnknown)
    {
        // We are still waiting for the begining of the header (0xff)
        if (msg_buff[0] != 0xff)
        {
            reset_rx_buff();
        }
        else
        {
            rx_state = RxReadingHeader;
            // Wait for the rest of the header
            HAL_UART_Receive_IT(&huart1, (uint8_t *)msg_buff + 1, 3);
        }
    }
    // Here, we should have the entire header at this point (4 bytes)
    else if (rx_state == RxReadingHeader) 
    { 
        // Wrong header
        if ((msg_buff[0] != 0xff) || (msg_buff[1] != 0xff))
        {
            reset_rx_buff();
        }
        else
        {
            // Wait for the instruction packet
            rx_state = RxReadingInstructionPacket;
            uint8_t instruction_packet_length = msg_buff[3];
            HAL_UART_Receive_IT(&huart1, (uint8_t *)msg_buff + 4, instruction_packet_length);
        }
    }
    // We should have the entire message here
    else if (rx_state == RxReadingInstructionPacket) 
    {
        rx_state = RxMessageReady;
    }
    // This should never happens
    else if (rx_state == RxMessageReady)
    {
        while (1) {};
    }
}

HAL_StatusTypeDef rs485_wait_for_message_IT()
{
    rs485_switch_to_rx();
    return reset_rx_buff();
}

HAL_StatusTypeDef rs485_get_instruction_packet(instruction_packet_t *dst, uint8_t *crc)
{
    if (rx_state != RxMessageReady)
    {
        return HAL_ERROR;
    }

    // [0xff, 0xff, ID, LEN, INSTR, PARAM, ..., CRC]
    dst->id = msg_buff[2];
    dst->payload_size = msg_buff[3] - 2;
    dst->type = msg_buff[4];

    dst->crc = msg_buff[5 + dst->payload_size];
    if (dst->payload_size > 0)
    {
        memcpy(dst->payload, (uint8_t *)msg_buff + 5, dst->payload_size);
    }

    *crc = compute_crc((uint8_t *)msg_buff + 2, 3 + dst->payload_size);

    rx_state = RxUnknown;

    return HAL_OK;
}

HAL_StatusTypeDef rs485_send_message_IT(uint8_t id, status_packet_t *p)
{
    rs485_switch_to_tx();

    // [0xff, 0xff, ID, LEN, ERR, PARAM, ..., CRC]
    //    0     1    2   3    4               5 + n
    msg_buff[0] = 255;
    msg_buff[1] = 255;
    msg_buff[2] = id;
    msg_buff[3] = p->payload_size + 2;
    msg_buff[4] = p->error;
    if (p->payload_size > 0)
    {
        memcpy((uint8_t *)msg_buff + 5, p->payload, p->payload_size);
    }

    msg_buff[5 + p->payload_size] = compute_crc((uint8_t *)msg_buff + 2, p->payload_size + 3);

    return HAL_UART_Transmit_IT(&huart1, (uint8_t *)msg_buff, p->payload_size + 6);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (UartHandle != &huart1)
    {
        return;
    }

    toggle_status_led();
    
    rs485_wait_for_message_IT();    
}

