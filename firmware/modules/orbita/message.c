#include "message.h"

#include "string.h"

#include "orbita.h"

int8_t parse_message_header(uint8_t *recv_buff, uint8_t *id, uint8_t *length)
{
    if (recv_buff[0] != 255 || recv_buff[1] != 255)
    {
        return -1;
    }

    *id = recv_buff[2];
    *length = recv_buff[3];

    return 0;
}

int8_t parse_message_instruction(uint8_t *recv_buff, uint8_t length, instruction_packet_t *p)
{
    p->type = recv_buff[0];
    // TODO: check CRC

    if (p->type == PING_MESSAGE)
    {
        p->size = 0;
    }
    else if (p->type == READ_DATA_MESSAGE)
    {
        p->size = 2;
        memcpy(p->payload, recv_buff + 1, p->size);
    }
    else if (p->type == WRITE_DATA_MESSAGE)
    {
        p->size = length - 2;
        memcpy(p->payload, recv_buff + 1, p->size);
    }
    else
    {
        return -1;
    }
    return 0;
}

void fill_read_status_with_uint8(uint8_t *data, int nb, status_packet_t *p)
{
    p->size = sizeof(uint8_t) * NB_MOTORS * nb;
    memcpy(p->payload, data, p->size);
}

void fill_read_status_with_int32(int32_t *data, int nb, status_packet_t *p)
{
    p->size = sizeof(int32_t) * NB_MOTORS * nb;
    memcpy(p->payload, data, p->size);
}

void fill_read_status_with_float(float *data, int nb, status_packet_t *p)
{
    p->size = sizeof(float) * NB_MOTORS * nb;
    memcpy(p->payload, data, p->size);
}

void fill_write_status_with_uint8(
    uint8_t *target_values, 
    uint8_t *coded_values, uint8_t size, 
    status_packet_t *status)
{
    if (size != (NB_MOTORS * sizeof(uint8_t)))
    {
        status->error = INSTRUCTION_ERROR;
        return;
    }

    memcpy(target_values, coded_values, size);
}

void fill_write_status_with_int32(
    int32_t *target_values, 
    uint8_t *coded_values, uint8_t size, 
    status_packet_t *status)
{
    if (size != (NB_MOTORS * sizeof(int32_t)))
    {
        status->error = INSTRUCTION_ERROR;
        return;
    }

    memcpy(target_values, coded_values, size);
}