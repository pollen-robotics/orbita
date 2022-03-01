#include "message.h"

#include "string.h"

#include "orbita.h"

uint8_t check_error_flag(uint8_t byte, error_t err)
{
    return (byte >> (7 - err) & 1);
}

void set_error_flag(uint8_t *byte, error_t err)
{
    *byte |= (1 << (7 - err));
}

void clear_error_flag(uint8_t *byte, error_t err)
{
    *byte &= ~(1 << (7 - err));
}

void fill_read_status_with_uint8(uint8_t *data, int nb, status_packet_t *p)
{
    p->payload_size = sizeof(uint8_t) * nb;
    memcpy(p->payload, data, p->payload_size);
}

void fill_read_status_with_uint32(uint32_t *data, int nb, status_packet_t *p)
{
    p->payload_size = sizeof(uint32_t) * nb;
    memcpy(p->payload, data, p->payload_size);
}

void fill_read_status_with_int32(int32_t *data, int nb, status_packet_t *p)
{
    p->payload_size = sizeof(int32_t) * nb;
    memcpy(p->payload, data, p->payload_size);
}

void fill_read_status_with_float(float *data, int nb, status_packet_t *p)
{
    p->payload_size = sizeof(float) * nb;
    memcpy(p->payload, data, p->payload_size);
}

void fill_write_status_with_uint8(
    uint8_t *target_values, 
    uint8_t *coded_values, uint8_t size, uint8_t nb,
    status_packet_t *status)
{
    if (size != (nb * sizeof(uint8_t)))
    {
        status->error = INSTRUCTION_ERROR;
        return;
    }

    memcpy(target_values, coded_values, size);
}

void fill_write_status_with_int32(
    int32_t *target_values, 
    uint8_t *coded_values, uint8_t size, uint8_t nb,
    status_packet_t *status)
{
    if (size != (nb * sizeof(int32_t)))
    {
        status->error = INSTRUCTION_ERROR;
        return;
    }

    memcpy(target_values, coded_values, size);
}

void fill_write_status_with_float(
    float *target_values, 
    uint8_t *coded_values, uint8_t size, uint8_t nb,
    status_packet_t *status)
{
    if (size != (nb * sizeof(float)))
    {
        status->error = INSTRUCTION_ERROR;
        return;
    }

    memcpy(target_values, coded_values, size);
}


uint8_t compute_crc(uint8_t *data, uint8_t size)
{
    uint8_t s = 0;
    for (uint8_t i=0; i < size; i++)
    {
        s += data[i];
    }
    return 255 - s;
}