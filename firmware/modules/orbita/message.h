#ifndef ORBITA_MESSAGE_H
#define ORBITA_MESSAGE_H

#include <stdint.h>

#define MAX_PAYLOAD_SIZE 64

#define MSG_HEADER_SIZE 4

typedef enum {
    PING_MESSAGE = 1,
    READ_DATA_MESSAGE = 2,
    WRITE_DATA_MESSAGE = 3,
} message_type_t;

typedef enum {
    INPUT_VOLTAGE_ERROR = 0,
    ANGLE_LIMIT_ERROR = 1,
    OVERHEATING_ERROR = 2,
    RANGE_ERROR = 3,
    CHECKSUM_ERROR = 4,
    OVERLOAD_ERROR = 5,
    INSTRUCTION_ERROR = 6,
    NONE_ERROR = 7,
} error_t;

typedef struct {
    message_type_t type;

    uint8_t id;
    
    uint8_t size;
    uint8_t payload[MAX_PAYLOAD_SIZE];

} instruction_packet_t;


typedef struct {
    uint8_t id;
    uint8_t error;

    uint8_t size;
    uint8_t payload[MAX_PAYLOAD_SIZE];

} status_packet_t;


int8_t parse_message_header(uint8_t *recv_buff, uint8_t *id, uint8_t *length);
int8_t parse_message_instruction(uint8_t *recv_buff, uint8_t length, instruction_packet_t *p);


void fill_read_status_with_uint8(uint8_t *data, int nb, status_packet_t *p);
void fill_read_status_with_int32(int32_t *data, int nb, status_packet_t *p);
void fill_read_status_with_float(float *data, int nb, status_packet_t *p);

void fill_write_status_with_uint8(uint8_t *target_values, uint8_t *coded_values, uint8_t size, status_packet_t *status);
void fill_write_status_with_int32(int32_t *target_values, uint8_t *coded_values, uint8_t size, status_packet_t *status);

#endif // ORBITA_MESSAGE_H