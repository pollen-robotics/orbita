#ifndef PUBLISH_H
#define PUBLISH_H

#include "main.h"
#include "luos.h"

void send_data_to_gate(container_t *src, uint8_t reg_type, uint8_t payload[], uint8_t payload_size);

void send_temperatures_to_gate(container_t *src, float temperatures[]);
void send_positions_to_gate(container_t *src, int32_t positions[]);

#endif // PUBLISH_H