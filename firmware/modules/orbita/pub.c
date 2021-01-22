#include "pub.h"
#include "reachy.h"
#include "orbita.h"

void send_data_to_gate(container_t *src, uint8_t reg_type, uint8_t payload[], uint8_t payload_size)
{
    // [MSG_TYPE_ORBITA_PUB_DATA, ORBITA_ID, REG_TYPE, (VAL1)+, (VAL2)+, (VAL3)+]

    msg_t msg;
    msg.header.target = 1;
    msg.header.target_mode = ID;
    msg.header.cmd = ASK_PUB_CMD;
    msg.header.size = payload_size + 3;

    msg.data[0] = MSG_TYPE_ORBITA_PUB_DATA;
    msg.data[1] = ORBITA_ID;
    msg.data[2] = reg_type;
    memcpy(msg.data + 3, payload, payload_size);

    Luos_SendMsg(src, &msg);
}


void send_temperatures_to_gate(container_t *src, float temperatures[])
{
    send_data_to_gate(src, ORBITA_TEMPERATURE, (uint8_t *)temperatures, sizeof(float) * NB_MOTORS);
}

void send_positions_to_gate(container_t *src, int32_t positions[])
{
    send_data_to_gate(src, ORBITA_PRESENT_POSITION, (uint8_t *)positions, sizeof(int32_t) * NB_MOTORS);
}