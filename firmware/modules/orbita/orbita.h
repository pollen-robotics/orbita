#ifndef ORBITA_H
#define ORBITA_H

#include <stdint.h>

#include "message.h"


#define NB_MOTORS 3

#define DEFAULT_MAX_TORQUE 100.0

#define DEFAULT_P_GAIN 0.15
#define DEFAULT_I_GAIN 0.025
#define DEFAULT_D_GAIN 0.0165

// #define MAX_ACC_ERR 65535
#define MAX_ACC_ERR 10000

#define DEFAULT_POSITION_LOWER_LIMIT -12888
#define DEFAULT_POSITION_UPPER_LIMIT 12888

#define DEFAULT_TEMPERATURE_FAN_TRIGGER_THRESHOLD 35.0
#define DEFAULT_SHUTDOWN_TEMPERATURE 50.0
#define TEMPERATURE_CHECK_PERIOD 1000 // in ms

typedef enum
{
    // ORBITA_ANGLE_LIMIT = 0,
    ORBITA_TEMPERATURE_SHUTDOWN = 1,

    ORBITA_PRESENT_POSITION = 10,
    // ORBITA_PRESENT_SPEED = 11,
    // ORBITA_PRESENT_LOAD = 12,
    ORBITA_POSITION_ABSOLUTE = 13,

    ORBITA_GOAL_POSITION = 20,
    // ORBITA_MAX_SPEED = 21,
    // ORBITA_MAX_TORQUE = 22,

    ORBITA_TORQUE_ENABLE = 30,
    ORBITA_PID = 31,
    ORBITA_TEMPERATURE = 32,

    ORBITA_ZERO = 40,
    // ORBITA_RECALIBRATE = 41,
    // ORBITA_MAGNETIC_QUALITY = 42,

    // ORBITA_FAN_STATE = 50,
    ORBITA_FAN_TRIGGER_TEMPERATURE_THRESHOLD = 51,

    // ORBITA_POSITION_PUB_PERIOD = 60,

    ORBITA_ID = 70,

} orbita_register_t;

void Orbita_Init(void);
void Orbita_Loop(void);


void Orbita_HandleMessage(instruction_packet_t *instr, uint8_t crc, status_packet_t *status);
void Orbita_HandleReadData(orbita_register_t reg, status_packet_t *status);
void Orbita_HandleWriteData(orbita_register_t reg, uint8_t *coded_values, uint8_t size, status_packet_t *status);

uint8_t Orbita_GetCurrentError(void);

void status_led(uint8_t state);

void setup_hardware(void);

void update_present_positions(void);
void set_motor_state(uint8_t motor_index, uint8_t enable);
void set_motor_ratio(uint8_t motor_index, float ratio);
// void set_motor_ratio(uint8_t motor_index, int32_t ratio);

void read_temperatures(float *temperatures);
void update_and_check_temperatures();

#endif /* ORBITA */
