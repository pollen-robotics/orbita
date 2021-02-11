#ifndef ORBITA_H
#define ORBITA_H

#include "luos.h"

#define ORBITA_ID 10

#define NB_MOTORS 3

#define DEFAULT_MAX_TORQUE 100.0

#define ASSERV_PERIOD 1 // in ms

#define DEFAULT_P_GAIN 1.0
#define DEFAULT_I_GAIN 0.0
#define DEFAULT_D_GAIN 0.0

#define MAX_ACC_ERR 100

#define DEFAULT_POSITION_LOWER_LIMIT -12888
#define DEFAULT_POSITION_UPPER_LIMIT 12888

#define POSITION_PUB_PERIOD 10 // in ms

#define TEMPERATURE_FAN_TRIGGER_THRESHOLD 35.0
#define DEFAULT_SHUTDOWN_TEMPERATURE 50.0
#define TEMPERATURE_PUB_PERIOD 1000 // in ms

#define KEEP_ALIVE_PERIOD 1100  // in ms

typedef enum 
{
    ORBITA_ANGLE_LIMIT = 0,
    ORBITA_TEMPERATURE_SHUTDOWN = 1,

    ORBITA_PRESENT_POSITION = 10,
    ORBITA_PRESENT_SPEED = 11,
    ORBITA_PRESENT_LOAD = 12,
    ORBITA_POSITION_ABSOLUTE = 13,

    ORBITA_GOAL_POSITION = 20,
    ORBITA_MAX_SPEED = 21,
    ORBITA_MAX_TORQUE = 22,

    ORBITA_TORQUE_ENABLE = 30,
    ORBITA_PID = 31,
    ORBITA_TEMPERATURE = 32,

    ORBITA_ZERO = 40,
    ORBITA_RECALIBRATE = 41,
    ORBITA_MAGNETIC_QUALITY = 42,

} orbita_register_t;

void Orbita_Init(void);
void Orbita_Loop(void);
void Orbita_MsgHandler(container_t *src, msg_t *msg);

uint8_t is_alive(void);
void status_led(uint8_t state);

void setup_hardware(void);

void update_present_positions(void);
void set_motor_state(uint8_t motor_index, uint8_t enable);
void set_motor_ratio(uint8_t motor_index, float ratio);

void read_temperatures(float *temperatures);


#endif /* ORBITA */
