#ifndef ORBITA_H
#define ORBITA_H

#include "luos.h"

#define ORBITA_ID 10

#define NB_MOTORS 3

#define DEFAULT_P_GAIN 1.0
#define DEFAULT_I_GAIN 0.0
#define DEFAULT_D_GAIN 0.0

#define DEFAULT_POSITION_LOWER_LIMIT -4096
#define DEFAULT_POSITION_UPPER_LIMIT 4096

#define POSITION_PUB_PERIOD 10 // in ms

#define TEMPERATURE_FAN_TRIGGER_THRESHOLD 35
#define DEFAULT_SHUTDOWN_TEMPERATURE 50
#define TEMPERATURE_PUB_PERIOD 1000 // in ms

#define KEEP_ALIVE_PERIOD 1100  // in ms

typedef enum 
{
    ORBITA_ANGLE_LIMIT = 0,
    ORBITA_TEMPERATURE_SHUTDOWN = 1,

    ORBITA_PRESENT_POSITION = 10,
    ORBITA_PRESENT_SPEED = 11,
    ORBITA_PRESENT_LOAD = 12,

    ORBITA_GOAL_POSITION = 20,
    ORBITA_MAX_SPEED = 21,
    ORBITA_MAX_TORQUE = 22,

    ORBITA_COMPLIANT = 30,
    ORBITA_PID = 31,
    ORBITA_TEMPERATURE = 32,

} orbita_register_t;

void Orbita_Init(void);
void Orbita_Loop(void);
void Orbita_MsgHandler(container_t *src, msg_t *msg);

uint8_t is_alive(void);
void status_led(uint8_t state);

void setup_hardware(void);

void update_present_positions(void);
void update_motor_asserv(void);
void set_motor_state(uint8_t motor_index, uint8_t enable);
void set_motor_ratio(uint8_t motor_index, float ratio);

void read_temperatures(float *temperatures);


#endif /* ORBITA */
