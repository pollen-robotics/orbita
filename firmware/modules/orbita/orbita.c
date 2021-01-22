#include "orbita.h"

#include "pub.h"
#include "tim.h"
#include "main.h"
#include "luos.h"
#include "stdio.h"
#include "reachy.h"
#include "MAX31730.h"
#include "AS5045B.h"

static uint32_t keep_alive = 0;

static volatile int32_t present_positions[NB_MOTORS] = {0.0};
static int32_t target_positions[NB_MOTORS] = {0};
static uint8_t torques_enabled[NB_MOTORS] = {0};

static float temperatures[NB_MOTORS] = {0.0};

container_t *my_container;


void Orbita_Init(void)
{
    setup_hardware();

    char alias[15];
    sprintf(alias, "orbita_%d", ORBITA_ID);
    revision_t revision = {.unmap = REV};
    my_container = Luos_CreateContainer(Orbita_MsgHandler, CONTROLLER_MOTOR_MOD, alias, revision);

    for (uint8_t motor_index=0; motor_index < NB_MOTORS; motor_index++)
    {
        set_motor_state(motor_index, 0);
    }
}

void Orbita_Loop(void)
{
    update_present_positions();
    update_motor_asserv();

    if (!is_alive())
    {
        status_led(1);
        return;
    }
    status_led(0);

    static uint32_t last_pos_published = 0;
    if ((HAL_GetTick() - last_pos_published) > POSITION_PUB_PERIOD)
    {
        send_positions_to_gate(my_container, (int32_t *)present_positions);
        last_pos_published = HAL_GetTick();
    }

    static uint32_t last_temp_published = 0;
    if ((HAL_GetTick() - last_temp_published) > TEMPERATURE_PUB_PERIOD)
    {
        read_temperatures(temperatures);
        send_temperatures_to_gate(my_container, temperatures);
        last_temp_published = HAL_GetTick();
    }
}

void Orbita_MsgHandler(container_t *src, msg_t *msg)
{
    if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_KEEP_ALIVE))
    {
        keep_alive = HAL_GetTick();
    }
    else if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_ORBITA_GET_REG))
    {
        // [MSG_TYPE_ORBITA_GET_REG, ORBITA_ID, REG_TYPE]
    }
    else if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_ORBITA_SET_REG))
    {
        // [MSG_TYPE_ORBITA_SET_REG, ORBITA_ID, REG_TYPE, (MOTOR_ID, (VAL+))+]
        uint8_t orbita_id = msg->data[1];
        LUOS_ASSERT (orbita_id == ORBITA_ID);

        orbita_register_t reg = msg->data[2];
        if (reg == ORBITA_COMPLIANT)
        {
            uint8_t payload_per_motor = (1 + sizeof(uint8_t));
            uint8_t num_motors = (msg->header.size - 3) / payload_per_motor;
            LUOS_ASSERT (num_motors <= NB_MOTORS);
            LUOS_ASSERT (payload_per_motor * num_motors + 3 == msg->header.size);

            for (uint8_t i=0; i < num_motors; i++)
            {
                uint8_t *motor_data = msg->data + 3 + i * payload_per_motor;

                uint8_t motor_id = motor_data[0];
                LUOS_ASSERT (motor_id < NB_MOTORS);

                uint8_t compliant = motor_data[1];
                LUOS_ASSERT (compliant == 0 || compliant == 1);

                uint8_t torque_enable = 1 - compliant;
                if (torque_enable == 1)
                {
                    target_positions[motor_id] = present_positions[motor_id];
                    // TODO: Reset PID errors
                }

                set_motor_state(motor_id, torque_enable);
                torques_enabled[motor_id] = torque_enable;
            }
        }
        else if (reg == ORBITA_GOAL_POSITION)
        {
            uint8_t payload_per_motor = (1 + sizeof(int32_t));
            uint8_t num_motors = (msg->header.size - 3) / payload_per_motor;
            LUOS_ASSERT (num_motors <= NB_MOTORS);
            LUOS_ASSERT (payload_per_motor * num_motors + 3 == msg->header.size);

            for (uint8_t i=0; i < num_motors; i++)
            {
                uint8_t *motor_data = msg->data + 3 + i * payload_per_motor;

                uint8_t motor_id = motor_data[0];
                LUOS_ASSERT (motor_id < NB_MOTORS);

                uint32_t goal_position;
                memcpy(&goal_position, motor_data + 1, sizeof(int32_t));

                // TODO: check the limits

                target_positions[motor_id] = goal_position;
            }
        }
        else 
        {
            LUOS_ASSERT (0);
        }
    }
}

void setup_hardware(void)
{
     // Motor 0
    //     Start PWM
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    //     Setup motor into PWM Control Mode
    HAL_GPIO_WritePin(MOT2_PMODE_GPIO_Port, MOT2_PMODE_Pin, 1);
    //     Start AB quadrature encoder measurement
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    // Motor 1
    //     Start PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    //     Setup motor into PWM Control Mode
    HAL_GPIO_WritePin(MOT1_PMODE_GPIO_Port, MOT1_PMODE_Pin, 1);
    //     Start AB quadrature encoder measurement
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    // Motor 2
    //     Start PWM
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
    //     Setup motor into PWM Control Mode
    HAL_GPIO_WritePin(MOT3_PMODE_GPIO_Port, MOT3_PMODE_Pin, 1);
    //     Start AB quadrature encoder measurement
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    AbsAng_struct_t angles[Nb_AS5045B_Chip] = {0};
    AS5045.ReadAngle(angles);

    present_positions[0] = (int32_t)angles[0].Bits.AngPos;
    present_positions[1] = (int32_t)angles[1].Bits.AngPos;
    present_positions[2] = (int32_t)angles[2].Bits.AngPos;

    // enable ABI mode on sensors
    HAL_GPIO_WritePin(AS5045B_SS_GPIO_Port, AS5045B_SS_Pin, GPIO_PIN_RESET);

    // set motors temperatures configurations
    MAX31730.SetFil(ENABLE);
    MAX31730.SetThr(TEMPERATURE_FAN_TRIGGER_THRESHOLD);
}

void update_present_positions(void)
{
    present_positions[0] += (int16_t)TIM2->CNT;
    TIM2->CNT = 0;
    present_positions[1] -= (int16_t)TIM3->CNT;
    TIM3->CNT = 0;
    present_positions[2] -= (int16_t)TIM4->CNT;
    TIM4->CNT = 0;
}

#define P_GAIN 0.5

void update_motor_asserv(void)
{
    for (uint8_t i=0; i < NB_MOTORS; i++)
    {
        if (torques_enabled[i] == 1)
        {
            int32_t position_error = target_positions[i] - present_positions[i];
            float ratio = (float)position_error * P_GAIN;
            set_motor_ratio(i, ratio);
        }
    }
}

void set_motor_state(uint8_t motor_index, uint8_t enable)
{
    LUOS_ASSERT (motor_index == 0 || motor_index == 1 || motor_index == 2);
    LUOS_ASSERT (enable == 0 || enable == 1);

    if (motor_index == 0)
    {
        HAL_GPIO_WritePin(MOT2_EN_GPIO_Port, MOT2_EN_Pin, enable);
    }
    if (motor_index == 1)
    {
        HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, enable);
    }
    if (motor_index == 2)
    {
        HAL_GPIO_WritePin(MOT3_EN_GPIO_Port, MOT3_EN_Pin, enable);
    }
}

void set_motor_ratio(uint8_t motor_index, float ratio)
{
    LUOS_ASSERT(motor_index == 0 || motor_index == 1 || motor_index == 2);

    if (ratio < -100)
    {
        ratio = -100;
    }
    else if (ratio > 100)
    {
        ratio = 100;
    }

    uint16_t pulse_1, pulse_2;
    if (ratio > 0)
    {
        pulse_1 = 0;
        pulse_2 = (uint16_t)(ratio * 85.0);
    }
    else 
    {
        pulse_1 = (uint16_t)(-ratio * 85.0);
        pulse_2 = 0;
    }

    if (motor_index == 0)
    {
        TIM8->CCR1 = pulse_1;
        TIM8->CCR2 = pulse_2;
    }
    else if (motor_index == 1)
    {
        TIM1->CCR1 = pulse_1;
        TIM1->CCR2 = pulse_2;
    }
    else
    {
        TIM20->CCR1 = pulse_1;
        TIM20->CCR2 = pulse_2;
    }
}

void read_temperatures(float *temperatures)
{
    MAX31730.Read(temperatures);
}

uint8_t is_alive(void)
{
    if (keep_alive == 0)
    {
        return 0;
    }
    return (HAL_GetTick() - keep_alive) <= KEEP_ALIVE_PERIOD;
}

void status_led(uint8_t state)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (state == 0));
}

void HAL_SYSTICK_Motor_Callback(void)
{
}