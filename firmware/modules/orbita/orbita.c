#include "orbita.h"

#include "pub.h"
#include "tim.h"
#include "main.h"
#include "luos.h"
#include "stdio.h"
#include "reachy.h"
#include "MAX31730.h"
#include "AS5045B.h"
#include "utils.h"

static uint32_t keep_alive = 0;

#define ORBITA_ZERO_EEPROM_ADDR 34

int32_t zero_positions[NB_MOTORS];

static volatile int32_t present_positions[NB_MOTORS] = {0};
static volatile int32_t target_positions[NB_MOTORS] = {0};
static volatile int32_t position_limits[NB_MOTORS][2] = {0};

static volatile uint8_t torques_enabled[NB_MOTORS] = {0};
static volatile float max_torque[NB_MOTORS] = {DEFAULT_MAX_TORQUE, DEFAULT_MAX_TORQUE, DEFAULT_MAX_TORQUE};

static volatile float pid[NB_MOTORS][3] = {0};
static volatile int32_t position_errors[NB_MOTORS] = {0};
static volatile int32_t acc_position_errors[NB_MOTORS] = {0};

static float temperatures[NB_MOTORS] = {0.0};
static float temperatures_shutdown[NB_MOTORS] = {DEFAULT_SHUTDOWN_TEMPERATURE, DEFAULT_SHUTDOWN_TEMPERATURE, DEFAULT_SHUTDOWN_TEMPERATURE};

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
        position_limits[motor_index][0] = DEFAULT_POSITION_LOWER_LIMIT;
        position_limits[motor_index][1] = DEFAULT_POSITION_UPPER_LIMIT;

        pid[motor_index][0] = DEFAULT_P_GAIN;
        pid[motor_index][1] = DEFAULT_I_GAIN;
        pid[motor_index][2] = DEFAULT_D_GAIN;

        set_motor_state(motor_index, 0);
    }

    status_led (0);
}

void Orbita_Loop(void)
{
    static uint32_t last_asserv = 0;
    if ((HAL_GetTick() - last_asserv) >= ASSERV_PERIOD)
    {
        update_present_positions();
        update_motor_asserv((float)(HAL_GetTick() - last_asserv));
        last_asserv = HAL_GetTick();
    }

    if (!is_alive())
    {
        status_led(1);
        return;
    }
    status_led(0);

    static uint32_t last_pos_published = 0;
    if ((HAL_GetTick() - last_pos_published) >= POSITION_PUB_PERIOD)
    {
        send_data_to_gate(my_container, ORBITA_PRESENT_POSITION, (uint8_t *)present_positions, sizeof(int32_t) * NB_MOTORS);
        last_pos_published = HAL_GetTick();
    }

    static uint32_t last_temp_published = 0;
    if ((HAL_GetTick() - last_temp_published) >= TEMPERATURE_PUB_PERIOD)
    {
        read_temperatures(temperatures);
        send_data_to_gate(my_container, ORBITA_TEMPERATURE, (uint8_t *)temperatures, sizeof(float) * NB_MOTORS);
        last_temp_published = HAL_GetTick();

        for (uint8_t i=0; i < NB_MOTORS; i++)
        {
            if (temperatures[i] > temperatures_shutdown[i])
            {
                for (uint8_t m=0; m < NB_MOTORS; m++)
                {
                    set_motor_state(m, 0);
                }
                LUOS_ASSERT (0);
            }
        }
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
        // <-- [MSG_TYPE_ORBITA_GET_REG, ORBITA_ID, REG_TYPE]
        // --> [MSG_TYPE_ORBITA_PUB_DATA, ORBITA_ID, REG_TYPE, (VAL1)+, (VAL2)+, (VAL3)+]

        uint8_t orbita_id = msg->data[1];
        LUOS_ASSERT (orbita_id == ORBITA_ID);

        orbita_register_t reg = msg->data[2];
        if (reg == ORBITA_PRESENT_POSITION || reg == ORBITA_TEMPERATURE)
        {
            // Just do nothing. The position/temperature will be published anyway
        }
        else if (reg == ORBITA_GOAL_POSITION)
        {
            send_data_to_gate(my_container, ORBITA_GOAL_POSITION, (uint8_t *)target_positions, sizeof(int32_t) * NB_MOTORS);
        }
        else if (reg == ORBITA_TORQUE_ENABLE)
        {
            send_data_to_gate(my_container, ORBITA_TORQUE_ENABLE, (uint8_t *)torques_enabled, sizeof(uint8_t) * NB_MOTORS);
        }
        else if (reg == ORBITA_PID)
        {
            send_data_to_gate(my_container, ORBITA_PID, (uint8_t *)pid, sizeof(float) * NB_MOTORS * 3);
        }
        else if (reg == ORBITA_ANGLE_LIMIT)
        {
            send_data_to_gate(my_container, ORBITA_ANGLE_LIMIT, (uint8_t *)position_limits, sizeof(int32_t) * NB_MOTORS * 2);
        }
        else if (reg == ORBITA_TEMPERATURE_SHUTDOWN)
        {
            send_data_to_gate(my_container, ORBITA_TEMPERATURE_SHUTDOWN, (uint8_t *)temperatures_shutdown, sizeof(float) * NB_MOTORS);
        }
        else if (reg == ORBITA_MAX_TORQUE)
        {
            send_data_to_gate(my_container, ORBITA_MAX_TORQUE, (uint8_t *)max_torque, sizeof(float) * NB_MOTORS);
        }
        else if (reg == ORBITA_ZERO)
        {
            Orbita_FlashReadLuosMemoryInfo(ORBITA_ZERO_EEPROM_ADDR, sizeof(int32_t) * NB_MOTORS, (uint8_t *)zero_positions);
            send_data_to_gate(my_container, ORBITA_ZERO, (uint8_t *)zero_positions, sizeof(int32_t) * NB_MOTORS);
        }
        else if (reg == ORBITA_POSITION_ABSOLUTE)
        {
            AbsAng_struct_t angles[Nb_AS5045B_Chip] = {0};
            AS5045.ReadAngle(angles);

            int32_t absolute_positions[NB_MOTORS];
            absolute_positions[0] = (int32_t)angles[0].Bits.AngPos;
            absolute_positions[1] = (int32_t)angles[2].Bits.AngPos;
            absolute_positions[2] = (int32_t)angles[1].Bits.AngPos;
            send_data_to_gate(my_container, ORBITA_POSITION_ABSOLUTE, (uint8_t *)absolute_positions, sizeof(int32_t) * NB_MOTORS);
        }
        else if (reg == ORBITA_MAGNETIC_QUALITY)
        {
            AbsAng_struct_t angles[Nb_AS5045B_Chip] = {0};
            AS5045.ReadAngle(angles);

            uint8_t quality[NB_MOTORS][3];
            for (uint8_t i=0; i < NB_MOTORS; i++)
            {
                quality[i][0] = angles[i].Bits.MagInc;
                quality[i][1] = angles[i].Bits.MagDec;
                quality[i][2] = angles[i].Bits.Lin;
            }

            send_data_to_gate(my_container, ORBITA_MAGNETIC_QUALITY, (uint8_t *)quality, sizeof(uint8_t) * NB_MOTORS * 3);
        }
        else if (reg == ORBITA_FAN_STATE)
        {
            uint8_t fan_status[NB_MOTORS];
            for (uint8_t i = 0; i < NB_MOTORS; i++)
            {
                fan_status[i] = MAX31730.Status();
            }
            send_data_to_gate(my_container, ORBITA_FAN_STATE, &fan_status, sizeof(uint8_t) * NB_MOTORS);
        }
        else 
        {
            LUOS_ASSERT (0);
        }
        // case ORBITA_PRESENT_SPEED:
        // case ORBITA_PRESENT_LOAD:
        // case ORBITA_MAX_SPEED:
    }
    else if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_ORBITA_SET_REG))
    {
        // [MSG_TYPE_ORBITA_SET_REG, ORBITA_ID, REG_TYPE, (MOTOR_ID, (VAL+))+]
        uint8_t orbita_id = msg->data[1];
        LUOS_ASSERT (orbita_id == ORBITA_ID);

        orbita_register_t reg = msg->data[2];

        if (reg == ORBITA_TORQUE_ENABLE)
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

                uint8_t torque_enable = motor_data[1];
                LUOS_ASSERT (torque_enable == 0 || torque_enable == 1);

                if (torque_enable == 1)
                {
                    target_positions[motor_id] = present_positions[motor_id];

                    // Reset PID errors
                    position_errors[motor_id] = 0;
                    acc_position_errors[motor_id] = 0;
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

                memcpy((int32_t *)target_positions + motor_id, motor_data + 1, sizeof(int32_t));
            }
        }
        else if (reg == ORBITA_ANGLE_LIMIT)
        {
            uint8_t payload_per_motor = (1 + sizeof(int32_t) * 2);
            uint8_t num_motors = (msg->header.size - 3) / payload_per_motor;
            LUOS_ASSERT (num_motors <= NB_MOTORS);
            LUOS_ASSERT (payload_per_motor * num_motors + 3 == msg->header.size);

            for (uint8_t i=0; i < num_motors; i++)
            {
                uint8_t *motor_data = msg->data + 3 + i * payload_per_motor;
                uint8_t motor_id = motor_data[0];
                LUOS_ASSERT (motor_id < NB_MOTORS);

                memcpy((int32_t *)position_limits + 2 * motor_id, motor_data + 1, sizeof(int32_t) * 2);
            }
        }
        else if (reg == ORBITA_TEMPERATURE_SHUTDOWN)
        {
            uint8_t payload_per_motor = (1 + sizeof(float));
            uint8_t num_motors = (msg->header.size - 3) / payload_per_motor;
            LUOS_ASSERT (num_motors <= NB_MOTORS);
            LUOS_ASSERT (payload_per_motor * num_motors + 3 == msg->header.size);

            for (uint8_t i=0; i < num_motors; i++)
            {
                uint8_t *motor_data = msg->data + 3 + i * payload_per_motor;
                uint8_t motor_id = motor_data[0];
                LUOS_ASSERT (motor_id < NB_MOTORS);

                memcpy((float *)temperatures_shutdown + motor_id, motor_data + 1, sizeof(float));
            }
        }
        else if (reg == ORBITA_PID)
        {
            uint8_t payload_per_motor = (1 + sizeof(float) * 3);
            uint8_t num_motors = (msg->header.size - 3) / payload_per_motor;
            LUOS_ASSERT (num_motors <= NB_MOTORS);
            LUOS_ASSERT (payload_per_motor * num_motors + 3 == msg->header.size);

            for (uint8_t i=0; i < num_motors; i++)
            {
                uint8_t *motor_data = msg->data + 3 + i * payload_per_motor;
                uint8_t motor_id = motor_data[0];
                LUOS_ASSERT (motor_id < NB_MOTORS);

                memcpy((float *)pid + 3 * motor_id, motor_data + 1, sizeof(float) * 3);
            }
        }
        else if (reg == ORBITA_MAX_TORQUE)
        {
            uint8_t payload_per_motor = (1 + sizeof(float));
            uint8_t num_motors = (msg->header.size - 3) / payload_per_motor;
            LUOS_ASSERT (num_motors <= NB_MOTORS);
            LUOS_ASSERT (payload_per_motor * num_motors + 3 == msg->header.size);

            for (uint8_t i=0; i < num_motors; i++)
            {
                uint8_t *motor_data = msg->data + 3 + i * payload_per_motor;
                uint8_t motor_id = motor_data[0];
                LUOS_ASSERT (motor_id < NB_MOTORS);

                float val;
                memcpy(&val + motor_id, motor_data + 1, sizeof(float));
                val = clip(val, -100, 100);
                max_torque[motor_id] = val;
            }
        }
        // case ORBITA_MAX_SPEED:
        else if (reg == ORBITA_ZERO)
        {
            status_led (1);

            AbsAng_struct_t angles[Nb_AS5045B_Chip] = {0};
            AS5045.ReadAngle(angles);

            zero_positions[0] = (int32_t)angles[0].Bits.AngPos;
            zero_positions[1] = (int32_t)angles[2].Bits.AngPos;
            zero_positions[2] = (int32_t)angles[1].Bits.AngPos;

            Orbita_FlashWriteLuosMemoryInfo(ORBITA_ZERO_EEPROM_ADDR, 3 * sizeof(int32_t), (uint8_t *)zero_positions);

            status_led (0);
        }
        else if (reg == ORBITA_RECALIBRATE)
        {
            AbsAng_struct_t angles[Nb_AS5045B_Chip] = {0};
            AS5045.ReadAngle(angles);

            present_positions[0] = (int32_t)angles[0].Bits.AngPos;
            present_positions[1] = (int32_t)angles[2].Bits.AngPos;
            present_positions[2] = (int32_t)angles[1].Bits.AngPos;

            TIM2->CNT = 0;
            TIM3->CNT = 0;
            TIM4->CNT = 0;
        }
        else if (reg == ORBITA_FAN_STATE)
        {
            // TODO: can we manually trigger the fan?
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
    present_positions[1] = (int32_t)angles[2].Bits.AngPos;
    present_positions[2] = (int32_t)angles[1].Bits.AngPos;

    // enable ABI mode on sensors
    HAL_GPIO_WritePin(AS5045B_SS_GPIO_Port, AS5045B_SS_Pin, GPIO_PIN_RESET);

    // set motors temperatures configurations
    MAX31730.SetFil(ENABLE);
    MAX31730.SetThr(TEMPERATURE_FAN_TRIGGER_THRESHOLD);
}

void update_present_positions(void)
{
    present_positions[0] -= (int16_t)TIM2->CNT;
    TIM2->CNT = 0;
    present_positions[1] += (int16_t)TIM3->CNT;
    TIM3->CNT = 0;
    present_positions[2] += (int16_t)TIM4->CNT;
    TIM4->CNT = 0;
}

void update_motor_asserv(float dt)
{
    for (uint8_t i=0; i < NB_MOTORS; i++)
    {
        if (torques_enabled[i] == 1)
        {
            int32_t target = clip(target_positions[i], position_limits[i][0], position_limits[i][1]);

            int32_t p_err = present_positions[i] - target;
            int32_t d_err = (position_errors[i] - p_err) / dt;
            int32_t acc_err = clip(acc_position_errors[i] + p_err, -MAX_ACC_ERR, MAX_ACC_ERR);

            float ratio = (float)p_err * pid[i][0] + (float)acc_err * pid[i][1] + (float)d_err * pid[i][2];
            set_motor_ratio(i, ratio);

            position_errors[i] = d_err;
            acc_position_errors[i] = acc_err;
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
    ratio = clip(ratio, -max_torque[motor_index], max_torque[motor_index]);

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
    float temp[NB_MOTORS];
    MAX31730.Read(temp);

    temperatures[0] = temp[0];
    temperatures[1] = temp[2];
    temperatures[2] = temp[1];

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