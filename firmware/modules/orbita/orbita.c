#include "orbita.h"

#include "tim.h"
#include "main.h"
#include "stdio.h"
#include "MAX31730.h"
#include "AS5045B.h"
#include "utils.h"

#include "message.h"
#include "rs485_com.h"

static volatile int32_t present_positions[NB_MOTORS] = {0};
static volatile int32_t target_positions[NB_MOTORS] = {0};
static volatile int32_t position_limits[NB_MOTORS][2] = {0};

static volatile uint8_t torques_enabled[NB_MOTORS] = {0};
static volatile float max_torque[NB_MOTORS] = {DEFAULT_MAX_TORQUE, DEFAULT_MAX_TORQUE, DEFAULT_MAX_TORQUE};

static volatile float pid[NB_MOTORS][3] = {0};
static volatile int32_t d_position_errors[NB_MOTORS] = {0};
static volatile int32_t acc_position_errors[NB_MOTORS] = {0};

static float temperatures[NB_MOTORS] = {0};
static float temperatures_shutdown[NB_MOTORS] = {DEFAULT_SHUTDOWN_TEMPERATURE, DEFAULT_SHUTDOWN_TEMPERATURE, DEFAULT_SHUTDOWN_TEMPERATURE};

static float temperature_fan_trigger_threshold = DEFAULT_TEMPERATURE_FAN_TRIGGER_THRESHOLD;

static uint8_t current_error = 0;


void Orbita_Init(void)
{
    setup_hardware();

    for (uint8_t motor_index=0; motor_index < NB_MOTORS; motor_index++)
    {
        position_limits[motor_index][0] = DEFAULT_POSITION_LOWER_LIMIT;
        position_limits[motor_index][1] = DEFAULT_POSITION_UPPER_LIMIT;

        pid[motor_index][0] = DEFAULT_P_GAIN;
        pid[motor_index][1] = DEFAULT_I_GAIN;
        pid[motor_index][2] = DEFAULT_D_GAIN;

        set_motor_state(motor_index, 0);
    }

    status_led(1);
}

static instruction_packet_t instruction_packet;
static status_packet_t status_packet;

void Orbita_Loop(void)
{    
    static uint32_t last_temp_published = 0;
    if ((HAL_GetTick() - last_temp_published) >= TEMPERATURE_CHECK_PERIOD)
    {
        update_and_check_temperatures();
        last_temp_published = HAL_GetTick();
    }

    HAL_StatusTypeDef ret = rs485_read_message(ORBITA_ID, &instruction_packet);
    if (ret != HAL_OK)
    {
        status_led(1);
        return;
    }
        
    Orbita_HandleMessage(instruction_packet, &status_packet);
    
    ret = rs485_send_message(ORBITA_ID, status_packet);
    if (ret != HAL_OK)
    {
        status_led(1);
        return;
    }

    status_led(0);
}

void Orbita_HandleMessage(instruction_packet_t instr, status_packet_t *status)
{
    status->error = Orbita_GetCurrentError();
    status->size = 0;

    switch (instr.type)
    {
    case PING_MESSAGE:
        break;

    case READ_DATA_MESSAGE:
        Orbita_HandleReadData(instr.payload[0], status);
        break;

    case WRITE_DATA_MESSAGE:
        Orbita_HandleWriteData(instr.payload[0], instr.payload + 1, instr.size - 1, status);
        break;
    
    default:
        status->error |= (1 << INSTRUCTION_ERROR);
        break;
    }
}

void Orbita_HandleReadData(orbita_register_t reg, status_packet_t *status)
{
    switch (reg)
    {
    case ORBITA_PRESENT_POSITION:
        fill_read_status_with_int32((int32_t *)present_positions, status);
        break;

    case ORBITA_TEMPERATURE:
        fill_read_status_with_float(temperatures, status);
        break;
    
    default:
        status->error |= (1 << INSTRUCTION_ERROR);
        break;
    }
}

void Orbita_HandleWriteData(orbita_register_t reg, uint8_t *coded_values, uint8_t size, status_packet_t *status)
{
    switch (reg)
    {
    case ORBITA_TORQUE_ENABLE:
        fill_write_status_with_uint8((uint8_t *)torques_enabled, coded_values, size, status);
        break;
    
    case ORBITA_GOAL_POSITION:
        fill_write_status_with_int32((int32_t *)target_positions, coded_values, size, status);
        break;

    default:
        status->error |= (1 << INSTRUCTION_ERROR);
        break;
    }
}

uint8_t Orbita_GetCurrentError(void)
{
    return current_error;
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
    MAX31730.SetThr(temperature_fan_trigger_threshold);
}

void update_motor_asserv()
{
    for (uint8_t i=0; i < NB_MOTORS; i++)
    {
        if (torques_enabled[i] == 1)
        {
            int32_t target = clip(target_positions[i], position_limits[i][0], position_limits[i][1]);

            int32_t pos_err = present_positions[i] - target;
            int32_t d_pos_err = (pos_err - d_position_errors[i]);
            int32_t i_err = clip(acc_position_errors[i] + pos_err, -MAX_ACC_ERR, MAX_ACC_ERR);

            float ratio = (float)pos_err * pid[i][0] + (float)i_err * pid[i][1] + (float)d_pos_err * pid[i][2];
            set_motor_ratio(i, ratio);

            d_position_errors[i] = d_pos_err;
            acc_position_errors[i] = i_err;
        }
    }
}

void set_motor_state(uint8_t motor_index, uint8_t enable)
{
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

void update_and_check_temperatures() 
{
    read_temperatures(temperatures);

    for (uint8_t i=0; i < NB_MOTORS; i++)
    {
        if (temperatures[i] > temperatures_shutdown[i])
        {
            for (uint8_t m=0; m < NB_MOTORS; m++)
            {
                set_motor_state(m, 0);
            }
            current_error |= (1 << OVERHEATING_ERROR);
        }
    }
}

void status_led(uint8_t state)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (state == 0));
}

volatile int32_t pos[10][3];
volatile uint8_t pos_i = 0;

#define INTEGRATION_LENGTH 10

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // This will be called every 100µs

    pos[pos_i][0] = (int16_t)TIM2->CNT;
    pos[pos_i][1] = (int16_t)TIM3->CNT;
    pos[pos_i][2] = (int16_t)TIM4->CNT;

    pos_i++;

    if (pos_i == INTEGRATION_LENGTH)
    {
        for (uint8_t i=0; i < NB_MOTORS; i++)
        {
            int32_t p = 0;
            for (uint8_t j = 0; j < INTEGRATION_LENGTH; j++)
            {
                p += pos[j][i];
            }
            p /= INTEGRATION_LENGTH;

            if (i == 0)
            {
                present_positions[i] -= p;
            }
            else 
            {
                present_positions[i] += p;
            }
        }
        pos_i = 0;

        TIM2->CNT = 0;
        TIM3->CNT = 0;
        TIM4->CNT = 0;

        update_motor_asserv();
    }
}