#include "orbita.h"

#include "tim.h"
#include "main.h"
#include "stdio.h"
#include "MAX31730.h"
#include "AS5045B.h"
#include "utils.h"

#include "message.h"
#include "rs485_com.h"
#include "fake_eeprom.h"

#define DEFAULT_ORBITA_ID 40

static uint8_t id = DEFAULT_ORBITA_ID;

static volatile int32_t present_positions[NB_MOTORS] = {0};
static volatile int32_t target_positions[NB_MOTORS] = {0};

static volatile uint8_t torques_enabled[NB_MOTORS] = {0};
static volatile float max_torque[NB_MOTORS] = {DEFAULT_MAX_TORQUE, DEFAULT_MAX_TORQUE, DEFAULT_MAX_TORQUE};

static volatile float pid[3] = {DEFAULT_P_GAIN, DEFAULT_I_GAIN, DEFAULT_D_GAIN};
static volatile int32_t d_position_errors[NB_MOTORS] = {0};
static volatile int32_t acc_position_errors[NB_MOTORS] = {0};

static int32_t zero[NB_MOTORS] = {0};

static float temperatures[NB_MOTORS] = {0};
static float temperatures_shutdown = DEFAULT_SHUTDOWN_TEMPERATURE;

static float temperature_fan_trigger_threshold = DEFAULT_TEMPERATURE_FAN_TRIGGER_THRESHOLD;

static uint8_t current_error = 0;


#define EEPROM_ADDR_ID 1 // (1 * sizeof(uint8_t) + 1 --> 2)
#define EEPROM_ADDR_PID 10 // (3 * sizeof(float) + 1 --> 13)
#define EEPROM_ADDR_ZERO 30 // (NB_MOTORS * sizeof(int32_t) + 1 --> 13)
#define EEPROM_ADDR_TEMP_SHUTDOWN 50 // sizeof(float) + 1


void Orbita_Init(void)
{
    setup_hardware();

    // If there is no custom value stored in EEPROM, the default value will be kept
    read_eeprom(EEPROM_ADDR_ID, sizeof(uint8_t), &id);
    read_eeprom(EEPROM_ADDR_PID, 3 * sizeof(float), (uint8_t *)pid);
    read_eeprom(EEPROM_ADDR_ZERO, 3 * sizeof(int32_t), (uint8_t *)zero);
    read_eeprom(EEPROM_ADDR_TEMP_SHUTDOWN, sizeof(float), (uint8_t *)&temperatures_shutdown);

    for (uint8_t motor_index=0; motor_index < NB_MOTORS; motor_index++)
    {
        set_motor_state(motor_index, 0);
    }

    rs485_wait_for_message_IT();

    status_led(0);
}

static instruction_packet_t instruction_packet;
static status_packet_t status_packet;
static uint8_t crc;

void Orbita_Loop(void)
{   
    static uint32_t last_temp_published = 0;
    if ((HAL_GetTick() - last_temp_published) >= TEMPERATURE_CHECK_PERIOD)
    {
        update_and_check_temperatures();
        last_temp_published = HAL_GetTick();
    } 

    if (rs485_get_instruction_packet(&instruction_packet, &crc) == HAL_OK)
    {
        if (instruction_packet.id == id)
        {
            Orbita_HandleMessage(&instruction_packet, crc, &status_packet);
            rs485_send_message_IT(id, &status_packet);
        }
        else
        {
            rs485_wait_for_message_IT();
        }
    }
}

void Orbita_HandleMessage(instruction_packet_t *instr, uint8_t crc, status_packet_t *status)
{
    status->error = Orbita_GetCurrentError();
    status->payload_size = 0;

    if (instr->crc != crc)
    {
        set_error_flag(&status->error, CHECKSUM_ERROR);
    }

    switch (instr->type)
    {
    case PING_MESSAGE:
        break;

    case READ_DATA_MESSAGE:
        Orbita_HandleReadData(instr->payload[0], status);
        break;

    case WRITE_DATA_MESSAGE:
        Orbita_HandleWriteData(instr->payload[0], instr->payload + 1, instr->payload_size - 1, status);
        break;

    default:
        set_error_flag(&status->error, INSTRUCTION_ERROR);
        break;
    }
}

void Orbita_HandleReadData(orbita_register_t reg, status_packet_t *status)
{
    switch (reg)
    {
    case ORBITA_TEMPERATURE_SHUTDOWN:
        fill_read_status_with_float(&temperatures_shutdown, 1, status);
        break;
    case ORBITA_PRESENT_POSITION:
        fill_read_status_with_int32((int32_t *)present_positions, NB_MOTORS, status);
        break;
    case ORBITA_POSITION_ABSOLUTE:
        {
            AbsAng_struct_t angles[Nb_AS5045B_Chip] = {0};
            AS5045.ReadAngle(angles);

            int32_t tmp[NB_MOTORS] = {
                (int32_t)angles[0].Bits.AngPos,
                (int32_t)angles[2].Bits.AngPos,
                (int32_t)angles[1].Bits.AngPos
            };
            fill_read_status_with_int32(tmp, NB_MOTORS, status);
        }
        break;
    case ORBITA_GOAL_POSITION:
        fill_read_status_with_int32((int32_t *)target_positions, NB_MOTORS, status);
        break;
    case ORBITA_TORQUE_ENABLE:
        fill_read_status_with_uint8((uint8_t *)torques_enabled, NB_MOTORS, status);
        break;
    case ORBITA_PID:
        fill_read_status_with_float((float *)pid, 3, status);
        break;
    case ORBITA_TEMPERATURE:
        fill_read_status_with_float(temperatures, NB_MOTORS, status);
        break;
    case ORBITA_FAN_TRIGGER_TEMPERATURE_THRESHOLD:
        fill_read_status_with_float(&temperature_fan_trigger_threshold, 1, status);
        break;
    case ORBITA_ZERO:
        fill_read_status_with_int32((int32_t *)zero, NB_MOTORS, status);
        break;
    case ORBITA_ID:
        fill_read_status_with_uint8(&id, 1, status);
        break;
    default:
        set_error_flag(&status->error, INSTRUCTION_ERROR);
        break;
    }
}

void Orbita_HandleWriteData(orbita_register_t reg, uint8_t *coded_values, uint8_t size, status_packet_t *status)
{
    switch (reg)
    {
    case ORBITA_TEMPERATURE_SHUTDOWN:
        fill_write_status_with_float(&temperatures_shutdown, coded_values, size, 1, status);
        write_eeprom(EEPROM_ADDR_TEMP_SHUTDOWN, sizeof(float), (uint8_t *)&temperatures_shutdown);
        break;
    case ORBITA_GOAL_POSITION:
        fill_write_status_with_int32((int32_t *)target_positions, coded_values, size, NB_MOTORS, status);
        break;
    case ORBITA_TORQUE_ENABLE:
        fill_write_status_with_uint8((uint8_t *)torques_enabled, coded_values, size, NB_MOTORS, status);
        for (uint8_t i=0; i < NB_MOTORS; i++)
        {
            set_motor_state(i, torques_enabled[i]);
        }
        break;
    case ORBITA_PID:
        fill_write_status_with_float((float *)pid, coded_values, size, 3, status);
        write_eeprom(EEPROM_ADDR_PID, 3 * sizeof(float), (uint8_t *)pid);
        break;
    case ORBITA_ZERO:
        fill_write_status_with_int32((int32_t *)zero, coded_values, size, NB_MOTORS, status);
        write_eeprom(EEPROM_ADDR_ZERO, NB_MOTORS * sizeof(int32_t), (uint8_t *)zero);
        break;
    case ORBITA_ID:
        fill_write_status_with_uint8(&id, coded_values, size, 1, status);
        write_eeprom(EEPROM_ADDR_ID, sizeof(uint8_t), &id);
        break;
    default:
        set_error_flag(&status->error, INSTRUCTION_ERROR);
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

    //Steve: do not read init value so we don't need to handle the zero
    /* present_positions[0] = (int32_t)angles[0].Bits.AngPos; */
    /* present_positions[1] = (int32_t)angles[2].Bits.AngPos; */
    /* present_positions[2] = (int32_t)angles[1].Bits.AngPos; */

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
            int32_t target = target_positions[i];

            int32_t pos_err = present_positions[i] - target;
            int32_t d_pos_err = (pos_err - d_position_errors[i]);
            int32_t i_err = clip(acc_position_errors[i] + pos_err, -MAX_ACC_ERR, MAX_ACC_ERR);

            float ratio = (float)pos_err * pid[0] + (float)i_err * pid[1] + (float)d_pos_err * pid[2];
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
        if (temperatures[i] > temperatures_shutdown)
        {
            // // WHAT TO DO ?
            // for (uint8_t m=0; m < NB_MOTORS; m++)
            // {
            //     set_motor_state(m, 0);
            // }
            set_error_flag(&current_error, OVERHEATING_ERROR);
        }
    }
    if (check_error_flag(current_error, OVERHEATING_ERROR))
    {
        uint8_t has_cooled_down = 1;
        for (uint8_t i=0; i < NB_MOTORS; i++)
        {
            if (temperatures[i] > temperature_fan_trigger_threshold)
            {
                has_cooled_down = 0;
            }
        }
        if (has_cooled_down)
        {
            clear_error_flag(&current_error, OVERHEATING_ERROR);
        }
    }
}

volatile int32_t pos[10][3];
volatile uint8_t pos_i = 0;


//Steve: this kind of breaks the position reading
#define INTEGRATION_LENGTH 1

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
