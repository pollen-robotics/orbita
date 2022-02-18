#include "orbita.h"

#include "tim.h"
#include "main.h"
#include "stdio.h"
#include "MAX31730.h"
#include "AS5045B.h"
#include "utils.h"
#include "usart.h"

static uint32_t keep_alive = 0;

#define ORBITA_ZERO_EEPROM_ADDR 34

int32_t zero_positions[NB_MOTORS];

static volatile int32_t present_positions[NB_MOTORS] = {0};
static volatile int32_t target_positions[NB_MOTORS] = {0};
static volatile int32_t position_limits[NB_MOTORS][2] = {0};

static volatile uint8_t torques_enabled[NB_MOTORS] = {0};
static volatile float max_torque[NB_MOTORS] = {DEFAULT_MAX_TORQUE, DEFAULT_MAX_TORQUE, DEFAULT_MAX_TORQUE};

static volatile float pid[NB_MOTORS][3] = {0};
static volatile int32_t d_position_errors[NB_MOTORS] = {0};
static volatile int32_t acc_position_errors[NB_MOTORS] = {0};

static float temperatures[NB_MOTORS] = {0.0};
static float temperatures_shutdown[NB_MOTORS] = {DEFAULT_SHUTDOWN_TEMPERATURE, DEFAULT_SHUTDOWN_TEMPERATURE, DEFAULT_SHUTDOWN_TEMPERATURE};

static uint8_t position_pub_period = DEFAULT_POSITION_PUB_PERIOD;
static float temperature_fan_trigger_threshold = DEFAULT_TEMPERATURE_FAN_TRIGGER_THRESHOLD;


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

#define SEND_BUFF_SIZE 10
#define RECV_BUFF_SIZE 2

uint8_t send_buff[SEND_BUFF_SIZE];
uint8_t recv_buff[RECV_BUFF_SIZE];


void Orbita_Loop(void)
{    
	// PASSER EN RX
	HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef ret = HAL_UART_Receive(&huart1, recv_buff, RECV_BUFF_SIZE, 1000);

    if (ret == HAL_TIMEOUT) {
        status_led(1);
    }

	if (ret == HAL_OK) {
        status_led(0);

		// PASSER EN TX
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);

		HAL_Delay(1);

		HAL_UART_Transmit(&huart1, send_buff, sizeof(send_buff), 5000);
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