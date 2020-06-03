#include "main.h"
#include "controlled_motor.h"
#include "tim.h"
#include "math.h"
#include <float.h>
#include "luos.h"
#include "AS5045B.h"
#include "MAX31730.h"
#include "main.h"
#include "luos_board.h"

#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

#define ASSERV_PERIOD 1
#define SPEED_PERIOD 50
#define TEMP_PERIOD 1000
#define SPEED_NB_INTEGRATION SPEED_PERIOD / ASSERV_PERIOD
#define SAMPLING_PERIOD_MS 10.0
#define BUFFER_SIZE 1000

module_t *modules[3];

volatile motor_config_t motor[3];

asserv_pid_t position[3];
asserv_pid_t speed[3];
float errSpeedSum[3] = {0.0};
float motion_target_position[3] = {0.0};
volatile time_luos_t time;

// Position Asserv things
volatile float errAngleSum[3] = {0.0};
volatile float lastErrAngle[3] = {0.0};

// Speed Asserv things
volatile float lastErrSpeed[3] = {0.0};

// Control management
volatile control_mode_t control[3];

// Trajectory management (can be position or speed)
volatile float trajectory_buf[3][BUFFER_SIZE];
streaming_channel_t trajectory[3];
volatile angular_position_t last_position[3] = {0.0};

// measurement management (can be position or speed)
volatile float measurement_buf[3][BUFFER_SIZE];
streaming_channel_t measurement[3];

static uint8_t motor_id_from_module(module_t *module)
{
    uint8_t i = 0;
    for (i = 0; i < 3; i++)
    {
        if (modules[i] == module)
        {
            return i;
        }
    }
    return 4;
}

void HAL_SYSTICK_Motor_Callback(void)
{
    // ************* motion planning *************
    // ****** recorder management *********
    static uint32_t last_rec_systick = 0;
    if ((HAL_GetTick() - last_rec_systick) >= time_to_ms(time))
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            if (control[i].mode_rec)
            {
                // We have to save a sample of current position
                set_sample(&measurement[i], &motor[i].angular_position);
            }
        }
        last_rec_systick = HAL_GetTick();
    }
    // ****** trajectory management *********
    static uint32_t last_systick = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        if (control[i].mode_control == STOP)
        {
            reset_streaming_channel(&trajectory[i]);
        }
    }
    if ((HAL_GetTick() - last_systick) >= time_to_ms(time))
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            if ((get_nb_available_samples(&trajectory[i]) > 0) && (control[i].mode_control == PLAY))
            {
                if (motor[i].mode.mode_linear_position == 1)
                {
                    linear_position_t linear_position_tmp;
                    get_sample(&trajectory[i], &linear_position_tmp);
                    motor[i].target_angular_position = (linear_position_tmp * 360.0) / (3.141592653589793 * motor[i].wheel_diameter);
                }
                else
                {
                    get_sample(&trajectory[i], &motor[i].target_angular_position);
                }
            }
        }
        last_systick = HAL_GetTick();
    }
    // ****** Linear interpolation *********
    for (uint8_t i = 0; i < 3; i++)
    {
        if ((motor[i].mode.mode_angular_position || motor[i].mode.mode_linear_position) &&
            (motor[i].mode.mode_angular_speed || motor[i].mode.mode_linear_speed))
        {

            // speed control and position control are enabled
            // we need to move target position following target speed
            float increment = (fabs(motor[i].target_angular_speed) / 1000.0);
            if (fabs(motor[i].target_angular_position - last_position[i]) <= increment)
            {
                // target_position is the final target position
                motion_target_position[i] = motor[i].target_angular_position;
            }
            else if ((motor[i].target_angular_position - motor[i].angular_position) < 0.0)
            {
                motion_target_position[i] = last_position[i] - increment;
            }
            else
            {
                motion_target_position[i] = last_position[i] + increment;
            }
        }
        else
        {
            // target_position is the final target position
            motion_target_position[i] = motor[i].target_angular_position;
        }
        last_position[i] = motion_target_position[i];
    }
}

void set_ratio(uint8_t index, float ratio)
{
    // limit power value
    if (ratio < -motor[index].limit_ratio)
        ratio = -motor[index].limit_ratio;
    if (ratio > motor[index].limit_ratio)
        ratio = motor[index].limit_ratio;
    // transform power ratio to timer value
    uint16_t pulse;
    if (ratio > 0.0)
    {
        pulse = (uint16_t)(ratio * 85.0);
        if (index == 0)
        {
            TIM8->CCR1 = 0;
            TIM8->CCR2 = pulse;
        }
        if (index == 1)
        {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = pulse;
        }
        if (index == 2)
        {
            TIM20->CCR1 = 0;
            TIM20->CCR2 = pulse;
        }
    }
    else
    {
        pulse = (uint16_t)(-ratio * 85.0);
        if (index == 0)
        {
            TIM8->CCR1 = pulse;
            TIM8->CCR2 = 0;
        }
        if (index == 1)
        {
            TIM1->CCR1 = pulse;
            TIM1->CCR2 = 0;
        }
        if (index == 2)
        {
            TIM20->CCR1 = pulse;
            TIM20->CCR2 = 0;
        }
    }
}

void enable_motor(uint8_t index, char state)
{
    if (index == 0)
    {
        HAL_GPIO_WritePin(MOT2_EN_GPIO_Port, MOT2_EN_Pin, state);
    }
    if (index == 1)
    {
        HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, state);
    }
    if (index == 2)
    {
        HAL_GPIO_WritePin(MOT3_EN_GPIO_Port, MOT3_EN_Pin, state);
    }
}

void rx_mot_cb(module_t *module, msg_t *msg)
{
    static temperature_t last_temp[3] = {0.0};
    uint8_t index = motor_id_from_module(module);
    if (msg->header.cmd == ASK_PUB_CMD)
    {
        // Report management
        msg_t pub_msg;
        pub_msg.header.target_mode = ID;
        pub_msg.header.target = msg->header.source;
        if (motor[index].mode.angular_position)
        {
            if (control[index].mode_rec)
            {
                // send back a record stream
                pub_msg.header.cmd = ANGULAR_POSITION;
                luos_send_streaming(module, &pub_msg, &measurement[index]);
            }
            else
            {
                angular_position_to_msg(&motor[index].angular_position, &pub_msg);
                luos_send(module, &pub_msg);
            }
        }
        if (motor[index].mode.angular_speed)
        {
            angular_speed_to_msg(&motor[index].angular_speed, &pub_msg);
            luos_send(module, &pub_msg);
        }
        if (motor[index].mode.linear_position)
        {
            linear_position_to_msg(&motor[index].linear_position, &pub_msg);
            luos_send(module, &pub_msg);
        }
        if (motor[index].mode.linear_speed)
        {
            linear_speed_to_msg(&motor[index].linear_speed, &pub_msg);
            luos_send(module, &pub_msg);
        }
        if (motor[index].mode.current)
        {
            current_to_msg(&motor[index].current, &pub_msg);
            luos_send(module, &pub_msg);
        }
        if (motor[index].mode.temperature)
        {
            // Check if temperature moved and send it
            if (last_temp[index] != motor[index].temperature)
            {
                last_temp[index] = motor[index].temperature;
                temperature_to_msg(&motor[index].temperature, &pub_msg);
                luos_send(module, &pub_msg);
            }
        }
        return;
    }
    if (msg->header.cmd == PID)
    {
        // check the message size
        if (msg->header.size == sizeof(asserv_pid_t))
        {
            // fill the message infos
            if ((motor[index].mode.mode_angular_position || motor[index].mode.mode_linear_position) &&
                !(motor[index].mode.mode_angular_speed || motor[index].mode.mode_linear_speed))
            {
                // only position control is enable, we can save PID for positioning
                memcpy(&position[index], msg->data, msg->header.size);
            }
            if ((motor[index].mode.mode_angular_speed || motor[index].mode.mode_linear_speed) &&
                !(motor[index].mode.mode_angular_position || motor[index].mode.mode_linear_position))
            {
                // only speed control is enable, we can save PID for speed
                memcpy(&speed[index], msg->data, msg->header.size);
            }
        }
        return;
    }
    if (msg->header.cmd == PARAMETERS)
    {
        // check the message size
        if (msg->header.size == sizeof(motor_mode_t))
        {
            // fill the message infos
            memcpy(&motor[index].mode, msg->data, msg->header.size);
            enable_motor(index, motor[index].mode.mode_compliant == 0);
            if (motor[index].mode.mode_compliant == 0)
            {
                last_position[index] = motor[index].angular_position;
                errAngleSum[index] = 0.0;
                lastErrAngle[index] = 0.0;
                motor[index].target_angular_position = motor[index].angular_position;
            }
        }
        return;
    }
    if (msg->header.cmd == CONTROL)
    {
        control[index].unmap = msg->data[0];
        if (control[index].mode_control == 3)
        {
            // impossible value, go back to default values
            control[index].unmap = 0;
        }
        return;
    }
    if (msg->header.cmd == RESOLUTION)
    {
        // set the encoder resolution
        memcpy(&motor[index].resolution, msg->data, sizeof(float));
        return;
    }
    if (msg->header.cmd == REDUCTION)
    {
        // set the motor reduction
        memcpy(&motor[index].motor_reduction, msg->data, sizeof(float));
        return;
    }
    if (msg->header.cmd == REINIT)
    {
        // set state to 0
        motor[index].angular_position = 0.0;
        motor[index].target_angular_position = 0.0;
        errAngleSum[index] = 0.0;
        lastErrAngle[index] = 0.0;
        last_position[index] = 0.0;
        return;
    }
    if (msg->header.cmd == DIMENSION)
    {
        // set the wheel diameter m
        linear_position_from_msg(&motor[index].wheel_diameter, msg);
        return;
    }
    if (msg->header.cmd == RATIO)
    {
        // set the motor power ratio (no asserv)
        ratio_from_msg(&motor[index].target_ratio, msg);
        return;
    }
    if (msg->header.cmd == ANGULAR_POSITION)
    {
        if (motor[index].mode.mode_angular_position)
        {
            // Check message size
            if (msg->header.size == sizeof(float))
            {
                // set the motor target angular position
                last_position[index] = motor[index].angular_position;
                angular_position_from_msg(&motor[index].target_angular_position, msg);
            }
            else
            {
                // this is a trajectory, save it into streaming channel.
                luos_receive_streaming(module, msg, &trajectory[index]);
            }
        }
        return;
    }
    if (msg->header.cmd == ANGULAR_SPEED)
    {
        // set the motor target angular position
        if (motor[index].mode.mode_angular_speed)
        {
            angular_speed_from_msg(&motor[index].target_angular_speed, msg);
            // reset the integral factor for speed
            errSpeedSum[index] = 0.0;
        }
        return;
    }
    if (msg->header.cmd == LINEAR_POSITION)
    {
        // set the motor target linear position
        // Check message size
        if (msg->header.size == sizeof(float))
        {
            linear_position_t linear_position = 0.0;
            linear_position_from_msg(&linear_position, msg);
            motor[index].target_angular_position = (linear_position * 360.0) / (3.141592653589793 * motor[index].wheel_diameter);
        }
        else
        {
            // this is a trajectory, save it into ring buffer.
            luos_receive_streaming(module, msg, &trajectory[index]);
            // values will be converted one by one during trajectory management.
        }
        return;
    }
    if (msg->header.cmd == LINEAR_SPEED)
    {
        // set the motor target linear speed
        if (motor[index].wheel_diameter > 0.0)
        {
            linear_speed_t linear_speed = 0.0;
            linear_speed_from_msg(&linear_speed, msg);
            motor[index].target_angular_speed = (linear_speed * 360.0) / (3.141592653589793 * motor[index].wheel_diameter);
        }
        return;
    }
    if (msg->header.cmd == ANGULAR_POSITION_LIMIT)
    {
        // set the motor limit anglular position
        memcpy(motor[index].limit_angular_position, msg->data, 2 * sizeof(float));
        return;
    }
    if (msg->header.cmd == LINEAR_POSITION_LIMIT)
    {
        // set the motor target linear position
        if (motor[index].mode.mode_linear_position & (motor[index].wheel_diameter != 0))
        {
            linear_position_t linear_position[2] = {0.0, 0.0};
            memcpy(linear_position, msg->data, 2 * sizeof(linear_position_t));
            motor[index].limit_angular_position[0] = (linear_position[0] * 360.0) / (3.141592653589793 * motor[index].wheel_diameter);
            motor[index].limit_angular_position[1] = (linear_position[1] * 360.0) / (3.141592653589793 * motor[index].wheel_diameter);
        }
        return;
    }
    if (msg->header.cmd == RATIO_LIMIT)
    {
        // set the motor power ratio limit
        memcpy(&motor[index].limit_ratio, msg->data, sizeof(float));
        motor[index].limit_ratio = fabs(motor[index].limit_ratio);
        if (motor[index].limit_ratio > 100.0)
            motor[index].limit_ratio = 100.0;
        return;
    }
    if (msg->header.cmd == CURRENT_LIMIT)
    {
        // set the motor current limit
        current_from_msg(&motor[index].limit_current, msg);
        return;
    }
    if (msg->header.cmd == TIME)
    {
        // save time in ms
        time_from_msg(&time, msg);
        return;
    }
}

void controlled_motor_init(void)
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

    // setup default sampling frequency
    time = time_from_ms(SAMPLING_PERIOD_MS);

    // motor mode by default
    enable_motor(0, 0);
    enable_motor(1, 0);
    enable_motor(2, 0);
    motor[0].mode.mode_compliant = 1;
    motor[0].mode.current = 0;
    motor[0].mode.temperature = 1;
    motor[0].mode.mode_ratio = 1;
    motor[0].mode.mode_angular_position = 0;
    motor[0].mode.mode_angular_speed = 0;
    motor[0].mode.mode_linear_position = 0;
    motor[0].mode.mode_linear_speed = 0;
    motor[0].mode.angular_position = 1;
    motor[0].mode.angular_speed = 0;
    motor[0].mode.linear_position = 0;
    motor[0].mode.linear_speed = 0;

    // default motor configuration
    motor[0].motor_reduction = 52.0 / 24.0;
    motor[0].resolution = 4096;
    motor[0].wheel_diameter = 0.100f;

    // default motor limits
    motor[0].limit_ratio = 100.0;
    motor[0].limit_angular_position[MIN] = -FLT_MAX;
    motor[0].limit_angular_position[MAX] = FLT_MAX;
    motor[0].limit_current = 6.0;

    // Copy default values into other motors structs
    motor[1] = motor[0];
    motor[2] = motor[0];

    // Position PID default values
    position[0].p = 20.0;
    position[0].i = 0.1;
    position[0].d = 400.0;
    position[1] = position[0];
    position[2] = position[0];

    // Speed PID default values
    speed[0].p = 0.1;
    speed[0].i = 0.1;
    speed[0].d = 0.0;
    speed[1] = speed[0];
    speed[2] = speed[0];

    // Control mode default values
    control[0].unmap = 0; // PLAY and no REC
    control[1].unmap = 0; // PLAY and no REC
    control[2].unmap = 0; // PLAY and no REC

    // Init streaming channels
    trajectory[0] = create_streaming_channel(trajectory_buf[0], BUFFER_SIZE, sizeof(float));
    measurement[0] = create_streaming_channel(measurement_buf[0], BUFFER_SIZE, sizeof(float));
    trajectory[1] = create_streaming_channel(trajectory_buf[1], BUFFER_SIZE, sizeof(float));
    measurement[1] = create_streaming_channel(measurement_buf[1], BUFFER_SIZE, sizeof(float));
    trajectory[2] = create_streaming_channel(trajectory_buf[2], BUFFER_SIZE, sizeof(float));
    measurement[2] = create_streaming_channel(measurement_buf[2], BUFFER_SIZE, sizeof(float));

    // get sensor offset reduction and resolution
    AbsAng_struct_t angles[Nb_AS5045B_Chip] = {0};
    AS5045.ReadAngle(angles);
    motor[0].angular_position = (angular_position_t)((double)angles[0].Bits.AngPos / (double)(motor[0].motor_reduction * motor[0].resolution)) * 360.0;
    motor[1].angular_position = (angular_position_t)((double)angles[1].Bits.AngPos / (double)(motor[1].motor_reduction * motor[1].resolution)) * 360.0;
    motor[2].angular_position = (angular_position_t)((double)angles[2].Bits.AngPos / (double)(motor[2].motor_reduction * motor[2].resolution)) * 360.0;

    // set motors temperatures configurations
    MAX31730.SetFil(ENABLE);
    MAX31730.SetThr(35);
    // get motor temperatures
    temperature_t temperatures[3] = {0.0};
    MAX31730.Read((float *)temperatures);
    motor[0].temperature = temperatures[0];
    motor[1].temperature = temperatures[1];
    motor[2].temperature = temperatures[2];

    modules[0] = luos_module_create(rx_mot_cb, CONTROLLED_MOTOR_MOD, "orbita_mot1", STRINGIFY(VERSION));
    modules[1] = luos_module_create(rx_mot_cb, CONTROLLED_MOTOR_MOD, "orbita_mot2", STRINGIFY(VERSION));
    modules[2] = luos_module_create(rx_mot_cb, CONTROLLED_MOTOR_MOD, "orbita_mot3", STRINGIFY(VERSION));

    // enable ABI mode on sensors
    HAL_GPIO_WritePin(AS5045B_SS_GPIO_Port, AS5045B_SS_Pin, GPIO_PIN_RESET);
}

void controlled_motor_loop(void)
{
    // Time management vars
    static uint32_t last_asserv_systick = 0;
    static uint32_t last_temp_systick = 0;
    uint32_t timestamp = HAL_GetTick();
    uint32_t deltatime = timestamp - last_asserv_systick;

    // Speed measurement vars
    static char settings[3] = {0};
    static angular_position_t last_angular_positions[3][SPEED_NB_INTEGRATION];

    // ************* Values computation *************
    // angular_posistion => degree
    int32_t encoder_count = (int16_t)TIM2->CNT;
    TIM2->CNT = 0;
    motor[0].angular_position += (angular_position_t)((double)encoder_count / (double)(motor[0].motor_reduction * motor[0].resolution)) * 360.0;
    encoder_count = (int16_t)TIM3->CNT;
    TIM3->CNT = 0;
    motor[1].angular_position -= (angular_position_t)((double)encoder_count / (double)(motor[1].motor_reduction * motor[1].resolution)) * 360.0;
    encoder_count = (int16_t)TIM4->CNT;
    TIM4->CNT = 0;
    motor[2].angular_position -= (angular_position_t)((double)encoder_count / (double)(motor[2].motor_reduction * motor[2].resolution)) * 360.0;

    // linear_distance => m
    motor[0].linear_position = (motor[0].angular_position / 360.0) * M_PI * motor[0].wheel_diameter;
    motor[1].linear_position = (motor[1].angular_position / 360.0) * M_PI * motor[1].wheel_diameter;
    motor[2].linear_position = (motor[2].angular_position / 360.0) * M_PI * motor[2].wheel_diameter;
    // current => A
    HAL_ADCEx_InjectedStart(&hadc1);
    if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 1) == HAL_OK)
    {
        // Motor 0
        motor[0].current = ((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_REGULAR_RANK_1) * VREF / 4096.0f) / 2.4f;
        // Motor 1
        motor[1].current = ((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_REGULAR_RANK_3) * VREF / 4096.0f) / 2.4f;
        // Motor 2
        motor[2].current = ((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_REGULAR_RANK_2) * VREF / 4096.0f) / 2.4f;
    }
    HAL_ADCEx_InjectedStop(&hadc1);

    // temperature => °C
    if ((timestamp - last_temp_systick) > TEMP_PERIOD)
    {
        last_temp_systick = timestamp;
        temperature_t temperatures[3] = {0.0};
        MAX31730.Read((float *)temperatures);
        motor[0].temperature = temperatures[0];
        motor[1].temperature = temperatures[1];
        motor[2].temperature = temperatures[2];
    }

    if (deltatime >= ASSERV_PERIOD)
    {
        static float surpCurrentSum[3] = {0.0};
        last_asserv_systick = timestamp;
        for (int i = 0; i < 3; i++)
        {
            // angular_speed => degree/seconds
            // add the position value into unfiltered speed measurement
            for (int nbr = 0; nbr < (SPEED_NB_INTEGRATION - 1); nbr++)
            {
                if (!settings[i])
                {
                    last_angular_positions[i][nbr] = motor[i].angular_position;
                }
                else
                {
                    last_angular_positions[i][nbr] = last_angular_positions[i][nbr + 1];
                }
            }
            settings[i] = 1;
            last_angular_positions[i][SPEED_NB_INTEGRATION - 1] = motor[i].angular_position;
            motor[i].angular_speed = (last_angular_positions[i][SPEED_NB_INTEGRATION - 1] - last_angular_positions[i][0]) * 1000.0 / SPEED_PERIOD;

            // linear_speed => m/seconds
            motor[i].linear_speed = (motor[i].angular_speed / 360.0) * M_PI * motor[i].wheel_diameter;
            // ************* Limit clamping *************
            if (motion_target_position[i] < motor[i].limit_angular_position[MIN])
            {
                motion_target_position[i] = motor[i].limit_angular_position[MIN];
            }
            if (motion_target_position[i] > motor[i].limit_angular_position[MAX])
            {
                motion_target_position[i] = motor[i].limit_angular_position[MAX];
            }
            float currentfactor = 1.0f;
            currentfactor = motor[i].limit_current / (motor[i].current * 2);
            float surpCurrent = motor[i].current - motor[i].limit_current;
            surpCurrentSum[i] += surpCurrent;
            // If surpCurrentSum > 0 do a real coef
            if (surpCurrentSum[i] > 0.0)
            {
                currentfactor = motor[i].limit_current / (motor[i].limit_current + (surpCurrentSum[i] / 1.5));
            }
            else
            {
                surpCurrentSum[i] = 0.0;
                currentfactor = 1.0f;
            }
            if (motor[i].mode.mode_compliant)
            {
                //Motor is compliant, only manage motor limits
                if (motor[i].angular_position < motor[i].limit_angular_position[MIN])
                {
                    //re-enable motor to avoid bypassing motors limits
                    enable_motor(i, 1);
                    set_ratio(i, 100.0 * (motor[i].limit_angular_position[MIN] - motor[i].angular_position));
                }
                else if (motor[i].angular_position > motor[i].limit_angular_position[MAX])
                {
                    enable_motor(i, 1);
                    set_ratio(i, -100.0 * (motor[i].angular_position - motor[i].limit_angular_position[MAX]));
                }
                else
                {
                    enable_motor(i, 0);
                }
            }
            else if (motor[i].mode.mode_ratio)
            {
                set_ratio(i, motor[i].target_ratio * currentfactor);
            }
            else
            {
                // ************* position asserv *************
                // Target Position is managed by the motion planning interrupt (systick interrupt)
                float errAngle = 0.0;
                float dErrAngle = 0.0;
                float anglePower = 0.0;
                if (motor[i].mode.mode_angular_position || motor[i].mode.mode_linear_position)
                {
                    errAngle = motion_target_position[i] - motor[i].angular_position;
                    dErrAngle = (errAngle - lastErrAngle[i]) / deltatime;
                    errAngleSum[i] += (errAngle * (float)deltatime);
                    if (errAngleSum[i] < -100.0)
                        errAngleSum[i] = -100.0;
                    if (errAngleSum[i] > 100.0)
                        errAngleSum[i] = 100;
                    anglePower = (errAngle * position[i].p) + (errAngleSum[i] * position[i].i) + (dErrAngle * position[i].d); // raw PID command
                    lastErrAngle[i] = errAngle;
                }
                // ************* speed asserv *************
                float errSpeed = 0.0;
                float dErrSpeed = 0.0;
                float speedPower = 0.0;
                if (motor[i].mode.mode_angular_speed || motor[i].mode.mode_linear_speed)
                {
                    errSpeed = motor[i].target_angular_speed - motor[i].angular_speed;
                    dErrSpeed = (errSpeed - lastErrSpeed[i]) / deltatime;
                    errSpeedSum[i] += (errSpeed * (float)deltatime);
                    if (errSpeedSum[i] < -100.0)
                        errSpeedSum[i] = -100.0;
                    if (errSpeedSum[i] > 100.0)
                        errSpeedSum[i] = 100;
                    speedPower = ((errSpeed * speed[i].p) + (errSpeedSum[i] * speed[i].i) + (dErrSpeed * speed[i].d)); // raw PID command
                    lastErrSpeed[i] = errSpeed;
                }
                // ************* command merge *************
                if (!(motor[i].mode.mode_angular_position || motor[i].mode.mode_linear_position) &&
                    (motor[i].mode.mode_angular_speed || motor[i].mode.mode_linear_speed))
                {
                    // Speed control only
                    set_ratio(i, speedPower * currentfactor);
                }
                else
                {
                    // we use position control by default
                    set_ratio(i, anglePower * currentfactor);
                }
            }
        }
    }
}
