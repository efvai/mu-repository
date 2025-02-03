#include "virtual_motor.h"

#include "main.h"

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define VIRTUAL_ENCODER_PHASE_A_GPIO_PORT GPIOD
#define VIRTUAL_ENCODER_PHASE_A_GPIO_PIN GPIO_PIN_9
#define VIRTUAL_ENCODER_PHASE_B_GPIO_PORT GPIOD
#define VIRTUAL_ENCODER_PHASE_B_GPIO_PIN GPIO_PIN_10

extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim13;

typedef struct
{
    float voltage;
    float v_bus;

    float current;
    float torque;

    float angle_vel;
    float angle_vel_rpm;
    float angle;
    float angle_deg;
    uint8_t encoder_phase_a;
    uint8_t encoder_phase_b;

    float ind;
    float res;
    float ktke;
    float friction;
    float inertia;
    float load_torque;
    float effective_torque;

    float dt;
    bool is_connected;
} virtual_motor_t;

virtual_motor_t g_vm = {0};

// private functions proto
void virtual_encoder_run(float angle);
void virtual_current_sensor_run(float current);
uint16_t adc_normalization(float value, float min_bound, float max_bound);

void virtual_motor_reinit()
{
    HAL_DAC_Start(&hdac, TIM_CHANNEL_1);
    HAL_DAC_Start(&hdac, TIM_CHANNEL_2);
    g_vm.v_bus = 12.0f;
    g_vm.ind = 0.01f;
    g_vm.res = 0.338f;
    g_vm.ktke = 0.029f;
    g_vm.friction = 0.5e-5f;
    g_vm.inertia = 0.0002f;
    g_vm.load_torque = 0.0f;

    g_vm.is_connected = false;

    virtual_motor_reset();

    g_vm.dt = 1.0f / (float)(SystemCoreClock / 2 / (TIM13->ARR + 1));
    HAL_TIM_Base_Start_IT(&htim13);
}

void virtual_motor_reset()
{
    g_vm.voltage = 0.0f;
    g_vm.current = 0.0f;
    g_vm.torque = 0.0f;
    g_vm.angle_vel = 0.0f;
    g_vm.angle_vel_rpm = 0.0f;
    g_vm.angle = 0.0f;
    g_vm.angle_deg = 0.0f;
    g_vm.encoder_phase_a = 0;
    g_vm.encoder_phase_b = 0;
}

// returns a string literal
char* virtual_motor_parse_command(char *cmd) {
	if (cmd == NULL) {
		return NULL;
	}
	if (!strcmp(cmd, "vm_connect\r\n"))
	{
		virtual_motor_commander(VM_CONNECT);
		return "virtual motor connected\r\n";
	}
	else if (!strcmp(cmd, "vm_disconnect\r\n"))
	{
		virtual_motor_commander(VM_DISCONNECT);
		return "virtual motor disconnected\r\n";
	}
	else if (!strcmp(cmd, "vm_load_inc\r\n"))
	{
		virtual_motor_commander(VM_LOAD_INC);
		return "virtual motor load torque increased\r\n";
	}
	else if (!strcmp(cmd, "vm_load_dec\r\n"))
	{
		virtual_motor_commander(VM_LOAD_DEC);
		return "virtual motor load torque decreased\r\n";
	}
	return NULL;
}

void virtual_motor_commander(VM_COMMAND command)
{
    switch (command)
    {
    case VM_CONNECT:
        virtual_motor_reset();
        g_vm.is_connected = true;
        break;
    case VM_DISCONNECT:
        g_vm.is_connected = false;
        virtual_motor_reset();
        break;
    case VM_LOAD_INC:
    	g_vm.load_torque += 0.05f;
    	break;
    case VM_LOAD_DEC:
    	g_vm.load_torque -= 0.05f;
    	break;
    default:
        break;
    }
}

void virtual_motor_run(float duty_cycle)
{
    if (!g_vm.is_connected)
    {
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)2048);
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)2048);
        return;
    }
    g_vm.voltage = duty_cycle * g_vm.v_bus;
    g_vm.torque = g_vm.ktke * g_vm.current;
    g_vm.effective_torque = (g_vm.torque - g_vm.load_torque);
    g_vm.angle_vel += (g_vm.effective_torque - g_vm.friction * g_vm.angle_vel) / g_vm.inertia * g_vm.dt;
    float di_dt = (g_vm.voltage - g_vm.res * g_vm.current - g_vm.ktke * g_vm.angle_vel) / g_vm.ind;
    g_vm.current += di_dt * g_vm.dt;
    g_vm.angle += g_vm.angle_vel * g_vm.dt;
    g_vm.angle = fmodf(g_vm.angle, 2.0f * PI);
    if (g_vm.angle < 0.0f)
    {
        g_vm.angle += 2.0f * PI;
    }
    g_vm.angle_deg = g_vm.angle * (180.0f / PI);
    g_vm.angle_vel_rpm = g_vm.angle_vel * 9.549297f;

    virtual_current_sensor_run(g_vm.current);
    virtual_encoder_run(g_vm.angle);
}

void virtual_encoder_run(float angle)
{
    const float angle_per_pulse = (2.0f * PI) / (float)ENCODER_PPR;
    const float app_div4 = angle_per_pulse / 4.0f;
    const float app_div2 = angle_per_pulse / 2.0f;
    const float app_mul3_div4 = (3.0f * angle_per_pulse) / 4.0f;

    const float ppr_round = fmodf(angle, angle_per_pulse);
    if (ppr_round > 0.0f && ppr_round < app_div4)
    {
        g_vm.encoder_phase_a = 1;
        g_vm.encoder_phase_b = 0;
    }
    else if (ppr_round >= app_div4 && ppr_round < app_div2)
    {
        g_vm.encoder_phase_a = 1;
        g_vm.encoder_phase_b = 1;
    }
    else if (ppr_round >= app_div2 && ppr_round < app_mul3_div4)
    {
        g_vm.encoder_phase_a = 0;
        g_vm.encoder_phase_b = 1;
    }
    else if (ppr_round >= app_mul3_div4 && ppr_round < angle_per_pulse)
    {
        g_vm.encoder_phase_a = 0;
        g_vm.encoder_phase_b = 0;
    }

    HAL_GPIO_WritePin(VIRTUAL_ENCODER_PHASE_A_GPIO_PORT, VIRTUAL_ENCODER_PHASE_A_GPIO_PIN, g_vm.encoder_phase_a);
    HAL_GPIO_WritePin(VIRTUAL_ENCODER_PHASE_B_GPIO_PORT, VIRTUAL_ENCODER_PHASE_B_GPIO_PIN, g_vm.encoder_phase_b);
}

void virtual_current_sensor_run(float current)
{
    const float curr_min = CURR_SENS_MIN;
    const float curr_max = CURR_SENS_MAX;

    uint16_t dac_value = adc_normalization(current, curr_min, curr_max);

    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)dac_value);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)dac_value);
}

uint16_t adc_normalization(float value, float min_bound, float max_bound)
{
    if (value < min_bound)
    {
        value = min_bound;
    }
    if (value > max_bound)
    {
        value = max_bound;
    }

    // linear mapping
    uint16_t adcValue = (uint16_t)(((value - min_bound) / (max_bound - min_bound)) * 4095.0f);

    return adcValue;
}
