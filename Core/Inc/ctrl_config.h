#ifndef CTRL_CONFIG_H
#define CTRL_CONFIG_H

#include "main.h"
#include "motor_driver.h"
#include "encoder.h"
#include "motor_pid.h"
#include "battery.h"
#include "tof_vl53l1x.h"

#define N_MOTORS 4U
#define PER_REV 1940U
#define MAX_RPM 160U
#define KP 3.0f
#define KI 0.5f
#define KD 0.2f
#define ALPHA 0.5f
#define CMD_TIMEOUT_US 500000U

#define V_REF 3.3f
#define ADC_MAX 4095.0f
#define DIV_RATIO 11.13374f

#define TOF_ADR 0x52U
#define TOF_TIMING_BUDGET_MS 100U
#define TOF_INTER_MEASUREMENT_MS 100U

static motor_t motors[N_MOTORS] = {
  { .htim = &htim8, .channel = TIM_CHANNEL_1, .dir_port = M1_DIR_GPIO_Port, .dir_pin = M1_DIR_Pin, .max_pwm = TIM8_ARR },
  { .htim = &htim8, .channel = TIM_CHANNEL_2, .dir_port = M2_DIR_GPIO_Port, .dir_pin = M2_DIR_Pin, .max_pwm = TIM8_ARR },
  { .htim = &htim8, .channel = TIM_CHANNEL_3, .dir_port = M3_DIR_GPIO_Port, .dir_pin = M3_DIR_Pin, .max_pwm = TIM8_ARR },
  { .htim = &htim8, .channel = TIM_CHANNEL_4, .dir_port = M4_DIR_GPIO_Port, .dir_pin = M4_DIR_Pin, .max_pwm = TIM8_ARR }
};

static encoder_t encoders[N_MOTORS] = {
  { .htim = &htim1, .per_rev = PER_REV },
  { .htim = &htim2, .per_rev = PER_REV },
  { .htim = &htim3, .per_rev = PER_REV },
  { .htim = &htim4, .per_rev = PER_REV }
};

static motor_pid_t motor_pids[N_MOTORS] = {
  { .motor = &motors[0], .encoder = &encoders[0], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA },
  { .motor = &motors[1], .encoder = &encoders[1], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA },
  { .motor = &motors[2], .encoder = &encoders[2], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA },
  { .motor = &motors[3], .encoder = &encoders[3], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA }
};

static const int8_t motor_sign[N_MOTORS] = { 1, -1, 1, -1};

static battery_t bat = {
  .hadc = &hadc1,
  .htim = &htim6,
  .vref = V_REF,
  .adc_max = ADC_MAX,
  .div_ratio = DIV_RATIO
};

static tof_t tof = {
  .dev = { &hi2c1, TOF_ADR },
  .timing_budget_ms = TOF_TIMING_BUDGET_MS,
  .inter_measurement_ms = TOF_INTER_MEASUREMENT_MS,
  .xshut_port = TOF_XSHUT_GPIO_Port,
  .xshut_pin = TOF_XSHUT_Pin
};

#endif
