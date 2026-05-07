#include "controller.h"
#include "ctrl_config.h"
#include <stdio.h>
#include <stdbool.h>
#include "uart_comm.h"
#include "micros.h"
#include "cmd_parser.h"

static RotationDirection_t dirs[N_MOTORS] =
    {ROTATION_CCW, ROTATION_CCW, ROTATION_CCW, ROTATION_CCW};

static float32_t set_speed[N_MOTORS];
static char rx_buff[100], out[128];
static uint32_t last_cmd_us = 0;
static bool debug_enabled = false;

void controller_init(bool debug)
{
  debug_enabled = debug;
  for (uint8_t i = 0; i < N_MOTORS; i++)
  {
    motor_init(&motors[i]);
    encoder_init(&encoders[i]);
    motor_pid_init(&motor_pids[i]);
  }
  battery_measure_init(&bat);
  tof_reset(&tof);
  tof_init(&tof);
}

void controller_adc_callback(ADC_HandleTypeDef *hadc)
{
  if (hadc == bat.hadc)
  {
    battery_measure_callback(&bat);
  }
}

static void controller_execute(const Command *cmd)
{
  CmdType type = cmd->cmd;
  switch (type)
  {
  case CMD_GET_POS:
  {
    float poses[N_MOTORS];
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      poses[i] = encoder_get_rotations(&encoders[i]) * (float)motor_sign[i];
    }
    sprintf(out, "%c %.3f %.3f %.3f %.3f\r",
            (char)type, poses[0], poses[1], poses[2], poses[3]);
    break;
  }
  case CMD_SET_SPEED:
  {
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      float dir = cmd->data.speeds[i] * motor_sign[i];
      dirs[i] = (dir >= 0) ? ROTATION_CCW : ROTATION_CW;
      set_speed[i] = fabsf(cmd->data.speeds[i]);
    }
    strcpy(out, "OK\r");
    break;
  }
  case CMD_SET_PID:
  {
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      motor_pid_set_pid(
          &motor_pids[i], cmd->set_pid.kp, cmd->set_pid.ki, cmd->set_pid.kd);
    }
    strcpy(out, "OK\r");
    break;
  }
  case CMD_FULL_FRAME_RX:
  {
    float poses[N_MOTORS];
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      // set motors speed
      float dir = cmd->data.speeds[i] * motor_sign[i];
      dirs[i] = (dir >= 0) ? ROTATION_CCW : ROTATION_CW;
      set_speed[i] = fabsf(cmd->data.speeds[i]);

      // read rotations
      poses[i] = encoder_get_rotations(&encoders[i]) * (float)motor_sign[i];
    }
    // read battery voltage 
    float vbat = battery_get_voltage(&bat);

    // read distance from tof
    float distance = (float)tof_get_distance(&tof) / 1000.0f;

    // sending 
    char tx_type = (char)CMD_FULL_FRAME_TX;
    sprintf(out, "%c %.3f %.3f %.3f %.3f %.3f %.3f\r",
            tx_type, poses[0], poses[1], poses[2], poses[3], vbat, distance);
    break;
  }
  case CMD_RESET:
  {
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      encoder_reset(&encoders[i]);
    }
    strcpy(out, "OK\r");
    break;
  }
  default:
    strcpy(out, "ERR\r");
    break;
  }
  if (debug_enabled)
    printf("[TX] %s\n", out);
  uart_send_str(out);
}

void controller_poll(void)
{
  Command cmd;
  if (uart_is_line())
  {
    uart_get_line(rx_buff, sizeof(rx_buff));
    if (debug_enabled)
      printf("[RX] %s\n", rx_buff);
    if (parse_command(rx_buff, &cmd))
    {
      last_cmd_us = micros();
      controller_execute(&cmd);
    }
    else
    {
      if (debug_enabled)
        printf("[TX] ERR\n");
      uart_send_str("ERR\r");
    }
  }
}

void controller_update()
{
  if (micros() - last_cmd_us > CMD_TIMEOUT_US)
  {
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      set_speed[i] = 0.0;
    }
  }
  for (uint8_t i = 0; i < N_MOTORS; i++)
  {
    motor_pid_update(&motor_pids[i], set_speed[i], dirs[i]);
  }
}