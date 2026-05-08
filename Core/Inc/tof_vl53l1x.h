#ifndef TOF_VL53L1X_H
#define TOF_VL53L1X_H

#include "VL53L1X_api.h"
#include "i2c.h"
#include "gpio.h"

/** VL53L1X sensor handle. */
typedef struct
{
  VL53L1_Dev_t dev;
  uint8_t sensor_ready;
  uint16_t timing_budget_ms;
  uint32_t inter_measurement_ms;
  GPIO_TypeDef *xshut_port;
  uint16_t xshut_pin;
  uint16_t distance;
} tof_t;

/**
 * @brief Initialize the VL53L1X sensor.
 * @param tof  Pointer to sensor handle.
 */
void tof_init(tof_t *tof);

void tof_reset(tof_t *tof);

/**
 * @brief Return the most recently measured distance in millimetres.
 * @param tof  Pointer to sensor handle.
 * @return Distance in mm.
 */
uint16_t tof_get_distance(tof_t *tof);

#endif // TOF_VL53L1X_H
