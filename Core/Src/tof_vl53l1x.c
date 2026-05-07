#include "tof_vl53l1x.h"
#include "stm32l4xx_hal_gpio.h"

void tof_init(tof_t *tof)
{
  uint8_t boot_state = 0;
  while (!boot_state)
    VL53L1X_BootState(tof->dev_adrr, &boot_state);

  VL53L1X_SensorInit(tof->dev_adrr);
  VL53L1X_SetTimingBudgetInMs(tof->dev_adrr, tof->timing_budget_ms);
  VL53L1X_SetInterMeasurementInMs(tof->dev_adrr, tof->inter_measurement_ms);
  VL53L1X_StartRanging(tof->dev_adrr);
}

void tof_reset(tof_t *tof)
{
  HAL_GPIO_WritePin(tof->xshut_port, tof->xshut_pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(tof->xshut_port, tof->xshut_pin, GPIO_PIN_SET);
  HAL_Delay(10);
}

uint16_t tof_get_distance(tof_t *tof)
{
  VL53L1X_CheckForDataReady(tof->dev_adrr, &tof->sensor_ready);
  if (tof->sensor_ready)
  {
    VL53L1X_GetDistance(tof->dev_adrr, &tof->distance);
    VL53L1X_ClearInterrupt(tof->dev_adrr);
  }
  return tof->distance;
}
