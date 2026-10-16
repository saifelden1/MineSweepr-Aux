/** Single local-detection relay on PB12; no LEDs, buzzer or remote override. */
#ifndef STATUS_BEACON_DRIVER_H
#define STATUS_BEACON_DRIVER_H

#include <stdbool.h>
#include "auxiliary_pin_config.h"

void Status_Beacon_Init(void);
void Status_Beacon_SetMineAlert(bool detected);
bool Status_Beacon_IsActive(void);

#endif /* STATUS_BEACON_DRIVER_H */
