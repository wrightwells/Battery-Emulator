#ifndef ZERO_14_4_BATTERY_H
#define ZERO_14_4_BATTERY_H
#include <Arduino.h>
#include "../include.h"
#include "../lib/miwagner-ESP32-Arduino-CAN/ESP32CAN.h"

#define BATTERY_SELECTED
#define MAX_CELL_DEVIATION_MV 150

void setup_battery(void);

#endif
