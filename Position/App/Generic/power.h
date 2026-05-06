#ifndef POWER_H
#define POWER_H

#include <stdbool.h>

typedef struct {
    float battery_voltage_V;
    bool  usb_connected;      // VBUS_sense PA9 HIGH = USB present
    bool  is_charging;        // CHRG  PB0  LOW  = TP4057 charging
    bool  charge_complete;    // STDBY PA6  LOW  = TP4057 standby (full)
} BatteryStatus_t;

void power_read(void);
const BatteryStatus_t* power_get_status(void);

#endif