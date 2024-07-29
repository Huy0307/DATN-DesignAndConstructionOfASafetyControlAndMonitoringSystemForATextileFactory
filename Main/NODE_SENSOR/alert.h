#ifndef ALERT_H
#define ALERT_H
#include <Arduino.h>
void alert_temp(float &t, byte &danger_temp);
void alert_humi(float &h, byte &danger_humi);
void alert_pm(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10, byte &danger_pm);
void alert_light(uint16_t &lux, byte &danger_lux, bool &lightOn);
void alert_CO_CO2(uint16_t &CO, uint16_t &CO2, byte &danger_CO_CO2);
bool alert(float &t, float &h, uint16_t &pm1, uint16_t &pm25, uint16_t &pm10, uint16_t &lux, uint16_t &CO, uint16_t &CO2);
#endif